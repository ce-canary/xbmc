/*
 *  Copyright (C) 2005-2026 Team Kodi
 *  This file is part of Kodi - https://kodi.tv
 *
 *  SPDX-License-Identifier: GPL-2.0-or-later
 *  See LICENSES/README.md for more information.
 */

#include "DVDInputStreamBluray.h"

#include "DVDCodecs/Overlay/DVDOverlay.h"
#include "DVDCodecs/Overlay/DVDOverlayImage.h"
#include "DVDInputStreamFile.h"
#include "DVDDemuxers/DVDDemuxUtils.h"
#include "DVDDemuxers/DemuxMVC.h"
#include "IVideoPlayer.h"
#include "LangInfo.h"
#include "ServiceBroker.h"
#include "URL.h"
#include "cores/VideoPlayer/Interface/TimingConstants.h"
#include "filesystem/BlurayCallback.h"
#include "filesystem/SpecialProtocol.h"
#include "guilib/StereoscopicsManager.h"
#include "settings/DisplaySettings.h"
#include "settings/Settings.h"
#include "settings/SettingsComponent.h"
#include "utils/AMLUtils.h"
#include "utils/Geometry.h"
#include "utils/HDRCapabilities.h"
#include "utils/LanguageTag.h"
#include "utils/StringUtils.h"
#include "utils/URIUtils.h"
#include "utils/XTimeUtils.h"
#include "utils/log.h"
#include "video/VideoFileItemClassify.h"
#include "video/VideoInfoTag.h"
#include "windowing/GraphicContext.h"
#include "windowing/WinSystem.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iterator>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <libbluray/bluray-version.h>
#include <libbluray/bluray.h>
#include <libbluray/clpi_data.h>
#include <libbluray/mpls_data.h>
#include <libbluray/log_control.h>

extern "C"
{
#include <libavutil/mathematics.h>
}

#define LIBBLURAY_BYTESEEK 0
#define EMPTY_QUEUE(x) { while(!x.empty()) x.pop(); }

using namespace KODI;
using namespace std::chrono_literals;

namespace
{
/*! libbluray skips an aligned unit (6144 bytes) it cannot read and reports each one. This many in
    a row, about 6 MB, is a disc that has gone rather than a damaged patch to play past */
constexpr unsigned int MAX_CONSECUTIVE_READ_ERRORS{1000};

#if (BLURAY_VERSION >= BLURAY_VERSION_CODE(1, 1, 0))
// HDMI frame packing stacks both views with one frame's vertical blanking between them
constexpr int FRAME_PACKED_1080_HEIGHT = 2205;
constexpr int FRAME_PACKED_720_HEIGHT = 1470;

// frame packing as the Amlogic windowing flags it or as CDisplaySettings stores it
bool IsFramePacking(const RESOLUTION_INFO& res, int width, int height, int packedHeight)
{
  if (res.dwFlags & D3DPRESENTFLAG_MODE3DFP)
    return res.iScreenHeight == height;
  return (res.dwFlags & D3DPRESENTFLAG_MODE3DTB) && res.iScreenWidth == width &&
         res.iScreenHeight == packedHeight;
}

uint32_t GetDisplayCap()
{
  uint32_t displayCap = 0;
  const CDisplaySettings& displaySettings = CDisplaySettings::GetInstance();
  for (size_t i = RES_CUSTOM; i < displaySettings.ResolutionInfoSize(); ++i)
  {
    const RESOLUTION_INFO& res = displaySettings.GetResolutionInfo(i);
    const long refreshRate = std::lround(res.fRefreshRate);
    if (res.dwFlags & D3DPRESENTFLAG_INTERLACED)
    {
      if (res.dwFlags &
          (D3DPRESENTFLAG_MODE3DSBS | D3DPRESENTFLAG_MODE3DTB | D3DPRESENTFLAG_MODE3DFP))
        displayCap |= BLURAY_DCAP_INTERLACED_3D;
    }
    else if (refreshRate == 24 && IsFramePacking(res, 1920, 1080, FRAME_PACKED_1080_HEIGHT))
      displayCap |= BLURAY_DCAP_1080p_720p_3D;
    else if (refreshRate == 50 && IsFramePacking(res, 1280, 720, FRAME_PACKED_720_HEIGHT))
      displayCap |= BLURAY_DCAP_720p_50Hz_3D;
  }
  return displayCap;
}

// the base view plays on any display, 3D output only on a 3D display not set to play 2D
void SetupStereoSettings(BLURAY* bd)
{
  const int playbackMode = CServiceBroker::GetSettingsComponent()->GetSettings()->GetInt(
      CSettings::SETTING_VIDEOPLAYER_STEREOSCOPICPLAYBACKMODE);
  const uint32_t displayCap = GetDisplayCap();
  const bool prefer3D = displayCap != 0 && playbackMode != STEREOSCOPIC_PLAYBACK_MODE_MONO &&
                        playbackMode != STEREOSCOPIC_PLAYBACK_MODE_IGNORE;
  CLog::Log(LOGDEBUG,
            "CDVDInputStreamBluray::Open - 3D disc, display capability (PSR 23) 0x{:x}, prefer 3D {}",
            displayCap | BLURAY_DCAP_1080p_720p_3D, prefer3D);
  bd_set_player_setting(bd, BLURAY_PLAYER_SETTING_DISPLAY_CAP,
                        displayCap | BLURAY_DCAP_1080p_720p_3D);
  bd_set_player_setting(bd, BLURAY_PLAYER_SETTING_OUTPUT_PREFER,
                        prefer3D ? BLURAY_OUTPUT_PREFER_3D : BLURAY_OUTPUT_PREFER_2D);
}

// PSR 25 player and PSR 26 display UHD capability bits, libbluray leaves them unnamed
constexpr uint32_t UHD_CAP_DOLBY_VISION = 0x02;
constexpr uint32_t UHD_DCAP_2160P = 0x01;
constexpr uint32_t UHD_DCAP_HDR = 0x02;
constexpr uint32_t UHD_DCAP_DOLBY_VISION = 0x04;

bool HasUHDMode()
{
  const CDisplaySettings& displaySettings = CDisplaySettings::GetInstance();
  for (size_t i = RES_CUSTOM; i < displaySettings.ResolutionInfoSize(); ++i)
  {
    const RESOLUTION_INFO& res = displaySettings.GetResolutionInfo(i);
    if (res.iScreenWidth >= 3840 && res.iScreenHeight >= 2160 &&
        !(res.dwFlags & D3DPRESENTFLAG_INTERLACED))
      return true;
  }
  return false;
}

// libbluray's UHD setup claims every capability, so a disc picks its menus for a sink it may not
// have
void SetupUHDSettings(BLURAY* bd)
{
  const CHDRCapabilities caps = CServiceBroker::GetWinSystem()->GetDisplayHDRCapabilities();
  // PSR 25 b0 (player converts sRGB BD-J graphics to HDR) stays clear, Kodi shows them as drawn
  uint32_t playerCap = 0;
  uint32_t displayCap = 0;
  if (HasUHDMode())
    displayCap |= UHD_DCAP_2160P;
  if (aml_dolby_vision_enabled())
    playerCap |= UHD_CAP_DOLBY_VISION;
  if (caps.SupportsHDR10())
    displayCap |= UHD_DCAP_HDR;
  if (caps.SupportsDolbyVision() != DolbyVisionFormat::DOLBYVISION_TYPE_NONE)
    displayCap |= UHD_DCAP_DOLBY_VISION;
  CLog::Log(LOGDEBUG,
            "CDVDInputStreamBluray::Open - UHD capability (PSR 25) 0x{:x}, display (PSR 26) 0x{:x}",
            playerCap, displayCap);
  bd_set_player_setting(bd, BLURAY_PLAYER_SETTING_UHD_CAP, playerCap);
  bd_set_player_setting(bd, BLURAY_PLAYER_SETTING_UHD_DISPLAY_CAP, displayCap);
}
#endif

constexpr uint64_t SOURCE_PACKET_SIZE = 192;
} // namespace

static int read_blocks(void* handle, void* buf, int lba, int num_blocks)
{
  CDVDInputStreamBluray* blurayStream = reinterpret_cast<CDVDInputStreamBluray*>(handle);
  if (!blurayStream)
    return -1;
  return blurayStream->ReadBlocks(reinterpret_cast<uint8_t*>(buf), lba, num_blocks);
}

static BD_DIR_H* dir_open(void* handle, const char* rel_path)
{
  return static_cast<CDVDInputStreamBluray*>(handle)->OpenDir(rel_path);
}

static BD_FILE_H* file_open(void* handle, const char* rel_path)
{
  return static_cast<CDVDInputStreamBluray*>(handle)->OpenFile(rel_path);
}

#if (BLURAY_VERSION >= BLURAY_VERSION_CODE(1, 5, 0))
static bool is_hdr_range(uint8_t range)
{
  return range == BLURAY_DYNAMIC_RANGE_HDR10 || range == BLURAY_DYNAMIC_RANGE_DOLBY_VISION;
}
#endif

static void bluray_overlay_cb(void *this_gen, const BD_OVERLAY * ov)
{
  static_cast<CDVDInputStreamBluray*>(this_gen)->OverlayCallback(ov);
}

#ifdef HAVE_LIBBLURAY_BDJ
void  bluray_overlay_argb_cb(void *this_gen, const struct bd_argb_overlay_s * const ov)
{
  static_cast<CDVDInputStreamBluray*>(this_gen)->OverlayCallbackARGB(ov);
}
#endif

CDVDInputStreamBluray::CDVDInputStreamBluray(IVideoPlayer* player, const CFileItem& fileitem) :
  CDVDInputStream(DVDSTREAM_TYPE_BLURAY, fileitem), m_player(player)
{
  m_content = "video/x-mpegts";
  memset(&m_event, 0, sizeof(m_event));
#ifdef HAVE_LIBBLURAY_BDJ
  memset(&m_argb,  0, sizeof(m_argb));
#endif
}

CDVDInputStreamBluray::~CDVDInputStreamBluray()
{
  Close();
}

void CDVDInputStreamBluray::Abort()
{
  m_aborted = true;
}

bool CDVDInputStreamBluray::IsEOF()
{
  return false;
}

BLURAY_TITLE_INFO* CDVDInputStreamBluray::GetTitleFromState(const std::string& xmlstate)
{
  BlurayState blurayState;
  if (!m_blurayStateSerializer.XMLToBlurayState(blurayState, xmlstate))
  {
    CLog::LogF(LOGWARNING, "Failed to deserialize Bluray state");
    return nullptr;
  }
  return bd_get_playlist_info(m_bd, blurayState.playlistId, 0);
}

BLURAY_TITLE_INFO* CDVDInputStreamBluray::GetTitleLongest()
{
  BLURAY_TITLE_INFO *s = nullptr;
  for(int i=0; i < m_nTitles; i++)
  {
    BLURAY_TITLE_INFO *t = bd_get_title_info(m_bd, i, 0);
    if(!t)
    {
      CLog::Log(LOGDEBUG, "get_main_title - unable to get title {}", i);
      continue;
    }
    if(!s || s->duration < t->duration)
      std::swap(s, t);

    if(t)
      bd_free_title_info(t);
  }
  return s;
}

BLURAY_TITLE_INFO* CDVDInputStreamBluray::GetTitleFile(const std::string& filename)
{
  unsigned int playlist;
  if(sscanf(filename.c_str(), "%05u.mpls", &playlist) != 1)
  {
    CLog::Log(LOGERROR, "get_playlist_title - unsupported playlist file selected {}",
              CURL::GetRedacted(filename));
    return nullptr;
  }

  return bd_get_playlist_info(m_bd, playlist, 0);
}


bool CDVDInputStreamBluray::Open()
{
  if(m_player == nullptr)
    return false;

  std::string strPath(m_item.GetDynPath());
  std::string filename;
  std::string root;

  bool openStream = false;
  bool openDisc = false;
  bool resumable = true;

  // The item was selected via the simple menu
  if (URIUtils::IsProtocol(strPath, "bluray"))
  {
    CURL url(strPath);
    root = url.GetHostName();
    filename = URIUtils::GetFileName(url.GetFileName());

    // Check whether disc is AACS protected
    CURL url2(root);
    CFileItem item(url2, false);
    openDisc = VIDEO::IsProtectedBlurayDisc(item);

    // check for a menu call for an image file
    if (StringUtils::EqualsNoCase(filename, "menu") &&
        !(m_item.GetStartOffset() == STARTOFFSET_RESUME && m_item.IsResumable()))
    {
      resumable = false;

      // Remove udf:// if present
      if (url2.IsProtocol("udf"))
      {
        item.SetPath(url2.GetHostName());
        openDisc = VIDEO::IsProtectedBlurayDisc(item);
      }

      if (item.IsDiscImage())
      {
        if (!OpenStream(item))
          return false;

        openStream = true;
      }
    }
  }
  else if (m_item.IsDiscImage())
  {
    CURL url2("udf://");

    url2.SetHostName(m_item.GetPath());
    root = url2.Get();

    if (!OpenStream(m_item))
      return false;

    openStream = true;
  }
  else if (VIDEO::IsProtectedBlurayDisc(m_item))
  {
    openDisc = true;
  }
  else
  {
    strPath = URIUtils::GetDirectory(strPath);
    URIUtils::RemoveSlashAtEnd(strPath);

    if(URIUtils::GetFileName(strPath) == "PLAYLIST")
    {
      strPath = URIUtils::GetDirectory(strPath);
      URIUtils::RemoveSlashAtEnd(strPath);
    }

    if(URIUtils::GetFileName(strPath) == "BDMV")
    {
      strPath = URIUtils::GetDirectory(strPath);
      URIUtils::RemoveSlashAtEnd(strPath);
    }
    root = strPath;
    // Use the resolved (dynamic) path so playlist selectors survive plugin
    // resolution and library .strm playback. m_item.GetPath() returns the
    // original library reference (e.g. .strm file) which has no .mpls
    // extension, causing the MPLS title selector to be lost and playback
    // to fall through to navigation/main-feature mode.
    filename = URIUtils::GetFileName(m_item.GetDynPath());
  }

  // root should not have trailing slash
  URIUtils::RemoveSlashAtEnd(root);

  bd_set_debug_handler(CBlurayCallback::bluray_logger);
  bd_set_debug_mask(DBG_CRIT | DBG_BLURAY | DBG_NAV);

  m_bd = bd_init();

  if (!m_bd)
  {
    CLog::Log(LOGERROR, "CDVDInputStreamBluray::Open - failed to initialize libbluray");
    return false;
  }

  SetupPlayerSettings();

  CLog::Log(LOGDEBUG, "CDVDInputStreamBluray::Open - opening {}", CURL::GetRedacted(root));

  if (openStream)
  {
    if (!bd_open_stream(m_bd, this, read_blocks))
    {
      CLog::Log(LOGERROR, "CDVDInputStreamBluray::Open - failed to open {} in stream mode",
                CURL::GetRedacted(root));
      return false;
    }
  }
  else if (openDisc)
  {
    // This special case is required for opening original AACS protected Blu-ray discs. Otherwise
    // things like Bus Encryption might not be handled properly and playback will fail.
    m_rootPath = root;
    if (!bd_open_disc(m_bd, root.c_str(), nullptr))
    {
      CLog::Log(LOGERROR, "CDVDInputStreamBluray::Open - failed to open {} in disc mode",
                CURL::GetRedacted(root));
      return false;
    }
  }
  else
  {
    m_rootPath = root;

#if defined(HAS_UDFREAD)
    // Only in files mode does libbluray reach the disc through Kodi's filesystem, opening a dozen
    // or so files and directories on it, and then the clips as they play. On a disc image each of
    // those opens would otherwise re-mount the image's UDF volume, so keep it mounted for as long
    // as the disc is open. (Stream mode reads the image itself, disc mode is a physical disc.)
    m_udfMount.emplace(root);
#endif

    if (!bd_open_files(m_bd, this, dir_open, file_open))
    {
      CLog::Log(LOGERROR, "CDVDInputStreamBluray::Open - failed to open {} in files mode",
                CURL::GetRedacted(root));
      return false;
    }
  }

  bd_get_event(m_bd, nullptr);

  m_root = root;
  const BLURAY_DISC_INFO *disc_info = bd_get_disc_info(m_bd);

  if (!disc_info)
  {
    CLog::Log(LOGERROR, "CDVDInputStreamBluray::Open - bd_get_disc_info() failed");
    return false;
  }

  if (disc_info->bluray_detected)
  {
#if (BLURAY_VERSION > BLURAY_VERSION_CODE(1,0,0))
    CLog::Log(LOGDEBUG, "CDVDInputStreamBluray::Open - Disc name           : {}",
              disc_info->disc_name ? disc_info->disc_name : "");
#endif
    CLog::Log(LOGDEBUG, "CDVDInputStreamBluray::Open - First Play supported: {}",
              disc_info->first_play_supported);
    CLog::Log(LOGDEBUG, "CDVDInputStreamBluray::Open - Top menu supported  : {}",
              disc_info->top_menu_supported);
    CLog::Log(LOGDEBUG, "CDVDInputStreamBluray::Open - HDMV titles         : {}",
              disc_info->num_hdmv_titles);
    CLog::Log(LOGDEBUG, "CDVDInputStreamBluray::Open - BD-J titles         : {}",
              disc_info->num_bdj_titles);
    CLog::Log(LOGDEBUG, "CDVDInputStreamBluray::Open - BD-J handled        : {}",
              disc_info->bdj_handled);
    CLog::Log(LOGDEBUG, "CDVDInputStreamBluray::Open - UNSUPPORTED titles  : {}",
              disc_info->num_unsupported_titles);
    CLog::Log(LOGDEBUG, "CDVDInputStreamBluray::Open - AACS detected       : {}",
              disc_info->aacs_detected);
    CLog::Log(LOGDEBUG, "CDVDInputStreamBluray::Open - libaacs detected    : {}",
              disc_info->libaacs_detected);
    CLog::Log(LOGDEBUG, "CDVDInputStreamBluray::Open - AACS handled        : {}",
              disc_info->aacs_handled);
    CLog::Log(LOGDEBUG, "CDVDInputStreamBluray::Open - BD+ detected        : {}",
              disc_info->bdplus_detected);
    CLog::Log(LOGDEBUG, "CDVDInputStreamBluray::Open - libbdplus detected  : {}",
              disc_info->libbdplus_detected);
    CLog::Log(LOGDEBUG, "CDVDInputStreamBluray::Open - BD+ handled         : {}",
              disc_info->bdplus_handled);
#if (BLURAY_VERSION >= BLURAY_VERSION_CODE(1,0,0))
    CLog::Log(LOGDEBUG, "CDVDInputStreamBluray::Open - no menus (libmmbd, or profile 6 bdj)  : {}",
              disc_info->no_menu_support);
#endif
    CLog::Log(LOGDEBUG, "CDVDInputStreamBluray::Open - 3D content exist    : {}", disc_info->content_exist_3D);
  }
  else
    CLog::Log(LOGERROR, "CDVDInputStreamBluray::Open - BluRay not detected");

  if (disc_info->aacs_detected && !disc_info->aacs_handled)
  {
    CLog::Log(LOGERROR, "CDVDInputStreamBluray::Open - Media stream scrambled/encrypted with AACS");
    m_player->OnDiscNavResult(nullptr, BD_EVENT_ENC_ERROR);
    return false;
  }

  if (disc_info->bdplus_detected && !disc_info->bdplus_handled)
  {
    CLog::Log(LOGERROR, "CDVDInputStreamBluray::Open - Media stream scrambled/encrypted with BD+");
    m_player->OnDiscNavResult(nullptr, BD_EVENT_ENC_ERROR);
    return false;
  }

#if (BLURAY_VERSION >= BLURAY_VERSION_CODE(1, 1, 0))
  // libbluray's 3D setup assumes a 3D display with 3D output preferred
  if (disc_info->content_exist_3D)
    SetupStereoSettings(m_bd);
  SetupUHDSettings(m_bd);
#endif

  m_nTitles = bd_get_titles(m_bd, TITLES_RELEVANT, 0);

  if (URIUtils::HasExtension(filename, ".mpls"))
  {
    m_navmode = false;
    m_titleInfo = GetTitleFile(filename);
  }
  else if (resumable && m_item.GetStartOffset() == STARTOFFSET_RESUME && m_item.IsResumable())
  {
    m_navmode = false;
    m_titleInfo = GetTitleFromState(m_item.GetVideoInfoTag()->GetResumePoint().playerState);
  }
  else
  {
    m_navmode = true;
    if (!disc_info->first_play_supported)
    {
      CLog::Log(LOGERROR, "CDVDInputStreamBluray::Open - Can't play disc in HDMV navigation mode - First Play title not supported");
      m_navmode = false;
    }

    if (m_navmode && disc_info->num_unsupported_titles > 0) {
      CLog::Log(LOGERROR, "CDVDInputStreamBluray::Open - Unsupported titles found - Some titles can't be played in navigation mode");
    }

    if(!m_navmode)
      m_titleInfo = GetTitleLongest();
  }

  if (m_navmode)
  {

    bd_register_overlay_proc (m_bd, this, bluray_overlay_cb);
#ifdef HAVE_LIBBLURAY_BDJ
    bd_register_argb_overlay_proc (m_bd, this, bluray_overlay_argb_cb, nullptr);
#endif
    bd_set_presentation_hold(m_bd, 1);

    if(bd_play(m_bd) <= 0)
    {
      CLog::Log(LOGERROR, "CDVDInputStreamBluray::Open - failed play disk {}",
                CURL::GetRedacted(strPath));
      return false;
    }
    m_hold = HOLD_DATA;
  }
  else
  {
    if(!m_titleInfo)
    {
      CLog::Log(LOGERROR, "CDVDInputStreamBluray::Open - failed to get title info");
      return false;
    }

    if(!bd_select_playlist(m_bd, m_titleInfo->playlist))
    {
      CLog::Log(LOGERROR, "CDVDInputStreamBluray::Open - failed to select playlist {}",
                m_titleInfo->idx);
      return false;
    }
  }

  // For playlist/chapter watch time
  m_startWatchTime = std::chrono::steady_clock::now();

  // Process any events that occurred during opening
  while (bd_get_event(m_bd, &m_event))
    ProcessEvent();
  m_playlistLeft = false;
  m_itemSeeked = false;

  OpenNextStream();

  return true;
}

// close file and reset everything
void CDVDInputStreamBluray::Close()
{
  CloseMVCDemux();
  FreeTitleInfo();

  if(m_bd)
  {
    bd_register_overlay_proc(m_bd, nullptr, nullptr);
    bd_close(m_bd);
  }

  m_bd = nullptr;
  m_pstream.reset();
  m_rootPath.clear();
  m_readPos = 0;
  m_clipSpans.clear();
  m_hasMVCExtension = false;
  m_clipStart.clear();

#if defined(HAS_UDFREAD)
  // Released last, as the files opened from the volume are closed above
  m_udfMount.reset();
#endif
}

void CDVDInputStreamBluray::FreeTitleInfo()
{
  if (m_titleInfo)
    bd_free_title_info(m_titleInfo);

  m_titleInfo = nullptr;
  m_clip = nullptr;
  m_nextClip = nullptr;
  m_heldClip = nullptr;
  m_endedTitleTime = 0;
  FreeClipInfo();
}

// the player still plays the title out of its queues until the next stream
void CDVDInputStreamBluray::ReleaseTitle()
{
  const int totalTime = GetTotalTime();
  FreeTitleInfo();
  m_endedTitleTime = totalTime;
}

void CDVDInputStreamBluray::FreeClipInfo()
{
  if (m_clipInfo)
    bd_free_clpi(m_clipInfo);

  m_clipInfo = nullptr;
}

void CDVDInputStreamBluray::UpdateClipInfo(unsigned int playItem)
{
  FreeClipInfo();

  // A copy of the .clpi libbluray read when it opened the title, so this costs no disc access.
  // The play items of the playlist being played and the clips of the open title are both in the
  // order the .mpls lists them, so the index of one is the index of the other.
  m_clipInfo = bd_get_clpi(m_bd, playItem);
  if (!m_clipInfo)
    CLog::Log(LOGDEBUG,
              "CDVDInputStreamBluray::UpdateClipInfo - no clip information for play item {} - the "
              "streams the playlist does not present will have no language",
              playItem);
}

void CDVDInputStreamBluray::ProcessEvent() {

  int pid = -1, ret;
  const bool titleEndWait = m_titleEndWait;
  switch (m_event.event) {

   /* errors */

  case BD_EVENT_ERROR:
    switch (m_event.param)
    {
    case BD_ERROR_HDMV:
    case BD_ERROR_BDJ:
      m_player->OnDiscNavResult(nullptr, BD_EVENT_MENU_ERROR);
      break;
    default:
      break;
    }
    CLog::Log(LOGERROR, "CDVDInputStreamBluray - BD_EVENT_ERROR: Fatal error. Playback can't be continued.");
    m_hold = HOLD_ERROR;
    m_titleEndWait = false;
    break;

  case BD_EVENT_READ_ERROR:
    ++m_readErrors;
    CLog::Log(LOGERROR, "CDVDInputStreamBluray - BD_EVENT_READ_ERROR ({} in a row)", m_readErrors);
    if (m_readErrors == MAX_CONSECUTIVE_READ_ERRORS)
      CLog::Log(LOGERROR,
                "CDVDInputStreamBluray - Giving up after {} read errors in a row, the disc may "
                "have been removed. Playback can't be continued.",
                m_readErrors);
    break;

  case BD_EVENT_ENCRYPTED:
    CLog::Log(LOGERROR, "CDVDInputStreamBluray - BD_EVENT_ENCRYPTED");
    switch (m_event.param)
    {
    case BD_ERROR_AACS:
      CLog::Log(LOGERROR, "CDVDInputStreamBluray - BD_ERROR_AACS");
      break;
    case BD_ERROR_BDPLUS:
      CLog::Log(LOGERROR, "CDVDInputStreamBluray - BD_ERROR_BDPLUS");
      break;
    default:
      break;
    }
    m_hold = HOLD_ERROR;
    m_player->OnDiscNavResult(nullptr, BD_EVENT_ENC_ERROR);
    break;

  /* playback control */

  case BD_EVENT_SEEK:
    CLog::Log(LOGDEBUG, "CDVDInputStreamBluray - BD_EVENT_SEEK");
    //m_player->OnDVDNavResult(nullptr, 1);
    //bd_read_skip_still(m_bd);
    //m_hold = HOLD_HELD;
    m_titleEndWait = false;
    m_itemSeeked = true;
    break;

  case BD_EVENT_STILL_TIME:
    CLog::Log(LOGDEBUG, "CDVDInputStreamBluray - BD_EVENT_STILL_TIME {}", m_event.param);
    pid = m_event.param;
    m_player->OnDiscNavResult(static_cast<void*>(&pid), BD_EVENT_STILL_TIME);
    m_hold = HOLD_STILL;
    break;

  case BD_EVENT_STILL:
    CLog::Log(LOGDEBUG, "CDVDInputStreamBluray - BD_EVENT_STILL {}", m_event.param);

    pid = m_event.param;

    if (pid == 0)
      m_player->OnDiscNavResult(static_cast<void*>(&pid), BD_EVENT_STILL);
    break;

  case BD_EVENT_DISCONTINUITY:
    CLog::Log(LOGDEBUG, "CDVDInputStreamBluray - BD_EVENT_DISCONTINUITY");
    m_player->OnDiscNavResult(&m_event.param, BD_EVENT_DISCONTINUITY);
    m_hold = HOLD_NONE;
    break;

    /* playback position */

  case BD_EVENT_ANGLE:
    CLog::Log(LOGDEBUG, "CDVDInputStreamBluray - BD_EVENT_ANGLE {}", m_event.param);
    m_angle = m_event.param;

    if (m_playlist <= MAX_PLAYLIST_ID)
    {
      FreeTitleInfo();
      m_titleInfo = bd_get_playlist_info(m_bd, m_playlist, m_angle);
    }
    break;

  case BD_EVENT_END_OF_TITLE:
    CLog::Log(LOGDEBUG, "CDVDInputStreamBluray - BD_EVENT_END_OF_TITLE {}", m_event.param);
    /* when a title ends, playlist WILL eventually change */
    m_playlistLeft = true;
    if (m_navmode)
      ReleaseTitle();
    break;

  case BD_EVENT_TITLE:
  {
    CLog::Log(LOGDEBUG, "CDVDInputStreamBluray - BD_EVENT_TITLE {}", m_event.param);
    const BLURAY_DISC_INFO* disc_info = bd_get_disc_info(m_bd);

    m_menu = false;
    m_isInMainMenu = false;
    m_playlistLeft = true;
    m_pendingMenuRange.reset();

    if (m_event.param == BLURAY_TITLE_TOP_MENU)
    {
      m_title = disc_info->top_menu;
      m_menu = true;
      m_isInMainMenu = true;
    }
    else if (m_event.param == BLURAY_TITLE_FIRST_PLAY)
      m_title = disc_info->first_play;
    else if (m_event.param <= disc_info->num_titles)
      m_title = disc_info->titles[m_event.param];
    else
      m_title = nullptr;

    break;
  }
  case BD_EVENT_PLAYLIST:
    CLog::Log(LOGDEBUG, "CDVDInputStreamBluray - BD_EVENT_PLAYLIST {}", m_event.param);
    m_playlist = m_event.param;
    ProcessItem(m_playlist);
    m_titleEndWait = false;
    m_playlistLeft = true;
    m_pendingMenuRange.reset();
    break;

  case BD_EVENT_PLAYITEM:
    CLog::Log(LOGDEBUG, "CDVDInputStreamBluray - BD_EVENT_PLAYITEM {}", m_event.param);
    m_titleEndWait = false;
    if (m_titleInfo && m_event.param < m_titleInfo->clip_count)
    {
      m_clip = &m_titleInfo->clips[m_event.param];
#if (BLURAY_VERSION >= BLURAY_VERSION_CODE(1, 5, 0))
      if (m_clip->video_stream_count > 0)
      {
        const bool hdr = is_hdr_range(m_clip->video_streams[0].dynamic_range_type);
        // with notifications held the disc program draws the pages of the item once it is shown
        if (m_notificationsHeld && m_title && m_title->bdj)
        {
          m_pendingMenuRange = hdr;
          m_pendingMenuRangeStart = static_cast<int64_t>(m_clip->start_time);
        }
        else
        {
          m_pendingMenuRange.reset();
          UpdateMenuRange(hdr);
        }
      }
#endif
      UpdateClipInfo(m_event.param);
    }
    else
      m_clip = nullptr;
    OpenClipSpan();
    uint64_t clip_start, clip_in, bytepos;
    ret = bd_get_clip_infos(m_bd, m_event.param, &clip_start, &clip_in, &bytepos, nullptr);
    if (ret)
      m_clipStartTime = clip_start / 90;
    break;

  case BD_EVENT_CHAPTER:
    CLog::Log(LOGDEBUG, "CDVDInputStreamBluray - BD_EVENT_CHAPTER {}", m_event.param);
    break;

    /* stream selection */

  case BD_EVENT_AUDIO_STREAM:
    pid = -1;
    if (m_titleInfo && m_clip && static_cast<uint32_t>(m_clip->audio_stream_count) > (m_event.param - 1))
      pid = m_clip->audio_streams[m_event.param - 1].pid;
    CLog::Log(LOGDEBUG, "CDVDInputStreamBluray - BD_EVENT_AUDIO_STREAM {} {}", m_event.param, pid);
    m_player->OnDiscNavResult(static_cast<void*>(&pid), BD_EVENT_AUDIO_STREAM);
    break;

  case BD_EVENT_PG_TEXTST:
    CLog::Log(LOGDEBUG, "CDVDInputStreamBluray - BD_EVENT_PG_TEXTST {}", m_event.param);
    pid = m_event.param;
    m_pgEnabled = m_event.param != 0;
    m_player->OnDiscNavResult(static_cast<void*>(&pid), BD_EVENT_PG_TEXTST);
    break;

  case BD_EVENT_PG_TEXTST_STREAM:
    pid = -1;
    if (m_titleInfo && m_clip && static_cast<uint32_t>(m_clip->pg_stream_count) > (m_event.param - 1))
      pid = m_clip->pg_streams[m_event.param - 1].pid;
    CLog::Log(LOGDEBUG, "CDVDInputStreamBluray - BD_EVENT_PG_TEXTST_STREAM {}, {}", m_event.param,
              pid);
    m_pgStreamNumber = m_event.param;
    m_player->OnDiscNavResult(static_cast<void*>(&pid), BD_EVENT_PG_TEXTST_STREAM);
    break;

  case BD_EVENT_MENU:
    CLog::Log(LOGDEBUG, "CDVDInputStreamBluray - BD_EVENT_MENU {}", m_event.param);
    m_menu = (m_event.param != 0);
    if (!m_menu)
      m_isInMainMenu = false;
    m_player->OnDiscNavResult(&m_event.param, BD_EVENT_MENU);
    break;

  case BD_EVENT_IDLE:
    KODI::TIME::Sleep(100ms);
    break;

  case BD_EVENT_SOUND_EFFECT:
  {
    BLURAY_SOUND_EFFECT effect;
    if (bd_get_sound_effect(m_bd, m_event.param, &effect) <= 0)
    {
      CLog::Log(LOGDEBUG, "CDVDInputStreamBluray - BD_EVENT_SOUND_EFFECT {} not valid",
                m_event.param);
    }
    else
    {
      CLog::Log(LOGDEBUG, "CDVDInputStreamBluray - BD_EVENT_SOUND_EFFECT {}", m_event.param);
    }
  }

  case BD_EVENT_IG_STREAM:
  case BD_EVENT_SECONDARY_AUDIO:
  case BD_EVENT_SECONDARY_AUDIO_STREAM:
  case BD_EVENT_SECONDARY_VIDEO:
  case BD_EVENT_SECONDARY_VIDEO_SIZE:
  case BD_EVENT_SECONDARY_VIDEO_STREAM:
  case BD_EVENT_KEY_INTEREST_TABLE:
    break;

  case BD_EVENT_UO_MASK_CHANGED:
    CLog::Log(LOGDEBUG, "CDVDInputStreamBluray - BD_EVENT_UO_MASK_CHANGED {:#x}", m_event.param);
    m_uoMask = m_event.param;
    break;

  case BD_EVENT_PLAYMARK:
    // a seek the disc program makes at a mark is not the answer to a key
    m_menuInput = false;
    break;

  case BD_EVENT_PLAYLIST_STOP:
    CLog::Log(LOGDEBUG, "CDVDInputStreamBluray - BD_EVENT_PLAYLIST_STOP: flush buffers");
    m_player->OnDiscNavResult(nullptr, BD_EVENT_PLAYLIST_STOP);
    m_titleEndWait = false;
    break;
  case BD_EVENT_NONE:
    break;

  default:
    CLog::Log(LOGWARNING, "CDVDInputStreamBluray - unhandled libbluray event {} [param {}]",
              m_event.event, m_event.param);
    break;
  }

  // an event that ends the wait holds the old demuxer until the next stream opens
  if (titleEndWait && !m_titleEndWait && m_hold != HOLD_ERROR)
    m_hold = HOLD_HELD;

  /* event has been consumed */
  m_event.event = BD_EVENT_NONE;

  if ( m_bMVCPlayback && m_clip
    && m_titleInfo
    && m_clip < m_titleInfo->clips + m_titleInfo->clip_count
    && m_nMVCClip != m_clip
    && (m_clipQueue.empty()
      || m_clip != m_titleInfo->clips + m_clipQueue.front()))
  {
    m_clipQueue.push(m_clip - m_titleInfo->clips);
    if (m_pMVCDemux == NULL)
      OpenNextStream();
  }

  SelectPendingStreams();
}

void CDVDInputStreamBluray::DisableExtention()
{
  CloseMVCDemux();
  m_bMVCDisabled = true;
  m_bMVCPlayback = false;
}

// counted after the events of the read have opened their clip spans
int CDVDInputStreamBluray::Read(uint8_t* buf, int buf_size)
{
  const int result = ReadData(buf, buf_size);
  if (result > 0)
    m_readPos += result;
  return result;
}

int CDVDInputStreamBluray::ReadData(uint8_t* buf, int buf_size)
{
  int result = 0;
  if (m_navmode && !m_clipStart.empty() && !m_clipStartHeld && m_clipStartOffset == 0)
    DropClipStartOnJump();
  if (!m_clipStart.empty())
  {
    if (m_clipStartHeld)
      return 0;

    result = std::min(buf_size, static_cast<int>(m_clipStart.size() - m_clipStartOffset));
    std::copy_n(m_clipStart.data() + m_clipStartOffset, result, buf);
    m_clipStartOffset += result;
    if (m_clipStartOffset == m_clipStart.size())
      m_clipStart.clear();
    return result;
  }
  // the packets a held demuxer plays out keep the time of the item they came from
  const int dispTimeHeld = m_dispTimeBeforeRead;
  if (!m_navmode || m_hold != HOLD_HELD)
    m_dispTimeBeforeRead = static_cast<int>((bd_tell_time(m_bd) / 90));
  if(m_navmode)
  {
    // the running demuxer plays on into the next play item of the title, a discontinuity holds it
    const auto joinsInLine = [this, &result]
    {
      return result > 0 && m_titleInfo && m_clip && m_event.param < m_titleInfo->clip_count &&
             m_event.param == static_cast<uint32_t>(m_clip - m_titleInfo->clips) + 1;
    };
    do {

      if (m_hold == HOLD_HELD || m_titleEndWait)
         return 0;

      if (m_hold == HOLD_ERROR || m_aborted || HasGivenUpReading())
        return -1;

      result = bd_read_ext (m_bd, buf, buf_size, &m_event);
      m_readSinceReport = true;

      if(result < 0)
      {
        m_hold = HOLD_ERROR;
        return result;
      }
      CheckPresentedLeft();

      const bool menuJump = TakeMenuJump(joinsInLine());

      // Before the holding events, as a read crossing a clip boundary returns data with one
      if (result > 0)
        m_readErrors = 0;

      /* Check for holding events */
      switch(m_event.event) {
        case BD_EVENT_PLAYITEM:
          if (joinsInLine())
            break;
          [[fallthrough]];
        case BD_EVENT_SEEK:
        case BD_EVENT_TITLE:
        case BD_EVENT_ANGLE:
        case BD_EVENT_PLAYLIST:
        case BD_EVENT_DISCONTINUITY:
          if(m_hold != HOLD_DATA)
          {
            const bool fromStill = m_hold == HOLD_STILL;
            m_hold = HOLD_HELD;
            // a hold without data keeps the time the demuxer being closed already has
            if (result == 0)
              m_dispTimeBeforeRead = dispTimeHeld;
            if (result == 0 && AbandonsItem(menuJump, fromStill))
              m_itemAbandoned = true;
            // the read starts the new play item, which the demuxer being closed must not consume
            if ((m_event.event == BD_EVENT_PLAYITEM || m_event.event == BD_EVENT_DISCONTINUITY) &&
                result > 0)
            {
              HoldClipStart(buf, result);
              return 0;
            }
            return result;
          }
          break;

        case BD_EVENT_STILL_TIME:
          // an infinite still starts here, so later disc events reach the player through the read
          if(m_hold == HOLD_STILL)
            m_event.event = 0; /* Consume duplicate still event */
          else if (m_event.param == 0)
            ProcessEvent();
          else
            m_hold = HOLD_HELD;
          return result;

        case BD_EVENT_END_OF_TITLE:
          // a data read here ends the playlist for the disc program while the player still plays it
          if (m_hold != HOLD_DATA && m_title && m_title->bdj)
            m_titleEndWait = true;
          break;

        default:
          break;
      }

      const bool queued = m_event.event != BD_EVENT_NONE && m_hold != HOLD_DATA;
      bool playItemInLine = m_event.event == BD_EVENT_PLAYITEM && queued;
      if(result > 0)
        m_hold = HOLD_NONE;

      ProcessEvent();

      // a clip open can queue its play item behind other events of the same read
      while(result > 0 && queued && m_hold == HOLD_NONE && bd_get_event(m_bd, &m_event))
      {
        CheckPresentedLeft();
        switch(m_event.event) {
          case BD_EVENT_PLAYITEM:
            if (joinsInLine())
            {
              playItemInLine = true;
              break;
            }
            m_hold = HOLD_HELD;
            HoldClipStart(buf, result);
            return 0;
          case BD_EVENT_SEEK:
          case BD_EVENT_TITLE:
          case BD_EVENT_ANGLE:
          case BD_EVENT_PLAYLIST:
          case BD_EVENT_STILL_TIME:
          case BD_EVENT_DISCONTINUITY:
            m_hold = HOLD_HELD;
            if (playItemInLine || m_event.event == BD_EVENT_DISCONTINUITY)
            {
              HoldClipStart(buf, result);
              return 0;
            }
            return result;

          default:
            break;
        }
        ProcessEvent();
      }

    } while(result == 0);

  }
  else
  {
    unsigned int readErrors{0};
    bool discontinuity{false};
    do
    {
      if (m_aborted || HasGivenUpReading())
        return -1;
      readErrors = m_readErrors;
      discontinuity = false;
      result = bd_read(m_bd, buf, buf_size);
      if (result > 0)
        m_readErrors = 0;
      while (bd_get_event(m_bd, &m_event))
      {
        if (m_event.event == BD_EVENT_DISCONTINUITY)
          discontinuity = true;
        ProcessEvent();
      }
      // A unit libbluray skips as broken is reported and read as nothing, which the demuxer would
      // take for the end of the title
    } while (result == 0 && m_readErrors > readErrors);
    // the read starts the new clip, which the demuxer being reset must not consume
    if (discontinuity && result > 0)
    {
      HoldClipStart(buf, result);
      return 0;
    }
  }
  return result;
}

bool CDVDInputStreamBluray::HasGivenUpReading() const
{
  return m_readErrors >= MAX_CONSECUTIVE_READ_ERRORS;
}

void CDVDInputStreamBluray::HoldClipStart(const uint8_t* buf, int size)
{
  m_clipStart.assign(buf, buf + size);
  m_clipStartOffset = 0;
  m_clipStartHeld = true;
  m_clipStartEnd = bd_tell(m_bd);
  m_heldClip = GetClipAt(m_clipStartEnd - size);
  CLog::Log(LOGDEBUG, "CDVDInputStreamBluray - holding {} bytes of the new clip", size);
}

// a menu action or a disc program can move away from the held play item during the hold; an HDMV
// jump is reported only when the VM runs again, a BD-J reposition shows only in the read position
void CDVDInputStreamBluray::DropClipStartOnJump()
{
  while (true)
  {
    if (bd_read_ext(m_bd, nullptr, 0, &m_event) < 0)
    {
      m_hold = HOLD_ERROR;
      break;
    }
    if (m_event.event == BD_EVENT_NONE)
      break;
    CheckPresentedLeft();

    switch (m_event.event)
    {
      case BD_EVENT_SEEK:
      case BD_EVENT_TITLE:
      case BD_EVENT_ANGLE:
      case BD_EVENT_PLAYLIST:
      case BD_EVENT_PLAYLIST_STOP:
        m_clipStart.clear();
        break;
      default:
        break;
    }
    ProcessEvent();
    if (m_hold == HOLD_ERROR || m_aborted)
      break;
  }

  if (m_hold == HOLD_ERROR || m_aborted || bd_tell(m_bd) != m_clipStartEnd)
    m_clipStart.clear();
  else if (!m_clipStart.empty() && m_hold == HOLD_DATA)
    m_hold = HOLD_NONE;
}

int CDVDInputStreamBluray::ReadBlocks(uint8_t* buf, int lba, int num_blocks)
{
  CDVDInputStreamFile* lpstream = m_pstream.get();
  if (!lpstream)
    return -1;
  int result = -1;
  int64_t offset = static_cast<int64_t>(lba) * 2048;
  std::unique_lock lock(m_readBlocksLock);
  if (lpstream->Seek(offset, SEEK_SET) >= 0)
  {
    int64_t size = static_cast<int64_t>(num_blocks) * 2048;
    if (size <= std::numeric_limits<int>::max())
      result = lpstream->Read(buf, static_cast<int>(size)) / 2048;
  }
  return result;
}

BD_DIR_H* CDVDInputStreamBluray::OpenDir(const char* relPath)
{
  return CBlurayCallback::dir_open(&m_rootPath, relPath);
}

// libbluray reads navigation-mode clips on the player thread, the file cache reads on ahead of it
BD_FILE_H* CDVDInputStreamBluray::OpenFile(const char* relPath)
{
  const unsigned int flags =
      m_navmode && URIUtils::HasExtension(relPath, ".m2ts|.ssif") ? XFILE::READ_AUDIO_VIDEO : 0;
  return CBlurayCallback::file_open(&m_rootPath, relPath, flags);
}

static uint8_t  clamp(double v)
{
  return (v) > 255.0 ? 255 : ((v) < 0.0 ? 0 : static_cast<uint32_t>((v + 0.5)));
}

static uint32_t build_rgba(const BD_PG_PALETTE_ENTRY &e, bool bt2020)
{
  const double rv = bt2020 ? 1.679 : 1.596;
  const double gu = bt2020 ? 0.187 : 0.391;
  const double gv = bt2020 ? 0.650 : 0.813;
  const double bu = bt2020 ? 2.142 : 2.018;
  double r = 1.164 * (e.Y - 16)                   + rv * (e.Cr - 128);
  double g = 1.164 * (e.Y - 16) - gu * (e.Cb - 128) - gv * (e.Cr - 128);
  double b = 1.164 * (e.Y - 16) + bu * (e.Cb - 128);
  return static_cast<uint32_t>(e.T)      << PIXEL_ASHIFT
       | static_cast<uint32_t>(clamp(r)) << PIXEL_RSHIFT
       | static_cast<uint32_t>(clamp(g)) << PIXEL_GSHIFT
       | static_cast<uint32_t>(clamp(b)) << PIXEL_BSHIFT;
}

void CDVDInputStreamBluray::OverlayClose()
{
#if(BD_OVERLAY_INTERFACE_VERSION >= 2)
  for(SPlane& plane : m_planes)
  {
    plane.o.clear();
    plane.drawing = false;
  }
  auto group = std::make_shared<CDVDOverlayGroup>();
  group->bForced = true;
  group->SetDiscMenu(true);
  group->SetOverlayContainerFlushable(false);
  m_player->OnDiscNavResult(static_cast<void*>(&group), BD_EVENT_MENU_OVERLAY);
  m_hasOverlay = false;
#endif
}

void CDVDInputStreamBluray::OverlayInit(SPlane& plane, int w, int h)
{
#if(BD_OVERLAY_INTERFACE_VERSION >= 2)
  plane.o.clear();
  plane.w = w;
  plane.h = h;
#endif
}

void CDVDInputStreamBluray::OverlayClear(SPlane& plane, int x, int y, int w, int h)
{
#if(BD_OVERLAY_INTERFACE_VERSION >= 2)
  CRectInt ovr(x
          , y
          , x + w
          , y + h);

  /* fixup existing overlays */
  for(SOverlays::iterator it = plane.o.begin(); it != plane.o.end();)
  {
    CRectInt old((*it)->x
            , (*it)->y
            , (*it)->x + (*it)->width
            , (*it)->y + (*it)->height);

    std::vector<CRectInt> rem = old.SubtractRect(ovr);

    /* if no overlap we are done */
    if(rem.size() == 1 && !(rem[0] != old))
    {
      ++it;
      continue;
    }

    SOverlays add;
    for(std::vector<CRectInt>::iterator itr = rem.begin(); itr != rem.end(); ++itr)
    {
      SOverlay overlay =
          std::make_shared<CDVDOverlayImage>(*(*it), itr->x1, itr->y1, itr->Width(), itr->Height());
      add.push_back(overlay);
    }

    it = plane.o.erase(it);
    plane.o.insert(it, add.begin(), add.end());
  }
#endif
}

void CDVDInputStreamBluray::OverlayFlush(int64_t pts)
{
#if(BD_OVERLAY_INTERFACE_VERSION >= 2)
  auto group = std::make_shared<CDVDOverlayGroup>();
  group->bForced       = true;
  group->iPTSStartTime = static_cast<double>(pts);
  group->iPTSStopTime  = 0;
  group->SetDiscMenu(true);
  group->SetOverlayContainerFlushable(false);

  for(SPlane& plane : m_planes)
  {
    for(SOverlays::iterator it = plane.o.begin(); it != plane.o.end(); ++it)
      group->m_overlays.push_back(*it);
  }

  m_player->OnDiscNavResult(static_cast<void*>(&group), BD_EVENT_MENU_OVERLAY);
  m_hasOverlay = true;
#endif
}

// libbluray draws the first menu page of a play item before its event reaches us, so a page
// already on screen is converted again when the range changes
void CDVDInputStreamBluray::UpdateMenuRange(bool hdr)
{
#if(BD_OVERLAY_INTERFACE_VERSION >= 2)
  std::unique_lock lock(m_overlayLock);

  if (hdr == m_hdrVideo)
    return;
  m_hdrVideo = hdr;

  bool changed = false;
  for (SPlane& plane : m_planes)
  {
    for (SOverlay& o : plane.o)
    {
      const bool paletted = !o->palette.empty();
      if (paletted && (&plane != &m_planes[BD_OVERLAY_IG] || plane.palette.empty()))
        continue;
      SOverlay copy = std::make_shared<CDVDOverlayImage>(*o, o->x, o->y, o->width, o->height);
      copy->m_isHDROverlay = hdr;
      if (paletted)
      {
        for (unsigned i = 0; i < 256; i++)
          copy->palette[i] = build_rgba(plane.palette[i], hdr);
      }
      o = copy;
      changed = true;
    }
  }
  // a draw in progress posts the plane with its own flush
  if (changed && !m_planes[0].drawing && !m_planes[1].drawing)
    OverlayFlush(-1);
#endif
}

void CDVDInputStreamBluray::UpdatePendingMenuRange(int64_t titleTime)
{
  if (!m_pendingMenuRange || titleTime < m_pendingMenuRangeStart)
    return;
  UpdateMenuRange(*m_pendingMenuRange);
  m_pendingMenuRange.reset();
}

void CDVDInputStreamBluray::OverlayCallback(const BD_OVERLAY * const ov)
{
#if(BD_OVERLAY_INTERFACE_VERSION >= 2)
  std::unique_lock lock(m_overlayLock);

  if(ov == nullptr || ov->cmd == BD_OVERLAY_CLOSE)
  {
    OverlayClose();
    return;
  }

  if (ov->plane > 1)
  {
    CLog::Log(LOGWARNING, "CDVDInputStreamBluray - Ignoring overlay with multiple planes");
    return;
  }

  SPlane& plane(m_planes[ov->plane]);
  plane.drawing = ov->cmd != BD_OVERLAY_FLUSH;

  if (ov->cmd == BD_OVERLAY_CLEAR)
  {
    plane.o.clear();
    return;
  }

  if (ov->cmd == BD_OVERLAY_INIT)
  {
    OverlayInit(plane, ov->w, ov->h);
    return;
  }

  if (ov->cmd == BD_OVERLAY_DRAW &&
      (ov->w == 0 || ov->h == 0 || ov->x >= plane.w || ov->y >= plane.h))
  {
    CLog::Log(
        LOGWARNING,
        "CDVDInputStreamBluray - invalid overlay rectangle {}x{} at {},{} on plane {} ({}x{})",
        ov->w, ov->h, ov->x, ov->y, ov->plane, plane.w, plane.h);
    return;
  }

  if (ov->cmd == BD_OVERLAY_DRAW || ov->cmd == BD_OVERLAY_WIPE)
    OverlayClear(plane, ov->x, ov->y, ov->w, ov->h);

  /* uncompress and draw bitmap */
  if (ov->img && ov->cmd == BD_OVERLAY_DRAW)
  {
    SOverlay overlay = std::make_shared<CDVDOverlayImage>();
    // a palette uses the colour matrix of the video it belongs to
    overlay->m_isHDROverlay = ov->plane == BD_OVERLAY_IG && m_hdrVideo;

    if (ov->palette)
    {
      if (ov->plane == BD_OVERLAY_IG)
        plane.palette.assign(ov->palette, ov->palette + 256);
      overlay->palette.resize(256);

      for(unsigned i = 0; i < 256; i++)
        overlay->palette[i] = build_rgba(ov->palette[i], overlay->m_isHDROverlay);
    }
    else
      overlay->palette.clear();

    const BD_PG_RLE_ELEM *rlep = ov->img;
    const size_t bytes = static_cast<size_t>(ov->w) * ov->h;
    overlay->pixels.resize(bytes);

    for (size_t i = 0; i < bytes; i += rlep->len, rlep++)
    {
      if (rlep->len > bytes - i)
      {
        CLog::Log(LOGWARNING, "CDVDInputStreamBluray - invalid overlay RLE run");
        return;
      }
      memset(overlay->pixels.data() + i, rlep->color, rlep->len);
    }

    overlay->linesize = ov->w;
    overlay->x = ov->x;
    overlay->y = ov->y;
    overlay->height = ov->h;
    overlay->width = ov->w;
    overlay->source_height = plane.h;
    overlay->source_width = plane.w;
    if (ov->w > plane.w - ov->x || ov->h > plane.h - ov->y)
      overlay = std::make_shared<CDVDOverlayImage>(*overlay, ov->x, ov->y,
                                                   std::min<int>(ov->w, plane.w - ov->x),
                                                   std::min<int>(ov->h, plane.h - ov->y));
    plane.o.push_back(overlay);
  }

  if (ov->cmd == BD_OVERLAY_FLUSH)
    OverlayFlush(ov->pts);
#endif
}

#ifdef HAVE_LIBBLURAY_BDJ
void CDVDInputStreamBluray::OverlayCallbackARGB(const struct bd_argb_overlay_s * const ov)
{
  std::unique_lock lock(m_overlayLock);

  if(ov == nullptr || ov->cmd == BD_ARGB_OVERLAY_CLOSE)
  {
    OverlayClose();
    return;
  }

  if (ov->plane > 1)
  {
    CLog::Log(LOGWARNING, "CDVDInputStreamBluray - Ignoring overlay with multiple planes");
    return;
  }

  SPlane& plane(m_planes[ov->plane]);
  plane.drawing = ov->cmd != BD_ARGB_OVERLAY_FLUSH;

  if (ov->cmd == BD_ARGB_OVERLAY_INIT)
  {
    OverlayInit(plane, ov->w, ov->h);
    return;
  }

  if (ov->cmd == BD_ARGB_OVERLAY_DRAW)
    OverlayClear(plane, ov->x, ov->y, ov->w, ov->h);

  /* uncompress and draw bitmap */
  if (ov->argb && ov->cmd == BD_ARGB_OVERLAY_DRAW)
  {
    SOverlay overlay = std::make_shared<CDVDOverlayImage>();
    // UHD BD-J apps draw their HDR artwork already PQ-coded, as a UHD player shows it
    overlay->m_isHDROverlay = m_hdrVideo;

    overlay->palette.clear();
    overlay->linesize = ov->w * 4;
    overlay->pixels.resize(static_cast<size_t>(overlay->linesize) * ov->h);

    // argb points into libbluray's full width image, so the last line's stride runs past its end
    for (int row = 0; row < ov->h; ++row)
      memcpy(overlay->pixels.data() + static_cast<size_t>(row) * overlay->linesize,
             ov->argb + static_cast<size_t>(row) * ov->stride, overlay->linesize);

    overlay->x = ov->x;
    overlay->y = ov->y;
    overlay->height = ov->h;
    overlay->width = ov->w;
    overlay->source_height = plane.h;
    overlay->source_width = plane.w;
    plane.o.push_back(overlay);
  }

  if(ov->cmd == BD_ARGB_OVERLAY_FLUSH)
    OverlayFlush(ov->pts);
}
#endif


int CDVDInputStreamBluray::GetTotalTime()
{
  if(m_titleInfo)
    return static_cast<int>(m_titleInfo->duration / 90);
  else
    return m_endedTitleTime;
}

int CDVDInputStreamBluray::GetTime()
{
  return m_dispTimeBeforeRead;
}

bool CDVDInputStreamBluray::PosTime(int ms)
{
  if(bd_seek_time(m_bd, ms * 90) < 0)
    return false;

  m_clipStart.clear();
  EMPTY_QUEUE(m_clipQueue);
  // a held still belongs to the position the seek left
  if (m_event.event != BD_EVENT_NONE && m_event.event != BD_EVENT_STILL_TIME)
    ProcessEvent();
  while (bd_get_event(m_bd, &m_event))
    ProcessEvent();
  // the player's own seek repositions an external subtitle
  m_itemSeeked = false;
  OpenClipSpan();

  if (m_bMVCPlayback)
  {
    OpenNextStream();
    SeekMVCDemux(ms - m_clipStartTime);
  }
  return true;
}

int CDVDInputStreamBluray::GetChapterCount()
{
  if(m_titleInfo)
    return static_cast<int>(m_titleInfo->chapter_count);
  else
    return 0;
}

int CDVDInputStreamBluray::GetChapter()
{
  if(m_titleInfo)
    return static_cast<int>(bd_get_current_chapter(m_bd) + 1);
  else
    return 0;
}

void CDVDInputStreamBluray::GetChapterName(std::string& name, int ch)
{
  name.clear();

#if (BLURAY_VERSION >= BLURAY_VERSION_CODE(1, 5, 0))
  if (ch == -1 || ch > GetChapterCount())
    ch = GetChapter();
  if (ch < 1 || ch > GetChapterCount())
    return;

  if (m_titleInfo && m_titleInfo->chapters && m_titleInfo->chapters[ch - 1].chapter_name)
    name = m_titleInfo->chapters[ch - 1].chapter_name;
#endif
}

bool CDVDInputStreamBluray::SeekChapter(int ch)
{
  if(m_titleInfo && bd_seek_chapter(m_bd, ch-1) < 0)
    return false;

  m_clipStart.clear();
  EMPTY_QUEUE(m_clipQueue);
  // a held still belongs to the position the seek left
  if (m_event.event != BD_EVENT_NONE && m_event.event != BD_EVENT_STILL_TIME)
    ProcessEvent();
  while (bd_get_event(m_bd, &m_event))
    ProcessEvent();
  // the player's own seek repositions an external subtitle
  m_itemSeeked = false;
  OpenClipSpan();

  if (m_bMVCPlayback)
  {
    OpenNextStream();
    SeekMVCDemux((GetChapterPos(ch) - std::chrono::milliseconds(m_clipStartTime)).count());
  }
  return true;
}

std::chrono::milliseconds CDVDInputStreamBluray::GetChapterPos(int ch)
{
  if (ch == -1 || ch > GetChapterCount())
    ch = GetChapter();

  if (m_titleInfo && m_titleInfo->chapters)
    return std::chrono::milliseconds{m_titleInfo->chapters[ch - 1].start / 90};
  else
    return std::chrono::milliseconds{0};
}

int64_t CDVDInputStreamBluray::Seek(int64_t offset, int whence)
{
#if LIBBLURAY_BYTESEEK
  if(whence == SEEK_POSSIBLE)
    return 1;
  else if(whence == SEEK_CUR)
  {
    if(offset == 0)
      return bd_tell(m_bd);
    else
      offset += bd_tell(m_bd);
  }
  else if(whence == SEEK_END)
    offset += bd_get_title_size(m_bd);
  else if(whence != SEEK_SET)
    return -1;

  int64_t pos = bd_seek(m_bd, offset);
  if(pos < 0)
  {
    CLog::Log(LOGERROR, "CDVDInputStreamBluray::Seek - seek to {}, failed with {}", offset, pos);
    return -1;
  }

  if(pos != offset)
    CLog::Log(LOGWARNING, "CDVDInputStreamBluray::Seek - seek to {}, ended at {}", offset, pos);

  return offset;
#else
  if (whence == DVDSTREAM_SEEK_POSSIBLE)
    return 0;
  return -1;
#endif
}

int64_t CDVDInputStreamBluray::GetLength()
{
  return static_cast<int64_t>(bd_get_title_size(m_bd));
}

static bool find_stream(int pid, BLURAY_STREAM_INFO *info, int count, std::string &language)
{
  int i=0;
  for(;i<count;i++,info++)
  {
    if(info->pid == static_cast<uint16_t>(pid))
      break;
  }
  if(i==count)
    return false;
  language = reinterpret_cast<char*>(info->lang);
  return true;
}

static bool is_first_stream(int pid, const BLURAY_STREAM_INFO* info, int count)
{
  return count > 0 && info[0].pid == static_cast<uint16_t>(pid);
}

bool CDVDInputStreamBluray::GetPlaylistStreamLanguage(int pid, std::string& language) const
{
  if (pid == HDMV_PID_VIDEO || pid == HDMV_PID_VIDEO_EL)
    return find_stream(pid, m_clip->video_streams, m_clip->video_stream_count, language);
  if (HDMV_PID_AUDIO_FIRST <= pid && pid <= HDMV_PID_AUDIO_LAST)
    return find_stream(pid, m_clip->audio_streams, m_clip->audio_stream_count, language);
  if ((HDMV_PID_PG_FIRST <= pid && pid <= HDMV_PID_PG_LAST) ||
      (HDMV_PID_PG_HDR_FIRST <= pid && pid <= HDMV_PID_PG_HDR_LAST))
    return find_stream(pid, m_clip->pg_streams, m_clip->pg_stream_count, language);
  if (HDMV_PID_IG_FIRST <= pid && pid <= HDMV_PID_IG_LAST)
    return find_stream(pid, m_clip->ig_streams, m_clip->ig_stream_count, language);

  CLog::Log(LOGDEBUG, "CDVDInputStreamBluray::GetStreamInfo - unhandled pid {}", pid);
  return false;
}

bool CDVDInputStreamBluray::GetClipStreamLanguage(int pid, std::string& language) const
{
  if (!m_clipInfo)
    return false;

  const CLPI_PROG_INFO& programs{m_clipInfo->program};
  for (unsigned int i = 0; i < programs.num_prog; ++i)
  {
    const CLPI_PROG& program{programs.progs[i]};
    for (unsigned int j = 0; j < program.num_streams; ++j)
    {
      const CLPI_PROG_STREAM& stream{program.streams[j]};
      if (stream.pid != static_cast<uint16_t>(pid))
        continue;

      // The language is three characters, and absent for a stream that has none (video)
      if (stream.lang[0] == 0)
        return false;

      language = std::string(stream.lang, strnlen(stream.lang, sizeof(stream.lang)));
      return true;
    }
  }

  return false;
}

void CDVDInputStreamBluray::GetStreamInfo(int pid, std::string &language)
{
  if(!m_titleInfo || !m_clip)
    return;

  if (GetPlaylistStreamLanguage(pid, language))
    return;

  // The playlist's stream number table lists only the streams it presents, whereas the m2ts of its
  // clip commonly carries more - two playlists sharing a clip each present their own selection of
  // them. The demuxer exposes every stream of the transport stream, so the language of one the
  // playlist does not present comes from the clip information, leaving it named rather than
  // unknown. Which stream the playlist starts on is unaffected (see IsDefaultStream).
  if (!GetClipStreamLanguage(pid, language))
    CLog::Log(LOGDEBUG,
              "CDVDInputStreamBluray::GetStreamInfo - no language for pid {} in the playlist or "
              "its clip",
              pid);
}

bool CDVDInputStreamBluray::IsDefaultStream(int pid) const
{
  if (!m_titleInfo || !m_clip)
    return false;

  // The clip's stream number table lists the primary streams in stream number order, and a player
  // starts with audio stream number 1 (PSR1) and presentation graphic stream number 1 (PSR2), so
  // the first entry of each is the disc's default.
  if (HDMV_PID_AUDIO_FIRST <= pid && pid <= HDMV_PID_AUDIO_LAST)
    return is_first_stream(pid, m_clip->audio_streams, m_clip->audio_stream_count);
  if ((HDMV_PID_PG_FIRST <= pid && pid <= HDMV_PID_PG_LAST) ||
      (HDMV_PID_PG_HDR_FIRST <= pid && pid <= HDMV_PID_PG_HDR_LAST))
    return is_first_stream(pid, m_clip->pg_streams, m_clip->pg_stream_count);

  return false;
}

static unsigned int stream_number(int pid, const BLURAY_STREAM_INFO* info, int count)
{
  for (int i = 0; i < count; ++i)
  {
    if (info[i].pid == static_cast<uint16_t>(pid))
      return i + 1;
  }
  return 0;
}

bool CDVDInputStreamBluray::CanSelectStream() const
{
  // libbluray already plays the play item of a held event, which m_clip does not describe yet
  return m_bd && m_navmode && m_titleInfo && m_clip && m_hold != HOLD_HELD &&
         m_event.event == BD_EVENT_NONE;
}

// a pick or toggle while the read is held is written once the read resumes, for the play item
// libbluray then plays
void CDVDInputStreamBluray::SelectPendingStreams()
{
  if ((!m_audioPidPending && !m_pgPidPending && !m_pgVisiblePending) || !CanSelectStream())
    return;

  if (const auto pid = std::exchange(m_audioPidPending, std::nullopt))
    SetActiveAudioStream(*pid);
  if (const auto pid = std::exchange(m_pgPidPending, std::nullopt))
    SetActiveSubtitleStream(*pid, m_pgVisiblePending.value_or(m_pgEnabled));
  else if (m_pgVisiblePending)
    SetSubtitleVisible(*m_pgVisiblePending);
}

bool CDVDInputStreamBluray::SetActiveAudioStream(int pid)
{
  if (!m_bd || !m_navmode)
    return false;
  if (!CanSelectStream())
  {
    m_audioPidPending = pid;
    return true;
  }
  m_audioPidPending.reset();

  const unsigned int number{stream_number(pid, m_clip->audio_streams, m_clip->audio_stream_count)};
  if (number == 0)
    return false;

  // returns an int only since libbluray 1.4.1, and 0 also when nothing changed
  bd_select_stream(m_bd, BLURAY_AUDIO_STREAM, number, 1);
  return true;
}

bool CDVDInputStreamBluray::SetActiveSubtitleStream(int pid, bool visible)
{
  if (!m_bd || !m_navmode)
    return false;
  if (!CanSelectStream())
  {
    m_pgPidPending = pid;
    m_pgVisiblePending = visible;
    return true;
  }
  m_pgPidPending.reset();
  m_pgVisiblePending.reset();

  const unsigned int number{stream_number(pid, m_clip->pg_streams, m_clip->pg_stream_count)};
  if (number == 0)
    return false;

  // the flag rides along with the stream number, so keep the disc's while it forbids toggling
  bd_select_stream(m_bd, BLURAY_PG_TEXTST_STREAM, number,
                   CanToggleSubtitle() ? visible : m_pgEnabled);
  m_pgStreamNumber = number;
  return true;
}

void CDVDInputStreamBluray::SetSubtitleVisible(bool visible)
{
  if (!m_bd || !m_navmode)
    return;
  if (!CanSelectStream())
  {
    m_pgVisiblePending = visible;
    return;
  }
  m_pgVisiblePending.reset();

  if (!CanToggleSubtitle() || m_pgStreamNumber == 0 ||
      m_pgStreamNumber > static_cast<uint32_t>(m_clip->pg_stream_count))
    return;

  bd_select_stream(m_bd, BLURAY_PG_TEXTST_STREAM, m_pgStreamNumber, visible);
}

// the audio and presentation graphic bits of the mask are reported since libbluray 1.4.1
bool CDVDInputStreamBluray::CanChangeAudioStream() const
{
#if (BLURAY_VERSION >= BLURAY_VERSION_CODE(1, 4, 1))
  return !m_navmode || !(m_uoMask & BLURAY_UO_PRIMARY_AUDIO_CHANGE_MASK);
#else
  return true;
#endif
}

bool CDVDInputStreamBluray::CanChangeSubtitleStream() const
{
#if (BLURAY_VERSION >= BLURAY_VERSION_CODE(1, 4, 1))
  return !m_navmode || !(m_uoMask & BLURAY_UO_PG_TEXTST_CHANGE_MASK);
#else
  return true;
#endif
}

bool CDVDInputStreamBluray::CanToggleSubtitle() const
{
#if (BLURAY_VERSION >= BLURAY_VERSION_CODE(1, 4, 1))
  return !m_navmode || !(m_uoMask & BLURAY_UO_PG_TEXTST_ENABLE_DISABLE_MASK);
#else
  return true;
#endif
}

std::optional<CDVDInputStreamBluray::ClipTime> CDVDInputStreamBluray::GetClipTime(
    uint64_t readPos) const
{
  const auto it = std::ranges::upper_bound(m_clipSpans, readPos, {}, &ClipSpan::readPos);
  if (it == m_clipSpans.begin())
    return {};

  return std::prev(it)->time;
}

// the bytes of the next Read come from m_clip, or carry no clip time without one
void CDVDInputStreamBluray::OpenClipSpan()
{
  ClipSpan span{m_readPos};
  if (m_clip)
    span.time = ClipTime{m_clip->in_time, m_clip->start_time};
  m_clipSpans.push_back(span);
}

CDVDInputStream::ENextStream CDVDInputStreamBluray::NextStream()
{
  m_nextClip = nullptr;
  if (!m_navmode || m_aborted || m_hold == HOLD_ERROR || HasGivenUpReading())
    return NEXTSTREAM_NONE;

  // the title the disc program ends on keeps its time until the next stream opens
  const bool titleEndWait = m_titleEndWait;
  if (!titleEndWait)
    m_endedTitleTime = 0;

  // m_clip names the next item after a play item event, or after a seek within it
  bool clipKnown = false;
  bool clipLeft = false;
  bool programJump = false;
  bool programStop = false;
  // the clip a discontinuity opened is the one its held read is in
  const BLURAY_CLIP_INFO* joinClip = nullptr;
  const auto processEvent = [this, &clipKnown, &clipLeft, &programJump, &programStop, &joinClip]
  {
    switch (m_event.event)
    {
      case BD_EVENT_PLAYITEM:
        clipKnown = true;
        joinClip = nullptr;
        break;
      case BD_EVENT_SEEK:
        clipKnown = clipKnown || !clipLeft;
        joinClip = nullptr;
        break;
      case BD_EVENT_DISCONTINUITY:
        clipKnown = clipKnown || !clipLeft;
        if (!clipLeft && !m_clipStart.empty())
          joinClip = m_heldClip;
        break;
      case BD_EVENT_TITLE:
      case BD_EVENT_PLAYLIST:
        programJump = true;
        [[fallthrough]];
      case BD_EVENT_ANGLE:
      case BD_EVENT_END_OF_TITLE:
        clipKnown = false;
        clipLeft = true;
        break;
      case BD_EVENT_PLAYLIST_STOP:
        programStop = programJump;
        break;
      default:
        break;
    }
    CheckPresentedLeft();
    ProcessEvent();
  };

  /* process any current event */
  processEvent();

  // the old item has entered the decoders, so the disc program may learn the playlist ended
  if (m_titleEndWait)
  {
    std::vector<uint8_t> buf(GetBlockSize());
    const int result = bd_read_ext(m_bd, buf.data(), static_cast<int>(buf.size()), &m_event);
    m_readSinceReport = true;
    if (result < 0)
    {
      m_hold = HOLD_ERROR;
      return NEXTSTREAM_NONE;
    }
    if (result > 0)
    {
      HoldClipStart(buf.data(), result);
      m_titleEndWait = false;
    }
    processEvent();
  }

  /* process all queued up events */
  while(bd_get_event(m_bd, &m_event))
    processEvent();

  // The last read error can be queued with the events of a clip boundary
  if (HasGivenUpReading())
    return NEXTSTREAM_NONE;

  // an HDMV program runs only in a read and stops at its first queued event; run it to its next
  // suspension so the play item of the playlist it starts is known before the old stream closes;
  // a playlist stopped after the jump (a menu call) closes without the hold
  if (programJump && !clipKnown && m_title && !m_title->bdj && !m_titleEndWait &&
      m_hold != HOLD_STILL && m_hold != HOLD_ERROR)
  {
    while (m_title && !m_title->bdj)
    {
      if (bd_read_ext(m_bd, nullptr, 0, &m_event) < 0)
      {
        m_hold = HOLD_ERROR;
        return NEXTSTREAM_NONE;
      }
      if (m_event.event == BD_EVENT_NONE)
        break;
      processEvent();
      if (m_hold == HOLD_ERROR || m_aborted)
        return NEXTSTREAM_NONE;
    }
  }

  if (m_hold == HOLD_STILL || m_titleEndWait)
    return NEXTSTREAM_RETRY;

  if (titleEndWait)
    m_endedTitleTime = 0;
  m_menuInput = false;
  m_itemAbandoned = false;
  m_hold = HOLD_DATA;
  m_nextClip = clipKnown && m_clip && !programStop ? (joinClip ? joinClip : m_clip) : nullptr;
  m_nextItemInPlaylist = !m_playlistLeft;
  m_nextItemSeekTime.reset();
  if (m_nextItemInPlaylist && m_itemSeeked)
    m_nextItemSeekTime = static_cast<int>(bd_tell_time(m_bd) / 90);
  m_playlistLeft = false;
  m_itemSeeked = false;
  // a join within the title plays the old item out to the start of the new one
  m_playedItemEnd =
      !m_presentedLeft && m_nextClip ? static_cast<int64_t>(m_nextClip->start_time) - 1 : -1;
  m_presentedLeft = false;
  SelectPendingStreams();
  // the next demuxer reads on from m_readPos, so no lookup reaches the spans before its first one
  const auto span = std::ranges::upper_bound(m_clipSpans, m_readPos, {}, &ClipSpan::readPos);
  if (span != m_clipSpans.begin())
    m_clipSpans.erase(m_clipSpans.begin(), std::prev(span));
  return NEXTSTREAM_OPEN;
}

// libbluray settles the BD-J notifications it holds when the disc program leaves the data read;
// an angle change keeps them
void CDVDInputStreamBluray::CheckPresentedLeft()
{
  switch (m_event.event)
  {
    case BD_EVENT_SEEK:
    case BD_EVENT_PLAYLIST:
    case BD_EVENT_TITLE:
    case BD_EVENT_PLAYLIST_STOP:
      if (m_hold != HOLD_DATA)
        m_presentedLeft = true;
      break;
    default:
      break;
  }
}

void CDVDInputStreamBluray::SetPresentedTime(double time)
{
  ReportPresented(time == DVD_NOPTS_VALUE || m_presentedLeft
                      ? -1
                      : av_rescale_q(static_cast<int64_t>(time), AVRational{1, DVD_TIME_BASE},
                                     CDVDDemuxUtils::BLURAY_CLIP_TIME_BASE));
}

void CDVDInputStreamBluray::SetReadPresented()
{
  if (!m_presentedLeft)
    ReportPresented(BD_PRESENTED_ALL);
}

void CDVDInputStreamBluray::ReportItemPlayedOut()
{
  if (m_playedItemEnd >= 0)
    ReportPresented(m_playedItemEnd);
  m_playedItemEnd = -1;
}

void CDVDInputStreamBluray::ReportPresented(int64_t titleTime)
{
  if (!m_bd || !m_navmode)
    return;

  UpdatePendingMenuRange(titleTime);

  // with notifications held, libbluray is asked again when one is due, after a read or when the
  // time becomes unknown; a pending start is released only after a read or once all data read
  // is presented
  const bool known = titleTime >= 0;
  if (m_heldDue != -2 && known == (m_reportedTime >= 0) &&
      (!known || (!m_readSinceReport &&
                  (m_heldDue < 0 || titleTime < m_heldDue ||
                   (m_heldDue == 0 && titleTime != BD_PRESENTED_ALL)))))
    return;

  m_heldDue = bd_presented(m_bd, titleTime);
  m_notificationsHeld = m_heldDue != -2;
  m_readSinceReport = false;
  m_reportedTime = titleTime;
}

void CDVDInputStreamBluray::PollEvents()
{
  if (!m_bd || !m_navmode || m_aborted)
    return;

  // a move away from the held clip start shows only in the read position until the next stream
  if (m_hold == HOLD_HELD)
  {
    if (!m_clipStart.empty() && bd_tell(m_bd) != m_clipStartEnd)
    {
      m_clipStart.clear();
      if ((m_menuInput || m_notificationsHeld) && m_title && m_title->bdj)
      {
        m_menuInput = false;
        m_itemAbandoned = true;
      }
    }
    return;
  }

  // a clip start being handed on is checked by its next read
  if (m_hold == HOLD_STILL || m_hold == HOLD_ERROR || !m_clipStart.empty())
    return;

  while (true)
  {
    if (bd_read_ext(m_bd, nullptr, 0, &m_event) < 0)
    {
      m_hold = HOLD_ERROR;
      return;
    }

    if (m_event.event == BD_EVENT_NONE)
      return;
    CheckPresentedLeft();

    // a read without data joins no play item in line
    const bool menuJump = TakeMenuJump(false);

    switch (m_event.event)
    {
      case BD_EVENT_STILL_TIME:
        // an infinite still starts here as in the read
        if (m_event.param == 0)
          break;
        m_hold = HOLD_HELD;
        return;

      case BD_EVENT_PLAYITEM:
      case BD_EVENT_SEEK:
      case BD_EVENT_TITLE:
      case BD_EVENT_ANGLE:
      case BD_EVENT_PLAYLIST:
      case BD_EVENT_DISCONTINUITY:
        if (m_hold != HOLD_DATA)
        {
          if (AbandonsItem(menuJump, false))
            m_itemAbandoned = true;
          m_hold = HOLD_HELD;
          return;
        }
        break;

      case BD_EVENT_END_OF_TITLE:
        if (m_hold != HOLD_DATA && m_title && m_title->bdj)
          m_titleEndWait = true;
        break;

      default:
        break;
    }

    ProcessEvent();
    if (m_hold == HOLD_STILL || m_hold == HOLD_ERROR || m_aborted || m_hold == HOLD_HELD)
      return;
  }
}

const BLURAY_CLIP_INFO* CDVDInputStreamBluray::GetNextItemClip() const
{
  if (m_bMVCPlayback || m_hasMVCExtension)
    return nullptr;
  return m_nextClip;
}

// the clip of the playlist a title byte position is in, from the packet counts of its angle as
// libbluray lays the clips out
const BLURAY_CLIP_INFO* CDVDInputStreamBluray::GetClipAt(uint64_t titlePos) const
{
  if (!m_titleInfo)
    return nullptr;
  uint64_t clipEnd = 0;
  for (uint32_t i = 0; i < m_titleInfo->clip_count; ++i)
  {
    clipEnd += m_titleInfo->clips[i].pkt_count * SOURCE_PACKET_SIZE;
    if (titlePos < clipEnd)
      return &m_titleInfo->clips[i];
  }
  return nullptr;
}

// the disc program answers an activation key with its next seek, play item, playlist or title
bool CDVDInputStreamBluray::TakeMenuJump(bool joinsInLine)
{
  const bool menuJump =
      m_menuInput &&
      (m_event.event == BD_EVENT_SEEK || m_event.event == BD_EVENT_TITLE ||
       m_event.event == BD_EVENT_PLAYLIST || (m_event.event == BD_EVENT_PLAYITEM && !joinsInLine));
  if (menuJump)
    m_menuInput = false;
  return menuJump;
}

// a BD-J play item or seek with no data answers the key, or a notification that came with its
// picture, and leaves the item unplayed; a still has nothing read past it
bool CDVDInputStreamBluray::AbandonsItem(bool menuJump, bool fromStill) const
{
  return (menuJump || (m_notificationsHeld && !fromStill)) && m_title && m_title->bdj &&
         (m_event.event == BD_EVENT_PLAYITEM || m_event.event == BD_EVENT_SEEK);
}

bool CDVDInputStreamBluray::TakeItemAbandoned()
{
  const bool abandoned = m_itemAbandoned;
  m_itemAbandoned = false;
  return abandoned;
}

void CDVDInputStreamBluray::UserInput(bd_vk_key_e vk)
{
  if(m_bd == nullptr || !m_navmode)
    return;

  // only an activation key makes the disc program jump away from the item
  if (vk == BD_VK_ENTER || vk == BD_VK_POPUP || vk == BD_VK_ROOT_MENU ||
      (vk >= BD_VK_0 && vk <= BD_VK_9))
    m_menuInput = true;
  // the events of the key reach the player thread through its next read
  int ret = bd_user_input(m_bd, -1, vk);
  if (ret < 0)
  {
    CLog::Log(LOGDEBUG, "CDVDInputStreamBluray::UserInput - user input failed");
  }
}

bool CDVDInputStreamBluray::MouseMove(const CPoint &point)
{
  if (m_bd == nullptr || !m_navmode)
    return false;

  // Disable mouse selection for BD-J menus, since it's not implemented in libbluray as of version 1.0.2
  if (m_title && m_title->bdj == 1)
    return false;

  if (bd_mouse_select(m_bd, -1, static_cast<uint16_t>(point.x), static_cast<uint16_t>(point.y)) < 0)
  {
    CLog::Log(LOGDEBUG, "CDVDInputStreamBluray::MouseMove - mouse select failed");
    return false;
  }

  return true;
}

bool CDVDInputStreamBluray::MouseClick(const CPoint &point)
{
  if (m_bd == nullptr || !m_navmode)
    return false;

  // Disable mouse selection for BD-J menus, since it's not implemented in libbluray as of version 1.0.2
  if (m_title && m_title->bdj == 1)
    return false;

  if (bd_mouse_select(m_bd, -1, static_cast<uint16_t>(point.x), static_cast<uint16_t>(point.y)) < 0)
  {
    CLog::Log(LOGDEBUG, "CDVDInputStreamBluray::MouseClick - mouse select failed");
    return false;
  }

  m_menuInput = true;
  if (bd_user_input(m_bd, -1, BD_VK_MOUSE_ACTIVATE) >= 0)
    return true;

  CLog::Log(LOGDEBUG, "CDVDInputStreamBluray::MouseClick - mouse click (user input) failed");
  return false;
}

bool CDVDInputStreamBluray::OnMenu()
{
  if(m_bd == nullptr || !m_navmode)
  {
    CLog::Log(LOGDEBUG, "CDVDInputStreamBluray::OnMenu - navigation mode not enabled");
    return false;
  }

  m_menuInput = true;

  // we can not use this event to track a possible popup menu state since bd-j blu-rays can
  // toggle the popup menu on their own without firing this event, and if they do this, our
  // internal tracking state would be wrong. So just process and return.
  if(bd_user_input(m_bd, -1, BD_VK_POPUP) >= 0)
  {
    return true;
  }

  CLog::Log(LOGDEBUG, "CDVDInputStreamBluray::OnMenu - popup failed, trying root");

  if (bd_user_input(m_bd, -1, BD_VK_ROOT_MENU) >= 0)
  {
    return true;
  }

  CLog::Log(LOGDEBUG, "CDVDInputStreamBluray::OnMenu - root failed, trying explicit");
  if (bd_menu_call(m_bd, -1) <= 0)
  {
    CLog::Log(LOGDEBUG, "CDVDInputStreamBluray::OnMenu - root failed");
    return false;
  }
  return true;
}

bool CDVDInputStreamBluray::IsInMenu()
{
  if(m_bd == nullptr || !m_navmode)
    return false;

  // since there is no way to tell in a BD-J blu-ray when a popup menu actually is visible,
  // we have to assume that the blu-ray is in menu/navigation mode when there is an overlay
  // on screen, even if it might be invisible (which is impossible to detect)
  if(m_menu || m_hasOverlay)
    return true;
  return false;
}

void CDVDInputStreamBluray::SkipStill()
{
  if(m_bd == nullptr || !m_navmode)
    return;

  if ( m_hold == HOLD_STILL)
  {
    m_hold = HOLD_HELD;
    bd_read_skip_still(m_bd);

    /* process all queued up events */
    while (bd_get_event(m_bd, &m_event))
      ProcessEvent();
  }
}

bool CDVDInputStreamBluray::CanSeek()
{
  return !IsInMenu() || !m_isInMainMenu;
}

MenuType CDVDInputStreamBluray::GetSupportedMenuType()
{
  if (m_navmode)
  {
    return MenuType::NATIVE;
  }
  return MenuType::NONE;
}

bool CDVDInputStreamBluray::ProcessItem(int playitem)
{
  FreeTitleInfo();

  m_titleInfo = bd_get_playlist_info(m_bd, playitem, m_angle);

  if (!m_bMVCDisabled)
  {
    MPLS_PL * mpls = bd_get_title_mpls(m_bd);
    if (mpls)
    {
      for (int i = 0; i < mpls->ext_sub_count; i++)
      {
        if (mpls->ext_sub_path[i].type == 8
          && mpls->ext_sub_path[i].sub_playitem_count == mpls->list_count)
        {
          CLog::Log(LOGDEBUG, "CDVDInputStreamBluray - Enabling BD3D MVC demuxing");
          CLog::Log(LOGDEBUG, "CDVDInputStreamBluray - MVC_Base_view_R_flag: {}", m_titleInfo->mvc_base_view_r_flag);
          m_bMVCPlayback = true;
          m_hasMVCExtension = true;
          m_nMVCSubPathIndex = i;
          m_bFlipEyes = m_titleInfo->mvc_base_view_r_flag != 0;
          break;
        }
      }
    }
  }
  CloseMVCDemux();
  return true;
}

int CDVDInputStreamBluray::Get3dSubtitlePlane(uint16_t pid)
{
  if (!m_bMVCDisabled)
  {
    MPLS_PL *mpls = bd_get_title_mpls(m_bd);
    if (mpls)
    {
      for (int i = 0; i < mpls->list_count; i++)
      {
        for (int s = 0; s < mpls->play_item[i].stn.num_pg; s++)
        {
          if (mpls->play_item[i].stn.pg[s].pid == pid && mpls->play_item[i].stn.pg[s].ss_offset_sequence_id != 0xff)
            return mpls->play_item[i].stn.pg[s].ss_offset_sequence_id;
        }
      }
    }
  }

  return 0;
}

bool CDVDInputStreamBluray::OpenNextStream()
{
  if (m_clipQueue.empty())
    return false;

  int clip = m_clipQueue.front();
  m_clipQueue.pop();

  CDemuxMVC *pMVCDemux = dynamic_cast<CDemuxMVC*>(m_pMVCDemux);
  if (!pMVCDemux) {
    // either it's not a CDemuxMVC or it's 2D playback
    CloseMVCDemux();
    return OpenMVCDemux(clip);
  }

  // save start time for the next clip
  int64_t start_time = pMVCDemux->GetStartTime();
  const bool clampPreroll = pMVCDemux->GetClampPreroll();

  CloseMVCDemux();

  bool res = OpenMVCDemux(clip);
  if (res) {
    CDemuxMVC *nextDemux = dynamic_cast<CDemuxMVC*>(m_pMVCDemux);
    if (nextDemux) {
      // set start time for next clip
      CDVDInputStream::IMenus *menu = dynamic_cast<CDVDInputStream::IMenus*>(this);
      nextDemux->SetStartTime(start_time, menu->GetSupportedMenuType(), clampPreroll);
    }
  }

  return res;
}

bool CDVDInputStreamBluray::OpenMVCDemux(int playItem)
{
  MPLS_PL *pl = bd_get_title_mpls(m_bd);
  if (!pl)
    return false;

  std::string strFileName;
  strFileName.append(m_root);
  strFileName.append("/BDMV/STREAM/");
  strFileName.append(pl->ext_sub_path[m_nMVCSubPathIndex].sub_play_item[playItem].clip->clip_id);
  strFileName.append(".m2ts");

  CLog::Log(LOGDEBUG, "CDVDInputStreamBluray::OpenMVCDemuxer(): Opening MVC extension stream at {}", strFileName);

  CFileItem fileitem(CURL(strFileName), false);
  m_pMVCInput = new CDVDInputStreamFile(fileitem, 0);

  // Try to open the MVC stream
  if (!m_pMVCInput->Open())
  {
    CloseMVCDemux();
    m_bMVCPlayback = false;
    return false;
  }

  if (m_pMVCDemux)
    delete m_pMVCDemux;

  CDemuxMVC* pMVCDemux = new CDemuxMVC;
  m_pMVCDemux = pMVCDemux;

  if (!pMVCDemux->Open(m_pMVCInput))
  {
    CloseMVCDemux();
    m_bMVCPlayback = false;
    return false;
  }

  m_nMVCClip = m_titleInfo->clips + playItem;
  // the dependent view shares the in and out time of its play item
  pMVCDemux->SetClipTime(m_nMVCClip->in_time, m_nMVCClip->out_time, m_nMVCClip->start_time);
  return true;
}

bool CDVDInputStreamBluray::CloseMVCDemux()
{
  if (m_pMVCDemux)
  {
    delete m_pMVCDemux;
    m_pMVCDemux = nullptr;
  }

  delete m_pMVCInput;
  m_pMVCInput = nullptr;
  m_nMVCClip = nullptr;
  return true;
}

void CDVDInputStreamBluray::SeekMVCDemux(int64_t time)
{
  if (m_bMVCPlayback && m_pMVCDemux)
    m_pMVCDemux->SeekTime(time, time < GetTime());
}

void CDVDInputStreamBluray::SetupPlayerSettings()
{
  int region = CServiceBroker::GetSettingsComponent()->GetSettings()->GetInt(CSettings::SETTING_BLURAY_PLAYERREGION);
  if ( region != BLURAY_REGION_A
    && region != BLURAY_REGION_B
    && region != BLURAY_REGION_C)
  {
    CLog::Log(LOGWARNING, "CDVDInputStreamBluray::Open - Blu-ray region must be set in setting, assuming region A");
    region = BLURAY_REGION_A;
  }
  bd_set_player_setting(m_bd, BLURAY_PLAYER_SETTING_REGION_CODE, static_cast<uint32_t>(region));
  bd_set_player_setting(m_bd, BLURAY_PLAYER_SETTING_PARENTAL, 99);
  bd_set_player_setting(m_bd, BLURAY_PLAYER_SETTING_3D_CAP, 0xffffffff);
#if (BLURAY_VERSION < BLURAY_VERSION_CODE(1, 1, 0))
#if (BLURAY_VERSION >= BLURAY_VERSION_CODE(1, 0, 2))
  bd_set_player_setting(m_bd, BLURAY_PLAYER_SETTING_PLAYER_PROFILE, BLURAY_PLAYER_PROFILE_6_v3_1);
  bd_set_player_setting(m_bd, BLURAY_PLAYER_SETTING_UHD_CAP, 0xffffffff);
  bd_set_player_setting(m_bd, BLURAY_PLAYER_SETTING_UHD_DISPLAY_CAP, 0xffffffff);
  bd_set_player_setting(m_bd, BLURAY_PLAYER_SETTING_HDR_PREFERENCE, 0xffffffff);
#else
  bd_set_player_setting(m_bd, BLURAY_PLAYER_SETTING_PLAYER_PROFILE, BLURAY_PLAYER_PROFILE_5_v2_4);
#endif
#endif

  const std::string audioLang{g_langInfo.GetDVDAudioLanguage().AsIso6392T()};
  bd_set_player_setting_str(m_bd, BLURAY_PLAYER_SETTING_AUDIO_LANG, audioLang.c_str());

  const std::string subtitleLang{g_langInfo.GetDVDSubtitleLanguage().AsIso6392T()};
  bd_set_player_setting_str(m_bd, BLURAY_PLAYER_SETTING_PG_LANG, subtitleLang.c_str());

  const std::string menuLang{g_langInfo.GetDVDMenuLanguage().AsIso6392T()};
  bd_set_player_setting_str(m_bd, BLURAY_PLAYER_SETTING_MENU_LANG, menuLang.c_str());

  const std::string countryCode{g_langInfo.GetRegionCodeAlpha2()};
  bd_set_player_setting_str(m_bd, BLURAY_PLAYER_SETTING_COUNTRY_CODE, countryCode.c_str());

#ifdef HAVE_LIBBLURAY_BDJ
  std::string cacheDir = CSpecialProtocol::TranslatePath("special://userdata/cache/bluray/cache");
  std::string persistentDir = CSpecialProtocol::TranslatePath("special://userdata/cache/bluray/persistent");
  bd_set_player_setting_str(m_bd, BLURAY_PLAYER_PERSISTENT_ROOT, persistentDir.c_str());
  bd_set_player_setting_str(m_bd, BLURAY_PLAYER_CACHE_ROOT, cacheDir.c_str());
#endif
}

bool CDVDInputStreamBluray::OpenStream(CFileItem &item)
{
  m_pstream = std::make_unique<CDVDInputStreamFile>(
      item, XFILE::READ_TRUNCATED | XFILE::READ_BITRATE | XFILE::READ_NO_CACHE);

  if (!m_pstream->Open())
  {
    CLog::Log(LOGERROR, "Error opening image file {}", CURL::GetRedacted(item.GetPath()));
    Close();
    return false;
  }

  return true;
}

bool CDVDInputStreamBluray::GetState(std::string& xmlstate)
{
  if (!m_bd || !m_titleInfo)
  {
    return false;
  }

  BlurayState blurayState;
  blurayState.playlistId = m_titleInfo->playlist;

  if (!m_blurayStateSerializer.BlurayStateToXML(xmlstate, blurayState))
  {
    CLog::LogF(LOGWARNING, "Failed to serialize Bluray state");
    return false;
  }

  return true;
}

bool CDVDInputStreamBluray::SetState(const std::string& xmlstate)
{
  if (!m_bd)
    return false;

  BlurayState blurayState;
  if (!m_blurayStateSerializer.XMLToBlurayState(blurayState, xmlstate))
  {
    CLog::LogF(LOGWARNING, "Failed to deserialize Bluray state");
    return false;
  }

  m_titleInfo = bd_get_playlist_info(m_bd, blurayState.playlistId, 0);
  if (!m_titleInfo)
  {
    CLog::LogF(LOGERROR, "Open - failed to get title info");
    return false;
  }

  if (!bd_select_playlist(m_bd, m_titleInfo->playlist))
  {
    CLog::LogF(LOGERROR, "Open - failed to select playlist {}", m_titleInfo->idx);
    return false;
  }

  return true;
}

void CDVDInputStreamBluray::SaveCurrentState(const CStreamDetails& details)
{
  std::unique_lock lock(m_statesLock);

  if (!m_titleInfo)
    return;

  // Details for this playlist
  SavePlaylistDetails(m_playedPlaylists, m_startWatchTime,
                      {.playlist = static_cast<int>(m_titleInfo->playlist),
                       .inMenu = m_isInMainMenu,
                       .duration = std::chrono::milliseconds(GetTotalTime()),
                       .details = details});

  // Reset watch timer for next playlist
  m_startWatchTime = std::chrono::steady_clock::now();
}

CDVDInputStream::UpdateState CDVDInputStreamBluray::UpdateItemFromSavedStates(CFileItem& item,
                                                                              double time,
                                                                              bool& closed)
{
  std::unique_lock lock(m_statesLock);

  // First add current state to the list of playlist states
  if (item.HasVideoInfoTag())
    SaveCurrentState(item.GetVideoInfoTag()->m_streamDetails);

  return UpdateItemFromPlaylistDetails(DVDSTREAM_TYPE_BLURAY, m_playedPlaylists, item, time,
                                       closed);
}

void CDVDInputStreamBluray::UpdateStack(CFileItem& item)
{
  return UpdateStackItem(item,
                         m_titleInfo ? std::chrono::milliseconds(m_titleInfo->duration / 90) : 0ms);
}
