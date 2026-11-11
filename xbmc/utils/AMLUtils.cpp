/*
 *  Copyright (C) 2011-2018 Team Kodi
 *  This file is part of Kodi - https://kodi.tv
 *
 *  SPDX-License-Identifier: GPL-2.0-or-later
 *  See LICENSES/README.md for more information.
 */

#include <fcntl.h>
#include <regex>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "AMLUtils.h"
#include "cores/VideoPlayer/DVDStreamInfo.h"
#include "utils/log.h"
#include "utils/StringUtils.h"
#include "ServiceBroker.h"
#include "cores/DataCacheCore.h"
#include "cores/VideoPlayer/DVDCodecs/Video/AMLCodec.h"
#include "utils/RegExp.h"
#include "settings/Settings.h"
#include "settings/SettingsComponent.h"
#include "platform/linux/SysfsPath.h"
#include "windowing/amlogic/WinSystemAmlogic.h"
#include "windowing/WinSystem.h"

#include <amcodec/codec.h>
#if defined(HAVE_LIBBLURAY)
#include <libbluray/bluray.h>
#endif

int aml_get_cpufamily_id()
{
  static int aml_cpufamily_id = -1;
  if (aml_cpufamily_id == -1)
  {
    std::ifstream cpuinfo("/proc/cpuinfo");
    std::regex re(".*: (.*)$");

    for (std::string line; std::getline(cpuinfo, line);)
    {
      if (line.find("Serial") != std::string::npos)
      {
        std::smatch match;

        if (std::regex_match(line, match, re) && match.size() == 2)
        {
          std::ssub_match value = match[1];
          std::string cpu_family = value.str().substr(0, 2);
          try
          {
            aml_cpufamily_id = std::stoi(cpu_family, nullptr, 16);
          }
          catch (const std::exception&)
          {
            aml_cpufamily_id = -1;
          }
          break;
        }
      }
    }
  }
  return aml_cpufamily_id;
}

std::string aml_get_cpufamily_name(int cpuid)
{
  switch(cpuid)
  {
    case AML_G12A:
      return "G12A";
    case AML_G12B:
      return "G12B";
    case AML_SM1:
      return "SM1";
    case AML_SC2:
      return "SC2";
    case AML_T7:
      return "T7";
    case AML_S4:
      return "S4";
    case AML_S5:
      return "S5";
    case AML_S7:
      return "S7";
    case AML_S7D:
      return "S7D";
    case AML_S6:
      return "S6";
    default:
      {
        int resolved = aml_get_cpufamily_id();
        if (cpuid != resolved)
          return aml_get_cpufamily_name(resolved);
      }
  }
  return "Unknown";
}

static bool aml_support_vcodec_profile(const char *regex)
{
  int profile = 0;
  CRegExp regexp;
  regexp.RegComp(regex);
  std::string valstr;
  CSysfsPath vcodec_profile{"/sys/class/amstream/vcodec_profile"};
  if (vcodec_profile.Exists())
  {
    valstr = vcodec_profile.Get<std::string>().value_or("");
    profile = (regexp.RegFind(valstr) >= 0) ? 1 : 0;
  }

  return profile;
}

bool aml_support_hevc()
{
  static int has_hevc = -1;

  if (has_hevc == -1)
      has_hevc = aml_support_vcodec_profile("(\\bhevc\\b|\\bhevc_fb\\b):");

  return (has_hevc == 1);
}

bool aml_support_hevc_4k2k()
{
  static int has_hevc_4k2k = -1;

  if (has_hevc_4k2k == -1)
    has_hevc_4k2k = aml_support_vcodec_profile("(\\bhevc\\b|\\bhevc_fb\\b):(?!\\;).*(4k|8k)");

  return (has_hevc_4k2k == 1);
}

bool aml_support_hevc_8k4k()
{
  static int has_hevc_8k4k = -1;

  if (has_hevc_8k4k == -1)
    has_hevc_8k4k = aml_support_vcodec_profile("(\\bhevc\\b|\\bhevc_fb\\b):(?!\\;).*8k");

  return (has_hevc_8k4k == 1);
}

bool aml_support_hevc_10bit()
{
  static int has_hevc_10bit = -1;

  if (has_hevc_10bit == -1)
    has_hevc_10bit = aml_support_vcodec_profile("(\\bhevc\\b|\\bhevc_fb\\b):(?!\\;).*10bit");

  return (has_hevc_10bit == 1);
}

bool aml_support_h266()
{
  static int has_h266 = -1;

  if (has_h266 == -1)
    has_h266 = aml_support_vcodec_profile("\\bh266\\b:");

  return (has_h266 == 1);
}

AML_SUPPORT_H264_4K2K aml_support_h264_4k2k()
{
  static AML_SUPPORT_H264_4K2K has_h264_4k2k = AML_SUPPORT_H264_4K2K_UNINIT;

  if (has_h264_4k2k == AML_SUPPORT_H264_4K2K_UNINIT)
  {
    has_h264_4k2k = AML_NO_H264_4K2K;

    if (aml_support_vcodec_profile("(\\bh264\\b|\\bmh264\\b):4k"))
      has_h264_4k2k = AML_HAS_H264_4K2K_SAME_PROFILE;
    else if (aml_support_vcodec_profile("\\bh264_4k2k\\b:"))
      has_h264_4k2k = AML_HAS_H264_4K2K;
  }
  return has_h264_4k2k;
}

bool aml_support_vp9()
{
  static int has_vp9 = -1;

  if (has_vp9 == -1)
    has_vp9 = aml_support_vcodec_profile("(\\bvp9\\b|\\bvp9_fb\\b):(?!\\;).*compressed");

  return (has_vp9 == 1);
}

bool aml_support_av1()
{
  static int has_av1 = -1;

  if (has_av1 == -1)
    has_av1 = aml_support_vcodec_profile("(\\bav1\\b|\\bav1_fb\\b):(?!\\;).*compressed");

  return (has_av1 == 1);
}

bool aml_support_avs2()
{
  static int has_avs2 = -1;

  if (has_avs2 == -1)
    has_avs2 = aml_support_vcodec_profile("(\\bavs2\\b|\\bavs2_fb\\b):(?!\\;).*compressed");

  return (has_avs2 == 1);
}

bool aml_support_avs3()
{
  static int has_avs3 = -1;

  if (has_avs3 == -1)
    has_avs3 = aml_support_vcodec_profile("\\bavs3\\b:(?!\\;).*compressed");

  return (has_avs3 == 1);
}

bool aml_support_dolby_vision()
{
  static int support_dv = -1;

  if (support_dv == -1)
  {
    CSysfsPath support_info{"/sys/class/amdolby_vision/support_info"};
    support_dv = 0;
    if (support_info.Exists())
    {
      support_dv = (int)((support_info.Get<int>().value_or(0) & 7) == 7);
      if (support_dv == 1) {
        CSysfsPath ko_info{"/sys/class/amdolby_vision/ko_info"};
        if (ko_info.Exists())
          CLog::Log(LOGINFO, "Amlogic Dolby Vision info: {}", ko_info.Get<std::string>().value_or("").c_str());
      }
    }
  }

  return (support_dv == 1);
}

bool aml_dolby_vision_enabled()
{
  bool dv_user_enabled(!CServiceBroker::GetSettingsComponent()->GetSettings()->GetBool(CSettings::SETTING_COREELEC_AMLOGIC_DV_DISABLE));

  bool dv_enabled = (!!aml_support_dolby_vision() &&
                     static_cast<CWinSystemAmlogic*>(CServiceBroker::GetWinSystem())
                         ->GetAmlDisplay()->aml_display_support_dv());

  return ((dv_enabled && !!dv_user_enabled) == 1);
}

bool aml_convert_to_sdr(StreamHdrType hdrType)
{
  const auto settings = CServiceBroker::GetSettingsComponent()->GetSettings();
  bool user_convert_to_sdr = false;

  if (hdrType != StreamHdrType::HDR_TYPE_NONE)
    user_convert_to_sdr = settings->GetBool(CSettings::SETTING_COREELEC_AMLOGIC_HDR2SDR);

  return ((!!user_convert_to_sdr) == 1);
}

namespace
{
bool UserMapsToHdr(StreamHdrType hdrType)
{
  const auto settings = CServiceBroker::GetSettingsComponent()->GetSettings();
  bool user_convert_to_hdr = false;

  if (hdrType == StreamHdrType::HDR_TYPE_NONE)
    user_convert_to_hdr = settings->GetBool(CSettings::SETTING_COREELEC_AMLOGIC_SDR2HDR);

  bool convert_to_hdr = CServiceBroker::GetWinSystem()->IsHDRDisplay();

  return ((convert_to_hdr && !!user_convert_to_hdr) == 1);
}

bool UserMapsToDv(StreamHdrType hdrType)
{
  const auto settings = CServiceBroker::GetSettingsComponent()->GetSettings();
  bool dv_user_enabled(!settings->GetBool(CSettings::SETTING_COREELEC_AMLOGIC_DV_DISABLE));
  bool user_convert_to_dv;

  if (hdrType == StreamHdrType::HDR_TYPE_NONE)
    user_convert_to_dv = settings->GetBool(CSettings::SETTING_COREELEC_AMLOGIC_SDR2DV);
  else
    user_convert_to_dv = settings->GetBool(CSettings::SETTING_COREELEC_AMLOGIC_HDR2DV);

  bool convert_to_dv = (!!aml_support_dolby_vision() &&
                        static_cast<CWinSystemAmlogic*>(CServiceBroker::GetWinSystem())
                            ->GetAmlDisplay()->aml_display_support_dv());

  return ((convert_to_dv && !!user_convert_to_dv && !!dv_user_enabled) == 1);
}
} // namespace

std::optional<StreamHdrType> aml_disc_output_hdr_type()
{
  const DiscFormat disc = CServiceBroker::GetDataCacheCore().GetDiscFormat();
  if (disc.width == 0)
    return std::nullopt;

  StreamHdrType hdrType = disc.hdrType;
  // the demuxer opens the base layer of a Dolby Vision stream while Dolby Vision is off
  if (hdrType == StreamHdrType::HDR_TYPE_DOLBYVISION && !aml_dolby_vision_enabled())
    hdrType = StreamHdrType::HDR_TYPE_HDR10;
  if (hdrType == StreamHdrType::HDR_TYPE_HDR10 &&
      !CServiceBroker::GetWinSystem()->GetDisplayHDRCapabilities().SupportsHDR10())
    hdrType = StreamHdrType::HDR_TYPE_NONE;

  // a mapping to sdr leaves no range to keep
  if (UserMapsToDv(hdrType) ||
      (hdrType == StreamHdrType::HDR_TYPE_DOLBYVISION && !aml_convert_to_sdr(hdrType) &&
       (!aml_convert_to_sdr_by_vs_engine(hdrType) || aml_convert_to_hdr_by_vs_engine(hdrType))))
    return StreamHdrType::HDR_TYPE_DOLBYVISION;
  if (hdrType == StreamHdrType::HDR_TYPE_HDR10 && !aml_convert_to_sdr(hdrType) &&
      !aml_convert_to_sdr_by_vs_engine(hdrType))
    return hdrType;
  return std::nullopt;
}

bool aml_convert_to_hdr(StreamHdrType hdrType)
{
  if (const auto target = aml_disc_output_hdr_type())
    return hdrType == StreamHdrType::HDR_TYPE_NONE && *target == StreamHdrType::HDR_TYPE_HDR10;
  return UserMapsToHdr(hdrType);
}

bool aml_convert_to_sdr_by_vs_engine(StreamHdrType hdrType)
{
  const auto settings = CServiceBroker::GetSettingsComponent()->GetSettings();
  bool dv_user_enabled(!settings->GetBool(CSettings::SETTING_COREELEC_AMLOGIC_DV_DISABLE));
  bool user_convert_to_sdr = false;

  if (hdrType != StreamHdrType::HDR_TYPE_NONE)
    user_convert_to_sdr = settings->GetBool(CSettings::SETTING_COREELEC_AMLOGIC_DV2SDR);

  return ((!!aml_support_dolby_vision() && !!user_convert_to_sdr && !!dv_user_enabled) == 1);
}

bool aml_convert_to_hdr_by_vs_engine(StreamHdrType hdrType)
{
  const auto settings = CServiceBroker::GetSettingsComponent()->GetSettings();
  bool dv_user_enabled(!settings->GetBool(CSettings::SETTING_COREELEC_AMLOGIC_DV_DISABLE));
  bool user_convert_to_hdr = false;

  if (hdrType == StreamHdrType::HDR_TYPE_DOLBYVISION)
    user_convert_to_hdr = settings->GetBool(CSettings::SETTING_COREELEC_AMLOGIC_DV2HDR);

  bool convert_to_hdr = (!!aml_support_dolby_vision() &&
                         CServiceBroker::GetWinSystem()->IsHDRDisplay());

  return ((convert_to_hdr && !!user_convert_to_hdr && !!dv_user_enabled) == 1);
}

bool aml_convert_to_dv_by_vs_engine(StreamHdrType hdrType)
{
  if (const auto target = aml_disc_output_hdr_type())
    return hdrType != StreamHdrType::HDR_TYPE_DOLBYVISION &&
           *target == StreamHdrType::HDR_TYPE_DOLBYVISION;
  return UserMapsToDv(hdrType);
}

static float aml_hdr_module_sdr_white()
{
  // a sink tone mapping lut replaces the gain, and the driver fills it only once video runs
  CSysfsPath sbtmEnable{"/sys/module/aml_media/parameters/sbtm_en"};
  CSysfsPath sbtmMode{"/sys/module/aml_media/parameters/sbtm_mode"};
  if (sbtmEnable.Get<int>().value_or(0) != 0 && sbtmMode.Get<int>().value_or(0) != 0)
    return 0.0f;

  // sdr to hdr ootf gain, 512 is 1.0 on the 10000 nit pq scale, last entry at sdr white
  constexpr size_t OOTF_LUT_SIZE = 149;
  constexpr float OOTF_GAIN_ONE = 512.0f;
  CSysfsPath gainLut{"/sys/module/aml_media/parameters/oo_y_lut_sdr_hdr"};
  const std::vector<std::string> gains =
      StringUtils::Split(gainLut.Get<std::string>().value_or(""), ",");
  if (gains.size() != OOTF_LUT_SIZE)
    return 0.0f;

  return StringUtils::ToUint32(gains.back()) * 10000.0f / OOTF_GAIN_ONE;
}

AMLVideoOutput aml_video_output(const CDVDStreamInfo& source)
{
  const StreamHdrType hdrType = source.hdrType;
  const bool dvByVsEngine = aml_convert_to_dv_by_vs_engine(hdrType);
  bool toHdr = false;
  bool toSdr = false;
  if (hdrType == StreamHdrType::HDR_TYPE_DOLBYVISION || dvByVsEngine)
  {
    if (dvByVsEngine)
      toHdr = hdrType == StreamHdrType::HDR_TYPE_NONE;
    else
      toSdr = aml_convert_to_sdr(hdrType) || (aml_convert_to_sdr_by_vs_engine(hdrType) &&
                                              !aml_convert_to_hdr_by_vs_engine(hdrType));
  }
  else
  {
    toHdr = aml_convert_to_hdr(hdrType);
    toSdr = !toHdr && (aml_convert_to_sdr(hdrType) || aml_convert_to_sdr_by_vs_engine(hdrType));
  }

  // dovi.ko draws an sdr source as bt.1886 at its default 300 nits and does not expose it
  constexpr float DV_SDR_WHITE = 300.0f;
  constexpr float BT1886_GAMMA = 2.4f;
  // the hdr module's sdr eotf, eo_y_lut_sdr, is a 2.2 power law
  constexpr float HDR_MODULE_SDR_GAMMA = 2.2f;
  // ITU-R BT.2408 graphics white
  constexpr float REFERENCE_WHITE = 203.0f;

  AMLVideoOutput output;
  output.hdrType = hdrType;
  output.colorSpace = source.colorSpace;
  output.colorPrimaries = source.colorPrimaries;
  output.colorTransfer = source.colorTransferCharacteristic;
  if (toHdr)
  {
    output.colorSpace = AVCOL_SPC_BT2020_NCL;
    output.colorPrimaries = AVCOL_PRI_BT2020;
    output.colorTransfer = AVCOL_TRC_SMPTE2084;
    output.sdrWhite = dvByVsEngine ? DV_SDR_WHITE : aml_hdr_module_sdr_white();
    output.sdrGamma = dvByVsEngine ? BT1886_GAMMA : HDR_MODULE_SDR_GAMMA;
  }
  else if (toSdr)
  {
    output.colorSpace = AVCOL_SPC_BT709;
    output.colorPrimaries = AVCOL_PRI_BT709;
    output.colorTransfer = AVCOL_TRC_BT709;
  }
  else if (hdrType != StreamHdrType::HDR_TYPE_NONE)
  {
    output.sdrWhite = REFERENCE_WHITE;
    output.sdrGamma = BT1886_GAMMA;
  }

  if (dvByVsEngine)
    output.hdrType = StreamHdrType::HDR_TYPE_DOLBYVISION;
  else if (aml_convert_to_sdr(hdrType))
    output.hdrType = StreamHdrType::HDR_TYPE_NONE;
  else if (toHdr)
    output.hdrType = StreamHdrType::HDR_TYPE_HDR10;

  return output;
}

bool aml_video_started()
{
  CSysfsPath videostarted{"/sys/class/tsync/videostarted"};
  return (StringUtils::EqualsNoCase(videostarted.Get<std::string>().value_or("0x0"), "0x1"));
}

bool aml_display_reset_pending()
{
  return static_cast<CWinSystemAmlogic*>(CServiceBroker::GetWinSystem())->IsDisplayResetPending();
}

std::chrono::steady_clock::time_point aml_display_reset_end()
{
  return static_cast<CWinSystemAmlogic*>(CServiceBroker::GetWinSystem())->GetDisplayResetEnd();
}

#if defined(HAVE_LIBBLURAY)
namespace
{
AVCodecID GetClipVideoCodec(uint8_t codingType)
{
  switch (codingType)
  {
    case BLURAY_STREAM_TYPE_VIDEO_MPEG1:
    case BLURAY_STREAM_TYPE_VIDEO_MPEG2:
      return AV_CODEC_ID_MPEG2VIDEO;
    case BLURAY_STREAM_TYPE_VIDEO_VC1:
      return AV_CODEC_ID_VC1;
    case BLURAY_STREAM_TYPE_VIDEO_H264:
      return AV_CODEC_ID_H264;
    case BLURAY_STREAM_TYPE_VIDEO_HEVC:
      return AV_CODEC_ID_HEVC;
    default:
      return AV_CODEC_ID_NONE;
  }
}

bool IsInterlacedClipVideo(uint8_t format)
{
  return format == BLURAY_VIDEO_FORMAT_480I || format == BLURAY_VIDEO_FORMAT_576I ||
         format == BLURAY_VIDEO_FORMAT_1080I;
}

// the hdr type the decoder of a stream of the demuxer's `hdrType` puts out
StreamHdrType GetOpenedHdrType(StreamHdrType hdrType)
{
  CDVDStreamInfo source;
  source.hdrType = hdrType;
  return aml_video_output(source).hdrType;
}

// the format the decoder will be opened with for the video of `clip`
std::optional<NextVideoFormat> GetClipVideoFormat(const BLURAY_CLIP_INFO& clip)
{
  NextVideoFormat next;
  if (clip.video_stream_count == 0)
    return next;

  const BLURAY_STREAM_INFO& video = clip.video_streams[0];
  next.codec = GetClipVideoCodec(video.coding_type);
  next.interlaced = IsInterlacedClipVideo(video.format);
  // the demuxer shows a dual layer item as Dolby Vision only while Dolby Vision is enabled
  if (clip.dv_stream_count > 0 && aml_dolby_vision_enabled())
  {
    next.hdrType = GetOpenedHdrType(StreamHdrType::HDR_TYPE_DOLBYVISION);
    return next;
  }

  switch (video.dynamic_range_type)
  {
    case BLURAY_DYNAMIC_RANGE_SDR:
      break;
    case BLURAY_DYNAMIC_RANGE_HDR10:
      next.hdrType = StreamHdrType::HDR_TYPE_HDR10;
      break;
    case BLURAY_DYNAMIC_RANGE_DOLBY_VISION:
      next.hdrType = StreamHdrType::HDR_TYPE_DOLBYVISION;
      break;
    default:
      return std::nullopt;
  }
  next.hdrType = GetOpenedHdrType(next.hdrType);
  return next;
}
} // namespace
#endif

void aml_continue_presentation(const BLURAY_CLIP_INFO* next, bool jump)
{
  std::optional<NextVideoFormat> format;
#if defined(HAVE_LIBBLURAY)
  if (next)
    format = GetClipVideoFormat(*next);
#endif
  CAMLCodec::ContinuePresentation(format, jump);
}

void aml_continue_presentation(const CDVDStreamInfo& next)
{
  CAMLCodec::ContinuePresentation(
      NextVideoFormat{next.codec, aml_video_output(next).hdrType, next.interlaced});
}

void aml_end_presentation(bool stop)
{
  CAMLCodec::EndPresentation(stop);
}

void aml_drop_kept_picture(bool taken)
{
  CAMLCodec::DropKeptPicture(taken);
}

void aml_set_3d_video_mode(unsigned int mode, bool framepacking_support, int view_mode)
{
  int fd;
  if ((fd = open("/dev/amvideo", O_RDWR)) >= 0)
  {
    if (ioctl(fd, AMSTREAM_IOC_SET_3D_TYPE, mode) != 0)
      CLog::Log(LOGERROR, "AMLUtils::{} - unable to set 3D video mode {:#x}", __FUNCTION__, mode);
    close(fd);

    CSysfsPath("/sys/module/aml_media/parameters/g_framepacking_support", framepacking_support ? 1 : 0);
    CSysfsPath("/sys/module/amvdec_h264mvc/parameters/view_mode", view_mode);
  }
}
