/*
 *  Copyright (C) 2005-2018 Team Kodi
 *  This file is part of Kodi - https://kodi.tv
 *
 *  SPDX-License-Identifier: GPL-2.0-or-later
 *  See LICENSES/README.md for more information.
 */

#include "DVDFactoryInputStream.h"

#include "DVDInputStream.h"
#include "network/NetworkFileItemClassify.h"
#ifdef HAVE_LIBBLURAY
#include "DVDInputStreamBluray.h"
#endif
#include "DVDInputStreamFFmpeg.h"
#include "DVDInputStreamFile.h"
#include "DVDInputStreamNavigator.h"
#include "FileItem.h"
#include "InputStreamAddon.h"
#include "InputStreamMultiSource.h"
#include "InputStreamPVRChannel.h"
#include "InputStreamPVRRecording.h"
#include "ServiceBroker.h"
#include "URL.h"
#include "Util.h"
#include "addons/AddonManager.h"
#include "addons/addoninfo/AddonType.h"
#include "cores/VideoPlayer/Interface/InputStreamConstants.h"
#include "filesystem/CurlFile.h"
#include "filesystem/IFileTypes.h"
#include "storage/MediaManager.h"
#include "utils/FileUtils.h"
#include "utils/URIUtils.h"
#include "video/VideoFileItemClassify.h"

#include <memory>

using namespace KODI;

namespace
{
ADDON::AddonInfoPtr GetInputStreamAddon(const CFileItem& fileitem)
{
  std::vector<ADDON::AddonInfoPtr> addonInfos;
  CServiceBroker::GetAddonMgr().GetAddonInfos(addonInfos, true /*enabled only*/,
                                              ADDON::AddonType::INPUTSTREAM);
  for (const auto& addonInfo : addonInfos)
  {
    if (CInputStreamAddon::Supports(addonInfo, fileitem))
      return addonInfo;
  }
  return {};
}

bool IsFFmpegProtocol(const std::string& file)
{
  return StringUtils::StartsWithNoCase(file, "rtp://") ||
         StringUtils::StartsWithNoCase(file, "rtsp://") ||
         StringUtils::StartsWithNoCase(file, "rtsps://") ||
         StringUtils::StartsWithNoCase(file, "satip://") ||
         StringUtils::StartsWithNoCase(file, "sdp://") ||
         StringUtils::StartsWithNoCase(file, "udp://") ||
         StringUtils::StartsWithNoCase(file, "tcp://") ||
         StringUtils::StartsWithNoCase(file, "mms://") ||
         StringUtils::StartsWithNoCase(file, "mmst://") ||
         StringUtils::StartsWithNoCase(file, "mmsh://") ||
         StringUtils::StartsWithNoCase(file, "rtmp://") ||
         StringUtils::StartsWithNoCase(file, "rtmpt://") ||
         StringUtils::StartsWithNoCase(file, "rtmpe://") ||
         StringUtils::StartsWithNoCase(file, "rtmpte://") ||
         StringUtils::StartsWithNoCase(file, "rtmps://");
}

// an internet stream that FFmpeg opens itself
bool IsFFmpegStream(const CFileItem& fileitem)
{
  return fileitem.IsType(".m3u8") || fileitem.GetMimeType() == "application/vnd.apple.mpegurl" ||
         fileitem.GetMimeType() == "application/x-mpegURL" ||
         URIUtils::IsProtocol(fileitem.GetPath(), "udp");
}

enum class InputType
{
  ADDON,
  FFMPEG,
  DISC_IMAGE,
#ifdef HAS_OPTICAL_DRIVE
  OPTICAL_DRIVE,
#endif
  DVD,
  PVR_CHANNEL,
  PVR_RECORDING,
#ifdef HAVE_LIBBLURAY
  BLURAY,
#endif
  FILE,
};

// a FILE item that is an internet stream can still open in FFmpeg once its redirects resolve
InputType GetInputType(const CFileItem& fileitem, ADDON::AddonInfoPtr& addonInfo)
{
  const std::string& file = fileitem.GetDynPath();
  addonInfo = GetInputStreamAddon(fileitem);
  if (addonInfo)
    return InputType::ADDON;

  if (fileitem.GetProperty(STREAM_PROPERTY_INPUTSTREAM).asString() ==
      STREAM_PROPERTY_VALUE_INPUTSTREAMFFMPEG)
    return InputType::FFMPEG;

  if (fileitem.IsDiscImage())
    return InputType::DISC_IMAGE;

#ifdef HAS_OPTICAL_DRIVE
  if (file.compare(CServiceBroker::GetMediaManager().TranslateDevicePath("")) == 0)
    return InputType::OPTICAL_DRIVE;
#endif

  if (VIDEO::IsDVDFile(fileitem, false, true))
    return InputType::DVD;
  else if (URIUtils::IsPVRChannel(file))
    return InputType::PVR_CHANNEL;
  else if (URIUtils::IsPVRRecording(file))
    return InputType::PVR_RECORDING;
#ifdef HAVE_LIBBLURAY
  else if (fileitem.IsType(".bdmv") || fileitem.IsType(".mpls") || fileitem.IsType(".bdm") ||
           fileitem.IsType(".mpl") || StringUtils::StartsWithNoCase(file, "bluray:"))
    return InputType::BLURAY;
#endif
  else if (IsFFmpegProtocol(file))
    return InputType::FFMPEG;

  return InputType::FILE;
}
} // namespace

std::shared_ptr<CDVDInputStream> CDVDFactoryInputStream::CreateInputStream(IVideoPlayer* pPlayer, const CFileItem &fileitem, bool scanforextaudio)
{
  using namespace ADDON;

  const std::string& file = fileitem.GetDynPath();
  if (scanforextaudio)
  {
    // find any available external audio tracks
    std::vector<std::string> filenames;
    filenames.push_back(file);
    CUtil::ScanForExternalAudio(file, filenames,
                                fileitem.HasVideoInfoTag() ? fileitem.GetVideoInfoTag() : nullptr);
    if (filenames.size() >= 2)
    {
      return CreateInputStream(pPlayer, fileitem, filenames);
    }
  }

  AddonInfoPtr addonInfo;
  switch (GetInputType(fileitem, addonInfo))
  {
    case InputType::ADDON:
    {
      // Used to inform input stream about special identifier;
      const std::string instanceId =
          fileitem.GetProperty(STREAM_PROPERTY_INPUTSTREAM_INSTANCE_ID).asString();

      return std::make_shared<CInputStreamAddon>(addonInfo, pPlayer, fileitem, instanceId);
    }
    case InputType::FFMPEG:
      return std::make_shared<CDVDInputStreamFFmpeg>(fileitem);
    case InputType::DISC_IMAGE:
    {
#ifdef HAVE_LIBBLURAY
      CURL url("udf://");
      url.SetHostName(file);
      url.SetFileName("BDMV/index.bdmv");
      if (CFileUtils::Exists(url.Get()))
        return std::make_shared<CDVDInputStreamBluray>(pPlayer, fileitem);
      url.SetHostName(file);
      url.SetFileName("BDMV/INDEX.BDM");
      if (CFileUtils::Exists(url.Get()))
        return std::make_shared<CDVDInputStreamBluray>(pPlayer, fileitem);
#endif

      return std::make_shared<CDVDInputStreamNavigator>(pPlayer, fileitem);
    }
#ifdef HAS_OPTICAL_DRIVE
    case InputType::OPTICAL_DRIVE:
    {
#ifdef HAVE_LIBBLURAY
      if (CFileUtils::Exists(URIUtils::AddFileToFolder(file, "BDMV", "index.bdmv")) ||
          CFileUtils::Exists(URIUtils::AddFileToFolder(file, "BDMV", "INDEX.BDM")))
        return std::make_shared<CDVDInputStreamBluray>(pPlayer, fileitem);
#endif

      return std::make_shared<CDVDInputStreamNavigator>(pPlayer, fileitem);
    }
#endif
    case InputType::DVD:
      return std::make_shared<CDVDInputStreamNavigator>(pPlayer, fileitem);
    case InputType::PVR_CHANNEL:
      return std::make_shared<CInputStreamPVRChannel>(fileitem);
    case InputType::PVR_RECORDING:
      return std::make_shared<CInputStreamPVRRecording>(fileitem);
#ifdef HAVE_LIBBLURAY
    case InputType::BLURAY:
      return std::make_shared<CDVDInputStreamBluray>(pPlayer, fileitem);
#endif
    case InputType::FILE:
      break;
  }

  CFileItem finalFileitem(fileitem);

  if (NETWORK::IsInternetStream(finalFileitem))
  {
    if (finalFileitem.ContentLookup())
    {
      CURL origUrl(finalFileitem.GetDynURL());
      XFILE::CCurlFile curlFile;
      // try opening the url to resolve all redirects if any
      try
      {
        if (curlFile.Open(finalFileitem.GetDynURL()))
        {
          CURL finalUrl(curlFile.GetURL());
          finalUrl.SetProtocolOptions(origUrl.GetProtocolOptions());
          finalUrl.SetUserName(origUrl.GetUserName());
          finalUrl.SetPassword(origUrl.GetPassWord());
          finalFileitem.SetDynPath(finalUrl.Get());
        }
        curlFile.Close();
      }
      catch (XFILE::CRedirectException *pRedirectEx)
      {
        if (pRedirectEx)
        {
          delete pRedirectEx->m_pNewFileImp;
          delete pRedirectEx;
        }
      }
    }

    if (IsFFmpegStream(finalFileitem))
      return std::make_shared<CDVDInputStreamFFmpeg>(finalFileitem);
  }

  // our file interface handles all these types of streams
  return std::make_shared<CDVDInputStreamFile>(finalFileitem,
                                               XFILE::READ_TRUNCATED | XFILE::READ_BITRATE);
}

bool CDVDFactoryInputStream::IsPlainFile(const CFileItem& fileitem)
{
  ADDON::AddonInfoPtr addonInfo;
  return GetInputType(fileitem, addonInfo) == InputType::FILE &&
         !URIUtils::IsStack(fileitem.GetDynPath()) &&
         !(NETWORK::IsInternetStream(fileitem) && IsFFmpegStream(fileitem));
}

std::shared_ptr<CDVDInputStream> CDVDFactoryInputStream::CreateInputStream(IVideoPlayer* pPlayer, const CFileItem &fileitem, const std::vector<std::string>& filenames)
{
  return std::make_shared<CInputStreamMultiSource>(pPlayer, fileitem, filenames);
}
