/*
 *  Copyright (C) 2026 Team CoreELEC
 *
 *  SPDX-License-Identifier: GPL-2.0-or-later
 *  See LICENSES/README.md for more information.
 */

#include "utils/AMLCapture.h"

#include "ServiceBroker.h"
#include "application/ApplicationComponents.h"
#include "application/ApplicationPlayer.h"
#include "cores/VideoPlayer/DVDCodecs/Video/AMLCodec.h"
#include "cores/VideoPlayer/DVDCodecs/Video/AMLFrameMetadata.h"
#include "cores/VideoPlayer/Interface/StreamInfo.h"
#include "jobs/JobManager.h"
#include "rendering/capture/CapturePixels.h"
#include "settings/Settings.h"
#include "settings/SettingsComponent.h"
#include "utils/AMLUtils.h"
#include "utils/log.h"
#include "windowing/GraphicContext.h"
#include "windowing/amlogic/WinSystemAmlogicGLESContext.h"

#include "platform/linux/SysfsPath.h"

#include <algorithm>
#include <array>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <filesystem>
#include <limits>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <fcntl.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <sys/eventfd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

using namespace KODI::RENDERING::CAPTURE;

namespace
{

// the widest stream outside a Native shot: a Standard shot stays easy to share
constexpr int MAX_CAPTURE_WIDTH = 1920;
// the first frame after STREAMON comes within this many frame periods
constexpr int FRAME_WAIT_PERIODS = 10;
// per input of a stream, the readers convert one and keep the newest; vdin writes one, holds the
// last and one stays free
constexpr unsigned int BUFFERS_PER_INPUT = 2;
constexpr unsigned int STREAM_BUFFERS = 3;
// V4L2_PIX_FMT_AML_YUV422_10P in the vendor tvin.h, which the sysroot does not ship
constexpr uint32_t PIX_FMT_YUV422_10P = v4l2_fourcc('A', 'Y', '2', 'P');
// V4L2_CID_EXT_CAPTURE_INPUT_SCHEDULE in v4l2-controls-ext.h: vdin captures the node inputs of
// its payload in turn, one per frame, at its size
constexpr uint32_t CID_CAPTURE_INPUT_SCHEDULE = V4L2_CID_USER_BASE + 0x2100 + 9;
constexpr size_t SCHEDULE_ENTRIES = 7;
// vdin raises this to the fewest it allows
constexpr unsigned int SHOT_BUFFERS = 1;
// V4L2_PIX_FMT_FLAG_AML_COSITED_CHROMA in the vendor tvin.h
constexpr uint8_t PIX_FMT_FLAG_COSITED_CHROMA = 0x80;
// DV player-led, dv2hdr and sdr2hdr frames are already display-mapped
constexpr float MAPPED_PEAK_NITS = 1000.0f;
// the grade of a Dolby Vision TV-led signal when the stream carries no HDR statics
constexpr float TVLED_PEAK_NITS = 4000.0f;
// the nominal display of ITU-R BT.2100 HLG
constexpr float HLG_PEAK_NITS = 1000.0f;
// the range of ITU-R BT.2100 PQ, the peak of a stream that states none
constexpr float PQ_PEAK_NITS = 10000.0f;
constexpr uint32_t BUFFER_TYPE = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
// far above the float error of a normalized edge times a size, far below a pixel
constexpr float CROP_END_EPSILON = 0.01f;

// amdv dv_mode and dv_ll_policy values (dolby_vision.h, amdv.h)
enum class DvMode
{
  IPT = 0,
  IPT_TUNNEL = 1,
  HDR10 = 2,
  SDR10 = 3,
  SDR8 = 4,
};
constexpr int DV_LL_YUV422 = 1;

// amvecm hdr_conversion_ctrl values (HDR_OUTPUT_MODE_* in amvecm.h)
enum class HdrOutput
{
  HDR10 = 1,
  HLG = 2,
  HDR10PLUS = 3,
  CUVA_HDR = 4,
  CUVA_HLG = 5,
};

constexpr std::array<AMLCaptureInput, 2> INPUTS = {AMLCaptureInput::SCREEN, AMLCaptureInput::VIDEO};

uint32_t Bit(AMLCaptureInput input)
{
  return 1u << static_cast<int>(input);
}

// the payload of CID_CAPTURE_INPUT_SCHEDULE; 0 x 0 is the size of S_FMT
struct CaptureSchedule
{
  uint32_t count{0};
  uint32_t width{0};
  uint32_t height{0};
  uint32_t inputs[SCHEDULE_ENTRIES]{};
};

struct AMLCaptureDevice
{
  std::string path;
  bool vdin1{false};
  //! per AMLCaptureInput, the node input of its tap, -1 without one
  std::array<int, 2> inputs{-1, -1};
  //! the widest scaled capture
  int maxWidth{0};
  bool shot{false};

  bool Offers(AMLCaptureInput input) const { return inputs[static_cast<size_t>(input)] >= 0; }
};

void FitOutput(int maxWidth, int& width, int& height)
{
  const RESOLUTION_INFO screen = CServiceBroker::GetWinSystem()->GetGfxContext().GetResInfo();
  const int screenWidth = std::max(screen.iScreenWidth, 1);
  const int screenHeight = std::max(screen.iScreenHeight, 1);
  const float scale = std::min(1.0f, static_cast<float>(maxWidth) / screenWidth);
  width = static_cast<int>(screenWidth * scale) & ~1;
  height = static_cast<int>(screenHeight * scale) & ~1;
}

std::vector<AMLCaptureDevice> Discover()
{
  std::vector<AMLCaptureDevice> devices;

  // only vdin nodes are opened: other V4L2 drivers allocate on open
  std::error_code ec;
  for (const auto& entry : std::filesystem::directory_iterator("/sys/class/video4linux", ec))
  {
    CSysfsPath name{entry.path().string() + "/name"};
    if (name.Get<std::string>().value_or("") != "vdin_video")
      continue;

    const std::string path = "/dev/" + entry.path().filename().string();
    const int fd = open(path.c_str(), O_RDWR | O_NONBLOCK | O_CLOEXEC);
    if (fd < 0)
      continue;

    // a node is an engine by its loopback taps; vdin binds a tap to a write-back channel itself,
    // so the first input of a tap names it
    AMLCaptureDevice device;
    device.path = path;
    v4l2_capability cap{};
    device.vdin1 = ioctl(fd, VIDIOC_QUERYCAP, &cap) == 0 &&
                   std::string(reinterpret_cast<const char*>(cap.bus_info)) == "vdin1 v4l2";
    v4l2_input input{};
    for (input.index = 0; ioctl(fd, VIDIOC_ENUMINPUT, &input) == 0; input.index++)
    {
      const std::string inputName(reinterpret_cast<const char*>(input.name));
      if (!inputName.starts_with("VIU1_"))
        continue;
      int* port = nullptr;
      if (inputName.ends_with("_VD1"))
        port = &device.inputs[static_cast<size_t>(AMLCaptureInput::VIDEO)];
      else if (inputName.ends_with("_POST_BLEND"))
        port = &device.inputs[static_cast<size_t>(AMLCaptureInput::SCREEN)];
      if (port && *port < 0)
        *port = static_cast<int>(input.index);
    }
    v4l2_fmtdesc format{};
    format.type = BUFFER_TYPE;
    for (format.index = 0; ioctl(fd, VIDIOC_ENUM_FMT, &format) == 0; format.index++)
      device.shot = device.shot || format.pixelformat == PIX_FMT_YUV422_10P;
    v4l2_frmsizeenum size{};
    size.pixel_format = V4L2_PIX_FMT_NV12;
    if (ioctl(fd, VIDIOC_ENUM_FRAMESIZES, &size) == 0)
      device.maxWidth = static_cast<int>(size.stepwise.max_width);
    else
      device.inputs = {-1, -1};
    close(fd);

    if (!device.Offers(AMLCaptureInput::VIDEO) && !device.Offers(AMLCaptureInput::SCREEN))
      continue;
    CLog::Log(LOGINFO, "CAMLCapture::{}: capture on {}: video input {}, screen input {}{}",
              __FUNCTION__, device.path, device.inputs[static_cast<size_t>(AMLCaptureInput::VIDEO)],
              device.inputs[static_cast<size_t>(AMLCaptureInput::SCREEN)],
              device.shot ? ", 10-bit shots" : "");
    devices.push_back(device);
  }

  // placement breaks ties by engine index, so vdin1, the node of a capture alone, is engine 0; the
  // others follow in path order, as the directory order varies
  std::sort(devices.begin(), devices.end(), [](const AMLCaptureDevice& a, const AMLCaptureDevice& b)
            { return a.vdin1 != b.vdin1 ? a.vdin1 : a.path < b.path; });
  if (devices.empty())
    CLog::Log(LOGINFO, "CAMLCapture::{}: no vdin capture node", __FUNCTION__);
  return devices;
}

const std::vector<AMLCaptureDevice>& FindDevices()
{
  static const std::vector<AMLCaptureDevice> devices = Discover();
  return devices;
}

// the title peak, else fallback
float GetPeak(const AMLCaptureSource& source, float fallback)
{
  return source.titlePeak > 0 ? source.titlePeak : fallback;
}

// the stream size that holds width x height pixels in the normalized crop
void GetStreamSize(const CRect& crop, int width, int height, int& streamWidth, int& streamHeight)
{
  CAMLCapture::GetCaptureSize(streamWidth, streamHeight);
  if (crop.IsEmpty())
    return;

  const auto size = [](int pixels, float part, int limit)
  { return std::min((static_cast<int>(std::ceil(pixels / part)) + 1) & ~1, limit); };
  streamWidth = size(width, crop.Width(), streamWidth);
  streamHeight = size(height, crop.Height(), streamHeight);
}

// width x height sRGB pixels
void DescribeSdr(int width, int height, bool rgb, CaptureResult& result)
{
  result.width = static_cast<unsigned int>(width);
  result.height = static_cast<unsigned int>(height);
  result.stride = width * (rgb ? 3 : 4);
  result.format = rgb ? AV_PIX_FMT_RGB24 : AV_PIX_FMT_BGRA;
  result.color.primaries = AVCOL_PRI_BT709;
  result.color.transfer = AVCOL_TRC_IEC61966_2_1;
  result.color.range = AVCOL_RANGE_JPEG;
}

ImageColorMetadata GetNativeColor(AMLCaptureSignal signal)
{
  ImageColorMetadata color;
  switch (CAMLCaptureConvert::GetNativeTransfer(signal))
  {
    case AMLCaptureCoding::Transfer::PQ:
      color.primaries = AVCOL_PRI_BT2020;
      color.transfer = AVCOL_TRC_SMPTE2084;
      break;
    case AMLCaptureCoding::Transfer::HLG:
      color.primaries = AVCOL_PRI_BT2020;
      color.transfer = AVCOL_TRC_ARIB_STD_B67;
      break;
    default:
      color.primaries = AVCOL_PRI_BT709;
      color.transfer = AVCOL_TRC_BT709;
      break;
  }
  color.range = AVCOL_RANGE_JPEG;
  return color;
}

class CAMLVideoPixels : public ICapturePixels
{
public:
  CAMLVideoPixels(std::weak_ptr<CAMLCapture> session,
                  std::shared_ptr<void> lease,
                  const AMLCaptureVideoSpec& spec,
                  int streamWidth,
                  int streamHeight)
    : m_session(std::move(session)),
      m_lease(std::move(lease)),
      m_spec(spec),
      m_streamWidth(streamWidth),
      m_streamHeight(streamHeight)
  {
    if (spec.shot == CaptureScreenshot::NATIVE)
      CAMLCapture::GetShotSize(spec.shot, m_shotWidth, m_shotHeight);
  }

  const uint8_t* Lock() override
  {
    std::lock_guard lock(m_lock);

    // per-frame buffers stay immutable: every Lock returns the first read
    if (m_read)
      return m_buffer.get();
    m_read = true;

    const auto session = m_session.lock();
    if (!session)
    {
      if (m_spec.shot == CaptureScreenshot::NATIVE)
        CLog::Log(LOGWARNING, "CAMLCapture::{}: no native screenshot, the video stopped",
                  __FUNCTION__);
      return nullptr;
    }

    if (m_spec.shot == CaptureScreenshot::NATIVE)
    {
      m_buffer = ReadNative(*session);
      return m_buffer.get();
    }

    // a standard screenshot is the rows of an 8-bit PNG
    const bool rgb = m_spec.shot == CaptureScreenshot::STANDARD;
    const int pixelBytes = rgb ? 3 : 4;
    auto buffer = std::make_unique_for_overwrite<uint8_t[]>(static_cast<size_t>(m_spec.width) *
                                                            m_spec.height * pixelBytes);
    const AMLCaptureImage dst =
        AMLCaptureImage::Packed(rgb ? AV_PIX_FMT_RGB24 : AV_PIX_FMT_BGRA, m_spec.width,
                                m_spec.height, buffer.get(), m_spec.width * pixelBytes);
    const bool ok =
        session->ReadVideo(m_spec.shot, m_streamWidth, m_streamHeight,
                           [this, &dst](const AMLCaptureFrame& frame, CAMLCaptureConvert& convert)
                           {
                             return convert.Convert(CAMLCapture::Crop(frame.image, m_spec.rect),
                                                    frame.color, AMLCaptureCoding(), dst);
                           });

    if (ok)
      m_buffer = std::move(buffer);
    return m_buffer.get();
  }

  bool Describe(CaptureResult& result) const override { return m_native.Describe(result); }

private:
  // the visible video of a native shot at the shot's size, which only the read knows
  std::unique_ptr<uint8_t[]> ReadNative(CAMLCapture& session)
  {
    AMLCaptureShotFrame shot;
    if (!session.ReadShot(m_spec.shot, m_shotWidth, m_shotHeight, shot))
      return {};

    const AMLCaptureImage video = CAMLCapture::Crop(shot.frame.image, m_spec.rect);
    const int height = video.height * shot.height / shot.frame.image.height;
    auto buffer =
        std::make_unique_for_overwrite<uint8_t[]>(static_cast<size_t>(video.width) * height * 6);
    const AMLCaptureImage dst = AMLCaptureImage::Packed(AV_PIX_FMT_RGB48BE, video.width, height,
                                                        buffer.get(), video.width * 6);
    const bool ok = CAMLCaptureConvert::ConvertNative(video, shot.frame.color, dst);
    m_native = {ok, shot.frame.color, video.width, height};
    if (!ok)
      return {};
    return buffer;
  }

  std::weak_ptr<CAMLCapture> m_session;
  std::shared_ptr<void> m_lease;
  AMLCaptureVideoSpec m_spec;
  int m_streamWidth{0};
  int m_streamHeight{0};
  int m_shotWidth{0};
  int m_shotHeight{0};
  std::mutex m_lock;
  bool m_read{false};
  std::unique_ptr<uint8_t[]> m_buffer;
  AMLCaptureNative m_native;
};

constexpr int FULL_SPAN_8 = 255;
constexpr int CODE_SPAN_8_TO_16 = 257;

// 16-bit RGBA codes to 8-bit pixels of STEP bytes
template<int STEP>
void Quantize(const uint16_t* in, int pixels, uint8_t* out)
{
  for (int i = 0; i < pixels; i++, in += 4, out += STEP)
  {
    for (int c = 0; c < STEP; c++)
      out[c] = static_cast<uint8_t>((in[c] + CODE_SPAN_8_TO_16 / 2) / CODE_SPAN_8_TO_16);
  }
}

// a normalized coordinate in pixels
int Edge(float v, int pixels)
{
  return std::clamp(static_cast<int>(std::lround(v * pixels)), 0, pixels);
}

AMLCaptureImage Offset(AMLCaptureImage image, int x, int y, int pixelBytes)
{
  image.data[0] += static_cast<std::ptrdiff_t>(y) * image.linesize[0] + x * pixelBytes;
  return image;
}

// fill(y, from, to) on the row spans of width x height around a w x h place at (x1, y1)
template<typename Fill>
void FillAround(int width, int height, int x1, int y1, int w, int h, const Fill& fill)
{
  for (int y = 0; y < height; y++)
  {
    if (y < y1 || y >= y1 + h)
      fill(y, 0, width);
    else
    {
      fill(y, 0, x1);
      fill(y, x1 + w, width);
    }
  }
}

class CAMLCompositePixels : public ICapturePixels
{
public:
  CAMLCompositePixels(std::weak_ptr<CAMLCapture> session,
                      CaptureResult gui,
                      const AMLCaptureCoding& coding,
                      const CRect& video)
    : m_session(std::move(session)),
      m_gui(std::move(gui)),
      m_coding(coding),
      m_video(video)
  {
    CAMLCapture::GetCaptureSize(m_streamWidth, m_streamHeight);
    CAMLCapture::GetShotSize(m_gui.screenshot, m_shotWidth, m_shotHeight);
  }

  const uint8_t* Lock() override
  {
    std::lock_guard lock(m_lock);

    if (m_read)
      return m_buffer.get();
    m_read = true;

    // read once; the encode that follows is the peak
    const std::shared_ptr<ICapturePixels> readback = std::move(m_gui.pixels);
    const CScopedCapturePixels guiPixels(*readback);
    const int width = static_cast<int>(m_gui.width);
    const int height = static_cast<int>(m_gui.height);
    const bool bgra = m_gui.format == AV_PIX_FMT_BGRA;
    const AMLCaptureImage overlay = AMLCaptureImage::Packed(
        bgra ? AV_PIX_FMT_BGRA : AV_PIX_FMT_RGBA, width, height,
        const_cast<uint8_t*>(CaptureSrcRow0(guiPixels.data(), m_gui.stride, m_gui.height)),
        m_gui.stride);
    const auto guiRow = [&overlay](int y)
    { return overlay.data[0] + static_cast<std::ptrdiff_t>(y) * overlay.linesize[0]; };
    const int r = bgra ? 2 : 0;
    const int b = bgra ? 0 : 2;
    // a standard screenshot is the rows of an 8-bit PNG
    const bool rgb = m_gui.screenshot == CaptureScreenshot::STANDARD;
    const bool native = m_gui.screenshot == CaptureScreenshot::NATIVE;
    const bool hdr = m_coding.transfer != AMLCaptureCoding::Transfer::SDR;

    // place the visible video, mapped to SDR, in the GUI's coding at its place on black, blend the
    // premultiplied GUI over it as the OSD blender does, then invert the coding to SDR, so the GUI
    // keeps its look and the video matches a video capture
    const auto blend = [&](const AMLCaptureFrame& frame, CAMLCaptureConvert& convert)
    {
      const AMLCaptureImage image = convert.GetCompositeImage(width, height);
      uint16_t* composite = reinterpret_cast<uint16_t*>(image.data[0]);

      // in an HDR coding the video under clear GUI goes straight to SDR, the value the inverse of
      // the coding would give
      const AMLCaptureImage dst =
          AMLCaptureImage::Packed(AV_PIX_FMT_RGBA, width, height, m_buffer.get(), width * 4);

      // vdin holds the visible video at the frame origin, at its size on screen
      const int x1 = Edge(m_video.x1, width);
      const int y1 = Edge(m_video.y1, height);
      AMLCaptureImage place = Offset(image, x1, y1, 8);
      place.width = Edge(m_video.x2, width) - x1;
      place.height = Edge(m_video.y2, height) - y1;
      const AMLCaptureImage overlayPlace = Offset(overlay, x1, y1, 4);
      const AMLCaptureImage dstPlace = Offset(dst, x1, y1, 4);
      if (place.width > 0 && place.height > 0 &&
          !convert.Convert(
              CAMLCapture::Crop(frame.image, CRect(0.0f, 0.0f, m_video.Width(), m_video.Height())),
              frame.color, m_coding, place, hdr ? &overlayPlace : nullptr, &dstPlace))
        return false;

      // the video place is converted whole; around it the GUI is over black
      FillAround(width, height, x1, y1, place.width, place.height,
                 [&](int y, int from, int to)
                 {
                   const size_t first = static_cast<size_t>(y) * width + from;
                   std::fill(composite + first * 4, composite + (first + to - from) * 4, 0);
                   uint8_t* out = m_buffer.get() + first * 4;
                   for (int x = from; hdr && x < to; x++, out += 4)
                   {
                     out[0] = 0;
                     out[1] = 0;
                     out[2] = 0;
                     out[3] = 255;
                   }
                 });

      for (int y = 0; y < height; y++)
      {
        const uint8_t* gui = guiRow(y);
        uint16_t* out = composite + static_cast<size_t>(y) * width * 4;
        for (int x = 0; x < width; x++, gui += 4, out += 4)
        {
          if (hdr && CAMLCaptureConvert::IsClear(gui))
            continue;
          const int alpha = gui[3];
          const int channel[3] = {gui[r], gui[1], gui[b]};
          for (int c = 0; c < 3; c++)
          {
            const int value =
                channel[c] * CODE_SPAN_8_TO_16 + (FULL_SPAN_8 - alpha) * out[c] / FULL_SPAN_8;
            out[c] = static_cast<uint16_t>(std::min(value, 65535));
          }
          out[3] = 65535;
        }
      }

      const int pixels = width * height;
      if (!hdr)
      {
        if (rgb)
          Quantize<3>(composite, pixels, m_buffer.get());
        else
          Quantize<4>(composite, pixels, m_buffer.get());
        return true;
      }

      convert.MapToSdr(image, m_coding, dst, &overlay, frame.color.luminance);

      if (rgb)
      {
        // RGBA repacked to RGB in place
        const uint8_t* in = m_buffer.get();
        uint8_t* out = m_buffer.get();
        for (int i = 0; i < pixels; i++, in += 4, out += 3)
        {
          for (int c = 0; c < 3; c++)
            out[c] = in[c];
        }
      }
      return true;
    };

    // place the visible video in its own coding at its place in the shot's size, scale the GUI to
    // that size, recode it where its coding differs and blend it over as the OSD blender does
    const auto blendNative = [&](AMLCaptureShotFrame& shot)
    {
      const AMLCaptureFrame& frame = shot.frame;
      const int shotWidth = frame.image.width;
      const int shotHeight = shot.height;
      const size_t size = static_cast<size_t>(shotWidth) * shotHeight * 4;

      AMLCaptureCoding videoCoding;
      videoCoding.transfer = CAMLCaptureConvert::GetNativeTransfer(frame.color.signal);
      videoCoding.whiteNits = m_coding.whiteNits;
      const bool recode = m_coding.transfer != videoCoding.transfer;

      m_buffer = std::make_unique_for_overwrite<uint8_t[]>(static_cast<size_t>(shotWidth) *
                                                           shotHeight * 6);
      const AMLCaptureImage image = AMLCaptureImage::Packed(
          AV_PIX_FMT_RGB48BE, shotWidth, shotHeight, m_buffer.get(), shotWidth * 6);

      const int x1 = Edge(m_video.x1, shotWidth);
      const int y1 = Edge(m_video.y1, shotHeight);
      AMLCaptureImage place = Offset(image, x1, y1, 6);
      // vdin holds the video at its size in the shot, or in a field of it, so it is placed 1:1 and
      // never resampled
      AMLCaptureImage crop =
          CAMLCapture::Crop(frame.image, CRect(0.0f, 0.0f, m_video.Width(), m_video.Height()));
      crop.width = std::min(crop.width, (shotWidth - x1) & ~1);
      place.width = crop.width;
      place.height = std::min(crop.height * shotHeight / frame.image.height, shotHeight - y1);
      crop.height = place.height * frame.image.height / shotHeight;
      // the place is converted whole; around it the video is black
      FillAround(shotWidth, shotHeight, x1, y1, place.width, place.height,
                 [&](int y, int from, int to)
                 {
                   uint8_t* row = m_buffer.get() + static_cast<size_t>(y) * shotWidth * 6;
                   std::fill(row + from * 6, row + to * 6, 0);
                 });
      if (place.width > 0 && place.height > 0 &&
          !CAMLCaptureConvert::ConvertNative(crop, frame.color, place))
        return false;

      m_native.color = frame.color;
      m_native.width = shotWidth;
      m_native.height = shotHeight;
      shot = {};

      std::unique_ptr<uint8_t[]> scaled;
      if (width != shotWidth || height != shotHeight)
      {
        scaled = std::make_unique_for_overwrite<uint8_t[]>(size);
        const AMLCaptureImage dst = AMLCaptureImage::Packed(AV_PIX_FMT_RGBA, shotWidth, shotHeight,
                                                            scaled.get(), shotWidth * 4);
        if (!CAMLCaptureConvert::Scale(overlay, dst))
          return false;
      }

      for (int y = 0; y < shotHeight; y++)
      {
        const uint8_t* gui =
            scaled ? scaled.get() + static_cast<size_t>(y) * shotWidth * 4 : guiRow(y);
        uint8_t* out = m_buffer.get() + static_cast<size_t>(y) * shotWidth * 6;
        for (int x = 0; x < shotWidth; x++, gui += 4)
        {
          const int alpha = gui[3];
          const int channel[3] = {gui[r], gui[1], gui[b]};
          int over[3] = {channel[0] * CODE_SPAN_8_TO_16, channel[1] * CODE_SPAN_8_TO_16,
                         channel[2] * CODE_SPAN_8_TO_16};
          // a premultiplied 8-bit GUI pixel recoded, as premultiplied 16-bit codes
          if (recode && alpha > 0)
          {
            float in[3];
            for (int c = 0; c < 3; c++)
              in[c] = std::clamp(static_cast<float>(channel[c]) * FULL_SPAN_8 / alpha / FULL_SPAN_8,
                                 0.0f, 1.0f);
            float coded[3];
            CAMLCaptureConvert::Recode(in, m_coding, videoCoding, coded);
            for (int c = 0; c < 3; c++)
              over[c] = static_cast<int>(
                  std::lround(coded[c] * FULL_SPAN_8 * alpha / FULL_SPAN_8 * CODE_SPAN_8_TO_16));
          }
          for (int c = 0; c < 3; c++, out += 2)
          {
            const int under = out[0] << 8 | out[1];
            const int value = over[c] + (FULL_SPAN_8 - alpha) * under / FULL_SPAN_8;
            const uint16_t blended = static_cast<uint16_t>(std::min(value, 65535));
            out[0] = static_cast<uint8_t>(blended >> 8);
            out[1] = static_cast<uint8_t>(blended);
          }
        }
      }
      return true;
    };

    if (!native)
      m_buffer = std::make_unique_for_overwrite<uint8_t[]>(static_cast<size_t>(width) * height * 4);
    const auto session = m_session.lock();
    bool ok = false;
    if (native)
    {
      AMLCaptureShotFrame shot;
      if (!session)
        CLog::Log(LOGWARNING, "CAMLCapture::{}: no native screenshot, the video stopped",
                  __FUNCTION__);
      ok = session && session->ReadShot(m_gui.screenshot, m_shotWidth, m_shotHeight, shot) &&
           blendNative(shot);
      m_native.resolved = ok;
    }
    else if (session)
      ok = session->ReadVideo(m_gui.screenshot, m_streamWidth, m_streamHeight, blend);
    // the GUI alone is clear where the video shows, and in an HDR coding would pass for SDR
    if (!ok)
    {
      if (rgb)
        CLog::Log(LOGWARNING, "CAMLCapture::{}: no screenshot, the video read failed",
                  __FUNCTION__);
      m_buffer.reset();
    }
    return m_buffer.get();
  }

  bool Describe(CaptureResult& result) const override { return m_native.Describe(result); }

private:
  std::weak_ptr<CAMLCapture> m_session;
  CaptureResult m_gui;
  AMLCaptureCoding m_coding;
  //! the visible video, normalized to the output
  CRect m_video;
  int m_streamWidth{0};
  int m_streamHeight{0};
  int m_shotWidth{0};
  int m_shotHeight{0};
  std::mutex m_lock;
  bool m_read{false};
  std::unique_ptr<uint8_t[]> m_buffer;
  AMLCaptureNative m_native;
};

class CAMLLivePixels : public ICapturePixels
{
public:
  CAMLLivePixels(std::shared_ptr<CAMLCapture> session,
                 std::shared_ptr<void> lease,
                 int width,
                 int height,
                 int streamWidth,
                 int streamHeight)
    : m_session(std::move(session)),
      m_lease(std::move(lease)),
      m_width(width),
      m_height(height),
      m_streamWidth(streamWidth),
      m_streamHeight(streamHeight)
  {
  }

  // every Lock reads a new frame, so a consumer is served while the GUI draws nothing; the
  // buffer holds it until Unlock
  const uint8_t* Lock() override
  {
    m_lock.lock();

    if (!m_buffer)
      m_buffer = std::make_unique<uint8_t[]>(static_cast<size_t>(m_width) * m_height * 4);
    const AMLCaptureImage dst =
        AMLCaptureImage::Packed(AV_PIX_FMT_BGRA, m_width, m_height, m_buffer.get(), m_width * 4);

    const bool ok =
        m_session->Read(AMLCaptureInput::SCREEN, m_streamWidth, m_streamHeight,
                        [&dst](const AMLCaptureFrame& frame, CAMLCaptureConvert& convert) {
                          return convert.Convert(frame.image, frame.color, AMLCaptureCoding(), dst);
                        });
    return ok ? m_buffer.get() : nullptr;
  }

  void Unlock() override { m_lock.unlock(); }

private:
  // the lease ends first, so the stream closes before the session can go
  std::shared_ptr<CAMLCapture> m_session;
  std::shared_ptr<void> m_lease;
  int m_width{0};
  int m_height{0};
  int m_streamWidth{0};
  int m_streamHeight{0};
  std::mutex m_lock;
  std::unique_ptr<uint8_t[]> m_buffer;
};

std::mutex s_sessionLock;
std::weak_ptr<CAMLCapture> s_session;
} // namespace

class CAMLCapture::CStream
{
public:
  enum class Wake
  {
    FRAME,
    STOPPED,
    TIMEOUT,
    LOST,
  };

  enum class Refusal
  {
    NONE,
    //! another user owns the node's queue
    OWNED,
    //! no write-back channel for one of the inputs
    NO_CHANNEL,
    //! the node takes no turns
    NO_TURNS,
  };

  //! A stream of the inputs of `device` in `inputs`, which vdin captures in turn, one per frame;
  //! with `turns` it takes turns even for one input. `refusal` tells why vdin refused it.
  static std::shared_ptr<CStream> Open(const AMLCaptureDevice& device,
                                       uint32_t inputs,
                                       uint32_t pixelFormat,
                                       unsigned int buffers,
                                       int width,
                                       int height,
                                       int logLevel,
                                       Refusal& refusal);
  ~CStream();

  void Stop();
  uint32_t GetInputs() const { return m_inputs; }
  uint32_t GetPixelFormat() const { return m_pixelFormat; }
  float GetFramePeriodMs() const { return m_framePeriodMs; }
  bool Matches(uint32_t inputs, uint32_t pixelFormat, int width, int height) const
  {
    return inputs == m_inputs && pixelFormat == m_pixelFormat && width == m_width &&
           height == m_height;
  }
  //! the stream calls `closed` once it is gone, after its node is closed
  void SetClosed(std::function<void()> closed) { m_closed = std::move(closed); }
  //! Takes the newest frame of `input` that holds width x height, or the running size when smaller;
  //! false without one or when a newer one must have come. It stays the readers' until Return.
  bool Take(AMLCaptureInput input, unsigned int& index);
  //! Any thread: the last reader of a frame queues it again
  void Return(unsigned int index);
  Wake Wait(AMLCaptureInput input, int timeoutMs, unsigned int& index);
  AMLCaptureImage Image(unsigned int index) const;
  size_t GetLength(unsigned int index) const { return m_maps[index].second; }
  //! a read-only dmabuf of buffer `index`, which keeps the buffer after the stream closes; -1 on
  //! failure
  int Export(unsigned int index) const;

private:
  bool Queue(unsigned int index);
  //! `input` is the AMLCaptureInput of the frame, -1 for one the stream lacks; `entries` and
  //! `generation` the length and the wrapping count of the turns vdin filled it under
  bool Dequeue(unsigned int& index, int& input, std::chrono::steady_clock::time_point& stamp);
  Wake Poll(int timeoutMs) const;

  int m_fd{-1};
  int m_eventFd{-1};
  uint32_t m_inputs{0};
  std::array<int, 2> m_ports{-1, -1};
  int m_entries{0};
  uint32_t m_pixelFormat{V4L2_PIX_FMT_NV12};
  int m_width{0};
  int m_height{0};
  int m_imageWidth{0};
  int m_imageHeight{0};
  int m_stride{0};
  bool m_cositedChroma{false};
  float m_framePeriodMs{0.0f};
  bool m_streaming{false};
  std::atomic<bool> m_stopped{false};
  std::vector<std::pair<void*, size_t>> m_maps;
  std::function<void()> m_closed;

  //! guards the dequeued frames: readers dequeue and return them on their own threads
  std::mutex m_bufferLock;
  std::vector<int> m_borrows;
  std::vector<std::chrono::steady_clock::time_point> m_stamps;
  std::array<int, 2> m_parked{-1, -1};
};

std::shared_ptr<CAMLCapture::CStream> CAMLCapture::CStream::Open(const AMLCaptureDevice& device,
                                                                 uint32_t inputs,
                                                                 uint32_t pixelFormat,
                                                                 unsigned int buffers,
                                                                 int width,
                                                                 int height,
                                                                 int logLevel,
                                                                 Refusal& refusal)
{
  auto stream = std::make_shared<CStream>();
  stream->m_inputs = inputs;
  stream->m_ports = device.inputs;
  int index = -1;
  CaptureSchedule schedule;
  for (AMLCaptureInput input : INPUTS)
  {
    if (!(inputs & Bit(input)))
      continue;
    if (!device.Offers(input))
      return {};
    const int port = device.inputs[static_cast<size_t>(input)];
    if (index < 0)
      index = port;
    stream->m_entries++;
    schedule.inputs[schedule.count++] = static_cast<uint32_t>(port);
  }
  if (index < 0)
    return {};

  stream->m_pixelFormat = pixelFormat;
  stream->m_width = width;
  stream->m_height = height;
  stream->m_fd = open(device.path.c_str(), O_RDWR | O_NONBLOCK | O_CLOEXEC);
  stream->m_eventFd = eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
  if (stream->m_fd < 0 || stream->m_eventFd < 0)
  {
    CLog::Log(logLevel, "CAMLCapture::{}: cannot open {}: {}", __FUNCTION__, device.path,
              strerror(errno));
    return {};
  }

  if (ioctl(stream->m_fd, VIDIOC_S_INPUT, &index) < 0)
  {
    if (errno == EBUSY)
      refusal = Refusal::OWNED;
    else
      CLog::Log(logLevel, "CAMLCapture::{}: input {} unavailable: {}", __FUNCTION__, index,
                strerror(errno));
    return {};
  }

  v4l2_format format{};
  format.type = BUFFER_TYPE;
  format.fmt.pix_mp.width = width;
  format.fmt.pix_mp.height = height;
  format.fmt.pix_mp.pixelformat = pixelFormat;
  format.fmt.pix_mp.field = V4L2_FIELD_NONE;
  format.fmt.pix_mp.num_planes = 1;
  const bool fullPack = pixelFormat == PIX_FMT_YUV422_10P;
  const int ioctlResult = ioctl(stream->m_fd, VIDIOC_S_FMT, &format);
  stream->m_stride = static_cast<int>(format.fmt.pix_mp.plane_fmt[0].bytesperline);
  stream->m_imageWidth = static_cast<int>(format.fmt.pix_mp.width);
  stream->m_imageHeight = static_cast<int>(format.fmt.pix_mp.height);
  stream->m_cositedChroma = format.fmt.pix_mp.flags & PIX_FMT_FLAG_COSITED_CHROMA;
  const size_t lumaSize = static_cast<size_t>(stream->m_stride) * stream->m_imageHeight;
  const size_t frameSize = fullPack ? lumaSize : lumaSize * 3 / 2;
  if (ioctlResult < 0 || format.fmt.pix_mp.pixelformat != pixelFormat ||
      stream->m_stride <
          (fullPack ? stream->m_imageWidth / 2 * AML_FULL_PACK_PAIR_BYTES : stream->m_imageWidth) ||
      format.fmt.pix_mp.plane_fmt[0].sizeimage < frameSize)
  {
    if (ioctlResult < 0 && errno == EBUSY)
      refusal = Refusal::OWNED;
    CLog::Log(logLevel, "CAMLCapture::{}: {} {}x{} refused", __FUNCTION__,
              fullPack ? "10-bit" : "NV12", width, height);
    return {};
  }

  v4l2_streamparm parm{};
  parm.type = BUFFER_TYPE;
  const v4l2_fract& period = parm.parm.capture.timeperframe;
  if (ioctl(stream->m_fd, VIDIOC_G_PARM, &parm) < 0 || period.numerator == 0 ||
      period.denominator == 0)
  {
    CLog::Log(logLevel, "CAMLCapture::{}: no frame period", __FUNCTION__);
    return {};
  }
  stream->m_framePeriodMs = 1000.0f * period.numerator / period.denominator;

  v4l2_requestbuffers request{};
  request.count = buffers;
  request.type = BUFFER_TYPE;
  request.memory = V4L2_MEMORY_MMAP;
  if (ioctl(stream->m_fd, VIDIOC_REQBUFS, &request) < 0 || request.count == 0)
  {
    CLog::Log(logLevel, "CAMLCapture::{}: no capture buffers: {}", __FUNCTION__, strerror(errno));
    return {};
  }

  for (unsigned int i = 0; i < request.count; i++)
  {
    v4l2_plane plane{};
    v4l2_buffer buffer{};
    buffer.index = i;
    buffer.type = BUFFER_TYPE;
    buffer.memory = V4L2_MEMORY_MMAP;
    buffer.m.planes = &plane;
    buffer.length = 1;
    if (ioctl(stream->m_fd, VIDIOC_QUERYBUF, &buffer) < 0 || plane.length < frameSize)
      return {};

    void* map =
        mmap(nullptr, plane.length, PROT_READ, MAP_SHARED, stream->m_fd, plane.m.mem_offset);
    if (map == MAP_FAILED)
      return {};
    stream->m_maps.emplace_back(map, plane.length);
  }
  stream->m_borrows.resize(stream->m_maps.size());
  stream->m_stamps.resize(stream->m_maps.size());

  for (unsigned int i = 0; i < stream->m_maps.size(); i++)
  {
    if (!stream->Queue(i))
      return {};
  }

  // vdin takes turns only from the queue's owner before it streams; one input needs none
  if (stream->m_entries > 1)
  {
    v4l2_ext_control control{};
    control.id = CID_CAPTURE_INPUT_SCHEDULE;
    control.size = sizeof(schedule);
    control.ptr = &schedule;
    v4l2_ext_controls controls{};
    controls.which = V4L2_CTRL_WHICH_CUR_VAL;
    controls.count = 1;
    controls.controls = &control;
    if (ioctl(stream->m_fd, VIDIOC_S_EXT_CTRLS, &controls) < 0)
    {
      const int error = errno;
      if (error == EBUSY)
        refusal = Refusal::OWNED;
      else if (error == EOPNOTSUPP)
        refusal = Refusal::NO_TURNS;
      if (refusal != Refusal::OWNED)
        CLog::Log(logLevel, "CAMLCapture::{}: turns of inputs {:#x} refused: {}", __FUNCTION__,
                  inputs, strerror(error));
      return {};
    }
  }

  uint32_t type = BUFFER_TYPE;
  if (ioctl(stream->m_fd, VIDIOC_STREAMON, &type) < 0)
  {
    const int error = errno;
    if (error == EBUSY)
      refusal = Refusal::NO_CHANNEL;
    else if (error == EOPNOTSUPP)
      refusal = Refusal::NO_TURNS;
    CLog::Log(logLevel, "CAMLCapture::{}: stream start failed: {}", __FUNCTION__, strerror(error));
    return {};
  }
  stream->m_streaming = true;

  CLog::Log(LOGINFO, "CAMLCapture::{}: started {} input {} of {} in turn {}x{}", __FUNCTION__,
            device.path, index, stream->m_entries, width, height);
  return stream;
}

CAMLCapture::CStream::~CStream()
{
  Stop();
  for (const auto& [map, length] : m_maps)
    munmap(map, length);
  if (m_fd >= 0)
    close(m_fd);
  if (m_eventFd >= 0)
    close(m_eventFd);
  if (m_closed)
    m_closed();
}

void CAMLCapture::CStream::Stop()
{
  if (m_stopped.exchange(true))
    return;

  if (m_eventFd >= 0)
  {
    const uint64_t wake = 1;
    if (write(m_eventFd, &wake, sizeof(wake)) < 0)
      CLog::Log(LOGDEBUG, "CAMLCapture::{}: wake failed: {}", __FUNCTION__, strerror(errno));
  }

  if (m_streaming)
  {
    // vdin serializes ioctls on the device lock, so this is safe beside a waiting reader
    uint32_t type = BUFFER_TYPE;
    ioctl(m_fd, VIDIOC_STREAMOFF, &type);
    CLog::Log(LOGINFO, "CAMLCapture::{}: stopped", __FUNCTION__);
  }
}

bool CAMLCapture::CStream::Queue(unsigned int index)
{
  v4l2_plane plane{};
  v4l2_buffer buffer{};
  buffer.index = index;
  buffer.type = BUFFER_TYPE;
  buffer.memory = V4L2_MEMORY_MMAP;
  buffer.m.planes = &plane;
  buffer.length = 1;
  return ioctl(m_fd, VIDIOC_QBUF, &buffer) == 0;
}

bool CAMLCapture::CStream::Dequeue(unsigned int& index,
                                   int& input,
                                   std::chrono::steady_clock::time_point& stamp)
{
  v4l2_plane plane{};
  v4l2_buffer buffer{};
  buffer.type = BUFFER_TYPE;
  buffer.memory = V4L2_MEMORY_MMAP;
  buffer.m.planes = &plane;
  buffer.length = 1;
  if (ioctl(m_fd, VIDIOC_DQBUF, &buffer) < 0)
    return false;

  index = buffer.index;
  stamp =
      std::chrono::steady_clock::time_point(std::chrono::seconds(buffer.timestamp.tv_sec) +
                                            std::chrono::microseconds(buffer.timestamp.tv_usec));
  input = -1;
  if (buffer.flags & V4L2_BUF_FLAG_TIMECODE)
  {
    for (AMLCaptureInput tap : INPUTS)
    {
      if ((m_inputs & Bit(tap)) &&
          m_ports[static_cast<size_t>(tap)] == static_cast<int>(buffer.timecode.userbits[0]))
        input = static_cast<int>(tap);
    }
  }
  return true;
}

CAMLCapture::CStream::Wake CAMLCapture::CStream::Poll(int timeoutMs) const
{
  pollfd fds[] = {{m_fd, POLLIN, 0}, {m_eventFd, POLLIN, 0}};
  int ret = -1;
  do
    ret = poll(fds, 2, timeoutMs);
  while (ret < 0 && errno == EINTR);

  if (m_stopped || (fds[1].revents & POLLIN))
    return Wake::STOPPED;
  if (fds[0].revents & POLLERR)
    return Wake::LOST;
  return ret > 0 && (fds[0].revents & POLLIN) ? Wake::FRAME : Wake::TIMEOUT;
}

bool CAMLCapture::CStream::Take(AMLCaptureInput input, unsigned int& index)
{
  std::lock_guard lock(m_bufferLock);

  unsigned int next = 0;
  int tag = -1;
  std::chrono::steady_clock::time_point stamp;
  // a DQBUF on an empty queue logs in the driver, so every one waits for poll
  while (Poll(0) == Wake::FRAME && Dequeue(next, tag, stamp))
  {
    if (tag < 0)
    {
      Queue(next);
      continue;
    }
    int& parked = m_parked[static_cast<size_t>(tag)];
    if (parked >= 0 && m_borrows[parked] == 0)
      Queue(parked);
    parked = static_cast<int>(next);
    m_stamps[next] = stamp;
  }
  int& parked = m_parked[static_cast<size_t>(input)];
  if (parked < 0)
    return false;

  // vdin hands a frame over a vsync after it finishes it, and each input takes its turn once
  // per round of the schedule, so a newer frame of the input is done after that
  const std::chrono::duration<float, std::milli> maxAge((m_entries + 1) * m_framePeriodMs);
  if (m_stopped || std::chrono::steady_clock::now() - m_stamps[parked] >= maxAge)
  {
    if (m_borrows[parked] == 0)
      Queue(parked);
    parked = -1;
    return false;
  }
  index = static_cast<unsigned int>(parked);
  m_borrows[index]++;
  return true;
}

void CAMLCapture::CStream::Return(unsigned int index)
{
  std::lock_guard lock(m_bufferLock);

  if (--m_borrows[index] > 0)
    return;
  for (int& parked : m_parked)
  {
    if (parked == static_cast<int>(index))
      parked = -1;
  }
  Queue(index);
}

CAMLCapture::CStream::Wake CAMLCapture::CStream::Wait(AMLCaptureInput input,
                                                      int timeoutMs,
                                                      unsigned int& index)
{
  const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeoutMs);
  for (;;)
  {
    const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(
                          end - std::chrono::steady_clock::now())
                          .count();
    const Wake wake = Poll(static_cast<int>(std::max<int64_t>(left, 0)));
    if (wake != Wake::FRAME)
      return wake;

    // the frame may be another input's, or another reader may have dequeued and parked it
    if (Take(input, index))
      return Wake::FRAME;
    if (left <= 0)
      return Wake::TIMEOUT;
  }
}

int CAMLCapture::CStream::Export(unsigned int index) const
{
  v4l2_exportbuffer exportBuffer{};
  exportBuffer.type = BUFFER_TYPE;
  exportBuffer.index = index;
  exportBuffer.flags = O_RDONLY | O_CLOEXEC;
  if (ioctl(m_fd, VIDIOC_EXPBUF, &exportBuffer) < 0)
    return -1;
  return exportBuffer.fd;
}

AMLCaptureImage CAMLCapture::CStream::Image(unsigned int index) const
{
  AMLCaptureImage image;
  image.width = m_imageWidth;
  image.height = m_imageHeight;
  image.data[0] = static_cast<uint8_t*>(m_maps[index].first);
  if (m_pixelFormat == PIX_FMT_YUV422_10P)
  {
    image.fullPack = true;
    image.cositedChroma = m_cositedChroma;
    image.linesize[0] = m_stride;
    return image;
  }

  // single-plane NV12: the chroma plane follows the luma plane at the same stride
  image.format = AV_PIX_FMT_NV12;
  image.data[1] = image.data[0] + static_cast<size_t>(m_stride) * m_imageHeight;
  image.linesize[0] = m_stride;
  image.linesize[1] = m_stride;
  return image;
}

std::shared_ptr<CAMLCapture> CAMLCapture::Get()
{
  std::lock_guard lock(s_sessionLock);

  auto session = s_session.lock();
  if (!session)
  {
    session = std::make_shared<CAMLCapture>();
    s_session = session;
  }
  return session;
}

void CAMLCapture::GetCaptureSize(int& width, int& height)
{
  // every engine an input may move to delivers the size
  const std::vector<AMLCaptureDevice>& devices = FindDevices();
  int maxWidth = devices.empty() ? 0 : MAX_CAPTURE_WIDTH;
  for (const AMLCaptureDevice& device : devices)
    maxWidth = std::min(maxWidth, device.maxWidth);
  FitOutput(maxWidth, width, height);
}

void CAMLCapture::GetShotSize(CaptureScreenshot shot, int& width, int& height)
{
  if (shot == CaptureScreenshot::NATIVE)
    FitOutput(std::numeric_limits<int>::max(), width, height);
  else
    GetCaptureSize(width, height);
}

bool AMLCaptureNative::Describe(CaptureResult& result) const
{
  if (!resolved)
    return false;

  result.width = static_cast<unsigned int>(width);
  result.height = static_cast<unsigned int>(height);
  result.stride = width * 6;
  result.format = AV_PIX_FMT_RGB48BE;
  result.color = GetNativeColor(color.signal);
  result.displayMapped = color.displayMapped;
  return true;
}

CAMLCapture::CAMLCapture() : m_engines(FindDevices().size())
{
  if (auto* winSystem = dynamic_cast<CWinSystemAmlogic*>(CServiceBroker::GetWinSystem()))
    winSystem->Register(this);
}

CAMLCapture::~CAMLCapture()
{
  if (auto* winSystem = dynamic_cast<CWinSystemAmlogic*>(CServiceBroker::GetWinSystem()))
    winSystem->Unregister(this);
}

void CAMLCapture::OnLostDisplay()
{
  // a stream's close callback may drop the last session ref, and the session must not unregister
  // inside the display's resource loop, so a job drops this one
  auto session = weak_from_this().lock();
  std::vector<std::shared_ptr<CStream>> streams;
  {
    std::lock_guard lock(m_lock);
    m_displayLost = true;
    m_displayEpoch++;
    // what a node refused may depend on the output mode
    for (size_t engine = 0; engine < m_engines.size(); engine++)
    {
      m_engines[engine].busy = 0;
      m_engines[engine].owned = false;
      m_engines[engine].unscheduled = false;
      streams.push_back(TakeStream(engine));
    }
  }
  for (const auto& stream : streams)
    Close(stream);
  streams.clear();
  if (session)
    CServiceBroker::GetJobManager()->Submit([session = std::move(session)] {});
}

void CAMLCapture::OnResetDisplay()
{
  std::lock_guard lock(m_lock);

  m_displayLost = false;
}

void CAMLCapture::Configure(const AMLCaptureSource& source)
{
  std::lock_guard lock(m_lock);

  m_source = source;
  m_staticsMeta.reset();
  m_presented = false;
  ClearBusy();
  for (Engine& engine : m_engines)
    engine.unscheduled = false;
}

void CAMLCapture::SetPresented()
{
  m_presented = true;
}

bool CAMLCapture::IsPresented() const
{
  // frames queued before the main video plane was hidden are no longer on screen
  return m_presented && !CAMLCodec::IsMainVideoHidden();
}

void CAMLCapture::ClearPresented()
{
  {
    std::lock_guard lock(m_lock);

    m_presented = false;
    m_source = {};
    m_staticsMeta.reset();
    // a video consumer lapses with the video, so its engine stops until the next video is read
    const uint32_t read = GetReadInputs();
    for (Consumer& consumer : m_consumers)
    {
      if (consumer.input == AMLCaptureInput::VIDEO)
      {
        consumer.read = false;
        consumer.engine = -1;
      }
    }
    if (GetReadInputs() != read)
      ClearBusy();
  }
  CloseIdle();
}

void CAMLCapture::SetVideoRect(const CRect& rect)
{
  std::lock_guard lock(m_lock);

  m_videoRect = rect;
}

CRect CAMLCapture::GetVisibleRect()
{
  // the codec places the video at the GUI rect scaled to the output, so the GUI size normalizes it
  const CGraphicContext& context = CServiceBroker::GetWinSystem()->GetGfxContext();
  const RESOLUTION_INFO gui = context.GetResInfo(context.GetVideoResolution());
  if (gui.iWidth <= 0 || gui.iHeight <= 0)
    return {};

  std::lock_guard lock(m_lock);

  CRect rect(m_videoRect.x1 / gui.iWidth, m_videoRect.y1 / gui.iHeight, m_videoRect.x2 / gui.iWidth,
             m_videoRect.y2 / gui.iHeight);
  return rect.Intersect(CRect(0.0f, 0.0f, 1.0f, 1.0f));
}

std::shared_ptr<CAMLCapture::CStream> CAMLCapture::TakeStream(size_t engine)
{
  std::shared_ptr<CStream>& stream = m_engines[engine].stream;
  if (stream)
  {
    m_closing++;
    m_streamChanges++;
    // a reader may still convert from it, so the node closes with its last holder
    stream->SetClosed(
        [weak = weak_from_this()]
        {
          if (const auto session = weak.lock())
          {
            std::lock_guard lock(session->m_lock);
            session->m_closing--;
            session->m_closed.notify_all();
          }
        });
  }
  return std::move(stream);
}

void CAMLCapture::Close(const std::shared_ptr<CStream>& stream)
{
  if (stream)
    stream->Stop();
}

std::shared_ptr<void> CAMLCapture::Lease(AMLCaptureInput input, int width, int height)
{
  std::lock_guard lock(m_lock);

  const auto it = FindConsumer(input, width, height);
  if (it != m_consumers.end())
    it->leases++;
  else
    m_consumers.push_back({input, width, height, 1, std::make_shared<ConsumerConvert>()});
  return std::shared_ptr<void>(nullptr,
                               [weak = weak_from_this(), input, width, height](void*)
                               {
                                 if (const auto session = weak.lock())
                                   session->ReleaseLease(input, width, height);
                               });
}

void CAMLCapture::ReleaseLease(AMLCaptureInput input, int width, int height)
{
  {
    std::lock_guard lock(m_lock);
    const auto it = FindConsumer(input, width, height);
    if (--it->leases == 0)
    {
      const uint32_t read = GetReadInputs();
      m_consumers.erase(it);
      if (GetReadInputs() != read)
        ClearBusy();
    }
  }
  CloseIdle();
}

void CAMLCapture::CloseIdle()
{
  {
    std::lock_guard lock(m_lock);
    const std::vector<Plan> plans = Place();
    bool idle = false;
    for (size_t engine = 0; engine < m_engines.size(); engine++)
      idle = idle || IsIdle(m_engines[engine], plans[engine]);
    if (!idle)
      return;
  }

  // a lease or the video can end on the render thread, which makes no stream ioctls; the job
  // holds the session, so its destructor does not stop the stream there either
  CServiceBroker::GetJobManager()->Submit(
      [session = shared_from_this()]
      {
        std::vector<std::shared_ptr<CStream>> streams;
        {
          std::lock_guard lock(session->m_lock);
          const std::vector<Plan> plans = session->Place();
          for (size_t engine = 0; engine < session->m_engines.size(); engine++)
          {
            if (session->IsIdle(session->m_engines[engine], plans[engine]))
              streams.push_back(session->TakeStream(engine));
          }
        }
        for (const auto& stream : streams)
          session->Close(stream);
      });
}

std::vector<CAMLCapture::Consumer>::iterator CAMLCapture::FindConsumer(AMLCaptureInput input,
                                                                       int width,
                                                                       int height)
{
  return std::find_if(
      m_consumers.begin(), m_consumers.end(), [&](const Consumer& consumer)
      { return consumer.input == input && consumer.width == width && consumer.height == height; });
}

bool CAMLCapture::IsIdle(const Engine& engine, const Plan& plan) const
{
  // a shot closes its own stream
  return engine.stream && engine.stream->GetPixelFormat() != PIX_FMT_YUV422_10P && plan.inputs == 0;
}

uint32_t CAMLCapture::GetReadInputs() const
{
  uint32_t inputs = 0;
  for (const Consumer& consumer : m_consumers)
  {
    if (consumer.read)
      inputs |= Bit(consumer.input);
  }
  return inputs;
}

void CAMLCapture::ClearBusy()
{
  for (Engine& engine : m_engines)
  {
    engine.busy = 0;
    engine.owned = false;
  }
}

std::vector<CAMLCapture::Plan> CAMLCapture::Place()
{
  const std::vector<AMLCaptureDevice>& devices = FindDevices();
  std::vector<Plan> plans(devices.size());
  const auto add = [&plans](Consumer& consumer, size_t engine)
  {
    Plan& plan = plans[engine];
    plan.inputs |= Bit(consumer.input);
    plan.width = std::max(plan.width, consumer.width);
    plan.height = std::max(plan.height, consumer.height);
    plan.consumers++;
    consumer.engine = static_cast<int>(engine);
  };
  for (Consumer& consumer : m_consumers)
  {
    if (consumer.read && consumer.engine >= 0)
      add(consumer, static_cast<size_t>(consumer.engine));
  }

  for (Consumer& consumer : m_consumers)
  {
    if (!consumer.read || consumer.engine >= 0)
      continue;

    const uint32_t input = Bit(consumer.input);
    const auto carries = [&](size_t engine) { return (plans[engine].inputs & input) != 0; };
    const auto fits = [&](size_t engine)
    { return plans[engine].width >= consumer.width && plans[engine].height >= consumer.height; };
    const auto ready = [&](size_t engine)
    { return devices[engine].Offers(consumer.input) && !(m_engines[engine].busy & input); };
    const auto first = [&](const auto& rule)
    {
      for (size_t engine = 0; engine < plans.size(); engine++)
      {
        if (rule(engine))
          return engine;
      }
      return plans.size();
    };
    // a busy engine is retried on each read once no other engine is free for the input
    const bool anyReady = first(ready) < plans.size();
    const auto usable = [&](size_t engine)
    { return anyReady ? ready(engine) : devices[engine].Offers(consumer.input); };

    size_t engine = first([&](size_t e) { return carries(e) && fits(e); });
    if (engine == plans.size())
      engine = first([&](size_t e) { return plans[e].inputs == 0 && usable(e); });
    if (engine == plans.size())
      engine = first(carries);
    if (engine == plans.size())
    {
      // turns on the stream with the fewest inputs; a node without turns only when no other can
      // carry the input, and its readers then restart it with their own
      const auto rank = [&](size_t e)
      { return std::make_pair(m_engines[e].unscheduled, std::popcount(plans[e].inputs)); };
      for (size_t e = 0; e < plans.size(); e++)
      {
        if (usable(e) && plans[e].inputs != 0 && (engine == plans.size() || rank(e) < rank(engine)))
          engine = e;
      }
    }
    if (engine < plans.size())
      add(consumer, engine);
  }
  return plans;
}

bool CAMLCapture::CanRead(AMLCaptureInput input) const
{
  if (m_displayLost)
    return false;
  return input == AMLCaptureInput::SCREEN || IsPresented();
}

bool CAMLCapture::Unreadable(bool log) const
{
  if (log)
    CLog::Log(LOGWARNING, "CAMLCapture::{}: no video frame: {}", __FUNCTION__,
              m_displayLost ? "display lost" : "no video presented");
  return false;
}

bool CAMLCapture::Read(AMLCaptureInput input, int width, int height, const ReadFunc& use)
{
  const auto lease = Lease(input, width, height);
  std::shared_ptr<ConsumerConvert> convert;
  {
    std::lock_guard lock(m_lock);
    if (!CanRead(input))
      return false;
    const auto consumer = FindConsumer(input, width, height);
    const uint32_t read = GetReadInputs();
    consumer->read = true;
    if (GetReadInputs() != read)
      ClearBusy();
    convert = consumer->convert;
  }
  const StreamFunc read = [&convert, &use](CStream&, unsigned int, const AMLCaptureFrame& frame)
  {
    std::lock_guard lock(convert->lock);
    return use(frame, convert->convert);
  };

  // a refused open moves the consumer to the next engine placement gives it, until none is left
  for (;;)
  {
    int engine = -1;
    {
      std::lock_guard lock(m_lock);
      Place();
      engine = FindConsumer(input, width, height)->engine;
    }
    if (engine < 0)
      return false;

    std::unique_lock open(m_engines[engine].open);
    bool refused = false;
    const bool ok = ReadStream(open, static_cast<size_t>(engine), input, V4L2_PIX_FMT_NV12, width,
                               height, read, refused);
    if (!refused)
      return ok;
  }
}

bool CAMLCapture::ReadVideo(CaptureScreenshot shot, int width, int height, const ReadFunc& use)
{
  if (shot != CaptureScreenshot::NONE)
  {
    AMLCaptureShotFrame frame;
    CAMLCaptureConvert convert;
    if (ReadShot(shot, width, height, frame) && use(frame.frame, convert))
      return true;
  }
  return Read(AMLCaptureInput::VIDEO, width, height, use);
}

bool CAMLCapture::ReadShot(CaptureScreenshot shot,
                           int width,
                           int height,
                           AMLCaptureShotFrame& frame)
{
  const std::vector<AMLCaptureDevice>& devices = FindDevices();
  std::vector<size_t> engines;
  {
    std::lock_guard lock(m_lock);
    // an idle engine first, so no consumer stops; else the one whose stream carries the video,
    // then the one with the fewest consumers
    const std::vector<Plan> plans = Place();
    const auto rank = [&plans](size_t e)
    {
      return std::make_tuple(plans[e].inputs != 0,
                             (plans[e].inputs & Bit(AMLCaptureInput::VIDEO)) == 0,
                             plans[e].consumers);
    };
    for (size_t e = 0; e < devices.size(); e++)
    {
      if (devices[e].shot && devices[e].Offers(AMLCaptureInput::VIDEO))
        engines.push_back(e);
    }
    std::stable_sort(engines.begin(), engines.end(),
                     [&rank](size_t a, size_t b) { return rank(a) < rank(b); });
  }
  if (engines.empty())
  {
    CLog::Log(LOGWARNING, "CAMLCapture::{}: no node captures 10-bit video", __FUNCTION__);
    return false;
  }

  // a full frame copy takes longer than a continuous reader can wait, so it runs from a dmabuf of
  // the frame after the stream closes
  int dmabuf = -1;
  void* map = MAP_FAILED;
  size_t length = 0;
  size_t size = 0;
  const auto copy = [&](CStream& stream, unsigned int index, const AMLCaptureFrame& source)
  {
    size = static_cast<size_t>(source.image.linesize[0]) * source.image.height;
    frame.data = std::make_unique_for_overwrite<uint8_t[]>(size);
    frame.frame = source;
    dmabuf = stream.Export(index);
    if (dmabuf >= 0)
    {
      length = stream.GetLength(index);
      map = mmap(nullptr, length, PROT_READ, MAP_SHARED, dmabuf, 0);
    }
    if (map == MAP_FAILED)
    {
      CLog::Log(LOGWARNING, "CAMLCapture::{}: no dmabuf of the frame: {}", __FUNCTION__,
                strerror(errno));
      if (dmabuf >= 0)
        close(dmabuf);
      return false;
    }
    // vdin must not refill the exported buffer once it is queued again
    stream.Stop();
    return true;
  };

  bool ok = false;
  for (const size_t engine : engines)
  {
    std::unique_lock open(m_engines[engine].open);
    bool refused = false;
    ok = ReadStream(open, engine, AMLCaptureInput::VIDEO, PIX_FMT_YUV422_10P, width, height, copy,
                    refused, shot == CaptureScreenshot::NATIVE);

    // close the shot stream whatever leases remain
    std::shared_ptr<CStream> stream;
    {
      std::lock_guard lock(m_lock);
      const std::shared_ptr<CStream>& current = m_engines[engine].stream;
      if (current && current->GetPixelFormat() == PIX_FMT_YUV422_10P)
        stream = TakeStream(engine);
    }
    Close(stream);
    stream.reset();
    open.unlock();
    if (!refused)
      break;
  }

  if (map != MAP_FAILED)
  {
    std::memcpy(frame.data.get(), map, size);
    frame.frame.image.data[0] = frame.data.get();
    munmap(map, length);
    close(dmabuf);
  }

  if (!ok)
  {
    CLog::Log(LOGWARNING, "CAMLCapture::{}: no 10-bit frame at {}x{}", __FUNCTION__, width, height);
    return false;
  }

  frame.height = height * frame.frame.image.width / width;
  return true;
}

bool CAMLCapture::ReadStream(std::unique_lock<std::mutex>& open,
                             size_t engine,
                             AMLCaptureInput input,
                             uint32_t pixelFormat,
                             int width,
                             int height,
                             const StreamFunc& use,
                             bool& refused,
                             bool logCause)
{
  Engine& node = m_engines[engine];
  const AMLCaptureDevice& device = FindDevices()[engine];
  const bool shot = pixelFormat == PIX_FMT_YUV422_10P;
  uint32_t inputs = Bit(input);
  uint32_t replaced = 0;
  std::shared_ptr<CStream> stream;
  std::vector<std::shared_ptr<CStream>> stale;
  AMLCaptureSource source;
  // the store lock is never taken under m_lock
  AMLFrameMetadataPtr meta = CAMLFrameMetadataStore::GetInstance().Get();
  {
    std::lock_guard lock(m_lock);
    if (!CanRead(input))
      return Unreadable(logCause);
    const std::vector<Plan> plans = Place();
    // on a node without turns, or with an input it refused, each reader runs its own input, at
    // the size its consumers read
    const auto wanted = [&](size_t e, AMLCaptureInput tap)
    {
      const Engine& node = m_engines[e];
      if (!(node.unscheduled || (node.busy & plans[e].inputs)) ||
          std::popcount(plans[e].inputs) < 2)
        return plans[e];
      Plan plan{Bit(tap)};
      for (const Consumer& consumer : m_consumers)
      {
        if (consumer.engine == static_cast<int>(e) && consumer.input == tap)
        {
          plan.width = std::max(plan.width, consumer.width);
          plan.height = std::max(plan.height, consumer.height);
        }
      }
      return plan;
    };
    if (!shot)
    {
      // a consumer came or went since the reader chose the engine
      if (FindConsumer(input, width, height)->engine != static_cast<int>(engine))
      {
        refused = true;
        return false;
      }
      const Plan plan = wanted(engine, input);
      inputs = plan.inputs;
      width = plan.width;
      height = plan.height;
      // an input the node refused moves to another engine rather than closing another stream
      const std::vector<AMLCaptureDevice>& devices = FindDevices();
      bool elsewhere = false;
      for (size_t e = 0; e < devices.size(); e++)
        elsewhere = elsewhere ||
                    (e != engine && devices[e].Offers(input) && !(m_engines[e].busy & Bit(input)));
      if (elsewhere && (node.busy & inputs) && node.stream &&
          (node.stream->GetInputs() & plans[engine].inputs & ~inputs))
      {
        for (Consumer& consumer : m_consumers)
        {
          if (consumer.engine == static_cast<int>(engine) && consumer.input == input)
            consumer.engine = -1;
        }
        refused = true;
        return false;
      }
    }
    if (node.stream && !node.stream->Matches(inputs, pixelFormat, width, height))
    {
      replaced = node.stream->GetInputs();
      stale.push_back(TakeStream(engine));
    }
    stream = node.stream;
    // our streams that no longer match still hold write-back channels the open may need
    for (size_t other = 0; !stream && other < m_engines.size(); other++)
    {
      const std::shared_ptr<CStream>& otherStream = m_engines[other].stream;
      if (!otherStream || otherStream->GetPixelFormat() != V4L2_PIX_FMT_NV12)
        continue;
      const Plan plan = wanted(other, otherStream->GetInputs() & Bit(AMLCaptureInput::SCREEN)
                                          ? AMLCaptureInput::SCREEN
                                          : AMLCaptureInput::VIDEO);
      if (!otherStream->Matches(plan.inputs, V4L2_PIX_FMT_NV12, plan.width, plan.height))
        stale.push_back(TakeStream(other));
    }
    UpdateStatics(std::move(meta));
    source = m_source;
  }
  for (const auto& old : stale)
    Close(old);
  stale.clear();

  if (!stream)
  {
    uint32_t epoch = 0;
    uint32_t changes = 0;
    {
      std::unique_lock lock(m_lock);
      m_closed.wait(lock, [this] { return m_closing == 0; });
      if (!CanRead(input))
        return Unreadable(logCause);
      epoch = m_displayEpoch;
      changes = m_streamChanges;
    }

    CStream::Refusal refusal = CStream::Refusal::NONE;
    const int logLevel = node.openFailed ? LOGDEBUG : LOGWARNING;
    const unsigned int buffers =
        shot
            ? SHOT_BUFFERS
            : BUFFERS_PER_INPUT * static_cast<unsigned int>(std::popcount(inputs)) + STREAM_BUFFERS;
    stream = CStream::Open(device, inputs, pixelFormat, buffers, width, height, logLevel, refusal);
    const bool busy = refusal == CStream::Refusal::OWNED || refusal == CStream::Refusal::NO_CHANNEL;
    if (!busy)
      node.openFailed = !stream;

    std::lock_guard lock(m_lock);
    if (shot)
      refused = !stream;
    else
    {
      // a mode set, a change of our own streams and a shot meanwhile refuse an open too
      const bool ours =
          epoch != m_displayEpoch || changes != m_streamChanges ||
          std::any_of(m_engines.begin(), m_engines.end(),
                      [](const Engine& other) {
                        return other.stream && other.stream->GetPixelFormat() == PIX_FMT_YUV422_10P;
                      });
      // an open ends the refusals of its inputs, and of all inputs when another user owned the node
      if (stream)
      {
        node.busy = node.owned ? 0 : node.busy & ~inputs;
        node.owned = false;
      }
      const uint32_t wasBusy = node.busy;
      const bool wasScheduled = !node.unscheduled;
      // vdin names no input it lacks a channel for, so of several the open added the reader's
      // is kept and retried alone
      uint32_t added = inputs & ~replaced;
      if (added == 0)
        added = inputs;
      if (!ours && refusal == CStream::Refusal::OWNED)
      {
        node.busy = Bit(AMLCaptureInput::SCREEN) | Bit(AMLCaptureInput::VIDEO);
        node.owned = true;
      }
      else if (!ours && refusal == CStream::Refusal::NO_CHANNEL)
        node.busy |= std::popcount(added) == 1 ? added : added & ~Bit(input);
      node.unscheduled =
          node.unscheduled || (epoch == m_displayEpoch && refusal == CStream::Refusal::NO_TURNS);
      refused = node.busy != wasBusy || (wasScheduled && node.unscheduled);
    }
    if (refused)
    {
      const bool unscheduled = refusal == CStream::Refusal::NO_TURNS;
      CLog::Log(LOGWARNING, "CAMLCapture::{}: {} refused inputs {:#x}: {}", __FUNCTION__,
                device.path, inputs, unscheduled ? "no turns" : "no room");
      // the consumers it was refused for are placed again; without turns the reader's stays; a
      // shot moves itself
      const uint32_t moved = shot ? 0 : unscheduled ? ~Bit(input) : node.busy;
      for (Consumer& consumer : m_consumers)
      {
        if (consumer.engine == static_cast<int>(engine) && (moved & Bit(consumer.input)))
          consumer.engine = -1;
      }
    }
    if (!stream)
      return false;
    // a stream opened across a mode set still holds the old mode
    if (epoch != m_displayEpoch)
    {
      if (logCause)
        CLog::Log(LOGWARNING, "CAMLCapture::{}: the output mode changed", __FUNCTION__);
      return false;
    }
    if (!CanRead(input))
      return Unreadable(logCause);
    node.stream = stream;
    m_streamChanges++;
  }
  if (!shot)
    open.unlock();

  const int waitMs = static_cast<int>(std::ceil(FRAME_WAIT_PERIODS * stream->GetFramePeriodMs()));
  unsigned int index = 0;
  const CStream::Wake wake =
      stream->Take(input, index) ? CStream::Wake::FRAME : stream->Wait(input, waitMs, index);
  if (wake == CStream::Wake::FRAME)
  {
    AMLCaptureFrame frame;
    frame.color = GetSignal(source);
    frame.image = stream->Image(index);
    const bool ok = use(*stream, index, frame);
    stream->Return(index);
    return ok;
  }
  // vdin stopped the stream without the reader, so the next read opens a new one
  if (wake == CStream::Wake::LOST)
  {
    {
      std::lock_guard lock(m_lock);
      if (node.stream == stream)
        stale.push_back(TakeStream(engine));
    }
    for (const auto& old : stale)
      Close(old);
    CLog::Log(LOGWARNING, "CAMLCapture::{}: vdin stopped the stream on {}", __FUNCTION__,
              device.path);
    return false;
  }
  // vdin gives no frames while the output is muted, so a stream stays open through silence
  if (wake == CStream::Wake::TIMEOUT)
  {
    std::lock_guard lock(m_lock);
    if (node.stream == stream)
    {
      if (logCause)
        CLog::Log(LOGWARNING, "CAMLCapture::{}: no frame in {} ms, stream kept open", __FUNCTION__,
                  waitMs);
      return false;
    }
  }
  if (logCause)
    CLog::Log(LOGWARNING, "CAMLCapture::{}: stream stopped", __FUNCTION__);
  return false;
}

void CAMLCapture::UpdateStatics(AMLFrameMetadataPtr meta)
{
  if (meta == m_staticsMeta)
    return;

  // the metadata object changes with the dynamic payloads, the statics rarely
  if (!meta || !m_staticsMeta || meta->hdrMdcv != m_staticsMeta->hdrMdcv ||
      meta->hdrCll != m_staticsMeta->hdrCll)
    m_source.titlePeak = meta ? AMLGetTitlePeak(*meta) : 0.0f;
  m_staticsMeta = std::move(meta);
}

AMLCaptureColor CAMLCapture::GetSignal(const AMLCaptureSource& source) const
{
  AMLCaptureColor color;

  auto* winSystem = dynamic_cast<CWinSystemAmlogic*>(CServiceBroker::GetWinSystem());
  CAMLDisplay* display = winSystem ? winSystem->GetAmlDisplay() : nullptr;
  if (!display || !display->aml_get_display_connected())
    return color;

  bool sourcePQ = source.transfer == AVCOL_TRC_SMPTE2084;
  bool sourceHLG = source.transfer == AVCOL_TRC_ARIB_STD_B67;
  // hdr2sdr and sdr2hdr give the picture the output colour; the stream keeps the source type
  const auto settings = CServiceBroker::GetSettingsComponent()->GetSettings();
  if (!source.dolbyVision && (settings->GetBool(CSettings::SETTING_COREELEC_AMLOGIC_HDR2SDR) ||
                              settings->GetBool(CSettings::SETTING_COREELEC_AMLOGIC_SDR2HDR)))
  {
    const auto& components = CServiceBroker::GetAppComponents();
    const auto appPlayer = components.GetComponent<CApplicationPlayer>();
    VideoStreamInfo stream;
    appPlayer->GetVideoStreamInfo(CURRENT_STREAM, stream);
    if (stream.valid && (aml_convert_to_sdr(stream.hdrType) || aml_convert_to_hdr(stream.hdrType)))
    {
      sourceHLG = stream.hdrType == StreamHdrType::HDR_TYPE_HLG;
      sourcePQ = stream.hdrType != StreamHdrType::HDR_TYPE_NONE && !sourceHLG;
    }
  }
  bool mapped = !sourcePQ;

  // hdr_conversion_ctrl does not follow the DV core, so it is read only without DV; under DV,
  // vdin sees the core3 input in every mode
  if (display->aml_get_drmProperty("dv_enable", DRM_MODE_OBJECT_CRTC) == 1 &&
      display->aml_get_drmProperty("dv_video_on", DRM_MODE_OBJECT_CRTC) == 1)
  {
    switch (static_cast<DvMode>(display->aml_get_drmProperty("dv_mode", DRM_MODE_OBJECT_CRTC)))
    {
      case DvMode::IPT:
      case DvMode::IPT_TUNNEL:
        if (display->aml_get_drmProperty("dv_ll_policy", DRM_MODE_OBJECT_CRTC) >= DV_LL_YUV422)
        {
          color.signal = AMLCaptureSignal::DV_LL;
          color.displayMapped = true;
          mapped = true;
        }
        else
          color.signal = AMLCaptureSignal::DV_TV_LED;
        break;
      case DvMode::HDR10:
        color.signal = AMLCaptureSignal::DV_HDR10;
        color.displayMapped = true;
        mapped = true;
        break;
      case DvMode::SDR10:
      case DvMode::SDR8:
        color.signal = AMLCaptureSignal::DV_SDR;
        color.displayMapped = true;
        break;
      default:
        if (sourcePQ)
          color.signal = AMLCaptureSignal::PQ;
        else if (sourceHLG)
          color.signal = AMLCaptureSignal::HLG;
        break;
    }
  }
  else
  {
    switch (static_cast<HdrOutput>(
        display->aml_get_drmProperty("hdr_conversion_ctrl", DRM_MODE_OBJECT_CRTC)))
    {
      case HdrOutput::HDR10:
      case HdrOutput::HDR10PLUS:
      case HdrOutput::CUVA_HDR:
        color.signal = AMLCaptureSignal::PQ;
        color.displayMapped = !sourcePQ;
        break;
      case HdrOutput::HLG:
      case HdrOutput::CUVA_HLG:
        color.signal = AMLCaptureSignal::HLG;
        color.displayMapped = sourcePQ;
        break;
      default:
        color.displayMapped = sourcePQ || sourceHLG;
        break;
    }
  }

  switch (color.signal)
  {
    case AMLCaptureSignal::PQ:
    case AMLCaptureSignal::DV_LL:
    case AMLCaptureSignal::DV_HDR10:
      color.luminance = mapped ? MAPPED_PEAK_NITS : GetPeak(source, PQ_PEAK_NITS);
      break;
    case AMLCaptureSignal::HLG:
      color.luminance = HLG_PEAK_NITS;
      break;
    case AMLCaptureSignal::DV_TV_LED:
      color.luminance = GetPeak(source, source.dolbyVision ? TVLED_PEAK_NITS : MAPPED_PEAK_NITS);
      break;
    default:
      break;
  }
  return color;
}

AMLCaptureImage CAMLCapture::Crop(const AMLCaptureImage& image, const CRect& rect)
{
  if (rect.IsEmpty())
    return image;

  const auto edge = [](float v, int size)
  { return std::clamp(static_cast<int>(std::lround(v * size / 2)) * 2, 0, size); };
  // the end rounds down to even, so a whole end stays put and a fractional one does not round up
  const auto end = [](float v, int size)
  { return std::clamp(static_cast<int>(std::floor(v * size + CROP_END_EPSILON)) & ~1, 0, size); };
  const int x1 = edge(rect.x1, image.width);
  const int x2 = end(rect.x2, image.width);
  const int y1 = edge(rect.y1, image.height);
  const int y2 = end(rect.y2, image.height);

  AMLCaptureImage crop = image;
  crop.width = x2 - x1;
  crop.height = y2 - y1;
  if (image.fullPack)
  {
    crop.data[0] += y1 * image.linesize[0] + x1 / 2 * AML_FULL_PACK_PAIR_BYTES;
    return crop;
  }
  crop.data[0] += y1 * image.linesize[0] + x1;
  crop.data[1] += y1 / 2 * image.linesize[1] + x1;
  return crop;
}

void CAMLCapture::CreateVideoPixels(const AMLCaptureVideoSpec& spec, CaptureResult& result)
{
  int width = 0;
  int height = 0;
  GetStreamSize(spec.rect, spec.width, spec.height, width, height);
  auto lease = spec.continuous ? Lease(AMLCaptureInput::VIDEO, width, height) : nullptr;
  result.pixels =
      std::make_shared<CAMLVideoPixels>(weak_from_this(), std::move(lease), spec, width, height);
  DescribeSdr(spec.width, spec.height, spec.shot == CaptureScreenshot::STANDARD, result);
  result.content = CaptureContent::VIDEO;
}

void CAMLCapture::CreateCompositePixels(CaptureResult& result)
{
  using KODI::WINDOWING::AML::CWinSystemAmlogicGLESContext;

  auto* winSystem = dynamic_cast<CWinSystemAmlogicGLESContext*>(CServiceBroker::GetWinSystem());
  if (!winSystem)
    return;

  AMLCaptureCoding coding;
  const int transfer = winSystem->GetGuiCompositeTransfer();
  if (transfer == AVCOL_TRC_SMPTE2084)
    coding.transfer = AMLCaptureCoding::Transfer::PQ;
  else if (transfer == AVCOL_TRC_ARIB_STD_B67)
    coding.transfer = AMLCaptureCoding::Transfer::HLG;
  // an SDR GUI over an HDR signal sits at the GUI's SDR white; the GUI composite peak is
  // PQ-normalized luminance
  const float white =
      transfer ? winSystem->GetGuiCompositePeak()
               : CGuiCompositeShaderGLES::PeakFromPQCode(winSystem->GetGuiSdrPeakLuminance());
  coding.whiteNits = white * PQ_PEAK_NITS;

  const bool rgb = result.screenshot == CaptureScreenshot::STANDARD;
  result.pixels =
      std::make_shared<CAMLCompositePixels>(weak_from_this(), result, coding, GetVisibleRect());
  result.stride = static_cast<int>(result.width) * (rgb ? 3 : 4);
  result.format = rgb ? AV_PIX_FMT_RGB24 : AV_PIX_FMT_RGBA;
  result.color.primaries = AVCOL_PRI_BT709;
  result.color.transfer = AVCOL_TRC_IEC61966_2_1;
  result.color.range = AVCOL_RANGE_JPEG;
}

void CAMLCapture::CreateLivePixels(int width, int height, CaptureResult& result)
{
  const std::vector<AMLCaptureDevice>& devices = FindDevices();
  if (std::none_of(devices.begin(), devices.end(), [](const AMLCaptureDevice& device)
                   { return device.Offers(AMLCaptureInput::SCREEN); }))
    return;

  const auto session = Get();
  int streamWidth = 0;
  int streamHeight = 0;
  GetStreamSize(CRect(0.0f, 0.0f, 1.0f, 1.0f), width, height, streamWidth, streamHeight);
  auto lease = session->Lease(AMLCaptureInput::SCREEN, streamWidth, streamHeight);
  result.pixels = std::make_shared<CAMLLivePixels>(session, std::move(lease), width, height,
                                                   streamWidth, streamHeight);
  DescribeSdr(width, height, false, result);
  result.content = CaptureContent::COMPOSITE;
  result.live = true;
}
