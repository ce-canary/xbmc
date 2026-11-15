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
#include <linux/dma-heap.h>
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
// the codec memory pool, which vdin's own buffers come from
constexpr const char* CAPTURE_HEAP = "/dev/dma_heap/heap-codecmm";
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

struct CaptureBuffer
{
  //! a dmabuf, -1 for a buffer of the node's own
  int fd{-1};
  void* map{MAP_FAILED};
  size_t length{0};
};

void Free(CaptureBuffer& buffer)
{
  if (buffer.map != MAP_FAILED)
    munmap(buffer.map, buffer.length);
  if (buffer.fd >= 0)
    close(buffer.fd);
  buffer = {};
}

// `count` dmabufs of `length` bytes, mapped for reading; none when one fails
std::vector<CaptureBuffer> Allocate(size_t count, size_t length, int logLevel)
{
  std::vector<CaptureBuffer> buffers;
  const int heap = open(CAPTURE_HEAP, O_RDWR | O_CLOEXEC);
  while (heap >= 0 && buffers.size() < count)
  {
    dma_heap_allocation_data data{};
    data.len = length;
    data.fd_flags = O_RDWR | O_CLOEXEC;
    if (ioctl(heap, DMA_HEAP_IOCTL_ALLOC, &data) < 0)
      break;
    CaptureBuffer& buffer = buffers.emplace_back();
    buffer.fd = static_cast<int>(data.fd);
    buffer.length = length;
    buffer.map = mmap(nullptr, length, PROT_READ, MAP_SHARED, buffer.fd, 0);
    if (buffer.map == MAP_FAILED)
      break;
  }
  const int error = errno;
  if (heap >= 0)
    close(heap);
  if (!buffers.empty() && buffers.size() == count && buffers.back().map != MAP_FAILED)
    return buffers;

  CLog::Log(logLevel, "CAMLCapture::{}: no {} buffers of {} bytes: {}", __FUNCTION__, count, length,
            strerror(error));
  for (CaptureBuffer& buffer : buffers)
    Free(buffer);
  return {};
}

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
    int streamWidth = 0;
    int streamHeight = 0;
    {
      std::lock_guard lock(m_streamLock);
      streamWidth = m_streamWidth;
      streamHeight = m_streamHeight;
    }

    const AMLCaptureRead read =
        m_session->Read(AMLCaptureInput::SCREEN, streamWidth, streamHeight,
                        [&dst](const AMLCaptureFrame& frame, CAMLCaptureConvert& convert) {
                          return convert.Convert(frame.image, frame.color, AMLCaptureCoding(), dst);
                        });
    m_read = m_read || read == AMLCaptureRead::FRAME;
    // while the screen shows no picture, the last one stands
    if (read == AMLCaptureRead::ENDED || !m_read)
      return nullptr;
    return m_buffer.get();
  }

  void Unlock() override { m_lock.unlock(); }

  //! Render thread: a new output mode may change the stream size, which takes a lease of its own;
  //! the image stays
  bool HasStreamSize(int streamWidth, int streamHeight)
  {
    std::lock_guard lock(m_streamLock);

    return streamWidth == m_streamWidth && streamHeight == m_streamHeight;
  }
  void SetStreamSize(std::shared_ptr<void> lease, int streamWidth, int streamHeight)
  {
    std::lock_guard lock(m_streamLock);

    m_lease = std::move(lease);
    m_streamWidth = streamWidth;
    m_streamHeight = streamHeight;
  }

private:
  // the lease ends first, so the stream closes before the session can go
  std::shared_ptr<CAMLCapture> m_session;
  std::shared_ptr<void> m_lease;
  int m_width{0};
  int m_height{0};
  int m_streamWidth{0};
  int m_streamHeight{0};
  //! guards the lease and the stream size; a Lock never holds it while it reads
  std::mutex m_streamLock;
  std::mutex m_lock;
  bool m_read{false};
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
    EVENT,
    NO_SIGNAL,
  };

  enum class Growth
  {
    GROWN,
    GROWING,
    //! no larger buffers, a window of frame periods without one taken in, or the stream stopped
    FAILED,
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
                                       bool turns,
                                       int logLevel,
                                       Refusal& refusal);
  ~CStream();

  void Stop();
  uint32_t GetInputs() const { return m_inputs; }
  uint32_t GetPixelFormat() const { return m_pixelFormat; }
  float GetFramePeriodMs() const { return m_framePeriodMs; }
  bool IsScheduled() const { return m_scheduled; }
  //! under the session lock
  bool Matches(uint32_t inputs, uint32_t pixelFormat, int width, int height) const
  {
    return inputs == m_inputs && pixelFormat == m_pixelFormat && width == m_readWidth &&
           height == m_readHeight;
  }
  //! under the session lock
  bool Serves(AMLCaptureInput input, int width, int height) const
  {
    return (m_inputs & Bit(input)) && width <= m_readWidth && height <= m_readHeight;
  }
  //! under the session lock
  void SetReadSize(int width, int height)
  {
    m_readWidth = width;
    m_readHeight = height;
  }
  bool Fits(int width, int height);
  //! Without the engine's open lock: gives each buffer that does not fit width x height a larger
  //! one when it next goes back to vdin; with `wait` waits until all of them did
  Growth Grow(int width, int height, bool wait, int logLevel);
  //! Under the engine's open lock: vdin captures `inputs` in turn at width x height from its next
  //! round on, which the buffers must fit. False when the stream stopped or vdin kept the old
  //! turns; `refusal` tells why. `settled` runs once vdin dropped the inputs that left, or at once.
  bool SetInputs(uint32_t inputs,
                 int width,
                 int height,
                 int logLevel,
                 std::function<void()> settled,
                 Refusal& refusal);
  //! the stream calls `closed` once it is gone, after its node is closed
  void SetClosed(std::function<void()> closed) { m_closed = std::move(closed); }
  //! Takes the newest frame of `input` that holds width x height, or the running size when smaller;
  //! false without one or when a newer one must have come. It stays the readers' until Return.
  bool Take(AMLCaptureInput input, int width, int height, unsigned int& index);
  //! Any thread: the last reader of a frame queues it again
  void Return(unsigned int index);
  Wake Wait(AMLCaptureInput input, int width, int height, int timeoutMs, unsigned int& index);
  //! width x height as vdin bounds it, 0 x 0 for a size it refuses
  void Bound(int& width, int& height);
  AMLCaptureImage Image(unsigned int index) const;
  size_t GetLength(unsigned int index) const { return m_buffers[index].length; }
  //! a read-only dmabuf of buffer `index`, which keeps the buffer after the stream closes; -1 on
  //! failure
  int Export(unsigned int index) const;

private:
  struct Layout
  {
    int width{0};
    int height{0};
    int stride{0};
    //! the bytes a buffer needs, 0 for a size vdin refuses
    size_t size{0};
  };

  struct Request
  {
    int width{0};
    int height{0};
    Layout layout;
  };

  //! the turns of `inputs` at width x height to vdin; 0, else the errno
  int Schedule(uint32_t inputs, int width, int height);
  void Subscribe(uint32_t inputs);
  //! under m_bufferLock once the stream runs; a failure loses a running stream
  bool Queue(unsigned int index);
  //! `input` is the AMLCaptureInput of the frame, -1 for one the stream lacks; `entries` and
  //! `generation` the length and the wrapping count of the turns vdin filled it under
  bool Dequeue(unsigned int& index,
               int& input,
               int& entries,
               int& generation,
               Layout& layout,
               std::chrono::steady_clock::time_point& stamp);
  //! under m_bufferLock: parks the frames vdin finished and queues the stale ones again
  void Drain();
  //! under m_bufferLock
  Layout GetLayout(int width, int height);
  Wake Poll(int leaveFd, int timeoutMs) const;

  int m_fd{-1};
  int m_eventFd{-1};
  //! per AMLCaptureInput, wakes its readers once it left the turns
  std::array<int, 2> m_leaveFds{-1, -1};
  //! changes under m_bufferLock
  std::atomic<uint32_t> m_inputs{0};
  std::array<int, 2> m_ports{-1, -1};
  //! guarded by m_bufferLock once the stream runs
  int m_entries{0};
  bool m_scheduled{false};
  uint32_t m_pixelFormat{V4L2_PIX_FMT_NV12};
  //! guarded by the session lock: the size its readers asked for
  int m_readWidth{0};
  int m_readHeight{0};
  Request m_format;
  bool m_cositedChroma{false};
  float m_framePeriodMs{0.0f};
  uint32_t m_memory{V4L2_MEMORY_MMAP};
  bool m_streaming{false};
  std::atomic<bool> m_stopped{false};
  std::atomic<bool> m_lost{false};
  std::function<void()> m_closed;

  //! guards the dequeued frames: readers dequeue and return them on their own threads
  std::mutex m_bufferLock;
  //! a buffer's memory changes only while no reader holds it
  std::vector<CaptureBuffer> m_buffers;
  std::vector<CaptureBuffer> m_grown;
  size_t m_grownLength{0};
  //! per buffer, the dmabuf it replaced, which vdin may still write until the buffer's next frame
  std::vector<int> m_replaced;
  //! the size the stream runs at and the one before, so a frame of either finds its layout
  std::array<Request, 2> m_schedules;
  std::array<Request, 2> m_layouts;
  std::vector<Layout> m_frameLayouts;
  std::vector<int> m_borrows;
  std::vector<std::chrono::steady_clock::time_point> m_stamps;
  std::vector<int> m_entryCounts;
  std::array<int, 2> m_parked{-1, -1};
  //! per AMLCaptureInput, whether vdin reported no signal at its last source change, and the
  //! source changes read so far
  std::array<bool, 2> m_noSignal{};
  std::array<uint32_t, 2> m_signalChanges{};
  //! the SetInputs callbacks still waiting for a frame of the new turns
  std::vector<std::function<void()>> m_settled;
  //! the turns generation of the newest dequeued frame, and of it at the latest leaving change
  int m_generation{0};
  int m_settledGeneration{0};
};

std::shared_ptr<CAMLCapture::CStream> CAMLCapture::CStream::Open(const AMLCaptureDevice& device,
                                                                 uint32_t inputs,
                                                                 uint32_t pixelFormat,
                                                                 unsigned int buffers,
                                                                 int width,
                                                                 int height,
                                                                 bool turns,
                                                                 int logLevel,
                                                                 Refusal& refusal)
{
  auto stream = std::make_shared<CStream>();
  stream->m_inputs = inputs;
  stream->m_ports = device.inputs;
  int index = -1;
  for (AMLCaptureInput input : INPUTS)
  {
    if (!(inputs & Bit(input)))
      continue;
    if (!device.Offers(input))
      return {};
    if (index < 0)
      index = device.inputs[static_cast<size_t>(input)];
    stream->m_entries++;
  }
  if (index < 0)
    return {};

  stream->m_pixelFormat = pixelFormat;
  stream->m_fd = open(device.path.c_str(), O_RDWR | O_NONBLOCK | O_CLOEXEC);
  stream->m_eventFd = eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
  for (int& fd : stream->m_leaveFds)
    fd = eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
  if (stream->m_fd < 0 || stream->m_eventFd < 0 || stream->m_leaveFds[0] < 0 ||
      stream->m_leaveFds[1] < 0)
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
  stream->m_format.width = width;
  stream->m_format.height = height;
  Layout& layout = stream->m_format.layout;
  layout.width = static_cast<int>(format.fmt.pix_mp.width);
  layout.height = static_cast<int>(format.fmt.pix_mp.height);
  layout.stride = static_cast<int>(format.fmt.pix_mp.plane_fmt[0].bytesperline);
  layout.size = format.fmt.pix_mp.plane_fmt[0].sizeimage;
  stream->m_schedules[0] = stream->m_format;
  stream->m_cositedChroma = format.fmt.pix_mp.flags & PIX_FMT_FLAG_COSITED_CHROMA;
  const size_t lumaSize = static_cast<size_t>(layout.stride) * layout.height;
  const size_t frameSize = fullPack ? lumaSize : lumaSize * 3 / 2;
  if (ioctlResult < 0 || format.fmt.pix_mp.pixelformat != pixelFormat ||
      layout.stride < (fullPack ? layout.width / 2 * AML_FULL_PACK_PAIR_BYTES : layout.width) ||
      layout.size < frameSize)
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

  // a shot exports its buffer, which vb2 does only for its own; a continuous stream brings dmabufs,
  // so it can give them a larger size while it runs
  stream->m_memory = fullPack ? V4L2_MEMORY_MMAP : V4L2_MEMORY_DMABUF;
  v4l2_requestbuffers request{};
  request.count = buffers;
  request.type = BUFFER_TYPE;
  request.memory = stream->m_memory;
  if (ioctl(stream->m_fd, VIDIOC_REQBUFS, &request) < 0 || request.count == 0)
  {
    CLog::Log(logLevel, "CAMLCapture::{}: no capture buffers: {}", __FUNCTION__, strerror(errno));
    return {};
  }

  if (!fullPack)
    stream->m_buffers = Allocate(request.count, layout.size, logLevel);
  else
  {
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

      CaptureBuffer& own = stream->m_buffers.emplace_back();
      own.length = plane.length;
      own.map =
          mmap(nullptr, plane.length, PROT_READ, MAP_SHARED, stream->m_fd, plane.m.mem_offset);
      if (own.map == MAP_FAILED)
        return {};
    }
  }
  if (stream->m_buffers.empty())
    return {};
  const size_t count = stream->m_buffers.size();
  stream->m_borrows.resize(count);
  stream->m_stamps.resize(count);
  stream->m_entryCounts.resize(count);
  stream->m_grown.resize(count);
  stream->m_replaced.resize(count, -1);
  stream->m_frameLayouts.resize(count);

  for (unsigned int i = 0; i < count; i++)
  {
    if (!stream->Queue(i))
      return {};
  }
  stream->Subscribe(inputs);

  // vdin takes turns only from the queue's owner; one input takes them where the node does, so
  // inputs join the running stream
  if (stream->m_entries > 1 || turns)
  {
    const int error = stream->Schedule(inputs, layout.width, layout.height);
    if (error != 0)
    {
      if (error == EBUSY)
        refusal = Refusal::OWNED;
      else if (error == EOPNOTSUPP)
        refusal = Refusal::NO_TURNS;
      if (refusal != Refusal::OWNED)
        CLog::Log(logLevel, "CAMLCapture::{}: turns of inputs {:#x} refused: {}", __FUNCTION__,
                  inputs, strerror(error));
      return {};
    }
    stream->m_scheduled = true;
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
  for (CaptureBuffer& buffer : m_buffers)
    Free(buffer);
  for (CaptureBuffer& buffer : m_grown)
    Free(buffer);
  for (int fd : m_replaced)
  {
    if (fd >= 0)
      close(fd);
  }
  if (m_fd >= 0)
    close(m_fd);
  if (m_eventFd >= 0)
    close(m_eventFd);
  for (int fd : m_leaveFds)
  {
    if (fd >= 0)
      close(fd);
  }
  for (const auto& settled : m_settled)
    settled();
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

int CAMLCapture::CStream::Schedule(uint32_t inputs, int width, int height)
{
  CaptureSchedule schedule;
  schedule.width = static_cast<uint32_t>(width);
  schedule.height = static_cast<uint32_t>(height);
  for (AMLCaptureInput input : INPUTS)
  {
    if (inputs & Bit(input))
      schedule.inputs[schedule.count++] =
          static_cast<uint32_t>(m_ports[static_cast<size_t>(input)]);
  }
  v4l2_ext_control control{};
  control.id = CID_CAPTURE_INPUT_SCHEDULE;
  control.size = sizeof(schedule);
  control.ptr = &schedule;
  v4l2_ext_controls controls{};
  controls.which = V4L2_CTRL_WHICH_CUR_VAL;
  controls.count = 1;
  controls.controls = &control;
  return ioctl(m_fd, VIDIOC_S_EXT_CTRLS, &controls) == 0 ? 0 : errno;
}

void CAMLCapture::CStream::Subscribe(uint32_t inputs)
{
  for (AMLCaptureInput input : INPUTS)
  {
    if (!(inputs & Bit(input)))
      continue;
    v4l2_event_subscription subscription{};
    subscription.type = V4L2_EVENT_SOURCE_CHANGE;
    subscription.id = static_cast<uint32_t>(m_ports[static_cast<size_t>(input)]);
    if (ioctl(m_fd, VIDIOC_SUBSCRIBE_EVENT, &subscription) < 0)
      CLog::Log(LOGDEBUG, "CAMLCapture::{}: no source changes of input {}: {}", __FUNCTION__,
                subscription.id, strerror(errno));
  }
}

bool CAMLCapture::CStream::Fits(int width, int height)
{
  std::lock_guard lock(m_bufferLock);

  const size_t size = GetLayout(width, height).size;
  return size > 0 && std::ranges::all_of(m_buffers, [size](const CaptureBuffer& buffer)
                                         { return buffer.length >= size; });
}

CAMLCapture::CStream::Growth CAMLCapture::CStream::Grow(int width,
                                                        int height,
                                                        bool wait,
                                                        int logLevel)
{
  size_t size = 0;
  size_t missing = 0;
  {
    std::lock_guard lock(m_bufferLock);
    size = GetLayout(width, height).size;
    if (size > m_grownLength)
      missing = std::ranges::count_if(m_buffers, [size](const CaptureBuffer& buffer)
                                      { return buffer.length < size; });
  }
  if (size == 0)
    return Growth::FAILED;

  // the set is allocated outside every lock, and kept across reads until each buffer took its own
  if (missing > 0)
  {
    std::vector<CaptureBuffer> buffers = Allocate(missing, size, logLevel);
    if (buffers.empty())
      return Growth::FAILED;
    {
      std::lock_guard lock(m_bufferLock);
      if (size > m_grownLength)
      {
        for (size_t i = 0; i < m_buffers.size(); i++)
        {
          Free(m_grown[i]);
          if (m_buffers[i].length < size && !buffers.empty())
          {
            m_grown[i] = buffers.back();
            buffers.pop_back();
          }
        }
        m_grownLength = size;
      }
    }
    for (CaptureBuffer& buffer : buffers)
      Free(buffer);
    CLog::Log(LOGINFO, "CAMLCapture::{}: buffers grow to {} bytes for {}x{}", __FUNCTION__, size,
              width, height);
  }
  if (!wait)
    return Fits(width, height) ? Growth::GROWN : Growth::GROWING;

  // the buffers come back as vdin finishes them, which another reader may not drive; each one
  // taken in starts a new window, so only a stream that stopped cycling fails
  const auto window =
      std::chrono::milliseconds(static_cast<int>(std::ceil(FRAME_WAIT_PERIODS * m_framePeriodMs)));
  auto end = std::chrono::steady_clock::now();
  size_t pending = std::numeric_limits<size_t>::max();
  for (;;)
  {
    size_t waiting = 0;
    {
      std::lock_guard lock(m_bufferLock);
      Drain();
      waiting = std::ranges::count_if(m_grown,
                                      [](const CaptureBuffer& buffer) { return buffer.fd >= 0; });
    }
    if (Fits(width, height))
      return Growth::GROWN;
    if (waiting < pending)
    {
      pending = waiting;
      end = std::chrono::steady_clock::now() + window;
    }
    const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(
                          end - std::chrono::steady_clock::now())
                          .count();
    if (left <= 0)
    {
      CLog::Log(logLevel, "CAMLCapture::{}: no buffer grew to {}x{} in {} ms", __FUNCTION__, width,
                height, window.count());
      return Growth::FAILED;
    }
    const Wake wake = Poll(-1, static_cast<int>(left));
    if (wake == Wake::STOPPED || wake == Wake::LOST)
      return Growth::FAILED;
  }
}

bool CAMLCapture::CStream::SetInputs(uint32_t inputs,
                                     int width,
                                     int height,
                                     int logLevel,
                                     std::function<void()> settled,
                                     Refusal& refusal)
{
  std::lock_guard lock(m_bufferLock);

  // the frames of a leaving input go back before vdin drops it; an input that joins counts from
  // vdin's answer on
  const uint32_t running = m_inputs;
  m_inputs = running & inputs;
  for (AMLCaptureInput input : INPUTS)
  {
    int& parked = m_parked[static_cast<size_t>(input)];
    if ((m_inputs & Bit(input)) || parked < 0)
      continue;
    if (m_borrows[parked] == 0)
      Queue(static_cast<unsigned int>(parked));
    parked = -1;
  }

  const Layout layout = GetLayout(width, height);
  if (!m_stopped)
    Subscribe(inputs & ~running);
  const int error = m_stopped ? 0 : Schedule(inputs, layout.width, layout.height);
  if (!m_stopped && error == 0)
  {
    m_inputs = inputs;
    m_entries = std::popcount(inputs);
    if (layout.width != m_schedules[0].layout.width ||
        layout.height != m_schedules[0].layout.height)
    {
      m_schedules[1] = m_schedules[0];
      m_schedules[0] = {width, height, layout};
    }
    for (AMLCaptureInput input : INPUTS)
    {
      const int fd = m_leaveFds[static_cast<size_t>(input)];
      uint64_t wake = 1;
      ssize_t done = 0;
      if (running & ~inputs & Bit(input))
        done = write(fd, &wake, sizeof(wake));
      else if (inputs & ~running & Bit(input))
        done = read(fd, &wake, sizeof(wake));
      if (done < 0 && errno != EAGAIN)
        CLog::Log(LOGDEBUG, "CAMLCapture::{}: wake failed: {}", __FUNCTION__, strerror(errno));
    }
    // vdin drops a leaving input's channel only at the first frame of the new turns
    if ((running & ~inputs) && settled)
    {
      m_settled.push_back(std::move(settled));
      m_settledGeneration = m_generation;
    }
    else if (settled)
      settled();
    CLog::Log(LOGINFO, "CAMLCapture::{}: inputs {:#x} take turns {}x{}", __FUNCTION__, inputs,
              layout.width, layout.height);
    return true;
  }

  m_inputs = running;
  if (settled)
    settled();
  if (error == EBUSY)
    refusal = Refusal::NO_CHANNEL;
  else if (error == EOPNOTSUPP)
    refusal = Refusal::NO_TURNS;
  else if (error != 0)
    CLog::Log(logLevel, "CAMLCapture::{}: turns of inputs {:#x} at {}x{} refused: {}", __FUNCTION__,
              inputs, layout.width, layout.height, strerror(error));
  return false;
}

bool CAMLCapture::CStream::Queue(unsigned int index)
{
  CaptureBuffer& own = m_buffers[index];
  if (m_grown[index].fd >= 0)
  {
    munmap(own.map, own.length);
    m_replaced[index] = own.fd;
    own = std::exchange(m_grown[index], {});
  }

  v4l2_plane plane{};
  v4l2_buffer buffer{};
  buffer.index = index;
  buffer.type = BUFFER_TYPE;
  buffer.memory = m_memory;
  buffer.m.planes = &plane;
  buffer.length = 1;
  if (m_memory == V4L2_MEMORY_DMABUF)
  {
    plane.m.fd = own.fd;
    plane.length = static_cast<uint32_t>(own.length);
  }
  if (ioctl(m_fd, VIDIOC_QBUF, &buffer) == 0)
    return true;

  // vdin would never finish another frame in it
  if (m_streaming && !m_stopped && !m_lost.exchange(true))
  {
    CLog::Log(LOGWARNING, "CAMLCapture::{}: buffer {} not queued: {}", __FUNCTION__, index,
              strerror(errno));
    const uint64_t wake = 1;
    if (write(m_eventFd, &wake, sizeof(wake)) < 0)
      CLog::Log(LOGDEBUG, "CAMLCapture::{}: wake failed: {}", __FUNCTION__, strerror(errno));
  }
  return false;
}

bool CAMLCapture::CStream::Dequeue(unsigned int& index,
                                   int& input,
                                   int& entries,
                                   int& generation,
                                   Layout& layout,
                                   std::chrono::steady_clock::time_point& stamp)
{
  v4l2_plane plane{};
  v4l2_buffer buffer{};
  buffer.type = BUFFER_TYPE;
  buffer.memory = m_memory;
  buffer.m.planes = &plane;
  buffer.length = 1;
  if (ioctl(m_fd, VIDIOC_DQBUF, &buffer) < 0)
    return false;

  index = buffer.index;
  if (m_replaced[index] >= 0)
    close(std::exchange(m_replaced[index], -1));
  // an unstamped frame has the size of the open
  layout = m_format.layout;
  stamp =
      std::chrono::steady_clock::time_point(std::chrono::seconds(buffer.timestamp.tv_sec) +
                                            std::chrono::microseconds(buffer.timestamp.tv_usec));
  input = -1;
  entries = 0;
  generation = 0;
  if (buffer.flags & V4L2_BUF_FLAG_TIMECODE)
  {
    entries = static_cast<int>(buffer.timecode.userbits[1]);
    generation = static_cast<int>(buffer.timecode.userbits[2]);
    for (AMLCaptureInput tap : INPUTS)
    {
      if ((m_inputs & Bit(tap)) &&
          m_ports[static_cast<size_t>(tap)] == static_cast<int>(buffer.timecode.userbits[0]))
        input = static_cast<int>(tap);
    }
    const int width = buffer.timecode.frames | buffer.timecode.seconds << 8;
    const int height = buffer.timecode.minutes | buffer.timecode.hours << 8;
    if (input >= 0 && width > 0 && height > 0)
      layout = GetLayout(width, height);
  }
  if (layout.size == 0)
    input = -1;
  return true;
}

CAMLCapture::CStream::Wake CAMLCapture::CStream::Poll(int leaveFd, int timeoutMs) const
{
  pollfd fds[] = {{m_fd, POLLIN | POLLPRI, 0}, {m_eventFd, POLLIN, 0}, {leaveFd, POLLIN, 0}};
  int ret = -1;
  do
    ret = poll(fds, 3, timeoutMs);
  while (ret < 0 && errno == EINTR);

  if (m_lost)
    return Wake::LOST;
  if (m_stopped || (fds[1].revents & POLLIN) || (fds[2].revents & POLLIN))
    return Wake::STOPPED;
  if (fds[0].revents & POLLERR)
    return Wake::LOST;
  if (ret > 0 && (fds[0].revents & POLLIN))
    return Wake::FRAME;
  return ret > 0 && (fds[0].revents & POLLPRI) ? Wake::EVENT : Wake::TIMEOUT;
}

void CAMLCapture::CStream::Drain()
{
  unsigned int next = 0;
  int tag = -1;
  int entries = 0;
  int generation = 0;
  Layout layout;
  std::chrono::steady_clock::time_point stamp;
  v4l2_event event{};
  // a DQBUF on an empty queue logs in the driver, so every one waits for poll
  for (;;)
  {
    const Wake wake = Poll(-1, 0);
    // vdin queues a source change on every blank frame of an input from the second in a row, and
    // once it clears; the input's status tells which
    if (wake == Wake::EVENT && ioctl(m_fd, VIDIOC_DQEVENT, &event) == 0)
    {
      v4l2_input status{};
      status.index = event.id;
      for (AMLCaptureInput tap : INPUTS)
      {
        const size_t i = static_cast<size_t>(tap);
        if (m_ports[i] != static_cast<int>(event.id) || ioctl(m_fd, VIDIOC_ENUMINPUT, &status) < 0)
          continue;
        m_noSignal[i] = (status.status & V4L2_IN_ST_NO_SIGNAL) != 0;
        m_signalChanges[i]++;
      }
      continue;
    }
    if (wake != Wake::FRAME || !Dequeue(next, tag, entries, generation, layout, stamp))
      break;
    m_generation = generation;
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
    m_entryCounts[next] = entries;
    m_frameLayouts[next] = layout;
    // frames of older turns of the same length may still be queued
    if (!m_settled.empty() && generation != m_settledGeneration && entries == m_entries)
    {
      for (const auto& settled : std::exchange(m_settled, {}))
        settled();
    }
  }

  // vdin hands a frame over a vsync after it finishes it, and each input takes its turn once
  // per round of the schedule, the frame's or the current one, so a newer frame of the input is
  // done after the longer round
  const auto now = std::chrono::steady_clock::now();
  for (int& parked : m_parked)
  {
    if (parked < 0)
      continue;
    const int round = std::max(m_entryCounts[parked], m_entries);
    const std::chrono::duration<float, std::milli> maxAge((round + 1) * m_framePeriodMs);
    if (m_stopped || now - m_stamps[parked] >= maxAge)
    {
      if (m_borrows[parked] == 0)
        Queue(parked);
      parked = -1;
    }
  }
}

CAMLCapture::CStream::Layout CAMLCapture::CStream::GetLayout(int width, int height)
{
  // a frame's size is one vdin bounded already, which it keeps
  const auto known = [width, height](const Request& request)
  {
    return (width == request.width && height == request.height) ||
           (width == request.layout.width && height == request.layout.height);
  };
  if (known(m_format))
    return m_format.layout;
  for (const Request& request : m_schedules)
  {
    if (known(request))
      return request.layout;
  }
  for (const Request& request : m_layouts)
  {
    if (known(request))
      return request.layout;
  }

  v4l2_format format{};
  format.type = BUFFER_TYPE;
  format.fmt.pix_mp.width = width;
  format.fmt.pix_mp.height = height;
  format.fmt.pix_mp.pixelformat = m_pixelFormat;
  format.fmt.pix_mp.field = V4L2_FIELD_NONE;
  format.fmt.pix_mp.num_planes = 1;
  if (ioctl(m_fd, VIDIOC_TRY_FMT, &format) < 0)
  {
    CLog::Log(LOGWARNING, "CAMLCapture::{}: no layout for {}x{}: {}", __FUNCTION__, width, height,
              strerror(errno));
    return {};
  }
  m_layouts[1] = m_layouts[0];
  m_layouts[0] = {width,
                  height,
                  {static_cast<int>(format.fmt.pix_mp.width),
                   static_cast<int>(format.fmt.pix_mp.height),
                   static_cast<int>(format.fmt.pix_mp.plane_fmt[0].bytesperline),
                   format.fmt.pix_mp.plane_fmt[0].sizeimage}};
  return m_layouts[0].layout;
}

bool CAMLCapture::CStream::Take(AMLCaptureInput input, int width, int height, unsigned int& index)
{
  std::lock_guard lock(m_bufferLock);

  Drain();
  const int parked = m_parked[static_cast<size_t>(input)];
  if (parked < 0)
    return false;
  const Layout& frame = m_frameLayouts[parked];
  const Layout& running = m_schedules[0].layout;
  if (frame.width < std::min(width, running.width) ||
      frame.height < std::min(height, running.height))
    return false;
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

CAMLCapture::CStream::Wake CAMLCapture::CStream::Wait(
    AMLCaptureInput input, int width, int height, int timeoutMs, unsigned int& index)
{
  const size_t tap = static_cast<size_t>(input);
  uint32_t changes = 0;
  {
    std::lock_guard lock(m_bufferLock);
    changes = m_signalChanges[tap];
  }
  const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeoutMs);
  for (;;)
  {
    const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(
                          end - std::chrono::steady_clock::now())
                          .count();
    const Wake wake = Poll(m_leaveFds[tap], static_cast<int>(std::max<int64_t>(left, 0)));
    if (wake != Wake::FRAME && wake != Wake::EVENT && wake != Wake::TIMEOUT)
      return wake;

    // the frame may be another input's, or another reader may have dequeued and parked it
    if (Take(input, width, height, index))
      return Wake::FRAME;
    {
      std::lock_guard lock(m_bufferLock);
      if (m_signalChanges[tap] != changes && m_noSignal[tap])
        return Wake::NO_SIGNAL;
    }
    if (wake == Wake::TIMEOUT || left <= 0)
      return Wake::TIMEOUT;
  }
}

void CAMLCapture::CStream::Bound(int& width, int& height)
{
  std::lock_guard lock(m_bufferLock);

  const Layout layout = GetLayout(width, height);
  width = layout.width;
  height = layout.height;
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
  const Layout& layout = m_frameLayouts[index];
  AMLCaptureImage image;
  image.width = layout.width;
  image.height = layout.height;
  image.data[0] = static_cast<uint8_t*>(m_buffers[index].map);
  if (m_pixelFormat == PIX_FMT_YUV422_10P)
  {
    image.fullPack = true;
    image.cositedChroma = m_cositedChroma;
    image.linesize[0] = layout.stride;
    return image;
  }

  // single-plane NV12: the chroma plane follows the luma plane at the same stride
  image.format = AV_PIX_FMT_NV12;
  image.data[1] = image.data[0] + static_cast<size_t>(layout.stride) * layout.height;
  image.linesize[0] = layout.stride;
  image.linesize[1] = layout.stride;
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
  m_displayReset.notify_all();
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
    m_closed.notify_all();
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

AMLCaptureRead CAMLCapture::Unreadable(bool log) const
{
  if (log)
    CLog::Log(LOGWARNING, "CAMLCapture::{}: no video frame: {}", __FUNCTION__,
              m_displayLost ? "display lost" : "no video presented");
  return m_displayLost ? AMLCaptureRead::NO_PICTURE : AMLCaptureRead::ENDED;
}

AMLCaptureRead CAMLCapture::Read(AMLCaptureInput input, int width, int height, const ReadFunc& use)
{
  // a lost display gives no frame, so a reader waits as long as for one
  {
    std::unique_lock lock(m_lock);
    const auto bound = std::chrono::milliseconds(
        static_cast<int>(std::ceil(FRAME_WAIT_PERIODS * m_framePeriodMs)));
    m_displayReset.wait_for(lock, bound, [this] { return !m_displayLost; });
  }
  const auto lease = Lease(input, width, height);
  std::shared_ptr<ConsumerConvert> convert;
  {
    std::lock_guard lock(m_lock);
    if (!CanRead(input))
      return Unreadable(false);
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
      return AMLCaptureRead::ENDED;

    std::unique_lock open(m_engines[engine].open);
    bool refused = false;
    const AMLCaptureRead result = ReadStream(open, static_cast<size_t>(engine), input,
                                             V4L2_PIX_FMT_NV12, width, height, read, refused);
    if (!refused)
      return result;
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
  return Read(AMLCaptureInput::VIDEO, width, height, use) == AMLCaptureRead::FRAME;
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
                    refused, shot == CaptureScreenshot::NATIVE) == AMLCaptureRead::FRAME;

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

AMLCaptureRead CAMLCapture::ReadStream(std::unique_lock<std::mutex>& open,
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
  // the reader's own size; the stream may run larger for other readers
  int readWidth = width;
  int readHeight = height;
  uint32_t inputs = Bit(input);
  uint32_t replaced = 0;
  bool update = false;
  bool leaving = false;
  bool turns = false;
  bool served = false;
  uint32_t epoch = 0;
  std::shared_ptr<CStream> stream;
  std::vector<std::shared_ptr<CStream>> stale;
  AMLCaptureSource source;
  // the store lock is never taken under m_lock
  AMLFrameMetadataPtr meta = CAMLFrameMetadataStore::GetInstance().Get();
  {
    std::lock_guard lock(m_lock);
    if (!CanRead(input))
      return Unreadable(logCause);
    epoch = m_displayEpoch;
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
        return AMLCaptureRead::ENDED;
      }
      served = node.stream && node.stream->Serves(input, width, height);
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
        return AMLCaptureRead::ENDED;
      }
    }
    turns = !shot && !node.unscheduled;
    if (node.stream && !node.stream->Matches(inputs, pixelFormat, width, height))
    {
      replaced = node.stream->GetInputs();
      update = node.stream->IsScheduled() && turns && !(node.busy & plans[engine].inputs) &&
               pixelFormat == node.stream->GetPixelFormat();
      leaving = update && (replaced & ~inputs);
      if (!update)
        stale.push_back(TakeStream(engine));
      // until vdin drops the inputs that leave, an open elsewhere may be refused for them
      if (leaving)
      {
        m_settling++;
        m_streamChanges++;
      }
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

  // a larger size waits for buffers that hold it, without the open lock; a reader the stream
  // already serves reads on meanwhile
  if (update && !stream->Fits(width, height))
  {
    const int logLevel = node.openFailed ? LOGDEBUG : LOGWARNING;
    open.unlock();
    const CStream::Growth growth = stream->Grow(width, height, !served, logLevel);
    {
      std::lock_guard lock(m_lock);
      if (leaving)
        m_settling--;
      if (growth == CStream::Growth::FAILED && node.stream == stream)
        stale.push_back(TakeStream(engine));
    }
    for (const auto& old : stale)
      Close(old);
    stale.clear();
    if (growth != CStream::Growth::GROWING)
    {
      refused = true;
      return AMLCaptureRead::ENDED;
    }
    update = false;
  }

  if (!stream || update)
  {
    uint32_t changes = 0;
    bool settling = false;
    {
      std::unique_lock lock(m_lock);
      // the stream a reader updates holds m_closing up once taken, until the reader lets it go
      m_closed.wait(lock, [&] { return m_closing == 0 || (update && node.stream != stream); });
      if (update && node.stream != stream)
      {
        if (leaving)
          m_settling--;
        refused = true;
        return AMLCaptureRead::ENDED;
      }
      if (!CanRead(input))
      {
        if (leaving)
          m_settling--;
        return Unreadable(logCause);
      }
      epoch = m_displayEpoch;
      changes = m_streamChanges;
      settling = m_settling > (leaving ? 1 : 0);
    }

    CStream::Refusal refusal = CStream::Refusal::NONE;
    bool started = false;
    const int logLevel = node.openFailed ? LOGDEBUG : LOGWARNING;
    if (update)
    {
      std::function<void()> settled;
      if (leaving)
        settled = [weak = weak_from_this()]
        {
          if (const auto session = weak.lock())
          {
            std::lock_guard lock(session->m_lock);
            session->m_settling--;
          }
        };
      started = stream->SetInputs(inputs, width, height, logLevel, std::move(settled), refusal);
    }
    else
    {
      // inputs join a stream that takes turns while it runs, so it holds buffers for every input
      // of the node
      const auto carried = static_cast<unsigned int>(
          turns ? std::ranges::count_if(INPUTS, [&device](AMLCaptureInput tap)
                                        { return device.Offers(tap); })
                : std::popcount(inputs));
      const unsigned int buffers =
          shot ? SHOT_BUFFERS : BUFFERS_PER_INPUT * carried + STREAM_BUFFERS;
      stream = CStream::Open(device, inputs, pixelFormat, buffers, width, height, turns, logLevel,
                             refusal);
      started = stream != nullptr;
    }
    const bool busy = refusal == CStream::Refusal::OWNED || refusal == CStream::Refusal::NO_CHANNEL;

    std::unique_lock lock(m_lock);
    if (shot)
      refused = !stream;
    else
    {
      // a mode set, a change of our own streams, inputs vdin has yet to drop and a shot meanwhile
      // refuse an open too
      const bool ours =
          epoch != m_displayEpoch || changes != m_streamChanges || settling || m_settling > 0 ||
          std::any_of(m_engines.begin(), m_engines.end(),
                      [](const Engine& other) {
                        return other.stream && other.stream->GetPixelFormat() == PIX_FMT_YUV422_10P;
                      });
      // an open ends the refusals of its inputs, and of all inputs when another user owned the node
      if (started)
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
    // a sweep or a mode set took the stream meanwhile
    if (update && node.stream != stream)
    {
      refused = true;
      return AMLCaptureRead::ENDED;
    }
    if (started)
      stream->SetReadSize(width, height);
    if (!busy)
      node.openFailed = !started;
    // vdin kept the old turns; for no reason that moves a consumer, the stream restarts
    if (update && !started && refusal == CStream::Refusal::NONE)
    {
      stale.push_back(TakeStream(engine));
      lock.unlock();
      for (const auto& old : stale)
        Close(old);
      refused = true;
      return AMLCaptureRead::ENDED;
    }
    if (update && !started && (refused || !(stream->GetInputs() & Bit(input))))
      return AMLCaptureRead::ENDED;
    if (!stream)
      return AMLCaptureRead::ENDED;
    // a stream opened across a mode set still holds the old mode
    if (epoch != m_displayEpoch)
    {
      if (logCause)
        CLog::Log(LOGWARNING, "CAMLCapture::{}: the output mode changed", __FUNCTION__);
      return AMLCaptureRead::NO_PICTURE;
    }
    if (!CanRead(input))
      return Unreadable(logCause);
    node.stream = stream;
    m_framePeriodMs = stream->GetFramePeriodMs();
    m_streamChanges++;
  }
  if (!shot && open.owns_lock())
    open.unlock();

  const int waitMs = static_cast<int>(std::ceil(FRAME_WAIT_PERIODS * stream->GetFramePeriodMs()));
  unsigned int index = 0;
  bool bounded = false;
  if (!shot)
  {
    std::lock_guard lock(m_lock);
    const auto consumer = FindConsumer(input, readWidth, readHeight);
    if (consumer->bounded.lock() == stream)
    {
      readWidth = consumer->boundWidth;
      readHeight = consumer->boundHeight;
      bounded = true;
    }
  }
  if (!bounded)
  {
    const int askedWidth = readWidth;
    const int askedHeight = readHeight;
    stream->Bound(readWidth, readHeight);
    if (!shot)
    {
      std::lock_guard lock(m_lock);
      const auto consumer = FindConsumer(input, askedWidth, askedHeight);
      consumer->bounded = stream;
      consumer->boundWidth = readWidth;
      consumer->boundHeight = readHeight;
    }
  }
  const CStream::Wake wake = stream->Take(input, readWidth, readHeight, index)
                                 ? CStream::Wake::FRAME
                                 : stream->Wait(input, readWidth, readHeight, waitMs, index);
  if (wake == CStream::Wake::FRAME)
  {
    AMLCaptureFrame frame;
    frame.color = GetSignal(source);
    frame.image = stream->Image(index);
    const bool ok = use(*stream, index, frame);
    stream->Return(index);
    return ok ? AMLCaptureRead::FRAME : AMLCaptureRead::ENDED;
  }
  if (wake == CStream::Wake::NO_SIGNAL)
  {
    if (logCause)
      CLog::Log(LOGDEBUG, "CAMLCapture::{}: no signal on the input", __FUNCTION__);
    return AMLCaptureRead::NO_PICTURE;
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
    return AMLCaptureRead::ENDED;
  }
  // without a frame or a change to no signal, the stream stays open for the next read
  if (wake == CStream::Wake::TIMEOUT)
  {
    std::lock_guard lock(m_lock);
    if (node.stream == stream)
    {
      if (logCause)
        CLog::Log(LOGWARNING, "CAMLCapture::{}: no frame in {} ms, stream kept open", __FUNCTION__,
                  waitMs);
      return AMLCaptureRead::ENDED;
    }
  }
  {
    std::lock_guard lock(m_lock);
    if (epoch != m_displayEpoch)
      return AMLCaptureRead::NO_PICTURE;
  }
  if (logCause)
    CLog::Log(LOGWARNING, "CAMLCapture::{}: stream stopped", __FUNCTION__);
  return AMLCaptureRead::ENDED;
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
  // a request keeps its pixels, and with them its image, from delivery to delivery
  if (result.live)
  {
    auto& pixels = static_cast<CAMLLivePixels&>(*result.pixels);
    // the new lease comes first, so the stream is never left without one
    if (!pixels.HasStreamSize(streamWidth, streamHeight))
      pixels.SetStreamSize(session->Lease(AMLCaptureInput::SCREEN, streamWidth, streamHeight),
                           streamWidth, streamHeight);
    return;
  }

  auto lease = session->Lease(AMLCaptureInput::SCREEN, streamWidth, streamHeight);
  result.pixels = std::make_shared<CAMLLivePixels>(session, std::move(lease), width, height,
                                                   streamWidth, streamHeight);
  DescribeSdr(width, height, false, result);
  result.content = CaptureContent::COMPOSITE;
  result.live = true;
}
