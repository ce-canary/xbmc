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
#include "cores/VideoPlayer/DVDCodecs/Video/AMLFrameMetadata.h"
#include "cores/VideoPlayer/Interface/StreamInfo.h"
#include "jobs/JobManager.h"
#include "rendering/capture/CapturePixels.h"
#include "settings/Settings.h"
#include "settings/SettingsComponent.h"
#include "utils/AMLUtils.h"
#include "utils/log.h"
#include "windowing/GraphicContext.h"
#include "windowing/amlogic/WinSystemAmlogic.h"

#include "platform/linux/SysfsPath.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <string>
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

// the first frame after STREAMON comes within this many frame periods
constexpr int FRAME_WAIT_PERIODS = 10;
// vdin1 writes one, holds the last, the reader holds one and one stays free
constexpr unsigned int CAPTURE_BUFFERS = 4;
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

struct AMLCaptureDevice
{
  std::string path;
  int input{-1};
  //! the widest scaled capture
  int maxWidth{0};
};

AMLCaptureDevice Discover()
{
  AMLCaptureDevice device;

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

    v4l2_capability cap{};
    if (ioctl(fd, VIDIOC_QUERYCAP, &cap) == 0 &&
        std::string(reinterpret_cast<const char*>(cap.bus_info)) == "vdin1 v4l2")
    {
      v4l2_input input{};
      for (input.index = 0; ioctl(fd, VIDIOC_ENUMINPUT, &input) == 0; input.index++)
      {
        const std::string inputName(reinterpret_cast<const char*>(input.name));
        if (inputName == "VIU1_WB0_VD1")
        {
          device.path = path;
          device.input = static_cast<int>(input.index);
        }
      }
      v4l2_frmsizeenum size{};
      size.pixel_format = V4L2_PIX_FMT_NV12;
      if (ioctl(fd, VIDIOC_ENUM_FRAMESIZES, &size) == 0)
        device.maxWidth = static_cast<int>(size.stepwise.max_width);
      else
        device.input = -1;
    }
    close(fd);
    if (device.input >= 0)
      break;
  }

  if (device.input >= 0)
    CLog::Log(LOGINFO, "CAMLCapture::{}: video capture on {} input {}", __FUNCTION__, device.path,
              device.input);
  else
    CLog::Log(LOGINFO, "CAMLCapture::{}: no vdin1 video capture node", __FUNCTION__);
  return device;
}

const AMLCaptureDevice& FindDevice()
{
  static const AMLCaptureDevice device = Discover();
  return device;
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
void DescribeSdr(int width, int height, CaptureResult& result)
{
  result.width = static_cast<unsigned int>(width);
  result.height = static_cast<unsigned int>(height);
  result.stride = width * 4;
  result.format = AV_PIX_FMT_BGRA;
  result.color.primaries = AVCOL_PRI_BT709;
  result.color.transfer = AVCOL_TRC_IEC61966_2_1;
  result.color.range = AVCOL_RANGE_JPEG;
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
      return nullptr;

    auto buffer = std::make_unique_for_overwrite<uint8_t[]>(static_cast<size_t>(m_spec.width) *
                                                            m_spec.height * 4);
    const AMLCaptureImage dst = AMLCaptureImage::Packed(
        AV_PIX_FMT_BGRA, m_spec.width, m_spec.height, buffer.get(), m_spec.width * 4);
    const bool ok =
        session->Read(m_streamWidth, m_streamHeight,
                      [this, &dst](const AMLCaptureFrame& frame, CAMLCaptureConvert& convert)
                      {
                        return convert.Convert(CAMLCapture::Crop(frame.image, m_spec.rect),
                                               frame.color, AMLCaptureCoding(), dst);
                      });

    if (ok)
      m_buffer = std::move(buffer);
    return m_buffer.get();
  }

private:
  std::weak_ptr<CAMLCapture> m_session;
  std::shared_ptr<void> m_lease;
  AMLCaptureVideoSpec m_spec;
  int m_streamWidth{0};
  int m_streamHeight{0};
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
  };

  //! failures log at `logLevel`; `busy` tells whether another vdin1 user holds the node
  static std::shared_ptr<CStream> Open(int width, int height, int logLevel, bool& busy);
  ~CStream();

  void Stop();
  int GetBuffers() const { return static_cast<int>(m_maps.size()); }
  float GetFramePeriodMs() const { return m_framePeriodMs; }
  bool Matches(int width, int height) const { return width == m_width && height == m_height; }
  //! Takes the newest frame vdin1 finished since the last dequeue, false when there is none or
  //! when that dequeue is older than `maxAge`
  bool TakeNewest(std::chrono::milliseconds maxAge, unsigned int& index);
  Wake Wait(int timeoutMs, unsigned int& index);
  bool Queue(unsigned int index);
  AMLCaptureImage Image(unsigned int index) const;

private:
  bool Dequeue(unsigned int& index);
  bool HasFrame(int timeoutMs, bool& stopped);

  int m_fd{-1};
  int m_eventFd{-1};
  int m_width{0};
  int m_height{0};
  // the size vdin1 agreed to deliver for the size asked for
  int m_imageWidth{0};
  int m_imageHeight{0};
  int m_stride{0};
  float m_framePeriodMs{0.0f};
  bool m_streaming{false};
  std::atomic<bool> m_stopped{false};
  bool m_lost{false};
  std::vector<std::pair<void*, size_t>> m_maps;
  std::chrono::steady_clock::time_point m_dequeued;
};

std::shared_ptr<CAMLCapture::CStream> CAMLCapture::CStream::Open(int width,
                                                                 int height,
                                                                 int logLevel,
                                                                 bool& busy)
{
  const AMLCaptureDevice& device = FindDevice();
  int index = device.input;
  if (index < 0)
    return {};

  auto stream = std::make_shared<CStream>();
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
    busy = errno == EBUSY;
    if (!busy)
      CLog::Log(logLevel, "CAMLCapture::{}: input {} unavailable: {}", __FUNCTION__, index,
                strerror(errno));
    return {};
  }

  v4l2_format format{};
  format.type = BUFFER_TYPE;
  format.fmt.pix_mp.width = width;
  format.fmt.pix_mp.height = height;
  format.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_NV12;
  format.fmt.pix_mp.field = V4L2_FIELD_NONE;
  format.fmt.pix_mp.num_planes = 1;
  const int ioctlResult = ioctl(stream->m_fd, VIDIOC_S_FMT, &format);
  stream->m_stride = static_cast<int>(format.fmt.pix_mp.plane_fmt[0].bytesperline);
  stream->m_imageWidth = static_cast<int>(format.fmt.pix_mp.width);
  stream->m_imageHeight = static_cast<int>(format.fmt.pix_mp.height);
  const size_t frameSize = static_cast<size_t>(stream->m_stride) * stream->m_imageHeight * 3 / 2;
  if (ioctlResult < 0 || format.fmt.pix_mp.pixelformat != V4L2_PIX_FMT_NV12 ||
      stream->m_stride < stream->m_imageWidth ||
      format.fmt.pix_mp.plane_fmt[0].sizeimage < frameSize)
  {
    busy = ioctlResult < 0 && errno == EBUSY;
    CLog::Log(logLevel, "CAMLCapture::{}: NV12 {}x{} refused", __FUNCTION__, width, height);
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
  request.count = CAPTURE_BUFFERS;
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

  for (unsigned int i = 0; i < stream->m_maps.size(); i++)
  {
    if (!stream->Queue(i))
      return {};
  }

  uint32_t type = BUFFER_TYPE;
  if (ioctl(stream->m_fd, VIDIOC_STREAMON, &type) < 0)
  {
    CLog::Log(logLevel, "CAMLCapture::{}: stream start failed: {}", __FUNCTION__, strerror(errno));
    return {};
  }
  stream->m_streaming = true;

  CLog::Log(LOGINFO, "CAMLCapture::{}: started input {} {}x{}", __FUNCTION__, index, width, height);
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

bool CAMLCapture::CStream::Dequeue(unsigned int& index)
{
  v4l2_plane plane{};
  v4l2_buffer buffer{};
  buffer.type = BUFFER_TYPE;
  buffer.memory = V4L2_MEMORY_MMAP;
  buffer.m.planes = &plane;
  buffer.length = 1;
  if (ioctl(m_fd, VIDIOC_DQBUF, &buffer) < 0)
    return false;

  m_dequeued = std::chrono::steady_clock::now();
  index = buffer.index;
  return true;
}

bool CAMLCapture::CStream::HasFrame(int timeoutMs, bool& stopped)
{
  pollfd fds[] = {{m_fd, POLLIN, 0}, {m_eventFd, POLLIN, 0}};
  int ret = -1;
  do
    ret = poll(fds, 2, timeoutMs);
  while (ret < 0 && errno == EINTR);

  stopped = m_stopped || (fds[1].revents & POLLIN);
  m_lost = fds[0].revents & POLLERR;
  return ret > 0 && !stopped && !m_lost && (fds[0].revents & POLLIN);
}

bool CAMLCapture::CStream::TakeNewest(std::chrono::milliseconds maxAge, unsigned int& index)
{
  const auto age = std::chrono::steady_clock::now() - m_dequeued;

  // a DQBUF on an empty queue logs in the driver, so every one waits for poll
  bool stopped = false;
  size_t done = 0;
  unsigned int next = 0;
  while (HasFrame(0, stopped) && Dequeue(next))
  {
    if (done++ > 0)
      Queue(index);
    index = next;
  }
  if (done == 0)
    return false;

  // vdin1 hands each frame over a vsync late and drops frames once the buffers the reader does
  // not hold are done, so after that many periods since the last dequeue the newest may be old
  if (stopped || age >= maxAge)
  {
    Queue(index);
    return false;
  }
  return true;
}

CAMLCapture::CStream::Wake CAMLCapture::CStream::Wait(int timeoutMs, unsigned int& index)
{
  const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeoutMs);
  for (;;)
  {
    const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(
                          end - std::chrono::steady_clock::now())
                          .count();
    bool stopped = false;
    if (!HasFrame(static_cast<int>(std::max<int64_t>(left, 0)), stopped))
    {
      if (stopped)
        return Wake::STOPPED;
      return m_lost ? Wake::LOST : Wake::TIMEOUT;
    }

    if (Dequeue(index))
      return Wake::FRAME;
    if (left <= 0)
      return Wake::TIMEOUT;
  }
}

AMLCaptureImage CAMLCapture::CStream::Image(unsigned int index) const
{
  // single-plane NV12: the chroma plane follows the luma plane at the same stride
  AMLCaptureImage image;
  image.format = AV_PIX_FMT_NV12;
  image.width = m_imageWidth;
  image.height = m_imageHeight;
  image.data[0] = static_cast<uint8_t*>(m_maps[index].first);
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

std::shared_ptr<CAMLCapture> CAMLCapture::GetActive()
{
  std::lock_guard lock(s_sessionLock);

  return s_session.lock();
}

void CAMLCapture::GetCaptureSize(int& width, int& height)
{
  const RESOLUTION_INFO screen = CServiceBroker::GetWinSystem()->GetGfxContext().GetResInfo();
  const int screenWidth = std::max(screen.iScreenWidth, 1);
  const int screenHeight = std::max(screen.iScreenHeight, 1);
  const float scale = std::min(1.0f, static_cast<float>(FindDevice().maxWidth) / screenWidth);
  width = static_cast<int>(screenWidth * scale) & ~1;
  height = static_cast<int>(screenHeight * scale) & ~1;
}

CAMLCapture::CAMLCapture()
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
  std::shared_ptr<CStream> stream;
  {
    std::lock_guard lock(m_lock);
    m_displayLost = true;
    m_displayEpoch++;
    stream = TakeStream();
  }
  Close(std::move(stream));
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
  m_busy = false;
}

void CAMLCapture::SetPresented()
{
  m_presented = true;
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

std::shared_ptr<CAMLCapture::CStream> CAMLCapture::TakeStream()
{
  if (m_stream)
    m_closing++;
  return std::move(m_stream);
}

void CAMLCapture::Close(std::shared_ptr<CStream> stream)
{
  if (!stream)
    return;

  stream->Stop();
  stream.reset();

  std::lock_guard lock(m_lock);
  m_closing--;
  m_closed.notify_all();
}

std::shared_ptr<void> CAMLCapture::Lease(int width, int height)
{
  std::lock_guard lock(m_lock);

  m_leases++;
  if (width > 0 && height > 0)
    m_continuousSizes.emplace_back(width, height);
  return std::shared_ptr<void>(nullptr,
                               [weak = weak_from_this(), width, height](void*)
                               {
                                 if (const auto session = weak.lock())
                                   session->ReleaseLease(width, height);
                               });
}

void CAMLCapture::ReleaseLease(int width, int height)
{
  {
    std::lock_guard lock(m_lock);
    const auto it = std::find(m_continuousSizes.begin(), m_continuousSizes.end(),
                              std::make_pair(width, height));
    if (it != m_continuousSizes.end())
      m_continuousSizes.erase(it);
    if (--m_leases > 0 || !m_stream)
      return;
  }

  // a lease can end on the render thread, which makes no stream ioctls; the job holds the
  // session, so its destructor does not stop the stream there either
  CServiceBroker::GetJobManager()->Submit(
      [session = shared_from_this()]
      {
        std::shared_ptr<CStream> stream;
        {
          std::lock_guard lock(session->m_lock);
          if (session->m_leases > 0)
            return;
          stream = session->TakeStream();
        }
        session->Close(std::move(stream));
      });
}

bool CAMLCapture::CanRead() const
{
  if (m_displayLost)
    return false;
  return m_presented && !m_busy;
}

bool CAMLCapture::Read(int width, int height, const ReadFunc& use)
{
  std::lock_guard read(m_readLock);

  return ReadStream(width, height,
                    [this, &use](CStream&, unsigned int, const AMLCaptureFrame& frame)
                    { return use(frame, m_convert); });
}

bool CAMLCapture::ReadStream(int width, int height, const StreamFunc& use)
{
  std::shared_ptr<CStream> stream;
  std::shared_ptr<CStream> stale;
  AMLCaptureSource source;
  // the store lock is never taken under m_lock
  AMLFrameMetadataPtr meta = CAMLFrameMetadataStore::GetInstance().Get();
  {
    std::lock_guard lock(m_lock);
    if (!CanRead())
      return false;
    for (const auto& [continuousWidth, continuousHeight] : m_continuousSizes)
    {
      width = std::max(width, continuousWidth);
      height = std::max(height, continuousHeight);
    }
    if (m_stream && !m_stream->Matches(width, height))
      stale = TakeStream();
    stream = m_stream;
    UpdateStatics(std::move(meta));
    source = m_source;
  }
  Close(std::move(stale));

  if (!stream)
  {
    uint32_t epoch = 0;
    {
      std::unique_lock lock(m_lock);
      m_closed.wait(lock, [this] { return m_closing == 0; });
      if (!CanRead())
        return false;
      epoch = m_displayEpoch;
    }

    bool busy = false;
    const int logLevel = m_openFailed ? LOGDEBUG : LOGWARNING;
    stream = CStream::Open(width, height, logLevel, busy);
    if (!busy)
      m_openFailed = !stream;

    std::lock_guard lock(m_lock);
    if (busy && !m_busy)
      CLog::Log(LOGWARNING, "CAMLCapture::{}: another vdin1 user holds the node", __FUNCTION__);
    m_busy = busy;
    if (!stream)
      return false;
    // a stream opened across a mode set still holds the old mode
    if (!CanRead() || epoch != m_displayEpoch)
      return false;
    m_stream = stream;
  }

  const float periodMs = stream->GetFramePeriodMs();
  const int waitMs = static_cast<int>(std::ceil(FRAME_WAIT_PERIODS * periodMs));
  const std::chrono::milliseconds maxAge(static_cast<int>((stream->GetBuffers() - 2) * periodMs));
  unsigned int index = 0;
  const CStream::Wake wake =
      stream->TakeNewest(maxAge, index) ? CStream::Wake::FRAME : stream->Wait(waitMs, index);
  if (wake == CStream::Wake::FRAME)
  {
    AMLCaptureFrame frame;
    frame.color = GetSignal(source);
    frame.image = stream->Image(index);
    const bool ok = use(*stream, index, frame);
    stream->Queue(index);
    return ok;
  }
  // vdin1 stopped the stream without the reader, so the next read opens a new one
  if (wake == CStream::Wake::LOST)
  {
    {
      std::lock_guard lock(m_lock);
      if (m_stream == stream)
        stale = TakeStream();
    }
    stream.reset();
    Close(std::move(stale));
    CLog::Log(LOGWARNING, "CAMLCapture::{}: vdin1 stopped the stream", __FUNCTION__);
    return false;
  }
  // vdin1 gives no frames while the output is muted, so a stream stays open through silence
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
  if (!display)
    return color;

  bool sourcePQ = source.transfer == AVCOL_TRC_SMPTE2084;
  bool sourceHLG = source.transfer == AVCOL_TRC_ARIB_STD_B67;
  // hdr2sdr and sdr2hdr rewrite the stream colour to the output colour; the demuxer keeps the type
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
  // vdin1 sees the core3 input in every mode
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
          mapped = true;
        }
        else
          color.signal = AMLCaptureSignal::DV_TV_LED;
        break;
      case DvMode::HDR10:
        color.signal = AMLCaptureSignal::DV_HDR10;
        mapped = true;
        break;
      case DvMode::SDR10:
      case DvMode::SDR8:
        color.signal = AMLCaptureSignal::DV_SDR;
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
        break;
      case HdrOutput::HLG:
      case HdrOutput::CUVA_HLG:
        color.signal = AMLCaptureSignal::HLG;
        break;
      default:
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
  crop.data[0] += y1 * image.linesize[0] + x1;
  crop.data[1] += y1 / 2 * image.linesize[1] + x1;
  return crop;
}

void CAMLCapture::CreateVideoPixels(const AMLCaptureVideoSpec& spec, CaptureResult& result)
{
  int width = 0;
  int height = 0;
  GetStreamSize(spec.rect, spec.width, spec.height, width, height);
  auto lease = spec.continuous ? Lease(width, height) : Lease();
  result.pixels =
      std::make_shared<CAMLVideoPixels>(weak_from_this(), std::move(lease), spec, width, height);
  DescribeSdr(spec.width, spec.height, result);
  result.content = CaptureContent::VIDEO;
}
