/*
 *  Copyright (C) 2026 Team CoreELEC
 *
 *  SPDX-License-Identifier: GPL-2.0-or-later
 *  See LICENSES/README.md for more information.
 */

#pragma once

#include "guilib/DispResource.h"
#include "rendering/capture/CaptureTypes.h"
#include "utils/AMLCaptureConvert.h"
#include "utils/Geometry.h"

#include <atomic>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <utility>
#include <vector>

struct AMLFrameMetadata;

//! What the renderer knows about the playing video, and the title peak of the frame on screen
struct AMLCaptureSource
{
  bool dolbyVision{false};
  AVColorTransferCharacteristic transfer{AVCOL_TRC_UNSPECIFIED};
  //! in cd/m2 from the HDR statics, 0 without them
  float titlePeak{0.0f};
};

//! The vdin loopback inputs: the video layer, and the screen as shown
enum class AMLCaptureInput
{
  VIDEO,
  SCREEN,
};

enum class AMLCaptureRead
{
  FRAME,
  //! the output shows no picture for now: its mode changes, or the input is blank
  NO_PICTURE,
  ENDED,
};

//! A vdin frame, valid only inside CAMLCapture::Read
struct AMLCaptureFrame
{
  AMLCaptureImage image;
  AMLCaptureColor color;
};

//! A 10-bit vdin frame copied out of its stream
struct AMLCaptureShotFrame
{
  //! frame.image points into data
  std::unique_ptr<uint8_t[]> data;
  AMLCaptureFrame frame;
  //! the shot's rows, more than the image holds when it is a field of an interlaced output
  int height{0};
};

//! What only the read of a native screenshot knows about its pixels
struct AMLCaptureNative
{
  bool resolved{false};
  AMLCaptureColor color;
  int width{0};
  int height{0};

  //! once resolved, width x height RGB48BE pixels in the signal's own coding
  bool Describe(KODI::RENDERING::CAPTURE::CaptureResult& result) const;
};

struct AMLCaptureVideoSpec
{
  int width{0};
  int height{0};
  //! the visible video, normalized to the output; vdin holds it at the frame origin
  CRect rect;
  bool continuous{false};
  KODI::RENDERING::CAPTURE::CaptureScreenshot shot{
      KODI::RENDERING::CAPTURE::CaptureScreenshot::NONE};
};

//! \brief The process-wide capture session on the vdin V4L2 nodes: video after the VPP and the
//! DV and HDR cores, without the GUI, or the post blend output, video and GUI as on screen.
//!
//! The renderer or a screen capture owns the session. A consumer that reads keeps the engine, a
//! vdin node, it has. A new one joins a stream that carries its input at its size, else takes a
//! free engine, else grows a stream that carries its input, else adds its input to the stream
//! with the fewest, whose inputs then take turns, one frame each, or on a node without turns
//! restart it for each reader.
//! An engine's stream starts on the first read and stops when no consumer is placed on it,
//! on OnLostDisplay, when vdin stops it and when the session is dropped. Only OnLostDisplay and
//! the destructor make ioctls on the calling thread; everything else runs on the consumer thread.
class CAMLCapture : public IDispResource, public std::enable_shared_from_this<CAMLCapture>
{
public:
  using ReadFunc = std::function<bool(const AMLCaptureFrame& frame, CAMLCaptureConvert& convert)>;

  //! the session, created when no renderer or screen capture holds one
  static std::shared_ptr<CAMLCapture> Get();
  //! the largest capture size for the current output mode
  static void GetCaptureSize(int& width, int& height);
  //! the size a shot of type `shot` reads; call on a thread that may read the resolutions
  static void GetShotSize(KODI::RENDERING::CAPTURE::CaptureScreenshot shot,
                          int& width,
                          int& height);

  CAMLCapture();
  ~CAMLCapture() override;

  void OnLostDisplay() override;
  void OnResetDisplay() override;

  void Configure(const AMLCaptureSource& source);
  void SetPresented();
  //! the renderer releases the session: no video is presented, the source is gone and the video
  //! consumers lapse
  void ClearPresented();
  bool IsPresented() const;
  //! Render thread: the video destination in GUI coordinates
  void SetVideoRect(const CRect& rect);
  //! Render thread: the video destination on screen, normalized to the output
  CRect GetVisibleRect();

  //! Consumer thread: take the newest frame of input from a stream at least width x height, or of
  //! the size another reader of its engine reads at when larger, else wait for the next one, and
  //! run `use` on it. A wait ends after a bound of frame periods, sooner when the input or the
  //! output mode shows no picture.
  AMLCaptureRead Read(AMLCaptureInput input, int width, int height, const ReadFunc& use);
  //! Consumer thread: copy out a frame of a one-shot 10-bit video stream of the size asked for;
  //! vdin settles what it delivers. The stream closes before this returns; a continuous reader
  //! reopens its own.
  bool ReadShot(KODI::RENDERING::CAPTURE::CaptureScreenshot shot,
                int width,
                int height,
                AMLCaptureShotFrame& frame);
  //! Consumer thread: run `use` on the frame of a shot, else, or when that fails, on a Read of
  //! the video at width x height.
  bool ReadVideo(KODI::RENDERING::CAPTURE::CaptureScreenshot shot,
                 int width,
                 int height,
                 const ReadFunc& use);

  //! The crop of an NV12 or full pack image to a normalized rect, on even pixels for the chroma
  static AMLCaptureImage Crop(const AMLCaptureImage& image, const CRect& rect);

  //! Deferred sRGB pixels of the visible video in result: BGRA, RGB24 for a STANDARD shot; a
  //! NATIVE shot describes its own.
  void CreateVideoPixels(const AMLCaptureVideoSpec& spec,
                         KODI::RENDERING::CAPTURE::CaptureResult& result);
  //! Render thread: the GUI readback in result with the presented video blended under it on
  //! Lock, as sRGB pixels of its size: RGBA, RGB24 for a STANDARD shot; a NATIVE shot describes
  //! its own.
  void CreateCompositePixels(KODI::RENDERING::CAPTURE::CaptureResult& result);

  //! Render thread: continuous sRGB BGRA pixels of the screen in result, read at each Lock, so they
  //! outlast a stretch with no drawn frame; live pixels in result stay, at the stream size of the
  //! output mode. No pixels without a vdin screen input.
  static void CreateLivePixels(int width,
                               int height,
                               KODI::RENDERING::CAPTURE::CaptureResult& result);

private:
  class CStream;
  using StreamFunc =
      std::function<bool(CStream& stream, unsigned int index, const AMLCaptureFrame& frame)>;

  struct ConsumerConvert
  {
    std::mutex lock;
    CAMLCaptureConvert convert;
  };

  //! the leases of one input and size
  struct Consumer
  {
    AMLCaptureInput input{AMLCaptureInput::VIDEO};
    int width{0};
    int height{0};
    int leases{0};
    //! kept while the leases last, so a consumer converts beside the others
    std::shared_ptr<ConsumerConvert> convert;
    bool read{false};
    //! the engine placement gave it, -1 for none
    int engine{-1};
    //! width x height as vdin bounds it on the stream `bounded`
    std::weak_ptr<CStream> bounded{};
    int boundWidth{0};
    int boundHeight{0};
  };

  //! what an engine's stream carries
  struct Plan
  {
    uint32_t inputs{0};
    int width{0};
    int height{0};
    int consumers{0};
  };

  //! a vdin node; the stream, busy, owned and unscheduled are guarded by m_lock
  struct Engine
  {
    std::shared_ptr<CStream> stream;
    //! serializes the opens and closes of its readers; a shot holds it across its stream, never
    //! taken by Stop or by a frame's return
    std::mutex open;
    //! the inputs an open of the node was refused for: another user holds it, or no write-back
    //! channel serves them
    uint32_t busy{0};
    //! busy holds both inputs because another user owned the node
    bool owned{false};
    //! the node refused turns, so placement adds an input to its stream only when no other engine
    //! can carry it
    bool unscheduled{false};
    //! guarded by `open`: the last open failed on a node that was not busy, so open failures
    //! log at debug until one succeeds
    bool openFailed{false};
  };

  //! Keeps a stream of input at least width x height running until the last lease is released.
  //! A continuous consumer holds one from its tap on, every read for its own duration.
  std::shared_ptr<void> Lease(AMLCaptureInput input, int width, int height);
  void ReleaseLease(AMLCaptureInput input, int width, int height);
  //! closes in a job the streams placement leaves without a consumer
  void CloseIdle();
  //! `open` holds the open lock of `engine`, which an NV12 read releases once its stream runs or
  //! while its buffers grow; `use` runs on buffer `index`, which is returned after it. `refused`
  //! tells that a refused open changed the placement, so the reader reads again. With `logCause`,
  //! it logs why it gets no frame
  AMLCaptureRead ReadStream(std::unique_lock<std::mutex>& open,
                            size_t engine,
                            AMLCaptureInput input,
                            uint32_t pixelFormat,
                            int width,
                            int height,
                            const StreamFunc& use,
                            bool& refused,
                            bool logCause = false);
  //! m_lock must be held
  bool CanRead(AMLCaptureInput input) const;
  //! m_lock must be held; no picture while the display is lost, else ended, after logging why the
  //! video cannot be read when `log`
  AMLCaptureRead Unreadable(bool log) const;
  //! takes the stream of `engine` for Close; m_lock must be held, and the caller drops the stream
  //! after it
  std::shared_ptr<CStream> TakeStream(size_t engine);
  void Close(const std::shared_ptr<CStream>& stream);
  //! m_lock must be held
  std::vector<Consumer>::iterator FindConsumer(AMLCaptureInput input, int width, int height);
  //! m_lock must be held: the engine runs a stream `plan` gives no consumer
  bool IsIdle(const Engine& engine, const Plan& plan) const;
  //! m_lock must be held: the inputs of the consumers that read
  uint32_t GetReadInputs() const;
  //! m_lock must be held
  void ClearBusy();
  //! m_lock must be held: places the consumers that read without an engine, in lease order, and
  //! returns per engine what its stream carries
  std::vector<Plan> Place();
  AMLCaptureColor GetSignal(const AMLCaptureSource& source) const;
  //! m_lock must be held
  void UpdateStatics(std::shared_ptr<const AMLFrameMetadata> meta);

  std::mutex m_lock;
  std::vector<Engine> m_engines;
  //! taken streams not gone yet, which hold their node and write-back channels until then
  int m_closing{0};
  //! streams whose inputs left the turns, which vdin holds the channels of until a frame of the
  //! new turns
  int m_settling{0};
  //! bumped when one of our streams is stored or taken; an open beside such a change may be
  //! refused for it
  uint32_t m_streamChanges{0};
  std::condition_variable m_closed;
  std::vector<Consumer> m_consumers;
  bool m_displayLost{false};
  std::condition_variable m_displayReset;
  uint32_t m_displayEpoch{0};
  float m_framePeriodMs{0.0f};
  AMLCaptureSource m_source;
  //! the frame metadata the title peak in m_source was decoded from
  std::shared_ptr<const AMLFrameMetadata> m_staticsMeta;
  CRect m_videoRect;
  std::atomic<bool> m_presented{false};
};
