/*
 *  Copyright (C) 2026 Team CoreELEC
 *
 *  SPDX-License-Identifier: GPL-2.0-or-later
 *  See LICENSES/README.md for more information.
 */

#include "ScreenshotSurfaceAML.h"

#include "rendering/capture/CaptureTypes.h"
#include "utils/AMLCapture.h"
#include "utils/Screenshot.h"

using namespace KODI::RENDERING::CAPTURE;

void CScreenshotSurfaceAML::Register()
{
  CScreenShot::Register(CScreenshotSurfaceAML::CreateSurface);
}

std::unique_ptr<IScreenshotSurface> CScreenshotSurfaceAML::CreateSurface()
{
  return std::make_unique<CScreenshotSurfaceAML>();
}

bool CScreenshotSurfaceAML::CaptureDirect(const CaptureSpec& spec, CaptureResult& result)
{
  if (spec.content != CaptureContent::COMPOSITE || spec.cadence != CaptureCadence::CONTINUOUS ||
      spec.format != CaptureFormat::BGRA8 || spec.width == 0 || spec.height == 0)
    return false;

  // without the vdin screen input the fallback is a full-size readback on every drawn frame, so
  // the request fails
  CAMLCapture::CreateLivePixels(static_cast<int>(spec.width), static_cast<int>(spec.height),
                                result);
  return true;
}
