/*
 *  Copyright (C) 2026 Team CoreELEC
 *
 *  SPDX-License-Identifier: GPL-2.0-or-later
 *  See LICENSES/README.md for more information.
 */

#pragma once

#include "rendering/gles/ScreenshotSurfaceGLES.h"

#include <memory>

//! \brief The GLES GUI readback, and a scaled continuous composite from the vdin screen input.
class CScreenshotSurfaceAML : public CScreenshotSurfaceGLES
{
public:
  static void Register();
  static std::unique_ptr<IScreenshotSurface> CreateSurface();

  //! a scaled continuous composite comes from vdin, or fails
  bool CaptureDirect(const KODI::RENDERING::CAPTURE::CaptureSpec& spec,
                     KODI::RENDERING::CAPTURE::CaptureResult& result) override;
};
