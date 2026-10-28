/*
 *  Copyright (C) 2026 Team CoreELEC
 *
 *  SPDX-License-Identifier: GPL-2.0-or-later
 *  See LICENSES/README.md for more information.
 */

#include "DVDMenuPlane.h"

#include "cores/VideoPlayer/DVDCodecs/Overlay/DVDOverlay.h"

#include <mutex>
#include <utility>

void CDVDMenuPlane::Set(std::shared_ptr<CDVDOverlay> page)
{
  std::unique_lock lock(m_section);

  m_page = std::move(page);
  m_sequence++;
}

void CDVDMenuPlane::Clear()
{
  Set(nullptr);
}

std::shared_ptr<CDVDOverlay> CDVDMenuPlane::Get(uint64_t& sequence) const
{
  std::unique_lock lock(m_section);

  sequence = m_sequence;
  return m_page;
}
