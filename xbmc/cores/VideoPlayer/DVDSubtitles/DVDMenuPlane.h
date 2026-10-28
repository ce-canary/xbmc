/*
 *  Copyright (C) 2026 Team CoreELEC
 *
 *  SPDX-License-Identifier: GPL-2.0-or-later
 *  See LICENSES/README.md for more information.
 */

#pragma once

#include "threads/CriticalSection.h"

#include <atomic>
#include <cstdint>
#include <memory>

class CDVDOverlay;

//! The Blu-ray menu page on screen. libbluray replaces it when the page is due, the renderer
//! draws it on every GUI frame.
class CDVDMenuPlane
{
public:
  void Set(std::shared_ptr<CDVDOverlay> page);
  void Clear();

  //! \return the page, and in \p sequence a number that changes with every Set or Clear
  std::shared_ptr<CDVDOverlay> Get(uint64_t& sequence) const;
  uint64_t GetSequence() const { return m_sequence; }

private:
  mutable CCriticalSection m_section;
  std::shared_ptr<CDVDOverlay> m_page;
  std::atomic<uint64_t> m_sequence{0};
};
