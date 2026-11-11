/*
 *  Copyright (C) 2011-2018 Team Kodi
 *  This file is part of Kodi - https://kodi.tv
 *
 *  SPDX-License-Identifier: GPL-2.0-or-later
 *  See LICENSES/README.md for more information.
 */

#pragma once

#include "utils/StreamDetails.h"

#include <chrono>
#include <optional>

extern "C"
{
#include <libavutil/pixfmt.h>
}

class CDVDStreamInfo;

struct bd_clip;
using BLURAY_CLIP_INFO = bd_clip;

enum AML_SUPPORT_H264_4K2K
{
  AML_SUPPORT_H264_4K2K_UNINIT = -1,
  AML_NO_H264_4K2K,
  AML_HAS_H264_4K2K,
  AML_HAS_H264_4K2K_SAME_PROFILE
};

enum AML_DISPLAY_DV_LED
{
  AML_DV_TV_LED = 0,
  AML_DV_PLAYER_LED
};

#define HDR10_PLUS_CAP      (int)(1<<0)
#define HDR10_CAP           (int)(1<<2)
#define SMPTE_ST_2084_CAP   (int)(1<<3)
#define HLG_CAP             (int)(1<<4)

#define DV_2160p60Hz        (int)(1<<2)
#define DV_RGB_444_8BIT     (int)(1<<3)
#define LL_YCbCr_422_12BIT  (int)(1<<5)

#define AML_GXBB    0x1F
#define AML_GXL     0x21
#define AML_GXM     0x22
#define AML_G12A    0x28
#define AML_G12B    0x29
#define AML_SM1     0x2B
#define AML_SC2     0x32
#define AML_T7      0x36
#define AML_S4      0x37
#define AML_S5      0x3E
#define AML_S7      0x46
#define AML_S7D     0x47
#define AML_S6      0x48

int  aml_get_cpufamily_id();
std::string aml_get_cpufamily_name(int cpuid = -1);
bool aml_support_hevc();
bool aml_support_hevc_4k2k();
bool aml_support_hevc_8k4k();
bool aml_support_hevc_10bit();
bool aml_support_h266();
AML_SUPPORT_H264_4K2K aml_support_h264_4k2k();
bool aml_support_vp9();
bool aml_support_av1();
bool aml_support_avs2();
bool aml_support_avs3();
bool aml_support_dolby_vision();
bool aml_dolby_vision_enabled();
bool aml_convert_to_sdr(StreamHdrType hdrType);
bool aml_convert_to_hdr(StreamHdrType hdrType);
bool aml_convert_to_sdr_by_vs_engine(StreamHdrType hdrType);
bool aml_convert_to_hdr_by_vs_engine(StreamHdrType hdrType);
bool aml_convert_to_dv_by_vs_engine(StreamHdrType hdrType);
// the engine every clip of a Blu-ray menu session runs through, the disc's range before a user's
// mapping lowers it; none outside a session or where the clips are mapped to sdr
std::optional<StreamHdrType> aml_disc_output_hdr_type();

// what the decoder puts out for a source under the conversion settings
struct AMLVideoOutput
{
  StreamHdrType hdrType{StreamHdrType::HDR_TYPE_NONE};
  AVColorSpace colorSpace{AVCOL_SPC_UNSPECIFIED};
  AVColorPrimaries colorPrimaries{AVCOL_PRI_UNSPECIFIED};
  AVColorTransferCharacteristic colorTransfer{AVCOL_TRC_UNSPECIFIED};
  float sdrWhite{0.0f}; // nits of sdr white on a pq output, 0 when unknown
  float sdrGamma{0.0f};
};
AMLVideoOutput aml_video_output(const CDVDStreamInfo& source);
bool aml_video_started();
bool aml_display_reset_pending();
// when the last delayed display reset ended, the clock's epoch when none ran
std::chrono::steady_clock::time_point aml_display_reset_end();
// `next` is the clip of the next item, nullptr when unknown; `jump` when the disc program jumped
// to a next it has not named yet
void aml_continue_presentation(const BLURAY_CLIP_INFO* next, bool jump);
// `next` is the stream the next decoder opens with
void aml_continue_presentation(const CDVDStreamInfo& next);
void aml_end_presentation(bool stop);
// `taken` also drops a picture a next decoder took before its first frame
void aml_drop_kept_picture(bool taken);
void aml_set_3d_video_mode(unsigned int mode, bool framepacking_support, int view_mode);
