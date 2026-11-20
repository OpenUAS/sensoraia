/*
 * Copyright 2026 OpenUAS
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef LWIR_RENDER_H
#define LWIR_RENDER_H

/** @file
 * @brief Display rendering of Tiny1-C frames into JPEG-ready Y/Cb/Cr scanlines.
 * @details Rendering is strictly a display concern: it reads an immutable combined frame
 * (see lwir_frame.h) and never reorders, rewrites or calibrates the stored raw thermal
 * bytes, so measurements and saved data stay in native coordinates.
 *
 * Workflow per frame: lwir_render_prepare() validates options and resolves one display
 * window (struct lwir_render_range); lwir_render_dimensions() gives the oriented size;
 * lwir_render_row() is then called once per display row to fill a caller-owned buffer
 * of `display_width * 3` bytes, so no full RGB or rotated frame is ever materialized.
 *
 * Views: LWIR_SENSOR passes the camera's own YUYV image through; LWIR_FIXED maps an
 * operator raw window; LWIR_PERCENTILE derives the window from the frame's histogram.
 * Thermal views map the window through a palette, flag below/above/invalid samples with
 * distinct colours, and may draw a legend strip. An optional measured correction profile
 * applies scale/offset, a support range and bad-pixel replacement for display only.
 *
 * Orientation is applied as clockwise rotation, then mirror, then flip; rows are produced
 * by one inverse transform per scanline and a signed native stride.
 *
 * SIMD: NEON (ARM64) and SSSE3 (x86-64, runtime-detected) accelerate +/-1-stride rows and
 * produce the same bytes as the scalar code, which handles every remaining case.
 */
#include "lwir_frame.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Source of display intensity. */
enum lwir_view {
    LWIR_SENSOR,     /**< Camera's own YUYV image; no thermal window, profile or denoise. */
    LWIR_FIXED,      /**< Fixed raw Kelvin*64 window from lwir_render_options low/high. */
    LWIR_PERCENTILE  /**< Window from per-frame low/high percentiles of valid samples. */
};
/** @brief Thermal palette; ignored by LWIR_SENSOR. */
enum lwir_palette {
    LWIR_WHITE_HOT,  /**< Greyscale, warmer is brighter. */
    LWIR_BLACK_HOT,  /**< Greyscale, warmer is darker. */
    LWIR_IRON        /**< Six-anchor black-purple-red-orange-yellow ramp (scalar path only). */
};

/** @brief Capacity of the sorted bad-pixel list in struct lwir_profile. */
#define LWIR_MAX_BAD_PIXELS 128U
/** @brief Measured display-correction profile for one camera, gain label and geometry.
 * @details Applied only when rendering; raw frames and measurements are unaffected.
 * Corrected value = round(raw * scale_ppm / 1e6) + offset_mk * 64 / 1000 (truncated),
 * in the same Kelvin*64 units as raw samples.
 */
struct lwir_profile {
    char serial[64];      /**< Device serial (or explicit replay identity) the profile applies to. */
    char gain[32];        /**< Operator gain label the profile was measured with. */
    char sha256[65];      /**< Hex digest text identifying the profile source (63 chars + NUL). */
    unsigned width;       /**< Native width the bad-pixel indices refer to. */
    unsigned height;      /**< Native height the bad-pixel indices refer to. */
    unsigned scale_ppm;   /**< Gain in parts per million; renderer accepts 500000..1500000. */
    int offset_mk;        /**< Offset in millikelvin; renderer accepts -50000..50000. */
    uint16_t range_min;   /**< Lowest supported raw code (non-zero); smaller samples render invalid. */
    uint16_t range_max;   /**< Highest supported raw code (< 65535); larger samples render invalid. */
    unsigned bad_count;   /**< Used entries of bad_pixels. */
    /** Native raster indices (row * width + column), strictly ascending for binary search. */
    uint32_t bad_pixels[LWIR_MAX_BAD_PIXELS];
};

/** @brief Load bounded measured display corrections, never vendor factory tables.
 * @param path Strict profile file.
 * @param profile Caller-owned output, valid on success only.
 * @return True for a valid profile; physical support still requires measurement.
 */
bool lwir_profile_load(const char *path, struct lwir_profile *profile);
/** @brief Require native geometry, serial and operator gain-label agreement.
 * @param profile Measured corrections.
 * @param width Native width.
 * @param height Native height.
 * @param serial Device identity or explicit replay identity.
 * @param gain Operator gain label, not an exposure-atomic gain measurement.
 * @return True for exact support matching; no silently broadened calibration.
 */
bool lwir_profile_matches(const struct lwir_profile *profile, unsigned width, unsigned height,
                            const char *serial, const char *gain);

/** @brief Display policy for one rendering request; never affects stored samples. */
struct lwir_render_options {
    enum lwir_view view;
    enum lwir_palette palette;
    /** LWIR_FIXED window in corrected Kelvin*64 units; requires 0 <= low < high <= 65535. */
    int32_t low;
    int32_t high;
    /** LWIR_PERCENTILE bounds, 0..100 with low_percentile < high_percentile. */
    unsigned low_percentile;
    unsigned high_percentile;
    /** Draw a 16-row gradient and degC label strip; thermal views, display >= 112x24 only. */
    bool legend;
    /* Display only: clockwise rotation, then left/right mirror, then up/down flip. */
    unsigned rotation;
    bool mirror;
    bool flip;
    /** Edge-preserving 3x3 mean threshold in raw units (0 disables, max 640 = 10 K). */
    unsigned denoise_raw;
    /** JPEG quality 1..100; zero preserves the library's quality-100 default. */
    unsigned jpeg_quality;
    /** Optional measured correction, borrowed; must match the native geometry. */
    const struct lwir_profile *profile;
};

/** @brief Resolved display window and diagnostic counts from lwir_render_prepare(). */
struct lwir_render_range {
    int32_t low;     /**< Window start in corrected Kelvin*64 units (0 for LWIR_SENSOR). */
    int32_t high;    /**< Window end, always > low (255 for LWIR_SENSOR). */
    size_t below;    /**< Valid samples below the window (rendered as the below flag colour). */
    size_t above;    /**< Valid samples above the window (rendered as the above flag colour). */
    size_t invalid;  /**< Endpoint, bad-without-neighbours or out-of-profile-support samples. */
};

/** @brief Resolve one deterministic display window while retaining original samples.
 * @details LWIR_SENSOR returns {0, 255} without reading samples. LWIR_FIXED copies the
 * requested window. LWIR_PERCENTILE ranks valid display samples in a 65536-bin histogram.
 * All thermal views then count invalid/below/above samples for diagnostics.
 * @param frame Immutable combined frame, or image-only YUYV for LWIR_SENSOR.
 * @param bytes Exact size.
 * @param width Native width.
 * @param height Native height.
 * @param options Display-only policy.
 * @param range Resolved bounds/clipping counts, valid only on success.
 * @return True for a supported rendering request. On failure errno is EINVAL for invalid
 * arguments/options/profile/legend geometry, or ENODATA when a percentile view finds no
 * valid sample.
 * @warning Uses a static histogram: not re-entrant or thread-safe; call serially.
 */
bool lwir_render_prepare(const uint8_t *frame, size_t bytes, unsigned width, unsigned height,
                           const struct lwir_render_options *options, struct lwir_render_range *range);
/** @brief Derive display dimensions; quarter-turns do not change thermal geometry.
 * @param width Native width.
 * @param height Native height.
 * @param options Rotation/mirror/flip policy.
 * @param display_width Output width.
 * @param display_height Output height.
 * @return True for supported native geometry and orientation.
 */
bool lwir_render_dimensions(unsigned width, unsigned height, const struct lwir_render_options *options,
                              unsigned *display_width, unsigned *display_height);
/** @brief Invert display orientation before accessing calibration or native samples.
 * @param width Native width.
 * @param height Native height.
 * @param options Rotate, then mirror, then flip policy.
 * @param display_x Display column.
 * @param display_y Display row.
 * @param native_x Native column, written on success.
 * @param native_y Native row, written on success.
 * @return True when both coordinate spaces are in bounds.
 */
bool lwir_render_native_coordinate(unsigned width, unsigned height, const struct lwir_render_options *options,
                                     unsigned display_x, unsigned display_y, unsigned *native_x, unsigned *native_y);
/** @brief Emit one bounded Y/Cb/Cr scanline, without a full RGB or rotated frame.
 * @param frame Immutable combined frame, or image-only YUYV for LWIR_SENSOR.
 * @param bytes Exact size.
 * @param width Native width.
 * @param height Native height.
 * @param row Display row.
 * @param options Display policy.
 * @param range Window resolved by lwir_render_prepare.
 * @param output Caller-owned interleaved Y/Cb/Cr row.
 * @param capacity Available bytes, at least display width * 3.
 * @return True on success; scalar and SIMD paths preserve the same mapping. On failure
 * errno is EINVAL (invalid arguments, row, capacity, range, profile or legend geometry)
 * and the output is unspecified.
 */
bool lwir_render_row(const uint8_t *frame, size_t bytes, unsigned width, unsigned height, unsigned row,
                       const struct lwir_render_options *options, const struct lwir_render_range *range,
                       uint8_t *output, size_t capacity);

#if defined(__x86_64__)
/** @brief SSSE3 helper for LWIR_SENSOR rows; internal to lwir_render.c, defined in lwir_ssse3.c.
 * @param frame Image plane start.
 * @param first Native pixel index of display column 0.
 * @param step +1 or -1 native stride.
 * @param width Display row width.
 * @param output Y/Cb/Cr destination of at least width * 3 bytes.
 * @return Columns written (multiple of 8); the caller finishes the tail.
 * @warning Call only after confirming SSSE3 support at runtime.
 */
unsigned lwir_sensor_ssse3(const uint8_t *frame, ptrdiff_t first, ptrdiff_t step,
                          unsigned width, uint8_t *output);
#endif

#ifdef __cplusplus
}
#endif
#endif
