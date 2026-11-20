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

#ifndef LWIRCAM_LWIR_FRAME_H
#define LWIRCAM_LWIR_FRAME_H

/** @file
 * @brief Native frame arithmetic and camera measurements; no codec or flight dependencies.
 * @details Combined frames contain packed YUY2 followed by LE uint16 Kelvin*64.
 * Display transforms never change these coordinates or original samples.
 *
 * Tiny1-C combined frame layout (width W, height H, W even):
 * - Bytes [0, 2WH): YUYV image plane, `Y0 U Y1 V` per horizontal pixel pair; both pixels
 *   of a pair share U and V.
 * - Bytes [2WH, 4WH): thermal plane, one little-endian uint16 per pixel in raster order.
 *   raw / 64 - 273.15 is degC as reported by the device; calibration is unverified.
 *   Codes 0 and 65535 are endpoint/invalid codes.
 *
 * Invariants: sizes are exact (lwir_frame_bytes()), all coordinates are native and
 * unrotated, inputs are borrowed read-only and nothing is allocated. Live capture qualifies
 * frames with lwir_frame_capture_valid() (and optionally lwir_frame_plausible()/
 * lwir_frame_settled()) before rendering or measuring; replay only needs lwir_frame_valid().
 *
 * Measurements have a scalar reference path; on little-endian ARM64 NEON processes
 * eight-pixel blocks with identical results (exact integer averages, first-occurrence
 * extrema), verified by parity tests.
 */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/** @brief Largest supported native width in pixels. */
#define LWIR_MAX_WIDTH 640U
/** @brief Largest supported native height in pixels. */
#define LWIR_MAX_HEIGHT 512U
/** @brief Largest combined frame: two planes of 2 bytes per pixel. */
#define LWIR_MAX_FRAME_BYTES (LWIR_MAX_WIDTH * LWIR_MAX_HEIGHT * 4U)

/** @brief Buffer content kinds: full image+thermal frame, or the YUYV image plane alone. */
enum lwir_frame_layout { LWIR_COMBINED, LWIR_YUYV_IMAGE };

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Native rectangle; a zero width/height pair selects the full image. */
struct lwir_roi {
    unsigned x;
    unsigned y;
    unsigned width;
    unsigned height;
};

/** @brief Ordinary camera measurements, independent of optional hotspot analysis. */
enum lwir_measurement_kind {
    LWIR_RECTANGLE,  /**< roi rectangle; an all-zero roi means the full image. */
    LWIR_POINT,      /**< Single pixel at roi.x, roi.y; roi width/height ignored. */
    LWIR_LINE        /**< Inclusive line from roi.x, roi.y to end_x, end_y. */
};

/** @brief One native-coordinate measurement request. */
struct lwir_measurement {
    enum lwir_measurement_kind kind;
    /** Rectangle, or the start point for LWIR_POINT/LWIR_LINE. */
    struct lwir_roi roi;
    /** Line end point; unused otherwise. */
    unsigned end_x;
    unsigned end_y;
    /** Replace each valid center by the rounded mean of the valid samples in its
     * edge-clipped 3x3 neighbourhood; coverage still uses the original center. */
    bool average_3x3;
};

/** @brief Optional coverage band counted alongside measurement statistics. */
struct lwir_thresholds {
    /* Inclusive raw Kelvin*64 band, NOT a display window or detector gain range. */
    uint16_t low;
    uint16_t high;
    /** When false the below/within/above counters stay zero. Enabled requires low < high. */
    bool enabled;
};

/** @brief Raw-unit measurement results; derive mean/variance from the sums and pixels. */
struct lwir_temperature_stats {
    /** Extremes of measured samples (after optional averaging), in Kelvin*64. */
    uint16_t min_raw;
    uint16_t max_raw;
    /* Native coordinates; ties keep the first visited sample (raster order for ROI). */
    unsigned min_x;
    unsigned min_y;
    unsigned max_x;
    unsigned max_y;
    /** Sum and sum of squares of measured samples; 64-bit cannot overflow at max geometry. */
    uint64_t sum_raw;
    uint64_t sum_squared_raw;
    /** Number of measured samples (centers visited). */
    size_t pixels;
    /** Samples whose code is 0 or 65535. */
    size_t endpoint_codes;
    /* Endpoints 0/65535 are excluded ONLY from coverage, not raw sums/extrema. */
    size_t below;
    size_t within;
    size_t above;
};


/** @brief Bounded combined-frame size, or zero for unsupported native geometry.
 * @param width Even native width from 2 through LWIR_MAX_WIDTH.
 * @param height Native height from 1 through LWIR_MAX_HEIGHT.
 * @return width * height * 4, or zero.
 */
size_t lwir_frame_bytes(unsigned width, unsigned height);
/** @brief Validate a native rectangle without clipping an operator's request.
 * @param width Native width.
 * @param height Native height.
 * @param roi Requested region.
 * @return True when every requested sample is within the image.
 */
bool lwir_roi_valid(unsigned width, unsigned height, const struct lwir_roi *roi);
/** @brief Validate ROI/point/line geometry before any sample is accessed.
 * @param width Native width.
 * @param height Native height.
 * @param measurement Requested native measurement.
 * @return True when the request is supported and in bounds.
 */
bool lwir_measurement_valid(unsigned width, unsigned height, const struct lwir_measurement *measurement);
/** @brief Measure apparent device temperatures, keeping diagnostic coverage separate.
 * @param frame Immutable combined frame.
 * @param bytes Exact frame size.
 * @param width Native width.
 * @param height Native height.
 * @param measurement ROI/point/line and optional measurement-only averaging.
 * @param thresholds Optional inclusive raw coverage band, not a display window.
 * @param statistics Output; ignore on failure. Endpoint codes remain in raw extrema/sums.
 * @return True on success. No surface correction or object identity is inferred.
 * False (errno untouched) for NULL pointers, invalid geometry or measurement, a size
 * mismatch, or enabled thresholds with low >= high.
 */
bool lwir_measure_temperature(const uint8_t *frame, size_t bytes, unsigned width, unsigned height,
                                const struct lwir_measurement *measurement, const struct lwir_thresholds *thresholds,
                                struct lwir_temperature_stats *statistics);
/** @brief Rectangle convenience interface to lwir_measure_temperature.
 * @param frame Combined frame.
 * @param bytes Exact size.
 * @param width Native width.
 * @param height Native height.
 * @param roi Native region.
 * @param statistics Output, valid only on success.
 * @return True on success.
 */
bool lwir_temperature_statistics(const uint8_t *frame, size_t bytes, unsigned width, unsigned height,
                                   const struct lwir_roi *roi, struct lwir_temperature_stats *statistics);
/** @brief Check pointer, geometry and exact combined layout, not physical accuracy.
 * @param frame Borrowed bytes.
 * @param bytes Accessible size.
 * @param width Native width.
 * @param height Native height.
 * @return True for a structurally valid combined frame.
 */
bool lwir_frame_valid(const uint8_t *frame, size_t bytes, unsigned width, unsigned height);
/** @brief Recognize the all-black YUYV/0x8000 thermal fill observed during camera transitions.
 * @details This whole-frame signature contains no usable sensor information.
 * An isolated 0x8000 temperature or any other uniform scene is not rejected.
 * Every byte pair in BOTH planes must be (0x00, 0x80): Y = 0 with neutral chroma in the
 * image, little-endian 0x8000 in the thermal plane.
 * @param frame Borrowed bytes.
 * @param bytes Accessible size.
 * @param width Native width.
 * @param height Native height.
 * @return True only for a structurally valid frame that is entirely neutral fill.
 */
bool lwir_frame_is_neutral_fill(const uint8_t *frame, size_t bytes, unsigned width, unsigned height);
/** @brief Qualify live combined data, rejecting neutral fill and an entirely invalid thermal plane.
 * @details Structural replay validation remains independent. This is not physical calibration.
 * Legitimate uniform scenes pass as long as one thermal code is neither 0 nor 65535.
 * @param frame Borrowed bytes.
 * @param bytes Accessible size.
 * @param width Native width.
 * @param height Native height.
 * @return True for a structurally valid, non-fill frame with at least one valid temperature.
 */
bool lwir_frame_capture_valid(const uint8_t *frame, size_t bytes, unsigned width, unsigned height);
/** @brief Optional conservative scene check; unusual valid scenes may be rejected.
 * @details Rejects frames where more than 1% of image chroma bytes leave 120..136, the
 * mean absolute horizontal luma step exceeds 45, or more than 10% of every-eighth-pixel
 * thermal samples fall outside raw 14000..40000 (about -54..352 degC).
 * @param frame Borrowed bytes.
 * @param bytes Accessible size.
 * @param width Native width.
 * @param height Native height.
 * @return True for a structurally valid frame that passes all three heuristics.
 */
bool lwir_frame_plausible(const uint8_t *frame, size_t bytes, unsigned width, unsigned height);
/** @brief Optional image-stability heuristic, never exposure or shutter certification.
 * @param frame Current combined frame.
 * @param previous_image Previous image half (width * height * 2 bytes), kept by the acquisition owner.
 * @param width Native width.
 * @param height Native height.
 * @return True when bounded luma change meets the existing settling policy
 *         (mean absolute luma difference <= 12); false for NULL inputs or bad geometry.
 */
bool lwir_frame_settled(const uint8_t *frame, const uint8_t *previous_image,
                         unsigned width, unsigned height);

#ifdef __cplusplus
}
#endif

#endif