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

#ifndef SENSORAIA_LWIR_DETECTION_H
#define SENSORAIA_LWIR_DETECTION_H
/** @file
 * @brief Optional thermal candidate detection, independent of pose and image files.
 *
 * @details Compiled only in analysis builds (`LWIR_ENABLE_ANALYSIS=1`). Input is
 * the exact native Kelvin*64 temperature plane embedded in a saved LWIR JPEG,
 * never the compressed display pixels, so thresholds operate on sensor values.
 * The detector does no I/O, uses no heap, and keeps all scratch storage in a
 * caller-provided lwir_detection_workspace. lwir_analysis.c is the in-tree caller.
 */
#include "lwir_frame.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief A thermal component expressed only in native sensor coordinates.
 * @details Temperatures are apparent device values, not corrected surface values.
 * Tray classification is a mission heuristic, not a certified object identity.
 */
struct lwir_hotspot {
    size_t pixels;
    size_t min_x, min_y, max_x, max_y;
    size_t peak_x, peak_y;
    double center_x, center_y;
    double peak_celsius, mean_celsius;
    double local_background_celsius, contrast_celsius, excess_celsius_pixels;
    bool touches_edge, tray_like;
    double fill_ratio, aspect_ratio;
};

/** @var lwir_hotspot::pixels
 * @brief 8-connected pixel count at or above the frame threshold.
 */
/** @var lwir_hotspot::min_x
 * @brief Inclusive native bounding box; likewise min_y, max_x and max_y.
 */
/** @var lwir_hotspot::peak_x
 * @brief Column of the first pixel (breadth-first order) holding the peak code; peak_y is its row.
 */
/** @var lwir_hotspot::center_x
 * @brief Subpixel centroid column weighted by excess over local background; center_y likewise.
 */
/** @var lwir_hotspot::mean_celsius
 * @brief Unweighted mean over component pixels.
 */
/** @var lwir_hotspot::local_background_celsius
 * @brief Median of sub-threshold pixels within a 5-pixel margin around the box.
 * @details Falls back to the frame median when the margin has no such pixels.
 */
/** @var lwir_hotspot::contrast_celsius
 * @brief Peak minus local background; components below 15 C are discarded.
 */
/** @var lwir_hotspot::excess_celsius_pixels
 * @brief Sum of per-pixel excess over local background, an integrated heat score.
 */
/** @var lwir_hotspot::touches_edge
 * @brief Bounding box reaches a frame border, so the object may be truncated.
 */
/** @var lwir_hotspot::tray_like
 * @brief Passed the compact, hot, high-contrast fire-tray heuristic.
 */
/** @var lwir_hotspot::fill_ratio
 * @brief Pixel count divided by bounding-box area.
 */
/** @var lwir_hotspot::aspect_ratio
 * @brief Longer bounding-box side divided by the shorter side (at least 1).
 */

/** @brief Bounded ranking, retaining the existing two-candidate mission policy.
 * @details When any tray-like component exists, `candidates` holds the best
 * tray-like ones (ranked by excess heat) and `tray_pass_selected` is true;
 * otherwise it holds the best generic components (ranked by contrast).
 */
struct lwir_hotspot_set {
    struct lwir_hotspot candidates[2];
    size_t count, generic_count, tray_count;
    bool tray_pass_selected;
    double background_celsius, threshold_celsius;
};

/** @var lwir_hotspot_set::candidates
 * @brief Best first; only the first `count` entries are valid.
 */
/** @var lwir_hotspot_set::count
 * @brief Valid entries in `candidates` (0..2).
 */
/** @var lwir_hotspot_set::generic_count
 * @brief All components that passed the contrast check, not only retained ones.
 */
/** @var lwir_hotspot_set::tray_count
 * @brief Subset of generic_count classified tray-like.
 */
/** @var lwir_hotspot_set::tray_pass_selected
 * @brief True when `candidates` came from the tray ranking.
 */
/** @var lwir_hotspot_set::background_celsius
 * @brief Median of valid (non-endpoint) codes; NaN when the plane has none.
 */
/** @var lwir_hotspot_set::threshold_celsius
 * @brief Frame threshold actually applied; NaN when the plane has no valid codes.
 */

/** @brief Caller-owned scratch storage; never reserved by an imagery-only build.
 * @details Use static or otherwise long-lived storage, not a small thread stack.
 * No initialization is needed. Separate workspaces permit independent detectors.
 * Size is roughly 4.1 bytes per maximum native pixel plus 256 KiB of histogram.
 */
struct lwir_detection_workspace {
    uint8_t seen[(LWIR_MAX_WIDTH * LWIR_MAX_HEIGHT + 7) / 8]; ///< One visited bit per pixel.
    uint32_t queue[LWIR_MAX_WIDTH * LWIR_MAX_HEIGHT]; ///< BFS queue; afterwards lists the component's pixels.
    uint32_t histogram[UINT16_MAX + 1U]; ///< One bin per raw code, used for exact medians.
};

/** @brief Rank native thermal components without touching hardware, pose or files.
 * @param temperature Immutable little-endian Kelvin*64 plane.
 * @param bytes Exact accessible plane length, width * height * 2.
 * @param width Native width, at most LWIR_MAX_WIDTH.
 * @param height Native height, at most LWIR_MAX_HEIGHT.
 * @param workspace Exclusive scratch storage for this call, separate from input/result.
 * @param result Bounded candidates; ignore on failure.
 * @return True for a valid plane, including a plane with no candidates or no
 * valid codes at all (then background_celsius and threshold_celsius are NaN).
 * False for NULL pointers, zero or oversized dimensions, or a length mismatch.
 * @details Threshold = max(60 C absolute, frame median + max(20 C, 6 robust
 * sigma)), where robust sigma is 1.4826 x the median absolute deviation. Pixels
 * at or above it form 8-connected components; each must exceed its local
 * background by at least 15 C. See lwir_detection.c for the full algorithm.
 * Endpoint codes 0 and 65535 are invalid samples: they are excluded from the
 * frame median, noise estimate, components and local backgrounds.
 * @note Temperatures are apparent sensor values without emissivity or
 * atmospheric correction; the 60/100 C limits apply to those values.
 */
bool lwir_detect_hotspots(const uint8_t *temperature, size_t bytes, unsigned width, unsigned height,
                          struct lwir_detection_workspace *workspace, struct lwir_hotspot_set *result);

#ifdef __cplusplus
}
#endif
#endif