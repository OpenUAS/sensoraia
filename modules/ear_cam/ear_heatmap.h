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

/**
 * @file ear_heatmap.h
 * @brief Render geotagged acoustic measurements into north-up JPEG heatmaps. */
#ifndef EARCAM_HEATMAP_H
#define EARCAM_HEATMAP_H

#include <stdbool.h>
#include <stddef.h>

#include "earcam_core.h"

#define EAR_HEATMAP_NO_DATA_RED 18U /**< Red channel identifying uncovered field pixels. */
#define EAR_HEATMAP_NO_DATA_GREEN 18U /**< Green channel identifying uncovered field pixels. */
#define EAR_HEATMAP_NO_DATA_BLUE 24U /**< Blue channel identifying uncovered field pixels. */

/**
 * @brief Validated geographic placement metadata shared by heatmap consumers.
 * @details The renderer uses a local equirectangular frame about the center; overlay
 * code must transform to its map projection rather than stretch the JPEG by size.
 */
struct ear_heatmap_georef {
    unsigned int width_px; /**< Encoded heatmap width; must match the JPEG consumer reads. */
    unsigned int height_px; /**< Encoded heatmap height. */
    double m_per_px; /**< Ground spacing represented by each heatmap pixel. */
    double lat_center; /**< Latitude origin for the local projection, degrees. */
    double lon_center; /**< Longitude origin for the local projection, degrees. */
    double lat_north; /**< Latitude at the upper image edge, degrees. */
    double lon_west; /**< Longitude at the left image edge, degrees. */
    bool has_spot; /**< True only when the sidecar contains a complete spot pair. */
    double spot_lat; /**< Estimated spot latitude, degrees, when `has_spot`. */
    double spot_lon; /**< Estimated spot longitude, degrees, when `has_spot`. */
};

/**
 * @brief Read the georeference sidecar emitted by the heatmap renderer.
 * @param filename Path to the `.geo` sidecar.
 * @param georef Receives validated dimensions, projection and optional spot.
 * @return 0 on success; -1 for invalid arguments, malformed data or I/O error.
 * @details Required dimensions, scale and edge/origin coordinates are validated
 * before publication to the caller; partial parses never replace its struct.
 * Duplicate fields, oversized lines and incomplete optional spot pairs fail.
 */
int ear_heatmap_read_georef(const char *filename, struct ear_heatmap_georef *georef);

/**
 * @brief Write a decorated north-up JPEG of relative measured acoustic intensity.
 * @param filename Output path; an adjacent .geo sidecar is attempted after JPEG success.
 * @param samples Borrowed, immutable geotagged windows; not retained after the call.
 * @param count Window count; at least three usable finite, unclipped windows are required.
 * @param spot Optional borrowed estimate; only valid estimates receive a marker.
 * @return 0 for a successfully written JPEG, -1 for insufficient data or file/codec failure.
 * @details Track dots, an estimated-source marker and a scale bar aid interpretation.
 * Calls serialize on bounded shared scratch storage. Grid cells are roughly AGL/4;
 * bilinear upsampling adds visual smoothness, not measurement resolution.
 * @warning Sidecar write failure does not change the return status. Neither the
 * smoothed field nor the marker demonstrates localization accuracy or alarm presence.
 */
int ear_heatmap_write(const char *filename, const struct ear_sample *samples, size_t count,
                      const struct ear_loudest_spot *spot);

/**
 * @brief Write the same field without track dots, source marker or scale bar.
 * @param filename JPEG path; a best-effort .geo sidecar supplies overlay placement.
 * @param samples Borrowed immutable acoustic windows.
 * @param count Number of supplied windows, with the same usable-data gate as ear_heatmap_write().
 * @param spot Optional estimate retained in sidecar metadata, not drawn into pixels.
 * @return 0 for JPEG success, -1 for insufficient usable data or file/codec failure.
 * @details Use the same interpolation and shared workspace as the decorated
 * renderer, keeping annotation pixels out of the field used for overlays.
 * @see ear_heatmap_write for locking, resolution and sidecar-status caveats.
 */
int ear_heatmap_write_field(const char *filename, const struct ear_sample *samples, size_t count,
                            const struct ear_loudest_spot *spot);

#endif
