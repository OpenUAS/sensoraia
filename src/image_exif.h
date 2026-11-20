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
 * @file image_exif.h
 * @brief Flight, timing, and thermal-analysis metadata writing for captured JPEGs. */
#ifndef USHER_IMAGE_EXIF_H
#define USHER_IMAGE_EXIF_H

#include "usher_protocol.h"
#include "capture_timing.h"
#include "exifite.h"
#include "sensor_capture.h"
#include "../modules/loom_cam/loom_features.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Application-owned opaque evidence retained by photo metadata updates. */
static const uint16_t image_exif_capture_tags[] = {SENSOR_CAPTURE_TAG, LOOM_FEATURES_TAG, LOOM_ONNX_FEATURES_TAG};

/** @brief Select additional image evidence, without teaching EXIFite its schemas.
 * @param context Unused; the application policy needs no mutable state.
 * @param directory Directory containing the field.
 * @param tag Directory-local identifier owned by the producing module.
 * @return True only for the application's relocatable camera evidence records. */
static inline bool image_exif_capture_tag(void *context, enum exif_directory directory, uint16_t tag)
{
    (void)context;
    if (directory != EXIF_CAMERA) return false;
    for (size_t index = 0; index < sizeof(image_exif_capture_tags) / sizeof(image_exif_capture_tags[0]); ++index) {
        if (tag == image_exif_capture_tags[index]) return true;
    }
    return false;
}

/** @brief Import standard metadata plus the application's approved capture evidence.
 * @param document Caller-owned bounded destination.
 * @param view Validated source with nonoverlapping backing storage.
 * @return Zero on success, otherwise -1 with EXIFite's import rollback rules.
 * @details A small inline policy wrapper keeps standalone image tools independent
 * of the flight writer's workspaces. Unknown vendor structures remain excluded. */
static inline int image_exif_import(struct exif_document *document, const struct exif_view *view)
{
    return exif_document_import_custom(document, view, image_exif_capture_tag, NULL);
}

/** @brief Select sibling-file atomic replacement instead of the default in-place update.
 * @details Set to 1 for original-file retention on publication failure. The default
 * 0 avoids temporary files but cannot recover from interrupted block moves or writes.
 * Both methods validate JPEG framing, retain compressed bytes and sync the result.
 * The caller must exclusively own updates to the file and its hard-linked aliases.
 */
#ifndef IMAGE_EXIF_ATOMIC_REPLACE
#define IMAGE_EXIF_ATOMIC_REPLACE 0
#endif
#if IMAGE_EXIF_ATOMIC_REPLACE != 0 && IMAGE_EXIF_ATOMIC_REPLACE != 1
#error "IMAGE_EXIF_ATOMIC_REPLACE must be 0 or 1"
#endif

/** @brief Add standard USHER flight metadata to a JPEG.
 * @param filename Existing JPEG to update using the selected publication method.
 * @param shot Flight-controller shot payload.
 * @return 0 on success or -1 on invalid input, JPEG, metadata, or storage failure.
 * @details Fresh capture writes replace prior camera EXIF with supported USHER
 * metadata while preserving supported raw capture/LOOM evidence. Thumbnail and
 * manufacturer records are deliberately discarded. */
int image_exif_write(const char *filename, const union dc_shot_union *shot);
/** @brief Add flight metadata with optional capture-delay motion compensation.
 * @param filename Existing JPEG to update using the selected publication method.
 * @param shot Flight-controller shot payload.
 * @param capture_delay_s Nonnegative request-to-frame latency, or a negative value when unknown.
 * @param compensate Nonzero to estimate ground displacement from speed/course and latency.
 * @details Original coordinates remain in the textual record so downstream analysis can audit
 * the correction rather than treating the constant-velocity estimate as ground truth. */
int image_exif_write_timed(const char *filename, const union dc_shot_union *shot,
                           double capture_delay_s, int compensate);
/** @brief Append structured thermal hotspot/geolocation evidence to JPEG EXIF.
 * @param filename Existing JPEG.
 * @param information Bounded UTF-8/ASCII analysis record.
 * @return 0 on metadata update success, otherwise -1. */
int image_exif_write_hotspots(const char *filename, const char *information);
/** @brief Append structured object-detection evidence (AICam) to JPEG EXIF.
 * @param filename Existing JPEG.
 * @param information Bounded UTF-8/ASCII analysis record.
 * @return 0 on metadata update success, otherwise -1. */
int image_exif_write_object_detections(const char *filename, const char *information);
/** @brief Label a synthetic map fixture with its geometry and provider-neutral XYZ tile scheme.
 * @param filename Existing mock JPEG.
 * @param horizontal_fov Horizontal field of view in degrees, strictly between 0 and 179.
 * @param vertical_fov Vertical field of view in degrees, strictly between 0 and 179.
 * @return Zero on metadata update success, otherwise -1.
 * @details Writes Sensoraia-owned geometry only; it does not write a tile-provider attribution.
 */
int image_exif_write_mock_map(const char *filename, double horizontal_fov, double vertical_fov);
/** @brief Add flight metadata using authoritative capture timing when available.
 * @param filename Existing JPEG to update using the selected publication method.
 * @param shot Flight-controller shot payload.
 * @param capture_delay_s Fallback latency when @p timing is absent or invalid.
 * @param compensate Nonzero to apply conservative constant-velocity ground correction.
 * @param timing Optional validated server timing evidence; it overrides a supplied delay.
 * @details Timing describes request-to-frame arrival, not sensor exposure time; the emitted
 * metadata labels this limitation explicitly. */
int image_exif_write_capture(const char *filename, const union dc_shot_union *shot,
                             double capture_delay_s, int compensate, const struct capture_timing *timing);
/** @brief Write flight provenance and an available object-detection result in one metadata transaction.
 * @param filename Existing JPEG; the caller exclusively owns metadata updates to this path.
 * @param shot Pose used for geotags, already exposure-aligned when request is supplied.
 * @param delay Fallback request-to-arrival latency in seconds; negative means unknown.
 * @param compensate Nonzero enables the legacy position estimate; use zero for exposure-aligned pose.
 * @param timing Optional arrival evidence and shared bounded observation-time pose.
 * @param request Original FC request to preserve alongside an aligned pose, or NULL.
 * @param uncertainty_m Nonnegative ground-position margin recorded when request is supplied.
 * @param objects Optional already-produced object summary, or NULL; no inference runs here.
 * @return Zero on publication, -1 on invalid metadata, JPEG or storage failure.
 * @details Compressed image data is copied, not decoded/re-encoded. Combining tags
 * avoids a second full-file rewrite but does not remove the camera's original JPEG
 * write. Validate frame identity before calling. In-place updates retain the inode;
 * optional atomic replacement changes it. In-place I/O failures can damage the file.
 */
int image_exif_write_complete(const char *filename, const union dc_shot_union *shot,
                              double delay, int compensate, const struct capture_timing *timing,
                              const union dc_shot_union *request, double uncertainty_m, const char *objects);

#ifdef __cplusplus
}
#endif

#endif