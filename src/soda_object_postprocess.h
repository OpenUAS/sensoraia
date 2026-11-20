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
 * @file soda_object_postprocess.h
 * @brief EXIF-driven object-hit post-processing for SODA. */
#ifndef SODA_OBJECT_POSTPROCESS_H
#define SODA_OBJECT_POSTPROCESS_H

/**
 * @brief Post-process a captured image for an object-detection hit, if it has one.
 * @param image_path Path to the JPEG SODA was invoked on.
 * @return true when there was nothing to do (no AICAM_OBJECTS_V1 hit in EXIF, or the
 * marker couldn't be read at all -- the common case for every non-spottercam shot) or
 * every post-processing step for a real hit succeeded; false only when a hit was found but
 * at least one downstream step (full-frame copy, crop, or CSV row) failed.
 * @details No-op beyond one EXIF read unless @p image_path carries an AICAM_OBJECTS_V1
 * record (written by usher's image_exif_write_object_detections()) reporting count >= 1.
 * On a hit, copies the image into a sibling `objects_captured/` directory, saves a padded
 * crop of the detected box into a sibling `tight_crop_objects/` directory, and appends one
 * row to `objects_captured/detections.csv`. Both directories are derived from
 * @p image_path's own directory -- there is no separately configured base directory.
 */
bool soda_process_object_detection(const char *image_path);

#endif // SODA_OBJECT_POSTPROCESS_H
