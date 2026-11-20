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

#ifndef SENSORAIA_LWIR_ANALYSIS_H
#define SENSORAIA_LWIR_ANALYSIS_H
/** @file
 * @brief Optional saved-image adapter combining thermal detection, pose and projection.
 *
 * @details Compiled only in analysis builds (`LWIR_ENABLE_ANALYSIS=1`, see
 * lwir_features.h); imagery-only builds contain none of this code or its static
 * scratch storage. The adapter post-processes an LWIR JPEG that was already saved
 * with its exact native temperature plane embedded (LWIRRAW1, or LWIRSIM1 for
 * simulated evidence, read via lwir_container.h). It is invoked by lwircam after a
 * capture is written and by the `--geolocate` command-line mode.
 *
 * Pipeline: temperature plane -> lwir_detect_hotspots() (pixel candidates) ->
 * pose parsed from the image's EXIF ImageDescription -> optional
 * lwir_geolocation_project() -> text record appended to EXIF UserComment.
 * Detection and projection stay separate modules so they can be reused without
 * JPEG or flight EXIF.
 */
#ifdef __cplusplus
extern "C" {
#endif

/** @brief Annotate a saved thermal JPEG without recompressing its display pixels.
 * @param filename Exclusively owned image; native thermal and SENSOR02 evidence survive.
 * @param calibration_path Projection calibration, or NULL for pixel-only results.
 * @return Zero on success, otherwise -1. Missing pose/calibration is recorded, not invented.
 * @details Serial worker adapter with its own bounded workspace. The underlying
 * detector and projection interfaces remain usable without JPEG or flight EXIF.
 *
 * The record is written to EXIF UserComment after the existing ImageDescription
 * text and a `LWIR_HOTSPOTS_V1` separator line. A record is still published (with
 * a `status=` explaining why) when the thermal plane is invalid or absent, the pose
 * is missing, calibration is missing/invalid/unverified, or nothing is detected.
 * Coordinates appear only for `status=estimated` hotspots whose projection succeeds.
 *
 * @warning Not reentrant or thread-safe: all large buffers are file-scope statics.
 * Call from one worker thread at a time.
 * @warning Projection is not gated on capture timing. The record carries
 * `capture_pose_synchronized` and `capture_pose_uncertainty_m`; consumers must
 * reject or down-weight coordinates whose pose is not exposure-qualified (for
 * example a host arrival timestamp).
 * @note C has no exceptions; -1 is returned when the file cannot be read or
 * rewritten, the detector rejects the plane, or the record would exceed its
 * fixed 8192-byte buffer. errno may reflect the last failing system call but is
 * not guaranteed to be meaningful. Failures before publishing leave the file
 * untouched; with IMAGE_EXIF_ATOMIC_REPLACE a failed rewrite also keeps the
 * original, whereas the in-place fallback offers no such guarantee.
 */
int lwir_geolocate_image(const char *filename, const char *calibration_path);

#ifdef __cplusplus
}
#endif
#endif