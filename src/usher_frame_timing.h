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

/** @file usher_frame_timing.h
 * @brief C interface for conservative frame-time estimation and EXIF evidence.
 * @details Camera timestamps use BOOTTIME while USHER pose history uses
 * MONOTONIC. Preserve original sensor evidence and conversion uncertainty so a
 * consumer can decline pose alignment instead of treating an estimate as a
 * measurement. The implementation is C11; C++ camera code links through this
 * stable C ABI.
 */
#ifndef USHER_FRAME_TIMING_H
#define USHER_FRAME_TIMING_H

#include <stdbool.h>
#include <stdint.h>

#include "exifite.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Timing evidence for one exposure, with microsecond uncertainty.
 * @details A zero monotonic_us and time_error_us of 100001 leave timing
 * unqualified for the 100000-us acceptance limit. Raw exposure evidence is
 * retained on estimation failure; never replace a rejected estimate with now.
 */
struct usher_frame_timing {
  uint64_t sequence; ///< Camera frame sequence, not a flight-controller shot number.
  uint64_t sensor_ns; ///< Original sensor timestamp in BOOTTIME nanoseconds; zero if invalid.
  uint64_t monotonic_us; ///< Estimated exposure/readout-window midpoint in MONOTONIC microseconds.
  uint64_t time_error_us; ///< Conservative alignment allowance, not calibrated accuracy.
  int64_t exposure_us; ///< Reported exposure duration, retained even when rejected.
  int64_t duration_us; ///< Reported frame duration used as a readout allowance.
};

/** @brief Translate fresh sensor evidence into a cautious pose-alignment time.
 * @param sequence Camera sequence belonging to this exposure.
 * @param sensor_ns Sensor timestamp in the local BOOTTIME domain.
 * @param exposure_us Exposure duration in microseconds; accepted range 0..100000.
 * @param duration_us Frame duration in microseconds; accepted range 1..200000.
 * @return Original evidence plus an estimated midpoint and uncertainty, or
 * unqualified defaults when clock sampling or plausibility checks fail.
 * @details Bracketing BOOTTIME with MONOTONIC samples avoids assuming equal
 * clock origins, including after suspend. The returned record is a value, so
 * callers can retain it with the exact frame without allocation or shared state.
 * @warning Sensor timestamps must follow BOOTTIME. Frame duration is not a
 * measured rolling-shutter readout time; this estimate alone proves neither
 * shutter synchronization nor geolocation accuracy.
 */
struct usher_frame_timing usher_frame_timing_estimate(uint64_t sequence, uint64_t sensor_ns,
                                                       int64_t exposure_us, int64_t duration_us);

/** @brief Initialize capture EXIF for the JPEG encoded from this exposure.
 * @param[in] timing Same-exposure evidence, qualified or not.
 * @param width Actual encoded image width.
 * @param height Actual encoded image height.
 * @param[out] document Caller-owned bounded destination; previous contents are replaced.
 * @return False for a null argument, invalid boot identity/dimensions or failed reservation.
 * @details The capture owner may add model-specific tags before serializing one
 * APP1 payload. No filesystem operation, sidecar or feature-sized allocation is
 * performed here. The consumer still checks boot, dimensions and timing bounds.
 * @warning Embedded identity is not a checksum or calibrated timing measurement.
 */
bool usher_frame_document(const struct usher_frame_timing *timing, unsigned width, unsigned height,
                          struct exif_document *document);

#ifdef __cplusplus
}
#endif

#endif