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
 * @file earcam_core.h
 * @brief Sensor-independent acoustic samples and ground-source estimates.
 * @details Supply geotagged audio windows from any capture adapter. This API
 * does not acquire audio, own a flight session, or know about USHER/UART.
 * Positions and heights must use a consistent navigation reference; the solver
 * cannot repair microphone gain changes, unsynchronized poses or bad terrain data.
 */
#ifndef EARCAM_CORE_H
#define EARCAM_CORE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/** Bounded solver capacity; USHER reserves sample storage separately from this workspace. */
#define EAR_MAX_SAMPLES 16384U

/** @brief Estimated ground source, not a surveyed target or guaranteed alarm detection. */
struct ear_loudest_spot {
  bool valid;            /**< A numerical estimate was produced; apply mission acceptance checks. */
  double lat_deg;        /**< Estimated source latitude, degrees. */
  double lon_deg;        /**< Estimated source longitude, degrees. */
  double agl_m;          /**< Representative microphone height from selected loud windows, metres. */
  double alt_m;          /**< Ground altitude: peak-window navigation altitude minus its AGL. */
  double level_db;       /**< Strongest retained level; dBFS unless the capture chain is calibrated. */
  double confidence;     /**< Relative prominence/concentration/support score, not a probability. */
  uint32_t sample_count; /**< Input window count, including windows later rejected. */
  uint32_t used_count;   /**< Window count remaining after clipping, pose and quiet-outlier gates. */
};

/**
 * @brief One audio window associated with microphone position and provenance.
 * @details Keep gain fixed across a session: relative level differences drive
 * localization. False pose_aligned preserves legacy arrival-time association;
 * it must not be reported as exposure/audio-window synchronization.
 */
struct ear_sample {
  uint64_t timestamp_ms;      /**< Producer monotonic report timestamp, milliseconds; capture bounds are separate. */
  int32_t shot_nr;            /**< Associated FC photo/request number; not the audio sequence. */
  double lat_deg;            /**< Microphone latitude from associated navigation evidence, degrees. */
  double lon_deg;            /**< Microphone longitude from associated navigation evidence, degrees. */
  double agl_m;              /**< Height above the supplied ground reference, metres. */
  double alt_m;              /**< Microphone navigation altitude, metres; datum supplied by caller. */
  double level_db;           /**< Band loudness, dBFS or calibrated dB SPL; never mix conventions. */
  double trend_db;           /**< Fast-minus-slow envelope level used to favor approaching evidence. */
  double contrast_db;        /**< Tone prominence against the capture detector's background estimate. */
  double frequency_hz;       /**< Dominant detector-bank frequency, hertz. */
  bool alarm;                /**< Capture detector's tone-like alarm flag, not solver confidence. */
  bool clipped;              /**< ADC saturation makes level comparisons unreliable. */
  bool pose_aligned;         /**< Pose association used bounded audio-window timing evidence. */
  double pose_uncertainty_m; /**< Adapter-supplied spatial uncertainty, metres; not solver error. */
  /** Audio-window bounds in host monotonic microseconds, followed by producer sequence. */
  uint64_t window_earliest_us, window_latest_us, window_sequence;
  /** Original FC coordinates in 1e7 degrees, retained for timing/pose diagnostics. */
  int32_t request_lat, request_lon;
  bool classifier; /**< Distinguish PCM model acceptance from the native tone detector's alarm rule. */
  double classifier_score; /**< Model probability, never the acoustic level or localization confidence. */
  unsigned classifier_class; /**< Model-owned class index; compare only within a fixed assignment. */
};

/**
 * @brief Estimate a ground source from relative loudness along a survey track.
 * @param samples Caller-owned immutable windows; never modified or retained.
 * @param count Input count, bounded by EAR_MAX_SAMPLES.
 * @param result Required destination; cleared before validation when non-NULL.
 * @return Zero for an estimate, -1 for invalid input or insufficient retained evidence.
 * @details A robust centroid seeds a damped inverse-distance fit; unstable fits
 * fall back to that centroid. Shared bounded scratch storage is mutex-protected,
 * so concurrent calls serialize without allocating another survey-sized buffer.
 * @warning A valid result is not an absolute alarm gate. With too few alarm
 * windows the solver deliberately uses other finite levels, even silent ones.
 * Mission policy must consider level, quality, geometry and independent evidence.
 * The legacy near-zero-latitude placeholder gate also excludes equatorial tracks.
 */
int calculated_loudestspot(const struct ear_sample *samples, size_t count, struct ear_loudest_spot *result);

#endif