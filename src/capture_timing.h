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

#ifndef USHER_CAPTURE_TIMING_H
#define USHER_CAPTURE_TIMING_H

/**
 * @file capture_timing.h
 * @brief Sensor observation times, aligned poses, and optional frame-arrival evidence.
 * @details Timing is deliberately represented with monotonic clocks only; wall-clock
 * adjustments must never turn an otherwise valid capture into a negative duration.
 */

#include <stdbool.h>
#include <stdint.h>
#include "usher_protocol.h"

enum capture_time_kind {
  CAPTURE_TIME_UNKNOWN,
  CAPTURE_TIME_SENSOR_EXPOSURE,
  CAPTURE_TIME_CALIBRATED_EXPOSURE,
  CAPTURE_TIME_SHUTTER_WINDOW,
  CAPTURE_TIME_FRAME_ARRIVAL,
  CAPTURE_TIME_AUDIO_WINDOW
};

struct capture_observation {
  uint64_t earliest_us;
  uint64_t latest_us;
  uint64_t sequence;
  uint64_t sensor_timestamp_ns;
  enum capture_time_kind kind;
  union dc_shot_union pose;
  double pose_uncertainty_m;
  bool pose_valid;
  bool frame_bound;
  bool navigation_projected;
  uint32_t navigation_age_ms;
};

/**
 * @brief Check that an observation supplies a usable time interval.
 * @param observation Nullable observation to check.
 * @return True for a known timing kind with an ordered, nonzero-start interval
 *         spanning no more than 20 seconds.
 * @details Check endpoint order before subtracting unsigned timestamps. This
 * validates the interval only, not its pose, sequence, or frame association.
 */
static inline bool capture_observation_valid(const struct capture_observation *observation)
{
  return observation != 0 && observation->kind > CAPTURE_TIME_UNKNOWN
         && observation->kind <= CAPTURE_TIME_AUDIO_WINDOW && observation->earliest_us > 0
         && observation->latest_us >= observation->earliest_us
         && observation->latest_us - observation->earliest_us <= UINT64_C(20000000);
}

/**
 * @brief Distinguish exposure evidence from other valid observation times.
 * @param observation Nullable observation to classify.
 * @return True for a valid sensor, calibrated, or shutter-window interval.
 * @details Downstream projection must not substitute frame delivery or audio
 * timing for exposure, even if those intervals are structurally valid.
 */
static inline bool capture_observation_exposure(const struct capture_observation *observation)
{
  return capture_observation_valid(observation)
         && observation->kind >= CAPTURE_TIME_SENSOR_EXPOSURE
         && observation->kind <= CAPTURE_TIME_SHUTTER_WINDOW;
}

/**
 * @brief Bound exposure time using calibrated frame delivery latency.
 * @param[in,out] observation Record to update; existing pose validity is cleared.
 * @param arrival_us Frame arrival in local monotonic microseconds.
 * @param minimum_us Shortest calibrated exposure-to-arrival delay, in microseconds.
 * @param maximum_us Longest calibrated exposure-to-arrival delay, in microseconds.
 * @return True if the interval was written; false leaves the record unchanged.
 * @details The longest delay gives the earliest plausible exposure. Reject bounds
 * that would underflow or reach time zero, and require a fresh pose alignment after
 * changing the interval.
 */
static inline bool capture_observation_latency(struct capture_observation *observation,
    uint64_t arrival_us, uint64_t minimum_us, uint64_t maximum_us)
{
  if (observation == 0 || minimum_us > maximum_us || maximum_us > UINT64_C(2000000)
      || arrival_us <= maximum_us) return false;
  observation->earliest_us = arrival_us - maximum_us;
  observation->latest_us = arrival_us - minimum_us;
  observation->kind = CAPTURE_TIME_CALIBRATED_EXPOSURE;
  observation->pose_valid = false;
  return true;
}

/**
 * @brief Get the text label used when recording observation timing.
 * @param kind Source of the timing evidence.
 * @return Borrowed static string, or @c unavailable for an unrecognized kind.
 * @details Labels retain the distinction between an exposure estimate and frame
 * delivery so that downstream log readers do not conflate the two.
 */
static inline const char *capture_time_name(enum capture_time_kind kind)
{
  switch (kind) {
    case CAPTURE_TIME_SENSOR_EXPOSURE: return "sensor_exposure_interval";
    case CAPTURE_TIME_CALIBRATED_EXPOSURE: return "calibrated_exposure_interval";
    case CAPTURE_TIME_SHUTTER_WINDOW: return "command_to_download_interval";
    case CAPTURE_TIME_FRAME_ARRIVAL: return "frame_arrival_not_exposure";
    case CAPTURE_TIME_AUDIO_WINDOW: return "audio_window";
    default: return "unavailable";
  }
}

/**
 * @brief Shared observation evidence with optional LWIR request/frame-delivery timing.
 * @details A non-callback capture has no callback sequence evidence. A callback-backed
 * capture must provide a strictly later arrival timestamp and a nonzero sequence number,
 * so consumers can distinguish a fresh frame from a stale parser result.
 */
struct capture_timing {
  /** Server request acceptance in monotonic microseconds. */
  uint64_t request_monotonic_us;
  /** Monotonic microseconds when the selected frame arrived. */
  uint64_t arrival_monotonic_us;
  /** Producer sequence number for callback delivery; zero for polling delivery. */
  uint64_t callback_sequence;
  /** Callback frames dropped before this frame; must be lower than @c callback_sequence. */
  uint64_t callback_drops;
  /** True when arrival evidence originated from the asynchronous callback path. */
  bool callback_arrival;
  struct capture_observation observation;
};

/**
 * @brief Check whether timing evidence is internally consistent and recent.
 * @param timing Evidence to inspect.
 * @return True for a plausible request/arrival pair, false otherwise.
 * @details The 20-second bound detects stale parser state without rejecting normal
 * warmup/USB recovery behavior. Callback and polling modes have intentionally distinct
 * sequence invariants, reflected in the final conditional.
 */
static inline bool capture_timing_valid(const struct capture_timing *timing)
{
  return timing != 0 && timing->request_monotonic_us > 0
         && timing->arrival_monotonic_us >= timing->request_monotonic_us
         && timing->arrival_monotonic_us - timing->request_monotonic_us <= UINT64_C(20000000)
         && (timing->callback_arrival
             ? timing->callback_sequence > 0 && timing->callback_drops < timing->callback_sequence
             && timing->arrival_monotonic_us > timing->request_monotonic_us
             : timing->callback_sequence == 0 && timing->callback_drops == 0);
}

#endif