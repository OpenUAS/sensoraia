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

/** @file loom_return.h
 * @brief Bounded direction-to-target policy, independent of inference and FC transport. */
#ifndef SENSORAIA_LOOM_RETURN_H
#define SENSORAIA_LOOM_RETURN_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Local ground point in metres east/north of the reference origin. */
struct loom_return_point { double east, north; };

/** @brief Wire metric values distinguishing active vector steering from arrival. */
enum loom_return_metric {
  LOOM_METRIC_NONE = 0,
  LOOM_METRIC_STEERING = 1,
  LOOM_METRIC_ARRIVED = 2,
  LOOM_METRIC_BARO_MAGNETO = 3
};

/** @brief Limits applied before a neural direction may become a target. */
struct loom_return_config {
  double home_radius_m; /**< Destination arrival / protection radius in metres. */
  double lookahead_m; /**< Carrot step distance ahead of the vehicle along a unit vector, metres. */
  double maximum_step_m; /**< Terminal per-observation target refinement bound, metres. */
  double minimum_confidence; /**< Required model confidence in 0..1. */
  double maximum_tilt_deg; /**< Maximum absolute roll or pitch accepted for a photo, degrees. */
};

/** @brief Caller-owned sequence and smoothing state; first observation seeds target. */
struct loom_return_state {
  struct loom_return_point target;
  uint32_t sequence, random_state;
};

/** @brief Explicit model-vector coordinates; embeddings are never guessed to be bearings. */
enum loom_return_frame {
  LOOM_RETURN_NONE,
  LOOM_RETURN_NORTH_EAST,
  LOOM_RETURN_BODY_FORWARD_RIGHT,
  LOOM_RETURN_SIMULATED
};

/** @brief Module-owned inference settings, separate from universal runtime settings. */
struct loom_return_options {
  struct loom_return_config limits;
  enum loom_return_frame frame;
  double home_latitude, home_longitude; /**< Optional live-sortie destination, geographic degrees. */
  unsigned seen;
};

/** @brief Initialize return limits with feature-only semantics by default. */
void loom_return_options_init(struct loom_return_options *options);
/** @brief Parse one LOOM-owned settings field, rejecting duplicates and unknown keys. */
bool loom_return_setting(void *context, const char *key, const char *value);
/** @brief Check that optional destination coordinates are either both present or both omitted. */
bool loom_return_options_valid(const struct loom_return_options *options);
/** @brief Convert a declared two-component vector to earth-frame east/north with attitude gating.
 * @param frame Declared model coordinate system.
 * @param first North or forward component.
 * @param second East or right component.
 * @param roll_deg Aircraft roll, right wing down positive, degrees.
 * @param pitch_deg Aircraft pitch, nose up positive, degrees.
 * @param yaw_deg Aircraft magnetometer heading, clockwise from north, degrees.
 * @param max_tilt_deg Maximum allowed absolute roll or pitch in degrees.
 * @param direction Written only for finite supported coordinates within tilt limits. */
bool loom_return_vector_attitude(enum loom_return_frame frame, double first, double second,
                                 double roll_deg, double pitch_deg, double yaw_deg,
                                 double max_tilt_deg, struct loom_return_point *direction);
/** @brief Convert a level declared two-component vector to earth-frame east/north.
 * @param frame Declared model coordinate system.
 * @param first North or forward component.
 * @param second East or right component.
 * @param yaw_deg Aircraft heading, clockwise from north, degrees.
 * @param direction Written only for finite supported coordinates. */
bool loom_return_vector(enum loom_return_frame frame, double first, double second,
                         double yaw_deg, struct loom_return_point *direction);
/** @brief Generate a slightly noisy body-frame simulated homing vector seeded by neural feature output. */
void loom_return_simulated_vector(uint64_t seed, struct loom_return_point position,
                                  double yaw_deg, float vector[3]);

/** @brief Conservative return-policy defaults, not aircraft flight settings.
 * @param config Writable destination. */
void loom_return_defaults(struct loom_return_config *config);
/** @brief Validate finite limits and their cross-field relationships. */
bool loom_return_config_valid(const struct loom_return_config *config);

/** @brief Convert an earth-frame direction into a HOME-bounded target.
 * @param config Valid policy limits.
 * @param position Vehicle east/north relative to HOME, metres.
 * @param direction Earth-frame east/north components; magnitude is not distance.
 * @param confidence Finite model confidence in 0..1.
 * @param target Written only for an accepted, nonzero direction toward HOME.
 * @return False for invalid, away-facing or inconsistent geometry. */
bool loom_return_direction(const struct loom_return_config *config,
                            struct loom_return_point position,
                            struct loom_return_point direction, double confidence,
                            struct loom_return_point *target);

/** @brief Admit a fresh bounded target and limit its movement.
 * @param state Exclusive caller state; not shared between sorties.
 * @param config Valid policy limits.
 * @param sequence Strictly increasing positive observation number.
 * @param candidate Proposed point within the HOME radius.
 * @param target Smoothed accepted point, unchanged on rejection.
 * @return True only when the state advances. Transport owners must commit a
 * copied state only after their waypoint transaction succeeds. */
bool loom_return_update(struct loom_return_state *state, const struct loom_return_config *config,
                         uint32_t sequence, struct loom_return_point candidate,
                         struct loom_return_point *target);

/** @brief Produce small mean-reverting target fluctuations for explicit PC simulation.
 * @param state Caller state; random seed zero selects a deterministic seed.
 * @param config Valid policy limits.
 * @param sequence Strictly increasing observation number.
 * @param target Simulated point, never represented as a neural observation.
 * @return True only for a fresh bounded update. */
bool loom_return_simulate(struct loom_return_state *state, const struct loom_return_config *config,
                           uint32_t sequence, struct loom_return_point *target);

#ifdef __cplusplus
}
#endif
#endif