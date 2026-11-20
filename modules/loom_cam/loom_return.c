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

/** @file loom_return.c
 * @brief Constant-storage return geometry with explicit simulation-only fallback. */
#include "loom_return.h"

#include <math.h>
#include <string.h>
#include "../../src/bounded_ini.h"

bool loom_return_config_valid(const struct loom_return_config *config)
{
  return config && isfinite(config->home_radius_m) && config->home_radius_m >= 1 && config->home_radius_m <= 300 &&
    isfinite(config->lookahead_m) && config->lookahead_m >= 1 && config->lookahead_m <= config->home_radius_m &&
    isfinite(config->maximum_step_m) && config->maximum_step_m > 0 && config->maximum_step_m <= config->home_radius_m &&
    isfinite(config->minimum_confidence) && config->minimum_confidence >= 0 && config->minimum_confidence <= 1 &&
    isfinite(config->maximum_tilt_deg) && config->maximum_tilt_deg >= 1 && config->maximum_tilt_deg <= 60;
}

void loom_return_defaults(struct loom_return_config *config)
{
  if (config) *config = (struct loom_return_config) {60, 50, 5, 0.6, 25};
}

void loom_return_options_init(struct loom_return_options *options)
{
  if (!options) return;
  *options = (struct loom_return_options){0};
  loom_return_defaults(&options->limits);
}

bool loom_return_options_valid(const struct loom_return_options *options)
{
  if (!options || !loom_return_config_valid(&options->limits)) return false;
  const unsigned home_fields = (1U << 5) | (1U << 6);
  const unsigned present = options->seen & home_fields;
  return present == 0 || present == home_fields;
}

bool loom_return_setting(void *context, const char *key, const char *value)
{
  struct loom_return_options *options = context;
  const char *const names[] = {"return_vector_frame", "return_home_radius_m", "return_lookahead_m",
                              "return_max_step_m", "return_min_confidence", "return_home_latitude",
                              "return_home_longitude", "return_max_tilt_deg"};
  unsigned field = 0;
  while (field < 8 && strcmp(key, names[field])) ++field;
  if (!options || field == 8 || (options->seen & (1U << field))) return false;
  options->seen |= 1U << field;
  if (!field) {
    if (!strcmp(value, "none")) options->frame = LOOM_RETURN_NONE;
    else if (!strcmp(value, "north_east")) options->frame = LOOM_RETURN_NORTH_EAST;
    else if (!strcmp(value, "body_forward_right")) options->frame = LOOM_RETURN_BODY_FORWARD_RIGHT;
    else if (!strcmp(value, "simulated")) options->frame = LOOM_RETURN_SIMULATED;
    else return false;
    return true;
  }
  double number;
  if (!ini_real(value, &number)) return false;
  if (field == 1 && number >= 1 && number <= 300) options->limits.home_radius_m = number;
  else if (field == 2 && number >= 1 && number <= 300) options->limits.lookahead_m = number;
  else if (field == 3 && number > 0 && number <= 300) options->limits.maximum_step_m = number;
  else if (field == 4 && number >= 0 && number <= 1) options->limits.minimum_confidence = number;
  else if (field == 5 && fabs(number) <= 85) options->home_latitude = number;
  else if (field == 6 && fabs(number) <= 180) options->home_longitude = number;
  else if (field == 7 && number >= 1 && number <= 60) options->limits.maximum_tilt_deg = number;
  else return false;
  return true;
}

bool loom_return_vector_attitude(enum loom_return_frame frame, double first, double second,
                                 double roll_deg, double pitch_deg, double yaw_deg,
                                 double max_tilt_deg, struct loom_return_point *direction)
{
  if (!direction || !isfinite(first) || !isfinite(second) ||
      !isfinite(roll_deg) || !isfinite(pitch_deg) || !isfinite(yaw_deg) ||
      !isfinite(max_tilt_deg) || max_tilt_deg < 1 || max_tilt_deg > 60 ||
      fabs(roll_deg) > max_tilt_deg || fabs(pitch_deg) > max_tilt_deg ||
      fabs(yaw_deg) > 36000) return false;
  if (frame == LOOM_RETURN_NORTH_EAST) *direction = (struct loom_return_point){second, first};
  else if (frame == LOOM_RETURN_BODY_FORWARD_RIGHT || frame == LOOM_RETURN_SIMULATED) {
    const double radians = 0.017453292519943295;
    const double roll = roll_deg * radians;
    const double pitch = pitch_deg * radians;
    const double yaw = remainder(yaw_deg, 360) * radians;
    const double forward = first * cos(pitch) + second * sin(roll) * sin(pitch);
    const double right = second * cos(roll);
    *direction = (struct loom_return_point){forward * sin(yaw) + right * cos(yaw),
                                          forward * cos(yaw) - right * sin(yaw)};
  } else return false;
  return isfinite(direction->east) && isfinite(direction->north);
}

void loom_return_simulated_vector(uint64_t seed, struct loom_return_point position,
                                  double yaw_deg, float vector[3])
{
  if (!vector) return;
  seed = seed * UINT64_C(6364136223846793005) + UINT64_C(1442695040888963407);
  const double jitter = ((double)(uint32_t)(seed >> 32) / 4294967295.0 * 2.0 - 1.0) * 0.22;
  seed = seed * UINT64_C(6364136223846793005) + UINT64_C(1442695040888963407);
  const double confidence = 0.85 + ((double)(uint32_t)(seed >> 32) / 4294967295.0) * 0.13;
  const double distance = hypot(position.east, position.north);
  if (isfinite(distance) && distance > 1e-6 && isfinite(yaw_deg)) {
    const double bearing = atan2(-position.east, -position.north) + jitter;
    const double heading = remainder(yaw_deg, 360.0) * 0.017453292519943295;
    const double relative = bearing - heading;
    const double scale = distance < 50.0 ? fmax(0.1, distance / 50.0) : 1.0;
    vector[0] = (float)(cos(relative) * scale);
    vector[1] = (float)(sin(relative) * scale);
  } else {
    vector[0] = (float)cos(jitter);
    vector[1] = (float)sin(jitter);
  }
  vector[2] = (float)confidence;
}

bool loom_return_vector(enum loom_return_frame frame, double first, double second,
                         double yaw_deg, struct loom_return_point *direction)
{
  return loom_return_vector_attitude(frame, first, second, 0, 0, yaw_deg, 25, direction);
}

bool loom_return_direction(const struct loom_return_config *config,
                            struct loom_return_point position,
                            struct loom_return_point direction, double confidence,
                            struct loom_return_point *target)
{
  if (!target || !loom_return_config_valid(config) || !isfinite(position.east) || !isfinite(position.north) ||
      !isfinite(direction.east) || !isfinite(direction.north) ||
      !isfinite(confidence) || confidence < config->minimum_confidence || confidence > 1) return false;
  const double distance = hypot(position.east, position.north);
  const double magnitude = hypot(direction.east, direction.north);
  if (!isfinite(distance) || distance > 5000 || !isfinite(magnitude) || magnitude < 1e-6) return false;
  direction.east /= magnitude;
  direction.north /= magnitude;
  const double along = -(position.east * direction.east + position.north * direction.north);
  if (distance > config->home_radius_m && along <= 0) return false;
  double step = distance > config->home_radius_m ? fmax(config->lookahead_m, along)
                                                 : config->lookahead_m * fmin(1.0, magnitude);
  if (distance > 0 && distance <= config->home_radius_m && along > 0 && along < step) step = along;
  struct loom_return_point proposed = {
    position.east + direction.east * step,
    position.north + direction.north * step
  };
  const double proposed_distance = hypot(proposed.east, proposed.north);
  if (!isfinite(proposed_distance)) return false;
  if (proposed_distance > config->home_radius_m) {
    if (distance <= config->home_radius_m || along < distance * 0.95) return false;
    proposed.east *= config->home_radius_m / proposed_distance;
    proposed.north *= config->home_radius_m / proposed_distance;
  }
  *target = proposed;
  return true;
}

bool loom_return_update(struct loom_return_state *state, const struct loom_return_config *config,
                         uint32_t sequence, struct loom_return_point candidate,
                         struct loom_return_point *target)
{
  if (!state || !target || !loom_return_config_valid(config) || !sequence || sequence <= state->sequence ||
      !isfinite(candidate.east) || !isfinite(candidate.north) ||
      !isfinite(state->target.east) || !isfinite(state->target.north) ||
      hypot(candidate.east, candidate.north) > config->home_radius_m ||
      hypot(state->target.east, state->target.north) > config->home_radius_m) return false;
  const double east = candidate.east - state->target.east, north = candidate.north - state->target.north;
  const double movement = hypot(east, north);
  if (movement > config->maximum_step_m) {
    candidate.east = state->target.east + east * config->maximum_step_m / movement;
    candidate.north = state->target.north + north * config->maximum_step_m / movement;
  }
  state->target = *target = candidate;
  state->sequence = sequence;
  return true;
}

bool loom_return_simulate(struct loom_return_state *state, const struct loom_return_config *config,
                           uint32_t sequence, struct loom_return_point *target)
{
  if (!state || !loom_return_config_valid(config)) return false;
  struct loom_return_state proposed = *state;
  if (!proposed.random_state) proposed.random_state = 19;
  proposed.random_state = proposed.random_state * UINT32_C(1664525) + UINT32_C(1013904223);
  const double east = (double)(proposed.random_state >> 8) / 16777215.0 * 2 - 1;
  proposed.random_state = proposed.random_state * UINT32_C(1664525) + UINT32_C(1013904223);
  const double north = (double)(proposed.random_state >> 8) / 16777215.0 * 2 - 1;
  const double spread = fmin(15, config->home_radius_m / 3);
  const struct loom_return_point candidate = {
    proposed.target.east * 0.85 + east * spread * 0.15,
    proposed.target.north * 0.85 + north * spread * 0.15
  };
  if (!loom_return_update(&proposed, config, sequence, candidate, target)) return false;
  *state = proposed;
  return true;
}