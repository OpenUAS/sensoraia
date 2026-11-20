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

/** @file spotter_targets.h
 * @brief Five ranked geographic targets, independent of navigation and camera transport. */
#ifndef SENSORAIA_SPOTTER_TARGETS_H
#define SENSORAIA_SPOTTER_TARGETS_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SPOTTER_TARGET_COUNT 5U

/** @brief Geographic observation in the shared USHER point units. */
struct spotter_target {
  int32_t latitude, longitude; /**< Degrees times 1e7. */
  int32_t altitude_mm; /**< Ellipsoid altitude in millimetres. */
  uint16_t score; /**< Detector confidence in 0..1000. */
  bool valid; /**< False means no observation has assigned this slot. */
};

/** @brief Caller-owned fixed storage; slot indices stay stable while scores improve. */
struct spotter_targets {
  struct spotter_target slots[SPOTTER_TARGET_COUNT];
};

/** @brief Forget all prior assignments without manufacturing target coordinates.
 * @param targets Writable store, or NULL to do nothing. */
void spotter_targets_reset(struct spotter_targets *targets);

/** @brief Admit a target only after its owner's coordinate application succeeds.
 * @param targets Exclusive target store.
 * @param observation Immutable valid geographic observation.
 * @param apply Optional coordinate transaction; false leaves the selected slot unchanged.
 * @param context Borrowed callback state.
 * @return Assigned zero-based slot, or minus one for invalid/nonimproving/rejected observations.
 * @details Empty slots fill first; once full, only a strictly better score replaces the lowest.
 * Reobserving exactly the same position improves its existing slot instead of consuming another. */
int spotter_targets_offer(struct spotter_targets *targets, const struct spotter_target *observation,
                          bool (*apply)(void *, unsigned, const struct spotter_target *), void *context);

#ifdef __cplusplus
}
#endif

#endif