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

/** @file spotter_targets.c
 * @brief Stable five-slot confidence ranking without heap, threads or flight-plan decisions. */
#include "spotter_targets.h"

#include <stddef.h>

void spotter_targets_reset(struct spotter_targets *targets)
{
  if (!targets) return;
  for (unsigned slot = 0; slot < SPOTTER_TARGET_COUNT; ++slot)
    targets->slots[slot] = (struct spotter_target){0};
}

int spotter_targets_offer(struct spotter_targets *targets, const struct spotter_target *observation,
                          bool (*apply)(void *, unsigned, const struct spotter_target *), void *context)
{
  if (!targets || !observation || !observation->valid || observation->score > 1000 ||
      observation->latitude < -900000000 || observation->latitude > 900000000 ||
      observation->longitude < -1800000000 || observation->longitude > 1800000000) return -1;
  unsigned selected = SPOTTER_TARGET_COUNT;
  for (unsigned slot = 0; slot < SPOTTER_TARGET_COUNT; ++slot) {
    const struct spotter_target *stored = &targets->slots[slot];
    if (stored->valid && stored->latitude == observation->latitude &&
        stored->longitude == observation->longitude) {
      if (observation->score <= stored->score) return -1;
      selected = slot;
      break;
    }
    if (!stored->valid && selected == SPOTTER_TARGET_COUNT) selected = slot;
  }
  if (selected == SPOTTER_TARGET_COUNT) {
    selected = 0;
    for (unsigned slot = 1; slot < SPOTTER_TARGET_COUNT; ++slot)
      if (targets->slots[slot].score < targets->slots[selected].score) selected = slot;
    if (observation->score <= targets->slots[selected].score) return -1;
  }
  if (apply && !apply(context, selected, observation)) return -1;
  targets->slots[selected] = *observation;
  return (int)selected;
}