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

/** @file dipla_contract.h
 * @brief Fixed model geometry and C/C++ local transport units, not runtime settings. */
#ifndef SENSORAIA_DIPLA_CONTRACT_H
#define SENSORAIA_DIPLA_CONTRACT_H
#include <stdint.h>

/** Supported model grid side; generic numerical grids remain independent. */
#define DIPLA_GRID_SIDE 51U
/** Supported square field extent in metres. */
#define DIPLA_FIELD_EXTENT_M 100.0
/** Uniform production lattice spacing in metres. */
#define DIPLA_GRID_STEP_M (DIPLA_FIELD_EXTENT_M / (DIPLA_GRID_SIDE - 1U))
/** Ordered route slots; changing this requires a new model and transport contract. */
#define DIPLA_WAYPOINT_COUNT 8U
/** XYZ coordinate count, independent of map channels. */
#define DIPLA_COORDINATE_COUNT 3U
/** Mean, variance and position-marker map channels. */
#define DIPLA_MAP_CHANNEL_COUNT 3U
/** Complete reverse-diffusion step count for the supported export. */
#define DIPLA_DIFFUSION_STEPS 20U
/** Training-time altitude normalization lower bound in metres. */
#define DIPLA_MODEL_Z_MIN_M 10.0
/** Training-time altitude normalization upper bound in metres. */
#define DIPLA_MODEL_Z_MAX_M 40.0
/** Fixed mission experiment height; not the model altitude normalization. */
#define DIPLA_DEMO_HEIGHT_MM 2000
#define DIPLA_FLIGHT_HEIGHT_MIN_MM 200
#define DIPLA_FLIGHT_HEIGHT_MAX_MM 40000
/** Accepted deviation from the fixed experiment height in metres. */
#define DIPLA_HEIGHT_TOLERANCE_M 0.75
/** Millimetres per metre. */
#define DIPLA_MM_PER_M 1000.0
/** Geographic integer units per degree. */
#define DIPLA_GEO_UNITS_PER_DEGREE 10000000.0
/** Q8 AGL integer units per metre. */
#define DIPLA_AGL_UNITS_PER_M 256.0
/** Maximum absolute latitude in geographic integer units. */
#define DIPLA_LATITUDE_LIMIT 900000000
/** Maximum absolute longitude in geographic integer units. */
#define DIPLA_LONGITUDE_LIMIT 1800000000
/** Low bits reserved for a legacy reply's zero-based route slot. */
#define DIPLA_SLOT_BITS 3U
/** Legacy metric slot mask. */
#define DIPLA_SLOT_MASK (DIPLA_WAYPOINT_COUNT - 1U)
/** Largest generation that preserves a nonnegative signed metric. */
#define DIPLA_MAX_GENERATION ((uint32_t)INT32_MAX >> DIPLA_SLOT_BITS)

#ifdef __cplusplus
static_assert(DIPLA_WAYPOINT_COUNT == (1U << DIPLA_SLOT_BITS), "Diplacam slot encoding changed");
#else
_Static_assert(DIPLA_WAYPOINT_COUNT == (1U << DIPLA_SLOT_BITS), "Diplacam slot encoding changed");
#endif
#endif