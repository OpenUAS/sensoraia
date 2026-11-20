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
 * @file image_mock_transform.h
 * @brief Local-simulation JPEG attitude transform API. */
#ifndef USHER_IMAGE_MOCK_TRANSFORM_H
#define USHER_IMAGE_MOCK_TRANSFORM_H

#include <stddef.h>
#include "usher_protocol.h"

/** @brief Successful result: the JPEG was rewritten with the simulated attitude transform. */
#define IMAGE_MOCK_TRANSFORM_APPLIED 0
/** @brief Successful result: transform was intentionally skipped for a near-horizon attitude. */
#define IMAGE_MOCK_TRANSFORM_SKIPPED 1

/**
 * @brief Rewrite a local test JPEG to simulate the shot attitude.
 * @param filename Writable path to the JPEG to transform in place.
 * @param shot Flight pose containing fixed-point roll, pitch, yaw, and position fields.
 * @return @c IMAGE_MOCK_TRANSFORM_APPLIED, @c IMAGE_MOCK_TRANSFORM_SKIPPED, or -1 on failure.
 * @details The transform uses a perspective projection and an atomic temporary-file rename so
 * consumers never observe a partial JPEG. Near-horizon frames are skipped or synthesized by
 * configured policy because planar image projection becomes numerically untrustworthy there.
 * @warning This is simulation evidence only; it does not model a calibrated physical camera.
 */
int image_mock_transform(const char *filename, const union dc_shot_union *shot);

int image_mock_map(const char *filename, const char *cache_directory,
                   const union dc_shot_union *shot, size_t width, size_t height,
                   double horizontal_fov_deg, double vertical_fov_deg);

#endif