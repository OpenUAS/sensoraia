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
 * @file local_pipe.h
 * @brief Local mock-camera backend API for simulator and desk testing. */
#ifndef USHER_LOCAL_PIPE_H
#define USHER_LOCAL_PIPE_H

#include <stddef.h>
#include "camera_backend.h"

/** @brief Initialize a deterministic mock image source for local camera simulation.
 * @param mock_image Readable JPEG copied for each shot.
 * @return 0 on readiness or -1 on missing source/output storage.
 * @details Copy the path into the backend so the caller need not retain it. */
int local_pipe_init(const char *mock_image);

/** @brief Initialize test mode with a supplied JPEG or a random fixture beside USHER.
 * @param mock_image Optional explicit JPEG; NULL enables testphotos discovery.
 * @return 0 on readiness or -1 on configuration failure.
 * @details If discovery finds no eligible fixture, use the bundled mock JPEG. */
int local_pipe_test_init(const char *mock_image);

/** @brief Create a mock capture file for a shot.
 * @param filename Destination path buffer.
 * @param filename_size Destination capacity.
 * @param image_number Shot sequence.
 * @param identity Backend-owned filename prefix and simulation namespace.
 * @return 0 after a complete stable regular-file copy, otherwise -1.
 * @details Existing destinations are never overwritten; failed copies remove only their new output. */
int local_pipe_shoot(char *filename, size_t filename_size, int image_number, const struct camera_backend *identity);
/** @brief Prepare output storage for map-backed simulation.
 * @param source Unused compatibility argument.
 * @return Zero on readiness, or -1 on path or storage failure.
 * @details Tile availability is checked per capture because the set of
 * required tiles depends on the shot position and attitude. */
int local_pipe_map_init(const char *source);
/** @brief Render a map-backed capture using the trigger's camera geometry.
 * @param[out] filename Output path buffer; use only after a successful capture.
 * @param filename_size Capacity including the terminator.
 * @param shot Trigger containing the photo number, position, attitude and AGL.
 * @param identity Simulated camera prefix and raster dimensions.
 * @param cache_directory Map tile cache root.
 * @param horizontal_fov_deg Horizontal field of view in degrees.
 * @param vertical_fov_deg Vertical field of view in degrees.
 * @return Zero on publication, or -1 on invalid input, rendering or storage failure.
 * @details Construct a simulation filename for this camera and pass its geometry
 * to the cached-tile renderer; this path does not use a fixture JPEG. */
int local_pipe_map_shoot(char *filename, size_t filename_size, const union dc_shot_union *shot,
                         const struct camera_backend *identity, const char *cache_directory,
                         double horizontal_fov_deg, double vertical_fov_deg);
                         
/** @brief Forget selected local fixture sources.
 * @details No subprocess is owned and previously captured files are retained. */
void local_pipe_deinit(void);

#endif