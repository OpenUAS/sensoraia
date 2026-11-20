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
 * @file spotter_camera_backend.h
 * @brief Spottercam backend descriptor and optional native UART target projection.
 */
#ifndef USHER_SPOTTER_CAMERA_BACKEND_H
#define USHER_SPOTTER_CAMERA_BACKEND_H

#include <stdbool.h>
#include "camera_backend.h"

/**
 * @brief Configure pinhole camera calibration for projecting Spottercam boxes to UART points.
 * @param filename Path to an AIcam PANGEA_CAMERA_V1 calibration profile.
 * @return True when the profile is valid for USHER_CAMERA_AICAM.
 */
bool spotter_camera_backend_set_targets_profile(const char *filename);

/** @brief True when native Spottercam UART target projection is configured and active. */
bool spotter_camera_backend_targets_enabled(void);

/** @brief Build the Spottercam backend descriptor for the registry. */
struct camera_backend spotter_camera_backend_get(void);

#endif
