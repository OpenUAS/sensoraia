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
 * @file chdk_camera_backend.h
 * @brief CHDK-owned download and photo publication. */
#ifndef USHER_CHDK_CAMERA_BACKEND_H
#define USHER_CHDK_CAMERA_BACKEND_H
#include "camera_backend.h"

/** Configure a borrowed photo directory before capture; used by isolated speed tests. */
void chdk_camera_backend_set_photo_directory(const char *directory);
/** Return CHDK's descriptor without acquiring the camera. */
struct camera_backend chdk_camera_backend_get(void);
#endif