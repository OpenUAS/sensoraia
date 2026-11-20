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
 * @file camera_profile.h
 * @brief Public C API for reading PANGEA camera calibration profiles.
 * @details The file binds measured intrinsics and mount angles to a camera
 * filename prefix so mapping geometry is never guessed from a backend ID.
 */
#ifndef USHER_CAMERA_PROFILE_H
#define USHER_CAMERA_PROFILE_H

#include <stdbool.h>
#include "coverage_scheduler.h"

#define CAMERA_PROFILE_MAX_BYTES 1024

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief One measured camera calibration loaded from the flat PANGEA profile format.
 * @details Intrinsics use pixels, mount angles use degrees, and the filename
 * prefix is a NUL-terminated discriminator owned by this structure.
 */
struct usher_camera_profile {
  struct coverage_camera calibration;
  unsigned camera_id;
  char filename_prefix[65];
};

/**
 * @brief Read a bounded PANGEA_CAMERA_V1 key=value calibration file.
 * @param[in] filename Profile path; comments and blank lines are allowed.
 * @param[out] profile Receives parsed geometry and filename prefix; unchanged on failure.
 * @return True only when all required fields are present and valid.
 * @details No heap allocation is performed. Duplicate and unknown keys are
 * rejected so a typo cannot silently select a different camera calibration.
 * A valid mapping prefix may have no USHER camera ID (reported as zero);
 * consumers also check that the calibration satisfies their geometry constraints.
 */
bool usher_camera_profile_read(const char *filename, struct usher_camera_profile *profile);

#ifdef __cplusplus
}
#endif

#endif
