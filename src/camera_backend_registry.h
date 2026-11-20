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
 * @file camera_backend_registry.h
 * @brief Backend definitions and physical-camera routing. */
#ifndef USHER_CAMERA_BACKEND_REGISTRY_H
#define USHER_CAMERA_BACKEND_REGISTRY_H

#include "camera_backend.h"

/** Logical profiles are independent of stable protocol camera IDs. */
enum camera_backend_type {
  CAMERA_BACKEND_UNSELECTED,
  CAMERA_BACKEND_CHDK,
  CAMERA_BACKEND_LOCAL,
  CAMERA_BACKEND_AI_CAM,
  CAMERA_BACKEND_SPOTTER_CAM,
  CAMERA_BACKEND_LWIR_CAM,
  CAMERA_BACKEND_EAR_CAM,
  CAMERA_BACKEND_LOOM_CAM,
  CAMERA_BACKEND_DIPLA_CAM,
  CAMERA_BACKEND_COUNT
};

/**
 * @brief Return a descriptor without selecting, configuring or opening hardware.
 * @param type Logical backend profile, not a protocol camera ID.
 * @param mock Select a plugin's simulation-safe variant where applicable.
 * @return Descriptor with stable hook lifetimes; zero for absent/invalid profiles.
 */
struct camera_backend camera_backend_registry_get(enum camera_backend_type type, bool mock);
/**
 * @brief Resolve a physical ID, preferring its configured logical profile.
 * @param camera_id Stable protocol identity; unsupported IDs are rejected.
 * @param preferred Configured profile, used only for its own physical camera.
 * @param local_only Replace image acquisition with the generic local driver.
 * @return Profile to acquire, or CAMERA_BACKEND_UNSELECTED if unavailable.
 * @details Identity is resolved separately so local acquisition never erases
 * the requested sensor's naming, geometry or metadata policy.
 */
enum camera_backend_type camera_backend_registry_route(int camera_id, enum camera_backend_type preferred,
    bool local_only);
/**
 * @brief Resolve naming/metadata identity from the profile used for acquisition.
 * @param camera_id Stable protocol identity to describe.
 * @param preferred Configured logical profile, including specialized AIcam users.
 * @param mock Select the profile's mock-safe hooks without changing its identity.
 * @return Side-effect-free descriptor; unavailable IDs return a zero descriptor.
 */
struct camera_backend camera_backend_registry_identity(int camera_id, enum camera_backend_type preferred,
    bool mock);

#endif