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
 * @file camera_backend_registry.c
 * @brief Shared backend catalogue; the service schedules capabilities, not devices.
 * @details Descriptor lookup is side-effect free. Specialized backends own their
 * hooks; simple adapters share this table. Profiles never allocate protocol IDs.
 */
#include "camera_backend_registry.h"
#include "ai_cam_pipe.h"
#include "chdk_camera_backend.h"
#include "local_pipe.h"
#include "loom_camera_backend.h"
#include "lwir_cam_pipe.h"
#include "../modules/lwir_cam/lwir_features.h"
#include "spotter_cam_pipe.h"
#include "spotter_camera_backend.h"

#include "ear_camera_backend.h"
#include "dipla_camera_backend.h"
#ifndef USHER_SODA
  #define USHER_SODA "./bin/soda"
#endif
#ifndef USHER_AI_CAM_PHOTO_DIR
  #define USHER_AI_CAM_PHOTO_DIR "~/Pictures/sensoraia"
#endif
#ifndef USHER_LWIR_CAM_PHOTO_DIR
  #define USHER_LWIR_CAM_PHOTO_DIR "~/Pictures/sensoraia"
#endif
#ifndef USHER_SPOTTERCAM_PHOTO_DIR
  #define USHER_SPOTTERCAM_PHOTO_DIR "~/Pictures/sensoraia"
#endif

struct camera_backend camera_backend_registry_get(enum camera_backend_type type, bool mock)
{
  if (type == CAMERA_BACKEND_CHDK) return chdk_camera_backend_get();
  if (type == CAMERA_BACKEND_LOOM_CAM) return loom_camera_backend_get(mock);
  if (type == CAMERA_BACKEND_SPOTTER_CAM) return spotter_camera_backend_get();
  if (type == CAMERA_BACKEND_EAR_CAM) return ear_camera_backend_get();
  if (type == CAMERA_BACKEND_DIPLA_CAM) return dipla_camera_backend_get();
  static const struct camera_backend backends[CAMERA_BACKEND_COUNT] = {
    [CAMERA_BACKEND_LOCAL] = {
      .name = "local", .init = local_pipe_init, .deinit = local_pipe_deinit,
      .soda_application = USHER_SODA, .camera_id = USHER_CAMERA_ALL,
      .capabilities = CAMERA_CAP_IMAGE | CAMERA_CAP_MOCK
    },
    [CAMERA_BACKEND_AI_CAM] = {
      .name = "aicam", .init = ai_cam_pipe_init, .shoot = ai_cam_pipe_shoot,
      .deinit = ai_cam_pipe_deinit, .soda_application = USHER_SODA,
      .configure = ai_cam_pipe_configure,
      .camera_id = USHER_CAMERA_AICAM, .prefix = 'a', .capabilities = CAMERA_CAP_IMAGE | CAMERA_CAP_PRIVATE_IMAGE,
      .mock = {.width = 2028, .height = 1520, .filename_marker = "m"},
      .soda_option = "--aicam", .photo_directory = USHER_AI_CAM_PHOTO_DIR
    },
    [CAMERA_BACKEND_SPOTTER_CAM] = {
      .name = "spottercam", .init = spotter_cam_pipe_init,
      .shoot = spotter_cam_pipe_shoot, .deinit = spotter_cam_pipe_deinit, .soda_application = USHER_SODA,
      .configure = spotter_cam_pipe_configure,
      .camera_id = USHER_CAMERA_AICAM, .prefix = 'a',
      .mock = {.width = 2028, .height = 1520, .filename_marker = "m"},
      .capabilities = CAMERA_CAP_IMAGE | CAMERA_CAP_DETECTION | CAMERA_CAP_FINISH_OWNED | CAMERA_CAP_PRIVATE_IMAGE, .soda_option = "--aicam",
      .bind_frame = spotter_cam_pipe_bind_frame, .detection_summary = spotter_cam_pipe_last_detection_summary,
      .save_detection_copy = spotter_cam_pipe_save_detection_copy, .photo_directory = USHER_SPOTTERCAM_PHOTO_DIR
    },
    [CAMERA_BACKEND_LWIR_CAM] = {
      .name = "lwircam", .init = lwir_cam_pipe_init, .shoot = lwir_cam_pipe_shoot,
      .configure = lwir_cam_pipe_configure,
      .deinit = lwir_cam_pipe_deinit, .soda_application = USHER_SODA, .camera_id = USHER_CAMERA_LWIRCAM,
      .prefix = 'l', .capabilities = CAMERA_CAP_IMAGE | CAMERA_CAP_FINISH_OWNED |
        (LWIR_ENABLE_ANALYSIS ? CAMERA_CAP_GEOLOCATION : 0),
    #if LWIR_ENABLE_MOCK
      .mock = {.width = 256, .height = 192, .filename_marker = "m"},
    #endif
      .soda_option = "--lwircam", .init_cancelable = lwir_cam_pipe_init_cancelable,
      .capture_delay = lwir_cam_pipe_capture_delay, .capture_timing = lwir_cam_pipe_capture_timing,
    #if LWIR_ENABLE_ANALYSIS
      .geolocate = lwir_cam_pipe_geolocate,
    #endif
      .photo_directory = USHER_LWIR_CAM_PHOTO_DIR
    },
  };
  if (type <= CAMERA_BACKEND_UNSELECTED || type >= CAMERA_BACKEND_COUNT) return (struct camera_backend) {0};
  struct camera_backend backend = backends[type];
  if (type == CAMERA_BACKEND_AI_CAM && ai_cam_pipe_native()) backend.soda_application = NULL;
  return backend;
}

enum camera_backend_type camera_backend_registry_route(int camera_id, enum camera_backend_type preferred,
    bool local_only)
{
  if (camera_id < USHER_CAMERA_ALL || camera_id > CAMERA_BACKEND_MAX_ID) return CAMERA_BACKEND_UNSELECTED;
  const struct camera_backend selected = camera_backend_registry_get(preferred, false);
  if (selected.name && selected.camera_id == camera_id)
    return local_only && (selected.capabilities & CAMERA_CAP_IMAGE) ? CAMERA_BACKEND_LOCAL : preferred;
  for (enum camera_backend_type type = CAMERA_BACKEND_CHDK; type < CAMERA_BACKEND_COUNT; ++type) {
    const struct camera_backend backend = camera_backend_registry_get(type, false);
    if (!backend.name || backend.camera_id != camera_id) continue;
    return local_only && (backend.capabilities & CAMERA_CAP_IMAGE) ? CAMERA_BACKEND_LOCAL : type;
  }
  return CAMERA_BACKEND_UNSELECTED;
}

struct camera_backend camera_backend_registry_identity(int camera_id, enum camera_backend_type preferred,
    bool mock)
{
  return camera_backend_registry_get(camera_backend_registry_route(camera_id, preferred, false), mock);
}