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
 * @file loom_camera_backend.c
 * @brief Own LOOM's AIcam specialization outside the universal USHER service.
 * @details The capture owner and saved-photo worker have separate lifecycles:
 * optional matching starts without acquiring IMX500, while physical capture is
 * lazy. A failed matcher never disables ordinary photograph capture.
 */
#include "loom_camera_backend.h"
#include "loom_cam.h"
#include "imx500_pipe.h"

#include <stdio.h>
#include <string.h>

#ifndef USHER_AI_CAM_PHOTO_DIR
  /** @brief Capture destination default; backend naming remains LOOM-owned. */
  #define USHER_AI_CAM_PHOTO_DIR "~/Pictures/sensoraia"
#endif
#ifndef USHER_SODA
  /** @brief Existing post-capture analysis executable, not the LOOM matcher. */
  #define USHER_SODA "./bin/soda"
#endif

static const char *reference_directory = USHER_LOOMCAM_REFERENCE_DIR; /**< Borrowed until worker init copies it. */
static const char *network_model = USHER_LOOMCAM_MODEL; /**< Borrowed RPK configuration for native capture lifetime. */
static struct imx500_pipe capture_owner = IMX500_PIPE_INITIALIZER; /**< LOOM's own neutral native worker, not detector state. */
static const char *inference_settings_path;

/** @brief Pass the service cancellation flag to the native capture owner.
 * @param host Nullable backend host with a running flag valid through capture.
 * @details Borrow only the flag; the host object itself need not persist. */
static void loom_capture_configure(const struct camera_backend_host *host)
{
  capture_owner.running = host ? host->running : NULL;
}

int loom_camera_backend_set_settings(const char *settings)
{
  if (!settings || !settings[0] || capture_owner.pid > 0) return -1;
  if (loom_cam_set_settings(settings)) return -1;
  inference_settings_path = settings;
  return 0;
}

void loom_camera_backend_configure(const char *references, const char *model)
{
  reference_directory = references;
  network_model = model;
}

/** @brief Select the native capture protocol only when this owner is acquired.
 * @param source Reserved common-backend input; native capture uses the configured RPK.
 * @return Zero after the camera child reports READY, otherwise -1.
 * @details Stream startup may happen earlier without camera acquisition. Delaying
 * physical ownership until capture keeps saved-photo processing independent. */
static int loom_capture_init(const char *source)
{
  (void)source;
  capture_owner.command = USHER_LOOMCAM_COMMAND;
  capture_owner.model = network_model;
  capture_owner.settings = inference_settings_path ? inference_settings_path :
    network_model && !strcmp(network_model, USHER_LOOMCAM_MODEL) ? "-" : NULL;
  capture_owner.directory = USHER_AI_CAM_PHOTO_DIR;
  capture_owner.prefix = LOOM_IMAGE_PREFIX;
  return imx500_pipe_start(&capture_owner);
}

/** @brief Request pixels through the neutral supervisor without detection parsing.
 * @param filename Caller-owned destination buffer.
 * @param capacity Writable path bytes including the terminator.
 * @param number FC photo number used only for the output filename.
 * @return Zero for a qualified saved image, otherwise -1. */
static int loom_capture_shoot(char *filename, size_t capacity, int number)
{
  return imx500_pipe_shoot(&capture_owner, filename, capacity, number);
}

/** @brief Stop only LOOM's capture child; the saved-photo stream drains separately. */
static void loom_capture_deinit(void)
{
  imx500_pipe_stop(&capture_owner);
}

/** @brief Start optional saved-photo processing without acquiring another sensor.
 * @return Zero for worker startup, otherwise -1; photograph capture remains a separate capability. */
static int loom_stream_init(void)
{
  const int result = loom_cam_init(reference_directory, network_model);
  if (result != 0)
    fprintf(stderr, "LOOM_cam: worker unavailable; photos continue without visual navigation\n");
  return result;
}

struct camera_backend loom_camera_backend_get(bool mock)
{
  static const struct camera_point_stream_ops stream = {
    .init = loom_stream_init, .poll_result = loom_cam_poll_result,
    .drain = loom_cam_deinit, .start = loom_cam_start, .stop = loom_cam_stop,
    .enabled = loom_cam_enabled, .completion_fd = loom_cam_completion_fd
  };
  const struct camera_backend backend = {
    /* LOOM consumes feature/frame evidence, not object boxes. Keep detection
     * hooks absent instead of making an empty object result a compatibility token. */
    .name = "loomcam", .camera_id = USHER_CAMERA_AICAM, .prefix = LOOM_IMAGE_PREFIX,
    .init = loom_capture_init, .shoot = loom_capture_shoot,
    .configure = loom_capture_configure,
    .deinit = loom_capture_deinit, .soda_application = USHER_SODA,
    .capabilities = mock ? CAMERA_CAP_IMAGE : CAMERA_CAP_IMAGE | CAMERA_CAP_FINISH_OWNED | CAMERA_CAP_PRIVATE_IMAGE,
    .soda_option = "--aicam", .photo_directory = USHER_AI_CAM_PHOTO_DIR,
    .mock = {.width = 2028, .height = 1520, .filename_marker = ""},
    .image_saved = loom_cam_image_saved, .point_stream = &stream
  };
  return backend;
}