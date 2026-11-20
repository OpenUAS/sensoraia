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
 * @file chdk_camera_backend.c
 * @brief Adapt CHDKbridge capture and lens controls to USHER's backend contract.
 * @details This layer resolves the configured photo directory and owns naming;
 * the subprocess layer owns USB/camera transactions. It advertises only the
 * controls for which CHDKbridge can return command-specific evidence.
 */
#include "chdk_camera_backend.h"
#include "chdk_pipe.h"
#include "camera_control.h"
#include "path_utils.h"
#include <errno.h>
#include <limits.h>
#include <stdio.h>
#include <sys/stat.h>
#include <unistd.h>

#ifndef USHER_CHDK_PHOTO_DIR
#define USHER_CHDK_PHOTO_DIR "~/Pictures/sensoraia"
#endif
#ifndef USHER_SODA
#define USHER_SODA "./bin/soda"
#endif
static const char *photo_directory = USHER_CHDK_PHOTO_DIR;

/** @brief Pass service cancellation to CHDKbridge supervision.
 * @param host Nullable backend host whose running flag outlives capture work.
 * @details Retain only the flag; the backend does not need the host's other callbacks. */
static void chdk_backend_configure(const struct camera_backend_host *host)
{
  chdk_pipe_set_running(host ? host->running : NULL);
}

void chdk_camera_backend_set_photo_directory(const char *directory)
{
  photo_directory = directory ? directory : USHER_CHDK_PHOTO_DIR;
}

/** @brief Prepare capture storage and initialize the CHDK controller path.
 * @param source_image Unused fixture argument from the shared backend interface.
 * @return Zero for successful setup, or -1 on storage/controller setup failure.
 * @details Reject unusable output storage before advertising capture readiness. */
static int chdk_backend_init(const char *source_image)
{
  (void)source_image;
  char resolved[PATH_MAX];
  const char *directory = usher_resolve_path(photo_directory, resolved, sizeof(resolved));
  if (usher_ensure_directory(directory) != 0) return -1;
  struct stat status;
  if (stat(directory, &status) != 0 || !S_ISDIR(status.st_mode) || access(directory, W_OK) != 0) {
    fprintf(stderr, "USHER: CHDK photo directory is not writable: %s\n", directory);
    return -1;
  }
  return chdk_pipe_init();
}

/** @brief Capture one CHDK JPEG at the path derived from its shot number.
 * @param[out] filename Output path; cleared on failure when writable.
 * @param capacity Path buffer size including the terminator.
 * @param number Nonnegative photo number.
 * @return Zero for a completed capture, or -1 on invalid input or capture failure.
 * @details Pass the path to the controller only if it fits so truncated
 * names cannot misassociate a saved image with another shot. */
static int chdk_backend_shoot(char *filename, size_t capacity, int number)
{
  if (!filename || !capacity) return -1;
  filename[0] = '\0';
  if (number < 0) return -1;
  char resolved[PATH_MAX];
  const char *directory = usher_resolve_path(photo_directory, resolved, sizeof(resolved));
  int length = snprintf(filename, capacity, "%s/c%06d.jpg", directory, number);
  if (length < 0 || (size_t)length >= capacity || chdk_pipe_shoot(filename) != 0) {
    filename[0] = '\0';
    return -1;
  }
  return 0;
}

struct camera_backend chdk_camera_backend_get(void)
{
  const struct camera_backend backend = {
    .name = "chdk", .init = chdk_backend_init, .shoot = chdk_backend_shoot, .configure = chdk_backend_configure,
    .deinit = chdk_pipe_deinit, .soda_application = USHER_SODA,
    .camera_id = USHER_CAMERA_CHDK, .prefix = 'c', .capabilities = CAMERA_CAP_IMAGE,
    .control_capabilities = CAM_CONTROL_CAP(CAM_CMD_ZOOM_ABSOLUTE) | CAM_CONTROL_CAP(CAM_CMD_LENS_RETRACT),
    .control_init = chdk_pipe_init, .control = chdk_pipe_control,
    .mock = {.width = 1280, .height = 960, .filename_marker = "m"},
    .soda_option = "--chdkcam", .photo_directory = photo_directory
  };
  return backend;
}
