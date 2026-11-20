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
 * @file camera_backend.h
 * @brief Shared capability and lifecycle contract for USHER camera backends.
 * @details USHER owns scheduling, UART admission, and common photo completion;
 * a backend owns only its device/process work and advertises optional behavior
 * through capabilities. Keeping this boundary explicit prevents a camera
 * adapter from silently taking over unrelated backends' capture semantics.
 */
#ifndef USHER_CAMERA_BACKEND_H
#define USHER_CAMERA_BACKEND_H

#include <signal.h>
#include "camera_capture.h"
#include "capture_timing.h"

enum { CAMERA_BACKEND_MAX_ID = 8 };

/** @brief Backend features and shared completion ownership.
 * @details CAMERA_CAP_PRIVATE_IMAGE returns paths from usher_prepare_photo.
 * Shared completion publishes them only after all metadata writes succeed.
 * Direct capture callers must likewise complete or retain those private files. */
enum camera_capability {
  CAMERA_CAP_IMAGE = 1U << 0,
  CAMERA_CAP_DETECTION = 1U << 1,
  CAMERA_CAP_GEOLOCATION = 1U << 2,
  CAMERA_CAP_POINT_SESSION = 1U << 3,
  CAMERA_CAP_FINISH_OWNED = 1U << 4,
  CAMERA_CAP_MOCK = 1U << 5,
  CAMERA_CAP_PRIVATE_IMAGE = 1U << 6,
  CAMERA_CAP_POINT_STREAM = 1U << 7
};

struct camera_point_result {
  uint32_t session_id;
  bool valid;
  int32_t lat, lon, alt_mm, agl_mm, quality, metric;
};

struct camera_session_ops {
  bool (*start)(uint32_t session_id);
  bool (*stop)(uint32_t session_id);
  bool (*listening)(void);
  int (*completion_fd)(void);
  bool (*poll_result)(struct camera_point_result *result);
  void (*drain)(void);
};

/** @brief Repeated correlated point updates from saved photos or non-image work.
 * @details Result session_id is the positive FC photo number for this stream;
 * the transport uses its explicit frame_number field, not a session token. */
struct camera_point_stream_ops {
  /** Initialize optional analysis independently of lazy sensor acquisition. */
  int (*init)(void);
  bool (*poll_result)(struct camera_point_result *result);
  void (*drain)(void);
  bool (*start)(uint32_t session_id);
  bool (*stop)(uint32_t session_id);
  bool (*enabled)(void);
  int (*completion_fd)(void);
};

struct camera_backend_host {
  /** @brief Borrowed shutdown flag; backend workers must stop when it clears. */
  const volatile sig_atomic_t *running;
  /** @brief Bounded pose lookup supplied by USHER's shared timing owner. */
  bool (*pose_lookup)(struct capture_observation *observation, uint64_t maximum_age_us);
  /** @brief Queue optional analysis after image ownership has been established. */
  void (*queue_analysis)(const char *filename, const union dc_shot_union *shot, int camera_id);
};

/**
 * @brief Function table describing one backend's owned camera operations.
 * @details Required capture hooks are selected by the advertised capability
 * bits. Optional sessions, point streams, controls, and analysis hooks remain
 * backend-owned; USHER does not infer support from a camera ID or name.
 */
struct camera_backend {
  /** @brief Stable backend name used for logs and diagnostics. */
  const char *name;
  /** @brief Initialize backend resources; optional mock image path is borrowed. */
  int (*init)(const char *source_image);
  /** @brief Trigger one photo and return the backend's owned staging path. */
  int (*shoot)(char *filename, size_t filename_size, int image_number);
  /** @brief Release backend resources and reap owned children. */
  void (*deinit)(void);
  /** @brief Optional SODA application name for this backend's saved files. */
  const char *soda_application;
  /** @brief Optional split-phase capture lifecycle. */
  const struct camera_capture_ops *capture_ops;
  uint8_t camera_id;
  char prefix;
  /** Backend-owned geometry and namespace for shared simulation capture. */
  struct {
    size_t width, height;
    const char *filename_marker;
  } mock;
  unsigned capabilities;
  const char *soda_option;
  int (*init_cancelable)(const char *source_image, const volatile sig_atomic_t *running);
  int (*record)(const union dc_shot_union *shot);
  const struct camera_session_ops *session;
  void (*configure)(const struct camera_backend_host *host);
  void (*pose_updated)(void);
  double (*capture_delay)(void);
  struct capture_timing(*capture_timing)(void);
  int (*geolocate)(char *filename);
  bool (*bind_frame)(uint64_t sequence, uint64_t sensor_timestamp_ns);
  void (*detection_summary)(char *buffer, size_t size);
  void (*save_detection_copy)(const char *filename);
  const char *photo_directory;
  /** Optional post-publication hook; may enqueue work but must not wait for inference. */
  void (*image_saved)(const char *filename, const union dc_shot_union *shot);
  /** @brief Optional repeated point stream, independent of start/stop sessions. */
  const struct camera_point_stream_ops *point_stream;
  /** @brief Bitset of controls supported by this backend; zero means none. */
  uint32_t control_capabilities;
  /** @brief Initialize control-only ownership without forcing photo capture setup. */
  int (*control_init)(void);
  /** @brief Execute one validated control and return command-specific evidence. */
  uint8_t (*control)(uint8_t command, uint8_t value, uint8_t *observed, uint8_t *evidence);
};

#endif