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
 * @file spotter_camera_backend.c
 * @brief Native Spottercam backend and optional C11 UART target-slot projection.
 * @details Projects validated vehicle detections through a measured PANGEA_CAMERA_V1
 * pinhole profile onto the five ranked geographic target slots (0..4 -> T1..T5)
 * and publishes correlated USHER_CAM_POINT_UPDATE frames without Python helpers.
 */
#include "spotter_camera_backend.h"
#include "camera_profile.h"
#include "coverage_scheduler.h"
#include "spotter_cam_pipe.h"
#include "../modules/spotter_cam/spotter_targets.h"

#include <errno.h>
#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include <sys/eventfd.h>
#include <unistd.h>

#ifndef USHER_SODA
  #define USHER_SODA "./bin/soda"
#endif
#ifndef USHER_AI_CAM_PHOTO_DIR
  #define USHER_AI_CAM_PHOTO_DIR "~/Pictures/sensoraia"
#endif
#ifndef USHER_SPOTTERCAM_PHOTO_DIR
  #define USHER_SPOTTERCAM_PHOTO_DIR USHER_AI_CAM_PHOTO_DIR
#endif

#define SPOTTER_POINT_QUEUE_CAPACITY 16U

static pthread_mutex_t target_mutex = PTHREAD_MUTEX_INITIALIZER;
static struct usher_camera_profile target_profile;
static bool target_profile_loaded;
static struct spotter_targets target_store;
static struct spotter_detections parsed_detections;
static struct camera_point_result pending_points[SPOTTER_POINT_QUEUE_CAPACITY];
static size_t pending_head;
static size_t pending_count;
static uint32_t last_admitted_shot;
static int notification_fd = -1;

/** @brief Clear the event hint while holding @c target_mutex.
 * @details The result queue, not the event counter, owns pending work. Clear
 * the counter only while producers are excluded and the queue is empty. */
static void drain_notification_locked(void)
{
  if (notification_fd < 0) return;
  eventfd_t value;
  int status;
  do {
    status = eventfd_read(notification_fd, &value);
  } while (status < 0 && errno == EINTR);
}

/** @brief Wake the event loop after queuing a result under @c target_mutex.
 * @details An eventfd wake is a readability hint, not a second result queue;
 * a saturated counter already has the desired readable state. */
static void notify_locked(void)
{
  if (notification_fd < 0) return;
  const uint64_t increment = 1U;
  ssize_t written;
  do {
    written = write(notification_fd, &increment, sizeof(increment));
  } while (written < 0 && errno == EINTR);
}

bool spotter_camera_backend_set_targets_profile(const char *filename)
{
  struct usher_camera_profile candidate;
  if (!filename || !filename[0] || !usher_camera_profile_read(filename, &candidate)
      || candidate.camera_id != USHER_CAMERA_AICAM
      || candidate.calibration.width < 2U || candidate.calibration.height < 2U) {
    return false;
  }
  pthread_mutex_lock(&target_mutex);
  target_profile = candidate;
  target_profile_loaded = true;
  pthread_mutex_unlock(&target_mutex);
  return true;
}

bool spotter_camera_backend_targets_enabled(void)
{
  return target_profile_loaded && !spotter_cam_pipe_photo_only();
}

/** @brief Reset target accumulation and prepare completion notification.
 * @return Zero with a usable event descriptor, or -1 on eventfd failure.
 * @details Reset shot ordering with the result ring so a prior stream's
 * observations cannot be mistaken for this stream's output. */
static int spotter_stream_init(void)
{
  pthread_mutex_lock(&target_mutex);
  spotter_targets_reset(&target_store);
  pending_head = 0U;
  pending_count = 0U;
  last_admitted_shot = 0U;
  if (notification_fd < 0) {
    notification_fd = eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
  }
  const int result = notification_fd >= 0 ? 0 : -1;
  pthread_mutex_unlock(&target_mutex);
  return result;
}

/** @brief Discard pending target results and close their notification channel.
 * @details Serialize descriptor lifetime with producers so they cannot write
 * to a closed or subsequently reused descriptor. */
static void spotter_stream_drain(void)
{
  pthread_mutex_lock(&target_mutex);
  pending_head = 0U;
  pending_count = 0U;
  if (notification_fd >= 0) {
    close(notification_fd);
    notification_fd = -1;
  }
  pthread_mutex_unlock(&target_mutex);
}

/** @brief Get the stream's borrowed poll descriptor.
 * @return Event descriptor, or -1 if the stream has none.
 * @details The lock protects lookup, not the lifetime after return; the
 * event-loop owner coordinates polling with teardown. */
static int spotter_stream_completion_fd(void)
{
  pthread_mutex_lock(&target_mutex);
  const int fd = notification_fd;
  pthread_mutex_unlock(&target_mutex);
  return fd;
}

/** @brief Pop one queued Spottercam point.
 * @param[out] result Destination, unchanged if no point is available.
 * @return True when a point was copied.
 * @details Keep the event descriptor readable while more points remain, and
 * clear it under the producer lock once the last point is removed. */
static bool spotter_stream_poll_result(struct camera_point_result *result)
{
  if (!result) return false;
  bool available = false;
  pthread_mutex_lock(&target_mutex);
  if (pending_count > 0U) {
    *result = pending_points[pending_head];
    pending_head = (pending_head + 1U) % SPOTTER_POINT_QUEUE_CAPACITY;
    --pending_count;
    available = true;
  }
  if (pending_count == 0U) {
    drain_notification_locked();
  }
  pthread_mutex_unlock(&target_mutex);
  return available;
}

/** @brief Convert a saved image's detections into stream observations.
 * @param filename Published JPEG used to locate matching detector evidence.
 * @param shot Pose and photo number associated with that image.
 * @details Reject non-increasing shot IDs and invalid evidence before
 * updating the target store. Rescale intrinsics to the detection raster
 * before projecting; these are flat-ground estimates, not surveyed positions. */
static void spotter_image_saved(const char *filename, const union dc_shot_union *shot)
{
  if (!spotter_camera_backend_targets_enabled() || !filename || !shot || shot->data.nr <= 0) {
    return;
  }
  const uint32_t frame_number = (uint32_t)shot->data.nr;
  pthread_mutex_lock(&target_mutex);
  if (frame_number <= last_admitted_shot
      || !spotter_cam_pipe_read_detections(filename, &parsed_detections)
      || parsed_detections.count == 0U
      || parsed_detections.width < 2U || parsed_detections.width > 4096U
      || parsed_detections.height < 2U || parsed_detections.height > 4096U) {
    pthread_mutex_unlock(&target_mutex);
    return;
  }
  struct coverage_camera camera = target_profile.calibration;
  const double scale_x = (double)parsed_detections.width / (double)camera.width;
  const double scale_y = (double)parsed_detections.height / (double)camera.height;
  camera.fx *= scale_x;
  camera.fy *= scale_y;
  /* Preserve the principal point's pixel-center convention during resize. */
  camera.cx = (camera.cx + 0.5) * scale_x - 0.5;
  camera.cy = (camera.cy + 0.5) * scale_y - 0.5;
  camera.width = parsed_detections.width;
  camera.height = parsed_detections.height;

  const double deg_per_rad = 57.29577951308232;
  const double rad_per_deg = 0.017453292519943295;
  const double earth_radius_m = 6378137.0;
  const double agl_m = shot->data.groundalt > 0 ? shot->data.groundalt / 256.0 : shot->data.alt / 1000.0;
  const struct coverage_pose pose = {
    .latitude_deg = shot->data.lat / 1e7,
    .longitude_deg = shot->data.lon / 1e7,
    .agl_m = agl_m,
    .roll_deg = shot->data.phi / 4096.0 * deg_per_rad,
    .pitch_deg = shot->data.theta / 4096.0 * deg_per_rad,
    .yaw_deg = shot->data.psi / 4096.0 * deg_per_rad,
    .uncertainty_m = 0.0
  };
  const double cos_lat = cos(pose.latitude_deg * rad_per_deg);
  if (!isfinite(cos_lat) || cos_lat <= 1e-6) {
    pthread_mutex_unlock(&target_mutex);
    return;
  }

  for (unsigned index = 0U; index < parsed_detections.count; ++index) {
    const struct spotter_detection *box = &parsed_detections.objects[index];
    const double cx = box->precise_center ? box->center_x : box->x + 0.5 * box->width;
    const double cy = box->precise_center ? box->center_y : box->y + 0.5 * box->height;
    struct coverage_point offset;
    if (!coverage_project_pixel(&camera, &pose, cx, cy, &offset)) {
      continue;
    }
    const double target_lat = pose.latitude_deg + (offset.north / earth_radius_m) * deg_per_rad;
    const double target_lon = remainder(
      pose.longitude_deg + (offset.east / (earth_radius_m * cos_lat)) * deg_per_rad, 360.0);
    if (!isfinite(target_lat) || !isfinite(target_lon)
        || fabs(target_lat) > 90.0 || fabs(target_lon) > 180.0) {
      continue;
    }
    const long long lat_e7 = llround(target_lat * 1e7);
    const long long lon_e7 = llround(target_lon * 1e7);
    const long long ground_alt_mm = (long long)shot->data.alt - llround(agl_m * 1000.0);
    const long long score = llround(box->confidence * 1000.0);
    if ((lat_e7 == 0 && lon_e7 == 0) || score < 0 || score > 1000
        || ground_alt_mm < INT32_MIN || ground_alt_mm > INT32_MAX) {
      continue;
    }
    const struct spotter_target observation = {
      .latitude = (int32_t)lat_e7,
      .longitude = (int32_t)lon_e7,
      .altitude_mm = (int32_t)ground_alt_mm,
      .score = (uint16_t)score,
      .valid = true
    };
    const int slot = spotter_targets_offer(&target_store, &observation, NULL, NULL);
    if (slot < 0) {
      continue;
    }
    last_admitted_shot = frame_number;
    const struct camera_point_result update = {
      .session_id = frame_number,
      .valid = true,
      .lat = observation.latitude,
      .lon = observation.longitude,
      .alt_mm = observation.altitude_mm,
      .agl_mm = 0,
      .quality = observation.score,
      .metric = slot
    };
    if (pending_count < SPOTTER_POINT_QUEUE_CAPACITY) {
      const size_t tail = (pending_head + pending_count) % SPOTTER_POINT_QUEUE_CAPACITY;
      pending_points[tail] = update;
      ++pending_count;
    } else {
      pending_points[pending_head] = update;
      pending_head = (pending_head + 1U) % SPOTTER_POINT_QUEUE_CAPACITY;
    }
    notify_locked();
    printf("SPOTTER_TARGETS:\tshot %u assigned T%d lat=%.7f lon=%.7f score=%u\n",
           frame_number, slot + 1, observation.latitude / 1e7, observation.longitude / 1e7,
           (unsigned)observation.score);
    break;
  }
  pthread_mutex_unlock(&target_mutex);
}

struct camera_backend spotter_camera_backend_get(void)
{
  static const struct camera_point_stream_ops stream = {
    .init = spotter_stream_init,
    .poll_result = spotter_stream_poll_result,
    .drain = spotter_stream_drain,
    .enabled = spotter_camera_backend_targets_enabled,
    .completion_fd = spotter_stream_completion_fd
  };
  struct camera_backend backend = {
    .name = "spottercam",
    .init = spotter_cam_pipe_init,
    .shoot = spotter_cam_pipe_shoot,
    .deinit = spotter_cam_pipe_deinit,
    .soda_application = USHER_SODA,
    .configure = spotter_cam_pipe_configure,
    .camera_id = USHER_CAMERA_AICAM,
    .prefix = 'a',
    .mock = {.width = 2028, .height = 1520, .filename_marker = "m"},
    .capabilities = CAMERA_CAP_IMAGE | CAMERA_CAP_DETECTION | CAMERA_CAP_FINISH_OWNED | CAMERA_CAP_PRIVATE_IMAGE,
    .soda_option = "--aicam",
    .bind_frame = spotter_cam_pipe_bind_frame,
    .detection_summary = spotter_cam_pipe_last_detection_summary,
    .save_detection_copy = spotter_cam_pipe_save_detection_copy,
    .photo_directory = USHER_SPOTTERCAM_PHOTO_DIR
  };
  if (spotter_cam_pipe_photo_only()) {
    backend.capabilities &= ~(CAMERA_CAP_DETECTION | CAMERA_CAP_FINISH_OWNED);
    backend.soda_application = NULL;
    backend.bind_frame = NULL;
    backend.detection_summary = NULL;
    backend.save_detection_copy = NULL;
  } else if (target_profile_loaded) {
    backend.image_saved = spotter_image_saved;
    backend.point_stream = &stream;
  }
  return backend;
}
