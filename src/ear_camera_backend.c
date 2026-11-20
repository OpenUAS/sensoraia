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
 * @file ear_camera_backend.c
 * @brief EAR-owned survey admission, finalization and generic point completions.
 * @details USHER's serial owner calls lifecycle/configuration/poll hooks. One
 * joinable worker solves immutable snapshots and writes evidence; it never owns
 * UART or live pose lookup. A small wake channel signals mailbox state, not data.
 */
#include "ear_camera_backend.h"
#include "ear_cam_pipe.h"
#include "image_exif.h"

#include <errno.h>
#include <limits.h>
#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#ifndef USHER_SODA
  #define USHER_SODA "./bin/soda"
#endif

/** Two jobs match the adapter's two buffers: stopped evidence is never evicted. */
#define EAR_FINALIZE_JOB_COUNT 2
#define EAR_COMPLETION_ENDPOINT_COUNT 2
#define EAR_COMPLETION_DRAIN_BYTES 16
#define EAR_EVIDENCE_PATH_CAPACITY 512
#define EAR_LATITUDE_LIMIT_DEG 90
#define EAR_LONGITUDE_LIMIT_DEG 180
#define EAR_COORDINATE_UNITS_PER_DEG 1e7
#define EAR_MILLIMETERS_PER_METER 1000.0
#define EAR_QUALITY_UNITS_PER_SCORE 1000.0
#define EAR_METRIC_UNITS_PER_DB 100.0
/** Legacy dc_shot groundalt carries AGL in Q8 metres, unlike CAM_POINT_RESULT millimetres. */
#define EAR_SHOT_AGL_UNITS_PER_METER 256.0

/* Serial-owner state is separate from the mutex-protected worker mailbox. Do
 * not call start/stop/poll/configure concurrently from additional owner threads. */
static struct camera_backend_host ear_host;
static bool earcam_active, earcam_unavailable, earcam_session_active, earcam_simulated;
/**
 * @brief One reservation progressing through snapshot -> point -> evidence completion.
 * @details Start reserves a slot before collecting audio. Stop attaches an owned
 * immutable snapshot. Poll consumes the point, but the slot remains occupied
 * until evidence work finishes, preventing reuse while the worker still uses it.
 */
static struct ear_finalize_job {
  struct ear_session_snapshot *snapshot;
  struct ear_loudest_spot spot;
  uint32_t session_id;
  uint64_t order;
  bool in_use, taken, point_ready, evidence_done, simulated;
} ear_finalize_jobs[EAR_FINALIZE_JOB_COUNT];
static struct ear_finalize_job *current_ear_job;
static uint64_t ear_finalize_order;
static pthread_mutex_t ear_finalize_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t ear_finalize_available = PTHREAD_COND_INITIALIZER;
static pthread_t ear_finalize_thread;
static bool ear_finalize_started, ear_finalize_stopping;
static int ear_finalize_wake[EAR_COMPLETION_ENDPOINT_COUNT] = {-1, -1};

/** @brief Guard finite values and integer conversion ranges before marking a wire point valid. */
static bool ear_point_valid(const struct ear_loudest_spot *spot)
{
  return spot && spot->valid && isfinite(spot->lat_deg) && isfinite(spot->lon_deg)
         && fabs(spot->lat_deg) <= EAR_LATITUDE_LIMIT_DEG && fabs(spot->lon_deg) <= EAR_LONGITUDE_LIMIT_DEG
         && (spot->lat_deg != 0 || spot->lon_deg != 0)
         && isfinite(spot->agl_m) && spot->agl_m >= 0 && spot->agl_m <= INT32_MAX / EAR_MILLIMETERS_PER_METER
         && isfinite(spot->alt_m) && spot->alt_m >= INT32_MIN / EAR_MILLIMETERS_PER_METER
         && spot->alt_m <= INT32_MAX / EAR_MILLIMETERS_PER_METER
         && isfinite(spot->confidence) && spot->confidence >= 0 && spot->confidence <= 1
         && isfinite(spot->level_db) && spot->level_db >= INT32_MIN / EAR_METRIC_UNITS_PER_DB
         && spot->level_db <= INT32_MAX / EAR_METRIC_UNITS_PER_DB;
}

/** @brief Notify the serial owner without blocking; mailbox flags remain authoritative if full. */
static void wake_ear_result_owner(void)
{
  const char notification = 1;
  ssize_t result;
  do { result = write(ear_finalize_wake[1], &notification, sizeof(notification)); } while (result < 0 && errno == EINTR);
  if (result < 0 && errno != EAGAIN && errno != EWOULDBLOCK)
    fprintf(stderr, "USHER:\tEAR completion wake failed: %s\n", strerror(errno));
}

/**
 * @brief Drain stopped sessions in admission order until shutdown has no work left.
 * @details The point mailbox is published before potentially slow CSV/JPEG/EXIF
 * and optional SODA submission. Never perform UART sends or live pose lookups here.
 */
static void *finalize_ear_sessions(void *unused)
{
  (void)unused;
  for (;;) {
    pthread_mutex_lock(&ear_finalize_mutex);
    struct ear_finalize_job *job = NULL;
    for (;;) {
      for (size_t slot = 0; slot < EAR_FINALIZE_JOB_COUNT; ++slot) {
        struct ear_finalize_job *candidate = &ear_finalize_jobs[slot];
        if (candidate->in_use && candidate->snapshot && !candidate->taken
            && (!job || candidate->order < job->order)) job = candidate;
      }
      if (job || ear_finalize_stopping) break;
      pthread_cond_wait(&ear_finalize_available, &ear_finalize_mutex);
    }
    if (!job) { pthread_mutex_unlock(&ear_finalize_mutex); return NULL; }
    job->taken = true;
    pthread_mutex_unlock(&ear_finalize_mutex);
    struct ear_loudest_spot spot = {0};
    if (ear_cam_pipe_snapshot_solve(job->snapshot, &spot) != 0) spot.valid = false;
    spot.valid = ear_point_valid(&spot);
    pthread_mutex_lock(&ear_finalize_mutex);
    job->spot = spot;
    job->point_ready = true;
    pthread_mutex_unlock(&ear_finalize_mutex);
    wake_ear_result_owner();

    char sound_picture[EAR_EVIDENCE_PATH_CAPACITY];
    if (ear_cam_pipe_snapshot_render(job->snapshot, job->session_id, &spot,
                                     sound_picture, sizeof(sound_picture), job->simulated) == 0) {
      union dc_shot_union picture_shot = {0};
      picture_shot.data.nr = ear_cam_pipe_snapshot_number(job->snapshot);
      if (spot.valid) {
        picture_shot.data.lat = (int32_t)llround(spot.lat_deg * EAR_COORDINATE_UNITS_PER_DEG);
        picture_shot.data.lon = (int32_t)llround(spot.lon_deg * EAR_COORDINATE_UNITS_PER_DEG);
        picture_shot.data.alt = (int32_t)llround(spot.alt_m * EAR_MILLIMETERS_PER_METER);
        picture_shot.data.groundalt = (int32_t)llround(spot.agl_m * EAR_SHOT_AGL_UNITS_PER_METER);
      }
      if (image_exif_write(sound_picture, &picture_shot) == 0) {
        printf("USHER:\tEAR session %u sound picture %s\n", job->session_id, sound_picture);
        printf("Photo take %d\n", picture_shot.data.nr);
        if (ear_host.queue_analysis) ear_host.queue_analysis(sound_picture, &picture_shot, USHER_CAMERA_EARCAM);
      } else fprintf(stderr, "USHER:\tEAR session %u EXIF failed; image retained: %s\n", job->session_id, sound_picture);
    } else fprintf(stderr, "USHER:\tEAR session %u sound-picture rendering failed\n", job->session_id);
    ear_cam_pipe_snapshot_release(job->snapshot);
    pthread_mutex_lock(&ear_finalize_mutex);
    job->evidence_done = true;
    pthread_mutex_unlock(&ear_finalize_mutex);
    wake_ear_result_owner();
  }
}

/**
 * @brief Consume one completion on the serial owner and convert physical units once.
 * @return True when result receives a correlated valid or invalid point.
 * @details Wake bytes may coalesce; scan mailbox flags even after draining the
 * socket. USHER retains responsibility for token correlation and actual UART delivery.
 */
static bool ear_poll_result(struct camera_point_result *result)
{
  if (!result) return false;
  if (ear_finalize_wake[0] >= 0) {
    char notifications[EAR_COMPLETION_DRAIN_BYTES];
    ssize_t count;
    do { count = read(ear_finalize_wake[0], notifications, sizeof(notifications)); }
    while (count > 0 || (count < 0 && errno == EINTR));
  }
  for (size_t slot = 0; slot < EAR_FINALIZE_JOB_COUNT; ++slot) {
    pthread_mutex_lock(&ear_finalize_mutex);
    struct ear_finalize_job *job = &ear_finalize_jobs[slot];
    const bool ready = job->in_use && job->point_ready;
    const uint32_t session_id = job->session_id;
    const struct ear_loudest_spot spot = job->spot;
    job->point_ready = false;
    if (job->in_use && job->evidence_done) *job = (struct ear_finalize_job) {0};
    pthread_mutex_unlock(&ear_finalize_mutex);
    if (!ready) continue;
    ear_cam_pipe_log_result(session_id, &spot);
    printf("USHER:\tEAR session %u result valid=%d lat=%.7f lon=%.7f conf=%.3f samples=%u/%u\n",
           session_id, spot.valid, spot.lat_deg, spot.lon_deg, spot.confidence, spot.used_count, spot.sample_count);
    *result = (struct camera_point_result) {.session_id = session_id, .valid = ear_point_valid(&spot)};
    if (result->valid) {
      result->lat = (int32_t)llround(spot.lat_deg * EAR_COORDINATE_UNITS_PER_DEG);
      result->lon = (int32_t)llround(spot.lon_deg * EAR_COORDINATE_UNITS_PER_DEG);
      result->alt_mm = (int32_t)llround(spot.alt_m * EAR_MILLIMETERS_PER_METER);
      result->agl_mm = (int32_t)llround(spot.agl_m * EAR_MILLIMETERS_PER_METER);
      result->quality = (int32_t)llround(spot.confidence * EAR_QUALITY_UNITS_PER_SCORE);
      result->metric = (int32_t)llround(spot.level_db * EAR_METRIC_UNITS_PER_DB);
    }
    return true;
  }
  return false;
}

/**
 * @brief Reserve both finalizer capacity and adapter storage before listening.
 * @return False for zero token, shutdown, exhausted capacity or worker/buffer failure.
 * @details Replacing an unfinished active session releases only its reservation;
 * stopped snapshots stay admitted. Generic USHER routing handles duplicate tokens.
 */
static bool ear_start(uint32_t session_id)
{
  if (!session_id) return false;
  earcam_session_active = false;
  if (!earcam_active || !ear_host.running || !capture_running(ear_host.running)) return false;
  pthread_mutex_lock(&ear_finalize_mutex);
  if (current_ear_job) { *current_ear_job = (struct ear_finalize_job) {0}; current_ear_job = NULL; }
  if (ear_finalize_stopping || ear_finalize_order == UINT64_MAX) { pthread_mutex_unlock(&ear_finalize_mutex); return false; }
  if (!ear_finalize_started) {
    if (socketpair(AF_UNIX, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0, ear_finalize_wake) != 0) {
      ear_finalize_wake[0] = ear_finalize_wake[1] = -1;
      pthread_mutex_unlock(&ear_finalize_mutex);
      fprintf(stderr, "USHER:\tEAR finalizer completion channel unavailable: %s\n", strerror(errno));
      return false;
    }
    int result = pthread_create(&ear_finalize_thread, NULL, finalize_ear_sessions, NULL);
    if (result != 0) {
      close(ear_finalize_wake[0]); close(ear_finalize_wake[1]);
      ear_finalize_wake[0] = ear_finalize_wake[1] = -1;
      pthread_mutex_unlock(&ear_finalize_mutex);
      fprintf(stderr, "USHER:\tEAR finalizer unavailable: %s\n", strerror(result));
      return false;
    }
    ear_finalize_started = true;
  }
  for (size_t slot = 0; slot < EAR_FINALIZE_JOB_COUNT; ++slot) {
    if (!ear_finalize_jobs[slot].in_use) {
      current_ear_job = &ear_finalize_jobs[slot];
      *current_ear_job = (struct ear_finalize_job) {
        .session_id = session_id,
        .order = ++ear_finalize_order, .in_use = true, .simulated = earcam_simulated
      };
      break;
    }
  }
  pthread_mutex_unlock(&ear_finalize_mutex);
  if (!current_ear_job) return false;
  if (ear_cam_pipe_begin_session(session_id)) { earcam_session_active = true; return true; }
  pthread_mutex_lock(&ear_finalize_mutex);
  *current_ear_job = (struct ear_finalize_job) {0}; current_ear_job = NULL;
  pthread_mutex_unlock(&ear_finalize_mutex);
  return false;
}

/**
 * @brief Validate the active token before a zero-copy snapshot handoff.
 * @return True only when an immutable session was queued for the worker.
 * @details Failed detach releases the reservation; a wrong token cannot stop listening.
 */
static bool ear_stop(uint32_t session_id)
{
  const bool finalize = earcam_active && earcam_session_active && current_ear_job
                        && current_ear_job->session_id == session_id;
  if (!finalize) return false;
  earcam_session_active = false;
  struct ear_session_snapshot *snapshot = ear_cam_pipe_detach_session();
  pthread_mutex_lock(&ear_finalize_mutex);
  if (snapshot) current_ear_job->snapshot = snapshot;
  else *current_ear_job = (struct ear_finalize_job) {0};
  current_ear_job = NULL;
  pthread_cond_signal(&ear_finalize_available);
  pthread_mutex_unlock(&ear_finalize_mutex);
  return snapshot != NULL;
}

/**
 * @brief Close admission and join all admitted stopped-session evidence work.
 * @details Completed point mailboxes remain pollable after the wake channel closes.
 * This is orderly draining, not a hard filesystem-I/O deadline or power-loss guarantee.
 */
static void ear_drain(void)
{
  earcam_session_active = false;
  pthread_mutex_lock(&ear_finalize_mutex);
  ear_finalize_stopping = true;
  pthread_cond_signal(&ear_finalize_available);
  pthread_mutex_unlock(&ear_finalize_mutex);
  if (ear_finalize_started) pthread_join(ear_finalize_thread, NULL);
  ear_finalize_started = false;
  for (size_t index = 0; index < EAR_COMPLETION_ENDPOINT_COUNT; ++index) {
    if (ear_finalize_wake[index] >= 0) close(ear_finalize_wake[index]);
    ear_finalize_wake[index] = -1;
  }
}

/** @brief Whether the serial owner should admit EAR samples for the current session. */
static bool ear_listening(void) { return earcam_active && earcam_session_active; }
/** @brief Nonblocking completion descriptor, or -1 before start/after drain. */
static int ear_completion_fd(void) { return ear_finalize_wake[0]; }
/** @brief Copy startup-only host callbacks; worker-side queue_analysis must be thread-safe. */
static void ear_configure(const struct camera_backend_host *host)
{
  if (!host || earcam_active || ear_finalize_started) return;
  ear_host = *host;
  ear_cam_pipe_set_pose_lookup(host->pose_lookup);
}
/** @brief Initialize supervision, not microphone readiness; hard setup failure stays latched. */
static int ear_init(const char *source)
{
  if (earcam_active) return 0;
  if (earcam_unavailable) return -1;
  if (ear_cam_pipe_init(source) != 0) { earcam_unavailable = true; return -1; }
  earcam_active = true;
  printf("USHER:\tacoustic backend: earcam\n");
  return 0;
}
/** @brief Drain before tearing down audio/storage, then reset quiescent finalizer state. */
static void ear_deinit(void)
{
  ear_drain();
  struct camera_point_result discarded;
  while (ear_poll_result(&discarded)) {}
  ear_cam_pipe_deinit();
  pthread_mutex_lock(&ear_finalize_mutex);
  for (size_t slot = 0; slot < EAR_FINALIZE_JOB_COUNT; ++slot) ear_finalize_jobs[slot] = (struct ear_finalize_job) {0};
  current_ear_job = NULL;
  ear_finalize_order = 0;
  ear_finalize_stopping = false;
  pthread_mutex_unlock(&ear_finalize_mutex);
  earcam_active = earcam_session_active = false;
}

bool ear_camera_backend_set_device(const char *device)
{
  return !earcam_active && !ear_finalize_started && ear_cam_pipe_set_device(device);
}

void ear_camera_backend_set_simulated_source(double lat_deg, double lon_deg, double level_db_at_1m)
{
  earcam_simulated = true;
  ear_cam_pipe_set_simulated_source(lat_deg, lon_deg, level_db_at_1m);
}
void ear_camera_backend_set_band(double low_cut_hz, double high_cut_hz) { ear_cam_pipe_set_band(low_cut_hz, high_cut_hz); }

struct camera_backend ear_camera_backend_get(void)
{
  static const struct camera_session_ops session = {
    .start = ear_start, .stop = ear_stop, .listening = ear_listening,
    .completion_fd = ear_completion_fd, .poll_result = ear_poll_result, .drain = ear_drain
  };
  return (struct camera_backend) {
    .name = "earcam", .init = ear_init, .deinit = ear_deinit,
                                        .soda_application = USHER_SODA, .camera_id = USHER_CAMERA_EARCAM, .prefix = 'e',
                                                                                                          .capabilities = CAMERA_CAP_POINT_SESSION, .soda_option = "--earcam", .record = ear_cam_pipe_record,
                                                                                                                                                                               .session = &session, .configure = ear_configure, .pose_updated = ear_cam_pipe_update_pose
  };
}