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
 * @file usher.c
 * @brief Main USHER daemon coordinating flight-controller triggers and camera backends.
 * @details The event loop receives validated UART frames, queues bounded capture jobs,
 * dispatches backend work on detached workers, records metadata, and returns status.
 * It deliberately keeps serial I/O separate from slow camera processes so one capture
 * cannot block command reception. Backend failure is isolated per camera wherever
 * possible; systemd owns whole-daemon restart for irrecoverable transport failures.
 *
 * @par Ownership at a glance
 * The event loop owns protocol dispatch and survey admission. Optical tickets
 * serialize CHDK/AIcam access; live LWIR has a separate worker and recovery lock.
 * A split-phase image retains its backend completion table after the optical lane
 * is released. Live SODA uses a separate single worker and owns filenames only.
 * Keep these boundaries distinct: accepting a trigger, owning pixels and saving a
 * geotagged original are three different events.
 * @warning Bounded workers prevent software backlog growth; they do not isolate
 * shared power, storage, kernel stalls or the whole board's memory consumption.
 */
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <errno.h>
#include <fcntl.h>
#include "version.h"
#include <math.h>
#include <poll.h>
#include <signal.h>
#include <spawn.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/un.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

#include "ai_cam_pipe.h"
#include "spotter_cam_pipe.h"
#include "lwir_spotter.h"
#include "camera_speedtest.h"
#include "ear_camera_backend.h"
#include "ear_cam_pipe.h"
#include "loom_camera_backend.h"
#include "camera_backend_registry.h"
#include "dipla_camera_backend.h"
#include "chdk_camera_backend.h"
#include "spotter_camera_backend.h"
#include "lwir_cam_pipe.h"
#include "serial.h"
#include "serial_tx.h"
#include "image_exif.h"
#include "image_mock_transform.h"
#include "image_lens.h"
#include "local_pipe.h"
#include "usher_protocol.h"
#include "camera_control.h"
#include <sys/random.h>
#include "pose_log.h"
#include "capture_arbiter.h"
#include "camera_profile.h"
#include "socket.h"
#include "path_utils.h"

#define MAX_FILENAME 512
#define MAX_PROCESSING_THREADS 8
#define USHER_MAX_ARGUMENTS 128
#define USHER_MAX_PARSED_ARGUMENTS (USHER_MAX_ARGUMENTS * 2 + 1)
#ifndef USHER_SODA_TIMEOUT_MS
  #define USHER_SODA_TIMEOUT_MS 120000
#endif
#ifndef USHER_SODA_IDLE_MS
  #define USHER_SODA_IDLE_MS 2000
#endif
#ifndef USHER_SODA_RETRY_MS
  #define USHER_SODA_RETRY_MS 5000
#endif
#ifndef SODA_BACKLOG_CAPACITY
  #define SODA_BACKLOG_CAPACITY 1024
#endif
#define MAX_IMAGE_BUFFERS 25
#define IMAGE_SIZE 70
// Search&Rescue Onboard Detection Application
#define SODA "./bin/soda"
#define ANGLE_BFP_SCALE 4096.0
#define SPEED_BFP_SCALE 524288.0
#define POSITION_BFP_SCALE 256.0
#define RAD_TO_DEG 57.29577951308232
#define DEBUG_HEARTBEAT_SECONDS 5
#define SERIAL_READ_BUFFER_SIZE 256
#define EVENT_LOOP_TIMEOUT_MS 1000
#define EVENT_LOOP_ACTIVE_TIMEOUT_MS 20
#define MAX_BACKEND_RESULTS_PER_PASS 16
#define SODA_SHUTDOWN_GRACE_MS 2000
#define SODA_WAIT_INTERVAL_US 10000
#ifndef LWIR_RETRY_COOLDOWN_US
  #define LWIR_RETRY_COOLDOWN_US 30000000U
#endif
#define LWIR_RETRY_POLL_US 100000

#ifndef USHER_SERIAL_DEVICE
  #define USHER_SERIAL_DEVICE "/dev/ttySAC0"
#endif

#ifndef USHER_LOCAL_SIM_DEVICE
  #define USHER_LOCAL_SIM_DEVICE "/tmp/usher-sim"
#endif

struct option_alias {
  const char *name;
  char option[3];
  bool requires_argument;
};

/** @brief Translate supported long options into the short-option parser's input.
 * @param argc Original argument count.
 * @param argv Original argument vector.
 * @param[out] normalized Output vector borrowing strings from @p argv and @p aliases.
 * @param capacity Number of pointer slots including room for the terminator.
 * @param aliases Accepted long names, short spellings and operand rules.
 * @param alias_count Number of entries in @p aliases.
 * @return Normalized count, or -1 on invalid input or insufficient capacity.
 * @details Split inline operands without copying their text and preserve
 * arguments after the option terminator. Keep both input tables alive. */
static int normalize_long_options(int argc, char *const argv[], char *normalized[],
                                  size_t capacity, const struct option_alias aliases[],
                                  size_t alias_count)
{
  if (argc < 1 || argc > USHER_MAX_ARGUMENTS || capacity == 0) {
    return -1;
  }
  size_t output_count = 0;
  normalized[output_count++] = argv[0];
  for (int index = 1; index < argc; ++index) {
    const char *argument = argv[index];
    if (strcmp(argument, "--") == 0) {
      if (output_count >= capacity) {
        return -1;
      }
      normalized[output_count++] = argv[index];
      for (++index; index < argc; ++index) {
        if (output_count >= capacity) {
          return -1;
        }
        normalized[output_count++] = argv[index];
      }
      break;
    }
    if (strncmp(argument, "--", 2) != 0 || argument[2] == '\0') {
      if (output_count >= capacity) {
        return -1;
      }
      normalized[output_count++] = argv[index];
      continue;
    }
    const char *name = argument + 2;
    size_t name_length = strcspn(name, "=");
    const char *inline_value = name[name_length] == '=' ? name + name_length + 1 : NULL;
    const struct option_alias *alias = NULL;
    for (size_t alias_index = 0; alias_index < alias_count; ++alias_index) {
      if (strlen(aliases[alias_index].name) == name_length
          && strncmp(name, aliases[alias_index].name, name_length) == 0) {
        alias = &aliases[alias_index];
        break;
      }
    }
    if (alias == NULL || (!alias->requires_argument && inline_value != NULL)
        || output_count >= capacity) {
      return -1;
    }
    normalized[output_count++] = (char *)alias->option;
    if (alias->requires_argument) {
      if (inline_value == NULL) {
        if (++index >= argc) {
          return -1;
        }
        inline_value = argv[index];
      }
      if (output_count >= capacity) {
        return -1;
      }
      normalized[output_count++] = (char *)inline_value;
    }
  }
  if (output_count >= capacity) {
    return -1;
  }
  normalized[output_count] = NULL;
  return (int)output_count;
}

#ifndef USHER_LOCAL_APP_DEVICE
  #define USHER_LOCAL_APP_DEVICE "/tmp/usher-app"
#endif

#ifndef USHER_LOCAL_LOCK_FILE
  #define USHER_LOCAL_LOCK_FILE "/tmp/usher-local.lock"
#endif

#ifndef USHER_MOCK_IMAGE
  #define USHER_MOCK_IMAGE "resources/mock_image_01.jpg"
#endif

#ifndef USHER_CHDK_PHOTO_DIR
  #define USHER_CHDK_PHOTO_DIR "~/Pictures/sensoraia"
#endif

#ifndef USHER_AI_CAM_PHOTO_DIR
  #define USHER_AI_CAM_PHOTO_DIR "~/Pictures/sensoraia"
#endif

#ifndef USHER_SPOTTERCAM_PHOTO_DIR
  #define USHER_SPOTTERCAM_PHOTO_DIR USHER_AI_CAM_PHOTO_DIR
#endif

#ifndef USHER_LWIR_CAM_PHOTO_DIR
  #define USHER_LWIR_CAM_PHOTO_DIR "~/Pictures/sensoraia"
#endif

#ifndef USHER_SODA
  #define USHER_SODA "./bin/soda"
#endif

#ifndef USHER_LOCAL_PHOTO_DIR
  #define USHER_LOCAL_PHOTO_DIR "~/Pictures/sensoraia"
#endif

/** @brief Pool-owned worker request; ownership transfers only after thread creation succeeds. */
struct capture_job {
  union dc_shot_union shot;
  uint32_t request_id;
  uint8_t camera_mask;
  uint64_t ticket; /**< Optical ordering token, not a physical exposure sequence. */
  uint64_t received_us; /**< Local receipt time for queue age, never an exposure timestamp. */
  bool arbiter_owned; /**< Protect admitted survey ownership from legacy newest-waiting replacement. */
  bool shot_counted;
};

/** @brief Per-image handoff from acquisition to publication, EXIF and optional analysis.
 * @details filename may still be empty while pending owns pixels. Keep the original
 * backend hooks with the image: another camera may have been selected by finish time.
 */
struct captured_image {
  struct camera_backend backend;
  char filename[MAX_FILENAME];
  int camera_id;
  double capture_delay;
  struct capture_timing capture_times;
  union dc_shot_union exposure_shot;
  double pose_uncertainty_m;
  bool exposure_pose_valid; /**< Only validated frame evidence permits replacing request-time geotags. */
  struct camera_pending pending;
};

/** @brief Saved-file analysis job; retaining pixels here would multiply decoder memory.
 * @details source binds the queued pose to the original file. The ledger is
 * session-only, and a retry may repeat partially completed analysis side effects.
 */
struct soda_job {
  char filename[MAX_FILENAME];
  union dc_shot_union shot;
  struct capture_timing timing;
  bool spotting_done;
  int camera_id;
  struct stat source;
  uint64_t queued_us, retry_after_us;
  unsigned int attempts;
};

static void *handle_msg_shoot(void *ptr);
static bool capture_image(const union dc_shot_union *shoot, struct captured_image *image,
                          int camera_id, const struct camera_backend *backend);
static bool finish_image(const union dc_shot_union *shoot, struct captured_image *image);
static void handle_received_message(void);
static void start_shoot_worker(const union dc_shot_union *shoot, uint8_t camera_mask, uint32_t request_id);
static void start_lwir_worker(const union dc_shot_union *shoot, uint32_t request_id, uint64_t received_us);
static void service_capture_arbiter(uint64_t now_us);
static void capture_arbiter_failed(const union dc_shot_union *shoot, uint8_t mask);
static void handle_targeted_stop(uint32_t camera_id, uint32_t session_id);
static void handle_targeted_start(uint32_t camera_id, uint32_t session_id);
static bool prepare_session_backend(unsigned camera_id);
static bool camera_point_valid(const struct camera_point_result *point);
static void send_msg_cam_point(uint32_t camera_id, uint32_t session_id, const struct camera_point_result *point);
static void flush_cam_points(void);
static void service_camera_backends(void);
static void drain_camera_backends(void);
static void send_msg_cam_point_update(uint32_t camera_id, const struct camera_point_result *point);
static void flush_cam_point_updates(void);
static bool loomcam_enabled;
static const char *loomcam_references = USHER_LOOMCAM_REFERENCE_DIR;
static const char *loomcam_model = USHER_LOOMCAM_MODEL;
static union usher_cam_point_update_union point_updates[CAMERA_BACKEND_MAX_ID];
static bool point_update_pending[CAMERA_BACKEND_MAX_ID];
static bool backend_service_pending;
static int run_soda(const char *filename, const union dc_shot_union *shoot, int camera_id);
static void defer_soda(const char *filename, const union dc_shot_union *shoot, int camera_id);
static void defer_saved_analysis(const char *filename, const union dc_shot_union *shoot, int camera_id,
                                  const struct capture_timing *timing);
static void stop_soda_backlog(void);
static void start_pangea(void);
static void stop_pangea(void);
static void release_worker_slot(void);
static struct capture_job *capture_job_acquire_locked(void);
static void capture_job_release_locked(struct capture_job *job);
static void capture_job_release(struct capture_job *job);
static inline void send_msg_image_buffer(void);
static inline void send_msg_status(void);
static void print_usage(const char *program);
static int lock_local_instance(void);
static pid_t start_local_serial_bridge(void);
static int wait_for_path(const char *path);
static void stop_local_serial_bridge(void);
static void handle_signal(int signal_number);
static void notify_systemd_ready(void);
static int camera_backend_select(enum camera_backend_type type, bool test_mode);
static struct camera_backend camera_backend_get(enum camera_backend_type type);
static struct camera_backend camera_identity_get(int camera_id);
static void camera_close_owner(unsigned camera_id);
static int camera_register_stream(const struct camera_backend *backend);
static enum camera_backend_type camera_type_for_id(int camera_id);
static int camera_prepare(int camera_id);
static bool lwir_ready_or_start_recovery(void);
static void lwir_mark_backoff(void);
static void cameras_deinit(void);
static int local_backend_shoot(char *filename, size_t filename_size, int image_number);

static volatile int is_shooting, image_idx, image_count, shooting_count, shooting_thread_count;
static char image_buffer[MAX_IMAGE_BUFFERS][IMAGE_SIZE];
static pthread_mutex_t mut = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t shot_result_mutex = PTHREAD_MUTEX_INITIALIZER;
#define SHOT_RESULT_CAPACITY 128
static union usher_shot_result_union shot_results[SHOT_RESULT_CAPACITY];
static size_t shot_result_head, shot_result_count;
static uint64_t shot_result_dropped;
/* mut protects scheduling transitions, not slow camera operations. LWIR's lock
 * also covers geolocation because its backend exposes one last-capture record. */
static pthread_mutex_t lwir_camera_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t camera_available = PTHREAD_COND_INITIALIZER;
static pthread_cond_t workers_finished = PTHREAD_COND_INITIALIZER;
static uint64_t next_job_ticket, next_capture_ticket;
#define CAPTURE_JOB_CAPACITY (MAX_PROCESSING_THREADS + 1)
static struct capture_job capture_jobs[CAPTURE_JOB_CAPACITY];
static bool capture_job_used[CAPTURE_JOB_CAPACITY];
static struct capture_job *waiting_capture;
static struct capture_job chdk_capture_job;
static bool chdk_capture_in_progress;
/* Second CHDK stage: EXIF and publication run here so the next capture can start meanwhile. */
static struct {
  struct capture_job job;
  struct captured_image image;
} chdk_finish;
static bool chdk_finish_in_progress;
static bool chdk_async_capture_enabled = true;
static struct usher_control_request chdk_control_job;
static struct {
  struct usher_control_result result;
  bool dirty;
} control_results[8];
static uint32_t control_session, control_nonce, control_high_water;
static bool control_confirmed;
static uint64_t control_negotiated_us;
static void handle_control_message(void);
static void flush_control_results(void);
static uint64_t chdk_retry_after_us;
#ifndef CHDK_RETRY_COOLDOWN_US
#define CHDK_RETRY_COOLDOWN_US UINT64_C(5000000)
#endif
static pthread_mutex_t live_soda_mutex = PTHREAD_MUTEX_INITIALIZER;
static uint64_t soda_busy_deferred_count;
static struct soda_job soda_backlog[SODA_BACKLOG_CAPACITY];
static size_t soda_backlog_head, soda_backlog_count;
static uint64_t last_photo_request_us;
static uint64_t soda_backlog_completed, soda_backlog_failed, soda_backlog_overflow;
static pthread_t soda_backlog_thread;
static pthread_cond_t soda_available = PTHREAD_COND_INITIALIZER;
static bool soda_backlog_started, soda_backlog_active, soda_backlog_stop;
static uint64_t superseded_capture_count;
static int event_wake_reader = -1;
static volatile sig_atomic_t event_wake_writer = -1;

/** @brief Prompt the event loop to recheck shared state.
 * @details The nonblocking self-pipe is a hint, not a counted event queue:
 * if it is full, an unread byte already makes it readable. Preserve errno
 * because a signal may interrupt code that is about to inspect it. */
static void wake_event_loop(void)
{
  const int error = errno;
  const uint8_t notification = 1;
  if (event_wake_writer >= 0) {
    ssize_t written;
    do { written = write(event_wake_writer, &notification, sizeof(notification)); } while (written < 0 && errno == EINTR);
  }
  errno = error;
}

/** @brief Set up the self-pipe before installing its producers.
 * @return Zero on success, or -1 with errno on failure.
 * @details Set both ends nonblocking and close-on-exec before publishing
 * either descriptor so workers cannot see a partly configured pipe. */
static int event_wake_init(void)
{
  if (event_wake_reader >= 0 || event_wake_writer >= 0) { errno = EBUSY; return -1; }
  int descriptors[2];
  if (pipe(descriptors) != 0) return -1;
  for (size_t index = 0; index < 2; ++index) {
    const int flags = fcntl(descriptors[index], F_GETFL);
    if (flags < 0 || fcntl(descriptors[index], F_SETFL, flags | O_NONBLOCK) < 0
        || fcntl(descriptors[index], F_SETFD, FD_CLOEXEC) < 0) {
      const int error = errno;
      close(descriptors[0]);
      close(descriptors[1]);
      errno = error;
      return -1;
    }
  }
  event_wake_reader = descriptors[0];
  event_wake_writer = descriptors[1];
  return 0;
}

/** @brief Consume pending wake hints without monopolizing the event loop.
 * @details Cap the drain work; if writers refill the pipe, the remaining
 * bytes will make the next poll readable. */
static void drain_event_wake(void)
{
  uint8_t notifications[64];
  for (unsigned attempt = 0; attempt < 4; ++attempt)
    if (read(event_wake_reader, notifications, sizeof(notifications)) <= 0) break;
}

/** @brief Withdraw the wake descriptors and close the self-pipe.
 * @details Invalidate the published writer first so later notifications
 * can skip the write. Call after ordinary worker producers are quiescent. */
static void event_wake_close(void)
{
  const int writer = event_wake_writer;
  event_wake_writer = -1;
  if (writer >= 0) close(writer);
  if (event_wake_reader >= 0) close(event_wake_reader);
  event_wake_reader = -1;
}

/** @brief Hand one camera's terminal shot result to the serial owner.
 * @param request_id Correlation ID; zero denotes a legacy request without an acknowledgement.
 * @param number Flight-controller photo number.
 * @param camera_id Camera that owns the outcome.
 * @param status Wire result code.
 * @details Do not block a capture worker on UART backpressure. Count drops
 * if the bounded queue fills, and wake the loop even for legacy work. */
static void queue_shot_result(uint32_t request_id, int32_t number, unsigned camera_id, uint32_t status)
{
  if (!request_id) { wake_event_loop(); return; }
  pthread_mutex_lock(&shot_result_mutex);
  if (shot_result_count == SHOT_RESULT_CAPACITY) ++shot_result_dropped;
  else {
    union usher_shot_result_union *result = &shot_results[(shot_result_head + shot_result_count++) % SHOT_RESULT_CAPACITY];
    *result = (union usher_shot_result_union) {.data = {request_id, number, camera_id, status}};
  }
  pthread_mutex_unlock(&shot_result_mutex);
  wake_event_loop();
}

/** @brief Report a shared outcome separately for each addressed image camera.
 * @param request_id Correlation ID, or zero for a legacy request.
 * @param number Flight-controller photo number.
 * @param camera_mask Bitmask indexed by camera ID minus one.
 * @param status Outcome to report for each selected camera.
 * @details Expand only image-camera bits so a shared admission failure
 * retains per-camera correlation without generating terminal results for
 * non-image backends. */
static void queue_image_results(uint32_t request_id, int32_t number, uint8_t camera_mask, uint32_t status)
{
  for (unsigned camera_id = USHER_CAMERA_CHDK; camera_id <= USHER_CAMERA_LWIRCAM; ++camera_id)
    if (camera_mask & (1U << (camera_id - 1))) queue_shot_result(request_id, number, camera_id, status);
}

/** @brief Transfer a bounded batch of shot results to the UART transmit queue.
 * @details The event loop is the only consumer. Leave the head queued until
 * transmit admission succeeds, and release the result lock around that call
 * so a worker can append without waiting on the UART path. */
static void flush_shot_results(void)
{
  for (unsigned sent = 0; sent < 16; ++sent) {
    pthread_mutex_lock(&shot_result_mutex);
    if (!shot_result_count) {
      pthread_mutex_unlock(&shot_result_mutex);
      return;
    }
    const union usher_shot_result_union result = shot_results[shot_result_head];
    pthread_mutex_unlock(&shot_result_mutex);
    if (serial_tx_send(USHER_SHOT_RESULT, result.bin, sizeof(result.bin)) != 0) return;
    pthread_mutex_lock(&shot_result_mutex);
    shot_result_head = (shot_result_head + 1) % SHOT_RESULT_CAPACITY;
    --shot_result_count;
    pthread_mutex_unlock(&shot_result_mutex);
  }
}
static bool local_mode;
static enum camera_backend_type requested_camera_backend = CAMERA_BACKEND_UNSELECTED;
static struct camera_backend camera;
static bool camera_initialized[CAMERA_BACKEND_MAX_ID + 1];
static struct camera_backend camera_owners[CAMERA_BACKEND_MAX_ID + 1];
static bool camera_unavailable[4];
static bool camera_warned[4];
/** Tiny1-C server startup can take tens of seconds. Recovery runs outside the ordered
 * optical capture lane and is rate-limited so a broken USB link cannot stall AIcam. */
static uint64_t lwir_retry_after_us;
static bool lwir_recovery_in_progress;
static bool lwir_capture_in_progress;
static bool lwir_handoff_in_progress;
static uint64_t lwir_active_received_us;
static struct {
  union dc_shot_union shot;
  uint32_t request_id;
  uint64_t received_us;
  bool valid;
} waiting_lwir;
#define LWIR_WAIT_MAX_US UINT64_C(2000000)
static bool lwir_async_recovery_enabled = true;
static uint64_t lwir_busy_skipped_count;
static bool local_capture_only;
/** True when --spottercam was selected: camera id USHER_CAMERA_AICAM still means "the
 * AI camera" on the wire (unchanged for the flight controller/NPS), but camera_prepare()
 * and cameras_deinit() route it to the object-detection backend instead of plain
 * ai_cam_pipe capture. Plain --aicam behavior is completely unaffected. */
static const char *camera_source_image;
static bool local_bridge_requested;
static bool local_bridge_owned;
static volatile sig_atomic_t keep_running = 1;

/** @brief Read service cancellation state through the capture flag accessor.
 * @return True while the service is running.
 * @details Use the same accessor as the workers so the main loop and
 * subprocess supervisors observe the same shutdown flag. */
static bool usher_running(void)
{
  return capture_running(&keep_running);
}
static pid_t socat_pid = -1;
static int local_lock_fd = -1;
static const char *mock_image = USHER_MOCK_IMAGE;
static bool mock_transform_enabled;
static bool mock_map_enabled;
static const char *mock_map_cache = "~/.local/share/paparazzi/pprzgcs/map";
static double mock_map_horizontal_fov = 60.0;
static double mock_map_vertical_fov = 45.0;
static const char *mock_lens_profile;
static const char *pangea_directory;
static const char *pangea_camera_profile;
static const char *pangea_pause_file;
static pid_t pangea_pid = -1;
static bool debug_enabled;
static bool test_capture_enabled;
static bool earcam_requested;
static struct {
  uint32_t id;
  union usher_cam_point_result_union result;
  bool pending;
  bool stop_requested;
} point_sessions[CAMERA_BACKEND_MAX_ID];
static int optical_camera_id = USHER_CAMERA_ALL;
static uint64_t serial_receive_monotonic_us;
static struct clock_alignment fc_clock;
static bool clock_probes_enabled;
#ifndef USHER_POSE_LOG_DIR
  #define USHER_POSE_LOG_DIR NULL
#endif
static const char *pose_log_dir = USHER_POSE_LOG_DIR;
static bool motion_compensation_enabled;
static struct {
  uint64_t minimum_us, maximum_us;
  bool valid;
} exposure_latency[4];
static enum camera_backend_type configured_camera_backend = CAMERA_BACKEND_UNSELECTED;
/** Optional analysis ownership is separate from lazy sensor acquisition. */
enum camera_stream_state { CAMERA_STREAM_CLOSED, CAMERA_STREAM_ACTIVE, CAMERA_STREAM_UNAVAILABLE, CAMERA_STREAM_DRAINED };
static enum camera_stream_state camera_stream_states[CAMERA_BACKEND_MAX_ID + 1];
static uint64_t next_clock_probe_us, clock_probe_count, clock_reply_count, clock_rejected_count;

/** @brief Read USHER's monotonic clock for local latency and event ordering.
 * @return Microseconds since an arbitrary monotonic origin, or zero on clock failure. */
static uint64_t monotonic_time_us(void)
{
  struct timespec now;
  return clock_gettime(CLOCK_MONOTONIC, &now) == 0
         ? (uint64_t)now.tv_sec * 1000000U + (uint64_t)now.tv_nsec / 1000U : 0;
}

/** @brief Select the next event-loop polling budget.
 * @return Active or idle timeout, in milliseconds.
 * @details Keep the shorter timeout while work may need servicing even
 * without another UART input. Read capture/control state and shot results
 * under their respective locks instead of nesting them. */
static int event_loop_timeout_ms(void)
{
  if (backend_service_pending || capture_arbiter_pending()) return EVENT_LOOP_ACTIVE_TIMEOUT_MS;
  pthread_mutex_lock(&mut);
  bool pending = shooting_thread_count > 0 || chdk_capture_in_progress || chdk_finish_in_progress
    || lwir_capture_in_progress || lwir_handoff_in_progress || lwir_recovery_in_progress;
  for (size_t index = 0; index < CAMERA_BACKEND_MAX_ID; ++index)
    if (point_sessions[index].id || point_sessions[index].pending || point_update_pending[index]) pending = true;
  for (size_t index = 0; index < 8; ++index)
    if (control_results[index].dirty) pending = true;
  pthread_mutex_unlock(&mut);
  pthread_mutex_lock(&shot_result_mutex);
  pending = pending || shot_result_count != 0;
  pthread_mutex_unlock(&shot_result_mutex);
  return pending ? EVENT_LOOP_ACTIVE_TIMEOUT_MS : EVENT_LOOP_TIMEOUT_MS;
}

/** @brief Pause for an interval using nanosleep's remaining-time result.
 * @param microseconds Requested interval in microseconds.
 * @details On signal interruption, resume only the unfinished portion. */
static void sleep_microseconds(uint64_t microseconds)
{
  struct timespec delay = {
    .tv_sec = (time_t)(microseconds / 1000000U),
    .tv_nsec = (long)(microseconds % 1000000U) * 1000L
  };
  while (nanosleep(&delay, &delay) != 0 && errno == EINTR) continue;
}

/** @brief Install a camera's calibrated exposure-to-delivery delay bounds.
 * @param camera_id Supported image camera identifier.
 * @param value Comma-separated minimum and maximum delays in milliseconds.
 * @return True if both bounds were accepted; false leaves the setting unchanged.
 * @details Round outward when converting to integer microseconds so unit
 * conversion never narrows the supplied uncertainty interval. */
static bool set_exposure_latency(unsigned camera_id, const char *value)
{
  double minimum_ms, maximum_ms;
  char extra;
  if (camera_id < USHER_CAMERA_CHDK || camera_id > USHER_CAMERA_LWIRCAM || !value
      || sscanf(value, "%lf,%lf%c", &minimum_ms, &maximum_ms, &extra) != 2
      || !isfinite(minimum_ms) || !isfinite(maximum_ms)
      || minimum_ms < 0 || maximum_ms < minimum_ms || maximum_ms > 2000) return false;
  exposure_latency[camera_id].minimum_us = (uint64_t)floor(minimum_ms * 1000);
  exposure_latency[camera_id].maximum_us = (uint64_t)ceil(maximum_ms * 1000);
  exposure_latency[camera_id].valid = true;
  return true;
}

/** @brief Opportunistically send a low-priority flight-controller clock probe.
 * @details Probes are rate-limited to one per second and sent only on an idle UART so
 * alignment evidence cannot delay capture/status traffic. */
static void send_clock_probe(uint64_t now_us)
{
  if (!clock_probes_enabled || now_us == 0 || now_us < next_clock_probe_us
      || now_us > UINT64_MAX - 1000000) return;
  next_clock_probe_us = now_us + 1000000;
  union usher_clock_request_union token = {.data = {
      .token_low = (uint32_t)now_us,
      .token_high = (uint32_t)(now_us >> 32) ^ (uint32_t)getpid() ^ (uint32_t)(clock_probe_count + 1)
    }
  };
  const uint64_t sent_us = monotonic_time_us();
  if (!clock_alignment_request(&fc_clock, token, sent_us)) return;
  if (serial_tx_send_if_idle(USHER_CLOCK_REQUEST, token.bin, sizeof(token.bin)) != 0) {
    fc_clock.pending = false;
    return;
  }
  ++clock_probe_count;
}

/** @brief Feed one acoustic sample from the pose stream while a listening session is open.
 * @details Sampling follows the ordinary pose stream, so a specialized camera needs no
 * capture commands of its own. Without that stream the session collects nothing. */
static void camera_record_pose(const uint8_t *payload, uint16_t length)
{
  if (length != USHER_POSE_SAMPLE_MSG_SIZE) return;
  union usher_pose_sample_union sample;
  for (size_t index = 0; index < sizeof(sample.bin); ++index) sample.bin[index] = payload[index];
  for (unsigned camera_id = 1; camera_id <= CAMERA_BACKEND_MAX_ID; ++camera_id) {
    const struct camera_backend *owner = &camera_owners[camera_id];
    if (owner->session && owner->session->listening && owner->session->listening() && owner->record)
      owner->record(&sample.data.shot);
  }
}

/** @brief Validate and enqueue a token-bound pose sample with clock evidence.
 * @details Mapping failure does not discard the pose: the log records it as unmapped,
 * preserving raw diagnostic evidence without claiming false time precision. */
static void record_clocked_pose(void)
{
  if (usher_protocol.payload_len != USHER_POSE_CLOCKED_MSG_SIZE) {
    pose_log_record(NULL, 0, serial_receive_monotonic_us);
    return;
  }
  union usher_pose_clocked_union message;
  for (size_t index = 0; index < sizeof(message.bin); ++index) message.bin[index] = usher_protocol.payload[index];
  struct pose_clock_evidence evidence = {.token = message.data.request};
  struct clock_interval begin, end;
  if (clock_alignment_map(&fc_clock, message.data.request, message.data.sample.data.sample_begin_us,
                          serial_receive_monotonic_us, &begin)
      && clock_alignment_map(&fc_clock, message.data.request, message.data.sample.data.sample_end_us,
                             serial_receive_monotonic_us, &end)) {
    evidence.sample_time = (struct clock_interval) {begin.earliest_us, end.latest_us};
    evidence.probe_sent_us = fc_clock.mapped_sent_us;
    evidence.probe_received_us = fc_clock.received_us;
    evidence.probe_receive_fc_us = fc_clock.receive_fc_us;
    evidence.probe_transmit_fc_us = fc_clock.anchor_fc_us;
    evidence.mapped = true;
    capture_arbiter_pose_sample(&message.data.sample, begin.earliest_us, end.latest_us);
  } else {
    capture_arbiter_pose_invalidate();
  }
  pose_log_record_clocked(message.data.sample.bin, sizeof(message.data.sample.bin),
                          serial_receive_monotonic_us, &evidence);
  for (unsigned camera_id = 1; camera_id <= CAMERA_BACKEND_MAX_ID; ++camera_id)
    if (camera_owners[camera_id].pose_updated) camera_owners[camera_id].pose_updated();
  camera_record_pose(message.data.sample.bin, sizeof(message.data.sample.bin));
}

extern char **environ;

/** @brief Configure USHER and run its multiplexed UART/local-UDP event loop.
 * @return Process status suitable for systemd restart policy.
 * @details After argument validation, startup claims the fixed local payload endpoint
 * before touching shared UART or camera resources. This rejects a second process early
 * when the systemd service is already active. USHER then validates selected backends,
 * starts optional persistent services, initializes non-blocking transport, and drains
 * serial bytes into the protocol parser. Shutdown wakes and drains detached workers
 * before releasing backend resources. */
int main(int argc, char *argv[])
{
  if (usher_enter_executable_directory() != 0) {
    perror("USHER: cannot enter executable directory");
    return 1;
  }
  const char *serial_device = USHER_SERIAL_DEVICE;
  bool serial_was_selected = false;
  bool mock_image_was_selected = false;
  bool mock_map_option_selected = false;
  static char resolved_mock_map_cache[PATH_MAX];
  static char resolved_mock_lens_profile[PATH_MAX];
  static char resolved_pangea_profile[PATH_MAX];
  bool test_mode = false;
  bool speedtest_enabled = false;
  bool native_photo_selected = false;
  bool earcam_option_selected = false;
  bool earcam_device_selected = false;
  bool earcam_sim_selected = false;
  bool pose_log_selected = false;
  bool clock_align_selected = false;
  bool clock_align_disabled = false;
  bool lwir_raw_selected = false;
  bool lwir_calibration_selected = false;
  bool lwir_config_selected = false;
  bool spotter_settings_selected = false;
  bool lwir_spotter_geometry_selected = false;
  bool motion_compensation_selected = false;
  bool capture_arbiter_enforce_selected = false;
  uint64_t serial_byte_count = 0;
  uint64_t valid_frame_count = 0;
  uint64_t rejected_frame_count = 0;
  time_t next_debug_heartbeat = 0;
  unsigned char serial_buffer[SERIAL_READ_BUFFER_SIZE];
  int option;

  static const struct option_alias long_options[] = {
    {"serial", "-s", true},
    {"local", "-l", false},
    {"chdk", "-c", false},
    {"aicam", "-a", false},
    {"spottercam", "-b", false},
    {"native-photo", "-n", false},
    {"spottercam-settings", "-I", true},
    {"spottercam-targets", "-4", true},
    {"lwir-spotter-settings", "-1", true},
    {"lwir-spotter-profile", "-2", true},
    {"lwir-spotter-limits", "-3", true},
    {"loomcam", "-H", false},
    {"loomcam-reference-dir", "-R", true},
    {"loomcam-model", "-T", true},
    {"loomcam-settings", "-Y", true},
    {"lwircam", "-w", false},
    {"lwir-config", "-Z", true},
    {"earcam", "-e", false},
    {"diplacam-config", "-J", true},
    {"earcam-device", "-A", true},
    {"earcam-settings", "-W", true},
    {"earcam-sim", "-E", true},
    {"earcam-band", "-B", true},
    {"test", "-t", false},
    {"mocktransform", "-f", false},
    {"debug", "-d", false},
    {"mock-image", "-i", true},
    {"mock-map", "-M", false},
    {"mock-map-cache", "-C", true},
    {"mock-map-fov", "-F", true},
    {"mock-lens", "-L", true},
    {"pangea", "-P", true},
    {"pangea-camera", "-G", true},
    {"pangea-pause-file", "-U", true},
    {"survey-camera", "-Q", true},
    {"survey-enforce", "-N", false},
    {"survey-overlap", "-O", true},
    {"pose-log", "-p", true},
    {"clock-align", "-k", false},
    {"no-clock-align", "-K", false},
    {"lwir-latency-ms", "-j", true},
    {"chdk-latency-ms", "-q", true},
    {"lwir-raw", "-r", false},
    {"lwir-calibration", "-y", true},
    {"lwir-motion-compensation", "-m", false},
    {"help", "-h", false},
    {"version", "-V", false},
    {"speedtest", "-S", false}
  };
  char *normalized_argv[USHER_MAX_PARSED_ARGUMENTS];
  int normalized_argc = normalize_long_options(argc, argv, normalized_argv,
                        USHER_MAX_PARSED_ARGUMENTS, long_options,
                        sizeof(long_options) / sizeof(long_options[0]));
  if (normalized_argc < 0) {
    fprintf(stderr, "USHER:\tinvalid or excessive command-line arguments\n");
    return 2;
  }
  optind = 1;
  while ((option = getopt(normalized_argc, normalized_argv,
                          "s:lcabnweA:E:B:tfdi:p:kKj:q:ry:mhMC:F:L:P:G:U:g:u:Q:NO:VSHR:T:Z:J:I:Y:W:1:2:3:4:")) != -1) {
    switch (option) {
      case '4':
        if (!spotter_camera_backend_set_targets_profile(optarg)) {
          fprintf(stderr, "USHER:\t--spottercam-targets requires a valid AIcam PANGEA_CAMERA_V1 profile\n");
          return 2;
        }
        break;
      case '1':
        if (!lwir_spotter_settings(optarg)) return 2;
        break;
      case '2':
        if (!lwir_spotter_profile(optarg)) return 2;
        lwir_spotter_geometry_selected = true;
        break;
      case '3':
        if (!lwir_spotter_limits(optarg)) return 2;
        lwir_spotter_geometry_selected = true;
        break;
      case 'I':
        if (spotter_cam_pipe_set_settings(optarg)) return 2;
        spotter_settings_selected = true;
        break;
      case 'W':
        if (!ear_cam_pipe_set_settings(optarg)) return 2;
        break;
      case 'Y':
        if (loom_camera_backend_set_settings(optarg)) return 2;
        loomcam_enabled = true;
        break;
      case 'J':
        if (!dipla_camera_backend_setup(optarg)) return 2;
        break;
      case 'H':
        loomcam_enabled = true;
        break;
      case 'R':
        if (!*optarg) return 2;
        loomcam_references = optarg;
        loomcam_enabled = true;
        break;
      case 'T':
        if (!*optarg) return 2;
        loomcam_model = optarg;
        loomcam_enabled = true;
        break;
      case 'Q':
        if (!capture_arbiter_load_camera(optarg)) return 2;
        clock_probes_enabled = true;
        break;
      case 'N':
        capture_arbiter_set_enforce(true);
        capture_arbiter_enforce_selected = true;
        break;
      case 'O': {
        char *end = NULL;
        const double overlap = strtod(optarg, &end);
        if (!end || end == optarg || *end || !capture_arbiter_set_overlap(overlap)) {
          fprintf(stderr, "USHER:\t--survey-overlap expects a fraction from 0 to 0.9\n");
          return 2;
        }
        break;
      }
      case 's':
        serial_device = optarg;
        serial_was_selected = true;
        break;
      case 'l':
        local_mode = true;
        local_bridge_requested = true;
        break;
      case 'c':
        if (requested_camera_backend != CAMERA_BACKEND_UNSELECTED) {
          fprintf(stderr, "USHER:\tonly one camera backend may be selected\n");
          return 2;
        }
        requested_camera_backend = CAMERA_BACKEND_CHDK;
        break;
      case 'a':
        if (requested_camera_backend != CAMERA_BACKEND_UNSELECTED) {
          fprintf(stderr, "USHER:\tonly one camera backend may be selected\n");
          return 2;
        }
        requested_camera_backend = CAMERA_BACKEND_AI_CAM;
        break;
      case 'b':
        if (requested_camera_backend != CAMERA_BACKEND_UNSELECTED) {
          fprintf(stderr, "USHER:\tonly one camera backend may be selected\n");
          return 2;
        }
        requested_camera_backend = CAMERA_BACKEND_SPOTTER_CAM;
        break;
      case 'n':
        native_photo_selected = true;
        break;
      case 'w':
        if (requested_camera_backend != CAMERA_BACKEND_UNSELECTED) {
          fprintf(stderr, "USHER:\tonly one camera backend may be selected\n");
          return 2;
        }
        requested_camera_backend = CAMERA_BACKEND_LWIR_CAM;
        break;
      case 'e':
        earcam_requested = true;
        earcam_option_selected = true;
        break;
      case 'A':
        if (!ear_camera_backend_set_device(optarg)) {
          fprintf(stderr, "USHER:\t--earcam-device expects a nonempty ALSA PCM name of at most 255 bytes\n");
          return 2;
        }
        earcam_requested = true;
        earcam_option_selected = true;
        earcam_device_selected = true;
        break;
      case 'E': {
        double sim_lat = 0.0;
        double sim_lon = 0.0;
        double sim_level = 95.0;
        int fields = sscanf(optarg, "%lf,%lf,%lf", &sim_lat, &sim_lon, &sim_level);
        if (fields < 2 || sim_lat < -90.0 || sim_lat > 90.0 || sim_lon < -180.0 || sim_lon > 180.0) {
          fprintf(stderr, "USHER:\t--earcam-sim expects LAT,LON[,LEVEL_DB_AT_1M]\n");
          return 2;
        }
        ear_camera_backend_set_simulated_source(sim_lat, sim_lon, sim_level);
        earcam_sim_selected = true;
        earcam_requested = true;
        earcam_option_selected = true;
        break;
      }
      case 'B': {
        double low_hz = 0.0;
        double high_hz = 0.0;
        if (sscanf(optarg, "%lf,%lf", &low_hz, &high_hz) != 2 || low_hz < 100.0 || high_hz <= low_hz
            || high_hz > 20000.0) {
          fprintf(stderr, "USHER:\t--earcam-band expects LOW_HZ,HIGH_HZ\n");
          return 2;
        }
        ear_camera_backend_set_band(low_hz, high_hz);
        earcam_option_selected = true;
        break;
      }
      case 't':
        test_mode = true;
        break;
      case 'f':
        mock_transform_enabled = true;
        break;
      case 'd':
        debug_enabled = true;
        break;
      case 'i':
        mock_image = optarg;
        mock_image_was_selected = true;
        break;
      case 'p':
        if (optarg[0] == '\0') {
          fprintf(stderr, "USHER:\t--pose-log expects a directory\n");
          return 2;
        }
        pose_log_dir = optarg;
        pose_log_selected = true;
        break;
      case 'M':
        mock_map_enabled = true;
        break;
      case 'C':
        mock_map_cache = optarg;
        mock_map_option_selected = true;
        break;
      case 'L':
        mock_lens_profile = optarg;
        break;
      case 'P':
        pangea_directory = optarg;
        break;
      case 'G':
        pangea_camera_profile = optarg;
        break;
      case 'U':
        pangea_pause_file = optarg;
        break;
      case 'F': {
        char extra;
        if (sscanf(optarg, "%lf,%lf%c", &mock_map_horizontal_fov, &mock_map_vertical_fov, &extra) != 2
            || !isfinite(mock_map_horizontal_fov) || !isfinite(mock_map_vertical_fov)
            || mock_map_horizontal_fov <= 0.0 || mock_map_horizontal_fov >= 179.0
            || mock_map_vertical_fov <= 0.0 || mock_map_vertical_fov >= 179.0) {
          fprintf(stderr, "USHER:\t--mock-map-fov expects HORIZONTAL,VERTICAL degrees between 0 and 179\n");
          return 2;
        }
        mock_map_option_selected = true;
        break;
      }
      case 'k':
        clock_probes_enabled = true;
        clock_align_selected = true;
        clock_align_disabled = false;
        break;
      case 'K':
        clock_probes_enabled = false;
        clock_align_disabled = true;
        break;
      case 'j':
      case 'q':
        if (!set_exposure_latency(option == 'j' ? USHER_CAMERA_LWIRCAM : USHER_CAMERA_CHDK, optarg)) {
          fprintf(stderr, "USHER:\tlatency expects measured MIN,MAX milliseconds within 0..2000\n");
          return 2;
        }
        break;
      case 'r':
        lwir_cam_pipe_set_native_raw(1);
        lwir_raw_selected = true;
        break;
      case 'y':
        if (lwir_cam_pipe_set_calibration(optarg) != 0) {
          fprintf(stderr, "USHER:\t--lwir-calibration expects a camera YAML file\n");
          return 2;
        }
        lwir_calibration_selected = true;
        break;
      case 'Z':
        if (lwir_cam_pipe_set_config(optarg) != 0) return 2;
        lwir_config_selected = true;
        break;
      case 'm':
        motion_compensation_enabled = true;
        motion_compensation_selected = true;
        break;
      case 'V':
        puts(USHER_BUILD_VERSION);
        return 0;
      case 'S':
        speedtest_enabled = true;
        break;
      case 'h':
        print_usage(argv[0]);
        return 0;
      default:
        print_usage(argv[0]);
        return 2;
    }
  }

  if (optind != normalized_argc) {
    print_usage(argv[0]);
    return 2;
  }
  if (lwir_config_selected && requested_camera_backend != CAMERA_BACKEND_LWIR_CAM) {
    fprintf(stderr, "USHER:\t--lwir-config requires --lwircam\n"); return 2;
  }
  if (lwir_spotter_enabled() && (requested_camera_backend != CAMERA_BACKEND_LWIR_CAM
      || test_mode || local_mode || mock_map_enabled || mock_transform_enabled || speedtest_enabled)) {
    fprintf(stderr, "USHER:\tLWIR spotting requires real --lwircam capture without test/effects/speedtest\n");
    return 2;
  }
  if (lwir_spotter_geometry_selected && !lwir_spotter_enabled()) {
    fprintf(stderr, "USHER:\tLWIR spotting geometry requires --lwir-spotter-settings\n");
    return 2;
  }

  if (earcam_device_selected && earcam_sim_selected) {
    fprintf(stderr, "USHER:\t--earcam-device cannot be combined with --earcam-sim\n");
    return 2;
  }

  if (loomcam_enabled) {
    if (requested_camera_backend == CAMERA_BACKEND_UNSELECTED) requested_camera_backend = CAMERA_BACKEND_AI_CAM;
    if (requested_camera_backend != CAMERA_BACKEND_AI_CAM) {
      fprintf(stderr, "USHER:\t--loomcam owns the AI camera and cannot be combined with another camera backend\n");
      return 2;
    }
    requested_camera_backend = CAMERA_BACKEND_LOOM_CAM;
  }
  if (native_photo_selected) {
    if ((requested_camera_backend != CAMERA_BACKEND_AI_CAM && requested_camera_backend != CAMERA_BACKEND_SPOTTER_CAM)
        || test_mode || mock_map_enabled || mock_image_was_selected || mock_transform_enabled
        || spotter_settings_selected) {
      fprintf(stderr, "USHER:\t--native-photo requires real --aicam or --spottercam without model settings\n");
      return 2;
    }
    if (requested_camera_backend == CAMERA_BACKEND_AI_CAM) {
      if (ai_cam_pipe_set_native(true)) return 2;
    } else if (spotter_cam_pipe_set_photo_only(true)) return 2;
  }

  if (spotter_camera_backend_targets_enabled()
      && ((requested_camera_backend != CAMERA_BACKEND_UNSELECTED
           && requested_camera_backend != CAMERA_BACKEND_SPOTTER_CAM)
          || native_photo_selected || speedtest_enabled)) {
    fprintf(stderr, "USHER:\t--spottercam-targets requires --spottercam detection mode\n");
    return 2;
  }
  if (mock_map_option_selected && !mock_map_enabled) {
    fprintf(stderr, "USHER:\t--mock-map-cache and --mock-map-fov require --mock-map\n");
    return 2;
  }
  if ((pangea_camera_profile != NULL || pangea_pause_file != NULL) && pangea_directory == NULL) {
    fprintf(stderr, "USHER:\tPANGEA options require --pangea DIR\n");
    return 2;
  }
  if (pangea_directory != NULL) {
    const bool real_camera = pangea_camera_profile != NULL;
    if (pangea_directory[0] == '\0' || speedtest_enabled
        || (pangea_pause_file != NULL && pangea_pause_file[0] == '\0')
        || (!real_camera && (!local_mode || !mock_map_enabled))
        || (real_camera && (local_mode || mock_map_enabled || mock_transform_enabled || test_mode
                            || mock_image_was_selected || mock_lens_profile != NULL
                            || (requested_camera_backend != CAMERA_BACKEND_CHDK && requested_camera_backend != CAMERA_BACKEND_AI_CAM
                                && requested_camera_backend != CAMERA_BACKEND_SPOTTER_CAM && requested_camera_backend != CAMERA_BACKEND_LWIR_CAM)))) {
      fprintf(stderr,
              "USHER:\t--pangea needs local MIMIC or --pangea-camera with an explicit real --aicam, --spottercam, --chdk or --lwircam backend; no test/effects/speedtest in real mode\n");
      return 2;
    }
    if (real_camera) {
      struct stat profile_status;
      if (realpath(pangea_camera_profile, resolved_pangea_profile) == NULL
          || stat(resolved_pangea_profile, &profile_status) != 0 || !S_ISREG(profile_status.st_mode)
          || profile_status.st_size <= 0 || profile_status.st_size > CAMERA_PROFILE_MAX_BYTES || access(resolved_pangea_profile, R_OK) != 0) {
        fprintf(stderr, "USHER:\t--pangea-camera requires a readable PANGEA camera profile\n");
        return 2;
      }
      pangea_camera_profile = resolved_pangea_profile;
    }
  }
  if (mock_lens_profile != NULL) {
    struct stat profile_status;
    if (!mock_map_enabled || !local_mode || realpath(mock_lens_profile, resolved_mock_lens_profile) == NULL
        || stat(resolved_mock_lens_profile, &profile_status) != 0 || !S_ISREG(profile_status.st_mode)
        || profile_status.st_size <= 0 || profile_status.st_size > 16384
        || access(resolved_mock_lens_profile, R_OK) != 0) {
      fprintf(stderr, "USHER:\t--mock-lens needs --local --mock-map and a readable profile file\n");
      return 2;
    }
    if (access("./bin/camera_lens", X_OK) != 0) {
      fprintf(stderr, "USHER:\t--mock-lens needs the native renderer; run make camera-lens\n");
      return 2;
    }
    mock_lens_profile = resolved_mock_lens_profile;
  }
  if (mock_map_enabled) {
    if (!local_mode || mock_image_was_selected || mock_transform_enabled
        || (requested_camera_backend == CAMERA_BACKEND_SPOTTER_CAM && !spotter_settings_selected)) {
      fprintf(stderr, "USHER:\t--mock-map requires --local; incompatible with --mock-image, "
                      "--mocktransform and --spottercam\n");
      return 2;
    }
    mock_map_cache = usher_resolve_path(mock_map_cache, resolved_mock_map_cache,
                                        sizeof(resolved_mock_map_cache));
    char tile_cache[PATH_MAX];
    int length = snprintf(tile_cache, sizeof(tile_cache), "%s/GOOGLE/18", mock_map_cache);
    struct stat status;
    if (mock_map_cache[0] == '\0' || length < 0 || (size_t)length >= sizeof(tile_cache)
        || stat(tile_cache, &status) != 0 || !S_ISDIR(status.st_mode) || access(tile_cache, R_OK | X_OK) != 0) {
      fprintf(stderr, "USHER:\t--mock-map needs zoom-18 XYZ JPEGs in the pprzgcs GOOGLE/18 cache layout\n");
      return 2;
    }
    test_mode = true;
  }
  if (mock_image_was_selected && !test_mode
      && (!local_mode || requested_camera_backend != CAMERA_BACKEND_UNSELECTED)) {
    fprintf(stderr, "USHER:\t--mock-image requires --test or --local without a camera backend\n");
    return 2;
  }
  if (mock_transform_enabled && !test_mode) {
    fprintf(stderr, "USHER:\t--mocktransform requires --test\n");
    return 2;
  }
  if (capture_arbiter_enforce_selected && !capture_arbiter_session_id()) {
    fprintf(stderr, "USHER:\tsurvey controls require at least one --survey-camera profile\n");
    return 2;
  }
  if (capture_arbiter_session_id() && (speedtest_enabled || test_mode || mock_map_enabled || mock_transform_enabled)) {
    fprintf(stderr, "USHER:\tsurvey mode is for real survey captures, not speedtest or mock operation\n");
    return 2;
  }
  if (setenv("USHER_FRAME_TIMING", "1", 1) != 0) return 1;
  if (speedtest_enabled) {
    if (requested_camera_backend == CAMERA_BACKEND_UNSELECTED) {
      fprintf(stderr, "USHER:\t--speedtest requires --chdk, --aicam, --spottercam, or --lwircam\n");
      return 2;
    }
    if (serial_was_selected || local_mode || test_mode || mock_image_was_selected
        || mock_transform_enabled || debug_enabled || earcam_option_selected || pose_log_selected
        || clock_align_selected || lwir_raw_selected || lwir_calibration_selected
        || motion_compensation_selected || clock_align_disabled || loomcam_enabled
        || exposure_latency[USHER_CAMERA_LWIRCAM].valid || exposure_latency[USHER_CAMERA_CHDK].valid) {
      fprintf(stderr, "USHER:\t--speedtest cannot be combined with daemon, mock, EARcam, pose, or LWIR processing options\n");
      return 2;
    }
  }

  if (debug_enabled) {
    setvbuf(stdout, NULL, _IOLBF, 0);
    setvbuf(stderr, NULL, _IOLBF, 0);
  }

  /* The fixed loopback payload endpoint is also the physical-mode instance
   * claim. Acquire it before opening UART or camera resources so a foreground
   * invocation cannot compete with the systemd-managed USHER service. */
  if (socket_init(1) != 0) {
    return 1;
  }

  if (!local_mode && !serial_was_selected && access(USHER_SERIAL_DEVICE, R_OK | W_OK) != 0
      && access(USHER_LOCAL_SIM_DEVICE, F_OK) == 0 && access(USHER_LOCAL_APP_DEVICE, F_OK) == 0) {
    local_mode = true;
    serial_device = USHER_LOCAL_APP_DEVICE;
    printf("USHER:\tauto-detected existing local serial bridge\n");
  }

  if (local_mode) {
    if (serial_was_selected) {
      fprintf(stderr, "USHER:\t--serial cannot be combined with --local\n");
      return 2;
    }
    if (local_bridge_requested) {
      if (lock_local_instance() != 0) {
        return 1;
      }
      socat_pid = start_local_serial_bridge();
      if (socat_pid < 0) {
        return 1;
      }
      local_bridge_owned = true;
    }
    serial_device = USHER_LOCAL_APP_DEVICE;
  }

  enum camera_backend_type selected_camera_backend = requested_camera_backend;
  if (selected_camera_backend == CAMERA_BACKEND_UNSELECTED) {
    selected_camera_backend = local_mode ? CAMERA_BACKEND_LOCAL : CAMERA_BACKEND_SPOTTER_CAM;
  }
  if (loomcam_enabled) loom_camera_backend_configure(loomcam_references, loomcam_model);
  configured_camera_backend = selected_camera_backend;
  if (camera_backend_select(selected_camera_backend, test_mode) != 0) {
    stop_local_serial_bridge();
    return 2;
  }
  test_capture_enabled = test_mode;

  optical_camera_id = camera.camera_id;

  if (speedtest_enabled) {
    if (!(camera.capabilities & CAMERA_CAP_IMAGE) || !camera.prefix || !camera.photo_directory) return 2;
    const char *photo_root = camera.photo_directory;
    const char filename_prefix = camera.prefix;
    char speedtest_directory[PATH_MAX];
    if (camera_speedtest_create_output_directory(photo_root, speedtest_directory,
        sizeof(speedtest_directory)) != 0) {
      fprintf(stderr, "USHER SPEEDTEST:\tunable to create isolated output directory: %s\n",
              strerror(errno));
      return 1;
    }
    chdk_camera_backend_set_photo_directory(speedtest_directory);
    ai_cam_pipe_set_photo_directory(speedtest_directory);
    spotter_cam_pipe_set_photo_directory(speedtest_directory);
    lwir_cam_pipe_set_photo_directory(speedtest_directory);
    struct camera_speedtest_config speedtest = {
      .backend_name = camera.name,
      .photo_directory = speedtest_directory,
      .filename_prefix = filename_prefix,
      .private_output = (camera.capabilities & CAMERA_CAP_PRIVATE_IMAGE) != 0,
      .shoot = camera.shoot,
      .keep_running = &keep_running
    };
    int first_image;
    if (camera_speedtest_find_image_block(&speedtest, &first_image) != 0) {
      fprintf(stderr, "USHER SPEEDTEST:\tunable to reserve an unused image-number block: %s\n",
              strerror(errno));
      return 1;
    }
    signal(SIGPIPE, SIG_IGN);
    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);
    printf("USHER:\tStarting Camera Speed Test\n");
    printf("USHER:\tcamera backend: %s\n", camera.name);
    uint64_t initialization_started_us = monotonic_time_us();
    int prepare_result = camera_prepare(optical_camera_id);
    uint64_t initialization_completed_us = monotonic_time_us();
    if (prepare_result != 0 || initialization_started_us == 0
        || initialization_completed_us < initialization_started_us) {
      fprintf(stderr, "USHER SPEEDTEST:\tcamera initialization failed\n");
      cameras_deinit();
      return 1;
    }
    printf("USHER SPEEDTEST:\tinitialization: %.3f ms (excluded from capture rate)\n",
           (initialization_completed_us - initialization_started_us) / 1000.0);
    int speedtest_result = camera_speedtest_run(&speedtest, first_image);
    cameras_deinit();
    printf("USHER:\tSpeed test shutdown\n");
    return speedtest_result == 0 ? 0 : 1;
  }

  // Initialization
  printf("USHER:\tStarting camera service\n");
  printf("USHER:\tserial device: %s\n", serial_device);
  printf("USHER:\tcamera backend: %s\n", camera.name);
  if (debug_enabled) {
    printf("USHER DEBUG:\tenabled\n");
  }
  if (test_mode) {
    if (mock_map_enabled) {
      printf("USHER:\tMIMIC: XYZ zoom 18 from %s; FOV %.2fx%.2f degrees (simulation model)\n",
             mock_map_cache, mock_map_horizontal_fov, mock_map_vertical_fov);
    } else if (mock_image_was_selected) {
      printf("USHER:\tcamera test mode: mock image %s\n", mock_image);
    } else {
      printf("USHER:\tcamera test mode: random image from testphotos when available\n");
    }
    printf("USHER:\tmock attitude transform: %s\n", mock_transform_enabled ? "enabled" : "disabled");
  }
  if (local_mode) {
    printf("USHER:\tlocal simulator device: %s\n", USHER_LOCAL_SIM_DEVICE);
  }
  camera_source_image = NULL;
  if (selected_camera_backend == CAMERA_BACKEND_LOCAL && !test_mode) {
    printf("USHER:\tmock camera image: %s\n", mock_image);
    camera_source_image = mock_image;
  } else if (test_mode) {
    camera_source_image = mock_image_was_selected ? mock_image : NULL;
  }
  signal(SIGPIPE, SIG_IGN);
  if ((test_mode || selected_camera_backend == CAMERA_BACKEND_LOCAL)
      && camera.init(camera_source_image) != 0) {
    camera.deinit();
    stop_local_serial_bridge();
    return 1;
  }
  local_capture_only = selected_camera_backend == CAMERA_BACKEND_LOCAL && !test_mode;
  if (clock_align_disabled) clock_probes_enabled = false;
  else if (!test_mode && !local_capture_only) clock_probes_enabled = true;
  camera_initialized[optical_camera_id] = test_mode || local_capture_only;
  if (camera_initialized[optical_camera_id]) camera_owners[optical_camera_id] = camera;
  if (earcam_requested) {
    prepare_session_backend(USHER_CAMERA_EARCAM);
  }
  int ret = serial_init(serial_device);
  if (ret < 0) {
    drain_camera_backends();
    cameras_deinit();
    stop_local_serial_bridge();
    return -1;
  }
  if (serial_tx_start(fd) != 0) {
    fprintf(stderr, "USHER:\tunable to initialize nonblocking UART output: %s\n", strerror(errno));
    close(fd);
    drain_camera_backends();
    cameras_deinit();
    stop_local_serial_bridge();
    return 1;
  }
  if (event_wake_init() != 0) {
    fprintf(stderr, "USHER:\tunable to initialize event wakeup: %s\n", strerror(errno));
    serial_tx_stop();
    close(fd);
    drain_camera_backends();
    cameras_deinit();
    stop_local_serial_bridge();
    return 1;
  }
  fc_clock = (struct clock_alignment) {0};
  next_clock_probe_us = clock_probe_count = clock_reply_count = clock_rejected_count = 0;
  if (pose_log_start(pose_log_dir) != 0) {
    fprintf(stderr, "USHER POSE:\tlog unavailable: %s; capture continues\n", strerror(errno));
  }
  signal(SIGINT, handle_signal);
  signal(SIGTERM, handle_signal);

  if (pangea_directory != NULL) start_pangea();
  camera_register_stream(&camera);

  // Initial settings
  is_shooting = 0;
  usher_protocol.status = 0;
  image_idx = 0;
  image_count = 0;
  shooting_count = 0;
  shooting_thread_count = 0;

  puts(USHER_BUILD_VERSION);
  printf("Started OK\n");
  notify_systemd_ready();
  if (!test_mode && !local_mode && selected_camera_backend == CAMERA_BACKEND_LWIR_CAM) {
    lwir_ready_or_start_recovery();
  }
  if (debug_enabled) {
    next_debug_heartbeat = time(NULL) + DEBUG_HEARTBEAT_SECONDS;
    printf("USHER DEBUG:\twaiting for USHER camera messages on %s\n", serial_device);
  }

  struct pollfd event_sources[3 + 2 * CAMERA_BACKEND_MAX_ID] = {
    {.fd = fd, .events = POLLIN, .revents = 0},
    {.fd = socket_get_fd(), .events = POLLIN, .revents = 0},
    [2 + 2 * CAMERA_BACKEND_MAX_ID] = {.fd = event_wake_reader, .events = POLLIN}
  };

  // MAIN loop
  while (usher_running()) {
    if (pangea_pid > 0) {
      int pangea_status = 0;
      if (waitpid(pangea_pid, &pangea_status, WNOHANG) == pangea_pid) {
        fprintf(stderr, "USHER:\tPANGEA exited (status %d); capture continues, saved photos can be replayed\n", pangea_status);
        pangea_pid = -1;
      }
    }
    send_clock_probe(monotonic_time_us());
    service_capture_arbiter(monotonic_time_us());
    flush_shot_results();
    flush_control_results();
    service_camera_backends();
    flush_cam_points();
    flush_cam_point_updates();
    event_sources[0].events = POLLIN | (serial_tx_pending() ? POLLOUT : 0);
    for (unsigned camera_id = 1; camera_id <= CAMERA_BACKEND_MAX_ID; ++camera_id) {
      const struct camera_session_ops *ops = camera_owners[camera_id].session;
      const struct camera_point_stream_ops *stream = camera_owners[camera_id].point_stream;
      event_sources[2 * camera_id] = (struct pollfd) {
        .fd = ops && ops->completion_fd ? ops->completion_fd() : -1, .events = POLLIN
      };
      event_sources[2 * camera_id + 1] = (struct pollfd) {
        .fd = stream && stream->completion_fd ? stream->completion_fd() : -1, .events = POLLIN
      };
    }
    int poll_result;
    do {
      poll_result = poll(event_sources, sizeof(event_sources) / sizeof(event_sources[0]), event_loop_timeout_ms());
    } while (poll_result < 0 && errno == EINTR && usher_running());
    if (poll_result < 0) {
      fprintf(stderr, "USHER:\tevent poll failed: %s\n", strerror(errno));
      break;
    }
    if (event_sources[2 + 2 * CAMERA_BACKEND_MAX_ID].revents & POLLIN) drain_event_wake();
    if ((event_sources[0].revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
      fprintf(stderr, "USHER:\tserial device disconnected\n");
      break;
    }
    if ((event_sources[0].revents & POLLOUT) != 0 && serial_tx_flush() != 0) {
      fprintf(stderr, "USHER:\tUART output failed: %s\n", strerror(errno));
      break;
    }

    // Drain available serial data in one syscall and process frames in order.
    ssize_t bytes_read = 0;
    if ((event_sources[0].revents & POLLIN) != 0) {
      bytes_read = read(fd, serial_buffer, sizeof(serial_buffer));
      serial_receive_monotonic_us = monotonic_time_us();
    }
    if (bytes_read > 0) {
      for (ssize_t index = 0; index < bytes_read; index++) {
        serial_byte_count++;
        uint8_t parser_errors_before = usher_protocol.error;
        if (debug_enabled && serial_buffer[index] == STX) {
          printf("USHER DEBUG:\treceived USHER frame start at serial byte %llu\n",
                 (unsigned long long)serial_byte_count);
        }
        parse_usher(&usher_protocol, serial_buffer[index]);
        if (usher_protocol.error != parser_errors_before) {
          rejected_frame_count++;
          if (debug_enabled) {
            printf("USHER DEBUG:\trejected USHER frame at serial byte %llu (total rejected: %llu)\n",
                   (unsigned long long)serial_byte_count,
                   (unsigned long long)rejected_frame_count);
          }
        }
        if (usher_protocol.msg_received) {
          valid_frame_count++;
          if (debug_enabled) {
            printf("USHER DEBUG:\taccepted USHER message id %u with %u payload bytes "
                   "(frame %llu)\n",
                   usher_protocol.msg_id, usher_protocol.payload_len,
                   (unsigned long long)valid_frame_count);
          }
          handle_received_message();
        }
      }
    } else if (bytes_read < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
      fprintf(stderr, "USHER:\tserial read failed: %s\n", strerror(errno));
    }

    if (debug_enabled && time(NULL) >= next_debug_heartbeat) {
      pthread_mutex_lock(&shot_result_mutex);
      printf("USHER SHOT RESULTS:\tpending=%zu dropped=%" PRIu64 "\n", shot_result_count, shot_result_dropped);
      pthread_mutex_unlock(&shot_result_mutex);
      printf("USHER DEBUG:\twaiting for USHER data: %llu bytes, %llu valid frames, "
             "%llu rejected frames\n",
             (unsigned long long)serial_byte_count,
             (unsigned long long)valid_frame_count,
             (unsigned long long)rejected_frame_count);
      struct pose_log_stats pose_status = pose_log_status();
      printf("USHER POSE:\taccepted=%" PRIu64 " synced=%" PRIu64 " dropped=%" PRIu64
             " rejected=%" PRIu64 " error=%d\n", pose_status.accepted, pose_status.synced,
             pose_status.dropped, pose_status.rejected, pose_status.error);
      next_debug_heartbeat = time(NULL) + DEBUG_HEARTBEAT_SECONDS;
      if (clock_probes_enabled) {
        printf("USHER CLOCK:\tprobes=%" PRIu64 " accepted=%" PRIu64 " rejected=%" PRIu64 "\n",
               clock_probe_count, clock_reply_count, clock_rejected_count);
      }
      pthread_mutex_lock(&mut);
      printf("USHER SODA:\tpending=%zu active=%d deferred=%" PRIu64 " completed=%" PRIu64
             " failed=%" PRIu64 " unavailable/full=%" PRIu64 "\n",
             soda_backlog_count, soda_backlog_active, soda_busy_deferred_count,
             soda_backlog_completed, soda_backlog_failed, soda_backlog_overflow);
      pthread_mutex_unlock(&mut);
    }

    // Read the socket
    if ((event_sources[1].revents & POLLIN) != 0
        && socket_recv(image_buffer[image_idx], IMAGE_SIZE) == IMAGE_SIZE) {
      image_idx = (image_idx + 1) % MAX_IMAGE_BUFFERS;

      if (image_count < MAX_IMAGE_BUFFERS) {
        image_count++;
      }
    }

  }

  // Close
  capture_set_running(&keep_running, false);
  capture_arbiter_stop();
  drain_camera_backends();
  stop_soda_backlog();
  stop_pangea();
  socket_deinit();
  serial_tx_stop();
  close(fd);
  pthread_mutex_lock(&mut);
  pthread_cond_broadcast(&camera_available);
  while (shooting_thread_count > 0 || chdk_capture_in_progress || chdk_finish_in_progress
         || lwir_capture_in_progress || lwir_handoff_in_progress || lwir_recovery_in_progress) {
    pthread_cond_wait(&workers_finished, &mut);
  }
  pthread_mutex_unlock(&mut);
  pthread_mutex_lock(&shot_result_mutex);
  fprintf(stderr, "USHER SHOT RESULTS:\tundelivered=%zu dropped=%" PRIu64 "\n",
          shot_result_count, shot_result_dropped);
  pthread_mutex_unlock(&shot_result_mutex);
  cameras_deinit();
  stop_local_serial_bridge();

  if (pose_log_stop() != 0) fprintf(stderr, "USHER POSE:\tlog incomplete: %s\n", strerror(errno));
  struct pose_log_stats pose_status = pose_log_status();
  fprintf(stderr, "USHER POSE:\tfinal accepted=%" PRIu64 " synced=%" PRIu64
          " dropped=%" PRIu64 " rejected=%" PRIu64 " error=%d\n",
          pose_status.accepted, pose_status.synced, pose_status.dropped, pose_status.rejected, pose_status.error);
  event_wake_close();
  printf("USHER:\tShutdown\n");
  return 0;
}

/** @brief Send READY=1 to systemd when launched with a notify socket.
 * @details Failure is diagnostic only: USHER also supports direct/manual execution. */
static void notify_systemd_ready(void)
{
  const char *socket_path = getenv("NOTIFY_SOCKET");
  if (socket_path == NULL || socket_path[0] == '\0') {
    return;
  }

  struct sockaddr_un address = {.sun_family = AF_UNIX};
  size_t path_length = strlen(socket_path);
  if (path_length >= sizeof(address.sun_path)) {
    fprintf(stderr, "USHER:\tsystemd notification socket path is too long\n");
    return;
  }

  for (size_t index = 0; index <= path_length; index++) {
    address.sun_path[index] = socket_path[index];
  }
  bool abstract_socket = address.sun_path[0] == '@';
  if (abstract_socket) {
    address.sun_path[0] = '\0';
  }

  int notify_fd = socket(AF_UNIX, SOCK_DGRAM, 0);
  if (notify_fd < 0) {
    fprintf(stderr, "USHER:\tfailed to create systemd notification socket: %s\n",
            strerror(errno));
    return;
  }
  if (fcntl(notify_fd, F_SETFD, FD_CLOEXEC) != 0) {
    close(notify_fd);
    return;
  }

  static const char ready_message[] = "READY=1\nSTATUS=MORA serial ready; camera selected on shoot command";
  socklen_t address_length = (socklen_t)(offsetof(struct sockaddr_un, sun_path)
                                         + path_length + (abstract_socket ? 0 : 1));
  if (sendto(notify_fd, ready_message, sizeof(ready_message) - 1, MSG_NOSIGNAL,
             (const struct sockaddr *)&address, address_length) < 0) {
    fprintf(stderr, "USHER:\tfailed to notify systemd that startup completed: %s\n",
            strerror(errno));
  }
  close(notify_fd);
}

/** @brief Consume one optical ticket, then finish images outside the lane where safe.
 * @param ptr Pool-owned capture_job, released on every worker exit.
 * @return NULL, as required by pthread entry points.
 * @details The worker slot and camera lane have different lifetimes: publication
 * may still own a bounded worker slot after the next acquisition has started.
 */
static void *handle_msg_shoot(void *ptr)
{
  struct capture_job *job = ptr;

  pthread_mutex_lock(&mut);
  while (job->ticket != next_capture_ticket && usher_running()) {
    pthread_cond_wait(&camera_available, &mut);
  }
  if (!usher_running()) {
    if (waiting_capture == job) waiting_capture = NULL;
    pthread_mutex_unlock(&mut);
    capture_arbiter_failed(&job->shot, job->camera_mask);
    queue_image_results(job->request_id, job->shot.data.nr, job->camera_mask, USHER_SHOT_FAILED);
    release_worker_slot();
    capture_job_release(job);
    return NULL;
  }

  is_shooting = 1;
  if (!job->shot_counted) shooting_count++;
  if (waiting_capture == job) waiting_capture = NULL;
  if (debug_enabled) {
    uint64_t now_us = monotonic_time_us();
    printf("USHER-%d DEBUG:\tqueue age %.3f s, superseded live triggers %" PRIu64 "\n",
           job->shot.data.nr, now_us >= job->received_us ? (now_us - job->received_us) / 1e6 : 0,
           superseded_capture_count);
  }
  pthread_mutex_unlock(&mut);

  /* Acquisition holds the camera lane; ordinary optical publication and EXIF can
   * finish after it is released. Live SODA is always submitted elsewhere. LWIR is finished
   * inside the slot because its geolocation reads the backend's last-capture state;
  * --spottercam has the same hazard (spotter_cam_pipe's last-detection state is
   * likewise a single static slot, not part of `image`), so it is finished inside too. */
  const bool separate_lwir = lwir_async_recovery_enabled
                             && !test_capture_enabled && !local_capture_only;
  struct captured_image images[USHER_CAMERA_LWIRCAM];
  int image_count_ready = 0;
  const int ordered_camera_max = separate_lwir ? USHER_CAMERA_AICAM : USHER_CAMERA_LWIRCAM;
  for (int camera_id = USHER_CAMERA_CHDK; camera_id <= ordered_camera_max && usher_running(); ++camera_id) {
    if ((job->camera_mask & (1U << (camera_id - 1))) != 0) {
      if (camera_prepare(camera_id) != 0) {
        capture_arbiter_complete(&job->shot, camera_id, false, false, 0, monotonic_time_us());
        queue_shot_result(job->request_id, job->shot.data.nr, camera_id, USHER_SHOT_FAILED);
        continue;
      }
      struct captured_image *image = &images[image_count_ready];
      if (!capture_image(&job->shot, image, camera_id, &camera)) {
        queue_shot_result(job->request_id, job->shot.data.nr, camera_id, USHER_SHOT_FAILED);
        continue;
      }
      bool finish_in_slot = (image->backend.capabilities & CAMERA_CAP_FINISH_OWNED) != 0;
      if (finish_in_slot)
        queue_shot_result(job->request_id, job->shot.data.nr, camera_id,
                          finish_image(&job->shot, image) ? USHER_SHOT_SAVED : USHER_SHOT_FAILED);
      else ++image_count_ready;
    }
  }

  pthread_mutex_lock(&mut);
  is_shooting = 0;
  ++next_capture_ticket;
  pthread_cond_broadcast(&camera_available);
  pthread_mutex_unlock(&mut);
  for (int index = 0; index < image_count_ready; ++index) {
    queue_shot_result(job->request_id, job->shot.data.nr, images[index].camera_id,
                      finish_image(&job->shot, &images[index]) ? USHER_SHOT_SAVED : USHER_SHOT_FAILED);
  }
  release_worker_slot();
  capture_job_release(job);
  return NULL;
}

/** @brief Acquire pixels or a completed file for a single camera.
 * @param shoot Original pose-tagged request; not automatically exposure-time pose.
 * @param[out] image Per-image state passed to finish_image() on success.
 * @param camera_id Sensor identity used for metadata and independent survey accounting.
 * @param backend Driver selected under the caller's camera ownership.
 * @return True when finish_image() is owed, including pending split-phase captures.
 * @note A negative acquire result must not fall through to a second exposure.
 * Only the driver's explicit positive fallback result allows synchronous shooting.
 */
static bool capture_image(const union dc_shot_union *shoot, struct captured_image *image,
                          int camera_id, const struct camera_backend *backend)
{
  if (!shoot || !image || !backend || !(backend->capabilities & CAMERA_CAP_IMAGE)
      || !backend->shoot || !backend->deinit
      || ((backend->capabilities & CAMERA_CAP_DETECTION)
          && (!backend->bind_frame || !backend->detection_summary || !backend->save_detection_copy))
      || ((backend->capabilities & CAMERA_CAP_GEOLOCATION) && !backend->geolocate)
      || camera_id < USHER_CAMERA_CHDK || camera_id > USHER_CAMERA_LWIRCAM) return false;
  char *filename = image->filename;
  image->backend = *backend;
  if (test_capture_enabled || local_capture_only || mock_map_enabled)
    image->backend.capabilities &= ~CAMERA_CAP_PRIVATE_IMAGE;
  const struct camera_backend identity = camera_identity_get(camera_id);
  image->backend.camera_id = camera_id;
  image->backend.prefix = identity.prefix;
  image->backend.mock = identity.mock;
  image->backend.soda_option = identity.soda_option;
  filename[0] = '\0';
  image->camera_id = camera_id;
  image->exposure_pose_valid = false;
  image->pose_uncertainty_m = 0;
  image->pending = (struct camera_pending) {0};
  image->capture_delay = -1;
  image->capture_times = (struct capture_timing) {0};
  if (!test_capture_enabled && !local_capture_only && !mock_map_enabled
      && image->backend.capture_ops && image->backend.capture_ops->acquire && image->backend.capture_ops->finish) {
    const int acquired = image->backend.capture_ops->acquire(&image->pending, shoot->data.nr);
    if (acquired == 0) {
      capture_arbiter_acquired(shoot->data.nr, camera_id, monotonic_time_us());
      return true;
    }
    if (acquired < 0) {
      capture_arbiter_failed(shoot, 1U << (camera_id - 1));
      return false;
    }
  }
  printf("USHER-%d:\tShooting: start\n", shoot->data.nr);
  if (debug_enabled) {
    if (test_capture_enabled) {
      printf("USHER-%d DEBUG:\trequesting test image for selected %s backend\n",
             shoot->data.nr, backend->name);
    } else {
      printf("USHER-%d DEBUG:\trequesting image from %s backend\n", shoot->data.nr, backend->name);
    }
  }
  uint64_t capture_started_us = monotonic_time_us();
  int capture_result = mock_map_enabled
                       ? local_pipe_map_shoot(filename, sizeof(image->filename), shoot, &image->backend,
                         mock_map_cache, mock_map_horizontal_fov, mock_map_vertical_fov)
                       : backend->shoot(filename, sizeof(image->filename), shoot->data.nr);
  const uint64_t capture_finished_us = monotonic_time_us();
  if (debug_enabled) {
    printf("USHER-%d TIMING:\tcamera=%d capture_ms=%.3f result=%d\n",
           shoot->data.nr, camera_id, (monotonic_time_us() - capture_started_us) / 1000.0,
           capture_result);
  }
  if (capture_result != 0) {
    capture_arbiter_complete(shoot, camera_id, false, false, 0, monotonic_time_us());
    fprintf(stderr, "USHER-%d:\t%s camera capture failed\n", shoot->data.nr, backend->name);
    if (!mock_map_enabled) {
      if (!(camera_id == USHER_CAMERA_CHDK && chdk_async_capture_enabled
        && !test_capture_enabled && !local_capture_only)) camera_close_owner(camera_id);
      if (camera_id == USHER_CAMERA_LWIRCAM) lwir_mark_backoff();
    }
    filename[0] = '\0';
  }
  printf("USHER-%d:\tShooting: got image %s\n", shoot->data.nr, filename);
  if (filename[0] != '\0' && mock_transform_enabled) {
    int transform_result = image_mock_transform(filename, shoot);
    if (transform_result < 0) {
      fprintf(stderr, "USHER-%d:\tfailed to apply mock attitude transform to %s\n",
              shoot->data.nr, filename);
      filename[0] = '\0';
    } else if (transform_result == IMAGE_MOCK_TRANSFORM_SKIPPED) {
      printf("USHER-%d:\tShooting: mock attitude transform skipped; source image retained\n",
             shoot->data.nr);
    } else {
      printf("USHER-%d:\tShooting: mock attitude transform applied\n", shoot->data.nr);
    }
  }
  if (filename[0] != '\0' && test_capture_enabled && !mock_map_enabled
      && camera_id == USHER_CAMERA_LWIRCAM
      && lwir_cam_pipe_process_mock(filename) != 0) {
    fprintf(stderr, "USHER-%d:\tLWIR mock processing failed\n", shoot->data.nr);
    filename[0] = '\0';
  }
  if (filename[0] && (test_capture_enabled || local_capture_only || mock_map_enabled)) {
    if (image->backend.name && !strcmp(image->backend.name, "spottercam") &&
        spotter_cam_pipe_analyze(filename, shoot->data.nr))
      fprintf(stderr, "USHER:\tONNX object analysis failed; retaining photograph\n");
  }
  const bool live_lwir = image->backend.capture_timing && !(image->backend.capabilities & CAMERA_CAP_MOCK);
  image->capture_delay = live_lwir && image->backend.capture_delay ? image->backend.capture_delay() : -1;
  image->capture_times = live_lwir ? image->backend.capture_timing() : (struct capture_timing) {0};
  if (live_lwir && capture_timing_valid(&image->capture_times)) {
    struct capture_observation *observation = &image->capture_times.observation;
    observation->earliest_us = observation->latest_us = image->capture_times.arrival_monotonic_us;
    observation->sequence = image->capture_times.callback_sequence;
    observation->kind = CAPTURE_TIME_FRAME_ARRIVAL;
    if (exposure_latency[camera_id].valid)
      capture_observation_latency(observation, image->capture_times.arrival_monotonic_us,
                                  exposure_latency[camera_id].minimum_us, exposure_latency[camera_id].maximum_us);
  } else if (camera_id == USHER_CAMERA_CHDK && !test_capture_enabled && !local_capture_only) {
    struct capture_observation *observation = &image->capture_times.observation;
    observation->earliest_us = capture_started_us;
    observation->latest_us = capture_finished_us;
    observation->kind = CAPTURE_TIME_SHUTTER_WINDOW;
    if (exposure_latency[camera_id].valid && capture_finished_us >= capture_started_us
        && exposure_latency[camera_id].maximum_us <= capture_finished_us - capture_started_us) {
      observation->earliest_us += exposure_latency[camera_id].minimum_us;
      observation->latest_us = capture_started_us + exposure_latency[camera_id].maximum_us;
      observation->kind = CAPTURE_TIME_CALIBRATED_EXPOSURE;
    }
  }
  return filename[0] != '\0';
}

/** @brief Resolve live pose evidence associated with an acquired image.
 * @param request Original trigger whose photo number is retained.
 * @param[in,out] image Image and timing evidence to qualify.
 * @details Prefer frame-bound timing when available. Wait briefly for a pose
 * bracket, but promote the aligned pose to exposure pose only when the time
 * evidence represents exposure, not merely frame delivery. */
static void resolve_capture_pose(const union dc_shot_union *request, struct captured_image *image)
{
  if (test_capture_enabled || local_capture_only) return;
  struct capture_observation frame = {0};
  if (capture_arbiter_frame_timing(image->filename, image->camera_id, &frame)) image->capture_times.observation = frame;
  struct capture_observation *observation = &image->capture_times.observation;
  image->exposure_pose_valid = false;
  if (!capture_arbiter_observation_pose(observation, 200000)) return;
  observation->pose.data.nr = request->data.nr;
  if (capture_observation_exposure(observation)) {
    image->exposure_shot = observation->pose;
    image->pose_uncertainty_m = observation->pose_uncertainty_m;
    image->exposure_pose_valid = true;
  }
}

/** @brief Complete publication and metadata before releasing survey ownership.
 * @param shoot Original request, retained as provenance if exposure pose is available.
 * @param[in,out] image Acquired image and its original driver's completion hooks.
 * @details Native completion can wait for a JPEG here without holding the optical
 * lane. Metadata-dependent drivers still call this under their capture ownership.
 * Live analysis is submitted only after saving; local/test callers retain the
 * synchronous behavior expected by their tools and regression fixtures.
 */
static bool finish_image(const union dc_shot_union *shoot, struct captured_image *image)
{
  if (image->pending.acquired) {
    if (!image->backend.capture_ops || !image->backend.capture_ops->finish
        || image->backend.capture_ops->finish(&image->pending, image->filename, sizeof(image->filename), &keep_running) != 0) {
      capture_arbiter_failed(shoot, 1U << (image->camera_id - 1));
      return false;
    }
  }
  resolve_capture_pose(shoot, image);
  const union dc_shot_union *request = image->exposure_pose_valid ? shoot : NULL;
  /* Keep both records. Replacing request pose with an exposure estimate must not
   * erase the original evidence or apply the old latency correction a second time. */
  if (image->exposure_pose_valid) shoot = &image->exposure_shot;
  char detection_summary[256];
  const bool object_metadata = (image->backend.capabilities & CAMERA_CAP_DETECTION) != 0;
  if (object_metadata)
    image->backend.bind_frame(image->capture_times.observation.sequence,
                              image->capture_times.observation.frame_bound
                              ? image->capture_times.observation.sensor_timestamp_ns : 0);
  if (object_metadata) image->backend.detection_summary(detection_summary, sizeof(detection_summary));
  uint64_t metadata_started_us = monotonic_time_us();
  char *filename = image->filename;
  bool image_ready = false;
  {
    if (debug_enabled) {
      printf("USHER-%d DEBUG:\twriting flight metadata to captured JPEG\n", shoot->data.nr);
    }
    const int compensate = motion_compensation_enabled && !image->exposure_pose_valid;
    if (image_exif_write_complete(filename, shoot, image->capture_delay, compensate,
                                  &image->capture_times, request, image->pose_uncertainty_m,
                                  object_metadata ? detection_summary : NULL) == 0) {
      image_ready = true;
      printf("USHER-%d:\tShooting: EXIF metadata added\n", shoot->data.nr);
      if (image->backend.capabilities & CAMERA_CAP_GEOLOCATION) {
        if (mock_map_enabled) {
          if (image_exif_write_hotspots(filename,
                                        "status=unsupported_mock_map; visible_map_texture=true; coordinates_omitted=true") != 0) {
            fprintf(stderr, "USHER-%d:\tfailed to record map mock provenance\n", shoot->data.nr);
          }
        } else if (mock_transform_enabled) {
          if (image_exif_write_hotspots(filename, "status=unsupported_mock_transform; coordinates_omitted=true") != 0) {
            fprintf(stderr, "USHER-%d:\tfailed to record unsupported thermal transform\n", shoot->data.nr);
          }
        } else if (image->backend.geolocate(filename) != 0) {
          fprintf(stderr, "USHER-%d:\tLWIR hotspot geolocation failed; photo retained\n", shoot->data.nr);
          lwir_mark_backoff();
          if (image_exif_write_hotspots(filename, "status=analysis_failed; coordinates_omitted=true") != 0) {
            fprintf(stderr, "USHER-%d:\tfailed to record hotspot analysis failure\n", shoot->data.nr);
          }
        }
      } else if (object_metadata) {
        /* The detector's result was captured above and included in the
         * same EXIF rewrite. Do not add a second detection-only rewrite here. */
        if (debug_enabled) {
          printf("USHER-%d DEBUG:\tobject detection: %s\n", shoot->data.nr, detection_summary);
        }
      }
      if (mock_map_enabled && image_exif_write_mock_map(filename, mock_map_horizontal_fov,
          mock_map_vertical_fov) != 0) {
        fprintf(stderr, "USHER-%d:\tfailed to record mock map geometry\n", shoot->data.nr);
        image_ready = false;
      }
      if (image_ready && mock_map_enabled && mock_lens_profile &&
          image->camera_id == USHER_CAMERA_AICAM) {
        char derivative[MAX_FILENAME];
        if (!image_lens_emulate("./bin/camera_lens", filename, mock_lens_profile, mock_map_cache,
                                derivative, sizeof(derivative), &keep_running)) {
          fprintf(stderr, "USHER-%d:\tlens emulation failed; clean photo retained, LOOM/SODA skipped\n",
                  shoot->data.nr);
          image_ready = false;
        } else {
          const size_t length = strlen(derivative);
          for (size_t index = 0; index <= length; ++index) filename[index] = derivative[index];
          printf("USHER-%d:\tlens-emulated analysis input %s\n", shoot->data.nr, filename);
        }
      }
      if (image_ready && (image->backend.capabilities & CAMERA_CAP_PRIVATE_IMAGE)) {
        const int publication = usher_complete_photo(filename, sizeof(image->filename));
        if (publication < 0) {
          fprintf(stderr, "USHER-%d:\tphoto publication failed; private capture retained at %s: %s\n",
                  shoot->data.nr, filename, strerror(errno));
          image_ready = false;
        } else if (publication > 0) {
            fprintf(stderr, "USHER-%d:\tphoto published; directory sync or private alias cleanup incomplete: %s\n",
              shoot->data.nr, strerror(errno));
        }
      }
      if (image_ready) {
        printf("Photo take %d\n", shoot->data.nr);
      }
    } else {
      fprintf(stderr, "USHER-%d:\tfailed to add EXIF metadata to %s\n", shoot->data.nr, filename);
    }
  }

  if (debug_enabled) {
    printf("USHER-%d TIMING:\tcamera=%d metadata_ms=%.3f ready=%d\n",
           shoot->data.nr, image->camera_id,
           (monotonic_time_us() - metadata_started_us) / 1000.0, image_ready);
  }
  capture_arbiter_complete(shoot, image->camera_id, image_ready, image->exposure_pose_valid,
                           image->pose_uncertainty_m, monotonic_time_us());
  if (image_ready) {
    if (object_metadata) image->backend.save_detection_copy(filename);
    if (image->backend.image_saved) image->backend.image_saved(filename, shoot);
    bool live_capture = lwir_async_recovery_enabled && !test_capture_enabled && !local_capture_only;
    if (live_capture || (lwir_spotter_enabled() && image->camera_id == USHER_CAMERA_LWIRCAM)) {
      defer_saved_analysis(filename, shoot, image->camera_id, &image->capture_times);
    } else {
      uint64_t analysis_started_us = monotonic_time_us();
      int soda_result = run_soda(filename, shoot, image->camera_id);
      printf("USHER-%d:\tShooting: soda return %d of image %s\n",
             shoot->data.nr, soda_result, filename);
      if (debug_enabled) {
        printf("USHER-%d TIMING:\tcamera=%d soda_ms=%.3f result=%d\n",
               shoot->data.nr, image->camera_id,
               (monotonic_time_us() - analysis_started_us) / 1000.0, soda_result);
      }
    }
  }
  return image_ready;
}

/** @brief Dispatch one complete UART frame without running slow camera work inline.
 * @details The serial event-loop owner calls this once per parsed message. Camera
 * masks are split into independent backend work; EARcam receipt-time tagging is
 * separate from photographic queueing. New protocols must validate lengths and
 * session ownership before causing camera or service side effects.
 */
static void handle_received_message(void)
{
  usher_protocol.msg_received = false;
  if (usher_protocol.msg_id == USHER_CONTROL_NEGOTIATE || usher_protocol.msg_id == USHER_CONTROL_COMMAND) {
    handle_control_message();
    return;
  }

  if (usher_protocol.msg_id == USHER_CAPTURE_ARBITER_STATE) {
    union usher_capture_arbiter_state_union message;
    if (usher_protocol.payload_len == sizeof(message.bin)) {
      for (size_t index = 0; index < sizeof(message.bin); ++index) message.bin[index] = usher_protocol.payload[index];
      capture_arbiter_set_scope(&message, serial_receive_monotonic_us);
    }
    return;
  }
  if (usher_protocol.msg_id == USHER_CAPTURE_ARBITER_SHOOT) {
    union usher_capture_arbiter_shoot_union message;
    if (usher_protocol.payload_len != sizeof(message.bin)) return;
    for (size_t index = 0; index < sizeof(message.bin); ++index) message.bin[index] = usher_protocol.payload[index];
    if (message.data.session != capture_arbiter_session_id() || !message.data.session
        || message.data.camera_mask > USHER_CAMERA_MASK_ALL) return;
    for (unsigned camera_id = 1; camera_id <= CAMERA_BACKEND_MAX_ID; ++camera_id) {
      if ((message.data.camera_mask & (1U << (camera_id - 1))) && prepare_session_backend(camera_id))
        camera_owners[camera_id].record(&message.data.shot);
    }
    capture_arbiter_offer(&message, serial_receive_monotonic_us);
    pthread_mutex_lock(&mut);
    last_photo_request_us = serial_receive_monotonic_us;
    pthread_mutex_unlock(&mut);
    return;
  }

  if (usher_protocol.msg_id == USHER_CLOCK_REPLY) {
    if (clock_probes_enabled) {
      if (clock_alignment_reply(&fc_clock, usher_protocol.payload, usher_protocol.payload_len,
                                serial_receive_monotonic_us)) ++clock_reply_count;
      else ++clock_rejected_count;
    }
    return;
  }
  if (usher_protocol.msg_id == USHER_POSE_CLOCKED) {
    record_clocked_pose();
    return;
  }
  if (usher_protocol.msg_id == USHER_POSE_SAMPLE) {
    fc_clock.valid = false;
    capture_arbiter_pose_invalidate();
    pose_log_record(usher_protocol.payload, usher_protocol.payload_len, serial_receive_monotonic_us);
    camera_record_pose(usher_protocol.payload, usher_protocol.payload_len);
    return;
  }
  if (usher_protocol.msg_id == USHER_SHOOT_MASK) {
    if (debug_enabled) {
      printf("USHER DEBUG:\tphoto trigger received; decoding shot payload\n");
    }
    union dc_shot_mask_union message;
    if (usher_protocol.payload_len != sizeof(message.bin)) {
      fprintf(stderr, "USHER:\tinvalid USHER shot payload length %u\n", usher_protocol.payload_len);
      return;
    }
    for (size_t index = 0; index < sizeof(message.bin); ++index) message.bin[index] = usher_protocol.payload[index];
    const union dc_shot_union *shoot = &message.data.shot;
    if (!message.data.request_id || shoot->data.nr <= 0) return;
    uint32_t camera_mask = message.data.camera_mask;
    if (camera_mask > USHER_CAMERA_MASK_ALL) {
      fprintf(stderr, "USHER:\tinvalid camera mask %" PRIu32 "\n", camera_mask);
      return;
    }
    if (camera_mask == USHER_CAMERA_MASK_NONE) return;
    for (unsigned camera_id = 6; camera_id <= 8; ++camera_id) {
      if (camera_mask & (1U << (camera_id - 1)))
        queue_shot_result(message.data.request_id, shoot->data.nr, camera_id, USHER_SHOT_UNSUPPORTED);
    }
    camera_mask &= USHER_CAMERA_MASK_SUPPORTED;
    if (!camera_mask) return;
    printf("USHER:\tSHOT %d mask 0x%02" PRIx32 " | lat %.7f lon %.7f | MSL %.1f m AGL %.1f m | "
           "roll %.1f pitch %.1f yaw %.1f deg | speed %.1f m/s course %.1f deg\n",
           shoot->data.nr, camera_mask,
           shoot->data.lat / 1e7,
           shoot->data.lon / 1e7,
           shoot->data.alt / 1000.0,
           shoot->data.groundalt / POSITION_BFP_SCALE,
           shoot->data.phi / ANGLE_BFP_SCALE * RAD_TO_DEG,
           shoot->data.theta / ANGLE_BFP_SCALE * RAD_TO_DEG,
           shoot->data.psi / ANGLE_BFP_SCALE * RAD_TO_DEG,
           shoot->data.vground / SPEED_BFP_SCALE,
           shoot->data.course / ANGLE_BFP_SCALE * RAD_TO_DEG);

    for (unsigned camera_id = 1; camera_id <= CAMERA_BACKEND_MAX_ID; ++camera_id) {
      const struct camera_backend backend = camera_identity_get(camera_id);
      if ((camera_mask & (1U << (camera_id - 1))) && (backend.capabilities & CAMERA_CAP_POINT_STREAM)) {
        const struct camera_backend *owner = &camera_owners[camera_id];
        const bool recorded = camera_stream_states[camera_id] == CAMERA_STREAM_ACTIVE && owner->record && owner->record(shoot) == 0;
        queue_shot_result(message.data.request_id, shoot->data.nr, camera_id,
                          recorded ? USHER_SHOT_RECORDED : USHER_SHOT_FAILED);
        continue;
      }
      if (!(camera_mask & (1U << (camera_id - 1))) || !(backend.capabilities & CAMERA_CAP_POINT_SESSION)) continue;
      bool recorded = false;
      if (prepare_session_backend(camera_id)) {
        const struct camera_backend *owner = &camera_owners[camera_id];
        recorded = owner->session->listening() && owner->record(shoot) == 0;
      }
      queue_shot_result(message.data.request_id, shoot->data.nr, camera_id,
                        recorded ? USHER_SHOT_RECORDED : USHER_SHOT_FAILED);
    }
    if ((camera_mask & (USHER_CAMERA_MASK_CHDK | USHER_CAMERA_MASK_AICAM | USHER_CAMERA_MASK_LWIR)) != 0) {
      start_shoot_worker(shoot, (uint8_t)camera_mask, message.data.request_id);
    }
  } else if (usher_protocol.msg_id == USHER_STOP_TARGETED || usher_protocol.msg_id == USHER_START_TARGETED) {
    if (usher_protocol.payload_len != USHER_STOP_TARGETED_MSG_SIZE) {
      fprintf(stderr, "USHER:\tinvalid USHER session payload length %u\n", usher_protocol.payload_len);
      return;
    }
    union usher_camera_session_union command;
    for (size_t index = 0; index < sizeof(command.bin); ++index) {
      command.bin[index] = usher_protocol.payload[index];
    }
    if (command.data.camera_id > CAMERA_BACKEND_MAX_ID || !command.data.session_id) return;
    if (usher_protocol.msg_id == USHER_START_TARGETED) {
      handle_targeted_start(command.data.camera_id, command.data.session_id);
    } else {
      handle_targeted_stop(command.data.camera_id, command.data.session_id);
    }
  } else if (usher_protocol.msg_id == USHER_BUFFER_EMPTY) {
    send_msg_image_buffer();
  }
}

/** @brief Ensure a point-session backend has a registered owner.
 * @param camera_id Protocol camera ID.
 * @return True once a complete session backend is initialized.
 * @details Check all lifecycle hooks before initialization and reuse an
 * existing owner rather than reopening a session-capable device. */
static bool prepare_session_backend(unsigned camera_id)
{
  if (camera_id < 1 || camera_id > CAMERA_BACKEND_MAX_ID) return false;
  const struct camera_backend backend = camera_identity_get(camera_id);
  const struct camera_session_ops *ops = backend.session;
  if (!(backend.capabilities & CAMERA_CAP_POINT_SESSION) || !ops || !ops->start || !ops->stop
      || !ops->listening || !ops->completion_fd || !ops->poll_result || !ops->drain
      || !backend.init || !backend.deinit || !backend.record) return false;
  if (camera_initialized[camera_id]) return true;
  const struct camera_backend_host host = {.running = &keep_running,
                                             .pose_lookup = capture_arbiter_observation_pose, .queue_analysis = defer_soda
  };
  if (backend.configure) backend.configure(&host);
  if (backend.init(NULL) != 0) {
    backend.deinit();
    fprintf(stderr, "USHER:\t%s not available; other cameras remain active\n", backend.name);
    return false;
  }
  camera_owners[camera_id] = backend;
  camera_initialized[camera_id] = true;
  return true;
}

/** @brief Resolve runtime routing without mutating the global selected backend.
 * @param camera_id Protocol camera identity.
 * @return Driver type, including configured AI detection and local overrides.
 */
static enum camera_backend_type camera_type_for_id(int camera_id)
{
  return camera_backend_registry_route(camera_id, configured_camera_backend, local_capture_only);
}

/** @brief Select and lazily initialize a backend under the caller's camera ownership.
 * @param camera_id Image-camera identity, not a mask.
 * @return Zero when ready, negative on invalid input, shutdown or backend failure.
 * @details A failed optional camera must not stop the other bits of a mixed request.
 */
static int camera_prepare(int camera_id)
{
  if (!usher_running() || camera_id < USHER_CAMERA_CHDK || camera_id > USHER_CAMERA_LWIRCAM) return -1;
  const enum camera_backend_type type = camera_type_for_id(camera_id);
  if (camera_backend_select(type, test_capture_enabled) != 0) return -1;
  optical_camera_id = camera_id;
  if (camera_initialized[camera_id]) {
    const struct camera_backend *owner = &camera_owners[camera_id];
    if (owner->init != camera.init || owner->shoot != camera.shoot || owner->deinit != camera.deinit
        || owner->capture_ops != camera.capture_ops || owner->capabilities != camera.capabilities
        || owner->point_stream != camera.point_stream || owner->image_saved != camera.image_saved
        || owner->prefix != camera.prefix) {
      fprintf(stderr, "USHER:\tcamera id %d backend change requires completed teardown\n", camera_id);
      return -1;
    }
    camera = *owner;
    return 0;
  }
  if (camera_id == USHER_CAMERA_LWIRCAM && camera_unavailable[camera_id]) {
    uint64_t now_us = monotonic_time_us();
    if (now_us == 0 || now_us < lwir_retry_after_us) return -1;
    camera_unavailable[camera_id] = false;
    printf("USHER:\tretrying LWIR camera after unavailable cooldown\n");
  }
  if (camera_register_stream(&camera) != 0) return -1;
  const struct camera_backend_host host = {.running = &keep_running,
    .pose_lookup = capture_arbiter_observation_pose, .queue_analysis = defer_soda};
  if (camera.configure) camera.configure(&host);
  if (camera.init(camera_source_image) != 0) {
    camera.deinit();
    if (camera_id == USHER_CAMERA_LWIRCAM) lwir_mark_backoff();
    if (!camera_warned[camera_id]) {
      camera_warned[camera_id] = true;
      fprintf(stderr, "USHER:\t%s (camera id %d) not available; continuing without it\n",
              camera.name, camera_id);
    } else if (debug_enabled) {
      printf("USHER DEBUG:\t%s (camera id %d) still not available\n", camera.name, camera_id);
    }
    return -1;
  }
  camera_warned[camera_id] = false;
  if (camera_id == USHER_CAMERA_LWIRCAM) {
    camera_unavailable[camera_id] = false;
    lwir_retry_after_us = 0;
  }
  if (!camera.point_stream) camera_owners[camera_id] = camera;
  camera_initialized[camera_id] = true;
  printf("USHER:\tactive camera backend: %s (id %d)\n", camera.name, optical_camera_id);
  return 0;
}

/** @brief Rate-limit thermal recovery so a missing USB camera does not consume every request. */
static void lwir_mark_backoff(void)
{
  uint64_t now_us = monotonic_time_us();
  pthread_mutex_lock(&mut);
  camera_initialized[USHER_CAMERA_LWIRCAM] = false;
  camera_unavailable[USHER_CAMERA_LWIRCAM] = true;
  lwir_retry_after_us = now_us == 0 || now_us > UINT64_MAX - LWIR_RETRY_COOLDOWN_US
                        ? UINT64_MAX : now_us + LWIR_RETRY_COOLDOWN_US;
  pthread_mutex_unlock(&mut);
}

/** @brief Retry thermal startup outside both UART reception and the optical lane.
 * @param unused Unused pthread argument.
 * @return NULL after recovery or shutdown; completion state is signalled under mut.
 */
static void *recover_lwir_camera(void *unused)
{
  (void)unused;
  bool retrying = false;
  while (usher_running()) {
    pthread_mutex_lock(&mut);
    uint64_t now_us = monotonic_time_us();
    bool cooling_down = camera_unavailable[USHER_CAMERA_LWIRCAM]
                        && (now_us == 0 || now_us < lwir_retry_after_us);
    pthread_mutex_unlock(&mut);
    if (cooling_down) {
      sleep_microseconds(LWIR_RETRY_POLL_US);
      continue;
    }
    if (retrying) printf("USHER:\tretrying LWIR camera after unavailable cooldown\n");

    pthread_mutex_lock(&lwir_camera_mutex);
    if (!usher_running()) {
      pthread_mutex_unlock(&lwir_camera_mutex);
      break;
    }
    const struct camera_backend backend = camera_backend_get(CAMERA_BACKEND_LWIR_CAM);
    int result = backend.init_cancelable(NULL, &keep_running);
    if (result == 0 && !usher_running()) result = -1;
    if (result != 0) backend.deinit();
    pthread_mutex_unlock(&lwir_camera_mutex);

    pthread_mutex_lock(&mut);
    if (result == 0) {
      camera_owners[USHER_CAMERA_LWIRCAM] = backend;
      camera_initialized[USHER_CAMERA_LWIRCAM] = true;
      camera_unavailable[USHER_CAMERA_LWIRCAM] = false;
      camera_warned[USHER_CAMERA_LWIRCAM] = false;
      lwir_retry_after_us = 0;
      lwir_recovery_in_progress = false;
      printf("USHER:\tactive camera backend: lwircam (id %d)\n", USHER_CAMERA_LWIRCAM);
      pthread_cond_broadcast(&workers_finished);
      pthread_mutex_unlock(&mut);
      return NULL;
    }
    camera_initialized[USHER_CAMERA_LWIRCAM] = false;
    camera_unavailable[USHER_CAMERA_LWIRCAM] = true;
    now_us = monotonic_time_us();
    lwir_retry_after_us = now_us == 0 || now_us > UINT64_MAX - LWIR_RETRY_COOLDOWN_US
                          ? UINT64_MAX : now_us + LWIR_RETRY_COOLDOWN_US;
    if (!camera_warned[USHER_CAMERA_LWIRCAM]) {
      camera_warned[USHER_CAMERA_LWIRCAM] = true;
      fprintf(stderr, "USHER:\tlwircam (camera id %d) not available; AIcam remains active\n",
              USHER_CAMERA_LWIRCAM);
    }
    pthread_cond_broadcast(&workers_finished);
    pthread_mutex_unlock(&mut);
    retrying = true;
  }

  pthread_mutex_lock(&mut);
  lwir_recovery_in_progress = false;
  pthread_cond_broadcast(&workers_finished);
  pthread_mutex_unlock(&mut);
  return NULL;
}

/** @brief Check thermal readiness or start at most one background recovery attempt.
 * @return True only when already initialized; false does not mean recovery is finished.
 */
static bool lwir_ready_or_start_recovery(void)
{
  pthread_mutex_lock(&mut);
  if (camera_initialized[USHER_CAMERA_LWIRCAM]) {
    pthread_mutex_unlock(&mut);
    return true;
  }
  if (!usher_running() || lwir_recovery_in_progress) {
    pthread_mutex_unlock(&mut);
    return false;
  }
  lwir_recovery_in_progress = true;
  pthread_mutex_unlock(&mut);

  pthread_t recovery_thread;
  int thread_result = pthread_create(&recovery_thread, NULL, recover_lwir_camera, NULL);
  if (thread_result != 0) {
    pthread_mutex_lock(&mut);
    lwir_recovery_in_progress = false;
    pthread_cond_broadcast(&workers_finished);
    pthread_mutex_unlock(&mut);
    fprintf(stderr, "USHER:\tfailed to start LWIR recovery: %s\n", strerror(thread_result));
    lwir_mark_backoff();
    return false;
  }
  pthread_detach(recovery_thread);
  return false;
}

/** @brief Admit an optional stream once per owner, without opening its sensor.
 * @details Failed optional startup is cleaned up once and does not disable photos.
 * Changing profiles requires teardown, even before the first sensor acquisition.
 */
static int camera_register_stream(const struct camera_backend *backend)
{
  if (!backend || backend->camera_id > CAMERA_BACKEND_MAX_ID) return -1;
  const unsigned camera_id = backend->camera_id;
  if (camera_stream_states[camera_id] != CAMERA_STREAM_CLOSED
      && camera_owners[camera_id].point_stream != backend->point_stream) return -1;
  if (!backend->point_stream) return 0;
  if (!camera_id || camera_stream_states[camera_id] == CAMERA_STREAM_DRAINED) return -1;
  if (camera_initialized[camera_id] && camera_owners[camera_id].point_stream != backend->point_stream) return -1;
  if (camera_stream_states[camera_id] != CAMERA_STREAM_CLOSED) return 0;
  camera_owners[camera_id] = *backend;
  const struct camera_point_stream_ops *stream = backend->point_stream;
  if (stream->init && stream->init() != 0) {
    camera_stream_states[camera_id] = CAMERA_STREAM_UNAVAILABLE;
    if (stream->drain) stream->drain();
  } else camera_stream_states[camera_id] = CAMERA_STREAM_ACTIVE;
  return 0;
}

/** @brief Withdraw one backend from service and release its resources.
 * @param camera_id Owner slot, including the default slot.
 * @details Clear dispatch state, drain an active point stream, then release
 * the initialized backend. Clear its result caches before the slot is reused. */
static void camera_close_owner(unsigned camera_id)
{
  if (camera_id > CAMERA_BACKEND_MAX_ID) return;
  const struct camera_backend owner = camera_owners[camera_id];
  const bool initialized = camera_initialized[camera_id];
  const bool stream_active = camera_stream_states[camera_id] == CAMERA_STREAM_ACTIVE;
  camera_stream_states[camera_id] = CAMERA_STREAM_CLOSED;
  camera_owners[camera_id] = (struct camera_backend) {0};
  camera_initialized[camera_id] = false;
  if (stream_active && owner.point_stream && owner.point_stream->drain) owner.point_stream->drain();
  if (owner.point_stream && camera_id) {
    memset(&point_sessions[camera_id - 1], 0, sizeof(point_sessions[camera_id - 1]));
    point_update_pending[camera_id - 1] = false;
  }
  if (initialized && owner.deinit) owner.deinit();
}

/** @brief Tear down registered cameras and independently owned analysis workers.
 * @details Walk the owner registry first, then stop auxiliary workers whose
 * lifetime is not completely represented by a registry slot. */
static void cameras_deinit(void)
{
  for (unsigned camera_id = USHER_CAMERA_ALL; camera_id <= CAMERA_BACKEND_MAX_ID; ++camera_id)
    camera_close_owner(camera_id);
  spotter_cam_pipe_deinit();
  lwir_spotter_stop();
  const struct camera_backend loom_capture = loom_camera_backend_get(false);
  if (loom_capture.deinit) loom_capture.deinit();
}

/** @brief Own thermal capture and its last-frame metadata independently of optical work.
 * @param pointer Pool-owned request, released after capture handling.
 * @return NULL after releasing the thermal in-progress flag.
 */
static void *handle_lwir_shoot(void *pointer)
{
  struct capture_job *job = pointer;
  pthread_mutex_lock(&lwir_camera_mutex);
  bool saved = false;
  if (usher_running()) {
    const struct camera_backend backend = camera_owners[USHER_CAMERA_LWIRCAM];
    struct captured_image image;
    if (capture_image(&job->shot, &image, USHER_CAMERA_LWIRCAM, &backend)) {
      saved = finish_image(&job->shot, &image);
    }
    lwir_ready_or_start_recovery();
  }
  pthread_mutex_unlock(&lwir_camera_mutex);
  queue_shot_result(job->request_id, job->shot.data.nr, USHER_CAMERA_LWIRCAM,
                    saved ? USHER_SHOT_SAVED : USHER_SHOT_FAILED);
  pthread_mutex_lock(&mut);
  capture_job_release_locked(job);
  lwir_capture_in_progress = false;
  const bool pending = waiting_lwir.valid;
  lwir_handoff_in_progress = pending;
  const union dc_shot_union next_shot = pending ? waiting_lwir.shot : (union dc_shot_union) {0};
  const uint32_t next_request_id = pending ? waiting_lwir.request_id : 0;
  const uint64_t received_us = pending ? waiting_lwir.received_us : 0;
  waiting_lwir.valid = false;
  pthread_cond_broadcast(&workers_finished);
  pthread_mutex_unlock(&mut);
  if (pending) {
    const uint64_t now_us = monotonic_time_us();
    if (usher_running() && now_us >= received_us && now_us - received_us <= LWIR_WAIT_MAX_US)
      start_lwir_worker(&next_shot, next_request_id, received_us);
    else {
      capture_arbiter_failed(&next_shot, USHER_CAMERA_MASK_LWIR);
      queue_shot_result(next_request_id, next_shot.data.nr, USHER_CAMERA_LWIRCAM, USHER_SHOT_FAILED);
    }
    pthread_mutex_lock(&mut);
    lwir_handoff_in_progress = false;
    pthread_cond_broadcast(&workers_finished);
    pthread_mutex_unlock(&mut);
  }
  return NULL;
}

/** @brief Dispatch a single-flight thermal request without waiting for USB recovery.
 * @param[in] shoot Request copied into worker-owned storage on admission.
 * @param[in] request_id Correlation ID returned to the flight controller; distinct from photo number.
 * @param[in] received_us Monotonic time when the request was admitted, used to reject stale queued work.
 * @details This function is called by the serial owner. Failed admission releases
 * matching survey ownership so a missing thermal camera cannot remain permanently busy.
 */
static void start_lwir_worker(const union dc_shot_union *shoot, uint32_t request_id, uint64_t received_us)
{
  pthread_mutex_lock(&mut);
  if (!usher_running()) {
    pthread_cond_broadcast(&workers_finished);
    pthread_mutex_unlock(&mut);
    capture_arbiter_failed(shoot, USHER_CAMERA_MASK_LWIR);
    queue_shot_result(request_id, shoot->data.nr, USHER_CAMERA_LWIRCAM, USHER_SHOT_FAILED);
    return;
  }
  if (lwir_capture_in_progress) {
    ++lwir_busy_skipped_count;
    if (lwir_active_received_us > received_us
        || (waiting_lwir.valid && waiting_lwir.received_us > received_us)) {
      pthread_mutex_unlock(&mut);
      capture_arbiter_failed(shoot, USHER_CAMERA_MASK_LWIR);
      queue_shot_result(request_id, shoot->data.nr, USHER_CAMERA_LWIRCAM, USHER_SHOT_SUPERSEDED);
      return;
    }
    const bool replaced = waiting_lwir.valid;
    const union dc_shot_union old_shot = replaced ? waiting_lwir.shot : (union dc_shot_union) {0};
    const uint32_t old_request_id = replaced ? waiting_lwir.request_id : 0;
    waiting_lwir.shot = *shoot;
    waiting_lwir.request_id = request_id;
    waiting_lwir.received_us = received_us;
    waiting_lwir.valid = true;
    pthread_cond_broadcast(&workers_finished);
    pthread_mutex_unlock(&mut);
    if (replaced) {
      capture_arbiter_failed(&old_shot, USHER_CAMERA_MASK_LWIR);
      queue_shot_result(old_request_id, old_shot.data.nr, USHER_CAMERA_LWIRCAM, USHER_SHOT_SUPERSEDED);
    }
    return;
  }
  pthread_mutex_unlock(&mut);
  if (!lwir_ready_or_start_recovery()) {
    capture_arbiter_failed(shoot, USHER_CAMERA_MASK_LWIR);
    queue_shot_result(request_id, shoot->data.nr, USHER_CAMERA_LWIRCAM, USHER_SHOT_FAILED);
    return;
  }
  pthread_mutex_lock(&mut);
  struct capture_job *job = capture_job_acquire_locked();
  if (job == NULL) {
    pthread_mutex_unlock(&mut);
    fprintf(stderr, "USHER-%d:\tLWIR job pool is full\n", shoot->data.nr);
    capture_arbiter_failed(shoot, USHER_CAMERA_MASK_LWIR);
    queue_shot_result(request_id, shoot->data.nr, USHER_CAMERA_LWIRCAM, USHER_SHOT_FAILED);
    return;
  }
  job->shot = *shoot;
  job->request_id = request_id;
  lwir_active_received_us = received_us;
  lwir_capture_in_progress = true;
  pthread_t worker;
  int result = pthread_create(&worker, NULL, handle_lwir_shoot, job);
  if (result != 0) {
    lwir_capture_in_progress = false;
    capture_job_release_locked(job);
    fprintf(stderr, "USHER-%d:\tfailed to start LWIR worker: %s\n", shoot->data.nr, strerror(result));
  } else {
    pthread_detach(worker);
  }
  pthread_mutex_unlock(&mut);
  if (result != 0) {
    capture_arbiter_failed(shoot, USHER_CAMERA_MASK_LWIR);
    queue_shot_result(request_id, shoot->data.nr, USHER_CAMERA_LWIRCAM, USHER_SHOT_FAILED);
  }
}

/** @brief Pause new CHDK attempts after a failed request; the caller holds mut. */
static void chdk_start_cooldown_locked(void)
{
  const uint64_t now_us = monotonic_time_us();
  chdk_retry_after_us = now_us && now_us <= UINT64_MAX - CHDK_RETRY_COOLDOWN_US
    ? now_us + CHDK_RETRY_COOLDOWN_US : UINT64_MAX;
}

/** @brief Write metadata for one captured CHDK image after the camera lane was released.
 * @param pointer Unused; the single-flight job lives in chdk_finish.
 * @return NULL after reporting the result and clearing the finisher flag. */
static void *handle_chdk_finish(void *pointer)
{
  (void)pointer;
  const bool saved = finish_image(&chdk_finish.job.shot, &chdk_finish.image);
  if (!saved && usher_running())
    fprintf(stderr, "USHER-%d: CHDK request failed; new CHDK attempts paused for %.3f s; other camera lanes remain independent\n",
            chdk_finish.job.shot.data.nr, CHDK_RETRY_COOLDOWN_US / 1e6);
  queue_shot_result(chdk_finish.job.request_id, chdk_finish.job.shot.data.nr, USHER_CAMERA_CHDK,
                    saved ? USHER_SHOT_SAVED : USHER_SHOT_FAILED);
  pthread_mutex_lock(&mut);
  if (!saved) chdk_start_cooldown_locked();
  chdk_finish_in_progress = false;
  pthread_cond_broadcast(&workers_finished);
  pthread_mutex_unlock(&mut);
  return NULL;
}

/** @brief Hand a captured image to the finisher, waiting if the previous one is still writing.
 * @return False when no finisher thread could start; the caller then finishes the image inline. */
static bool start_chdk_finish(const struct capture_job *job, const struct captured_image *image)
{
  pthread_mutex_lock(&mut);
  while (chdk_finish_in_progress) pthread_cond_wait(&workers_finished, &mut);
  chdk_finish.job = *job;
  chdk_finish.image = *image;
  chdk_finish_in_progress = true;
  pthread_attr_t attributes;
  int result = pthread_attr_init(&attributes);
  const bool attributes_ready = result == 0;
  if (!result) result = pthread_attr_setstacksize(&attributes, 256U * 1024U);
  if (!result) result = pthread_attr_setdetachstate(&attributes, PTHREAD_CREATE_DETACHED);
  pthread_t worker;
  if (!result) result = pthread_create(&worker, &attributes, handle_chdk_finish, NULL);
  if (attributes_ready) pthread_attr_destroy(&attributes);
  if (result) {
    chdk_finish_in_progress = false;
    pthread_cond_broadcast(&workers_finished);
  }
  pthread_mutex_unlock(&mut);
  if (result) fprintf(stderr, "USHER-%d: unable to start CHDK finisher: %s; finishing inline\n",
                      job->shot.data.nr, strerror(result));
  return result == 0;
}

/** @brief Capture one CHDK request and release the camera lane before metadata is written.
 * @param pointer Dedicated single-flight request storage, retained until the capture ends.
 * @return NULL after teardown and signalling worker completion.
 * @details A captured image moves to handle_chdk_finish(), which owns the shot result,
 * so EXIF and publication overlap the next camera operation. */
static void *handle_chdk_shoot(void *pointer)
{
  const struct capture_job *job = pointer;
  const struct camera_backend backend = camera_backend_get(camera_type_for_id(USHER_CAMERA_CHDK));
  bool saved = false;
  bool handed_off = false;
  bool initialized = false;
  if (usher_running()) {
    const struct camera_backend_host host = {.running = &keep_running,
      .pose_lookup = capture_arbiter_observation_pose, .queue_analysis = defer_soda};
    if (backend.configure) backend.configure(&host);
    initialized = backend.init && backend.init(camera_source_image) == 0;
    if (initialized) {
      struct captured_image image;
      if (capture_image(&job->shot, &image, USHER_CAMERA_CHDK, &backend)) {
        handed_off = start_chdk_finish(job, &image);
        if (!handed_off) saved = finish_image(&job->shot, &image);
      }
    }
  }
  if (backend.deinit) backend.deinit();
  if (!initialized) capture_arbiter_failed(&job->shot, USHER_CAMERA_MASK_CHDK);
  if (!saved && !handed_off) {
    if (usher_running())
      fprintf(stderr, "USHER-%d: CHDK request failed; new CHDK attempts paused for %.3f s; other camera lanes remain independent\n",
              job->shot.data.nr, CHDK_RETRY_COOLDOWN_US / 1e6);
    queue_shot_result(job->request_id, job->shot.data.nr, USHER_CAMERA_CHDK, USHER_SHOT_FAILED);
  } else if (saved) {
    queue_shot_result(job->request_id, job->shot.data.nr, USHER_CAMERA_CHDK, USHER_SHOT_SAVED);
  }
  pthread_mutex_lock(&mut);
  if (saved || handed_off) chdk_retry_after_us = 0;
  else chdk_start_cooldown_locked();
  chdk_capture_in_progress = false;
  pthread_cond_broadcast(&workers_finished);
  pthread_mutex_unlock(&mut);
  return NULL;
}

/** @brief Resolve the currently advertised control capabilities.
 * @param camera_id Protocol camera ID.
 * @return Supported command bits, or zero when controls are unavailable.
 * @details Advertise physical lens operations only when the asynchronous
 * CHDK lane and its required hooks are available, never in mock capture modes. */
static uint32_t camera_control_capabilities(uint8_t camera_id)
{
  if (camera_id != USHER_CAMERA_CHDK || !chdk_async_capture_enabled || test_capture_enabled
      || local_capture_only || mock_map_enabled) return 0;
  const struct camera_backend backend = camera_identity_get(camera_id);
  return backend.control && backend.control_init ? backend.control_capabilities : 0;
}

/** @brief Stage a worker's result for serial transmission.
 * @param result Completion including its original request identity.
 * @details Update only a matching session and request slot so late work
 * cannot overwrite state from a renewed control session. */
static void queue_control_result(const struct usher_control_result *result)
{
  pthread_mutex_lock(&mut);
  if (result->request.session == control_session && result->camera_id >= 1 && result->camera_id <= 8) {
    const size_t index = result->camera_id - 1;
    if (control_results[index].result.request.request_id == result->request.request_id) {
      control_results[index].result = *result;
      control_results[index].dirty = true;
    }
  }
  pthread_mutex_unlock(&mut);
  wake_event_loop();
}

/** @brief Pass dirty control receipts to the UART transmit queue.
 * @details Clear a slot's dirty flag only after successful queue admission;
 * preserve the cached receipt for duplicate requests. */
static void flush_control_results(void)
{
  pthread_mutex_lock(&mut);
  for (size_t index = 0; index < 8; ++index) {
    if (!control_results[index].dirty) continue;
    uint8_t bytes[USHER_CONTROL_RESULT_SIZE];
    if (usher_control_encode_result(bytes, sizeof(bytes), &control_results[index].result)
        && serial_tx_send(USHER_CONTROL_RESULT, bytes, sizeof(bytes)) == 0) control_results[index].dirty = false;
  }
  pthread_mutex_unlock(&mut);
}

/** @brief Execute an admitted lens command away from the serial owner.
 * @param pointer Address of the request in the reserved CHDK control slot.
 * @return NULL after releasing the CHDK lane.
 * @details Copy the job, execute under exclusive device ownership and report
 * observation evidence only for applied controls. An unsuccessful operation
 * sets a cooldown before another command or capture may use the lane. */
static void *handle_chdk_control(void *pointer)
{
  const struct usher_control_request request = *(const struct usher_control_request *)pointer;
  const struct camera_backend backend = camera_identity_get(USHER_CAMERA_CHDK);
  struct usher_control_result result = {.request = request, .camera_id = USHER_CAMERA_CHDK,
    .status = CAM_CONTROL_FAILED};
  if (usher_running()) {
    const struct camera_backend_host host = {.running = &keep_running};
    if (backend.configure) backend.configure(&host);
    if (backend.control_init && backend.control && backend.control_init() == 0)
      result.status = backend.control(request.command, request.value, &result.observed, &result.evidence);
  }
  if (backend.deinit) backend.deinit();
  if (result.status != CAM_CONTROL_APPLIED) result.evidence = CAM_CONTROL_EVIDENCE_NONE;
  queue_control_result(&result);
  pthread_mutex_lock(&mut);
  const uint64_t now = monotonic_time_us();
  chdk_retry_after_us = result.status == CAM_CONTROL_APPLIED ? 0 : now && now <= UINT64_MAX - CHDK_RETRY_COOLDOWN_US
    ? now + CHDK_RETRY_COOLDOWN_US : UINT64_MAX;
  chdk_capture_in_progress = false;
  pthread_cond_broadcast(&workers_finished);
  pthread_mutex_unlock(&mut);
  return NULL;
}

/** @brief Admit a control into the exclusive CHDK lane under @c mut.
 * @param request Validated command to copy into the reserved slot.
 * @param now Current local monotonic time in microseconds.
 * @return False for busy, cooldown, or shutdown; true if handled, even if worker creation fails.
 * @details Claim the same lane used by CHDK capture. If spawn fails, release
 * the lane and stage a failure receipt rather than leaving a pending request. */
static bool admit_chdk_control_locked(const struct usher_control_request *request, uint64_t now)
{
  if (!usher_running() || chdk_capture_in_progress || now < chdk_retry_after_us) return false;
  chdk_control_job = *request;
  chdk_capture_in_progress = true;
  pthread_attr_t attributes;
  int error = pthread_attr_init(&attributes);
  bool initialized = error == 0;
  if (!error) error = pthread_attr_setstacksize(&attributes, 256U * 1024U);
  if (!error) error = pthread_attr_setdetachstate(&attributes, PTHREAD_CREATE_DETACHED);
  pthread_t worker;
  if (!error) error = pthread_create(&worker, &attributes, handle_chdk_control, &chdk_control_job);
  if (initialized) pthread_attr_destroy(&attributes);
  if (error) {
    chdk_capture_in_progress = false;
    control_results[0].result.status = CAM_CONTROL_FAILED;
    control_results[0].dirty = true;
    pthread_cond_broadcast(&workers_finished);
  }
  return true;
}

/** @brief Establish or renew a camera-control lease from the current UART message.
 * @details Pair a fresh session with the peer's nonce, report available
 * capabilities, and require a timely matching echo before accepting commands.
 * Replacing the session clears old receipt and ordering state. */
static void negotiate_camera_controls(void)
{
  const uint8_t *bytes = usher_protocol.payload;
  uint32_t nonce, requested_session;
  if (usher_protocol.payload_len != USHER_CONTROL_NEGOTIATE_SIZE || bytes[0] != USHER_CONTROL_VERSION
      || !usher_control_get_u32(bytes, usher_protocol.payload_len, 1, &nonce) || !nonce
      || !usher_control_get_u32(bytes, usher_protocol.payload_len, 5, &requested_session)) return;
  const uint64_t now = monotonic_time_us();
  if (!now) return;
  pthread_mutex_lock(&mut);
  if (!requested_session) {
    uint32_t fresh = 0;
    ssize_t count = getrandom(&fresh, sizeof(fresh), GRND_NONBLOCK);
    if (count != (ssize_t)sizeof(fresh) || !fresh || fresh == control_session) {
      pthread_mutex_unlock(&mut);
      return;
    }
    control_session = fresh;
    control_confirmed = false;
    control_nonce = nonce;
    control_high_water = 0;
    for (size_t index = 0; index < 8; ++index) {
      control_results[index].result = (struct usher_control_result){0};
      control_results[index].dirty = false;
    }
  }
  if (!control_session || nonce != control_nonce) { pthread_mutex_unlock(&mut); return; }
  if (requested_session == control_session && now - control_negotiated_us < USHER_CONTROL_LEASE_US)
    control_confirmed = true;
  else if (requested_session == control_session) { pthread_mutex_unlock(&mut); return; }
  if (!requested_session || requested_session == control_session) control_negotiated_us = now;
  uint8_t hello[USHER_CONTROL_HELLO_SIZE] = {USHER_CONTROL_VERSION};
  usher_control_put_u32(hello, sizeof(hello), 1, nonce);
  usher_control_put_u32(hello, sizeof(hello), 5, control_session);
  for (uint8_t camera_id = 1; camera_id <= 8; ++camera_id)
    usher_control_put_u32(hello, sizeof(hello), 9 + 4 * (camera_id - 1), camera_control_capabilities(camera_id));
  serial_tx_send(USHER_CONTROL_HELLO, hello, sizeof(hello));
  pthread_mutex_unlock(&mut);
}

/** @brief Process negotiation or dispatch an addressed control request.
 * @details Require an active confirmed lease. A request at or below the
 * session high-water mark cannot run again: report the matching cached
 * result when available, or report an unknown outcome. */
static void handle_control_message(void)
{
  if (usher_protocol.msg_id == USHER_CONTROL_NEGOTIATE) { negotiate_camera_controls(); return; }
  struct usher_control_request request;
  if (!usher_control_decode_request(usher_protocol.payload, usher_protocol.payload_len, &request)) return;
  const uint64_t now = monotonic_time_us();
  pthread_mutex_lock(&mut);
  if (!usher_running() || !control_confirmed || !now || request.session != control_session
      || now - control_negotiated_us >= USHER_CONTROL_LEASE_US) { pthread_mutex_unlock(&mut); return; }
  const bool duplicate = request.request_id <= control_high_water;
  if (!duplicate) control_high_water = request.request_id;
  for (uint8_t camera_id = 1; camera_id <= 8; ++camera_id) {
    if (!(request.camera_mask & (1U << (camera_id - 1)))) continue;
    const size_t index = camera_id - 1;
    if (duplicate) {
      const struct usher_control_result *cached = &control_results[index].result;
      if (cached->request.request_id == request.request_id && cached->request.command == request.command
          && cached->request.value == request.value && cached->request.camera_mask == request.camera_mask) {
        if (cached->status != CAM_CONTROL_PENDING) control_results[index].dirty = true;
      } else {
        const struct usher_control_result unknown = {.request = request, .camera_id = camera_id,
          .status = CAM_CONTROL_UNKNOWN};
        uint8_t bytes[USHER_CONTROL_RESULT_SIZE];
        if (usher_control_encode_result(bytes, sizeof(bytes), &unknown)) serial_tx_send(USHER_CONTROL_RESULT, bytes, sizeof(bytes));
      }
      continue;
    }
    if (control_results[index].result.status == CAM_CONTROL_PENDING) {
      const struct usher_control_result busy = {.request = request, .camera_id = camera_id, .status = CAM_CONTROL_BUSY};
      uint8_t bytes[USHER_CONTROL_RESULT_SIZE];
      if (usher_control_encode_result(bytes, sizeof(bytes), &busy)) serial_tx_send(USHER_CONTROL_RESULT, bytes, sizeof(bytes));
      continue;
    }
    control_results[index].result = (struct usher_control_result){.request = request, .camera_id = camera_id,
      .status = CAM_CONTROL_UNSUPPORTED};
    control_results[index].dirty = true;
    const uint32_t capabilities = camera_control_capabilities(camera_id);
    if (request.command >= 32 || !(capabilities & CAM_CONTROL_CAP(request.command))) continue;
    if (!usher_control_value_valid(request.command, request.value)) {
      control_results[index].result.status = CAM_CONTROL_INVALID_VALUE;
      continue;
    }
    if (admit_chdk_control_locked(&request, now)) {
      if (control_results[index].result.status != CAM_CONTROL_FAILED) {
        control_results[index].result.status = CAM_CONTROL_PENDING;
        control_results[index].dirty = false;
      }
    } else control_results[index].result.status = CAM_CONTROL_BUSY;
  }
  pthread_mutex_unlock(&mut);
}

/** @brief Schedule a shot on CHDK's independent capture lane.
 * @param shoot Trigger copied into the lane's single job slot.
 * @param request_id Correlation ID, or zero for an unacknowledged request.
 * @return True when the detached worker was started.
 * @details Share admission with lens control. Release survey admission and
 * report failure if shutdown, cooldown, contention, or thread setup prevents a shot. */
static bool start_chdk_worker(const union dc_shot_union *shoot, uint32_t request_id)
{
  pthread_mutex_lock(&mut);
  uint64_t now_us = monotonic_time_us();
  if (!usher_running() || chdk_capture_in_progress || !now_us || now_us < chdk_retry_after_us) {
    pthread_mutex_unlock(&mut);
    capture_arbiter_failed(shoot, USHER_CAMERA_MASK_CHDK);
    queue_shot_result(request_id, shoot->data.nr, USHER_CAMERA_CHDK, USHER_SHOT_FAILED);
    return false;
  }
  chdk_capture_job = (struct capture_job){.shot = *shoot, .request_id = request_id,
    .camera_mask = USHER_CAMERA_MASK_CHDK, .received_us = now_us};
  chdk_capture_in_progress = true;
  pthread_attr_t attributes;
  int result = pthread_attr_init(&attributes);
  bool attributes_ready = result == 0;
  if (!result) result = pthread_attr_setstacksize(&attributes, 256U * 1024U);
  if (!result) result = pthread_attr_setdetachstate(&attributes, PTHREAD_CREATE_DETACHED);
  pthread_t worker;
  if (!result) result = pthread_create(&worker, &attributes, handle_chdk_shoot, &chdk_capture_job);
  if (attributes_ready) pthread_attr_destroy(&attributes);
  if (result) {
    chdk_capture_in_progress = false;
    pthread_cond_broadcast(&workers_finished);
  }
  pthread_mutex_unlock(&mut);
  if (result) {
    fprintf(stderr, "USHER: unable to start CHDK worker: %s; other cameras remain active\n", strerror(result));
    capture_arbiter_failed(shoot, USHER_CAMERA_MASK_CHDK);
    queue_shot_result(request_id, shoot->data.nr, USHER_CAMERA_CHDK, USHER_SHOT_FAILED);
  }
  return result == 0;
}

/** @brief Dispatch admitted image bits while preserving bounded legacy/local policies.
 * @param shoot Request to copy; the caller need not retain its storage.
 * @param camera_mask Selected image cameras; live CHDK and LWIR have independent lanes.
 * @param request_id Per-request reply identity, distinct from the photo number.
 * @details Survey jobs retain their ledger ownership; local/test jobs stay FIFO. */
static void start_shoot_worker(const union dc_shot_union *shoot, uint8_t camera_mask, uint32_t request_id)
{
  pthread_mutex_lock(&mut);
  last_photo_request_us = monotonic_time_us();
  pthread_mutex_unlock(&mut);
  const bool live_capture = lwir_async_recovery_enabled && !test_capture_enabled && !local_capture_only;
  bool chdk_admitted = false;
  if (chdk_async_capture_enabled && !test_capture_enabled && !local_capture_only && !mock_map_enabled) {
    if (camera_mask & USHER_CAMERA_MASK_CHDK) chdk_admitted = start_chdk_worker(shoot, request_id);
    camera_mask &= ~USHER_CAMERA_MASK_CHDK;
  }
  if (chdk_admitted) {
    pthread_mutex_lock(&mut);
    ++shooting_count;
    pthread_mutex_unlock(&mut);
  }
  if (live_capture) {
    if ((camera_mask & USHER_CAMERA_MASK_LWIR) != 0) start_lwir_worker(shoot, request_id, monotonic_time_us());
    camera_mask &= ~USHER_CAMERA_MASK_LWIR;
  }
  if ((camera_mask & (USHER_CAMERA_MASK_CHDK | USHER_CAMERA_MASK_AICAM | USHER_CAMERA_MASK_LWIR)) == 0) {
    if (chdk_admitted) send_msg_status();
    return;
  }
  pthread_mutex_lock(&mut);
  if (!usher_running()) {
    pthread_mutex_unlock(&mut);
    capture_arbiter_failed(shoot, camera_mask);
    queue_image_results(request_id, shoot->data.nr, camera_mask, USHER_SHOT_FAILED);
    return;
  }
  if (live_capture && waiting_capture != NULL && !waiting_capture->arbiter_owned) {
    /* Replace the whole request, not just its number: old pose with a new image
     * would be harder to detect than an explicitly superseded trigger. */
    const union dc_shot_union replaced = waiting_capture->shot;
    const uint32_t replaced_id = waiting_capture->request_id;
    const uint8_t replaced_mask = waiting_capture->camera_mask;
    waiting_capture->shot = *shoot;
    waiting_capture->request_id = request_id;
    waiting_capture->camera_mask = camera_mask;
    waiting_capture->received_us = monotonic_time_us();
    waiting_capture->shot_counted = chdk_admitted;
    ++superseded_capture_count;
    pthread_mutex_unlock(&mut);
    queue_image_results(replaced_id, replaced.data.nr, replaced_mask, USHER_SHOT_SUPERSEDED);
    return;
  }
  if (shooting_thread_count >= MAX_PROCESSING_THREADS) {
    pthread_mutex_unlock(&mut);
    fprintf(stderr, "USHER-%d:\tprocessing queue is full\n", shoot->data.nr);
    capture_arbiter_failed(shoot, camera_mask);
    queue_image_results(request_id, shoot->data.nr, camera_mask, USHER_SHOT_FAILED);
    return;
  }
  struct capture_job *job = capture_job_acquire_locked();
  if (job == NULL) {
    pthread_mutex_unlock(&mut);
    fprintf(stderr, "USHER-%d:\tcapture job pool is full\n", shoot->data.nr);
    capture_arbiter_failed(shoot, camera_mask);
    queue_image_results(request_id, shoot->data.nr, camera_mask, USHER_SHOT_FAILED);
    return;
  }
  job->shot = *shoot;
  job->request_id = request_id;
  job->camera_mask = camera_mask;
  job->received_us = monotonic_time_us();
  job->arbiter_owned = capture_arbiter_owns(shoot->data.nr);
  job->shot_counted = chdk_admitted;
  shooting_thread_count++;
  job->ticket = next_job_ticket;

  pthread_t shooting_thread;
  int thread_result = pthread_create(&shooting_thread, NULL, handle_msg_shoot, job);
  if (thread_result != 0) {
    shooting_thread_count--;
    pthread_mutex_unlock(&mut);
    fprintf(stderr, "USHER-%d:\tfailed to start shooting thread: %s\n",
            shoot->data.nr, strerror(thread_result));
    capture_job_release(job);
    capture_arbiter_failed(shoot, camera_mask);
    queue_image_results(request_id, shoot->data.nr, camera_mask, USHER_SHOT_FAILED);
    return;
  }
  ++next_job_ticket;
  if (live_capture) waiting_capture = job;
  pthread_mutex_unlock(&mut);
  pthread_detach(shooting_thread);
  send_msg_status();
}

/** @brief Release per-camera survey admission on a failure before normal completion.
 * @param shoot Original request identifying active entries.
 * @param mask Only the camera bits that failed; other sensors may still succeed.
 */
static void capture_arbiter_failed(const union dc_shot_union *shoot, uint8_t mask)
{
  for (unsigned camera_id = 1; camera_id <= COVERAGE_CAMERAS; ++camera_id)
    if (mask & (1U << (camera_id - 1))) capture_arbiter_complete(shoot, camera_id, false, false, 0, monotonic_time_us());
}

/** @brief Reserve a zeroed slot in the bounded job pool.
 * @return Slot owned by the caller, or NULL if the pool is full.
 * @details The caller holds @c mut. A static pool makes overload explicit
 * rather than allocating more jobs as capture latency grows. */
static struct capture_job *capture_job_acquire_locked(void)
{
  for (size_t index = 0; index < CAPTURE_JOB_CAPACITY; ++index) {
    if (capture_job_used[index]) continue;
    capture_job_used[index] = true;
    capture_jobs[index] = (struct capture_job) {0};
    return &capture_jobs[index];
  }
  return NULL;
}

/** @brief Return a job slot while the caller owns the pool mutex.
 * @param job Previously reserved slot; non-pool pointers are ignored.
 * @details Clear the job before marking the slot available so the next
 * owner cannot inherit request or camera state. */
static void capture_job_release_locked(struct capture_job *job)
{
  for (size_t index = 0; index < CAPTURE_JOB_CAPACITY; ++index) {
    if (job != &capture_jobs[index]) continue;
    capture_jobs[index] = (struct capture_job) {0};
    capture_job_used[index] = false;
    return;
  }
}

/** @brief Return a job slot from a worker that does not hold the pool lock.
 * @param job Previously reserved slot, no longer in use by the caller.
 * @details Take @c mut around pool bookkeeping; callers already holding
 * it must use capture_job_release_locked() instead. */
static void capture_job_release(struct capture_job *job)
{
  pthread_mutex_lock(&mut);
  capture_job_release_locked(job);
  pthread_mutex_unlock(&mut);
}

/** @brief Bridge survey decisions to current backend readiness from the serial event loop.
 * @param now_us Current MORA monotonic time in microseconds.
 * @details Readiness is a snapshot, not a reservation. Dispatch still checks driver
 * admission. Retain one popped result across a full UART TX queue rather than
 * discarding it merely because the wire was briefly busy.
 */
static void service_capture_arbiter(uint64_t now_us)
{
  const uint32_t session = capture_arbiter_session_id();
  if (!session) return;
  static uint64_t hello_us;
  if (!hello_us || now_us < hello_us || now_us - hello_us >= 500000) {
    union usher_capture_arbiter_hello_union hello = {.data = {USHER_CAPTURE_ARBITER_VERSION, session}};
    if (serial_tx_send(USHER_CAPTURE_ARBITER_HELLO, hello.bin, sizeof(hello.bin)) == 0) hello_us = now_us;
  }
  pthread_mutex_lock(&mut);
  uint32_t ready = !is_shooting && !waiting_capture && shooting_thread_count < MAX_PROCESSING_THREADS
                   ? USHER_CAMERA_MASK_CHDK | USHER_CAMERA_MASK_AICAM : 0;
  if (chdk_async_capture_enabled && !test_capture_enabled && !local_capture_only && !mock_map_enabled) {
    ready &= ~USHER_CAMERA_MASK_CHDK;
    if (!chdk_capture_in_progress && now_us && now_us >= chdk_retry_after_us) ready |= USHER_CAMERA_MASK_CHDK;
  }
  if (!lwir_capture_in_progress && !lwir_recovery_in_progress) ready |= USHER_CAMERA_MASK_LWIR;
  pthread_mutex_unlock(&mut);
  for (unsigned camera_id = 1; camera_id <= COVERAGE_CAMERAS; ++camera_id) {
    const struct camera_backend backend = camera_backend_get(camera_type_for_id(camera_id));
    const struct camera_capture_ops *ops = test_capture_enabled ? NULL : backend.capture_ops;
    /* Live CHDK publishes one image while the next is captured. */
    const bool chdk_pipelined = camera_id == USHER_CAMERA_CHDK && chdk_async_capture_enabled
                                && !test_capture_enabled && !local_capture_only && !mock_map_enabled;
    const unsigned limit = ops ? ops->max_in_flight : chdk_pipelined ? 2 : 1;
    if (!capture_arbiter_set_camera_capacity(camera_id, limit)
        || (ops && ops->has_capacity && !ops->has_capacity())) ready &= ~(1U << (camera_id - 1));
  }
  static union usher_capture_arbiter_result_union result;
  static bool result_pending;
  if (!result_pending) result_pending = capture_arbiter_result(&result, ready);
  if (result_pending
      && serial_tx_send(USHER_CAPTURE_ARBITER_RESULT, result.bin, sizeof(result.bin)) == 0) result_pending = false;
  union dc_shot_union shot;
  uint8_t mask;
  if (usher_running() && capture_arbiter_take(ready, now_us, &shot, &mask)) start_shoot_worker(&shot, mask, 0);
  if (!usher_running()) capture_arbiter_stop();
}

/** @brief Collect stream updates and completed point-session results.
 * @details Bound the work per backend to preserve UART service latency.
 * Stream updates and stopped-session results use separate caches; reject
 * session completions not matching the current stopped session. */
static void service_camera_backends(void)
{
  backend_service_pending = false;
  for (unsigned camera_id = 1; camera_id <= CAMERA_BACKEND_MAX_ID; ++camera_id) {
    const struct camera_point_stream_ops *stream = camera_owners[camera_id].point_stream;
    if (stream && stream->poll_result
        && (camera_stream_states[camera_id] == CAMERA_STREAM_ACTIVE
            || camera_stream_states[camera_id] == CAMERA_STREAM_DRAINED)) {
      struct camera_point_result update;
      unsigned received = 0;
      for (; received < MAX_BACKEND_RESULTS_PER_PASS && stream->poll_result(&update); ++received)
        send_msg_cam_point_update(camera_id, &update);
      if (received == MAX_BACKEND_RESULTS_PER_PASS) backend_service_pending = true;
    }
    const struct camera_session_ops *ops = camera_owners[camera_id].session;
    if (!ops || !ops->poll_result) continue;
    struct camera_point_result point;
    unsigned received = 0;
    for (; received < MAX_BACKEND_RESULTS_PER_PASS && ops->poll_result(&point); ++received) {
      if (point.session_id && point_sessions[camera_id - 1].id == point.session_id
          && point_sessions[camera_id - 1].stop_requested
          && point_sessions[camera_id - 1].result.data.session_id != point.session_id)
        send_msg_cam_point(camera_id, point.session_id, &point);
    }
    if (received == MAX_BACKEND_RESULTS_PER_PASS) backend_service_pending = true;
  }
}

/** @brief Ask active streams and sessions to drain, then service remaining results.
 * @details Mark streams drained before invoking callbacks. The service pass
 * still permits already-available results from drained streams. */
static void drain_camera_backends(void)
{
  for (unsigned camera_id = 1; camera_id <= CAMERA_BACKEND_MAX_ID; ++camera_id) {
    const struct camera_point_stream_ops *stream = camera_owners[camera_id].point_stream;
    if (stream && camera_stream_states[camera_id] == CAMERA_STREAM_ACTIVE) {
      camera_stream_states[camera_id] = CAMERA_STREAM_DRAINED;
      if (stream->drain) stream->drain();
    }
    const struct camera_session_ops *ops = camera_owners[camera_id].session;
    if (ops && ops->drain) ops->drain();
  }
  service_camera_backends();
}

/** @brief Start a point session for the addressed backend or backends.
 * @param camera_id Camera ID, or USHER_CAMERA_ALL for broadcast.
 * @param session_id Nonzero session identifier from the caller.
 * @details Treat a duplicate START as idempotent and clear old result state
 * only for a new ID. Prefer the configured point stream when present. */
static void handle_targeted_start(uint32_t camera_id, uint32_t session_id)
{
  if (camera_id > CAMERA_BACKEND_MAX_ID || !session_id) return;
  service_camera_backends();
  printf("USHER:\tSTART cam %u session %u\n", camera_id, session_id);
  for (size_t slot = 0; slot < CAMERA_BACKEND_MAX_ID; ++slot) {
    if (camera_id != USHER_CAMERA_ALL && slot != camera_id - 1) continue;
    if (point_sessions[slot].id == session_id) continue;
    point_sessions[slot].id = session_id;
    point_sessions[slot].result = (union usher_cam_point_result_union) {0};
    point_sessions[slot].pending = false;
    point_sessions[slot].stop_requested = false;
    if (!camera_owners[slot + 1].point_stream) {
      const struct camera_backend backend = camera_backend_get(camera_type_for_id(slot + 1));
      if (backend.point_stream) camera_register_stream(&backend);
    }
    const struct camera_point_stream_ops *stream = camera_owners[slot + 1].point_stream;
    if (stream) {
      if (camera_stream_states[slot + 1] == CAMERA_STREAM_ACTIVE && stream->start
          && (!stream->enabled || stream->enabled()) && !stream->start(session_id))
        fprintf(stderr, "USHER:\tphoto point stream not enabled\n");
      continue;
    }
    if (prepare_session_backend(slot + 1) && !camera_owners[slot + 1].session->start(session_id))
      fprintf(stderr, "USHER:\t%s session %u unavailable/full; stopped evidence retained\n",
              camera_owners[slot + 1].name, session_id);
  }
}

/** @brief Request the stop phase for a matching point session.
 * @param camera_id Camera ID, or USHER_CAMERA_ALL for broadcast.
 * @param session_id Nonzero session identifier.
 * @details Ignore a stale session. For a repeated STOP, resend a cached
 * result when available instead of rerunning the stop transition. */
static void handle_targeted_stop(uint32_t camera_id, uint32_t session_id)
{
  if (camera_id > CAMERA_BACKEND_MAX_ID || !session_id) return;
  if (camera_id == USHER_CAMERA_ALL) {
    for (uint32_t selected = 1; selected <= CAMERA_BACKEND_MAX_ID; ++selected) handle_targeted_stop(selected, session_id);
    return;
  }
  const size_t slot = camera_id - 1;
  if (point_sessions[slot].id != session_id) return;
  if (point_sessions[slot].result.data.session_id == session_id) {
    point_sessions[slot].pending = true;
    flush_cam_points();
    return;
  }
  if (point_sessions[slot].stop_requested) return;
  point_sessions[slot].stop_requested = true;
  printf("USHER:\tSTOP cam %u session %u\n", camera_id, session_id);
  const struct camera_point_stream_ops *stream = camera_owners[camera_id].point_stream;
  if (stream) {
    if (camera_stream_states[camera_id] == CAMERA_STREAM_ACTIVE && stream->stop
        && (!stream->enabled || stream->enabled()) && !stream->stop(session_id))
      fprintf(stderr, "USHER:\tphoto point return phase unavailable\n");
    return;
  }
  const struct camera_session_ops *ops = camera_owners[camera_id].session;
  if (!ops || !ops->stop || !ops->stop(session_id)) send_msg_cam_point(camera_id, session_id, NULL);
}

/** @brief Check whether a backend result is suitable for the point wire format.
 * @param point Nullable candidate point.
 * @return True when marked valid and within the coordinate, height, and quality bounds.
 * @details Reject the all-zero geographic placeholder; passing this check
 * does not establish survey accuracy. */
static bool camera_point_valid(const struct camera_point_result *point)
{
  return point && point->valid && point->lat >= -900000000 && point->lat <= 900000000
         && point->lon >= -1800000000 && point->lon <= 1800000000
         && (point->lat != 0 || point->lon != 0) && point->agl_mm >= 0
         && point->quality >= 0 && point->quality <= 1000;
}

/** @brief Cache and attempt to queue a terminal point-session result.
 * @param camera_id Concrete backend ID.
 * @param session_id Nonzero identifier for the session being completed.
 * @param point Backend result, or NULL for no valid point.
 * @details Keep the result after successful transmission so a repeated STOP
 * can recover it. Invalid points carry status but no coordinates. */
static void send_msg_cam_point(uint32_t camera_id, uint32_t session_id, const struct camera_point_result *point)
{
  if (camera_id < 1 || camera_id > CAMERA_BACKEND_MAX_ID || !session_id) return;
  union usher_cam_point_result_union result;
  memset(&result, 0, sizeof(result));
  result.data.camera_id = camera_id;
  result.data.session_id = session_id;
  const bool valid = camera_point_valid(point);
  result.data.status = valid ? USHER_CAM_POINT_VALID : USHER_CAM_POINT_INVALID;
  if (point && valid) {
    result.data.lat = point->lat;
    result.data.lon = point->lon;
    result.data.agl_mm = point->agl_mm;
    result.data.alt_mm = point->alt_mm;
    result.data.metric = point->metric;
    result.data.quality = point->quality;
  }

  point_sessions[camera_id - 1].result = result;
  point_sessions[camera_id - 1].pending = true;
  flush_cam_points();
}

/** @brief Keep only the newest stream result per camera across UART backpressure. */
static void send_msg_cam_point_update(uint32_t camera_id, const struct camera_point_result *point)
{
  if (!point || !point->session_id || camera_id < 1 || camera_id > CAMERA_BACKEND_MAX_ID) return;
  union usher_cam_point_update_union update = {.data = {
      .camera_id = camera_id, .frame_number = point->session_id,
      .status = camera_point_valid(point) ? USHER_CAM_POINT_VALID : USHER_CAM_POINT_INVALID
    }
  };
  if (update.data.status == USHER_CAM_POINT_VALID) {
    update.data.lat = point->lat; update.data.lon = point->lon;
    update.data.alt_mm = point->alt_mm; update.data.agl_mm = point->agl_mm;
    update.data.quality = point->quality; update.data.metric = point->metric;
  }
  point_updates[camera_id - 1] = update;
  point_update_pending[camera_id - 1] = true;
}

/** @brief Publish fresh stream mailboxes without changing stopped-session result caches. */
static void flush_cam_point_updates(void)
{
  for (size_t slot = 0; slot < CAMERA_BACKEND_MAX_ID; ++slot) {
    if (!point_update_pending[slot]) continue;
    if (serial_tx_send(USHER_CAM_POINT_UPDATE, point_updates[slot].bin, sizeof(point_updates[slot].bin)) != 0) return;
    point_update_pending[slot] = false;
  }
}

/** @brief Submit unsent point-session results to the UART transmit queue.
 * @details Clear only the pending flag on success. A full transmit queue
 * leaves the receipt pending for the next event-loop pass. */
static void flush_cam_points(void)
{
  for (size_t slot = 0; slot < CAMERA_BACKEND_MAX_ID; ++slot) {
    if (!point_sessions[slot].pending) continue;
    const union usher_cam_point_result_union *result = &point_sessions[slot].result;
    if (serial_tx_send(USHER_CAM_POINT_RESULT, result->bin, sizeof(result->bin)) != 0) return;
    point_sessions[slot].pending = false;
  }
}

/** @brief Launch optional mapping as a separate process with no inherited UART descriptors.
 * @details A map worker consumes saved originals, not capture slots. Failure to
 * start it is reported but never grounds the photo pipeline.
 */
static void start_pangea(void)
{
  char executable[PATH_MAX], photos[PATH_MAX], map_dir[PATH_MAX], pause_path[PATH_MAX], parent[32], since[32];
  ssize_t length = readlink("/proc/self/exe", executable, sizeof(executable) - 1);
  if (length <= 0 || (size_t)length >= sizeof(executable) - 1) {
    fprintf(stderr, "USHER:\tPANGEA path unavailable; capture continues\n");
    return;
  }
  executable[length] = '\0';
  char *base = strrchr(executable, '/');
  if (base == NULL || (size_t)(base - executable) + sizeof("/bin/pangea") > sizeof(executable)) return;
  snprintf(base, sizeof(executable) - (size_t)(base - executable), "/bin/pangea");
  const char *photo_root = USHER_LOCAL_PHOTO_DIR;
  if (pangea_camera_profile != NULL) {
    if (!camera.photo_directory) return;
    photo_root = camera.photo_directory;
  }
  const char *photo_directory = usher_resolve_path(photo_root, photos, sizeof(photos));
  const char *output_directory = usher_resolve_path(pangea_directory, map_dir, sizeof(map_dir));
  if (photo_directory == NULL || output_directory == NULL || output_directory[0] == '\0') return;
  snprintf(parent, sizeof(parent), "%ld", (long)getpid());
  snprintf(since, sizeof(since), "%lld", (long long)time(NULL));
  char *arguments[15] = {executable, "--input", (char *)photo_directory, "--output", (char *)output_directory,
                         "--watch", "--parent-pid", parent, "--since", since
                        };
  size_t argument_count = 10;
  if (pangea_camera_profile != NULL) {
    arguments[argument_count++] = "--camera-profile";
    arguments[argument_count++] = (char *)pangea_camera_profile;
  }
  if (pangea_pause_file != NULL) {
    const char *resolved_pause = usher_resolve_path(pangea_pause_file, pause_path, sizeof(pause_path));
    if (resolved_pause == NULL || resolved_pause[0] == '\0') return;
    arguments[argument_count++] = "--pause-file";
    arguments[argument_count++] = (char *)resolved_pause;
  }
  arguments[argument_count] = NULL;
  int result = posix_spawn(&pangea_pid, executable, NULL, NULL, arguments, environ);
  if (result != 0) {
    pangea_pid = -1;
    fprintf(stderr, "USHER:\tPANGEA unavailable: %s; capture continues\n", strerror(result));
  } else {
    printf("USHER:\tPANGEA background watcher pid %ld, map directory %s\n", (long)pangea_pid, output_directory);
  }
}

/** @brief Poll an owned subprocess for exit.
 * @param[in,out] child Positive child PID, reset to -1 when ownership ends.
 * @param timeout_ms Polling budget in milliseconds.
 * @return True if reaped or no longer a child; false on deadline or wait error.
 * @details Retain the PID on failure so later cleanup still knows which
 * child it owns. */
static bool wait_owned_child(pid_t *child, unsigned timeout_ms)
{
  const uint64_t started_us = monotonic_time_us();
  for (;;) {
    int status = 0;
    const pid_t result = waitpid(*child, &status, WNOHANG);
    if (result == *child || (result < 0 && errno == ECHILD)) { *child = -1; return true; }
    if (result < 0 && errno != EINTR) return false;
    const uint64_t now_us = monotonic_time_us();
    if (!started_us || !now_us || now_us < started_us
        || now_us - started_us >= (uint64_t)timeout_ms * 1000) return false;
    if (poll(NULL, 0, 10) < 0 && errno != EINTR) return false;
  }
}

/** @brief Bound optional map-worker shutdown rather than waiting for its input backlog. */
static void stop_pangea(void)
{
  if (pangea_pid <= 0 || wait_owned_child(&pangea_pid, 0)) return;
  kill(pangea_pid, SIGTERM);
  if (wait_owned_child(&pangea_pid, 100)) return;
  kill(pangea_pid, SIGKILL);
  if (!wait_owned_child(&pangea_pid, 1500))
    fprintf(stderr, "USHER:\tPANGEA has not exited after SIGKILL; retaining child ownership\n");
}

/** @brief Run saved-file analysis with fresh priority and idle catch-up.
 * @param unused Unused pthread argument.
 * @return NULL when stopped; pending filenames remain on disk, not in a durable job queue.
 * @details One lower-priority worker limits concurrent decoding. It never owns a
 * camera slot. An already running analysis is not preempted when a trigger arrives.
 */
static void *handle_soda_backlog(void *unused)
{
  (void)unused;
  if (setpriority(PRIO_PROCESS, 0, 10) != 0) {
    fprintf(stderr, "USHER:\tcannot lower SODA backlog priority: %s\n", strerror(errno));
  }
  for (;;) {
    pthread_mutex_lock(&mut);
    while (soda_backlog_count == 0 && !soda_backlog_stop && usher_running()) {
      pthread_cond_wait(&soda_available, &mut);
    }
    if (soda_backlog_stop || !usher_running()) {
      pthread_mutex_unlock(&mut);
      return NULL;
    }
    uint64_t now_us = monotonic_time_us();
    size_t selected = (soda_backlog_head + soda_backlog_count - 1) % SODA_BACKLOG_CAPACITY;
    bool fresh = soda_backlog[selected].attempts == 0 && now_us != 0
                 && now_us >= soda_backlog[selected].queued_us
                 && now_us - soda_backlog[selected].queued_us < (uint64_t)USHER_SODA_IDLE_MS * 1000;
    bool quiet = now_us != 0 && now_us >= last_photo_request_us
                 && now_us - last_photo_request_us >= (uint64_t)USHER_SODA_IDLE_MS * 1000;
    if (!fresh && quiet) {
      /* A retry in cooldown must not block other ready originals. Rotate at
       * most the current queue length, preserving bounded work even if all wait. */
      for (size_t scanned = 0; scanned < soda_backlog_count; ++scanned) {
        if (soda_backlog[soda_backlog_head].retry_after_us <= now_us) break;
        size_t tail = (soda_backlog_head + soda_backlog_count) % SODA_BACKLOG_CAPACITY;
        soda_backlog[tail] = soda_backlog[soda_backlog_head];
        soda_backlog_head = (soda_backlog_head + 1) % SODA_BACKLOG_CAPACITY;
      }
      selected = soda_backlog_head;
    }
    if ((!fresh && (!quiet || shooting_thread_count != 0 || lwir_capture_in_progress))
        || soda_backlog[selected].retry_after_us > now_us
        || pthread_mutex_trylock(&live_soda_mutex) != 0) {
      pthread_mutex_unlock(&mut);
      poll(NULL, 0, 50);
      continue;
    }
    struct soda_job job = soda_backlog[selected];
    if (!fresh) soda_backlog_head = (soda_backlog_head + 1) % SODA_BACKLOG_CAPACITY;
    --soda_backlog_count;
    soda_backlog_active = true;
    pthread_mutex_unlock(&mut);

    /* Stat identity catches ordinary replacement between enqueue and replay.
     * Do not apply old pose to a new file with a familiar name; this is not a
     * security boundary against concurrent hostile file modification. */
    struct stat source;
    bool same_source = lstat(job.filename, &source) == 0 && S_ISREG(source.st_mode)
                       && source.st_dev == job.source.st_dev && source.st_ino == job.source.st_ino
                       && source.st_size == job.source.st_size
                       && source.st_mtim.tv_sec == job.source.st_mtim.tv_sec
                       && source.st_mtim.tv_nsec == job.source.st_mtim.tv_nsec;
    uint64_t analysis_started_us = monotonic_time_us();
    if (same_source && !job.spotting_done && job.camera_id == USHER_CAMERA_LWIRCAM && lwir_spotter_enabled()) {
      const int spotting = lwir_spotter_analyze(job.filename, &job.shot, &job.timing, &keep_running);
      job.spotting_done = true;
      fprintf(stderr, "USHER-%d:\tLWIR spotting %s; original and capture timing retained\n",
              job.shot.data.nr, spotting == 0 ? "complete" : "failed");
      same_source = lstat(job.filename, &source) == 0 && S_ISREG(source.st_mode)
        && source.st_dev == job.source.st_dev && source.st_ino == job.source.st_ino
        && source.st_size == job.source.st_size && source.st_mtim.tv_sec == job.source.st_mtim.tv_sec
        && source.st_mtim.tv_nsec == job.source.st_mtim.tv_nsec;
    }
    int result = same_source ? run_soda(job.filename, &job.shot, job.camera_id) : -1;
    if (debug_enabled) {
      printf("USHER-%d TIMING:\tcamera=%d soda_ms=%.3f mode=%s result=%d\n",
             job.shot.data.nr, job.camera_id,
             (monotonic_time_us() - analysis_started_us) / 1000.0,
             fresh ? "fresh" : "idle", result);
    }
    if (!same_source) {
      fprintf(stderr, "USHER-%d:\tSODA backlog source missing/changed; old pose not applied: %s\n",
              job.shot.data.nr, job.filename);
    }
    pthread_mutex_unlock(&live_soda_mutex);
    pthread_mutex_lock(&mut);
    /* One retry allows transient output failures without a poisoned image
     * monopolizing the worker. Partial CSV/crop side effects are not exactly-once. */
    if (result != 0 && same_source && ++job.attempts < 2 && usher_running()
        && !soda_backlog_stop && soda_backlog_count < SODA_BACKLOG_CAPACITY) {
      job.retry_after_us = monotonic_time_us() + (uint64_t)USHER_SODA_RETRY_MS * 1000;
      size_t tail = (soda_backlog_head + soda_backlog_count) % SODA_BACKLOG_CAPACITY;
      soda_backlog[tail] = job;
      ++soda_backlog_count;
    } else if (result == 0) ++soda_backlog_completed;
    else ++soda_backlog_failed;
    soda_backlog_active = false;
    size_t pending = soda_backlog_count;
    pthread_cond_broadcast(&workers_finished);
    pthread_mutex_unlock(&mut);
    printf("USHER-%d:\tSODA backlog mode=%s result=%d pending=%zu, original retained: %s\n",
           job.shot.data.nr, fresh ? "fresh" : "idle", result, pending, job.filename);
  }
}

/** @brief Queue analysis of a saved original without retaining camera or pixel buffers.
 * @param filename Nonempty regular image file, unchanged while queued.
 * @param shoot Pose/number associated with that saved image, copied into the job.
 * @param camera_id Sensor-specific SODA route.
 * @details Overflow keeps existing queued work and leaves the new original on disk.
 * Worker-creation failure retains the job so a later submission can retry startup.
 */
static void defer_soda(const char *filename, const union dc_shot_union *shoot, int camera_id)
{
  defer_saved_analysis(filename, shoot, camera_id, NULL);
}

/** @brief Submit a published image to the bounded saved-analysis backlog.
 * @param filename Path of the nonempty original JPEG.
 * @param shoot Trigger evidence copied into the job.
 * @param camera_id Camera identity used to route the analysis.
 * @param timing Optional capture timing copied into the job.
 * @details Save file identity with the job so the worker can detect a
 * later replacement. If the backlog is unavailable, preserve the original
 * for offline processing rather than blocking capture. */
static void defer_saved_analysis(const char *filename, const union dc_shot_union *shoot, int camera_id,
                                  const struct capture_timing *timing)
{
  if (!filename || !filename[0] || !shoot) return;
  const struct camera_backend backend = camera_identity_get(camera_id);
  if (!backend.name || !backend.soda_application || !backend.soda_option) return;
  size_t length = strnlen(filename, MAX_FILENAME);
  struct stat source;
  bool source_valid = lstat(filename, &source) == 0 && S_ISREG(source.st_mode) && source.st_size > 0;
  pthread_mutex_lock(&mut);
  if (soda_backlog_stop || !usher_running() || !source_valid || length >= MAX_FILENAME
      || soda_backlog_count == SODA_BACKLOG_CAPACITY) {
    ++soda_backlog_overflow;
    pthread_mutex_unlock(&mut);
    fprintf(stderr,
            "USHER-%d:\tSODA backlog unavailable/full; original and flight EXIF retained for offline analysis: %s\n",
            shoot->data.nr, filename);
    return;
  }
  size_t tail = (soda_backlog_head + soda_backlog_count) % SODA_BACKLOG_CAPACITY;
  struct soda_job *job = &soda_backlog[tail];
  for (size_t index = 0; index <= length; ++index) job->filename[index] = filename[index];
  job->shot = *shoot;
  job->timing = timing ? *timing : (struct capture_timing) {0};
  job->spotting_done = false;
  job->camera_id = camera_id;
  job->source = source;
  job->queued_us = monotonic_time_us();
  job->retry_after_us = 0;
  job->attempts = 0;
  bool busy = soda_backlog_active || soda_backlog_count != 0;
  if (busy) ++soda_busy_deferred_count;
  size_t pending = ++soda_backlog_count;
  if (!soda_backlog_started) {
    int result = pthread_create(&soda_backlog_thread, NULL, handle_soda_backlog, NULL);
    soda_backlog_started = result == 0;
    if (result != 0) {
      fprintf(stderr, "USHER:\tSODA backlog worker unavailable: %s; photos continue\n", strerror(result));
    }
  }
  pthread_cond_signal(&soda_available);
  pthread_mutex_unlock(&mut);
  printf("USHER-%d:\tSODA %s pending=%zu: %s\n",
         shoot->data.nr, busy ? "deferred" : "submitted", pending, filename);
}

/** @brief Join the analysis worker without draining session-only pending jobs.
 * @pre Set keep_running to zero first so an active run_soda() follows its shutdown deadline.
 * @details Joining ensures analysis no longer touches state that camera cleanup
 * may release. Originals survive; pending analysis is not automatically persisted.
 */
static void stop_soda_backlog(void)
{
  pthread_mutex_lock(&mut);
  soda_backlog_stop = true;
  pthread_cond_broadcast(&soda_available);
  pthread_mutex_unlock(&mut);
  if (soda_backlog_started) pthread_join(soda_backlog_thread, NULL);
  pthread_mutex_lock(&mut);
  soda_backlog_started = false;
  fprintf(stderr, "USHER SODA:\tcompleted=%" PRIu64 " failed=%" PRIu64
          " pending=%zu unavailable/full=%" PRIu64 "; original photos retained\n",
          soda_backlog_completed, soda_backlog_failed, soda_backlog_count, soda_backlog_overflow);
  pthread_mutex_unlock(&mut);
}

/** @brief Run one analysis child directly with bounded runtime and termination grace.
 * @param filename Saved original to analyze, never deleted by this function.
 * @param shoot Immutable image-associated pose used to form arguments.
 * @param camera_id Sensor route selecting the SODA option.
 * @return Child exit status, or -1 for spawn/wait/cancellation/deadline failure.
 * @note Direct argv avoids shell interpretation. Successful exit means processing
 * completed, not that an object was detected or that each side effect is unique.
 */
static int run_soda(const char *filename, const union dc_shot_union *shoot, int camera_id)
{
  if (!filename || !filename[0] || !shoot) return -1;
  const struct camera_backend backend = camera_identity_get(camera_id);
  if (!backend.name) {
    fprintf(stderr, "USHER:\tinvalid SODA camera id %d\n", camera_id);
    return -1;
  }
  const char *soda_application = test_capture_enabled || local_capture_only ? USHER_SODA : backend.soda_application;
  const char *camera_option = backend.soda_option;
  if (!soda_application || !soda_application[0] || !camera_option) return -1;
  char values[10][16];
  const int32_t fields[10] = {
    shoot->data.nr, shoot->data.lat, shoot->data.lon, shoot->data.alt,
    shoot->data.phi, shoot->data.theta, shoot->data.psi, shoot->data.vground,
    shoot->data.course, shoot->data.groundalt
  };
  for (size_t index = 0; index < 10; index++) {
    int length = snprintf(values[index], sizeof(values[index]), "%d", fields[index]);
    if (length < 0 || (size_t)length >= sizeof(values[index])) {
      return -1;
    }
  }

  char *arguments[19] = {
    (char *)soda_application, (char *)filename,
    values[0], values[1], values[2], values[3], values[4],
    values[5], values[6], values[7], values[8], values[9], NULL, NULL, NULL
  };
  size_t argument_count = 12;
  if (camera_option != NULL) arguments[argument_count++] = (char *)camera_option;
  if (local_mode) arguments[argument_count++] = (char *)"--local";
  arguments[argument_count] = NULL;
  if (debug_enabled) {
    printf("USHER-%d DEBUG:\tstarting SODA directly: %s\n",
           shoot->data.nr, soda_application);
  }

  pid_t soda_pid;
  posix_spawnattr_t attributes;
  int spawn_result = posix_spawnattr_init(&attributes);
  if (spawn_result) return -1;
  sigset_t mask, defaults;
  sigemptyset(&mask);
  sigemptyset(&defaults);
  sigaddset(&defaults, SIGINT);
  sigaddset(&defaults, SIGTERM);
  spawn_result = posix_spawnattr_setsigmask(&attributes, &mask);
  if (!spawn_result) spawn_result = posix_spawnattr_setsigdefault(&attributes, &defaults);
  if (!spawn_result) spawn_result = posix_spawnattr_setpgroup(&attributes, 0);
  if (!spawn_result) spawn_result = posix_spawnattr_setflags(&attributes,
    POSIX_SPAWN_SETSIGMASK | POSIX_SPAWN_SETSIGDEF | POSIX_SPAWN_SETPGROUP);
  if (!spawn_result) spawn_result = posix_spawnp(&soda_pid, soda_application, NULL, &attributes,
                                                arguments, environ);
  posix_spawnattr_destroy(&attributes);
  if (spawn_result != 0) {
    fprintf(stderr, "USHER-%d:\tfailed to start SODA: %s\n",
            shoot->data.nr, strerror(spawn_result));
    return -1;
  }

  int status = 0;
  const uint64_t started_us = monotonic_time_us();
  uint64_t termination_us = 0;
  bool terminating = false;
  for (;;) {
    pid_t wait_result = waitpid(soda_pid, &status, WNOHANG);
    if (wait_result == soda_pid) {
      break;
    }
    if (wait_result < 0 && errno != EINTR) {
      return -1;
    }
    const uint64_t now_us = monotonic_time_us();
    if (!terminating && (!usher_running() || now_us == 0 || started_us == 0
                         || now_us - started_us >= (uint64_t)USHER_SODA_TIMEOUT_MS * 1000)) {
      fprintf(stderr, "USHER-%d:\tSODA %s; stopping analysis, original photo retained\n",
              shoot->data.nr, usher_running() ? "runtime deadline reached" : "shutdown");
            kill(-soda_pid, SIGTERM);
      kill(soda_pid, SIGTERM);
      termination_us = now_us;
      terminating = true;
    } else if (terminating && (now_us == 0
                               || now_us - termination_us >= (uint64_t)SODA_SHUTDOWN_GRACE_MS * 1000)) {
      kill(-soda_pid, SIGKILL);
      kill(soda_pid, SIGKILL);
    }
    sleep_microseconds(SODA_WAIT_INTERVAL_US);
  }
  return !terminating && WIFEXITED(status) ? WEXITSTATUS(status) : -1;
}

/** @brief End worker ownership after publication, distinct from releasing the capture ticket. */
static void release_worker_slot(void)
{
  pthread_mutex_lock(&mut);
  shooting_thread_count--;
  wake_event_loop();
  pthread_cond_broadcast(&workers_finished);
  pthread_mutex_unlock(&mut);
}

/** @brief Consume the newest buffered payload for a legacy image request.
 * @details This compatibility path consumes the entry before queueing
 * transmission; if queue admission fails, the entry is not retried. */
static inline void send_msg_image_buffer(void)
{
  // Check if image is available
  if (image_count > 0) {
    printf("USHER:\thandle_msg_buffer: Send %d\n", image_idx);
    // Send the image
    image_idx = (MAX_IMAGE_BUFFERS + image_idx - 1) % MAX_IMAGE_BUFFERS;
    image_count--;

    if (serial_tx_send(USHER_PAYLOAD, (const uint8_t *)image_buffer[image_idx], IMAGE_SIZE) != 0) {
      fprintf(stderr, "USHER:\timage UART send failed: %s\n", strerror(errno));
    }
  }
}

/** @brief Queue a snapshot of USHER's capture counters.
 * @details Copy shared counters under the mutex, then release it before
 * UART admission so capture workers need not wait on the transport path. */
static inline void send_msg_status(void)
{
  union usher_status_union status_msg = {0};

  pthread_mutex_lock(&mut);
  status_msg.data.cpu = 0;
  status_msg.data.threads = shooting_thread_count + (chdk_capture_in_progress ? 1 : 0);
  status_msg.data.shots = shooting_count;
  status_msg.data.extra = 0;
  pthread_mutex_unlock(&mut);

  if (serial_tx_send(USHER_STATUS, status_msg.bin, sizeof(status_msg.bin)) != 0) {
    fprintf(stderr, "USHER:\tstatus UART send failed: %s\n", strerror(errno));
  }
}

/** @brief Print the invocation modes and build-specific defaults.
 * @param program Executable spelling to use in examples.
 * @details Report compiled endpoint values rather than assuming the same
 * defaults for native and board builds. */
static void print_usage(const char *program)
{
  puts(USHER_BUILD_VERSION);
    printf("Usage: %s [--serial DEVICE | --local] [--chdk | --aicam | --spottercam | --lwircam | --loomcam] [--earcam] [--test] [OPTIONS]\n",
      program);
  printf("       %s (--chdk | --aicam | --spottercam | --lwircam) --speedtest\n", program);
  printf("  --serial DEVICE   serial endpoint (default: %s)\n", USHER_SERIAL_DEVICE);
  printf("  --local           create local serial bridge %s <-> %s\n",
         USHER_LOCAL_SIM_DEVICE, USHER_LOCAL_APP_DEVICE);
  printf("  --chdk            use the CHDK camera backend\n");
  printf("  --aicam           use system-installed rpicam-still (plain capture, no detection)\n");
  printf("  --spottercam      use native IMX500 vehicle detection (camera id 2; default outside local mode)\n");
  printf("  --native-photo    use native libcamera photos without inference with --aicam or --spottercam\n");
  printf("  --lwir-spotter-settings FILE  analyze saved LWIR JPEGs with a separate ONNX model\n");
  printf("  --lwir-spotter-profile FILE   measured native 256x192 LWIR PANGEA camera profile\n");
  printf("  --lwir-spotter-limits US,M    maximum exposure interval and pose uncertainty (5000,1)\n");
  printf("  --spottercam-settings FILE select detector model/backend settings\n");
  printf("  --spottercam-targets FILE  project detected vehicles onto UART target slots 0..4\n");
  printf("  --loomcam-settings FILE select LOOM model/backend settings\n");
  printf("  --earcam-settings FILE select acoustic model/backend settings\n");
  printf("  --loomcam         enable LOOM_cam visual-return framework on AIcam (camera id 2)\n");
  printf("  --loomcam-model FILE  IMX500 RPK uploaded by loomcam (default: %s)\n", USHER_LOOMCAM_MODEL);
  printf("  --loomcam-reference-dir DIR  outbound references, same filesystem as photos (default: %s)\n",
         USHER_LOOMCAM_REFERENCE_DIR);
  printf("  --lwircam         use the Tiny 1-C LWIR camera backend\n");
  printf("  --lwir-config FILE load LWIR rendering/correction policy\n");
  printf("  --earcam          also run the acoustic earcam backend (camera id 4)\n");
  printf("  --diplacam-config FILE  enable native file-backed XY planning (camera id 5; no photographs)\n");
  printf("  --earcam-device PCM  enable real EARcam with this ALSA input (e.g. default or plughw:CARD=Device,DEV=0)\n");
  printf("  --earcam-sim LAT,LON[,DB]  earcam backend with a virtual loudspeaker instead of a microphone\n");
  printf("  --earcam-band LOW,HIGH  tone search band in Hz (default earcam 1800,3200; motionSCOUT K-T-R: 2400,3200)\n");
  printf("  --test            process a mock image for any camera backend\n");
  printf("                    randomly selects testphotos/*.jpg beside this executable\n");
  printf("  --mocktransform   transform test image using shot roll, pitch, and yaw\n");
  printf("  --debug           show serial, USHER frame, trigger, and capture diagnostics\n");
  printf("  --mock-image FILE image used by local or test capture (default: %s)\n", USHER_MOCK_IMAGE);
  printf("  --mock-map        MIMIC: Map-tile Imagerie Modeling in Interactive Camera-sim\n");
  printf("                    local map-based mock captures (implies --test; requires --local)\n");
  printf("  --mock-map-cache DIR  pprzgcs XYZ cache root (legacy tile path: GOOGLE/18)\n");
  printf("  --mock-map-fov H,V    horizontal,vertical FOV degrees (default model: 60,45; fixed nadir mount)\n");
  printf("  --mock-lens FILE      AIcam/LOOM saved-photo lens profile, before neural inference; requires --local --mock-map\n");
  printf("  --pangea DIR           optional idle photo-map worker; local MIMIC or real camera profile\n");
  printf("  --pangea-camera FILE   real JPEG mapping geometry; no test images or simulated effects\n");
  printf("  --pangea-pause-file FILE  defer map work while FILE exists; capture/detection unchanged\n");
  printf("  --survey-camera FILE    calibrated PANGEA_CAMERA_V1 key=value profile; repeat per camera\n");
  printf("  --survey-enforce        enable conservative redundant-photo suppression and bounded lookahead\n");
  printf("  --survey-overlap RATIO  minimum overlap for predictive delay (0..0.9, default 0.3)\n");
  printf("  --pose-log DIR    record flight pose samples as CSV in DIR (default: off)\n");
  printf("  --clock-align     bound FC-to-MORA time offset (automatic for live cameras)\n");
  printf("  --no-clock-align  disable clock probes and negotiated pose streaming\n");
  printf("  --lwir-latency-ms MIN,MAX  measured exposure-to-callback bounds, not JPEG latency\n");
  printf("  --chdk-latency-ms MIN,MAX measured command-to-exposure bounds\n");
  printf("  --lwir-calibration FILE  camera YAML used to turn LWIR hotspots into coordinates\n");
  printf("  --lwir-raw        keep the full sensor frame as photos/lNNNNNN.jpg.raw instead of\n");
  printf("                    storing the temperatures inside the JPEG\n");
  printf("  --lwir-motion-compensation  advance LWIR GPS over the measured capture delay\n");
  printf("  --speedtest       capture one warm-up plus %d timed photos, report photos/s and time/photo, and exit\n",
         CAMERA_SPEEDTEST_SAMPLE_COUNT);
  printf("  --help            show this help\n");
  printf("  --version         show application version and build Git revision\n");
}

/** @brief Return an immutable capability description without selecting or opening a camera.
 * @param type Resolved runtime backend type.
 * @return Descriptor with static hook lifetimes, or a zero descriptor for an invalid type.
 */
static struct camera_backend camera_backend_get(enum camera_backend_type type)
{
  struct camera_backend backend = camera_backend_registry_get(type, test_capture_enabled || mock_map_enabled);
  if (type == CAMERA_BACKEND_LOCAL) backend.shoot = local_backend_shoot;
  return backend;
}

/** @brief Resolve metadata and hooks for a concrete protocol camera identity.
 * @param camera_id Protocol camera ID.
 * @return Registry entry for the current routing mode.
 * @details Keep identity lookup separate from selecting the global backend
 * so a second sensor's result does not change capture configuration. */
static struct camera_backend camera_identity_get(int camera_id)
{
  return camera_backend_registry_identity(camera_id, configured_camera_backend,
                                          test_capture_enabled || mock_map_enabled);
}

/** @brief Install the startup backend, optionally with local test hooks.
 * @param type Requested registry backend.
 * @param test_mode True to replace hardware capture with simulation.
 * @return Zero on selection, or -1 if the registry has no such backend.
 * @details Preserve the selected camera identity in test mode while removing
 * native capture operations so a mock job cannot acquire the hardware. */
static int camera_backend_select(enum camera_backend_type type, bool test_mode)
{
  struct camera_backend selected = camera_backend_registry_get(type, test_mode || mock_map_enabled);
  if (type == CAMERA_BACKEND_LOCAL) selected.shoot = local_backend_shoot;
  if (!selected.name) {
    fprintf(stderr, "USHER:\tinvalid camera backend\n");
    return -1;
  }
  camera = selected;

  if (test_mode) {
    camera.init = mock_map_enabled ? local_pipe_map_init : local_pipe_test_init;
    camera.shoot = local_backend_shoot;
    camera.deinit = local_pipe_deinit;
    camera.soda_application = USHER_SODA;
    camera.capture_ops = NULL;
    camera.capabilities |= CAMERA_CAP_MOCK;
  }
  return 0;
}

/** @brief Capture a local fixture under the selected optical camera identity.
 * @param[out] filename Output path; use only on success.
 * @param filename_size Capacity including the terminator.
 * @param image_number Photo number.
 * @return Zero on capture, or -1 on invalid identity or capture failure.
 * @details Supply the concrete camera identity so the fixture uses the
 * appropriate prefix while remaining distinct from a physical capture. */
static int local_backend_shoot(char *filename, size_t filename_size, int image_number)
{
  const struct camera_backend identity = camera_identity_get(optical_camera_id);
  if (optical_camera_id < USHER_CAMERA_CHDK || optical_camera_id > USHER_CAMERA_EARCAM) {
    return -1;
  }
  return local_pipe_shoot(filename, filename_size, image_number, &identity);
}

/** @brief Reserve the shared local serial endpoint namespace.
 * @return Zero on exclusive reservation, or -1 when it cannot be acquired.
 * @details Keep the advisory lock descriptor open until teardown to stop
 * another local instance from replacing the active endpoint links. */
static int lock_local_instance(void)
{
  local_lock_fd = open(USHER_LOCAL_LOCK_FILE, O_RDWR | O_CREAT | O_CLOEXEC, 0644);
  if (local_lock_fd < 0) {
    fprintf(stderr, "USHER:\tfailed to open local instance lock %s: %s\n",
            USHER_LOCAL_LOCK_FILE, strerror(errno));
    return -1;
  }
  struct flock lock = {
    .l_type = F_WRLCK,
    .l_whence = SEEK_SET,
    .l_start = 0,
    .l_len = 0
  };
  if (fcntl(local_lock_fd, F_SETLK, &lock) != 0) {
    fprintf(stderr, "USHER:\tlocal mode is already running; stop it before starting another instance\n");
    close(local_lock_fd);
    local_lock_fd = -1;
    return -1;
  }
  return 0;
}

/** @brief Launch the PTY bridge for a hardware-free session.
 * @return Owned child PID once both links appear, or -1 on failure.
 * @details Call with the local instance lock held. Remove stale links
 * before launch and clean up the child and links if startup fails. */
static pid_t start_local_serial_bridge(void)
{
  unlink(USHER_LOCAL_SIM_DEVICE);
  unlink(USHER_LOCAL_APP_DEVICE);

  pid_t child = fork();
  if (child < 0) {
    perror("USHER: failed to fork socat");
    return -1;
  }
  if (child == 0) {
    char sim_endpoint[256];
    char app_endpoint[256];
    snprintf(sim_endpoint, sizeof(sim_endpoint), "pty,raw,echo=0,link=%s", USHER_LOCAL_SIM_DEVICE);
    snprintf(app_endpoint, sizeof(app_endpoint), "pty,raw,echo=0,link=%s", USHER_LOCAL_APP_DEVICE);
    execlp("socat", "socat", sim_endpoint, app_endpoint, (char *)NULL);
    perror("USHER: failed to start socat");
    _exit(127);
  }

  if (wait_for_path(USHER_LOCAL_SIM_DEVICE) != 0 || wait_for_path(USHER_LOCAL_APP_DEVICE) != 0) {
    fprintf(stderr, "USHER:\tsocat did not create the local serial endpoints\n");
    kill(child, SIGTERM);
    waitpid(child, NULL, 0);
    unlink(USHER_LOCAL_SIM_DEVICE);
    unlink(USHER_LOCAL_APP_DEVICE);
    return -1;
  }
  return child;
}

/** @brief Give the new serial bridge time to publish one endpoint.
 * @param path Endpoint path to check.
 * @return Zero if found during polling, or -1 if it never appears.
 * @details Poll with a fixed attempt limit so failure to create a link
 * is detected rather than waiting indefinitely for readiness. */
static int wait_for_path(const char *path)
{
  for (int attempt = 0; attempt < 100; attempt++) {
    if (access(path, F_OK) == 0) {
      return 0;
    }
    sleep_microseconds(10000);
  }
  return -1;
}

/** @brief Stop the owned simulator bridge and release the local lock.
 * @details Preserve child and endpoint ownership if termination cannot
 * be confirmed. Links owned elsewhere are not removed. */
static void stop_local_serial_bridge(void)
{
  if (local_bridge_owned && socat_pid > 0) {
    kill(socat_pid, SIGTERM);
    if (!wait_owned_child(&socat_pid, 100)) {
      kill(socat_pid, SIGKILL);
      if (!wait_owned_child(&socat_pid, 1500)) {
        fprintf(stderr, "USHER:\tlocal serial bridge has not exited; retaining child ownership\n");
        return;
      }
    }
  }
  if (local_bridge_owned) {
    unlink(USHER_LOCAL_SIM_DEVICE);
    unlink(USHER_LOCAL_APP_DEVICE);
    local_bridge_owned = false;
  }
  if (local_lock_fd >= 0) {
    close(local_lock_fd);
    local_lock_fd = -1;
  }
}

/** @brief Request service shutdown from a signal handler.
 * @param signal_number Signal delivered by the OS; all installed cases request shutdown.
 * @details Notify the event loop rather than closing camera or UART state in
 * the handler. Owners perform teardown in ordinary execution context. */
static void handle_signal(int signal_number)
{
  (void)signal_number;
  capture_set_running(&keep_running, false);
  wake_event_loop();
}
