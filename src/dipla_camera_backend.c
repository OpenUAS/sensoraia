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

/** @file dipla_camera_backend.c
 * @brief Nonblocking USHER-to-Diplacam process admission and reply supervision.
 * @details Only the native worker owns field truth, GP state and inference.
 * This adapter keeps the UART owner responsive by sending bounded local packets
 * and publishing correlated completed replies, never waiting for a new plan. */
#include "dipla_camera_backend.h"
#include "../modules/dipla_cam/dipla_ipc.h"
#include <errno.h>
#include <poll.h>
#include <spawn.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

extern char **environ; /**< Inherited environment; executable/configuration remain explicit argv. */
enum {
  configuration_path_capacity = 512,
  worker_descriptor = 198,
  shutdown_term_grace_ms = 100,
  shutdown_kill_grace_ms = 1000,
  shutdown_poll_ms = 10,
  reply_batch_limit = 16,
  computational_validity_quality = 1000,
  milliseconds_per_second = 1000,
  nanoseconds_per_millisecond = 1000000
};
static char config_path[configuration_path_capacity]; /**< Owned configuration path, immutable while a worker is acquired. */
static int descriptor = -1; /**< One private packet socket; minus one means no process channel. */
static pid_t child = -1; /**< Sole child reaped by this owner, not by unrelated camera backends. */
static uint32_t active_session, pending_number; /**< Distinct sortie and observation correlation tokens. */
static uint32_t admitted_number, assimilated_number; /**< One unassimilated measurement credit, separate from point retrieval. */
static struct dipla_reply pending_reply;
static bool reply_received;
static uint64_t pending_since; /**< Monotonic admission time, not a worker/FC wall-clock timestamp. */
static bool survey_lost; /**< Worker loss requires STOP before another survey can establish a geographic frame. */

/** @brief Read elapsed-time evidence unaffected by changes to civil time.
 * @return Monotonic milliseconds, or zero when a valid deadline cannot be formed. */
static uint64_t milliseconds(void)
{
  struct timespec now;
  if (clock_gettime(CLOCK_MONOTONIC, &now) != 0) return 0;
  return (uint64_t)now.tv_sec * milliseconds_per_second + (uint64_t)now.tv_nsec / nanoseconds_per_millisecond;
}

bool dipla_camera_backend_setup(const char *configuration)
{
  const size_t length = configuration ? strnlen(configuration, sizeof(config_path)) : 0;
  if (!length || length >= sizeof(config_path) || descriptor >= 0 || child > 0) return false;
  for (size_t index = 0; index <= length; ++index) config_path[index] = configuration[index];
  return true;
}

/** @brief Report explicit opt-in independently of process or model readiness. */
static bool enabled(void) { return config_path[0] != '\0'; }
/** @brief Expose only completion readiness to USHER's existing event poll. */
static int completion_fd(void) { return descriptor; }
/** @brief Admit one whole packet without blocking the UART owner.
 * @param request Immutable stack-owned packet; no pointer survives this call.
 * @return False on backpressure, a missing channel or an incomplete send.
 * @details SOCK_SEQPACKET preserves boundaries, so a failed packet is not retried
 * as an unsafely reconstructed suffix. The caller keeps session ownership. */
static bool transmit(const struct dipla_request *request)
{
  if (descriptor < 0) return false;
  ssize_t sent;
  do { sent = send(descriptor, request, sizeof(*request), MSG_DONTWAIT | MSG_NOSIGNAL); } while (sent < 0 && errno == EINTR);
  return sent == sizeof(*request);
}

/** @brief Invalidate requests before escalating worker shutdown and reaping it.
 * @details Closing the channel prevents a late completion from reaching a new
 * session. The polling grace permits ordinary teardown; SIGKILL contains a
 * worker that does not respond. This is supervision, not a real-time guarantee. */
static bool reap_child(unsigned timeout_ms)
{
  if (child <= 0) return true;
  const uint64_t started = milliseconds();
  for (;;) {
    int status = 0;
    const pid_t result = waitpid(child, &status, WNOHANG);
    if (result == child || (result < 0 && errno == ECHILD)) { child = -1; return true; }
    if (result < 0 && errno != EINTR) return false;
    const uint64_t now = milliseconds();
    if (!started || !now || now < started || now - started >= timeout_ms) return false;
    if (poll(NULL, 0, shutdown_poll_ms) < 0 && errno != EINTR) return false;
  }
}

/** @brief Close the planner channel, reset session state, and stop the child.
 * @details Retain ownership if the child cannot be reaped even after
 * escalation, so another planner cannot be launched over the old one. */
static void deinit(void)
{
  if (descriptor >= 0) close(descriptor);
  descriptor = -1;
  active_session = pending_number = 0;
  admitted_number = assimilated_number = 0;
  reply_received = false;
  survey_lost = false;
  if (child > 0 && !reap_child(0)) {
    kill(child, SIGTERM);
    if (reap_child(shutdown_term_grace_ms)) return;
    kill(child, SIGKILL);
    if (!reap_child(shutdown_kill_grace_ms))
      fprintf(stderr, "USHER:\tdiplacam has not exited after SIGKILL; retaining child ownership\n");
  }
}

/** @brief Spawn one native planner with a private, close-on-exec packet socket.
 * @return Zero for an existing/started channel or a disabled backend; minus one on failure.
 * @details The deliberately reserved child descriptor makes argv independent of
 * the parent's transient descriptor numbers. Duplication clears CLOEXEC only
 * for the endpoint the worker must retain; the parent's other endpoint is closed. */
static int initialize(void)
{
  if (!enabled()) return 0;
  if (survey_lost) return -1;
  if (descriptor >= 0) return 0;
  if (!reap_child(0)) return -1;
  int channel[2];
  if (socketpair(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC, 0, channel) != 0) return -1;
  posix_spawn_file_actions_t actions;
  int result = posix_spawn_file_actions_init(&actions);
  if (result) { close(channel[0]); close(channel[1]); return -1; }
  const int worker_fd = worker_descriptor;
  result = posix_spawn_file_actions_adddup2(&actions, channel[1], worker_fd);
  if (!result && channel[0] != worker_fd) result = posix_spawn_file_actions_addclose(&actions, channel[0]);
  if (!result && channel[1] != worker_fd) result = posix_spawn_file_actions_addclose(&actions, channel[1]);
  char application[] = "./bin/diplacam", config_option[] = "--config", fd_option[] = "--ipc-fd";
  char fd_text[sizeof("-2147483648")];
  const int written = snprintf(fd_text, sizeof(fd_text), "%d", worker_fd);
  if (written < 0 || (size_t)written >= sizeof(fd_text)) result = EINVAL;
  char *arguments[] = {application, config_option, config_path, fd_option, fd_text, NULL};
  if (!result) result = posix_spawn(&child, application, &actions, NULL, arguments, environ);
  posix_spawn_file_actions_destroy(&actions);
  close(channel[1]);
  if (result) {
    close(channel[0]); child = -1;
    fprintf(stderr, "USHER:\tdiplacam start failed: %s\n", strerror(result));
    return -1;
  }
  descriptor = channel[0];
  return 0;
}

/** @brief Admit a nonzero session and discard correlation from the previous one.
 * @param session Owner-selected session token, not a photo number.
 * @return True for packet admission; it does not mean a plan is ready. */
static bool start(uint32_t session)
{
  if (!session || !enabled()) return false;
  if (survey_lost) { active_session = session; return false; }
  if (initialize() != 0) return false;
  const struct dipla_request request = {.version = DIPLA_IPC_VERSION, .operation = DIPLA_IPC_START, .session = session};
  if (!transmit(&request)) return false;
  active_session = session;
  pending_number = 0;
  admitted_number = assimilated_number = 0;
  reply_received = false;
  return true;
}

/** @brief End only the matching session, even if its stop packet cannot be sent.
 * @param session Token returned to the owner at start admission.
 * @return Whether the stop packet was admitted, or a lost survey was ended locally.
 * @details Local invalidation is unconditional after the match so a lost stop
 * cannot leave old replies eligible for a later sortie. */
static bool stop(uint32_t session)
{
  if (!session || session != active_session) return false;
  if (survey_lost) { survey_lost = false; active_session = pending_number = 0; return true; }
  const struct dipla_request request = {.version = DIPLA_IPC_VERSION, .operation = DIPLA_IPC_STOP, .session = session};
  const bool sent = transmit(&request);
  deinit();
  return sent;
}

/** @brief Retrieve a point and admit a new observation only when its credit is free.
 * @param shot Immutable FC request; groundalt carries Q8 height above ground.
 * @return Zero on packet admission, minus one while busy/disabled or on invalid input.
 * @details One unassimilated measurement credit bounds evidence admission in
 * addition to point-reply correlation. While it is occupied, point-only requests
 * can read the retained route without admitting another measurement.
 * A successful send does not imply ONNX finished or a navigation point exists. */
static int record(const union dc_shot_union *shot)
{
  const uint64_t now = milliseconds();
  if (!shot || shot->data.nr <= 0 || !active_session || survey_lost || !now) return -1;
  if (pending_number && now - pending_since <= DIPLA_CYCLE_TIMEOUT_MS) return -1;
  const bool observation_busy = admitted_number != assimilated_number;
  const struct dipla_request request = {
    .version = DIPLA_IPC_VERSION,
    .operation = observation_busy ? DIPLA_IPC_POINT : DIPLA_IPC_CREDIT_SHOT, .session = active_session,
    .number = (uint32_t)shot->data.nr, .lat = shot->data.lat, .lon = shot->data.lon,
    .alt_mm = shot->data.alt, .agl_q8 = shot->data.groundalt
  };
  if (!transmit(&request)) return -1;
  if (!observation_busy && request.number > admitted_number) admitted_number = request.number;
  pending_number = request.number;
  pending_since = now;
  reply_received = false;
  return 0;
}

/** @brief Drain a bounded reply batch and publish only the active request's result.
 * @param point Destination changed only for a complete correlated reply.
 * @return True for a completed valid or invalid point, false for no eligible reply.
 * @details MSG_TRUNC detects oversized packets instead of accepting their prefix.
 * The sixteen-packet work limit prevents stale traffic monopolizing the event
 * loop. Credit and point packets may arrive in either order. Point completion
 * does not wait for assimilation, and credits never become FC camera results. Quality 1000 denotes
 * a valid computation, not physical accuracy. */
static bool poll_result(struct camera_point_result *point)
{
  if (!point || descriptor < 0) return false;
  for (unsigned attempt = 0; attempt < reply_batch_limit; ++attempt) {
    struct dipla_reply reply;
    const ssize_t count = recv(descriptor, &reply, sizeof(reply), MSG_DONTWAIT | MSG_TRUNC);
    if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) break;
    if (count < 0 && errno == EINTR) continue;
    if (count != sizeof(reply) || reply.version != DIPLA_IPC_VERSION || reply.valid > 1) {
      const uint32_t lost_session = active_session;
      deinit();
      if (lost_session) {
        active_session = lost_session;
        survey_lost = true;
        fprintf(stderr, "USHER:\tdiplacam survey lost; STOP required before a new anchor\n");
      }
      return false;
    }
    if (reply.session == active_session && reply.number == admitted_number && !reply.valid &&
        reply.metric == DIPLA_IPC_ASSIMILATED_METRIC) {
      assimilated_number = reply.number;
      continue;
    }
    if (reply.session != active_session || reply.number != pending_number) continue;
    pending_reply = reply;
    reply_received = true;
  }
  if (!reply_received) return false;
  const struct dipla_reply reply = pending_reply;
  const uint64_t now = milliseconds();
  const bool valid = reply.valid && now && now - pending_since <= DIPLA_CYCLE_TIMEOUT_MS && reply.metric >= (int32_t)DIPLA_WAYPOINT_COUNT &&
    reply.lat >= -DIPLA_LATITUDE_LIMIT && reply.lat <= DIPLA_LATITUDE_LIMIT &&
    reply.lon >= -DIPLA_LONGITUDE_LIMIT && reply.lon <= DIPLA_LONGITUDE_LIMIT &&
    reply.agl_mm >= DIPLA_FLIGHT_HEIGHT_MIN_MM && reply.agl_mm <= DIPLA_FLIGHT_HEIGHT_MAX_MM;
  pending_number = 0;
  reply_received = false;
  *point = (struct camera_point_result) {
    .session_id = reply.number, .valid = valid,
    .lat = reply.lat, .lon = reply.lon, .alt_mm = reply.alt_mm, .agl_mm = reply.agl_mm,
    .quality = valid ? computational_validity_quality : 0, .metric = reply.metric
  };
  return true;
}

/** @brief Stop the selected session and reap the worker during owner shutdown. */
static void drain(void) { if (active_session) stop(active_session); deinit(); }
/** @brief Acquire the worker without opening any optical camera.
 * @param source Unused image-source argument from the common backend interface.
 * @return Result of local worker acquisition. */
static int init(const char *source) { (void)source; return initialize(); }

struct camera_backend dipla_camera_backend_get(void)
{
  static const struct camera_point_stream_ops stream = {
    .init = initialize, .poll_result = poll_result, .drain = drain,
    .start = start, .stop = stop, .enabled = enabled, .completion_fd = completion_fd
  };
  return (struct camera_backend) {
    .name = "diplacam", .camera_id = DIPLA_CAMERA_ID, .capabilities = CAMERA_CAP_POINT_STREAM,
    .record = record, .init = init, .deinit = deinit, .point_stream = &stream
  };
}
