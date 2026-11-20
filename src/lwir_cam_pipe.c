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

#include "lwir_cam_pipe.h"
#include "camera_capture.h"
#include "../modules/lwir_cam/lwir_features.h"
#include "path_utils.h"

/**
 * @file lwir_cam_pipe.c
 * @brief USHER-side supervisor for the persistent LWIR capture-server process.
 * @details The server holds the Tiny1-C stream open, avoiding per-shot sensor warmup.
 * This parent owns its pipes and child lifecycle, validates line-protocol replies and
 * capture timing. All failure paths clear state before returning so USHER's supervisor
 * can schedule a later retry without reusing a stale descriptor.
 */

#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <math.h>
#include <poll.h>
#include <signal.h>
#include <spawn.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

#include "camera_backend.h"

#ifndef USHER_LWIR_CAM_PHOTO_DIR
  #define USHER_LWIR_CAM_PHOTO_DIR "~/Pictures/sensoraia"
#endif

#ifndef USHER_LWIR_CAM_COMMAND
  #define USHER_LWIR_CAM_COMMAND "./bin/lwircam"
#endif

#ifndef CHILD_STOP_GRACE_MS
  #define CHILD_STOP_GRACE_MS 500
#endif
#define CHILD_WAIT_INTERVAL_MS 10
#define IMAGE_PROCESS_TIMEOUT_MS 30000
#ifndef LWIR_SERVER_START_TIMEOUT_MS
  #define LWIR_SERVER_START_TIMEOUT_MS 30000
#endif
#define LWIR_WARMUP_TIMEOUT_MS 30000
#ifndef LWIR_REQUEST_TIMEOUT_MS
  #define LWIR_REQUEST_TIMEOUT_MS 8000
#endif

extern char **environ;

static pid_t capture_server_pid = -1;
static int capture_server_input = -1;
static int capture_server_output = -1;
/** Buffered reassembly for read_server_status(): avoids one poll()+read() syscall
 * pair per byte, which otherwise adds up to hundreds of syscalls per response line
 * set on every single shot. */
static char status_buffer[512];
static size_t status_buffer_size, status_buffer_pos;
static double capture_delay_s = -1;
static struct capture_timing capture_times;
static bool native_raw_enabled;
static const char *calibration_path;
static const char *photo_directory = USHER_LWIR_CAM_PHOTO_DIR;
static const char *capture_command = USHER_LWIR_CAM_COMMAND;
static const char *lwir_config;
static const volatile sig_atomic_t *running;

void lwir_cam_pipe_configure(const struct camera_backend_host *host)
{
  running = host ? host->running : NULL;
}

int lwir_cam_pipe_set_config(const char *path)
{
  if (!path || !*path || capture_server_pid >= 0) return -1;
  lwir_config = path;
  return 0;
}

/** @brief Read the capture-server watchdog clock.
 * @return Monotonic milliseconds, or zero on failure.
 * @details Keep supervision independent of system time corrections. */
static uint64_t monotonic_ms(void)
{
  struct timespec now;
  return clock_gettime(CLOCK_MONOTONIC, &now) == 0
         ? (uint64_t)now.tv_sec * 1000U + (uint64_t)now.tv_nsec / 1000000U : 0;
}

/** @brief Wait up to a bounded time for the capture child to exit.
 * @param pid Owned positive child PID.
 * @param[out] status Wait status, set if this call reaps the child.
 * @param timeout_ms Nonnegative budget; zero checks without sleeping.
 * @return One if reaped or no longer a child, zero on timeout, or -1 on error.
 * @details Use nonblocking waits so a child stuck in a device call does not
 * indefinitely block its supervisor. */
static int wait_child(pid_t pid, int *status, int timeout_ms)
{
  const uint64_t started_ms = monotonic_ms();
  if (started_ms == 0) return -1;
  for (;;) {
    pid_t result = waitpid(pid, status, WNOHANG);
    if (result == pid || (result < 0 && errno == ECHILD)) return 1;
    if (result < 0 && errno != EINTR) return -1;
    uint64_t now_ms = monotonic_ms();
    if (now_ms == 0 || now_ms - started_ms >= (uint64_t)timeout_ms) return 0;
    struct timespec pause = {.tv_sec = 0, .tv_nsec = CHILD_WAIT_INTERVAL_MS * 1000000L};
    while (nanosleep(&pause, &pause) != 0 && errno == EINTR) {}
  }
}

/** @brief Stop an owned capture child, escalating only after its grace period.
 * @param pid Owned positive child PID.
 * @param[out] status Wait status, set if the child is reaped here.
 * @return One if ownership ended, zero if still waiting after escalation, or -1 on error.
 * @details Reap an already exited child first. Give normal teardown a chance
 * to release the camera before sending SIGKILL. */
static int terminate_child(pid_t pid, int *status)
{
  int result = wait_child(pid, status, 0);
  if (result != 0) return result;
  if (kill(pid, SIGTERM) != 0 && errno != ESRCH) return -1;
  result = wait_child(pid, status, CHILD_STOP_GRACE_MS);
  if (result != 0) return result;
  if (kill(pid, SIGKILL) != 0 && errno != ESRCH) return -1;
  return wait_child(pid, status, CHILD_STOP_GRACE_MS);
}

/** @brief Open a close-on-exec pipe for child protocol setup.
 * @param[out] descriptors Read and write descriptors; valid only on success.
 * @return Zero on success, or -1 with errno on failure.
 * @details Keep both descriptors above stderr so child's descriptor
 * redirection cannot alias an endpoint when a standard descriptor was closed. */
static int create_cloexec_pipe(int descriptors[2])
{
  if (pipe(descriptors) != 0) return -1;
  for (size_t index = 0; index < 2; ++index) {
    if (descriptors[index] <= STDERR_FILENO) {
      const int replacement = fcntl(descriptors[index], F_DUPFD_CLOEXEC, STDERR_FILENO + 1);
      if (replacement < 0) {
        const int saved_errno = errno;
        close(descriptors[0]);
        close(descriptors[1]);
        errno = saved_errno;
        return -1;
      }
      close(descriptors[index]);
      descriptors[index] = replacement;
    }
    int flags = fcntl(descriptors[index], F_GETFD);
    if (flags < 0 || fcntl(descriptors[index], F_SETFD, flags | FD_CLOEXEC) != 0) {
      int saved_errno = errno;
      close(descriptors[0]);
      close(descriptors[1]);
      errno = saved_errno;
      return -1;
    }
  }
  return 0;
}

void lwir_cam_pipe_set_photo_directory(const char *directory)
{
  photo_directory = directory == NULL ? USHER_LWIR_CAM_PHOTO_DIR : directory;
}

void lwir_cam_pipe_set_native_raw(int enabled)
{
  native_raw_enabled = enabled != 0;
}

int lwir_cam_pipe_set_calibration(const char *path)
{
  if (!LWIR_ENABLE_ANALYSIS) { errno = ENOTSUP; return -1; }
  if (path == NULL || path[0] == '\0') return -1;
  calibration_path = path;
  return 0;
}

struct capture_timing lwir_cam_pipe_capture_timing(void)
{
  return capture_times;
}

/** @brief Parse one server timing payload after its protocol prefix.
 * @param text Space-separated unsigned timing fields.
 * @param timing Receives validated timing evidence.
 * @param callback True for callback sequence/drop fields, false for polling timing.
 * @return 1 for valid evidence, 0 for malformed or implausible evidence.
 * @details Parsing is intentionally strict: timing metadata enriches EXIF but must
 * never be accepted merely because a server process printed a vaguely similar line. */
static int parse_capture_timing(const char *text, struct capture_timing *timing, bool callback)
{
  if (!text || !timing) return 0;
  uint64_t values[4] = {0};
  const size_t count = callback ? 4 : 2;
  for (size_t index = 0; index < count; ++index) {
    char *end;
    if (*text < '0' || *text > '9') return 0;
    errno = 0;
    unsigned long long value = strtoull(text, &end, 10);
    if (errno != 0 || value > UINT64_MAX) return 0;
    values[index] = value;
    if (index + 1 == count) {
      if (*end != '\0') return 0;
    } else {
      if (*end != ' ') return 0;
      text = end + 1;
    }
  }
  struct capture_timing parsed = {
    .request_monotonic_us = values[0], .arrival_monotonic_us = values[1],
    .callback_sequence = values[2], .callback_drops = values[3], .callback_arrival = callback
  };
  if (!capture_timing_valid(&parsed)) return 0;
  *timing = parsed;
  return 1;
}

double lwir_cam_pipe_capture_delay(void)
{
  return capture_delay_s;
}

/** @brief Read server diagnostic lines until an expected terminal reply arrives.
 * @param expected Success line required to complete the operation.
 * @param timeout_ms Whole-operation deadline.
 * @param keep_running Optional cancellation flag; checked between bounded polls
 * so stopping USHER need not wait for a thermal camera's full startup deadline.
 * @return 0 on the expected reply, -1 for cancellation, timeout, EOF, or server error.
 * @details Informational timing lines are parsed before the terminal reply. The buffer
 * avoids a syscall per character while retaining line framing. One absolute deadline
 * prevents a noisy or malicious child from extending an operation indefinitely. */
static int read_server_status(const char *expected, int timeout_ms,
                              const volatile sig_atomic_t *keep_running)
{
  char line[256] = {0};
  size_t line_size = 0;
  const uint64_t started_ms = monotonic_ms();
  if (started_ms == 0) return -1;

  for (;;) {
    if (!capture_running(keep_running)) return -1;
    if (status_buffer_pos == status_buffer_size) {
      if (capture_server_output < 0) return -1;
      uint64_t now_ms = monotonic_ms();
      if (now_ms == 0 || now_ms - started_ms >= (uint64_t)timeout_ms) return -1;
      int remaining_ms = timeout_ms - (int)(now_ms - started_ms);
      int poll_timeout_ms = keep_running != NULL && remaining_ms > 100 ? 100 : remaining_ms;
      struct pollfd descriptor = {capture_server_output, POLLIN, 0};
      int poll_result = poll(&descriptor, 1, poll_timeout_ms);
      if (poll_result < 0 && errno == EINTR) continue;
      if (poll_result == 0) continue;
      if (poll_result < 0
          || ((descriptor.revents & POLLIN) == 0
              && (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0)) {
        return -1;
      }
      ssize_t bytes_read = read(capture_server_output, status_buffer, sizeof(status_buffer));
      if (bytes_read < 0 && (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)) continue;
      if (bytes_read <= 0) {
        return -1;
      }
      status_buffer_size = (size_t)bytes_read;
      status_buffer_pos = 0;
    }
    char character = status_buffer[status_buffer_pos++];
    if (character != '\n' && ((unsigned char)character < 32 || character == 127 || line_size + 1 >= sizeof(line))) return -1;
    if (character != '\n' && line_size + 1 < sizeof(line)) {
      line[line_size++] = character;
      continue;
    }

    line[line_size] = '\0';
    double delay;
    char trailing;
    if (strcmp(expected, "LWIR_SERVER_OK") == 0 && !capture_timing_valid(&capture_times)
        && sscanf(line, "LWIR_SERVER_DELAY %lf %c", &delay, &trailing) == 1
        && isfinite(delay) && delay >= 0 && delay <= 20) {
      capture_delay_s = delay;
    }
    const char timing_prefix[] = "LWIR_SERVER_TIMING ";
    if (strcmp(expected, "LWIR_SERVER_OK") == 0
        && !capture_times.callback_arrival
      && line_size >= sizeof(timing_prefix)
        && strncmp(line, timing_prefix, sizeof(timing_prefix) - 1) == 0
        && parse_capture_timing(line + sizeof(timing_prefix) - 1, &capture_times, false)) {
      capture_delay_s = (capture_times.arrival_monotonic_us - capture_times.request_monotonic_us) / 1e6;
    }
    const char callback_prefix[] = "LWIR_SERVER_CALLBACK_TIMING ";
    if (strcmp(expected, "LWIR_SERVER_OK") == 0
      && line_size >= sizeof(callback_prefix)
        && strncmp(line, callback_prefix, sizeof(callback_prefix) - 1) == 0
        && parse_capture_timing(line + sizeof(callback_prefix) - 1, &capture_times, true)) {
      capture_delay_s = (capture_times.arrival_monotonic_us - capture_times.request_monotonic_us) / 1e6;
    }
    if (strcmp(line, expected) == 0) {
      return 0;
    }
    if (strncmp(line, "LWIR_SERVER_ERROR", 17) == 0) {
      fprintf(stderr, "LWIR_CAM_PIPE:\t%s\n", line);
      return -1;
    }
    line_size = 0;
  }
}

/** @brief Close pipes and reap or terminate the owned capture server.
 * @details Buffered response state is reset before the next spawn. Closing stdin asks
 * cooperative server versions to exit; SIGTERM is used only when it remains alive. */
static void stop_capture_server(void)
{
  status_buffer_size = status_buffer_pos = 0;
  if (capture_server_input >= 0) {
    close(capture_server_input);
    capture_server_input = -1;
  }
  if (capture_server_output >= 0) {
    close(capture_server_output);
    capture_server_output = -1;
  }
  if (capture_server_pid < 0) {
    return;
  }

  int status = 0;
  if (terminate_child(capture_server_pid, &status) <= 0) {
    fprintf(stderr, "LWIR_CAM_PIPE:\tcapture server pid %ld could not be reaped\n",
            (long)capture_server_pid);
    return;
  }
  capture_server_pid = -1;
}

/** Send one line request to the running server and wait for its reply. On any
 * failure (broken pipe, dead process, or a timed-out/erroring reply) the server
 * state is fully reset so a caller's retry starts with a clean respawn instead of
 * repeatedly hitting the same stale, already-dead connection. */
/** @brief Send one newline-terminated server request and require an OK reply.
 * @return 0 on end-to-end success or -1 after fully resetting failed server state. */
static int send_request_and_wait(const char *request, size_t request_size, int timeout_ms, const char *expected)
{
  const uint64_t started_ms = monotonic_ms();
  if (!started_ms || timeout_ms <= 0) { stop_capture_server(); return -1; }
  size_t sent = 0;
  while (sent < request_size) {
    if (!capture_running(running)) { stop_capture_server(); return -1; }
    const uint64_t now_ms = monotonic_ms();
    if (!now_ms || now_ms < started_ms || now_ms - started_ms >= (uint64_t)timeout_ms) {
      stop_capture_server(); return -1;
    }
    ssize_t written = write(capture_server_input, request + sent, request_size - sent);
    if (written > 0) {
      sent += (size_t)written;
    } else if (written < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      struct pollfd writable = {.fd = capture_server_input, .events = POLLOUT};
      const int remaining = timeout_ms - (int)(now_ms - started_ms);
      const int ready = poll(&writable, 1, running && remaining > 100 ? 100 : remaining);
      if (ready < 0 && errno == EINTR) continue;
      if (ready == 0 || (ready > 0 && !(writable.revents & (POLLERR | POLLHUP | POLLNVAL)))) continue;
      stop_capture_server();
      return -1;
    } else if (written == 0 || errno != EINTR) {
      if (!written) errno = EIO;
      fprintf(stderr, "LWIR_CAM_PIPE:\tfailed to send request: %s\n", strerror(errno));
      stop_capture_server();
      return -1;
    }
  }
  const uint64_t elapsed_ms = monotonic_ms() - started_ms;
  if (elapsed_ms >= (uint64_t)timeout_ms || read_server_status(expected, timeout_ms - (int)elapsed_ms, running) != 0) {
    stop_capture_server();
    return -1;
  }
  return 0;
}

/** @brief Start the persistent thermal capture subprocess.
 * @param unused Reserved backend argument.
 * @param keep_running Optional shutdown flag used during the readiness wait.
 * @return Zero after readiness, or -1 on ownership, setup, or protocol failure.
 * @details Resolve any old owner before launching. Connect separate command
 * and response pipes and require the startup acknowledgement before capture. */
static int start_capture_server(const char *unused,
                                const volatile sig_atomic_t *keep_running)
{
  (void)unused;
  if (capture_server_pid >= 0) {
    stop_capture_server();
    if (capture_server_pid >= 0) return -1;
  }
  char resolved_dir[PATH_MAX];
  const char *photo_dir = usher_resolve_path(photo_directory, resolved_dir, sizeof(resolved_dir));

  if (usher_ensure_directory(photo_dir) != 0) {
    fprintf(stderr, "LWIR_CAM_PIPE:\tfailed to create photo directory %s: %s\n",
            photo_dir, strerror(errno));
    return -1;
  }

  struct stat directory_status;
  if (stat(photo_dir, &directory_status) != 0
      || !S_ISDIR(directory_status.st_mode)
      || access(photo_dir, W_OK) != 0) {
    fprintf(stderr, "LWIR_CAM_PIPE:\tphoto directory is not writable: %s\n",
            photo_dir);
    return -1;
  }
  if (access(capture_command, X_OK) != 0) {
    fprintf(stderr, "LWIR_CAM_PIPE:\tcapture command is not executable: %s: %s\n",
            capture_command, strerror(errno));
    return -1;
  }

  int command_pipe[2];
  int response_pipe[2];
  if (create_cloexec_pipe(command_pipe) != 0) {
    fprintf(stderr, "LWIR_CAM_PIPE:\tfailed to create capture-server pipes: %s\n",
            strerror(errno));
    return -1;
  }
  if (create_cloexec_pipe(response_pipe) != 0) {
    fprintf(stderr, "LWIR_CAM_PIPE:\tfailed to create capture-server pipes: %s\n",
            strerror(errno));
    close(command_pipe[0]);
    close(command_pipe[1]);
    return -1;
  }

  posix_spawn_file_actions_t actions;
  int action_result = posix_spawn_file_actions_init(&actions);
  bool actions_initialized = action_result == 0;
  if (action_result == 0) action_result = posix_spawn_file_actions_adddup2(&actions, command_pipe[0], STDIN_FILENO);
  if (action_result == 0) action_result = posix_spawn_file_actions_adddup2(&actions, response_pipe[1], STDOUT_FILENO);
  if (action_result == 0) action_result = posix_spawn_file_actions_addclose(&actions, command_pipe[1]);
  if (action_result == 0) action_result = posix_spawn_file_actions_addclose(&actions, response_pipe[0]);
  if (action_result != 0) {
    fprintf(stderr, "LWIR_CAM_PIPE:\tfailed to configure capture server: %s\n",
            strerror(action_result));
    if (actions_initialized) posix_spawn_file_actions_destroy(&actions);
    close(command_pipe[0]);
    close(command_pipe[1]);
    close(response_pipe[0]);
    close(response_pipe[1]);
    return -1;
  }

  char *arguments[10];
  size_t count = 0;
  arguments[count++] = (char *)capture_command;
  arguments[count++] = (char *)"--capture-server";
  arguments[count++] = (char *)"--bare";
  if (lwir_config) {
    arguments[count++] = (char *)"--config";
    arguments[count++] = (char *)lwir_config;
  }
  if (native_raw_enabled) arguments[count++] = (char *)"--native-raw";
  if (calibration_path != NULL) {
    arguments[count++] = (char *)"--calibration";
    arguments[count++] = (char *)calibration_path;
  }
  arguments[count] = NULL;
  int spawn_result = posix_spawn(&capture_server_pid, capture_command,
                                 &actions, NULL, arguments, environ);
  posix_spawn_file_actions_destroy(&actions);
  close(command_pipe[0]);
  close(response_pipe[1]);
  if (spawn_result != 0) {
    fprintf(stderr, "LWIR_CAM_PIPE:\tfailed to start %s: %s\n",
            capture_command, strerror(spawn_result));
    close(command_pipe[1]);
    close(response_pipe[0]);
    capture_server_pid = -1;
    return -1;
  }
  capture_server_input = command_pipe[1];
  capture_server_output = response_pipe[0];
  const int endpoints[] = {capture_server_input, capture_server_output};
  for (size_t index = 0; index < sizeof(endpoints) / sizeof(endpoints[0]); ++index) {
    const int flags = fcntl(endpoints[index], F_GETFL);
    if (flags < 0 || fcntl(endpoints[index], F_SETFL, flags | O_NONBLOCK) < 0) {
      stop_capture_server();
      return -1;
    }
  }
  if (read_server_status("LWIR_SERVER_READY", LWIR_SERVER_START_TIMEOUT_MS, keep_running) != 0) {
    fprintf(stderr, "LWIR_CAM_PIPE:\tcapture server did not become ready\n");
    stop_capture_server();
    return -1;
  }

  printf("LWIR_CAM_PIPE:\tcapture command: %s\n", capture_command);
  printf("LWIR_CAM_PIPE:\tphoto directory: %s\n", photo_dir);
  printf("LWIR_CAM_PIPE:\tpersistent capture server ready\n");
  return 0;
}

int lwir_cam_pipe_init(const char *unused)
{
  return start_capture_server(unused, NULL);
}

int lwir_cam_pipe_init_cancelable(const char *unused,
                                  const volatile sig_atomic_t *keep_running)
{
  return start_capture_server(unused, keep_running);
}

int lwir_cam_pipe_warmup(void)
{
  /** Open the sensor once, wait past its power-on "wiggly" image, then close it
   * again. Runs regardless of whether LWIR ends up selected for a shot, so a
   * later real open (persistent server or standalone capture) never has to
   * absorb that transient inline. Failure here is not fatal to USHER. */
  if (access(USHER_LWIR_CAM_COMMAND, X_OK) != 0) {
    fprintf(stderr, "LWIR_CAM_PIPE:\twarmup skipped, command is not executable: %s: %s\n",
            USHER_LWIR_CAM_COMMAND, strerror(errno));
    return -1;
  }

  char *const arguments[] = {
    (char *)USHER_LWIR_CAM_COMMAND,
    (char *)"--warmup",
    NULL
  };
  pid_t warmup_pid;
  int spawn_result = posix_spawn(&warmup_pid, USHER_LWIR_CAM_COMMAND,
                                 NULL, NULL, arguments, environ);
  if (spawn_result != 0) {
    fprintf(stderr, "LWIR_CAM_PIPE:\tfailed to start warmup: %s\n", strerror(spawn_result));
    return -1;
  }

  int status = 0;
  int wait_result = wait_child(warmup_pid, &status, LWIR_WARMUP_TIMEOUT_MS);
  if (wait_result == 0) wait_result = terminate_child(warmup_pid, &status);
  if (wait_result <= 0 || !WIFEXITED(status) || WEXITSTATUS(status) != 0) {
    fprintf(stderr, "LWIR_CAM_PIPE:\tsensor not accessible or warmup failed\n");
    return -1;
  }
  return 0;
}

/* Keep standalone mock/geolocation work separate from real capture: real shots
 * use the warmed server to avoid startup latency and competing camera ownership. */
#if LWIR_ENABLE_ANALYSIS || LWIR_ENABLE_MOCK
static int process_image(char *filename, int geolocate)
{
  if (filename == NULL || filename[0] == '\0') {
    return -1;
  }

  char *arguments[7];
  size_t count = 0;
  arguments[count++] = (char *)USHER_LWIR_CAM_COMMAND;
  if (geolocate) {
    arguments[count++] = (char *)"--geolocate";
    arguments[count++] = filename;
    if (calibration_path != NULL) {
      arguments[count++] = (char *)"--calibration";
      arguments[count++] = (char *)calibration_path;
    }
  } else {
    arguments[count++] = (char *)"--mock-image";
    arguments[count++] = filename;
    arguments[count++] = (char *)"--output";
    arguments[count++] = filename;
  }
  arguments[count] = NULL;
  pid_t camera_pid;
  int spawn_result = posix_spawn(&camera_pid, USHER_LWIR_CAM_COMMAND,
                                 NULL, NULL, arguments, environ);
  if (spawn_result != 0) {
    fprintf(stderr, "LWIR_CAM_PIPE:\tfailed to start image processing: %s\n",
            strerror(spawn_result));
    return -1;
  }

  int status = 0;
  int wait_result = wait_child(camera_pid, &status, IMAGE_PROCESS_TIMEOUT_MS);
  if (wait_result == 0) wait_result = terminate_child(camera_pid, &status);
  if (wait_result <= 0 || !WIFEXITED(status) || WEXITSTATUS(status) != 0) {
    fprintf(stderr, "LWIR_CAM_PIPE:\timage processing failed\n");
    return -1;
  }
  return 0;
}
#endif

int lwir_cam_pipe_process_mock(char *filename)
{
#if LWIR_ENABLE_MOCK
  return process_image(filename, 0);
#else
  (void)filename;
  errno = ENOTSUP;
  return -1;
#endif
}

int lwir_cam_pipe_geolocate(char *filename)
{
#if LWIR_ENABLE_ANALYSIS
  if (filename == NULL || filename[0] == '\0') {
    return -1;
  }
  if (capture_server_pid < 0 || capture_server_input < 0 || capture_server_output < 0) {
    /** No persistent server running (test/mock mode): fall back to a one-off process. */
    return process_image(filename, 1);
  }

  /** Reuse the already-running, already-warmed-up server for analysis instead of
   * spawning a fresh lwircam process for every shot. */
  char request[PATH_MAX + 8];
  int request_size = snprintf(request, sizeof(request), "GEO:%s\n", filename);
  if (request_size < 0 || (size_t)request_size >= sizeof(request)) {
    return -1;
  }
  if (send_request_and_wait(request, (size_t)request_size, LWIR_REQUEST_TIMEOUT_MS, "LWIR_SERVER_OK") != 0) {
    fprintf(stderr, "LWIR_CAM_PIPE:\tcapture server failed to geolocate %s\n", filename);
    return -1;
  }
  return 0;
#else
  (void)filename;
  errno = ENOTSUP;
  return -1;
#endif
}

int lwir_cam_pipe_shoot(char *filename, size_t filename_size, int image_number)
{
  capture_delay_s = -1;
  capture_times = (struct capture_timing) {0};
  if (filename == NULL || filename_size == 0 || image_number < 0) {
    return -1;
  }

  char resolved_dir[PATH_MAX];
  const char *photo_dir = usher_resolve_path(photo_directory, resolved_dir, sizeof(resolved_dir));

  int length = snprintf(filename, filename_size, "%s/l%06d.jpg",
                        photo_dir, image_number);
  if (length < 0 || (size_t)length >= filename_size) {
    filename[0] = '\0';
    return -1;
  }

  struct stat existing;
  if (lstat(filename, &existing) == 0 || errno != ENOENT) {
    fprintf(stderr, "LWIR_CAM_PIPE:\tfailed to replace %s: %s\n", filename, strerror(errno));
    filename[0] = '\0';
    return -1;
  }

  if (capture_server_pid < 0 || capture_server_input < 0 || capture_server_output < 0) {
    filename[0] = '\0';
    return -1;
  }
  char request[PATH_MAX + 2];
  int request_size = snprintf(request, sizeof(request), "%s\n", filename);
  if (request_size < 0 || (size_t)request_size >= sizeof(request)) {
    filename[0] = '\0';
    return -1;
  }
  if (send_request_and_wait(request, (size_t)request_size, LWIR_REQUEST_TIMEOUT_MS, "LWIR_SERVER_OK") != 0) {
    fprintf(stderr, "LWIR_CAM_PIPE:\tcapture server failed to save %s\n", filename);
    unlink(filename);
    filename[0] = '\0';
    return -1;
  }

  struct stat image_status;
  if (stat(filename, &image_status) != 0 || !S_ISREG(image_status.st_mode)
      || image_status.st_size == 0) {
    fprintf(stderr, "LWIR_CAM_PIPE:\t%s did not create a valid image at %s\n",
                capture_command, filename);
          unlink(filename);
    filename[0] = '\0';
    return -1;
  }
  return 0;
}

void lwir_cam_pipe_deinit(void)
{
  stop_capture_server();
}