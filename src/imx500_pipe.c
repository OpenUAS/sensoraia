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

/** @file imx500_pipe.c
 * @brief Neutral native camera-worker supervision, independent of model results.
 * @details Keep process transport separate from inference interpretation. The
 * same READY/SHOT/OK exchange serves detector and LOOM owners without inventing
 * object results for a feature-only capture. */
#include "imx500_pipe.h"
#include "camera_capture.h"
#include "path_utils.h"

#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <poll.h>
#include <signal.h>
#include <spawn.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

/** @brief First-use RPK upload/warm-up needs more time than a warm still capture. */
#define IMX500_STARTUP_TIMEOUT_MS 180000
/** @brief Bound response waiting so an abandoned shot cannot block the owner indefinitely. */
#define IMX500_SHOT_TIMEOUT_MS 15000
/** @brief Fixed space for short protocol replies, never arbitrary child diagnostics. */
#define IMX500_RESPONSE_BYTES 16
#define IMX500_EXIT_GRACE_MS 2000
#define IMX500_KILL_GRACE_MS 1000

/** @brief Inherit the process environment; executable and model remain explicit argv. */
extern char **environ;

/** @brief Read elapsed-time evidence unaffected by wall-clock corrections.
 * @return Monotonic milliseconds, or -1 if a deadline cannot be established. */
static int64_t milliseconds(void)
{
  struct timespec now;
  if (clock_gettime(CLOCK_MONOTONIC, &now) != 0) {
    return -1;
  }
  return (int64_t)now.tv_sec * 1000 + now.tv_nsec / 1000000;
}

/** @brief Consume one exact line before an unchanged monotonic response deadline.
 * @param[in,out] pipe State containing the nonblocking status descriptor.
 * @param[in] expected Complete expected line, including its newline.
 * @param[in] timeout Response allowance in milliseconds.
 * @return Zero for an exact line, otherwise -1.
 * @details Bytewise reads are cheap for these tiny replies and do not steal bytes
 * belonging to a later exchange. EINTR retries retain the original deadline. */
static int expect_response(struct imx500_pipe *pipe, const char *expected, int timeout)
{
  const int64_t start = milliseconds();
  if (start < 0) {
    return -1;
  }
  char response[IMX500_RESPONSE_BYTES];
  size_t length = 0;
  while (length + 1 < sizeof(response)) {
    if (!capture_running(pipe->running)) return -1;
    const int64_t now = milliseconds();
    if (now < start || now - start >= timeout) {
      return -1;
    }
    struct pollfd descriptor = {.fd = pipe->socket, .events = POLLIN};
    int wait_ms = timeout - (int)(now - start);
    if (pipe->running && wait_ms > 100) wait_ms = 100;
    const int ready = poll(&descriptor, 1, wait_ms);
    if (ready < 0 && errno == EINTR) {
      continue;
    }
    if (ready == 0) continue;
    if (ready < 0) {
      return -1;
    }
    const ssize_t bytes = read(pipe->socket, response + length, 1);
    if (bytes < 0 && (errno == EINTR || errno == EAGAIN)) {
      continue;
    }
    if (bytes != 1) {
      return -1;
    }
    if (response[length++] == '\n') {
      response[length] = '\0';
      return strcmp(response, expected) == 0 ? 0 : -1;
    }
  }
  return -1;
}

/** @brief Poll for exit of the camera child.
 * @param[in,out] pipe Subprocess owner; clears the PID once ownership ends.
 * @param timeout_ms Polling budget in milliseconds.
 * @return True when no child remains owned.
 * @details Leave the PID intact on timeout or hard wait errors so callers
 * cannot launch another sensor owner while the previous child may be alive. */
static bool reap_owned_camera(struct imx500_pipe *pipe, int timeout_ms)
{
  if (pipe->pid <= 0) return true;
  const int64_t started = milliseconds();
  for (;;) {
    const pid_t result = waitpid(pipe->pid, NULL, WNOHANG);
    if (result == pipe->pid || (result < 0 && errno == ECHILD)) { pipe->pid = -1; return true; }
    if (result < 0 && errno != EINTR) return false;
    const int64_t now = milliseconds();
    if (started < 0 || now < started || now - started >= timeout_ms) return false;
    if (poll(NULL, 0, 20) < 0 && errno != EINTR) return false;
  }
}

void imx500_pipe_stop(struct imx500_pipe *pipe)
{
  if (!pipe) {
    return;
  }
  if (pipe->socket >= 0) {
    /* EOF is the healthy worker's shutdown request; close it before escalating. */
    close(pipe->socket);
  }
  pipe->socket = -1;
  if (!reap_owned_camera(pipe, IMX500_EXIT_GRACE_MS)) {
    kill(pipe->pid, SIGKILL);
    if (!reap_owned_camera(pipe, IMX500_KILL_GRACE_MS))
      fprintf(stderr, "IMX500_CAPTURE:\tkilled worker has not exited; retaining process ownership\n");
  }
}

int imx500_pipe_start(struct imx500_pipe *pipe)
{
    if (!pipe || !pipe->command || !pipe->command[0] ||
  (pipe->photo_only && (pipe->analyze || pipe->settings || pipe->model)) ||
  (!pipe->photo_only && ((!pipe->settings || !pipe->settings[0]) && (!pipe->model || !pipe->model[0]))) ||
      (!pipe->analyze && (!pipe->directory || pipe->prefix < 'a' || pipe->prefix > 'z'))) {
    return -1;
  }
  imx500_pipe_stop(pipe);
  if (pipe->pid > 0 || !capture_running(pipe->running)) return -1;
  /* A full-duplex private channel avoids shell quoting and shared stdout parsing. */
  int sockets[2];
  if (socketpair(AF_UNIX, SOCK_STREAM, 0, sockets) != 0) {
    return -1;
  }
  for (size_t index = 0; index < 2; ++index) {
    /* Keep endpoints above stdio before dup2, even when the parent had closed fd 0 or 1. */
    const int descriptor = fcntl(sockets[index], F_DUPFD_CLOEXEC, 3);
    close(sockets[index]);
    sockets[index] = descriptor;
    if (descriptor < 0) {
      close(sockets[1 - index]);
      return -1;
    }
  }
  posix_spawn_file_actions_t actions;
  int status = posix_spawn_file_actions_init(&actions);
  if (status == 0) {
    status = posix_spawn_file_actions_adddup2(&actions, sockets[1], STDIN_FILENO);
    if (status == 0) {
      status = posix_spawn_file_actions_adddup2(&actions, sockets[1], STDOUT_FILENO);
    }
    if (status == 0) {
      status = posix_spawn_file_actions_addclose(&actions, sockets[0]);
    }
    if (status == 0) {
      status = posix_spawn_file_actions_addclose(&actions, sockets[1]);
    }
    char *arguments[] = {(char *)pipe->command, "--serve", (char *)pipe->model,
                         USHER_AI_CAM_WIDTH, USHER_AI_CAM_HEIGHT, NULL};
    char *configured[] = {(char *)pipe->command, pipe->analyze ? "--analyze-server" : "--serve",
      "--settings", (char *)pipe->settings, pipe->analyze ? "--private-images" : USHER_AI_CAM_WIDTH,
      pipe->analyze ? NULL : USHER_AI_CAM_HEIGHT, NULL};
    char *photo_only[] = {(char *)pipe->command, "--photo-only", "--serve",
                          USHER_AI_CAM_WIDTH, USHER_AI_CAM_HEIGHT, NULL};
    if (status == 0) {
      pid_t spawned = -1;
      status = posix_spawnp(&spawned, arguments[0], &actions, NULL,
                            pipe->photo_only ? photo_only : pipe->settings ? configured : arguments, environ);
      if (!status) pipe->pid = spawned;
    }
    posix_spawn_file_actions_destroy(&actions);
  }
  close(sockets[1]);
  pipe->socket = sockets[0];
  /* Spawn success is not readiness: the child's selected processor must qualify a frame. */
  if (status != 0 || fcntl(pipe->socket, F_SETFL, O_NONBLOCK) < 0
      || expect_response(pipe, "READY\n", IMX500_STARTUP_TIMEOUT_MS) != 0) {
    fprintf(stderr, "IMX500_CAPTURE:\tstartup failed: %s\n", pipe->command);
    imx500_pipe_stop(pipe);
    return -1;
  }
  return 0;
}

/** @brief Send one request before a single absolute deadline, including partial writes. */
static bool send_request(struct imx500_pipe *pipe, const char *request, size_t length)
{
  const int64_t start = milliseconds();
  if (start < 0) return false;
  size_t sent = 0;
  while (sent < length) {
    if (!capture_running(pipe->running)) return false;
    const int64_t now = milliseconds();
    if (now < start || now - start >= 2000) return false;
    struct pollfd descriptor = {.fd = pipe->socket, .events = POLLOUT};
    int wait_ms = 2000 - (int)(now - start);
    if (pipe->running && wait_ms > 100) wait_ms = 100;
    const int ready = poll(&descriptor, 1, wait_ms);
    if (ready < 0 && errno == EINTR) continue;
    if (!ready) continue;
    if (ready < 0 || (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL))) return false;
    const ssize_t bytes = send(pipe->socket, request + sent, length - sent, MSG_NOSIGNAL);
    if (bytes < 0 && (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)) continue;
    if (bytes <= 0) return false;
    sent += (size_t)bytes;
  }
  return true;
}

int imx500_pipe_analyze(struct imx500_pipe *pipe, const char *filename, unsigned number)
{
  if (!pipe || !pipe->analyze || pipe->socket < 0 || !filename || !number ||
      strchr(filename, '\n') || strchr(filename, '\r')) return -1;
  char request[PATH_MAX + 64];
  const int count = snprintf(request, sizeof(request), "ANALYZE %u %s\n", number, filename);
  if (count < 0 || (size_t)count >= sizeof(request)) return -1;
  if (send_request(pipe, request, (size_t)count) && expect_response(pipe, "OK\n", IMX500_SHOT_TIMEOUT_MS) == 0) return 0;
  imx500_pipe_stop(pipe);
  return -1;
}

int imx500_pipe_shoot(struct imx500_pipe *pipe, char *filename, size_t capacity, int number)
{
  if (!pipe || !filename || capacity == 0 || number < 0) {
    return -1;
  }
  filename[0] = '\0';
  char resolved[PATH_MAX];
  const char *directory = usher_resolve_path(pipe->directory, resolved, sizeof(resolved));
  if (usher_ensure_directory(directory) != 0) {
    return -1;
  }
  char absolute[PATH_MAX];
  /* Resolve the parent-selected directory once; the child receives no relative path policy. */
  if (!realpath(directory, absolute)) {
    return -1;
  }
  int length = snprintf(filename, capacity, "%s/%c%06d.jpg", absolute, pipe->prefix, number);
  char request[PATH_MAX + 16];
  if (length < 0 || (size_t)length >= capacity || strchr(filename, '\n') || strchr(filename, '\r')
      || usher_prepare_photo(filename, capacity) != 0) {
    filename[0] = '\0';
    return -1;
  }
  length = snprintf(request, sizeof(request), "SHOT %s\n", filename);
  if (length < 0 || (size_t)length >= sizeof(request)) {
    filename[0] = '\0';
    return -1;
  }
  if (pipe->socket < 0 && imx500_pipe_start(pipe) != 0) {
    filename[0] = '\0';
    return -1;
  }
  struct stat image;
  /* A response alone cannot authorize publication. Keep the file check separate
   * from model-specific metadata validation, which belongs above this supervisor. */
  if (!send_request(pipe, request, (size_t)length) || expect_response(pipe, "OK\n", IMX500_SHOT_TIMEOUT_MS) != 0
      || stat(filename, &image) != 0 || !S_ISREG(image.st_mode) || image.st_size <= 0) {
    imx500_pipe_stop(pipe);
    filename[0] = '\0';
    return -1;
  }
  return 0;
}