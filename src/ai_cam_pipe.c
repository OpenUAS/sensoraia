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
 * @file ai_cam_pipe.c
 * @brief Supervise the selected system or native AI-camera owner and publish captured JPEGs.
 * @details Persistent capture uses private session filenames and an inotify
 * completion event so an incomplete exposure is never visible under its final
 * name. One-shot capture is allowed only when persistent mode proves no trigger
 * was sent; ambiguous post-trigger failures are never replayed.
 */
#include "ai_cam_pipe.h"
#include "camera_backend.h"
#include "imx500_pipe.h"
#include "path_utils.h"

#include <errno.h>
#include <limits.h>
#include <poll.h>
#include <signal.h>
#include <spawn.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/inotify.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

#ifndef USHER_AI_CAM_COMMAND
#define USHER_AI_CAM_COMMAND "rpicam-still"
#endif
#ifndef USHER_AI_CAM_NATIVE_COMMAND
#define USHER_AI_CAM_NATIVE_COMMAND "./bin/aicam"
#endif
#ifndef USHER_AI_CAM_PHOTO_DIR
#define USHER_AI_CAM_PHOTO_DIR "~/Pictures/sensoraia"
#endif
#ifndef USHER_AI_CAM_WIDTH
#define USHER_AI_CAM_WIDTH "2028"
#endif
#ifndef USHER_AI_CAM_HEIGHT
#define USHER_AI_CAM_HEIGHT "1520"
#endif
#ifndef USHER_AI_CAM_PERSISTENT
#define USHER_AI_CAM_PERSISTENT 1
#endif
#ifndef USHER_AI_CAM_STARTUP_MS
#define USHER_AI_CAM_STARTUP_MS 3000
#endif
#ifndef USHER_AI_CAM_SIGNAL_TIMEOUT_MS
#define USHER_AI_CAM_SIGNAL_TIMEOUT_MS 4000
#endif
#ifndef USHER_AI_CAM_TIMEOUT_SECONDS
#define USHER_AI_CAM_TIMEOUT_SECONDS 30
#endif
#ifndef USHER_AI_CAM_RETRY_MS
#define USHER_AI_CAM_RETRY_MS 30000
#endif

enum capture_result {
  CAPTURE_FAILED = -1,
  CAPTURE_SAVED = 0,
  CAPTURE_NOT_TRIGGERED = 1
};

extern char **environ;
static const char *photo_directory = USHER_AI_CAM_PHOTO_DIR;
static pid_t child_pid = -1;
static int watch_fd = -1;
static unsigned session_number;
static int64_t ready_ms;
static int64_t retry_ms;
static unsigned failures;
static bool signal_zsl;
static bool zsl_failed;
static bool timing_enabled;
static bool native_enabled;
static struct imx500_pipe native_capture = IMX500_PIPE_INITIALIZER;
static char signal_directory[PATH_MAX];
static char signal_prefix[80];

/** @brief Read the child supervision clock.
 * @return Monotonic milliseconds, or zero on clock failure.
 * @details Use elapsed time rather than calendar time for startup and shutdown. */
static int64_t monotonic_ms(void)
{
  struct timespec now;
  if (clock_gettime(CLOCK_MONOTONIC, &now) != 0) return 0;
  return (int64_t)now.tv_sec * 1000 + now.tv_nsec / 1000000;
}

/** @brief Wait until a short polling interval elapses.
 * @param milliseconds Requested wait in milliseconds.
 * @details After interruption, recompute the remaining budget instead of
 * restarting the full delay. */
static void wait_ms(int milliseconds)
{
  const int64_t deadline = monotonic_ms() + milliseconds;
  int64_t remaining;
  while ((remaining = deadline - monotonic_ms()) > 0) {
    if (poll(NULL, 0, remaining > INT_MAX ? INT_MAX : (int)remaining) >= 0 || errno != EINTR) break;
  }
}

/** @brief Poll the owned child and reap it if it exited.
 * @param[out] status Optional wait status; use only when waitpid reports exit.
 * @return True while the child remains owned.
 * @details Clear the PID only after reaping or ECHILD. Other wait errors
 * leave ownership intact rather than allowing a second camera process. */
static bool child_alive(int *status)
{
  if (child_pid <= 0) return false;
  pid_t result;
  do {
    result = waitpid(child_pid, status, WNOHANG);
  } while (result < 0 && errno == EINTR);
  if (result == 0 || (result < 0 && errno != ECHILD)) return true;
  child_pid = -1;
  return false;
}

/** @brief Give the owned child a bounded interval to exit.
 * @param milliseconds Supervision budget in milliseconds.
 * @return True once no child is owned; false on timeout.
 * @details Poll so a driver that does not return cannot block a waitpid call. */
static bool wait_child(int milliseconds)
{
  const int64_t deadline = monotonic_ms() + milliseconds;
  while (child_alive(NULL)) {
    if (monotonic_ms() >= deadline) return false;
    wait_ms(10);
  }
  return true;
}

/** @brief Request shutdown and escalate if the camera child does not exit.
 * @return True when no child remains owned.
 * @details Use the persistent application's signal protocol when its
 * watch is active. Retain the PID if even forced termination cannot be reaped. */
static bool stop_child(void)
{
  if (!child_alive(NULL)) return true;
  kill(child_pid, watch_fd >= 0 ? SIGUSR2 : SIGTERM);
  if (wait_child(1000)) return true;
  kill(child_pid, SIGKILL);
  if (wait_child(1000)) return true;
  fprintf(stderr, "AI_CAM_PIPE:\tchild %ld has not exited; camera remains owned\n", (long)child_pid);
  return false;
}

/** @brief Retire the subprocess and its directory watch.
 * @return True if subprocess ownership was released.
 * @details Close the watch even after a timeout; subsequent starts must
 * still resolve the retained child PID before opening the camera. */
static bool persistent_stop(void)
{
  const bool stopped = stop_child();
  if (watch_fd >= 0) close(watch_fd);
  watch_fd = -1;
  return stopped;
}

/** @brief Resolve the default persistent capture policy against the one-shot override.
 * @return True if the build enables persistence and the environment does not opt out.
 * @details An unset, empty, or zero one-shot override retains persistent mode. */
static bool persistent_enabled(void)
{
  const char *oneshot = getenv("USHER_AI_CAM_ONESHOT");
  return USHER_AI_CAM_PERSISTENT && (!oneshot || !*oneshot || !strcmp(oneshot, "0"));
}

/** @brief Check whether the next persistent child may use zero shutter lag.
 * @return True unless configuration or a previous ZSL failure disabled it.
 * @details Retain the failure latch across child restarts so recovery need
 * not repeat the same launch mode. */
static bool zsl_enabled(void)
{
  const char *configured = getenv("USHER_AI_CAM_ZSL");
  return !zsl_failed && (!configured || strcmp(configured, "0"));
}

/** @brief Reserve a private filename prefix for a camera child.
 * @param directory Existing output directory.
 * @return True if both path components fit and the counter did not wrap.
 * @details Include both process ID and session counter so a late event from
 * an earlier child is not accepted as a current capture. */
static bool new_session(const char *directory)
{
  if (session_number == UINT_MAX) return false;
  ++session_number;
  const int directory_length = snprintf(signal_directory, sizeof(signal_directory), "%s", directory);
  const int prefix_length = snprintf(signal_prefix, sizeof(signal_prefix), ".aicam_%ld_%u_",
                                     (long)getpid(), session_number);
  return directory_length > 0 && (size_t)directory_length < sizeof(signal_directory)
         && prefix_length > 0 && (size_t)prefix_length < sizeof(signal_prefix);
}

/** @brief Launch the configured camera application and record ownership.
 * @param arguments Null-terminated argument vector, including argv[0].
 * @return Zero on launch, or -1 on spawn failure.
 * @details Pass arguments directly rather than interpreting paths as shell input.
 * A successful spawn does not prove that the camera is ready. */
static int spawn_camera(char *const arguments[])
{
  const int result = posix_spawnp(&child_pid, USHER_AI_CAM_COMMAND, NULL, NULL, arguments, environ);
  if (result) {
    child_pid = -1;
    fprintf(stderr, "AI_CAM_PIPE:\tcannot start %s: %s\n", USHER_AI_CAM_COMMAND, strerror(result));
    return -1;
  }
  return 0;
}

/** @brief Start the signal-triggered camera application.
 * @param directory Existing output directory to watch and write.
 * @return Zero after launch, or -1 on ownership, watch, path, or spawn failure.
 * @details Attach the watch before launch to avoid missing completion events.
 * Record a warmup deadline; the first trigger waits for it outside startup. */
static int persistent_start(const char *directory)
{
  if (!persistent_stop() || !new_session(directory)) return -1;
  watch_fd = inotify_init1(IN_NONBLOCK | IN_CLOEXEC);
  if (watch_fd < 0 || inotify_add_watch(watch_fd, directory, IN_CLOSE_WRITE | IN_MOVED_TO) < 0) {
    persistent_stop();
    return -1;
  }
  char pattern[PATH_MAX];
  const int length = snprintf(pattern, sizeof(pattern), "%s/%s%%06d.jpg", directory, signal_prefix);
  if (length < 0 || (size_t)length >= sizeof(pattern)) {
    persistent_stop();
    return -1;
  }
  char *arguments[20] = {
    (char *)USHER_AI_CAM_COMMAND, (char *)"-n", (char *)"-t", (char *)"0",
    (char *)"--signal", (char *)"-o", pattern,
    (char *)"--width", (char *)USHER_AI_CAM_WIDTH,
    (char *)"--height", (char *)USHER_AI_CAM_HEIGHT
  };
  size_t count = 11;
  signal_zsl = zsl_enabled();
  if (signal_zsl) {
    arguments[count++] = (char *)"--zsl";
    arguments[count++] = (char *)"--buffer-count";
    arguments[count++] = (char *)"2";
    arguments[count++] = (char *)"--viewfinder-width";
    arguments[count++] = (char *)"640";
    arguments[count++] = (char *)"--viewfinder-height";
    arguments[count++] = (char *)"480";
  }
  arguments[count] = NULL;
  if (spawn_camera(arguments)) {
    persistent_stop();
    return -1;
  }
  ready_ms = monotonic_ms() + USHER_AI_CAM_STARTUP_MS;
  return 0;
}

/** @brief Publish a completed private JPEG at the caller's staging path.
 * @param source Private camera output, required to be nonempty and regular.
 * @param destination New sibling path; existing files are not replaced.
 * @return Zero if linked, even if private-name cleanup failed, or -1 if not linked.
 * @details A hard link makes the complete file visible without copying pixels
 * or reopening the JPEG for writing. */
static int publish_image(const char *source, const char *destination)
{
  struct stat image;
  /* Linking the completed private file publishes atomically and refuses overwrite. */
  if (lstat(source, &image) || !S_ISREG(image.st_mode) || image.st_size <= 0
      || link(source, destination)) return -1;
  if (unlink(source)) fprintf(stderr, "AI_CAM_PIPE:\tprivate image retained: %s\n", source);
  return 0;
}

/** @brief Request and collect one persistent-child capture.
 * @param destination New staging path for the completed JPEG.
 * @return Saved, not triggered, or failed after a trigger may have been delivered.
 * @details Drain old notifications before signaling. Require a completed event
 * with the current child's private prefix. Once signaled, an ambiguous outcome
 * is terminal; replay could duplicate an exposure. */
static enum capture_result persistent_shoot(const char *destination)
{
  if (watch_fd < 0 || !child_alive(NULL)) return CAPTURE_NOT_TRIGGERED;
  const int64_t remaining = ready_ms - monotonic_ms();
  if (remaining > 0) wait_ms((int)remaining);
  if (!child_alive(NULL)) return CAPTURE_NOT_TRIGGERED;
  _Alignas(struct inotify_event) char buffer[4096];
  while (read(watch_fd, buffer, sizeof(buffer)) > 0) {}
  if (kill(child_pid, SIGUSR1)) return CAPTURE_NOT_TRIGGERED;
  const int64_t deadline = monotonic_ms() + USHER_AI_CAM_SIGNAL_TIMEOUT_MS;
  while (monotonic_ms() < deadline) {
    struct pollfd watch = {.fd = watch_fd, .events = POLLIN};
    const int result = poll(&watch, 1, 20);
    if (result < 0 && errno != EINTR) break;
    if (result > 0 && (watch.revents & POLLIN)) {
      const ssize_t length = read(watch_fd, buffer, sizeof(buffer));
      size_t offset = 0;
      while (length > 0 && offset + sizeof(struct inotify_event) <= (size_t)length) {
        const struct inotify_event *event = (const struct inotify_event *)(buffer + offset);
        const size_t event_size = sizeof(*event) + event->len;
        if (event_size > (size_t)length - offset) break;
        offset += event_size;
        const size_t prefix_length = strlen(signal_prefix);
        if (!event->len || !memchr(event->name, '\0', event->len)
            || !(event->mask & (IN_CLOSE_WRITE | IN_MOVED_TO))
            || strncmp(event->name, signal_prefix, prefix_length)) continue;
        const char *sequence = event->name + prefix_length;
        const size_t digits = strspn(sequence, "0123456789");
        if (!digits || strcmp(sequence + digits, ".jpg")) continue;
        char source[PATH_MAX];
        const int source_length = snprintf(source, sizeof(source), "%s/%s", signal_directory, event->name);
        if (source_length < 0 || (size_t)source_length >= sizeof(source)) return CAPTURE_FAILED;
        return publish_image(source, destination) ? CAPTURE_FAILED : CAPTURE_SAVED;
      }
    }
    if (!child_alive(NULL)) break;
  }
  fprintf(stderr, "AI_CAM_PIPE:\tshot completion ambiguous; no replay, private prefix %s/%s\n",
          signal_directory, signal_prefix);
  return CAPTURE_FAILED;
}

/** @brief Capture using a fresh camera subprocess.
 * @param directory Existing directory for private output.
 * @param destination New staging path for the completed JPEG.
 * @return Zero after successful child exit and publication, or -1 on failure.
 * @details Do not launch until the previous child is reaped. Use an unused
 * private path and publish only after the child exits successfully. */
static int oneshot_shoot(const char *directory, const char *destination)
{
  if (!persistent_stop() || !new_session(directory)) return -1;
  char source[PATH_MAX];
  const int length = snprintf(source, sizeof(source), "%s/%s000000.jpg", directory, signal_prefix);
  if (length < 0 || (size_t)length >= sizeof(source)) return -1;
  struct stat existing;
  if (!lstat(source, &existing) || errno != ENOENT) return -1;
  char *arguments[] = {
    (char *)USHER_AI_CAM_COMMAND, (char *)"-o", source,
    (char *)"--width", (char *)USHER_AI_CAM_WIDTH,
    (char *)"--height", (char *)USHER_AI_CAM_HEIGHT,
    (char *)"-t", (char *)"200", (char *)"-n", NULL
  };
  if (spawn_camera(arguments)) return -1;
  const int64_t deadline = monotonic_ms() + (int64_t)USHER_AI_CAM_TIMEOUT_SECONDS * 1000;
  int status = -1;
  while (child_alive(&status)) {
    if (monotonic_ms() >= deadline) {
      stop_child();
      fprintf(stderr, "AI_CAM_PIPE:\tone-shot timed out; private image retained if present: %s\n", source);
      return -1;
    }
    wait_ms(20);
  }
  if (status < 0 || !WIFEXITED(status) || WEXITSTATUS(status)) return -1;
  return publish_image(source, destination);
}

void ai_cam_pipe_set_photo_directory(const char *directory)
{
  photo_directory = directory ? directory : USHER_AI_CAM_PHOTO_DIR;
}

int ai_cam_pipe_set_native(bool enabled)
{
  if (child_pid > 0 || native_capture.pid > 0) return -1;
  native_enabled = enabled;
  return 0;
}

bool ai_cam_pipe_native(void)
{
  return native_enabled;
}

void ai_cam_pipe_configure(const struct camera_backend_host *host)
{
  native_capture.running = host ? host->running : NULL;
}

int ai_cam_pipe_init(const char *unused)
{
  (void)unused;
  if (!persistent_stop()) return -1;
  failures = 0;
  retry_ms = 0;
  zsl_failed = false;
  const char *timing = getenv("USHER_AI_CAM_TIMING");
  timing_enabled = timing && !strcmp(timing, "1");
  char resolved[PATH_MAX];
  const char *directory = usher_resolve_path(photo_directory, resolved, sizeof(resolved));
  struct stat storage;
  if (usher_ensure_directory(directory) || stat(directory, &storage) || !S_ISDIR(storage.st_mode)
      || access(directory, W_OK)) return -1;
  if (native_enabled) {
    native_capture.command = USHER_AI_CAM_NATIVE_COMMAND;
    native_capture.model = NULL;
    native_capture.settings = NULL;
    native_capture.analyze = false;
    native_capture.photo_only = true;
    native_capture.directory = photo_directory;
    native_capture.prefix = 'a';
    printf("AI_CAM_PIPE:\tnative command %s, JPEG %sx%s, directory %s\n",
           native_capture.command, USHER_AI_CAM_WIDTH, USHER_AI_CAM_HEIGHT, directory);
    return imx500_pipe_start(&native_capture);
  }
  printf("AI_CAM_PIPE:\tsystem command %s, JPEG %sx%s, directory %s\n",
         USHER_AI_CAM_COMMAND, USHER_AI_CAM_WIDTH, USHER_AI_CAM_HEIGHT, directory);
  if (persistent_enabled()) persistent_start(directory);
  return 0;
}

int ai_cam_pipe_shoot(char *filename, size_t filename_size, int image_number)
{
  if (native_enabled) return imx500_pipe_shoot(&native_capture, filename, filename_size, image_number);
  if (!filename || !filename_size) return -1;
  filename[0] = '\0';
  if (image_number < 0) return -1;
  char resolved[PATH_MAX];
  const char *directory = usher_resolve_path(photo_directory, resolved, sizeof(resolved));
  const int length = snprintf(filename, filename_size, "%s/a%06d.jpg", directory, image_number);
  if (length < 0 || (size_t)length >= filename_size || usher_prepare_photo(filename, filename_size) != 0) {
    filename[0] = '\0';
    return -1;
  }
  const int64_t started = monotonic_ms();
  enum capture_result result = CAPTURE_NOT_TRIGGERED;
  const bool persistent = persistent_enabled();
  if (persistent && started >= retry_ms) {
    if (child_pid <= 0) persistent_start(directory);
    result = persistent_shoot(filename);
    if (result != CAPTURE_SAVED) {
      if (signal_zsl) zsl_failed = true;
      if (failures < 2) ++failures;
      if (failures >= 2) retry_ms = monotonic_ms() + USHER_AI_CAM_RETRY_MS;
      persistent_stop();
    } else {
      failures = 0;
      retry_ms = 0;
    }
  }
  /* Only a proven pre-trigger failure permits the one-shot fallback. */
  if (result == CAPTURE_NOT_TRIGGERED) {
    result = oneshot_shoot(directory, filename) ? CAPTURE_FAILED : CAPTURE_SAVED;
    if (persistent && monotonic_ms() >= retry_ms && child_pid <= 0) persistent_start(directory);
  }
  if (timing_enabled) fprintf(stderr, "AI_CAM_PIPE TIMING:\tshot=%d capture_total_ms=%lld result=%d\n",
                              image_number, (long long)(monotonic_ms() - started), result);
  if (result != CAPTURE_SAVED) filename[0] = '\0';
  return result == CAPTURE_SAVED ? 0 : -1;
}

void ai_cam_pipe_deinit(void)
{
  imx500_pipe_stop(&native_capture);
  persistent_stop();
}
