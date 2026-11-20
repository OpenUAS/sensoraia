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
 * @file chdk_pipe.c
 * @brief Own and supervise bounded CHDKbridge child invocations for USHER.
 * @details The child is launched directly, not through a shell, and remains
 * owned until reaped. Deadlines and process-group shutdown are required because
 * USB work can outlive a single caller; an ambiguous exposure is never replayed.
 */
#include "chdk_pipe.h"
#include "camera_capture.h"
#include "usher_protocol.h"
#include <errno.h>
#include <dirent.h>
#include <fcntl.h>
#include <signal.h>
#include <spawn.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/resource.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

#ifndef USHER_CHDK_COMMAND
#define USHER_CHDK_COMMAND "./bin/chdkbridge"
#endif
#ifndef USHER_CHDK_ISO_MODE
#define USHER_CHDK_ISO_MODE "1600"
#endif
#ifndef USHER_CHDK_FLASH
#define USHER_CHDK_FLASH "2"
#endif
#ifndef USHER_CHDK_RESOLUTION
#define USHER_CHDK_RESOLUTION "0"
#endif
#ifndef USHER_CHDK_QUALITY
#define USHER_CHDK_QUALITY "2"
#endif
#ifndef USHER_CHDK_DATE_STAMP
#define USHER_CHDK_DATE_STAMP "0"
#endif
#ifndef USHER_CHDK_AF_ASSIST
#define USHER_CHDK_AF_ASSIST "0"
#endif
#ifndef USHER_CHDK_RAW
#define USHER_CHDK_RAW "0"
#endif
#ifndef USHER_CHDK_FOCUS_MODE
#define USHER_CHDK_FOCUS_MODE ""
#endif
#ifndef USHER_CHDK_FOCUS_DISTANCE
#define USHER_CHDK_FOCUS_DISTANCE ""
#endif
#ifndef USHER_CHDK_FOCUS_TIMEOUT
#define USHER_CHDK_FOCUS_TIMEOUT ""
#endif
#ifndef USHER_CHDK_FOCUS_TOLERANCE
#define USHER_CHDK_FOCUS_TOLERANCE ""
#endif
#ifndef USHER_CHDK_SHUTTER
#define USHER_CHDK_SHUTTER ""
#endif
#ifndef USHER_CHDK_TV96
#define USHER_CHDK_TV96 ""
#endif
#ifndef USHER_CHDK_ISO_REAL
#define USHER_CHDK_ISO_REAL ""
#endif
#ifndef USHER_CHDK_ISO_MARKET
#define USHER_CHDK_ISO_MARKET ""
#endif
#ifndef USHER_CHDK_SV96
#define USHER_CHDK_SV96 ""
#endif
#ifndef USHER_CHDK_APERTURE
#define USHER_CHDK_APERTURE ""
#endif
#ifndef USHER_CHDK_AV96
#define USHER_CHDK_AV96 ""
#endif
#ifndef USHER_CHDK_ND
#define USHER_CHDK_ND ""
#endif
#ifndef USHER_CHDK_EXPOSURE_TIMEOUT
#define USHER_CHDK_EXPOSURE_TIMEOUT ""
#endif
#ifndef USHER_CHDK_EXPOSURE_TOLERANCE
#define USHER_CHDK_EXPOSURE_TOLERANCE ""
#endif
#ifndef USHER_CHDK_TIMEOUT
#define USHER_CHDK_TIMEOUT "60"
#endif
#ifndef USHER_CHDK_USB_TIMEOUT
#define USHER_CHDK_USB_TIMEOUT "5000"
#endif
#ifndef USHER_CHDK_MAX_DOWNLOAD_MIB
#define USHER_CHDK_MAX_DOWNLOAD_MIB "64"
#endif
#ifndef USHER_CHDK_DIRECT
#define USHER_CHDK_DIRECT "0"
#endif
#ifndef USHER_CHDK_KEEP_AWAKE
#define USHER_CHDK_KEEP_AWAKE "1"
#endif
#ifndef USHER_CHDK_PORT
#define USHER_CHDK_PORT ""
#endif

#ifndef USHER_CHDK_BUS
#define USHER_CHDK_BUS ""
#endif
#ifndef USHER_CHDK_DEVICE
#define USHER_CHDK_DEVICE ""
#endif

extern char **environ;
static pid_t camera_pid = -1;
static bool camera_ready;
static bool partial_cleanup_pending = true;
static const volatile sig_atomic_t *camera_running;
static struct {
  int descriptor;
  size_t size;
  uint8_t bytes[CHDK_CONTROL_REPLY_SIZE];
  bool invalid;
} camera_reply = {.descriptor = -1};

/** @brief Collect one chunk from the nonblocking control reply pipe.
 * @details Retain partial bytes across polls; flag overflow and hard I/O
 * errors so a truncated or overlong reply cannot count as confirmation. */
static void drain_control_reply(void)
{
  if (camera_reply.descriptor < 0) return;
  uint8_t bytes[32];
  ssize_t received = read(camera_reply.descriptor, bytes, sizeof(bytes));
  if (received < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) camera_reply.invalid = true;
  for (ssize_t index = 0; index < received; ++index) {
    if (camera_reply.size < sizeof(camera_reply.bytes)) camera_reply.bytes[camera_reply.size++] = bytes[index];
    else camera_reply.invalid = true;
  }
}

/** @brief Configure cancellation for controller operations.
 * @param running Borrowed service flag, or NULL to disable flag-based cancellation.
 * @details Keep the flag alive for the lifetime of controller work; cleanup
 * waits must still reap a child after cancellation. */
void chdk_pipe_set_running(const volatile sig_atomic_t *running)
{
  camera_running = running;
}

/** @brief Read the clock used for controller supervision.
 * @return Monotonic milliseconds, or zero on clock failure.
 * @details Deadlines must not move when the wall clock is corrected. */
static uint64_t chdk_monotonic_ms(void)
{
  struct timespec now;
  if (clock_gettime(CLOCK_MONOTONIC, &now) != 0) return 0;
  return (uint64_t)now.tv_sec * 1000 + (uint64_t)now.tv_nsec / 1000000;
}

/** @brief Parse a decimal build-profile limit.
 * @param text Null-terminated numeric setting.
 * @param minimum Inclusive lower bound.
 * @param maximum Inclusive upper bound.
 * @return Parsed value, or zero on syntax or range error.
 * @details Limit callers use positive ranges so zero is an unambiguous
 * failure sentinel. Reject signs and suffixes rather than accepting a prefix. */
static unsigned long chdk_number(const char *text, unsigned long minimum, unsigned long maximum)
{
  char *end;
  errno = 0;
  unsigned long value = strtoul(text, &end, 10);
  return errno || *text < '0' || *text > '9' || *end || value < minimum || value > maximum ? 0 : value;
}

/** @brief Poll the owned controller for exit while collecting its reply.
 * @param[out] status Wait status; valid when the result is one.
 * @param timeout_ms Polling budget; zero checks without sleeping.
 * @param cancelable True to honor the service cancellation flag.
 * @return One if reaped, zero on timeout or cancellation, or -1 on wait/setup error.
 * @details Drain during waits and again on exit; the last reply can arrive
 * between the initial read and the successful waitpid. */
static int wait_camera(int *status, unsigned timeout_ms, bool cancelable)
{
  const uint64_t started = chdk_monotonic_ms();
  if (!started) return -1;
  for (;;) {
    drain_control_reply();
    pid_t result = waitpid(camera_pid, status, WNOHANG);
    if (result == camera_pid) { drain_control_reply(); camera_pid = -1; return 1; }
    if (result < 0 && errno == ECHILD) { camera_pid = -1; return -1; }
    if (result < 0 && errno != EINTR) return -1;
    if (cancelable && !capture_running(camera_running)) return 0;
    uint64_t now = chdk_monotonic_ms();
    if (!now || now - started >= timeout_ms) return 0;
    struct timespec pause = {.tv_sec = 0, .tv_nsec = 10000000};
    nanosleep(&pause, NULL);
  }
}

/** @brief Stop the owned controller group within bounded grace periods.
 * @details Allow graceful PTP cleanup before forced termination. Preserve an
 * unreaped PID so future commands cannot acquire the camera concurrently. */
static void stop_camera(void)
{
  if (camera_pid <= 0) return;
  int status = 0;
  if (wait_camera(&status, 0, false) == 1 || camera_pid <= 0) return;
  kill(-camera_pid, SIGTERM);
  /* The controller closes its PTP session on SIGTERM; SIGKILL would leave camera state behind. */
  if (wait_camera(&status, 3000, false) == 1 || camera_pid <= 0) return;
  kill(-camera_pid, SIGKILL);
  partial_cleanup_pending = true;
  if (wait_camera(&status, 1500, false) != 1 && camera_pid > 0)
    fprintf(stderr, "CHDK: killed controller has not exited; retaining process ownership\n");
}

void chdk_pipe_deinit(void)
{
  camera_ready = false;
  stop_camera();
}

/** @brief Run one CHDKbridge capture or lens operation.
 * @param destination Private capture path; unused for a control operation.
 * @param command Zero for capture, otherwise the validated control command.
 * @param value Control operand, unused for capture and lens retraction.
 * @return Zero for successful child execution, or -1 on setup, cancellation or child failure.
 * @details Validate build-profile limits and acquire exclusive child
 * ownership before spawning. Lens operations use a distinct reply pipe;
 * child exit alone does not establish that the camera applied the command. */
static int run_camera(const char *destination, uint8_t command, uint8_t value)
{
  if ((!destination && !command) || !capture_running(camera_running)) return -1;
  if (camera_pid > 0) {
    int status = 0;
    wait_camera(&status, 0, false);
    if (camera_pid > 0) return -1;
  }
  unsigned long timeout = chdk_number(USHER_CHDK_TIMEOUT, 1, 3600);
  if (!timeout || !chdk_number(USHER_CHDK_USB_TIMEOUT, 50, 10000) ||
      !chdk_number(USHER_CHDK_MAX_DOWNLOAD_MIB, 1, 256) ||
      (USHER_CHDK_DIRECT[0] != '0' && USHER_CHDK_DIRECT[0] != '1') || USHER_CHDK_DIRECT[1] ||
      (USHER_CHDK_KEEP_AWAKE[0] && ((USHER_CHDK_KEEP_AWAKE[0] != '0' && USHER_CHDK_KEEP_AWAKE[0] != '1') ||
                                    USHER_CHDK_KEEP_AWAKE[1])) ||
      (USHER_CHDK_PORT[0] && (USHER_CHDK_BUS[0] || USHER_CHDK_DEVICE[0])) ||
      ((USHER_CHDK_BUS[0] == '\0') != (USHER_CHDK_DEVICE[0] == '\0'))) {
    fprintf(stderr, "CHDK: invalid timeout, size limit or conflicting USB selection in build profile\n");
    return -1;
  }
  char *arguments[64];
  char timeout_text[12], value_text[4];
  if (command && timeout > 60) timeout = 60;
  if (snprintf(timeout_text, sizeof(timeout_text), "%lu", timeout) < 1
      || snprintf(value_text, sizeof(value_text), "%u", value) < 1) return -1;
  size_t count = 0;
  arguments[count++] = (char *)USHER_CHDK_COMMAND;
  arguments[count++] = "--timeout";
  arguments[count++] = timeout_text;
  arguments[count++] = "--usb-timeout";
  arguments[count++] = (char *)USHER_CHDK_USB_TIMEOUT;
  arguments[count++] = "--max-download-mib";
  arguments[count++] = (char *)USHER_CHDK_MAX_DOWNLOAD_MIB;
  if (USHER_CHDK_PORT[0]) {
    arguments[count++] = "--port";
    arguments[count++] = (char *)USHER_CHDK_PORT;
  }
  if (USHER_CHDK_BUS[0]) {
    arguments[count++] = "--bus";
    arguments[count++] = (char *)USHER_CHDK_BUS;
    arguments[count++] = "--device";
    arguments[count++] = (char *)USHER_CHDK_DEVICE;
  }
  static const char *const options[][2] = {
    {"--iso-mode", USHER_CHDK_ISO_MODE}, {"--flash", USHER_CHDK_FLASH},
    {"--resolution", USHER_CHDK_RESOLUTION}, {"--quality", USHER_CHDK_QUALITY},
    {"--date-stamp", USHER_CHDK_DATE_STAMP}, {"--af-assist", USHER_CHDK_AF_ASSIST},
    {"--raw", USHER_CHDK_RAW},
    {"--focus-mode", USHER_CHDK_FOCUS_MODE}, {"--focus-distance", USHER_CHDK_FOCUS_DISTANCE},
    {"--focus-timeout", USHER_CHDK_FOCUS_TIMEOUT}, {"--focus-tolerance", USHER_CHDK_FOCUS_TOLERANCE},
    {"--shutter", USHER_CHDK_SHUTTER}, {"--tv96", USHER_CHDK_TV96},
    {"--iso-real", USHER_CHDK_ISO_REAL}, {"--iso-market", USHER_CHDK_ISO_MARKET}, {"--sv96", USHER_CHDK_SV96},
    {"--aperture", USHER_CHDK_APERTURE}, {"--av96", USHER_CHDK_AV96}, {"--nd", USHER_CHDK_ND},
    {"--exposure-timeout", USHER_CHDK_EXPOSURE_TIMEOUT}, {"--exposure-tolerance", USHER_CHDK_EXPOSURE_TOLERANCE},
    {"--keep-awake", USHER_CHDK_KEEP_AWAKE}
  };
  for (size_t index = 0; index < sizeof(options) / sizeof(options[0]); ++index) {
    if (command) break;
    if (!options[index][1][0]) continue;
    if (index == 0 && (USHER_CHDK_ISO_REAL[0] || USHER_CHDK_ISO_MARKET[0] || USHER_CHDK_SV96[0])) continue;
    if (count + 2 >= sizeof(arguments) / sizeof(arguments[0])) return -1;
    arguments[count++] = (char *)options[index][0];
    arguments[count++] = (char *)options[index][1];
  }
  if (command) {
    arguments[count++] = "--control-result-fd";
    arguments[count++] = "3";
    arguments[count++] = command == CAM_CMD_ZOOM_ABSOLUTE ? "zoom" : "--retract-lens";
    if (command == CAM_CMD_ZOOM_ABSOLUTE) arguments[count++] = value_text;
  } else {
    if (USHER_CHDK_DIRECT[0] == '1') arguments[count++] = "--direct-download";
    arguments[count++] = (char *)destination;
  }
  arguments[count] = NULL;
  posix_spawnattr_t attributes;
  int error = posix_spawnattr_init(&attributes);
  if (error) return -1;
  sigset_t mask, defaults;
  sigemptyset(&mask);
  sigemptyset(&defaults);
  sigaddset(&defaults, SIGINT);
  sigaddset(&defaults, SIGTERM);
  error = posix_spawnattr_setsigmask(&attributes, &mask);
  if (!error) error = posix_spawnattr_setsigdefault(&attributes, &defaults);
  if (!error) error = posix_spawnattr_setpgroup(&attributes, 0);
  if (!error) error = posix_spawnattr_setflags(&attributes,
    POSIX_SPAWN_SETSIGMASK | POSIX_SPAWN_SETSIGDEF | POSIX_SPAWN_SETPGROUP);
  posix_spawn_file_actions_t actions;
  bool actions_ready = false;
  int descriptors[2] = {-1, -1};
  if (!error && command) {
    if (pipe(descriptors) != 0) error = errno;
    if (!error && (fcntl(descriptors[0], F_SETFD, FD_CLOEXEC) < 0
                   || fcntl(descriptors[1], F_SETFD, FD_CLOEXEC) < 0
                   || fcntl(descriptors[0], F_SETFL, O_NONBLOCK) < 0)) error = errno;
    if (!error) { error = posix_spawn_file_actions_init(&actions); actions_ready = error == 0; }
    if (!error) error = posix_spawn_file_actions_adddup2(&actions, descriptors[1], 3);
    if (!error && descriptors[0] != 3) error = posix_spawn_file_actions_addclose(&actions, descriptors[0]);
    if (!error && descriptors[1] != 3) error = posix_spawn_file_actions_addclose(&actions, descriptors[1]);
  }
  if (!error) error = posix_spawn(&camera_pid, USHER_CHDK_COMMAND, actions_ready ? &actions : NULL,
                                 &attributes, arguments, environ);
  if (actions_ready) posix_spawn_file_actions_destroy(&actions);
  posix_spawnattr_destroy(&attributes);
  if (descriptors[1] >= 0) close(descriptors[1]);
  if (error) {
    if (descriptors[0] >= 0) close(descriptors[0]);
    camera_pid = -1;
    fprintf(stderr, "CHDK: cannot start %s: %s\n", USHER_CHDK_COMMAND, strerror(error));
    return -1;
  }
  camera_reply.descriptor = descriptors[0];
  camera_reply.size = 0;
  camera_reply.invalid = false;
  errno = 0;
  if (setpriority(PRIO_PROCESS, (id_t)camera_pid, 10) != 0 && errno != ESRCH)
    fprintf(stderr, "CHDK: cannot lower controller scheduling priority: %s\n", strerror(errno));
  int status = 0;
  int result = wait_camera(&status, (unsigned)timeout * 1000 + 2000, true);
  if (camera_reply.descriptor >= 0) close(camera_reply.descriptor);
  camera_reply.descriptor = -1;
  if (result != 1) {
    fprintf(stderr, "CHDK: operation cancelled or deadline exceeded; no automatic capture retry\n");
    stop_camera();
    return -1;
  }
  if (!WIFEXITED(status) || WEXITSTATUS(status) != 0) {
    if (WIFSIGNALED(status)) fprintf(stderr, "CHDK: controller stopped by signal %d; no capture replay\n", WTERMSIG(status));
    else fprintf(stderr, "CHDK: controller exited with code %d; camera state may have changed\n", WIFEXITED(status) ? WEXITSTATUS(status) : -1);
    return -1;
  }
  return 0;
}

int chdk_pipe_init(void)
{
  chdk_pipe_deinit();
  if (camera_pid > 0 || !capture_running(camera_running)) return -1;
  if (access(USHER_CHDK_COMMAND, X_OK) != 0) {
    fprintf(stderr, "CHDK: controller unavailable at %s: %s\n", USHER_CHDK_COMMAND, strerror(errno));
    return -1;
  }
  camera_ready = true;
  return 0;
}

/** @brief True for the controller's private mkstemp name, e.g. c000123.jpg.part.Ab12Cd. */
static bool partial_output_name(const char *name)
{
  static const char middle[] = ".jpg.part.";
  if (name[0] != 'c') return false;
  for (size_t index = 1; index <= 6; ++index)
    if (name[index] < '0' || name[index] > '9') return false;
  for (size_t index = 0; index < sizeof(middle) - 1; ++index)
    if (name[7 + index] != middle[index]) return false;
  for (size_t index = 17; index < 23; ++index)
    if (!name[index] || name[index] == '/') return false;
  return name[23] == '\0';
}

/** @brief Delete partial outputs left by a killed or powered-off controller; none is running. */
static void remove_partial_outputs(const char *destination)
{
  char directory[4096];
  size_t separator = 0, length = 0;
  bool found = false;
  for (; destination[length]; ++length) {
    if (length + 1 >= sizeof(directory)) return;
    directory[length] = destination[length];
    if (destination[length] == '/') { separator = length; found = true; }
  }
  if (found) directory[separator ? separator : 1] = '\0';
  else { directory[0] = '.'; directory[1] = '\0'; }
  DIR *listing = opendir(directory);
  if (!listing) return;
  unsigned removed = 0;
  struct dirent *entry;
  while ((entry = readdir(listing))) {
    struct stat status;
    if (!partial_output_name(entry->d_name) ||
        fstatat(dirfd(listing), entry->d_name, &status, AT_SYMLINK_NOFOLLOW) != 0 ||
        !S_ISREG(status.st_mode) || unlinkat(dirfd(listing), entry->d_name, 0) != 0) continue;
    ++removed;
  }
  closedir(listing);
  if (removed) fprintf(stderr, "CHDK: removed %u partial download(s) left by an interrupted controller\n", removed);
}

int chdk_pipe_shoot(const char *filename)
{
  if (!camera_ready || !filename || !filename[0] || filename[0] == '-') return -1;
  if (partial_cleanup_pending && camera_pid <= 0) {
    remove_partial_outputs(filename);
    partial_cleanup_pending = false;
  }
  struct stat status;
  if (lstat(filename, &status) == 0 || errno != ENOENT) return -1;
  if (run_camera(filename, 0, 0) != 0) return -1;
  int descriptor = open(filename, O_RDONLY | O_NOFOLLOW | O_NONBLOCK | O_CLOEXEC);
  if (descriptor < 0) {
    fprintf(stderr, "CHDK: controller did not publish an accessible JPEG: %s\n", strerror(errno));
    return -1;
  }
  unsigned char first[2], last[2];
  uint64_t size_limit = chdk_number(USHER_CHDK_MAX_DOWNLOAD_MIB, 1, 256) * UINT64_C(1048576);
  bool valid = size_limit && fstat(descriptor, &status) == 0 && S_ISREG(status.st_mode) && status.st_size >= 4 &&
    (uint64_t)status.st_size <= size_limit &&
    pread(descriptor, first, sizeof(first), 0) == (ssize_t)sizeof(first) &&
    pread(descriptor, last, sizeof(last), status.st_size - 2) == (ssize_t)sizeof(last) &&
    first[0] == 0xff && first[1] == 0xd8 && last[0] == 0xff && last[1] == 0xd9;
  if (close(descriptor) != 0) valid = false;
  if (!valid) {
    fprintf(stderr, "CHDK: controller returned no valid saved JPEG\n");
  }
  return valid ? 0 : -1;
}

/** @brief Apply one supported lens command and qualify device evidence.
 * @param command Absolute zoom or lens retraction.
 * @param value Requested zoom value; must be zero for retraction.
 * @param[out] observed Returned device value; meaningful with applied status.
 * @param[out] evidence Returned evidence kind; meaningful with applied status.
 * @return A CAM_CONTROL status; UNKNOWN means execution could not be confirmed.
 * @details Check both the reply shape and echoed command. Do not retry
 * an ambiguous operation or infer success from the controller's exit alone. */
uint8_t chdk_pipe_control(uint8_t command, uint8_t value, uint8_t *observed, uint8_t *evidence)
{
  if (command != CAM_CMD_ZOOM_ABSOLUTE && command != CAM_CMD_LENS_RETRACT) return CAM_CONTROL_UNSUPPORTED;
  if (command == CAM_CMD_LENS_RETRACT && value) return CAM_CONTROL_INVALID_VALUE;
  if (!observed || !evidence || !camera_ready || !capture_running(camera_running)) return CAM_CONTROL_FAILED;
  if (camera_pid > 0) {
    int status = 0;
    wait_camera(&status, 0, false);
    if (camera_pid > 0) return CAM_CONTROL_BUSY;
  }
  *observed = 0;
  *evidence = CAM_CONTROL_EVIDENCE_NONE;
  if (run_camera(NULL, command, value) != 0 || camera_reply.invalid
      || camera_reply.size != CHDK_CONTROL_REPLY_SIZE || camera_reply.bytes[0] != CHDK_CONTROL_REPLY_TAG
      || camera_reply.bytes[1] != command
      || camera_reply.bytes[3] != (command == CAM_CMD_LENS_RETRACT ? CAM_CONTROL_EVIDENCE_PLAYBACK : CAM_CONTROL_EVIDENCE_VALUE)
      || (command == CAM_CMD_LENS_RETRACT && camera_reply.bytes[2])) return CAM_CONTROL_UNKNOWN;
  *observed = camera_reply.bytes[2];
  *evidence = camera_reply.bytes[3];
  return CAM_CONTROL_APPLIED;
}
