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
 * @file loom_cam.c
 * @brief Fresh-first saved-image requests to the standalone loomcam process.
 * @details The AI camera remains owned by its existing driver. This adapter owns
 * only paths, one pending request, one joinable worker and one completion mailbox.
 * It never sends UART or mutates flight waypoints. Return vectors are validated,
 * but archived reference photographs are not yet compared against live images.
 */
#include "loom_cam.h"
#include <dirent.h>
#include "inference_settings.h"
#include "../modules/loom_cam/loom_return.h"
#include <math.h>
#include "path_utils.h"

#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <limits.h>
#include <poll.h>
#include <pthread.h>
#include <signal.h>
#include <spawn.h>
#include <stdatomic.h>
#include <stdio.h>
#include <string.h>
#include <sys/wait.h>
#include <sys/eventfd.h>
#include <time.h>
#include <unistd.h>

#ifndef USHER_LOOMCAM_COMMAND
  /** @brief Saved-photo child executable; not another camera-acquisition process. */
  #define USHER_LOOMCAM_COMMAND "./bin/loomcam"
#endif
/** @name Bounded child-process policy
 * One monotonic deadline bounds reply handling independently of fragmentation
 * and interrupted polls. A short exit grace permits a flushed reply before exit.
 * @{ */
#define LOOM_RESULT_LINE_BYTES 384U /**< Fixed result storage bounds a child's reply. */
#define LOOM_SEQUENCE_TEXT_BYTES 16U /**< Decimal uint32 ID plus terminator, without truncation. */
#define LOOM_WORKER_POLL_MS 50 /**< Watchdog poll interval; shutdown is checked between polls. */
#define LOOM_WORKER_TIMEOUT_MS 5000U /**< Whole reply operation budget in monotonic milliseconds. */
#define LOOM_EXIT_GRACE_MS 200U /**< Allow a flushed reply to precede normal child exit. */
#define LOOM_KILL_GRACE_MS 1000U /**< Maximum wait for a killed owned child. */
#define LOOM_NS_PER_MS 1000000L /**< Convert the poll interval for nanosleep. */
#define LOOM_REFERENCE_PHOTO_LIMIT 256U /**< Bound hard links retained for one sortie. */
#define LOOM_REFERENCE_SESSION_LIMIT 64U /**< Bound retained sortie directories without pruning evidence. */
/** @} */

/** @name Saved-photo and wire-unit conventions
 * Geographic limits are degrees multiplied by 1e7; quality is in thousandths.
 * The reference filename uses ten digits to cover every uint32_t photo number.
 * @{ */
#define LOOM_LATITUDE_LIMIT_E7 INT32_C(900000000) /**< Latitude range in degrees times 1e7. */
#define LOOM_LONGITUDE_LIMIT_E7 INT32_C(1800000000) /**< Longitude range in the same wire units. */
#define LOOM_QUALITY_MAX INT32_C(1000) /**< Quality uses thousandths, not an arbitrary float score. */
#define LOOM_REFERENCE_SEQUENCE_DIGITS 10 /**< Fits every uint32 photo number in the archive basename. */
#define LOOM_JPEG_EXTENSION ".jpg" /**< Final suffix replaced when associating saved sidecars. */
#define LOOM_JPEG_EXTENSION_BYTES (sizeof(LOOM_JPEG_EXTENSION) - 1U) /**< Exclude the string terminator. */
#define LOOM_DECIMAL_RADIX 10 /**< Reply grammar accepts explicit decimal integers only. */
/** @} */

/** @brief Ordered fields of the strict LOOM_POINT_V1 text contract. */
enum loom_result_field {
  LOOM_FIELD_SEQUENCE,
  LOOM_FIELD_VALID,
  LOOM_FIELD_LATITUDE,
  LOOM_FIELD_LONGITUDE,
  LOOM_FIELD_ALTITUDE_MM,
  LOOM_FIELD_AGL_MM,
  LOOM_FIELD_QUALITY,
  LOOM_FIELD_METRIC,
  LOOM_RESULT_FIELD_COUNT
};

/** @brief The validity field accepts only these two numeric wire values. */
enum loom_wire_validity { LOOM_WIRE_INVALID = 0, LOOM_WIRE_VALID = 1 };
/** @brief POSIX pipe endpoints; names distinguish child output from parent input. */
enum loom_pipe_endpoint { LOOM_PIPE_READ, LOOM_PIPE_WRITE, LOOM_PIPE_ENDPOINT_COUNT };

extern char **environ; /**< Borrowed POSIX environment passed to the explicit-argv child. */
/* The mutex protects paths, phase, admission and the completion mailbox.
 * Atomic lifecycle flags additionally let the child watchdog and enabled()
 * observe shutdown without holding that mutex through blocking process I/O. */
static pthread_mutex_t loom_mutex = PTHREAD_MUTEX_INITIALIZER; /**< Protects admission, phase and mailbox state. */
static pthread_cond_t loom_available = PTHREAD_COND_INITIALIZER; /**< Wakes the worker for pending work or shutdown. */
static pthread_t loom_thread; /**< Joinable saved-photo worker; no sensor ownership. */
static atomic_bool loom_stopping; /**< Watchdog-visible shutdown flag outside slow I/O. */
static atomic_bool loom_running; /**< Lifecycle availability queried without taking the mutex. */
/** @var loom_pending
 * @brief The latest copied path awaits worker consumption; older pending work is replaceable. */
static bool loom_pending, loom_ready; /**< One completion mailbox is available for consumption. */
static uint32_t loom_latest_sequence; /**< Highest admitted FC photo number; rejects duplicates/reversed work. */
/** @var loom_session
 * @brief START token used to admit the corresponding STOP, not a photo number. */
static uint32_t loom_session, loom_phase; /**< Generation that invalidates results crossing a phase change. */
static bool loom_recording; /**< Outbound reference collection rather than return matching. */
/** @var loom_filename
 * @brief Latest admitted immutable saved-photo path, copied into bounded storage. */
/** @var loom_references
 * @brief Reference-directory configuration, fixed until the worker has been joined. */
static char loom_filename[PATH_MAX], loom_references[PATH_MAX], loom_session_references[PATH_MAX];
static char loom_model[PATH_MAX]; /**< Owned model/configuration paths. */
static char loom_settings[PATH_MAX]; /**< Selected assignment, immutable during worker lifetime. */
static struct loom_return_options loom_return_options;
static struct loom_return_state loom_return_state;
static struct camera_point_result loom_result; /**< Single correlated matched/no-match completion. */
static pid_t loom_matcher_pid = -1;
static int loom_notification = -1;

/** @brief Clear the completion event counter.
 * @details The mailbox owns the result; the event counter only indicates
 * that the event loop should inspect it under the stream's synchronization. */
static void drain_notification(void)
{
  if (loom_notification < 0) return;
  eventfd_t count;
  int received;
  do { received = eventfd_read(loom_notification, &count); } while (received < 0 && errno == EINTR);
}

/** @brief Wake the event loop when completion state may have changed.
 * @details The nonblocking event descriptor is not a result queue. A
 * saturated counter already has the desired readable state. */
static void notify_completion(void)
{
  if (loom_notification < 0) return;
  const uint64_t count = 1;
  ssize_t written;
  do { written = write(loom_notification, &count, sizeof(count)); } while (written < 0 && errno == EINTR);
}

/**
 * @brief Copy a complete NUL-terminated path into owned, fixed-size storage.
 * @param destination Writable buffer of @p capacity bytes.
 * @param capacity Buffer size, including space for the terminator.
 * @param source Borrowed path; NULL selects an empty optional configuration.
 * @return True on complete copy; false rather than silently truncating a path.
 */
static bool copy_path(char *destination, size_t capacity, const char *source)
{
  if (!source) source = "";
  const size_t length = strnlen(source, capacity);
  if (length >= capacity) return false;
  for (size_t index = 0; index <= length; ++index) destination[index] = source[index];
  return true;
}

/**
 * @brief Consume one exact field prefix followed by a signed decimal integer.
 * @param[in,out] cursor Current parse position; advanced only after conversion.
 * @param key Exact prefix, including the separator between fields.
 * @param[out] value Parsed wide integer, before wire-range validation.
 * @return False for missing digits, prefix mismatch or integer overflow.
 * @details Reject leading whitespace and a plus sign instead of allowing
 * strtoimax() to quietly accept a different producer grammar.
 */
static bool read_result_field(const char **cursor, const char *key, intmax_t *value)
{
  const size_t length = strlen(key);
  if (strncmp(*cursor, key, length) != 0) return false;
  const char *number = *cursor + length;
  const char *digit = number[0] == '-' ? number + 1 : number;
  if (*digit < '0' || *digit > '9') return false;
  char *end;
  errno = 0;
  *value = strtoimax(number, &end, LOOM_DECIMAL_RADIX);
  if (errno == ERANGE || end == number) return false;
  *cursor = end;
  return true;
}

/**
 * @brief Validate and correlate one complete reply before publishing any fields.
 * @param line NUL-terminated reply; only one optional final newline is allowed.
 * @param sequence Photo number that the child must echo exactly.
 * @param[out] result Destination, changed only after the whole reply is accepted.
 * @return True for a well-formed matched or no-match reply.
 * @details A valid point must fit geographic and quality limits. An invalid
 * reply may retain int32 diagnostics, but cannot authorize navigation. FC-side
 * freshness, request eligibility and waypoint guards remain independent gates.
 */
static bool parse_result(const char *line, uint32_t sequence, struct camera_point_result *result)
{
  if (!line || !result || !sequence) return false;
  static const char *const keys[LOOM_RESULT_FIELD_COUNT] = {
    "LOOM_POINT_V1 sequence=", " valid=", " lat=", " lon=", " alt_mm=", " agl_mm=", " quality=", " metric="
  };
  intmax_t values[LOOM_RESULT_FIELD_COUNT];
  const char *cursor = line;
  for (size_t index = 0; index < LOOM_RESULT_FIELD_COUNT; ++index) {
    if (!read_result_field(&cursor, keys[index], &values[index])) return false;
    if (index >= LOOM_FIELD_LATITUDE && (values[index] < INT32_MIN || values[index] > INT32_MAX)) return false;
  }
  if (values[LOOM_FIELD_SEQUENCE] != sequence
      || (values[LOOM_FIELD_VALID] != LOOM_WIRE_INVALID && values[LOOM_FIELD_VALID] != LOOM_WIRE_VALID)
      || (*cursor != '\0' && !(*cursor == '\n' && cursor[1] == '\0'))) return false;
  struct camera_point_result point = {
    .session_id = sequence, .valid = values[LOOM_FIELD_VALID] == LOOM_WIRE_VALID,
    .lat = (int32_t)values[LOOM_FIELD_LATITUDE], .lon = (int32_t)values[LOOM_FIELD_LONGITUDE],
    .alt_mm = (int32_t)values[LOOM_FIELD_ALTITUDE_MM], .agl_mm = (int32_t)values[LOOM_FIELD_AGL_MM],
    .quality = (int32_t)values[LOOM_FIELD_QUALITY], .metric = (int32_t)values[LOOM_FIELD_METRIC]
  };
  if (point.valid && (point.lat < -LOOM_LATITUDE_LIMIT_E7 || point.lat > LOOM_LATITUDE_LIMIT_E7
                      || point.lon < -LOOM_LONGITUDE_LIMIT_E7 || point.lon > LOOM_LONGITUDE_LIMIT_E7
                      || (!point.lat && !point.lon) || point.agl_mm < 0 || point.quality < 0
                      || point.quality > LOOM_QUALITY_MAX)) return false;
  *result = point;
  return true;
}

/**
 * @brief Read the watchdog clock in milliseconds.
 * @return Monotonic milliseconds, or zero if the clock cannot be read.
 * @details Wall-clock corrections must not extend or shorten a matcher deadline.
 */
static uint64_t loom_monotonic_ms(void)
{
  struct timespec now;
  if (clock_gettime(CLOCK_MONOTONIC, &now) != 0 || now.tv_sec < 0) return 0;
  return (uint64_t)now.tv_sec * 1000U + (uint64_t)now.tv_nsec / LOOM_NS_PER_MS;
}

/**
 * @brief Wait up to a bounded time for the owned matcher to exit.
 * @param timeout_ms Polling budget in milliseconds.
 * @param[out] status Optional wait status, set when ownership is cleared.
 * @return True if no matcher remains owned; false on timeout or a wait error.
 * @details Use nonblocking waits to preserve the deadline. On failure retain
 * the PID so a later cleanup attempt does not lose ownership.
 */
static bool reap_matcher(unsigned timeout_ms, int *status)
{
  if (loom_matcher_pid <= 0) return true;
  const uint64_t started = loom_monotonic_ms();
  for (;;) {
    int observed = -1;
    const pid_t result = waitpid(loom_matcher_pid, &observed, WNOHANG);
    if (result == loom_matcher_pid || (result < 0 && errno == ECHILD)) {
      loom_matcher_pid = -1;
      if (status) *status = observed;
      return true;
    }
    if (result < 0 && errno != EINTR) return false;
    const uint64_t now = loom_monotonic_ms();
    if (!started || !now || now < started || now - started >= timeout_ms) return false;
    if (poll(NULL, 0, 10) < 0 && errno != EINTR) return false;
  }
}

/**
 * @brief Check whether an in-flight match still belongs to the active phase.
 * @param phase Generation saved when the photo was admitted.
 * @return True if that generation can still produce a result.
 * @details Consult shared phase state under the mutex; photo numbers alone
 * cannot distinguish work across phase changes.
 */
static bool matcher_phase_current(uint32_t phase)
{
  pthread_mutex_lock(&loom_mutex);
  const bool current = !loom_stopping && phase == loom_phase;
  pthread_mutex_unlock(&loom_mutex);
  return current;
}

/**
 * @brief Run the saved-photo matcher with bounded output and a watchdog.
 * @param filename Immutable saved JPEG to process, not another capture request.
 * @param references Reference archive for this phase.
 * @param sequence Photo number that the matcher must echo.
 * @param phase Generation used to cancel superseded work.
 * @param[out] result Accepted reply; unchanged on execution or parsing failure.
 * @return True only for a well-formed current reply and a successful child exit.
 * @details Pass arguments directly and collect the reply on a private stdout
 * pipe. Monitor the deadline and phase while reading so slow or superseded
 * work cannot indefinitely hold the worker.
 */
static bool run_matcher(const char *filename, const char *references, uint32_t sequence,
                        struct camera_point_result *result, uint32_t phase)
{
  if (!filename || !result || !sequence || !reap_matcher(0, NULL)) return false;
  const uint64_t started = loom_monotonic_ms();
  if (!started) return false;
  int output[LOOM_PIPE_ENDPOINT_COUNT];
  if (pipe(output) != 0) return false;
  if (fcntl(output[LOOM_PIPE_READ], F_SETFD, FD_CLOEXEC) != 0
      || fcntl(output[LOOM_PIPE_WRITE], F_SETFD, FD_CLOEXEC) != 0) {
    close(output[LOOM_PIPE_READ]); close(output[LOOM_PIPE_WRITE]); return false;
  }
  posix_spawn_file_actions_t actions;
  if (posix_spawn_file_actions_init(&actions) != 0) {
    close(output[LOOM_PIPE_READ]); close(output[LOOM_PIPE_WRITE]); return false;
  }
  int setup = posix_spawn_file_actions_addopen(&actions, STDIN_FILENO, "/dev/null", O_RDONLY, 0);
  if (!setup) setup = posix_spawn_file_actions_adddup2(&actions, output[LOOM_PIPE_WRITE], STDOUT_FILENO);
  DIR *descriptors = opendir("/proc/self/fd");
  if (!descriptors) setup = errno ? errno : EIO;
  struct dirent *entry;
  while (descriptors && !setup && (entry = readdir(descriptors)) != NULL) {
    char *end = NULL;
    const long descriptor = strtol(entry->d_name, &end, 10);
    if (!entry->d_name[0] || !end || *end || descriptor <= STDERR_FILENO || descriptor > INT_MAX ||
        descriptor == dirfd(descriptors)) continue;
    setup = posix_spawn_file_actions_addclose(&actions, (int)descriptor);
  }
  if (descriptors) closedir(descriptors);
  char sequence_text[LOOM_SEQUENCE_TEXT_BYTES];
  snprintf(sequence_text, sizeof(sequence_text), "%" PRIu32, sequence);
  char *arguments[] = {(char *)USHER_LOOMCAM_COMMAND, "--image", (char *)filename,
                       "--sequence", sequence_text, "--reference-dir", (char *)references,
                       loom_settings[0] ? "--settings" : "--model", loom_settings[0] ? loom_settings : loom_model, NULL
                      };
  posix_spawnattr_t attributes;
  bool attributes_ready = false;
  if (!setup) { setup = posix_spawnattr_init(&attributes); attributes_ready = setup == 0; }
  sigset_t mask, defaults;
  sigemptyset(&mask);
  sigemptyset(&defaults);
  sigaddset(&defaults, SIGINT);
  sigaddset(&defaults, SIGTERM);
  if (!setup) setup = posix_spawnattr_setsigmask(&attributes, &mask);
  if (!setup) setup = posix_spawnattr_setsigdefault(&attributes, &defaults);
  if (!setup) setup = posix_spawnattr_setpgroup(&attributes, 0);
  if (!setup) setup = posix_spawnattr_setflags(&attributes,
    POSIX_SPAWN_SETSIGMASK | POSIX_SPAWN_SETSIGDEF | POSIX_SPAWN_SETPGROUP);
  const int spawned = setup ? setup : posix_spawn(&loom_matcher_pid, USHER_LOOMCAM_COMMAND,
                                                &actions, &attributes, arguments, environ);
  if (attributes_ready) posix_spawnattr_destroy(&attributes);
  posix_spawn_file_actions_destroy(&actions);
  close(output[LOOM_PIPE_WRITE]);
  if (spawned) { close(output[LOOM_PIPE_READ]); return false; }
  char line[LOOM_RESULT_LINE_BYTES];
  size_t length = 0;
  bool complete = false;
  while (matcher_phase_current(phase)) {
    const uint64_t now = loom_monotonic_ms();
    if (!now || now < started || now - started >= LOOM_WORKER_TIMEOUT_MS) break;
    const uint64_t remaining = LOOM_WORKER_TIMEOUT_MS - (now - started);
    const int wait_ms = remaining < LOOM_WORKER_POLL_MS ? (int)remaining : LOOM_WORKER_POLL_MS;
    struct pollfd descriptor = {.fd = output[LOOM_PIPE_READ], .events = POLLIN};
    const int ready = poll(&descriptor, 1, wait_ms);
    if (ready < 0) { if (errno == EINTR) continue; break; }
    if (!ready) continue;
    const ssize_t count = read(output[LOOM_PIPE_READ], line + length, sizeof(line) - length - 1);
    if (count <= 0) break;
    length += (size_t)count;
    line[length] = '\0';
    if (memchr(line, '\0', length) || memchr(line, '\r', length)) break;
    const char *newline = strchr(line, '\n');
    if (newline) { complete = newline == line + length - 1; break; }
    if (length + 1 == sizeof(line)) break;
  }
  close(output[LOOM_PIPE_READ]);
  int status = -1;
  if (!reap_matcher(complete && !loom_stopping ? LOOM_EXIT_GRACE_MS : 0, &status)) {
    kill(-loom_matcher_pid, SIGKILL);
    kill(loom_matcher_pid, SIGKILL);
    if (!reap_matcher(LOOM_KILL_GRACE_MS, &status))
      fprintf(stderr, "LOOM_cam: killed matcher has not exited; retaining child ownership\n");
    return false;
  }
  return complete && matcher_phase_current(phase) && WIFEXITED(status) && WEXITSTATUS(status) == 0
         && parse_result(line, sequence, result);
}

/**
 * @brief Check whether a session reference directory has room for another entry.
 * @param directory Path of the existing session directory.
 * @return True if the directory could be read and is below its entry limit.
 * @details Count existing directory entries instead of relying on in-memory
 * counters so pre-existing files still count against the archive bound.
 */
static bool reference_directory_available(const char *directory)
{
  DIR *stream = opendir(directory);
  if (!stream) return false;
  unsigned entries = 0;
  errno = 0;
  struct dirent *entry;
  while ((entry = readdir(stream)) != NULL) {
    if (!strcmp(entry->d_name, ".") || !strcmp(entry->d_name, "..")) continue;
    if (++entries > LOOM_REFERENCE_PHOTO_LIMIT) { errno = ENOSPC; break; }
  }
  const bool available = errno == 0 && entries < LOOM_REFERENCE_PHOTO_LIMIT;
  if (closedir(stream) != 0) return false;
  if (!available && !errno) errno = ENOSPC;
  return available;
}

static bool reference_root_available(const char *directory);

/**
 * @brief Create an exclusive archive directory for a new session.
 * @param base Archive root path.
 * @param session_id Identifier formatted into the directory name.
 * @param[out] destination Buffer for the formatted directory path.
 * @param capacity Size of @p destination, including its terminator.
 * @return True only if a new directory was created.
 * @details Apply the root entry limit first and use exclusive directory
 * creation so a reused session ID cannot silently inherit old references.
 */
static bool create_session_reference_directory(const char *base, uint32_t session_id,
                                               char *destination, size_t capacity)
{
  if (!base || !base[0] || usher_ensure_directory(base) != 0 || !reference_root_available(base)) return false;
  const int length = snprintf(destination, capacity, "%s/session-%010" PRIu32, base, session_id);
  if (length < 0 || (size_t)length >= capacity) { errno = ENAMETOOLONG; return false; }
  return mkdir(destination, 0700) == 0;
}

/**
 * @brief Archive a published outbound JPEG without copying its pixels.
 * @param filename Original saved JPEG.
 * @param directory Existing session archive.
 * @param sequence Photo number used in the reference basename.
 * @return True if the new reference was linked.
 * @details A hard link neither copies the image nor replaces an existing
 * reference. The original must remain immutable and on the same filesystem.
 */
static bool record_reference(const char *filename, const char *directory, uint32_t sequence)
{
  char destination[PATH_MAX];
  const int length = snprintf(destination, sizeof(destination), "%s/%c%0*" PRIu32 LOOM_JPEG_EXTENSION,
                              directory, LOOM_IMAGE_PREFIX, LOOM_REFERENCE_SEQUENCE_DIGITS, sequence);
  if (!directory || !directory[0] || length < 0 || (size_t)length >= sizeof(destination)
      || !reference_directory_available(directory) || link(filename, destination) != 0) return false;
  return true;
}

/**
 * @brief Check whether the archive root can admit another session.
 * @param directory Path of the archive root.
 * @return True if the directory could be read and is below its entry limit.
 * @details Stop scanning at the bound; old sessions remain on disk and
 * consume capacity rather than being silently evicted.
 */
static bool reference_root_available(const char *directory)
{
  DIR *stream = opendir(directory);
  if (!stream) return false;
  unsigned entries = 0;
  errno = 0;
  struct dirent *entry;
  while ((entry = readdir(stream)) != NULL) {
    if (!strcmp(entry->d_name, ".") || !strcmp(entry->d_name, "..")) continue;
    if (++entries >= LOOM_REFERENCE_SESSION_LIMIT) { errno = ENOSPC; break; }
  }
  const bool available = errno == 0 && entries < LOOM_REFERENCE_SESSION_LIMIT;
  if (closedir(stream) != 0) return false;
  if (!available && !errno) errno = ENOSPC;
  return available;
}

/**
 * @brief Validate a return candidate in the persistent owner.
 * @param[in,out] result Candidate, updated if the configured filter accepts it.
 * @return True for an accepted candidate or an explicit no-point result.
 * @details Keep ordering state across matcher subprocesses. Apply confidence
 * and reference checks before passing an eligible candidate to the estimator.
 */
static bool filter_return_point(struct camera_point_result *result)
{
  if (!result) return false;
  if (!result->valid) return true;
  if (result->quality < 0 || result->quality > LOOM_QUALITY_MAX ||
      (double)result->quality / LOOM_QUALITY_MAX < loom_return_options.limits.minimum_confidence) return false;
  if (!loom_return_options_valid(&loom_return_options) ||
      loom_return_options.frame == LOOM_RETURN_NONE) return false;
  const unsigned home_fields = (1U << 5) | (1U << 6);
  const bool home_configured = (loom_return_options.seen & home_fields) == home_fields;
  if (!home_configured || result->metric == LOOM_METRIC_BARO_MAGNETO) {
    if (!result->session_id || result->session_id <= loom_return_state.sequence ||
        result->lat < -LOOM_LATITUDE_LIMIT_E7 || result->lat > LOOM_LATITUDE_LIMIT_E7 ||
        result->lon < -LOOM_LONGITUDE_LIMIT_E7 || result->lon > LOOM_LONGITUDE_LIMIT_E7 ||
        (!result->lat && !result->lon)) return false;
    loom_return_state.sequence = result->session_id;
    return true;
  }
  const double radians = 0.017453292519943295, earth = 6378137;
  const double scale = cos(loom_return_options.home_latitude * radians);
  struct loom_return_point candidate = {
    earth * scale * remainder(result->lon / 1e7 - loom_return_options.home_longitude, 360) * radians,
    earth * (result->lat / 1e7 - loom_return_options.home_latitude) * radians
  };
  struct loom_return_state proposed = loom_return_state;
  if (!loom_return_update(&proposed, &loom_return_options.limits, result->session_id,
                          candidate, &candidate)) return false;
  const double latitude = loom_return_options.home_latitude + candidate.north / earth / radians;
  const double longitude = remainder(loom_return_options.home_longitude + candidate.east / (earth * scale) / radians, 360);
  if (!isfinite(latitude) || !isfinite(longitude) || fabs(latitude) > 90 || fabs(longitude) > 180 ||
      (!llround(latitude * 1e7) && !llround(longitude * 1e7))) return false;
  result->lat = (int32_t)llround(latitude * 1e7);
  result->lon = (int32_t)llround(longitude * 1e7);
  loom_return_state = proposed;
  return true;
}

/**
 * @brief Process the newest pending photo and suppress superseded completions.
 * @param unused Required pthread argument; no per-thread heap state is used.
 * @return NULL after shutdown.
 * @details Snapshot the pending work under the mutex and release it before
 * filesystem or child I/O. The phase generation prevents work from a previous
 * phase from publishing into the new one, even if a photo number is reused.
 */
static void *loom_worker(void *unused)
{
  (void)unused;
  for (;;) {
    char filename[PATH_MAX];
    pthread_mutex_lock(&loom_mutex);
    while (!loom_pending && !loom_stopping) pthread_cond_wait(&loom_available, &loom_mutex);
    if (loom_stopping) { pthread_mutex_unlock(&loom_mutex); return NULL; }
    const uint32_t sequence = loom_latest_sequence;
    const uint32_t phase = loom_phase;
    const bool recording = loom_recording;
    char references[PATH_MAX];
    const bool paths_valid = copy_path(filename, sizeof(filename), loom_filename)
      && copy_path(references, sizeof(references), loom_session_references);
    loom_pending = false;
    pthread_mutex_unlock(&loom_mutex);
    /* Both collection and failed matching publish correlated INVALID results.
     * Originals remain the evidence archive; dropped work is never a waypoint. */
    struct camera_point_result result = {.session_id = sequence};
    if (!paths_valid) fprintf(stderr, "LOOM_cam: photo path snapshot unavailable; no waypoint\n");
    else if (recording && !record_reference(filename, references, sequence))
      fprintf(stderr, "LOOM_cam: photo %" PRIu32 " reference unavailable (%s); original retained\n",
              sequence, strerror(errno));
    else if (!recording && !run_matcher(filename, references, sequence, &result, phase))
      fprintf(stderr, "LOOM_cam: photo %" PRIu32 " matcher unavailable/invalid; no waypoint\n", sequence);
    pthread_mutex_lock(&loom_mutex);
    if (!loom_stopping && sequence == loom_latest_sequence && phase == loom_phase) {
      if (!filter_return_point(&result))
        result = (struct camera_point_result){.session_id = sequence};
      loom_result = result; loom_ready = true;
      notify_completion();
    }
    pthread_mutex_unlock(&loom_mutex);
  }
}

int loom_cam_set_settings(const char *settings)
{
  pthread_mutex_lock(&loom_mutex);
  const int result = loom_running || !copy_path(loom_settings, sizeof(loom_settings), settings) ? -1 : 0;
  pthread_mutex_unlock(&loom_mutex);
  return result;
}

int loom_cam_init(const char *references, const char *model)
{
  pthread_mutex_lock(&loom_mutex);
  if (loom_running) { pthread_mutex_unlock(&loom_mutex); return -1; }
  if (!copy_path(loom_references, sizeof(loom_references), references)
      || !copy_path(loom_session_references, sizeof(loom_session_references), references)
      || !copy_path(loom_model, sizeof(loom_model), model)) { pthread_mutex_unlock(&loom_mutex); return -1; }
  loom_return_options_init(&loom_return_options);
  loom_return_state = (struct loom_return_state){0};
  char settings_path[INFERENCE_PATH_CAPACITY], settings_error[1024];
  const char *selected_settings = loom_settings[0] ? loom_settings : NULL;
  if (!selected_settings && inference_settings_find("loomcam", "loom_cam", settings_path, sizeof(settings_path)))
    selected_settings = settings_path;
  if (selected_settings) {
    struct inference_settings inference;
    inference_settings_init(&inference, INFERENCE_AUTO);
    if (!inference_settings_read(&inference, selected_settings, loom_return_setting, &loom_return_options,
                                 settings_error, sizeof(settings_error)) ||
        !loom_return_options_valid(&loom_return_options)) {
      pthread_mutex_unlock(&loom_mutex);
      return -1;
    }
  }
  loom_pending = loom_ready = false;
  loom_latest_sequence = 0;
  loom_session = loom_phase = 0;
  /* Return mode permits bench matching against an existing reference directory
   * before START. START explicitly changes to outbound reference collection. */
  loom_recording = false;
  loom_stopping = false;
  if (!reap_matcher(0, NULL)) { pthread_mutex_unlock(&loom_mutex); return -1; }
  loom_notification = eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
  if (loom_notification < 0) { pthread_mutex_unlock(&loom_mutex); return -1; }
  const int error = pthread_create(&loom_thread, NULL, loom_worker, NULL);
  loom_running = error == 0;
  if (error) { close(loom_notification); loom_notification = -1; }
  pthread_mutex_unlock(&loom_mutex);
  return error ? -1 : 0;
}

bool loom_cam_enabled(void)
{
  return atomic_load(&loom_running) && !atomic_load(&loom_stopping);
}

int loom_cam_completion_fd(void)
{
  pthread_mutex_lock(&loom_mutex);
  const int descriptor = loom_notification;
  pthread_mutex_unlock(&loom_mutex);
  return descriptor;
}

bool loom_cam_start(uint32_t session_id)
{
  char session_references[PATH_MAX];
  pthread_mutex_lock(&loom_mutex);
  const bool eligible = loom_running && !loom_stopping && session_id != 0 && session_id != loom_session;
  pthread_mutex_unlock(&loom_mutex);
  if (!eligible) return false;
  if (!create_session_reference_directory(loom_references, session_id,
                                          session_references, sizeof(session_references))) {
    fprintf(stderr, "LOOM_cam: cannot create references for session %" PRIu32 " (%s)\n",
            session_id, strerror(errno));
    return false;
  }
  pthread_mutex_lock(&loom_mutex);
  const bool accepted = loom_running && !loom_stopping && session_id != loom_session
    && copy_path(loom_session_references, sizeof(loom_session_references), session_references);
  if (accepted) {
    /* The photo number alone cannot distinguish results from an earlier mode.
     * Advance the phase so an in-flight old child cannot fill the new mailbox. */
    loom_session = session_id;
    ++loom_phase;
    loom_recording = true;
    loom_return_state = (struct loom_return_state){0};
    loom_pending = loom_ready = false;
    drain_notification();
  }
  pthread_mutex_unlock(&loom_mutex);
  if (!accepted) rmdir(session_references);
  return accepted;
}

bool loom_cam_stop(uint32_t session_id)
{
  pthread_mutex_lock(&loom_mutex);
  const bool accepted = loom_running && !loom_stopping && session_id && session_id == loom_session;
  if (accepted) {
    /* STOP changes processing policy, not camera ownership or flight waypoints.
     * Matching still needs its own validated result after this transition. */
    ++loom_phase;
    loom_recording = false;
    loom_return_state = (struct loom_return_state){0};
    loom_pending = loom_ready = false;
    drain_notification();
  }
  pthread_mutex_unlock(&loom_mutex);
  return accepted;
}

void loom_cam_image_saved(const char *filename, const union dc_shot_union *shot)
{
  /* Never stall capture completion on matcher contention. Replace older work
   * rather than building a queue that would steer from obsolete photographs. */
  if (!filename || !shot || shot->data.nr <= 0 || pthread_mutex_trylock(&loom_mutex) != 0) return;
  if (loom_running && !loom_stopping && (uint32_t)shot->data.nr > loom_latest_sequence
      && copy_path(loom_filename, sizeof(loom_filename), filename)) {
    loom_latest_sequence = (uint32_t)shot->data.nr;
    loom_pending = true;
    loom_ready = false;
    drain_notification();
    pthread_cond_signal(&loom_available);
  }
  pthread_mutex_unlock(&loom_mutex);
}

bool loom_cam_poll_result(struct camera_point_result *result)
{
  if (!result || pthread_mutex_trylock(&loom_mutex) != 0) return false;
  const bool ready = loom_ready;
  if (ready) { *result = loom_result; loom_ready = false; }
  drain_notification();
  pthread_mutex_unlock(&loom_mutex);
  return ready;
}

void loom_cam_deinit(void)
{
  pthread_mutex_lock(&loom_mutex);
  const bool running = loom_running;
  loom_stopping = true;
  pthread_cond_signal(&loom_available);
  pthread_mutex_unlock(&loom_mutex);
  /* Joining outside the mutex lets the worker observe shutdown and reap its
   * child before configuration storage can be reused by the next init. */
  if (running) pthread_join(loom_thread, NULL);
  if (!reap_matcher(0, NULL)) {
    kill(-loom_matcher_pid, SIGKILL);
    kill(loom_matcher_pid, SIGKILL);
    if (!reap_matcher(LOOM_KILL_GRACE_MS, NULL))
      fprintf(stderr, "LOOM_cam: matcher exit remains unconfirmed\n");
  }
  pthread_mutex_lock(&loom_mutex);
  loom_running = loom_pending = loom_ready = false;
  if (loom_notification >= 0) close(loom_notification);
  loom_notification = -1;
  pthread_mutex_unlock(&loom_mutex);
}