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
 * @file ear_cam_pipe.c
 * @brief USHER-side audio supervision, pose association and immutable EAR snapshots.
 * @details A persistent EARcam child produces newline-delimited acoustic measurements.
 * A reader thread retains only the latest measurement while USHER geotags it at shots;
 * the bounded session can be detached without copying for the backend's finalizer.
 * Numerical fusion and JPEG rendering live in the standalone EAR core, not USHER's
 * serial loop. Missing microphones remain nonfatal and are retried.
 * Thread ownership: the supervisor owns child lifecycle, the reader publishes
 * latest under ear_mutex, the serial owner handles sessions/pending poses/log
 * enqueue, and the log worker alone accesses its FILE. The finalizer borrows
 * detached snapshots until release; it must not touch active-session APIs.
 */
#include "ear_cam_pipe.h"
#include "modules/ear_cam/ear_heatmap.h"
#include "path_utils.h"

#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <math.h>
#include <poll.h>
#include <pthread.h>
#include <signal.h>
#include <spawn.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

#ifndef USHER_EAR_CAM_COMMAND
  #define USHER_EAR_CAM_COMMAND "./bin/earcam"
#endif

#ifndef USHER_EAR_CAM_DEVICE
  #define USHER_EAR_CAM_DEVICE "auto"   // earcam picks the first USB capture card
#endif

#ifndef USHER_EAR_CAM_LOG_DIR
  #define USHER_EAR_CAM_LOG_DIR "earlogs"
#endif

#ifndef USHER_EAR_CAM_PHOTO_DIR
  #define USHER_EAR_CAM_PHOTO_DIR "~/Pictures/sensoraia"
#endif

#ifndef EAR_READY_TIMEOUT_MS
  #define EAR_READY_TIMEOUT_MS 15000
#endif
#ifndef EAR_RESTART_INTERVAL_MS
  #define EAR_RESTART_INTERVAL_MS 3000U
#endif
#define EAR_WARNING_INTERVAL_MS 1000U
#define EAR_SAMPLE_STALE_MS 500U
#define EAR_LINE_MAX 512U
#define ANGLE_BFP_SCALE 4096.0
#define POSITION_BFP_SCALE 256.0
#define EARTH_RADIUS_M 6378137.0
#define DEG_TO_RAD 0.017453292519943295
#define RAD_TO_DEG 57.29577951308232

/** Storage/admission policies; two snapshots match the backend's two finalizer jobs. */
#define EAR_SESSION_BUFFER_COUNT 2
#define EAR_MICROPHONE_NAME_CAPACITY 256
#define EAR_LOG_PATH_CAPACITY 512
#define EAR_LOG_ROW_CAPACITY 512
#define EAR_LOG_QUEUE_ROWS 128
#define EAR_LOG_BATCH_ROWS 16
#define EAR_SESSION_MARKER_CAPACITY 128
#define EAR_PENDING_AUDIO_WINDOWS 16
#define EAR_BAND_ARGUMENT_CAPACITY 32
#define EAR_READER_CHUNK_BYTES 256
#define EAR_READER_POLL_MS 200
#define EAR_POSE_WAIT_MS 200
#define EAR_SAMPLE_FUTURE_SLACK_MS 1
/** The shutdown loop escalates QUIT -> SIGTERM -> SIGKILL; it does not block waitpid. */
#define EAR_CHILD_REAP_ATTEMPTS 25
#define EAR_CHILD_TERM_ATTEMPT 10
#define EAR_CHILD_KILL_ATTEMPT 20
#define EAR_SUPERVISOR_TICK_MS 20
#define EAR_LOG_FILENAME_ATTEMPTS 128
#define EAR_EVIDENCE_FILENAME_ATTEMPTS 32
#define EAR_LOG_FILE_MODE 0644
#define EAR_EVIDENCE_FILE_MODE 0600
#define EAR_TM_YEAR_ORIGIN 1900
#define EAR_JPEG_SUFFIX_LENGTH 4
#define EAR_MS_PER_SECOND 1000U
#define EAR_NS_PER_MS 1000000U
#define EAR_SLEEP_NS_PER_MS 1000000L
#define EAR_US_PER_MS 1000
#define EAR_COORDINATE_UNITS_PER_DEG 1e7
#define EAR_ALTITUDE_UNITS_PER_METER 1000.0
/** Plausibility limits for producer capture windows, not clock synchronization guarantees. */
#define EAR_AUDIO_WINDOW_LIMIT_US 1000000
#define EAR_AUDIO_TIMING_ERROR_LIMIT_US 100000
#define EAR_AUDIO_FUTURE_SLACK_US 1000
#define EAR_SAMPLE_PREFIX_LENGTH 7
#define EAR_READY_PREFIX_LENGTH 12
#define EAR_ERROR_PREFIX_LENGTH 12
#define EAR_STOPPED_PREFIX_LENGTH 14
/** xorshift32 parameters are algorithm invariants; the nonzero seed is process-local. */
#define EAR_SIM_NOISE_INITIAL_STATE 0x2545F491U
#define EAR_SIM_XORSHIFT_LEFT_FIRST 13
#define EAR_SIM_XORSHIFT_RIGHT 17
#define EAR_SIM_XORSHIFT_LEFT_LAST 5
#define EAR_SIM_NOISE_STEPS 30001U
#define EAR_SIM_NOISE_STEPS_PER_DB 10000.0
#define EAR_SIM_NOISE_HALF_RANGE_DB 1.5
#define EAR_SIM_REFERENCE_DISTANCE_M 1.0
#define EAR_SIM_DISTANCE_DB_PER_DECADE 20.0
#define EAR_SIM_CONTRAST_DB 6.0
#define EAR_SIM_FREQUENCY_HZ 2700.0

extern char **environ;

/** @brief Latest parsed window plus optional producer timing; published under ear_mutex. */
struct latest_measurement {
  uint64_t timestamp_ms;
  double level_db;
  double trend_db;
  double contrast_db;
  double frequency_hz;
  bool alarm;
  bool clipped;
  bool valid;
  struct capture_observation observation;
  bool classifier;
  double classifier_score;
  unsigned classifier_class;
};

static pid_t server_pid = -1;
static int server_input = -1;
static int server_output = -1;
static pthread_t reader_thread;
static bool reader_running;
static atomic_bool reader_stop;
static pthread_t supervisor_thread;
static bool supervisor_running;
static char microphone_device[EAR_MICROPHONE_NAME_CAPACITY] = USHER_EAR_CAM_DEVICE;
static char ear_settings[PATH_MAX];

bool ear_cam_pipe_set_settings(const char *settings)
{
  if (!settings || !settings[0] || supervisor_running || reader_running) return false;
  const size_t length = strnlen(settings, sizeof(ear_settings));
  if (length >= sizeof(ear_settings)) return false;
  for (size_t index = 0; index <= length; ++index) ear_settings[index] = settings[index];
  return true;
}
static atomic_bool supervisor_stop;
static pthread_mutex_t ear_mutex = PTHREAD_MUTEX_INITIALIZER;
static struct latest_measurement latest;
static bool server_ready;
static uint64_t server_error_count;
static uint64_t server_started_ms;
static uint64_t last_warning_ms;

/**
 * @brief Statically reserved sample buffer with a single active or detached owner.
 * @details Count is finalized before handoff. Atomic ownership lets the worker
 * release storage without racing serial-owner admission. Detached contents need
 * no lock because the producer and active session must not modify them.
 */
struct ear_session_snapshot {
  struct ear_sample samples[EAR_MAX_SAMPLES];
  size_t count;
  atomic_bool owned;
};
static struct ear_session_snapshot session_buffers[EAR_SESSION_BUFFER_COUNT] = {{.owned = true}, {.owned = false}};
static struct ear_session_snapshot *active_session = &session_buffers[0];
static struct ear_sample *session = session_buffers[0].samples;
static size_t session_count;
static uint32_t log_session_id;
static FILE *session_log;
static char session_log_path[EAR_LOG_PATH_CAPACITY];
static char log_queue[EAR_LOG_QUEUE_ROWS][EAR_LOG_ROW_CAPACITY];
static size_t log_head, log_count;
static pthread_mutex_t log_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t log_available = PTHREAD_COND_INITIALIZER;
static pthread_t log_thread;
static bool log_running, log_stopping;
static atomic_uint_fast64_t log_dropped;
static const char session_log_header[] =
  "t_ms,shot,lat_deg,lon_deg,agl_m,alt_m,level_db,trend_db,contrast_db,freq_hz,alarm,clip,"
  "pose_aligned,pose_uncertainty_m,window_earliest_us,window_latest_us,sequence,request_lat_deg,request_lon_deg,session_id,detector_kind,classification_score,classification_class\n";
static struct {
  struct latest_measurement measurement;
  union dc_shot_union request;
  uint64_t queued_ms;
} pending_audio[EAR_PENDING_AUDIO_WINDOWS];
static size_t pending_head, pending_count;
static uint64_t last_window_end;
static bool (*pose_lookup)(struct capture_observation *, uint64_t);
static void flush_pending_audio(bool force);

void ear_cam_pipe_set_pose_lookup(bool (*lookup)(struct capture_observation *, uint64_t))
{
  pose_lookup = lookup;
}

bool ear_cam_pipe_set_device(const char *device)
{
  if (!device || supervisor_running || reader_running) return false;
  const size_t length = strnlen(device, sizeof(microphone_device));
  if (!length || length >= sizeof(microphone_device)) return false;
  for (size_t index = 0; index <= length; ++index) microphone_device[index] = device[index];
  return true;
}

static bool simulated;
static double simulated_lat_deg;
static double simulated_lon_deg;
static double simulated_level_db_at_1m;
static uint32_t simulated_noise_state = EAR_SIM_NOISE_INITIAL_STATE;

/** @brief Select deterministic virtual acoustic measurements for simulation.
 * @details Must precede initialization so no microphone child is launched. */
void ear_cam_pipe_set_simulated_source(double lat_deg, double lon_deg, double level_db_at_1m)
{
  simulated = true;
  simulated_lat_deg = lat_deg;
  simulated_lon_deg = lon_deg;
  simulated_level_db_at_1m = level_db_at_1m;
}

static char band_low_arg[EAR_BAND_ARGUMENT_CAPACITY];
static char band_high_arg[EAR_BAND_ARGUMENT_CAPACITY];
static bool band_set;

/** @brief Set the detector band forwarded to a subsequently spawned EARcam child. */
void ear_cam_pipe_set_band(double low_cut_hz, double high_cut_hz)
{
  if (supervisor_running || reader_running) return;
  band_set = isfinite(low_cut_hz) && isfinite(high_cut_hz) && low_cut_hz > 0.0 && high_cut_hz > low_cut_hz;
  if (band_set) {
    snprintf(band_low_arg, sizeof(band_low_arg), "%.17g", low_cut_hz);
    snprintf(band_high_arg, sizeof(band_high_arg), "%.17g", high_cut_hz);
  }
}

// Deterministic +/-1.5 dB microphone noise (xorshift), independent of libc rand().
/** @brief Produce reproducible bounded simulation noise without shared libc PRNG state. */
static double simulated_noise_db(void)
{
  uint32_t x = simulated_noise_state;
  x ^= x << EAR_SIM_XORSHIFT_LEFT_FIRST;
  x ^= x >> EAR_SIM_XORSHIFT_RIGHT;
  x ^= x << EAR_SIM_XORSHIFT_LEFT_LAST;
  simulated_noise_state = x;
  return ((double)(x % EAR_SIM_NOISE_STEPS) / EAR_SIM_NOISE_STEPS_PER_DB) - EAR_SIM_NOISE_HALF_RANGE_DB;
}

/**
 * @brief Model a virtual point source using free-field inverse-distance attenuation.
 * @details This fixture includes height and bounded noise, not terrain,
 * directivity, aircraft noise, microphone calibration or physical alarm validation.
 */
static struct latest_measurement simulated_measurement(const union dc_shot_union *shot, uint64_t now)
{
  double lat = shot->data.lat / EAR_COORDINATE_UNITS_PER_DEG;
  double lon = shot->data.lon / EAR_COORDINATE_UNITS_PER_DEG;
  double agl = shot->data.groundalt / POSITION_BFP_SCALE;
  double north = (lat - simulated_lat_deg) * DEG_TO_RAD * EARTH_RADIUS_M;
  double east = (lon - simulated_lon_deg) * DEG_TO_RAD * EARTH_RADIUS_M * cos(simulated_lat_deg * DEG_TO_RAD);
  double distance = sqrt(north * north + east * east + agl * agl);
  if (distance < EAR_SIM_REFERENCE_DISTANCE_M) {
    distance = EAR_SIM_REFERENCE_DISTANCE_M;
  }
  struct latest_measurement measurement = {0};
  measurement.timestamp_ms = now;
  measurement.level_db = simulated_level_db_at_1m - EAR_SIM_DISTANCE_DB_PER_DECADE * log10(
                           distance) + simulated_noise_db();
  measurement.contrast_db = EAR_SIM_CONTRAST_DB;
  measurement.trend_db = 0.0;
  measurement.frequency_hz = EAR_SIM_FREQUENCY_HZ;
  measurement.alarm = true;
  measurement.clipped = false;
  measurement.valid = true;
  return measurement;
}

/** @brief Return monotonic milliseconds for server freshness and retry decisions. */
static uint64_t monotonic_ms(void)
{
  struct timespec now;
  clock_gettime(CLOCK_MONOTONIC, &now);
  return (uint64_t)now.tv_sec * EAR_MS_PER_SECOND + (uint64_t)now.tv_nsec / EAR_NS_PER_MS;
}

/** @brief Sleep through signal interruption for bounded lifecycle polling. */
static void sleep_ms(unsigned int milliseconds)
{
  struct timespec interval = {
    .tv_sec = milliseconds / EAR_MS_PER_SECOND,
    .tv_nsec = (long)(milliseconds % EAR_MS_PER_SECOND) * EAR_SLEEP_NS_PER_MS
  };
  while (nanosleep(&interval, &interval) != 0 && errno == EINTR) {
    continue;
  }
}

/** @brief Parse one named numeric field from an EARcam server line.
 * @return 0 on one finite complete value, otherwise -1.
 * @details Field-boundary checks prevent a short key from matching a suffix of another key. */
static int parse_field(const char *line, const char *key, double *value)
{
  if (!line || !key || !*key || !value) return -1;
  const char *position = line;
  size_t key_length = strlen(key);
  bool found = false;
  double result = 0;
  while ((position = strstr(position, key)) != NULL) {
    if ((position == line || position[-1] == ' ') && position[key_length] == '=') {
      if (found) return -1;
      char *end = NULL;
      errno = 0;
      double parsed = strtod(position + key_length + 1, &end);
      if (errno || end == position + key_length + 1 || !isfinite(parsed)
          || (*end && *end != ' ' && *end != '\r' && *end != '\n')) {
        return -1;
      }
      result = parsed;
      found = true;
    }
    position += key_length;
  }
  if (found) *value = result;
  return found ? 0 : -1;
}

/** @brief Parse exact unsigned timing/sequence evidence without rounding through double. */
static bool parse_unsigned_field(const char *line, const char *key, uint64_t *value)
{
  if (!line || !key || !*key || !value) return false;
  const size_t length = strlen(key);
  const char *position = line;
  bool found = false;
  uint64_t result = 0;
  while ((position = strstr(position, key)) != NULL) {
    if ((position == line || position[-1] == ' ') && position[length] == '=') {
      if (found) return false;
      position += length + 1;
      if (*position < '0' || *position > '9') return false;
      char *end = NULL;
      errno = 0;
      const unsigned long long parsed = strtoull(position, &end, 10);
      if (errno || end == position || (*end && *end != ' ' && *end != '\r' && *end != '\n')) return false;
      result = parsed;
      found = true;
    }
    position += length;
  }
  if (found) *value = result;
  return found;
}

/** @brief Interpret one complete server output line and atomically update shared state.
 * @details Malformed samples are ignored rather than poisoning the latest valid reading. */
static void handle_server_line(const char *line)
{
  if (!line) return;
  const bool classifier = !strncmp(line, "CLASS ", 6);
  if (classifier || strncmp(line, "SAMPLE ", EAR_SAMPLE_PREFIX_LENGTH) == 0) {
    struct latest_measurement measurement = {0};
    uint64_t timestamp = 0;
    double alarm = 0.0;
    double clipped = 0.0;
    if (!parse_unsigned_field(line, "t", &timestamp)
        || parse_field(line, "level", &measurement.level_db) != 0
        || (!classifier && parse_field(line, "freq", &measurement.frequency_hz) != 0)
        || (!classifier && parse_field(line, "contrast", &measurement.contrast_db) != 0)
        || (!classifier && parse_field(line, "trend", &measurement.trend_db) != 0)
        || parse_field(line, "alarm", &alarm) != 0
        || parse_field(line, "clip", &clipped) != 0) {
      return;
    }
    if (classifier) {
      uint64_t category;
      if (parse_field(line, "score", &measurement.classifier_score) ||
          measurement.classifier_score < 0 || measurement.classifier_score > 1 ||
          !parse_unsigned_field(line, "class", &category) || category > 4095) return;
      measurement.classifier = true;
      measurement.classifier_class = (unsigned)category;
    }
    if (timestamp > UINT64_MAX / EAR_US_PER_MS - 1 ||
      (alarm != 0 && alarm != 1) || (clipped != 0 && clipped != 1)) return;
    measurement.timestamp_ms = timestamp;
    if (strstr(line, " begin_us=")) {
      uint64_t begin_us, end_us, error_us, sequence;
      if (!parse_unsigned_field(line, "begin_us", &begin_us) || !parse_unsigned_field(line, "end_us", &end_us)
          || !parse_unsigned_field(line, "error_us", &error_us) || !parse_unsigned_field(line, "sequence", &sequence)) return;
      if (begin_us || end_us) {
        /* Widen producer bounds by its error only after plausibility/overflow
         * checks. Legacy/all-zero timing stays explicitly unaligned rather
         * than being treated as a precise capture timestamp. */
        if (begin_us <= error_us || end_us < begin_us || end_us - begin_us > EAR_AUDIO_WINDOW_LIMIT_US
            || error_us > EAR_AUDIO_TIMING_ERROR_LIMIT_US
            || end_us > measurement.timestamp_ms * EAR_US_PER_MS + EAR_AUDIO_FUTURE_SLACK_US
            || end_us > UINT64_MAX - error_us || !sequence) return;
        measurement.observation = (struct capture_observation) {
          .earliest_us = begin_us - error_us, .latest_us = end_us + error_us,
                                              .sequence = sequence, .kind = CAPTURE_TIME_AUDIO_WINDOW
        };
      }
    }
    measurement.alarm = alarm != 0.0;
    measurement.clipped = clipped != 0.0;
    measurement.valid = true;
    pthread_mutex_lock(&ear_mutex);
    latest = measurement;
    pthread_mutex_unlock(&ear_mutex);
    return;
  }
  if (strncmp(line, "EARCAM_READY", EAR_READY_PREFIX_LENGTH) == 0) {
    pthread_mutex_lock(&ear_mutex);
    server_ready = true;
    pthread_mutex_unlock(&ear_mutex);
    printf("EAR_CAM_PIPE:\t%s\n", line);
    return;
  }
  if (strncmp(line, "EARCAM_ERROR", EAR_ERROR_PREFIX_LENGTH) == 0) {
    pthread_mutex_lock(&ear_mutex);
    server_error_count++;
    pthread_mutex_unlock(&ear_mutex);
    fprintf(stderr, "EAR_CAM_PIPE:\t%s\n", line);
    return;
  }
  if (strncmp(line, "EARCAM_STOPPED", EAR_STOPPED_PREFIX_LENGTH) == 0) {
    printf("EAR_CAM_PIPE:\t%s\n", line);
  }
}

/** @brief Read and frame EARcam output in a dedicated thread.
 * @details Chunked reads avoid per-byte syscalls; only the latest measurement is retained
 * because USHER correlates a shot to current acoustic evidence, not a delayed backlog. */
static void *reader_main(void *unused)
{
  (void)unused;
  char line[EAR_LINE_MAX];
  size_t line_size = 0;
  bool discarding_line = false;

  while (!reader_stop && server_output >= 0) {
    struct pollfd descriptor = {.fd = server_output, .events = POLLIN, .revents = 0};
    int poll_result = poll(&descriptor, 1, EAR_READER_POLL_MS);
    if (poll_result < 0) {
      if (errno == EINTR) {
        continue;
      }
      break;
    }
    if (poll_result == 0) {
      continue;
    }
    char chunk[EAR_READER_CHUNK_BYTES];
    ssize_t count = read(server_output, chunk, sizeof(chunk));
    if (count == 0) {
      break;
    }
    if (count < 0) {
      if (errno == EINTR || errno == EAGAIN) {
        continue;
      }
      break;
    }
    for (ssize_t index = 0; index < count; index++) {
      if (chunk[index] == '\n') {
        if (!discarding_line) {
          line[line_size] = '\0';
          handle_server_line(line);
        }
        line_size = 0;
        discarding_line = false;
      } else if (!discarding_line && line_size + 1 < sizeof(line) &&
                 (((unsigned char)chunk[index] >= 32 && (unsigned char)chunk[index] < 127) || chunk[index] == '\r')) {
        line[line_size++] = chunk[index];
      } else {
        line_size = 0;
        discarding_line = true;
      }
    }
  }
  pthread_mutex_lock(&ear_mutex);
  server_ready = false;
  latest.valid = false;
  pthread_mutex_unlock(&ear_mutex);
  return NULL;
}

/** @brief Request child shutdown, join its reader, and reap the owned process.
 * @details Joining precedes descriptor teardown so the reader cannot race reused state. */
static void stop_server(void)
{
  reader_stop = 1;
  if (server_input >= 0) {
    static const char quit[] = "QUIT\n";
    ssize_t written = write(server_input, quit, sizeof(quit) - 1);
    (void)written;
    close(server_input);
    server_input = -1;
  }
  if (reader_running) {
    pthread_join(reader_thread, NULL);
    reader_running = false;
  }
  if (server_output >= 0) {
    close(server_output);
    server_output = -1;
  }
  if (server_pid > 0) {
    int status;
    pid_t wait_result;
    for (int attempt = 0; attempt < EAR_CHILD_REAP_ATTEMPTS; attempt++) {
      wait_result = waitpid(server_pid, &status, WNOHANG);
      if (wait_result == server_pid || (wait_result < 0 && errno == ECHILD)) {
        break;
      }
      if (attempt == EAR_CHILD_TERM_ATTEMPT) kill(server_pid, SIGTERM);
      if (attempt == EAR_CHILD_KILL_ATTEMPT) kill(server_pid, SIGKILL);
      sleep_ms(EAR_SUPERVISOR_TICK_MS);
    }
    if (wait_result == server_pid || (wait_result < 0 && errno == ECHILD)) {
      server_pid = -1;
    } else {
      fprintf(stderr, "EAR_CAM_PIPE:\tchild %ld not reaped; replacement deferred\n", (long)server_pid);
    }
  }
  pthread_mutex_lock(&ear_mutex);
  server_ready = false;
  latest.valid = false;
  pthread_mutex_unlock(&ear_mutex);
}

/** @brief Create the CSV audit log lazily on the first accepted acoustic sample. */
static int open_session_log(void)
{
  char resolved_log[PATH_MAX];
  const char *log_dir = usher_resolve_path(USHER_EAR_CAM_LOG_DIR, resolved_log, sizeof(resolved_log));

  if (usher_ensure_directory(log_dir) != 0) {
    fprintf(stderr, "EAR_CAM_PIPE:\tfailed to create %s: %s\n",
            log_dir, strerror(errno));
    return -1;
  }
  time_t now = time(NULL);
  struct tm stamp;
  localtime_r(&now, &stamp);
  int descriptor = -1;
  for (unsigned attempt = 0; attempt < EAR_LOG_FILENAME_ATTEMPTS; ++attempt) {
    int length = snprintf(session_log_path, sizeof(session_log_path),
                          "%s/ear_%04d%02d%02d_%02d%02d%02d_%03u.csv", log_dir,
                          stamp.tm_year + EAR_TM_YEAR_ORIGIN, stamp.tm_mon + 1, stamp.tm_mday,
                          stamp.tm_hour, stamp.tm_min, stamp.tm_sec, attempt);
    if (length < 0 || (size_t)length >= sizeof(session_log_path)) return -1;
    descriptor = open(session_log_path, O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, EAR_LOG_FILE_MODE);
    if (descriptor >= 0 || errno != EEXIST) break;
  }
  session_log = descriptor >= 0 ? fdopen(descriptor, "w") : NULL;
  if (descriptor >= 0 && !session_log) close(descriptor);
  if (session_log == NULL) {
    fprintf(stderr, "EAR_CAM_PIPE:\tfailed to open %s: %s\n", session_log_path, strerror(errno));
    return -1;
  }
  fputs(session_log_header, session_log);
  fflush(session_log);
  return 0;
}

/**
 * @brief Drain bounded row batches; this worker alone opens/writes/flushes/closes the log.
 * @details Release log_mutex before filesystem I/O. A latched write failure counts
 * lost rows without repeatedly blocking capture on an already failed destination.
 */
static void *write_session_log(void *unused)
{
  (void)unused;
  bool failed = false;
  for (;;) {
    char batch[EAR_LOG_BATCH_ROWS][EAR_LOG_ROW_CAPACITY];
    pthread_mutex_lock(&log_mutex);
    while (!log_count && !log_stopping) pthread_cond_wait(&log_available, &log_mutex);
    if (!log_count && log_stopping) {
      pthread_mutex_unlock(&log_mutex);
      break;
    }
    const size_t count = log_count < EAR_LOG_BATCH_ROWS ? log_count : EAR_LOG_BATCH_ROWS;
    for (size_t row = 0; row < count; ++row) {
      for (size_t index = 0; index < sizeof(batch[row]); ++index) batch[row][index] = log_queue[log_head][index];
      log_head = (log_head + 1) % (sizeof(log_queue) / sizeof(log_queue[0]));
    }
    log_count -= count;
    pthread_mutex_unlock(&log_mutex);
    if (!failed && !session_log && open_session_log() != 0) failed = true;
    if (!failed) {
      for (size_t row = 0; row < count; ++row) if (fputs(batch[row], session_log) == EOF) failed = true;
      if (fflush(session_log) != 0) failed = true;
    }
    if (failed) atomic_fetch_add(&log_dropped, count);
  }
  if (session_log && fclose(session_log) != 0) atomic_fetch_add(&log_dropped, 1);
  session_log = NULL;
  return NULL;
}

/**
 * @brief Best-effort serial-owner enqueue with copied rows and no filesystem wait.
 * @details A busy/full queue drops an audit row rather than delaying UART/audio;
 * snapshot CSV is written separately from immutable samples at stop.
 */
static void queue_session_log(const char *line)
{
  if (!log_running) {
    log_stopping = false;
    log_head = log_count = 0;
    atomic_store(&log_dropped, 0);
    if (pthread_create(&log_thread, NULL, write_session_log, NULL) != 0) {
      atomic_fetch_add(&log_dropped, 1);
      return;
    }
    log_running = true;
  }
  if (pthread_mutex_trylock(&log_mutex) != 0) {
    atomic_fetch_add(&log_dropped, 1);
    return;
  }
  const size_t capacity = sizeof(log_queue) / sizeof(log_queue[0]);
  const size_t length = strnlen(line, sizeof(log_queue[0]));
  if (log_count == capacity || length >= sizeof(log_queue[0])) atomic_fetch_add(&log_dropped, 1);
  else {
    const size_t slot = (log_head + log_count++) % capacity;
    for (size_t index = 0; index <= length; ++index) log_queue[slot][index] = line[index];
    pthread_cond_signal(&log_available);
  }
  pthread_mutex_unlock(&log_mutex);
}

/** @brief Join the log writer after queued rows drain; not a hard filesystem deadline. */
static void close_session_log(void)
{
  if (!log_running) return;
  pthread_mutex_lock(&log_mutex);
  log_stopping = true;
  pthread_cond_signal(&log_available);
  pthread_mutex_unlock(&log_mutex);
  pthread_join(log_thread, NULL);
  log_running = false;
  if (atomic_load(&log_dropped)) fprintf(stderr, "EAR_CAM_PIPE:\taudio log dropped/failed rows: %llu\n",
                                           (unsigned long long)atomic_load(&log_dropped));
}

/**
 * @brief Prevent the audio child from inheriting unrelated USHER descriptors.
 * @details Only intended stdin/stdout pipes survive setup; leaked UART/SODA FDs
 * could keep channels alive after their real owners close them. Supervisor-only.
 */
static int add_inherited_descriptor_closes(posix_spawn_file_actions_t *actions,
    const int command_pipe[2],
    const int response_pipe[2])
{
  long descriptor_limit = sysconf(_SC_OPEN_MAX);
  if (descriptor_limit < 0) {
    return errno != 0 ? errno : EINVAL;
  }
  if (descriptor_limit > INT_MAX) {
    descriptor_limit = INT_MAX;
  }
  for (int descriptor = STDERR_FILENO + 1; descriptor < descriptor_limit; ++descriptor) {
    if (descriptor == command_pipe[0] || descriptor == command_pipe[1]
        || descriptor == response_pipe[0] || descriptor == response_pipe[1]) {
      continue;
    }
    if (fcntl(descriptor, F_GETFD) < 0) {
      if (errno == EBADF) {
        continue;
      }
      return errno;
    }
    int result = posix_spawn_file_actions_addclose(actions, descriptor);
    if (result != 0) {
      return result;
    }
  }
  return 0;
}

// Spawn `earcam --server` with a reader thread; returns without waiting for EARCAM_READY.
/** @brief Spawn `earcam --server` and its asynchronous line reader.
 * @return 0 after ownership is established, otherwise -1 with all partial resources closed.
 * @details Readiness is observed asynchronously so process spawn cannot block USHER's
 * startup indefinitely on a missing microphone. */
static int start_server(void)
{
  if (server_pid > 0) return -1;
  int command_pipe[2];
  int response_pipe[2];
  if (pipe(command_pipe) != 0) {
    fprintf(stderr, "EAR_CAM_PIPE:\tfailed to create pipes: %s\n", strerror(errno));
    return -1;
  }
  if (pipe(response_pipe) != 0) {
    fprintf(stderr, "EAR_CAM_PIPE:\tfailed to create pipes: %s\n", strerror(errno));
    close(command_pipe[0]);
    close(command_pipe[1]);
    return -1;
  }
  int *endpoints[] = {&command_pipe[0], &command_pipe[1], &response_pipe[0], &response_pipe[1]};
  for (size_t index = 0; index < sizeof(endpoints) / sizeof(endpoints[0]); ++index) {
    int descriptor = *endpoints[index];
    if (descriptor <= STDERR_FILENO) {
      descriptor = fcntl(descriptor, F_DUPFD_CLOEXEC, STDERR_FILENO + 1);
      if (descriptor >= 0) { close(*endpoints[index]); *endpoints[index] = descriptor; }
    }
    if (descriptor < 0 || fcntl(descriptor, F_SETFD, FD_CLOEXEC) != 0) {
      const int saved_errno = errno;
      close(command_pipe[0]);
      close(command_pipe[1]);
      close(response_pipe[0]);
      close(response_pipe[1]);
      errno = saved_errno;
      return -1;
    }
  }
  int flags = fcntl(command_pipe[1], F_GETFL);
  if (flags < 0 || fcntl(command_pipe[1], F_SETFL, flags | O_NONBLOCK) != 0) {
    close(command_pipe[0]);
    close(command_pipe[1]);
    close(response_pipe[0]);
    close(response_pipe[1]);
    return -1;
  }

  posix_spawn_file_actions_t actions;
  if (posix_spawn_file_actions_init(&actions) != 0) {
    fprintf(stderr, "EAR_CAM_PIPE:\tfailed to configure earcam server\n");
    close(command_pipe[0]);
    close(command_pipe[1]);
    close(response_pipe[0]);
    close(response_pipe[1]);
    return -1;
  }
  if (posix_spawn_file_actions_adddup2(&actions, command_pipe[0], STDIN_FILENO) != 0
      || posix_spawn_file_actions_adddup2(&actions, response_pipe[1], STDOUT_FILENO) != 0
      || add_inherited_descriptor_closes(&actions, command_pipe, response_pipe) != 0
      || posix_spawn_file_actions_addclose(&actions, command_pipe[0]) != 0
      || posix_spawn_file_actions_addclose(&actions, command_pipe[1]) != 0
      || posix_spawn_file_actions_addclose(&actions, response_pipe[0]) != 0
      || posix_spawn_file_actions_addclose(&actions, response_pipe[1]) != 0) {
    fprintf(stderr, "EAR_CAM_PIPE:\tfailed to configure earcam server\n");
    posix_spawn_file_actions_destroy(&actions);
    close(command_pipe[0]);
    close(command_pipe[1]);
    close(response_pipe[0]);
    close(response_pipe[1]);
    return -1;
  }

  char *arguments[13];
  size_t argument_count = 0;
  arguments[argument_count++] = (char *)USHER_EAR_CAM_COMMAND;
  arguments[argument_count++] = (char *)"--server";
  arguments[argument_count++] = (char *)"--device";
  arguments[argument_count++] = microphone_device;
  if (ear_settings[0]) {
    arguments[argument_count++] = (char *)"--settings";
    arguments[argument_count++] = ear_settings;
  }
  if (band_set) {
    arguments[argument_count++] = (char *)"--low-cut";
    arguments[argument_count++] = band_low_arg;
    arguments[argument_count++] = (char *)"--high-cut";
    arguments[argument_count++] = band_high_arg;
  }
  arguments[argument_count] = NULL;
  int spawn_result = posix_spawn(&server_pid, USHER_EAR_CAM_COMMAND, &actions, NULL,
                                 arguments, environ);
  posix_spawn_file_actions_destroy(&actions);
  close(command_pipe[0]);
  close(response_pipe[1]);
  if (spawn_result != 0) {
    fprintf(stderr, "EAR_CAM_PIPE:\tfailed to start %s: %s\n",
            USHER_EAR_CAM_COMMAND, strerror(spawn_result));
    close(command_pipe[1]);
    close(response_pipe[0]);
    server_pid = -1;
    return -1;
  }
  server_input = command_pipe[1];
  server_output = response_pipe[0];
  reader_stop = 0;
  pthread_mutex_lock(&ear_mutex);
  server_ready = false;
  server_error_count = 0;
  latest.valid = false;
  pthread_mutex_unlock(&ear_mutex);
  server_started_ms = monotonic_ms();
  if (pthread_create(&reader_thread, NULL, reader_main, NULL) != 0) {
    fprintf(stderr, "EAR_CAM_PIPE:\tfailed to start reader thread\n");
    stop_server();
    return -1;
  }
  reader_running = true;
  return 0;
}

/**
 * @brief Retry missing/disconnected capture independently of optical operation.
 * @details Error, lost readiness or startup timeout triggers owned teardown
 * before another spawn. The retry delay checks shutdown in small polling ticks.
 */
static void *supervise_server(void *unused)
{
  (void)unused;
  while (!supervisor_stop) {
    if (start_server() == 0) {
      bool was_ready = false;
      for (;;) {
        pthread_mutex_lock(&ear_mutex);
        bool ready = server_ready;
        uint64_t errors = server_error_count;
        pthread_mutex_unlock(&ear_mutex);
        if (supervisor_stop || errors > 0
            || (was_ready && !ready)
            || (!ready && monotonic_ms() - server_started_ms >= EAR_READY_TIMEOUT_MS)) break;
        was_ready = was_ready || ready;
        sleep_ms(EAR_SUPERVISOR_TICK_MS);
      }
    }
    stop_server();
    uint64_t retry_at = monotonic_ms() + EAR_RESTART_INTERVAL_MS;
    while (!supervisor_stop && monotonic_ms() < retry_at) sleep_ms(EAR_SUPERVISOR_TICK_MS);
  }
  stop_server();
  return NULL;
}

/** @brief Rate-limit repeated unavailable/stale microphone diagnostics. */
static bool warning_due(uint64_t now)
{
  if (now - last_warning_ms < EAR_WARNING_INTERVAL_MS) {
    return false;
  }
  last_warning_ms = now;
  return true;
}

/** @brief Reset session storage and start or configure the acoustic backend.
 * @return -1 only for unrecoverable setup errors; missing hardware stays retryable. */
int ear_cam_pipe_init(const char *unused)
{
  (void)unused;
  if (supervisor_running) return 0;

  session_count = 0;
  pending_head = pending_count = 0;
  last_window_end = 0;

  if (simulated) {
    pthread_mutex_lock(&ear_mutex);
    server_ready = true;
    latest.valid = false;
    pthread_mutex_unlock(&ear_mutex);
    printf("EAR_CAM_PIPE:\tSIMULATED loudspeaker at %.7f %.7f, %.1f dB at 1 m (no microphone)\n",
           simulated_lat_deg, simulated_lon_deg, simulated_level_db_at_1m);
    return 0;
  }

  if (access(USHER_EAR_CAM_COMMAND, X_OK) != 0) {
    fprintf(stderr, "EAR_CAM_PIPE:\tcapture command is not executable: %s: %s\n",
            USHER_EAR_CAM_COMMAND, strerror(errno));
    return -1;
  }
  supervisor_stop = false;
  int result = pthread_create(&supervisor_thread, NULL, supervise_server, NULL);
  if (result != 0) {
    fprintf(stderr, "EAR_CAM_PIPE:\tfailed to start supervisor: %s\n", strerror(result));
    return -1;
  }
  supervisor_running = true;
  printf("EAR_CAM_PIPE:\tcapture command: %s\n", USHER_EAR_CAM_COMMAND);
  printf("EAR_CAM_PIPE:\tmicrophone: %s\n", microphone_device);
  printf("EAR_CAM_PIPE:\tasynchronous microphone supervisor started\n");
  return 0;
}

/** @brief Stop the child and clear all bounded session state. */
void ear_cam_pipe_deinit(void)
{
  if (supervisor_running) {
    supervisor_stop = true;
    pthread_join(supervisor_thread, NULL);
    supervisor_running = false;
  }
  pthread_mutex_lock(&ear_mutex);
  server_ready = false;
  pthread_mutex_unlock(&ear_mutex);
  flush_pending_audio(true);
  close_session_log();
  session_count = 0;
}

/** @brief Report whether an actual or simulated measurement source is ready. */
bool ear_cam_pipe_ready(void)
{
  pthread_mutex_lock(&ear_mutex);
  bool ready = server_ready;
  pthread_mutex_unlock(&ear_mutex);
  return ready;
}

/** @brief Format one evidence row with both chosen pose and original request provenance. */
static int format_session_row(const struct ear_sample *sample, uint32_t session_id, char *line, size_t size)
{
  return snprintf(line, size,
                  "%llu,%d,%.7f,%.7f,%.2f,%.2f,%.2f,%.2f,%.2f,%.0f,%d,%d,%d,%.3f,%llu,%llu,%llu,%.7f,%.7f,%u,%s,%.6f,%u\n",
                  (unsigned long long)sample->timestamp_ms, sample->shot_nr, sample->lat_deg,
                  sample->lon_deg, sample->agl_m, sample->alt_m, sample->level_db,
                  sample->trend_db, sample->contrast_db, sample->frequency_hz,
                  sample->alarm ? 1 : 0, sample->clipped ? 1 : 0, sample->pose_aligned ? 1 : 0, sample->pose_uncertainty_m,
                  (unsigned long long)sample->window_earliest_us, (unsigned long long)sample->window_latest_us,
                  (unsigned long long)sample->window_sequence, sample->request_lat / EAR_COORDINATE_UNITS_PER_DEG,
                  sample->request_lon / EAR_COORDINATE_UNITS_PER_DEG,
                  session_id, sample->classifier ? "onnx_pcm" : "tone", sample->classifier_score, sample->classifier_class);
}

/**
 * @brief Append on the serial owner using aligned pose when available, labelled request pose otherwise.
 * @details At capacity keep the newest half of active evidence. This cannot
 * overwrite a detached snapshot; admission must acquire its own buffer first.
 */
static void commit_audio_sample(const union dc_shot_union *request, const struct latest_measurement *measurement)
{
  const union dc_shot_union *shot = measurement->observation.pose_valid ? &measurement->observation.pose : request;
  if (session_count >= EAR_MAX_SAMPLES) {
    // Keep the newest data: drop the oldest half to stay bounded on a small board.
    size_t keep = EAR_MAX_SAMPLES / 2;
    for (size_t index = 0; index < keep; index++) {
      session[index] = session[index + keep];
    }
    session_count = keep;
  }
  struct ear_sample *sample = &session[session_count++];
  sample->timestamp_ms = measurement->timestamp_ms;
  sample->shot_nr = request->data.nr;
  sample->lat_deg = shot->data.lat / EAR_COORDINATE_UNITS_PER_DEG;
  sample->lon_deg = shot->data.lon / EAR_COORDINATE_UNITS_PER_DEG;
  sample->agl_m = shot->data.groundalt / POSITION_BFP_SCALE;
  sample->alt_m = shot->data.alt / EAR_ALTITUDE_UNITS_PER_METER;
  sample->level_db = measurement->level_db;
  sample->trend_db = measurement->trend_db;
  sample->contrast_db = measurement->contrast_db;
  sample->frequency_hz = measurement->frequency_hz;
  sample->classifier = measurement->classifier;
  sample->classifier_score = measurement->classifier_score;
  sample->classifier_class = measurement->classifier_class;
  sample->alarm = measurement->alarm;
  sample->clipped = measurement->clipped;
  sample->pose_aligned = measurement->observation.pose_valid;
  sample->pose_uncertainty_m = measurement->observation.pose_valid ? measurement->observation.pose_uncertainty_m : -1;
  sample->window_earliest_us = measurement->observation.earliest_us;
  sample->window_latest_us = measurement->observation.latest_us;
  sample->window_sequence = measurement->observation.sequence;
  sample->request_lat = request->data.lat;
  sample->request_lon = request->data.lon;
  char line[EAR_LOG_ROW_CAPACITY];
  const int length = format_session_row(sample, log_session_id, line, sizeof(line));
  if (length > 0 && (size_t)length < sizeof(line)) queue_session_log(line);
  else atomic_fetch_add(&log_dropped, 1);
}

/**
 * @brief Retry pose lookup without waiting; bound unresolved evidence by count and age.
 * @details FIFO ordering keeps window provenance stable. Stop/full-queue forces
 * request-pose fallback rather than waiting for poses or losing admitted audio.
 */
static void flush_pending_audio(bool force)
{
  const uint64_t now = monotonic_ms();
  while (pending_count) {
    struct capture_observation *observation = &pending_audio[pending_head].measurement.observation;
    const bool aligned = pose_lookup && pose_lookup(observation, 0);
    if (!aligned && !force && now >= pending_audio[pending_head].queued_ms
        && now - pending_audio[pending_head].queued_ms < EAR_POSE_WAIT_MS) break;
    commit_audio_sample(&pending_audio[pending_head].request, &pending_audio[pending_head].measurement);
    pending_head = (pending_head + 1) % (sizeof(pending_audio) / sizeof(pending_audio[0]));
    --pending_count;
  }
}

void ear_cam_pipe_update_pose(void)
{
  if (!session) return;
  flush_pending_audio(false);
}

int ear_cam_pipe_record(const union dc_shot_union *shot)
{
  if (!shot || !session) return -1;
  flush_pending_audio(false);
  const uint64_t now = monotonic_ms();
  pthread_mutex_lock(&ear_mutex);
  struct latest_measurement measurement = simulated ? simulated_measurement(shot, now) : latest;
  const bool ready = server_ready;
  pthread_mutex_unlock(&ear_mutex);
    const uint64_t window_end_ms = measurement.observation.latest_us / EAR_US_PER_MS;
  if (!ready || !measurement.valid || measurement.timestamp_ms > now + EAR_SAMPLE_FUTURE_SLACK_MS
      || (now > measurement.timestamp_ms && now - measurement.timestamp_ms > EAR_SAMPLE_STALE_MS)
      || (capture_observation_valid(&measurement.observation) && now > window_end_ms &&
        now - window_end_ms > EAR_SAMPLE_STALE_MS)) {
    if (warning_due(now)) fprintf(stderr, "EAR_CAM_PIPE-%d:\tno fresh microphone measurement available\n", shot->data.nr);
    return -1;
  }
  if (!simulated && pose_lookup && capture_observation_valid(&measurement.observation)) {
    if (measurement.observation.latest_us <= last_window_end) return -1;
    last_window_end = measurement.observation.latest_us;
    const size_t capacity = sizeof(pending_audio) / sizeof(pending_audio[0]);
    if (pending_count == capacity) flush_pending_audio(true);
    const size_t slot = (pending_head + pending_count++) % capacity;
    pending_audio[slot].measurement = measurement;
    pending_audio[slot].request = *shot;
    pending_audio[slot].queued_ms = now;
    flush_pending_audio(false);
  } else commit_audio_sample(shot, &measurement);
  return 0;
}

int32_t ear_cam_pipe_last_shot_nr(void)
{
  return session_count > 0 ? session[session_count - 1].shot_nr : 0;
}

bool ear_cam_pipe_begin_session(uint32_t session_id)
{
  if (!active_session) {
    for (size_t slot = 0; slot < EAR_SESSION_BUFFER_COUNT; ++slot) {
      bool available = false;
      if (atomic_compare_exchange_strong(&session_buffers[slot].owned, &available, true)) {
        active_session = &session_buffers[slot];
        break;
      }
    }
    if (!active_session) return false;
  }
  flush_pending_audio(true);
  session = active_session->samples;
  session_count = 0;
  pending_head = pending_count = 0;
  last_window_end = 0;
  log_session_id = session_id;
  char line[EAR_SESSION_MARKER_CAPACITY];
  snprintf(line, sizeof(line), "# start session_id=%u\n", session_id);
  queue_session_log(line);
  return true;
}

struct ear_session_snapshot *ear_cam_pipe_detach_session(void)
{
  if (!active_session) return NULL;
  flush_pending_audio(true);
  struct ear_session_snapshot *snapshot = active_session;
  snapshot->count = session_count;
  char line[EAR_SESSION_MARKER_CAPACITY];
  snprintf(line, sizeof(line), "# stop session_id=%u samples=%zu\n", log_session_id, session_count);
  queue_session_log(line);
  active_session = NULL;
  session = NULL;
  session_count = 0;
  pending_head = pending_count = 0;
  last_window_end = 0;
  return snapshot;
}

int ear_cam_pipe_snapshot_solve(const struct ear_session_snapshot *snapshot, struct ear_loudest_spot *result)
{
  if (!snapshot || !result) return -1;
  *result = (struct ear_loudest_spot) {0};
  return calculated_loudestspot(snapshot->samples, snapshot->count, result);
}

int32_t ear_cam_pipe_snapshot_number(const struct ear_session_snapshot *snapshot)
{
  return snapshot && snapshot->count ? snapshot->samples[snapshot->count - 1].shot_nr : 0;
}

void ear_cam_pipe_snapshot_release(struct ear_session_snapshot *snapshot)
{
  if (snapshot) atomic_store(&snapshot->owned, false);
}

/**
 * @brief Write evidence from borrowed active or immutable snapshot samples.
 * @details Nonzero session IDs reserve JPEG and CSV names together with O_EXCL;
 * old evidence is never silently overwritten. Snapshot CSV failure and optional
 * field/.geo failure are reported separately from decorated JPEG success.
 */
static int render_session(const struct ear_sample *samples, size_t count, int32_t number, uint32_t session_id,
                          const struct ear_loudest_spot *result, char *filename, size_t filename_size,
                          bool simulated)
{
  if (filename == NULL || filename_size == 0) {
    return -1;
  }
  filename[0] = '\0';
  if (count == 0) {
    return -1;
  }

  char resolved_photo[PATH_MAX];
  const char *photo_dir = usher_resolve_path(USHER_EAR_CAM_PHOTO_DIR, resolved_photo, sizeof(resolved_photo));

  if (usher_ensure_directory(photo_dir) != 0) {
    fprintf(stderr, "EAR_CAM_PIPE:\tfailed to create %s: %s\n", photo_dir, strerror(errno));
    return -1;
  }
  const char *prefix = simulated ? "me" : "e";
  int length = snprintf(filename, filename_size, "%s/%s%06d.jpg", photo_dir,
                        prefix, number);
  if (session_id) {
    bool reserved = false;
    int csv_descriptor = -1;
    char csv_path[PATH_MAX];
    for (unsigned attempt = 0; attempt < EAR_EVIDENCE_FILENAME_ATTEMPTS; ++attempt) {
      length = snprintf(filename, filename_size, "%s/%s%06d_s%08x_%02u.jpg", photo_dir,
                        prefix, number, session_id, attempt);
      if (length < 0 || (size_t)length >= filename_size) break;
      int descriptor = open(filename, O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, EAR_EVIDENCE_FILE_MODE);
      if (descriptor < 0) {
        if (errno == EEXIST) continue;
        break;
      }
      int csv_length = snprintf(csv_path, sizeof(csv_path), "%.*s.csv", length - EAR_JPEG_SUFFIX_LENGTH, filename);
      if (csv_length > 0 && (size_t)csv_length < sizeof(csv_path))
        csv_descriptor = open(csv_path, O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, EAR_EVIDENCE_FILE_MODE);
      if (csv_descriptor < 0) {
        const int error = errno;
        close(descriptor);
        unlink(filename);
        if (error == EEXIST) continue;
        break;
      }
      if (close(descriptor) != 0) {
        close(csv_descriptor);
        unlink(filename);
        break;
      }
      reserved = true;
      break;
    }
    if (!reserved) {
      filename[0] = '\0';
      return -1;
    }
    FILE *csv = fdopen(csv_descriptor, "w");
    bool csv_saved = csv != NULL;
    if (csv) {
      if (fputs(session_log_header, csv) == EOF) csv_saved = false;
      for (size_t index = 0; index < count && csv_saved; ++index) {
        char line[EAR_LOG_ROW_CAPACITY];
        int row_length = format_session_row(&samples[index], session_id, line, sizeof(line));
        csv_saved = row_length > 0 && (size_t)row_length < sizeof(line) && fputs(line, csv) != EOF;
      }
      if (fprintf(csv,
                  "# result session_id=%u valid=%d lat=%.7f lon=%.7f agl=%.2f alt=%.2f level=%.2f conf=%.3f used=%u/%u\n",
                  session_id, result->valid, result->lat_deg, result->lon_deg, result->agl_m,
                  result->alt_m, result->level_db, result->confidence, result->used_count, result->sample_count) < 0)
        csv_saved = false;
      if (fclose(csv) != 0) csv_saved = false;
    } else close(csv_descriptor);
    if (!csv_saved) fprintf(stderr, "EAR_CAM_PIPE:\tfailed/incomplete snapshot CSV retained: %s\n", csv_path);
  }
  if (length < 0 || (size_t)length >= filename_size) {
    filename[0] = '\0';
    return -1;
  }
  if (ear_heatmap_write(filename, samples, count, result) != 0) {
    fprintf(stderr, "EAR_CAM_PIPE:\tfailed to render %s\n", filename);
    unlink(filename);
    filename[0] = '\0';
    return -1;
  }
  // Preserve a clean raster for map compositing; reusing the decorated JPEG would blend
  // observation dots, the estimate marker and scale bar a second time over the basemap.
  char field[PATH_MAX];
  length = snprintf(field, sizeof(field), "%.*s_field.jpg", (int)strlen(filename) - EAR_JPEG_SUFFIX_LENGTH, filename);
  if (length > 0 && (size_t)length < sizeof(field)) {
    if (ear_heatmap_write_field(field, samples, count, result) != 0)
      fprintf(stderr, "EAR_CAM_PIPE:\tfield rendering failed: %s\n", field);
  } else fprintf(stderr, "EAR_CAM_PIPE:\tfield filename too long for %s\n", filename);
  return 0;
}

int ear_cam_pipe_render(const struct ear_loudest_spot *result, char *filename, size_t filename_size,
                        bool simulated)
{
  return render_session(session, session_count, ear_cam_pipe_last_shot_nr(), 0,
                        result, filename, filename_size, simulated);
}

int ear_cam_pipe_snapshot_render(const struct ear_session_snapshot *snapshot, uint32_t session_id,
                                 const struct ear_loudest_spot *result, char *filename,
                                 size_t filename_size, bool simulated)
{
  if (!snapshot || !session_id || !result) return -1;
  return render_session(snapshot->samples, snapshot->count, ear_cam_pipe_snapshot_number(snapshot),
                        session_id, result, filename, filename_size, simulated);
}

/** @brief Solve the current session without consuming it, for intermediate mission feedback. */
int ear_cam_pipe_solve(struct ear_loudest_spot *result)
{
  flush_pending_audio(true);
  if (result == NULL) {
    return -1;
  }
  memset(result, 0, sizeof(*result));
  int status = calculated_loudestspot(session, session_count, result);
  char line[EAR_LOG_ROW_CAPACITY];
  snprintf(line, sizeof(line), "# interim session_id=%u valid=%d lat=%.7f lon=%.7f agl=%.2f conf=%.3f used=%u/%u\n",
           log_session_id, result->valid ? 1 : 0, result->lat_deg, result->lon_deg, result->agl_m,
           result->confidence, result->used_count, result->sample_count);
  queue_session_log(line);
  return status;
}

void ear_cam_pipe_log_result(uint32_t session_id, const struct ear_loudest_spot *result)
{
  if (!result) return;
  char line[EAR_LOG_ROW_CAPACITY];
  snprintf(line, sizeof(line),
           "# result session_id=%u valid=%d lat=%.7f lon=%.7f agl=%.2f alt=%.2f level=%.2f conf=%.3f used=%u/%u\n",
           session_id, result->valid ? 1 : 0, result->lat_deg, result->lon_deg, result->agl_m,
           result->alt_m, result->level_db, result->confidence, result->used_count, result->sample_count);
  queue_session_log(line);
}

/** @brief Solve and close the current session, clearing accumulated samples afterward. */
int ear_cam_pipe_finish(struct ear_loudest_spot *result)
{
  if (result == NULL) {
    return -1;
  }
  memset(result, 0, sizeof(*result));
  int status = calculated_loudestspot(session, session_count, result);
  ear_cam_pipe_log_result(log_session_id, result);
  close_session_log();
  printf("EAR_CAM_PIPE:\tsession saved to %s\n", session_log_path);
  session_count = 0;
  return status;
}
