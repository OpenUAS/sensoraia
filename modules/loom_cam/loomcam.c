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
 * @file loomcam.c
 * @brief Guarded retained-vector return processing and transactional T1 policy.
 * @details Declared model vectors, aligned observation metadata and configured
 * HOME produce bounded point proposals from explicitly declared model vectors.
 * Outbound references are archived; reference-image matching is not implemented.
 */
#include "camera_backend.h"
#include "loom_return.h"
#include "loom_features.h"
#include "../../src/inference_settings.h"
#include "../../src/inference_model.h"
#include "../../src/inference_jpeg.h"
#include "../../src/exifite.h"
#include "../../src/imx500_tensor.h"

#include <errno.h>
#include <inttypes.h>
#include <math.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

/** @brief The CLI accepts decimal photo numbers, never signed or truncated IDs. */
#define LOOM_DECIMAL_RADIX 10
/** @brief argv[0] is the program name; a sole flag gives two total arguments. */
enum loom_cli_layout { LOOM_FIRST_OPTION_INDEX = 1, LOOM_SINGLE_OPTION_ARGC = 2 };
enum { LOOM_CONFIRMATION_TIMEOUT_MS = 4000 };
/** @brief Nonzero fixtures deliberately expose failure to clear previous state. */
#define LOOM_SELF_TEST_SEQUENCE UINT32_C(1)
/** @brief Nonzero latitude fixture checks that no-match clears stale evidence. */
#define LOOM_SELF_TEST_LATITUDE INT32_C(1)
/** @brief First integer outside the accepted uint32_t correlation range. */
#define LOOM_SELF_TEST_SEQUENCE_OVERFLOW "4294967296"

/**
 * @brief Borrowed per-invocation input; no member owns or acquires a camera.
 * @details The selected model/descriptor identifies the current photo's retained
 * vector. The outbound reference path is carried by the invocation but is not
 * read or compared by this vector adapter.
 */
struct loom_frame {
  const char *image;      /**< Readable saved photograph, immutable during matching. */
  const char *references; /**< Optional outbound reference directory; not compared. */
  const char *model;      /**< Optional model config used to validate this photo's vector. */
  const char *settings;   /**< Executable-owned settings, resolved through common code. */
  uint32_t sequence;     /**< Positive FC photo number echoed in every result. */
};

static bool return_settings(const char *path, struct inference_settings *settings,
                             struct loom_return_options *options);

/** @brief Read one bounded observation field from an already-fetched EXIF description. */
static bool description_lookup(const struct exif_value *description, const char *key,
                               char value[64], bool *found)
{
  *found = false;
  if (!description || description->format != EXIF_ASCII || !description->data) return false;
  const size_t length = strlen(key);
  char selected[64];
  size_t selected_length = 0;
  for (size_t index = 0; index + length < description->size; ++index) {
    if (index && description->data[index - 1] != ' ' && description->data[index - 1] != ';') continue;
    if (memcmp(description->data + index, key, length)) continue;
    if (*found) return false;
    size_t used = 0, cursor = index + length;
    while (cursor < description->size && description->data[cursor] && description->data[cursor] != ';' &&
           description->data[cursor] != ' ') {
      if (used + 1 == 64) return false;
      selected[used++] = (char)description->data[cursor++];
    }
    if (!used) return false;
    selected[used] = '\0';
    selected_length = used;
    *found = true;
  }
  if (*found) {
    for (size_t index = 0; index <= selected_length; ++index) value[index] = selected[index];
  }
  return true;
}

/** @brief Require a uniquely identified flight-description value.
 * @param description EXIF description to scan.
 * @param key Exact field prefix including the equals sign.
 * @param[out] value Buffer for up to 63 characters and the terminator.
 * @return True only if a well-formed matching field was found.
 * @details Use the duplicate-aware lookup even when absence is an error. */
static bool description_field(const struct exif_value *description, const char *key, char value[64])
{
  bool found = false;
  return description_lookup(description, key, value, &found) && found;
}

/** @brief Read a required finite numeric metadata field.
 * @param description EXIF description to scan.
 * @param key Exact field prefix including the equals sign.
 * @param[out] number Parsed value, written on success.
 * @return True for a present, fully parsed number.
 * @details Reject missing and malformed fields alike when no default is allowed. */
static bool description_number(const struct exif_value *description, const char *key, double *number)
{
  char value[64];
  return description_field(description, key, value) && ini_real(value, number);
}

/** @brief Read numeric metadata with a default for absence only.
 * @param description EXIF description to scan.
 * @param key Exact field prefix including the equals sign.
 * @param default_value Result if the key is absent.
 * @param[out] number Parsed or default value on success.
 * @return False for malformed or duplicate evidence, even if a default exists.
 * @details Keep corrupt evidence distinct from older records missing an optional field. */
static bool description_optional_number(const struct exif_value *description, const char *key,
                                        double default_value, double *number)
{
  char value[64];
  bool found = false;
  if (!description_lookup(description, key, value, &found)) return false;
  if (!found) {
    *number = default_value;
    return true;
  }
  return ini_real(value, number);
}

/** @brief Read the graph and descriptor identities declared by the model metadata.
 * @param settings Selected model and descriptor paths.
 * @param[out] graph Buffer for the declared graph digest.
 * @param[out] descriptor Buffer for the descriptor digest.
 * @return True after parsing the bounded descriptor and checking model readability.
 * @details Read descriptor identity without creating a runtime session. This
 * helper does not hash the graph file itself. */
static bool read_onnx_descriptor_identity(const struct inference_settings *settings,
                                          char graph[65], char descriptor[65])
{
  struct stat model_status;
  if (!settings || !settings->model[0] || !settings->descriptor[0] ||
      stat(settings->model, &model_status) != 0 || !S_ISREG(model_status.st_mode) ||
      model_status.st_size <= 0 || access(settings->model, R_OK) != 0) return false;
  static char contents[16385];
  static struct inference_model model;
  char error[256];
  size_t length = 0;
  if (!ini_read_file(settings->descriptor, contents, sizeof(contents), &length) ||
      !inference_model_parse(&model, contents, length, error, sizeof(error))) return false;
  for (size_t index = 0; index <= 64; ++index) {
    graph[index] = model.graph_sha256[index];
    descriptor[index] = model.descriptor_sha256[index];
  }
  return true;
}

/** @brief Decode an unsigned field from LOOM's little-endian record.
 * @param bytes At least eight readable bytes.
 * @return Decoded unsigned value.
 * @details Assemble bytes rather than reading a native word at a potentially
 * unaligned record offset. */
static uint64_t feature_u64(const unsigned char *bytes)
{
  uint64_t number = 0;
  for (unsigned index = 0; index < 8; ++index) number |= (uint64_t)bytes[index] << (8 * index);
  return number;
}

/** @brief Generate bounded LOOM feature evidence for an existing JPEG.
 * @param settings ONNX model assignment and runtime configuration.
 * @param options Contract that governs acceptable output shape.
 * @param image Input JPEG path.
 * @param sequence Frame identity to embed.
 * @param timestamp_ns Sensor timestamp in nanoseconds to embed.
 * @param[out] record Destination for packed feature evidence.
 * @param capacity Available bytes in @p record.
 * @param[out] record_size Packed length; use only on success.
 * @return True after successful inference and packing.
 * @details Validate tensor shape before inference, enforce the stricter
 * direction-vector shape when required, and serialize before closing the session.
 * Calls must not overlap because the context is shared. */
static bool run_onnx_in_memory(const struct inference_settings *settings,
                               const struct loom_return_options *options,
                               const char *image, uint64_t sequence, uint64_t timestamp_ns,
                               unsigned char *record, size_t capacity, size_t *record_size)
{
  static struct inference_context context;
  char error[512];
  context = (struct inference_context){0};
  if (!inference_context_open(&context, settings, "loom_features_v1", error, sizeof(error))) return false;
  size_t values = 0;
  bool valid_shape = context.model.input_count == 1 && context.model.image.channels != 0;
  for (size_t index = 0; valid_shape && index < context.model.output_count; ++index) {
    const struct onnx_tensor *tensor = &context.model.outputs[index];
    if (tensor->type != ONNX_FLOAT32 || tensor->bytes / sizeof(float) > 4096 - values) valid_shape = false;
    else values += tensor->bytes / sizeof(float);
  }
  if (options->frame != LOOM_RETURN_NONE && options->frame != LOOM_RETURN_SIMULATED &&
      (context.model.output_count != 1 || (values != 2 && values != 3))) valid_shape = false;
  bool ok = valid_shape &&
            inference_jpeg_prepare(&context, image, error, sizeof(error)) &&
            inference_context_run(&context, error, sizeof(error));
  if (ok) {
    *record_size = loom_onnx_features_pack(record, capacity, &context.model, sequence, timestamp_ns);
    ok = *record_size != 0;
  }
  inference_context_close(&context);
  return ok;
}

/**
 * @brief Interpret retained same-photo neural vectors under the declared return contract.
 * @param frame Current saved AIcam photo and reference/model configuration.
 * @param point Required destination in camera-point wire units.
 * @return False when model/pose/identity/direction validation rejects the photo.
 * @warning Never replace no-match with the current pose, home position or guessed
 * direction: those would look like valid navigation evidence on the FC.
 * @details Generic embeddings are never guessed to be return directions. Only
 * the PC simulation harness may turn them into an explicitly marked mock vector.
 * The persistent owner subsequently limits movement and suppresses stale phases.
 */
static bool match_previous_photos(const struct loom_frame *frame, struct camera_point_result *point)
{
  *point = (struct camera_point_result) {.session_id = frame->sequence};
  struct inference_settings settings;
  struct loom_return_options options;
  if (!return_settings(frame->settings, &settings, &options) || options.frame == LOOM_RETURN_NONE) return false;
  static unsigned char payload[EXIF_MAX_PAYLOAD], information[IMX500_TENSOR_INFO_BYTES];
  static unsigned char onnx_record[LOOM_ONNX_FEATURE_CAPACITY];
  static float features[4096];
  size_t size = 0;
  struct exif_view view;
  struct exif_value description;
  if (exif_read_jpeg(frame->image, payload, sizeof(payload), &size) || exif_parse(&view, payload, size) ||
      !exif_get(&view, EXIF_PRIMARY, EXIF_IMAGE_DESCRIPTION, &description) ||
      description.format != EXIF_ASCII || !description.data)
    return false;
  char aligned_value[64];
  bool aligned_found = false;
  if (!description_lookup(&description, "observation_pose_aligned=", aligned_value, &aligned_found)) return false;
  if (aligned_found && strcmp(aligned_value, "true") && strcmp(aligned_value, "false")) return false;
  const bool aligned = aligned_found && !strcmp(aligned_value, "true");
  const unsigned home_fields = (1U << 5) | (1U << 6);
  const bool home_configured = (options.seen & home_fields) == home_fields;
  double latitude = 0, longitude = 0, altitude = 0, agl = 0, roll = 0, pitch = 0, yaw = 0;
  if (aligned) {
    if (!description_number(&description, "observation_lat_deg=", &latitude) || fabs(latitude) > 85 ||
        !description_number(&description, "observation_lon_deg=", &longitude) || fabs(longitude) > 180 ||
        !description_number(&description, "observation_alt_m=", &altitude) || fabs(altitude) > 2000000 ||
        !description_number(&description, "observation_agl_m=", &agl) || agl < 0 || agl > 10000 ||
        !description_optional_number(&description, "observation_roll_deg=", 0, &roll) ||
        !description_optional_number(&description, "observation_pitch_deg=", 0, &pitch) ||
        !description_number(&description, "observation_yaw_deg=", &yaw)) return false;
  } else {
    if (!description_optional_number(&description, "lat_deg=",
                                     home_configured ? options.home_latitude : NAN, &latitude) ||
        !isfinite(latitude) || fabs(latitude) > 85 ||
        !description_optional_number(&description, "lon_deg=",
                                     home_configured ? options.home_longitude : NAN, &longitude) ||
        !isfinite(longitude) || fabs(longitude) > 180 ||
        !description_number(&description, "alt_m=", &altitude) || fabs(altitude) > 2000000 ||
        !description_number(&description, "groundalt_m=", &agl) || agl < 0 || agl > 10000 ||
        !description_optional_number(&description, "roll_deg=", 0, &roll) ||
        !description_optional_number(&description, "pitch_deg=", 0, &pitch) ||
        !description_number(&description, "yaw_deg=", &yaw)) return false;
    if (!latitude && !longitude && home_configured) {
      latitude = options.home_latitude;
      longitude = options.home_longitude;
    }
  }
  if (!isfinite(roll) || !isfinite(pitch) || !isfinite(yaw) ||
      fabs(roll) > options.limits.maximum_tilt_deg ||
      fabs(pitch) > options.limits.maximum_tilt_deg ||
      fabs(yaw) > 36000) return false;
  char identity[64];
  uint64_t observed_sequence = 0, observed_time = 0, shot_number = 0;
  if (!description_field(&description, "observation_sequence=", identity) || !ini_uint(identity, &observed_sequence) ||
      !description_field(&description, "sensor_timestamp_ns=", identity) || !ini_uint(identity, &observed_time))
    return false;
  if (observed_sequence) {
    if (observed_sequence != frame->sequence) return false;
  } else if (!description_field(&description, "nr=", identity) || !ini_uint(identity, &shot_number) ||
             shot_number != frame->sequence) {
    return false;
  }
  float vector[3];
  uint64_t feature_sequence = 0, feature_time = 0;
  struct exif_value record;
  if (settings.backend == INFERENCE_ONNX) {
    char graph[65], descriptor[65];
    if (!read_onnx_descriptor_identity(&settings, graph, descriptor)) return false;
    if (!exif_get(&view, EXIF_CAMERA, LOOM_ONNX_FEATURES_TAG, &record)) {
      size_t packed_size = 0;
      if (!run_onnx_in_memory(&settings, &options, frame->image,
                              observed_sequence ? observed_sequence : frame->sequence,
                              observed_time, onnx_record, sizeof(onnx_record), &packed_size))
        return false;
      record.data = onnx_record;
      record.size = packed_size;
    }
    if (options.frame == LOOM_RETURN_SIMULATED) {
      if (!loom_onnx_features_valid(record.data, record.size, graph, descriptor)) return false;
    } else if (!loom_onnx_return_vector(record.data, record.size, graph, descriptor, vector)) {
      return false;
    }
    feature_sequence = feature_u64(record.data + 136);
    feature_time = feature_u64(record.data + 144);
  } else if (settings.backend == INFERENCE_IMX500) {
    size_t count = 0;
    uint64_t sequence, sensor_timestamp_ns;
    if (!exif_get(&view, EXIF_CAMERA, LOOM_FEATURES_TAG, &record) ||
        !loom_features_unpack(record.data, record.size, features, 4096, information, sizeof(information),
                              &count, &sequence, &sensor_timestamp_ns) ||
        !count || (options.frame != LOOM_RETURN_SIMULATED && count != 2 && count != 3) ||
        !sequence || !sensor_timestamp_ns) return false;
    if (options.frame != LOOM_RETURN_SIMULATED) {
      vector[0] = features[0]; vector[1] = features[1]; vector[2] = count == 3 ? features[2] : 1;
    }
    feature_sequence = sequence;
    feature_time = sensor_timestamp_ns;
  } else return false;
  if (feature_time != observed_time ||
      feature_sequence != (observed_sequence ? observed_sequence : frame->sequence))
    return false;
  const bool use_home_position = (aligned || options.frame == LOOM_RETURN_SIMULATED) && home_configured;
  const double origin_latitude = use_home_position ? options.home_latitude : latitude;
  const double origin_longitude = use_home_position ? options.home_longitude : longitude;
  const double radians = 0.017453292519943295, earth = 6378137;
  const double scale = cos(origin_latitude * radians);
  const struct loom_return_point position = use_home_position ? (struct loom_return_point){
    earth * scale * remainder(longitude - origin_longitude, 360) * radians,
    earth * (latitude - origin_latitude) * radians
  } : (struct loom_return_point){0, 0};
  if (options.frame == LOOM_RETURN_SIMULATED) {
    uint64_t seed = UINT64_C(1469598103934665603) ^ frame->sequence;
    for (size_t index = 0; index < record.size; ++index)
      seed = (seed ^ record.data[index]) * UINT64_C(1099511628211);
    loom_return_simulated_vector(seed, position, yaw, vector);
  }
  struct loom_return_point direction, target;
  if (!loom_return_vector_attitude(options.frame, vector[0], vector[1], roll, pitch, yaw,
                                   options.limits.maximum_tilt_deg, &direction) ||
      !loom_return_direction(&options.limits, position, direction, vector[2], &target)) return false;
  const double target_latitude = origin_latitude + target.north / earth / radians;
  const double target_longitude = remainder(origin_longitude + target.east / (earth * scale) / radians, 360);
  if (!isfinite(target_latitude) || !isfinite(target_longitude) ||
      fabs(target_latitude) > 85 || fabs(target_longitude) > 180 ||
      (!llround(target_latitude * 1e7) && !llround(target_longitude * 1e7))) return false;
  const double step_m = hypot(target.east - position.east, target.north - position.north);
  const bool arrived = step_m <= options.limits.maximum_step_m;
  *point = (struct camera_point_result){.session_id = frame->sequence, .valid = true,
    .lat = (int32_t)llround(target_latitude * 1e7), .lon = (int32_t)llround(target_longitude * 1e7),
    .alt_mm = (int32_t)llround(altitude * 1000), .agl_mm = (int32_t)llround(agl * 1000),
    .quality = (int32_t)llround(vector[2] * 1000),
    .metric = arrived ? LOOM_METRIC_ARRIVED
                      : (use_home_position ? LOOM_METRIC_STEERING : LOOM_METRIC_BARO_MAGNETO)};
  return true;
}

/**
 * @brief Parse a nonzero correlation sequence without truncation or signed input.
 * @param text Entire decimal token; no whitespace, sign or suffix is accepted.
 * @param[out] sequence Destination, written only on successful validation.
 * @return True for a positive uint32_t photo number; false otherwise.
 */
static bool parse_sequence(const char *text, uint32_t *sequence)
{
  if (!text || *text < '0' || *text > '9') return false;
  char *end = NULL;
  errno = 0;
  const unsigned long value = strtoul(text, &end, LOOM_DECIMAL_RADIX);
  if (errno || *end || !value || value > UINT32_MAX) return false;
  *sequence = (uint32_t)value;
  return true;
}

/** @brief Reuse common path/INI policy without opening a model or camera. */
static bool return_settings(const char *path, struct inference_settings *settings,
                             struct loom_return_options *options)
{
  if (path && !strcmp(path, "-")) path = NULL;
  char discovered[INFERENCE_PATH_CAPACITY], error[2048];
  inference_settings_init(settings, INFERENCE_AUTO);
  loom_return_options_init(options);
  if (!path) {
    if (!inference_settings_find("loomcam", "loom_cam", discovered, sizeof(discovered))) return true;
    path = discovered;
  }
  if (inference_settings_read(settings, path, loom_return_setting, options, error, sizeof(error))) {
    if (loom_return_options_valid(options)) return true;
    fprintf(stderr, "loomcam: inconsistent return limits\n");
    return false;
  }
  fprintf(stderr, "loomcam: %s\n", error);
  return false;
}

/** @brief Describe whether PC imagery can run a selected ONNX graph. */
static int simulation_mode(int argc, char **argv)
{
  const char *path = NULL;
  bool strict = false;
  for (int index = 2; index < argc; ++index) {
    if (!strcmp(argv[index], "--strict") && !strict) strict = true;
    else if (!strcmp(argv[index], "--settings") && !path && ++index < argc) path = argv[index];
    else return 2;
  }
  struct inference_settings settings;
  struct loom_return_options options;
  if (!return_settings(path, &settings, &options)) return 1;
  if (strict && (settings.backend != INFERENCE_ONNX || !settings.model[0] ||
                 (options.frame != LOOM_RETURN_NORTH_EAST &&
                  options.frame != LOOM_RETURN_BODY_FORWARD_RIGHT))) {
    fprintf(stderr, "loomcam: strict return requires an ONNX model and a declared return_vector_frame\n");
    return 1;
  }
  if (!settings.model[0] || settings.backend == INFERENCE_IMX500) {
    puts(settings.model[0] ? "RANDOM rpk_requires_ai_camera" : "RANDOM model_not_set");
    return 0;
  }
  if (settings.backend != INFERENCE_ONNX) return 1;
  struct stat status;
  if (stat(settings.model, &status)) {
    if (!strict && (errno == ENOENT || errno == ENOTDIR)) { puts("RANDOM onnx_model_missing"); return 0; }
    fprintf(stderr, "loomcam: cannot access ONNX model: %s\n", strerror(errno));
    return 1;
  }
  if (!S_ISREG(status.st_mode) || access(settings.model, R_OK)) return 1;
  puts("ONNX");
  return 0;
}

/** @brief Split one bounded ASCII command, rejecting binary/trailing/oversized input. */
static int return_line(char *line, size_t capacity, unsigned timeout_ms)
{
  struct timespec started;
  if (timeout_ms && clock_gettime(CLOCK_MONOTONIC, &started)) return -1;
  size_t length = 0;
  for (;;) {
    if (timeout_ms) {
      struct timespec now;
      if (clock_gettime(CLOCK_MONOTONIC, &now)) return -1;
      const int64_t elapsed = (int64_t)(now.tv_sec - started.tv_sec) * 1000 +
                              (now.tv_nsec - started.tv_nsec) / 1000000;
      if (elapsed < 0 || elapsed >= timeout_ms) return -1;
      struct pollfd input = {.fd = STDIN_FILENO, .events = POLLIN};
      const int ready = poll(&input, 1, (int)(timeout_ms - elapsed));
      if (ready < 0 && errno == EINTR) continue;
      if (ready <= 0) return -1;
    }
    const int character = fgetc(stdin);
    if (character == EOF) return length ? -1 : 0;
    if (character == '\n') { line[length] = '\0'; return length ? 1 : -1; }
    if (character < 32 || character >= 127 || length + 1 >= capacity) return -1;
    line[length++] = (char)character;
  }
}

/** @brief Keep bounded T1 state separate from the waypoint transport transaction. */
static int return_server(int argc, char **argv)
{
  const char *path = NULL;
  bool simulation = false;
  uint32_t seed = 19;
  for (int index = 2; index < argc; ++index) {
    if (!strcmp(argv[index], "--simulation") && !simulation) simulation = true;
    else if (!strcmp(argv[index], "--settings") && !path && ++index < argc) path = argv[index];
    else if (!strcmp(argv[index], "--seed") && ++index < argc && parse_sequence(argv[index], &seed)) {}
    else return 2;
  }
  struct inference_settings settings;
  struct loom_return_options options;
  if (!return_settings(path, &settings, &options)) return 1;
  struct loom_return_state state = {.random_state = seed};
  uint32_t previous = 0;
  if (setvbuf(stdin, NULL, _IONBF, 0)) return 1;
  puts("READY LOOM_RETURN_V1");
  if (fflush(stdout)) return 1;
  char line[256];
  int line_status;
  while ((line_status = return_line(line, sizeof(line), 0)) > 0) {
    char *fields[10];
    unsigned count = 1;
    fields[0] = line;
    for (char *cursor = line; *cursor; ++cursor) {
      if (*cursor != ' ') continue;
      *cursor = '\0';
      if (count == 10 || !cursor[1] || cursor[1] == ' ') return 2;
      fields[count++] = cursor + 1;
    }
    uint32_t sequence;
    if (count < 2 || !parse_sequence(fields[1], &sequence) || sequence <= previous) return 2;
    previous = sequence;
    const bool simulated_request = !strcmp(fields[0], "SIM");
    const bool mocked_observation = !strcmp(fields[0], "SIMOBS");
    const bool model_observation = !strcmp(fields[0], "OBS");
    if ((!simulated_request && !mocked_observation && !model_observation) ||
        (model_observation && count != 8 && count != 10) || (mocked_observation && (!simulation || count != 7)) ||
        (simulated_request && (count != 2 || !simulation))) return 2;
    double values[8] = {0};
    if (model_observation || mocked_observation) {
      const unsigned value_count = model_observation ? count - 2 : 5;
      for (unsigned index = 0; index < value_count; ++index)
        if (!ini_real(fields[index + 2], &values[index])) return 2;
    }
    struct loom_return_state proposed = state;
    struct loom_return_point target, direction;
    bool accepted_vector = false;
    if (model_observation) {
      accepted_vector = loom_return_vector_attitude(options.frame, values[2], values[3],
                                                    values[6], values[7], values[5],
                                                    options.limits.maximum_tilt_deg, &direction) &&
        loom_return_direction(&options.limits, (struct loom_return_point){values[0], values[1]},
                              direction, values[4], &target);
    } else if (mocked_observation) {
      direction = (struct loom_return_point){values[2], values[3]};
      accepted_vector = loom_return_direction(&options.limits,
        (struct loom_return_point){values[0], values[1]}, direction, values[4], &target);
    }
    accepted_vector = accepted_vector &&
      loom_return_update(&proposed, &options.limits, sequence, target, &target);
    const bool simulated_fallback = !accepted_vector && !mocked_observation && simulation &&
      loom_return_simulate(&proposed, &options.limits, sequence, &target);
    if (!accepted_vector && !simulated_fallback) {
      printf("SKIP %" PRIu32 " invalid_direction\n", sequence);
      if (fflush(stdout)) return 1;
      continue;
    }
    printf("MOVE %" PRIu32 " %.9f %.9f %s\n", sequence, target.east, target.north,
           accepted_vector ? (mocked_observation ? "simulated" : "neural") : "simulated");
    if (fflush(stdout)) return 1;
    char expected[64];
    snprintf(expected, sizeof(expected), "ACK %" PRIu32, sequence);
    if (return_line(line, sizeof(line), LOOM_CONFIRMATION_TIMEOUT_MS) != 1) return 1;
    if (!strcmp(line, expected)) {
      state = proposed;
      printf("OK %" PRIu32 "\n", sequence);
    } else printf("SKIP %" PRIu32 " unconfirmed\n", sequence);
    if (fflush(stdout)) return 1;
  }
  return line_status < 0 || ferror(stdin) ? 2 : 0;
}

/**
 * @brief Hardware-free check that the empty matcher cannot manufacture a waypoint.
 * @return EXIT_SUCCESS when clearing, correlation and invalid-ID rejection hold.
 * @note The image is not opened here; this tests the stub contract, not capture.
 */
static int self_test(void)
{
  const struct loom_frame frame = {.image = "fixture.jpg", .sequence = LOOM_SELF_TEST_SEQUENCE};
  struct camera_point_result point = {.valid = true, .lat = LOOM_SELF_TEST_LATITUDE};
  uint32_t sequence = 0;
  if (match_previous_photos(&frame, &point) || point.valid || point.lat
      || point.session_id != frame.sequence || !parse_sequence("1", &sequence)
      || parse_sequence("0", &sequence) || parse_sequence("-1", &sequence)
      || parse_sequence(LOOM_SELF_TEST_SEQUENCE_OVERFLOW, &sequence)) return EXIT_FAILURE;
  puts("LOOM return: absent vector evidence and sequence validation passed");
  return EXIT_SUCCESS;
}

/**
 * @brief Parse one saved-photo request and emit exactly one correlated result line.
 * @param argc Number of program-name and option/value arguments.
 * @param argv Borrowed command-line tokens; paths are passed without a shell.
 * @return EXIT_SUCCESS for a processed photo, including no-match; EXIT_FAILURE
 * for invalid input or output failure. Diagnostics use stderr, results stdout.
 * @details The embedded entry point shares the C++ capture executable but does
 * not acquire the sensor. A syntactically successful reply is not a valid match.
 */
#ifdef LOOMCAM_EMBEDDED
  int loomcam_photo_main(int argc, char **argv)
#else
  int main(int argc, char **argv)
#endif
{
  if (argc > 1 && !strcmp(argv[1], "--return-server")) return return_server(argc, argv);
  if (argc > 1 && !strcmp(argv[1], "--simulation-mode")) return simulation_mode(argc, argv);
  struct loom_frame frame = {0};
  for (int index = LOOM_FIRST_OPTION_INDEX; index < argc; ++index) {
    if (strcmp(argv[index], "--self-test") == 0 && argc == LOOM_SINGLE_OPTION_ARGC) return self_test();
    if (strcmp(argv[index], "--help") == 0 && argc == LOOM_SINGLE_OPTION_ARGC) {
       puts("Usage: loomcam --image FILE --sequence N [--reference-dir DIR] [--model FILE]\n"
         "Uses this photo's retained ONNX/RPK vector when configured; outbound references are not compared.");
      return EXIT_SUCCESS;
    }
    const char *option = argv[index];
    if (++index >= argc) return EXIT_FAILURE;
    const char *value = argv[index];
    if (strcmp(option, "--image") == 0) frame.image = value;
    else if (strcmp(option, "--settings") == 0) frame.settings = value;
    else if (strcmp(option, "--reference-dir") == 0) frame.references = value;
    else if (strcmp(option, "--model") == 0) frame.model = value;
    else if (strcmp(option, "--sequence") == 0) {
      if (!parse_sequence(value, &frame.sequence)) return EXIT_FAILURE;
    } else return EXIT_FAILURE;
  }
  struct stat status;
  /* Require one readable saved photo; output identity and capture alignment are
   * separately validated before a retained vector can authorize a point. */
  if (!frame.image || !frame.sequence || stat(frame.image, &status) != 0
      || !S_ISREG(status.st_mode) || access(frame.image, R_OK) != 0) {
    fprintf(stderr, "loomcam: a readable photo and nonzero sequence are required\n");
    return EXIT_FAILURE;
  }
  struct camera_point_result point;
  /* Processing success and match validity are different outcomes. Echo the
   * requested photo number even on no-match so the parent can reject stale replies. */
  point.valid = match_previous_photos(&frame, &point);
  printf("LOOM_POINT_V1 sequence=%" PRIu32 " valid=%d lat=%" PRId32 " lon=%" PRId32
         " alt_mm=%" PRId32 " agl_mm=%" PRId32 " quality=%" PRId32 " metric=%" PRId32 "\n",
         point.session_id, point.valid, point.lat, point.lon, point.alt_mm,
         point.agl_mm, point.quality, point.metric);
  return ferror(stdout) ? EXIT_FAILURE : EXIT_SUCCESS;
}