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
 * @file spotter_cam_pipe.c
 * @brief Native USHER adapter for on-sensor IMX500 object detection.
 * @details The native Spottercam worker owns capture. This adapter checks
 * its bounded object sidecar and retains the highest-confidence box for EXIF/SODA.
 * Missing analysis does not delete a useful photograph, and a valid empty result
 * is not a hit. Review copies happen only after USHER has tagged the original.
 * Shared process supervision and frame timing deliberately remain separate from
 * detector interpretation; LOOM does not use this adapter's result semantics.
 */

#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <math.h>
#include <poll.h>
#include <signal.h>
#include <spawn.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

#include "spotter_cam_pipe.h"
#include "camera_backend.h"
#include "imx500_pipe.h"
#include "path_utils.h"

#include "bounded_ini.h"
#include "sha256fst.h"
#include "exifite.h"

/** Defaults to the same directory as plain AICam (USHER_AI_CAM_PHOTO_DIR, already
 * defined globally via the build's CFLAGS): --spottercam is still conceptually "the
 * AI camera", just with detection added, so it belongs in the same photo directory. */
#ifndef USHER_SPOTTERCAM_PHOTO_DIR
  #define USHER_SPOTTERCAM_PHOTO_DIR USHER_AI_CAM_PHOTO_DIR
#endif

/** A sidecar is generated locally; 32 KiB bounds 64 boxes with precise centres. */
#define OBJECT_METADATA_LIMIT 32768
/** Bound candidate records even when a producer sends a syntactically valid oversized array. */
#define OBJECT_MAX_DETECTIONS 64
/** Bound labels copied from model metadata into the fixed EXIF summary. */
#define OBJECT_LABEL_SIZE 64
/** Stream large photo copies in fixed chunks instead of allocating image-sized RAM. */
#define OBJECT_COPY_BUFFER_SIZE (64U * 1024U)

#ifndef USHER_SPOTTERCAM_COMMAND
  /** @brief Standalone object-worker executable, never the LOOM entry point. */
  #define USHER_SPOTTERCAM_COMMAND "./bin/spottercam"
#endif
#ifndef USHER_SPOTTERCAM_MODEL
  /** @brief Native detector RPK path borrowed for the worker's lifetime. */
  #define USHER_SPOTTERCAM_MODEL "models/ai_cam/nn_vehicles.rpk"
#endif

static struct imx500_pipe spotter_capture = IMX500_PIPE_INITIALIZER; /**< Exclusive native-detector process state. */
static struct imx500_pipe spotter_analysis = IMX500_PIPE_INITIALIZER;
static const char *spotter_settings;
static bool spotter_photo_only;

static const char *photo_directory = USHER_SPOTTERCAM_PHOTO_DIR; /**< Borrowed configured photo directory. */

/** Validity distinguishes an empty detection result from missing/corrupt analysis. */
struct object_detection_result {
  bool valid; /**< Sidecar validated against the image and schema. */
  bool hash_bound;
  bool has_frame_identity; /**< Sequence/time were both present and exactly typed. */
  /** @var object_detection_result::sequence
   * @brief Producer camera sequence, not an FC photo number. */
  uint64_t sequence, sensor_timestamp_ns; /**< Sensor timestamp paired with that producer sequence. */
  int count; /**< Number of object boxes in the bounded result. */
  double conf; /**< Confidence of the retained best box. */
  char label[OBJECT_LABEL_SIZE]; /**< Model-provided label for the retained box. */
  /** @var object_detection_result::box_x
   * @brief Selected box's left edge in JPEG pixels. */
  /** @var object_detection_result::box_y
   * @brief Selected box's top edge in JPEG pixels. */
  /** @var object_detection_result::box_w
   * @brief Selected box's validated JPEG pixel width. */
  int box_x, box_y, box_w, box_h; /**< Selected box's validated JPEG pixel height. */
  double center_x, center_y;
};
static struct object_detection_result last_detection; /**< Current shot only; cleared on capture/binding failure. */

void spotter_cam_pipe_configure(const struct camera_backend_host *host)
{
  spotter_capture.running = host ? host->running : NULL;
  spotter_analysis.running = host ? host->running : NULL;
}

int spotter_cam_pipe_set_photo_only(bool enabled)
{
  if (spotter_capture.pid > 0 || spotter_analysis.pid > 0) return -1;
  spotter_photo_only = enabled;
  return 0;
}

/** @brief Report whether capture was configured without analysis.
 * @return True in photo-only mode.
 * @details This is a configuration check, not a camera readiness check. */
bool spotter_cam_pipe_photo_only(void)
{
  return spotter_photo_only;
}

/** Build the sidecar name by replacing the final `.jpg` suffix.
 * @param filename Completed JPEG filename.
 * @param path Caller-owned output buffer.
 * @param path_size Output capacity.
 * @return Zero on a bounded JPEG path, otherwise -1. */
static int metadata_path(const char *filename, char *path, size_t path_size)
{
  if (!filename || !path || !path_size) return -1;
  size_t filename_length = strnlen(filename, PATH_MAX);
  if (filename_length < 4 || filename_length >= PATH_MAX || strcmp(filename + filename_length - 4, ".jpg") != 0) {
    return -1;
  }
  int length = snprintf(path, path_size, "%.*s.objects.ini",
                        (int)(filename_length - 4), filename);
  return length >= 0 && (size_t)length < path_size ? 0 : -1;
}

struct detection_fields {
  struct object_detection_result objects[OBJECT_MAX_DETECTIONS];
  unsigned seen[OBJECT_MAX_DETECTIONS];
  unsigned header_seen;
  uint64_t sequence, sensor_timestamp_ns, width, height, count;
  unsigned hashes_seen;
  char image_hash[SHA256_HEX_SIZE];
  char model_hash[SHA256_HEX_SIZE], descriptor_hash[SHA256_HEX_SIZE];
};

/** @brief Parse one bounded detector sidecar field.
 * @param[in,out] context Pointer to the parser's detection_fields accumulator.
 * @param key Current key from the INI reader.
 * @param value Associated text value.
 * @return True when the field is accepted.
 * @details Track field presence independently of values so duplicate
 * metadata or box fields cannot replace the first reading silently. */
static bool detection_field(void *context, const char *key, const char *value)
{
  struct detection_fields *fields = context;
  const char *const names[] = {"schema", "sequence", "sensor_timestamp_ns", "image_width", "image_height", "count"};
  size_t field = 0;
  while (field < 6 && strcmp(key, names[field])) ++field;
  if (field < 6) {
    if (fields->header_seen & (1U << field)) return false;
    fields->header_seen |= 1U << field;
    if (!field) return !strcmp(value, "USHER_OBJECTS_V2");
    uint64_t parsed;
    if (!ini_uint(value, &parsed)) return false;
    if (field == 1) fields->sequence = parsed;
    if (field == 2) fields->sensor_timestamp_ns = parsed;
    if (field == 3) fields->width = parsed;
    if (field == 4) fields->height = parsed;
    if (field == 5) fields->count = parsed;
    return true;
  }
  const char *const hashes[] = {"model_sha256", "model_descriptor_sha256", "image_sha256"};
  for (unsigned index = 0; index < 3; ++index) {
    if (strcmp(key, hashes[index])) continue;
    if ((fields->hashes_seen & (1U << index)) || strlen(value) != 64 || strspn(value, "0123456789abcdef") != 64) return false;
    fields->hashes_seen |= 1U << index;
    char *destination = index == 0 ? fields->model_hash : index == 1 ? fields->descriptor_hash : fields->image_hash;
    for (size_t character = 0; character < SHA256_HEX_SIZE; ++character) destination[character] = value[character];
    return true;
  }
  if (strncmp(key, "detection.", 10)) return false;
  const char *suffix = strchr(key + 10, '.');
  if (!suffix || suffix == key + 10 || (size_t)(suffix - key - 10) >= 3) return false;
  char index_text[3] = {0};
  for (size_t index = 0; index < (size_t)(suffix - key - 10); ++index) index_text[index] = key[10 + index];
  uint64_t object_index;
  if (!ini_uint(index_text, &object_index) || object_index >= OBJECT_MAX_DETECTIONS) return false;
  const char *const object_names[] = {"name", "confidence", "x", "y", "width", "height", "center_x", "center_y"};
  field = 0;
  while (field < 8 && strcmp(suffix + 1, object_names[field])) ++field;
  if (field == 8 || (fields->seen[object_index] & (1U << field))) return false;
  fields->seen[object_index] |= 1U << field;
  struct object_detection_result *object = &fields->objects[object_index];
  if (!field) {
    const size_t length = strlen(value);
    if (length >= sizeof(object->label)) return false;
    for (size_t index = 0; index < length; ++index) {
      const unsigned char character = (unsigned char)value[index];
      if (character < 32 || character == 127 || character == ';') return false;
    }
    for (size_t index = 0; index <= length; ++index) object->label[index] = value[index];
    return true;
  }
  if (field == 1) return ini_real(value, &object->conf) && object->conf >= 0 && object->conf <= 1;
  if (field >= 6) return ini_real(value, field == 6 ? &object->center_x : &object->center_y);
  uint64_t parsed;
  if (!ini_uint(value, &parsed) || parsed > INT_MAX) return false;
  if (field == 2) object->box_x = (int)parsed;
  if (field == 3) object->box_y = (int)parsed;
  if (field == 4) object->box_w = (int)parsed;
  if (field == 5) object->box_h = (int)parsed;
  return true;
}

/** @brief Read a stable bounded sidecar and validate schema, dimensions, and every box.
 * @param filename Matching JPEG path.
 * @param[out] result Cleared on any I/O, parse, or schema failure.
 * @param[out] detections Optional expanded detection list and provenance.
 * @return True for valid metadata, including a valid zero-detection frame.
 * @details Size/mtime checks reject ordinary changes during the read, while
 * caller-owned buffers bound parser storage. Exposure association is a later
 * exact sequence/timestamp check, not a property of INI syntax alone. */
static bool read_detection_sidecar(const char *filename, struct object_detection_result *result,
                                    struct spotter_detections *detections)
{
  if (!result) return false;
  *result = (struct object_detection_result) {0};
  if (detections) *detections = (struct spotter_detections) {0};
  char sidecar[PATH_MAX];
  if (metadata_path(filename, sidecar, sizeof(sidecar)) != 0) {
    return false;
  }
  char input[OBJECT_METADATA_LIMIT + 1];
  size_t length = 0;
  bool valid = false;
  if (!ini_read_file(sidecar, input, sizeof(input), &length)) return false;

  struct detection_fields fields = {0};
  unsigned long expected_width = strtoul(USHER_AI_CAM_WIDTH, NULL, 10);
  unsigned long expected_height = strtoul(USHER_AI_CAM_HEIGHT, NULL, 10);
  if (!ini_parse(input, length, detection_field, &fields)) goto done;
  if (fields.hashes_seen) {
    if (fields.hashes_seen != 7) goto done;
    char digest[SHA256_HEX_SIZE];
    if (!sha256_file(filename, digest) || strcmp(digest, fields.image_hash)) goto done;
    FILE *jpeg = fopen(filename, "rb");
    uint32_t width = 0, height = 0;
    unsigned components = 0;
    if (!jpeg) goto done;
    const bool dimensions = exif_jpeg_dimensions(jpeg, &width, &height, &components) == 0;
    const int closed = fclose(jpeg);
    if (!dimensions || closed || !width || !height) goto done;
    expected_width = width;
    expected_height = height;
  }
  if (fields.header_seen != 63
      || fields.width != expected_width || fields.height != expected_height
      || expected_width == 0 || expected_width > UINT_MAX
      || expected_height == 0 || expected_height > UINT_MAX
      || fields.count > OBJECT_MAX_DETECTIONS || (detections && fields.hashes_seen != 7)) {
    goto done;
  }
  *result = (struct object_detection_result) {.valid = true, .has_frame_identity = fields.sensor_timestamp_ns != 0,
    .hash_bound = fields.hashes_seen == 7,
    .sequence = fields.sequence, .sensor_timestamp_ns = fields.sensor_timestamp_ns, .count = (int)fields.count};
  for (size_t index = 0; index < OBJECT_MAX_DETECTIONS; ++index) {
    if (index >= fields.count) {
      if (fields.seen[index]) goto done;
      continue;
    }
    const struct object_detection_result *object = &fields.objects[index];
    if ((fields.seen[index] != 63 && fields.seen[index] != 255) || object->box_w <= 0 || object->box_h <= 0
        || (uint64_t)object->box_x + object->box_w > expected_width
        || (uint64_t)object->box_y + object->box_h > expected_height) goto done;
    if (fields.seen[index] == 255 && (object->center_x < object->box_x || object->center_y < object->box_y
      || object->center_x > object->box_x + object->box_w || object->center_y > object->box_y + object->box_h
      || object->center_x >= expected_width || object->center_y >= expected_height)) goto done;
    if (detections) {
      struct spotter_detection *box = &detections->objects[index];
      for (size_t character = 0; character < sizeof(box->label); ++character) box->label[character] = object->label[character];
      box->confidence = object->conf;
      box->precise_center = fields.seen[index] == 255;
      box->center_x = object->center_x; box->center_y = object->center_y;
      box->x = (unsigned)object->box_x; box->y = (unsigned)object->box_y;
      box->width = (unsigned)object->box_w; box->height = (unsigned)object->box_h;
    }
    if (!index || object->conf > result->conf) {
      result->conf = object->conf;
      for (size_t character = 0; character < sizeof(result->label); ++character) result->label[character] = object->label[character];
      result->box_x = object->box_x;
      result->box_y = object->box_y;
      result->box_w = object->box_w;
      result->box_h = object->box_h;
    }
  }
  if (detections) {
    detections->sequence = fields.sequence;
    detections->width = (unsigned)fields.width;
    detections->height = (unsigned)fields.height;
    detections->count = (unsigned)fields.count;
    for (size_t index = 0; index < sizeof(detections->image_sha256); ++index) {
      detections->image_sha256[index] = fields.image_hash[index];
      detections->model_sha256[index] = fields.model_hash[index];
      detections->descriptor_sha256[index] = fields.descriptor_hash[index];
    }
  }
  valid = true;
done:
  if (!valid) {
    *result = (struct object_detection_result) {0};
    if (detections) *detections = (struct spotter_detections) {0};
  }
  return valid;
}

/** @brief Read only the capture-facing detector summary.
 * @param filename JPEG paired with the sidecar.
 * @param[out] result Summary; cleared on read or validation failure.
 * @return True for valid evidence, including zero detections.
 * @details Reuse the full parser so requesting only the summary never
 * skips box or provenance validation. */
static bool parse_detection_sidecar(const char *filename, struct object_detection_result *result)
{
  return read_detection_sidecar(filename, result, NULL);
}

/** @brief Read the full detector result associated with a JPEG.
 * @param filename JPEG paired with the sidecar.
 * @param[out] detections Required destination for boxes and provenance.
 * @return True for a valid sidecar, including an empty detector result.
 * @details Use the same validation as the capture summary so downstream
 * projection cannot admit results the capture pipeline rejects. */
bool spotter_cam_pipe_read_detections(const char *filename, struct spotter_detections *detections)
{
  struct object_detection_result summary;
  return detections && read_detection_sidecar(filename, &summary, detections);
}

bool spotter_cam_pipe_bind_frame(uint64_t sequence, uint64_t sensor_timestamp_ns)
{
  if (last_detection.valid && last_detection.hash_bound && !last_detection.sensor_timestamp_ns)
    return true;
  /* A correctly shaped record may still belong to another exposure. Erase it
   * on mismatch so later summary/copy calls cannot reuse that stale hit. */
  if (!sensor_timestamp_ns || !last_detection.valid || !last_detection.has_frame_identity
      || last_detection.sequence != sequence || last_detection.sensor_timestamp_ns != sensor_timestamp_ns) {
    last_detection = (struct object_detection_result) {0};
    return false;
  }
  return true;
}

/* Keep the directory borrowed for backend lifetime; configuration does not
 * acquire a sensor or copy the caller's path into hidden storage. */
void spotter_cam_pipe_set_photo_directory(const char *directory)
{
  photo_directory = directory == NULL ? USHER_SPOTTERCAM_PHOTO_DIR : directory;
}

/* Preserve the distinction between failed analysis, valid empty output and a
 * hit. All three outcomes can accompany an otherwise usable photograph. */
void spotter_cam_pipe_last_detection_summary(char *buffer, size_t buffer_size)
{
  if (buffer == NULL || buffer_size == 0) return;
  if (!last_detection.valid) {
    snprintf(buffer, buffer_size, "status=analysis_failed; coordinates_omitted=true");
  } else if (last_detection.count <= 0) {
    snprintf(buffer, buffer_size, "status=ok; count=0");
  } else {
    snprintf(buffer, buffer_size,
             "status=ok; count=%d; label=%s; confidence=%.4f; box=%d,%d,%d,%d",
             last_detection.count, last_detection.label, last_detection.conf,
             last_detection.box_x, last_detection.box_y, last_detection.box_w, last_detection.box_h);
  }
}

/** @brief Stream a review copy without allocating another image-sized buffer.
 * @param source_path Readable original photograph, which is never modified.
 * @param dest_path New review destination; existing files and links are never replaced.
 * @return Zero after writes/close succeed, otherwise -1; opened partial output is removed.
 * @details This is a convenience copy, not the exclusive flight-image publication
 * path. It neither fsyncs the destination nor guarantees power-loss durability. */
static int copy_file(const char *source_path, const char *dest_path)
{
  if (!source_path || !dest_path || !source_path[0] || !dest_path[0]) { errno = EINVAL; return -1; }
  int source_fd = open(source_path, O_RDONLY | O_CLOEXEC | O_NOFOLLOW | O_NONBLOCK);
  if (source_fd < 0) {
    return -1;
  }
  struct stat source;
  if (fstat(source_fd, &source) != 0 || !S_ISREG(source.st_mode)) {
    close(source_fd);
    errno = EINVAL;
    return -1;
  }
  int dest_fd = open(dest_path, O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC | O_NOFOLLOW, 0644);
  if (dest_fd < 0) {
    const int error = errno;
    close(source_fd);
    errno = error;
    return -1;
  }

  char buffer[OBJECT_COPY_BUFFER_SIZE];
  ssize_t bytes_read;
  int result = 0;
  for (;;) {
    bytes_read = read(source_fd, buffer, sizeof(buffer));
    if (bytes_read < 0 && errno == EINTR) continue;
    if (bytes_read <= 0) break;
    size_t written = 0;
    while (written < (size_t)bytes_read) {
      ssize_t bytes_written = write(dest_fd, buffer + written, (size_t)bytes_read - written);
      if (bytes_written <= 0) {
        if (bytes_written < 0 && errno == EINTR) {
          continue;
        }
        if (!bytes_written) errno = EIO;
        result = -1;
        break;
      }
      written += (size_t)bytes_written;
    }
    if (result != 0) {
      break;
    }
  }
  if (bytes_read < 0) {
    result = -1;
  }

  int error = errno;
  if (close(source_fd) != 0 && result == 0) { result = -1; error = errno; }
  if (close(dest_fd) != 0) {
    if (result == 0) error = errno;
    result = -1;
  }
  if (result != 0) {
    unlink(dest_path);
  }
  errno = error;
  return result;
}

/* Copy after EXIF publication so review evidence matches the saved original.
 * The original remains authoritative if this optional convenience copy fails. */
void spotter_cam_pipe_save_detection_copy(const char *filename)
{
  if (filename == NULL || filename[0] == '\0' || !last_detection.valid || last_detection.count <= 0) {
    return;
  }

  char resolved_dir[PATH_MAX];
  const char *photo_dir = usher_resolve_path(photo_directory, resolved_dir, sizeof(resolved_dir));

  char hits_dir[PATH_MAX];
  int dir_len = snprintf(hits_dir, sizeof(hits_dir), "%s/detections", photo_dir);
  if (dir_len < 0 || (size_t)dir_len >= sizeof(hits_dir) || usher_ensure_directory(hits_dir) != 0) {
    fprintf(stderr, "SPOTTER_CAM_PIPE:\tfailed to prepare detections directory under %s\n", photo_dir);
    return;
  }

  const char *base_name = strrchr(filename, '/');
  base_name = base_name != NULL ? base_name + 1 : filename;

  char dest_path[PATH_MAX];
  int dest_len = snprintf(dest_path, sizeof(dest_path), "%s/%s", hits_dir, base_name);
  if (dest_len < 0 || (size_t)dest_len >= sizeof(dest_path)) {
    fprintf(stderr, "SPOTTER_CAM_PIPE:\tdetections path too long for %s\n", filename);
    return;
  }

  if (copy_file(filename, dest_path) != 0) {
    fprintf(stderr, "SPOTTER_CAM_PIPE:\tfailed to copy %s into detections directory: %s\n",
            filename, strerror(errno));
  } else {
    printf("SPOTTER_CAM_PIPE:\t%s detected (count=%d, conf=%.2f); copied to %s\n",
           last_detection.label, last_detection.count, last_detection.conf, dest_path);
  }
}

/* Qualify the selected producer, not just process existence. The native path
 * delegates mechanics to the neutral supervisor; object interpretation stays here. */
int spotter_cam_pipe_init(const char *unused)
{
  (void)unused;
  spotter_capture.command = USHER_SPOTTERCAM_COMMAND;
  spotter_capture.model = spotter_photo_only ? NULL : USHER_SPOTTERCAM_MODEL;
  spotter_capture.settings = spotter_photo_only ? NULL : spotter_settings ? spotter_settings : "-";
  spotter_capture.photo_only = spotter_photo_only;
  spotter_capture.directory = photo_directory;
  spotter_capture.prefix = 'a';
  return imx500_pipe_start(&spotter_capture);
}

/* Capture success does not require successful inference. Inspect only this
 * image's sidecar and keep failure explicit in the summary, not in its pixels. */
int spotter_cam_pipe_shoot(char *filename, size_t filename_size, int image_number)
{
  last_detection = (struct object_detection_result) {0};
  /* Clearing before capture prevents a successful old inference from surviving
   * a new photograph whose producer or sidecar failed. */
  if (filename == NULL || filename_size == 0 || image_number < 0) {
    return -1;
  }

  int status = imx500_pipe_shoot(&spotter_capture, filename, filename_size, image_number);
  if (status != 0) {
    filename[0] = '\0';
    return -1;
  }
  if (!spotter_photo_only && !parse_detection_sidecar(filename, &last_detection)) {
    fprintf(stderr, "SPOTTER_CAM_PIPE:\tmissing or invalid native detection metadata for %s\n",
            filename);
  }
  return 0;
}

/** Stop the selected camera owner and release detector-specific configuration. */
void spotter_cam_pipe_deinit(void)
{
  imx500_pipe_stop(&spotter_capture);
  imx500_pipe_stop(&spotter_analysis);
  last_detection = (struct object_detection_result) {0};
}

int spotter_cam_pipe_set_settings(const char *settings)
{
  if (!settings || !settings[0] || spotter_capture.pid > 0 || spotter_analysis.pid > 0) return -1;
  spotter_settings = settings;
  return 0;
}

int spotter_cam_pipe_analyze(const char *filename, unsigned number)
{
  last_detection = (struct object_detection_result){0};
  if (!filename || !filename[0] || !number || strnlen(filename, PATH_MAX) == PATH_MAX ||
      strchr(filename, '\n') || strchr(filename, '\r')) return -1;
  if (!spotter_settings) return 0;
  if (spotter_analysis.pid <= 0) {
    spotter_analysis.command = USHER_SPOTTERCAM_COMMAND;
    spotter_analysis.settings = spotter_settings;
    spotter_analysis.analyze = true;
    if (imx500_pipe_start(&spotter_analysis)) return -1;
  }
  if (imx500_pipe_analyze(&spotter_analysis, filename, number) ||
      !parse_detection_sidecar(filename, &last_detection) || last_detection.sequence != number ||
      last_detection.sensor_timestamp_ns) {
    last_detection = (struct object_detection_result){0};
    return -1;
  }
  return 0;
}
