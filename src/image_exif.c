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
 * @file image_exif.c
 * @brief JPEG EXIF updates for USHER flight and thermal-capture provenance.
 * @details Existing JPEG markers are copied while prior EXIF is replaced by a standards
 * compliant APP1 block. Default in-place updates avoid temporary files and leave
 * equal-position image bytes untouched. IMAGE_EXIF_ATOMIC_REPLACE selects the older
 * sibling-file/fsync/rename method when original-file retention is more important.
 * In-place writes are not atomic; interrupted mutation can damage the JPEG. Neither
 * method promises full power-loss durability, since the parent directory is not synced.
 */
#include "image_exif.h"
#include "boot_id.h"
#include "exifite.h"
#include "motion_compensation.h"

#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <limits.h>
#include <math.h>
#include <stdbool.h>
#include <stdatomic.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#define ANGLE_BFP_SCALE 4096.0
#define SPEED_BFP_SCALE 524288.0
#define POSITION_BFP_SCALE 256.0
#define RAD_TO_DEG 57.29577951308232
#define ASCII_COMMENT_PREFIX_SIZE 8U
#define EXIF_WORKSPACE_COUNT 8U
#define EXIF_SCRATCH_CAPACITY 8192U
#define SHOT_COMMENT_CAPACITY 1536U
#define COMPLETE_RECORD_CAPACITY 4096U

struct exif_workspace {
  atomic_bool in_use;
  char temporary_name[PATH_MAX];
  unsigned char scratch[EXIF_SCRATCH_CAPACITY];
  unsigned char payload[EXIF_MAX_PAYLOAD];
  struct exif_document document;
};

static struct exif_workspace exif_workspaces[EXIF_WORKSPACE_COUNT];
static _Thread_local struct exif_workspace *active_workspace;

/** @brief Claim a bounded EXIF editing slot for the current thread.
 * @return Owned slot, or NULL with EBUSY for reentry or EAGAIN for exhaustion.
 * @details Keep large scratch buffers off worker stacks. Acquire ordering
 * pairs with release so a new owner does not race the previous user. */
static struct exif_workspace *workspace_acquire(void)
{
  if (active_workspace != NULL) {
    errno = EBUSY;
    return NULL;
  }
  for (size_t index = 0; index < EXIF_WORKSPACE_COUNT; ++index) {
    bool expected = false;
    if (atomic_compare_exchange_strong_explicit(&exif_workspaces[index].in_use, &expected, true,
        memory_order_acquire, memory_order_relaxed)) {
      struct exif_workspace *workspace = &exif_workspaces[index];
      workspace->temporary_name[0] = '\0';
      exif_document_init(&workspace->document);
      active_workspace = workspace;
      return workspace;
    }
  }
  errno = EAGAIN;
  return NULL;
}

/** @brief Return an editing slot to the shared pool.
 * @param workspace Thread-owned slot, or NULL.
 * @details Clear the thread-local reference before making the slot
 * available. All borrowed document and scratch pointers expire here. */
static void workspace_release(struct exif_workspace *workspace)
{
  if (workspace == NULL) {
    return;
  }
  active_workspace = NULL;
  atomic_store_explicit(&workspace->in_use, false, memory_order_release);
}

/** @brief Import an existing JPEG EXIF block into the active editing slot.
 * @param filename JPEG to read.
 * @return Borrowed editable document, or NULL if the slot or metadata is unavailable.
 * @details Parse the bounded payload before importing its entries so a
 * metadata-only update can retain the image's existing tags. */
static struct exif_document *load_exif(const char *filename)
{
  if (!active_workspace) return NULL;
  size_t size;
  struct exif_view view;
  if (exif_read_jpeg(filename, active_workspace->payload, sizeof(active_workspace->payload), &size) != 0
      || exif_parse(&view, active_workspace->payload, size) != 0
      || image_exif_import(&active_workspace->document, &view) != 0) return NULL;
  return &active_workspace->document;
}

/** @brief Encode one signed GPS coordinate as EXIF hemisphere and DMS tags.
 * @param[in,out] exif Editable EXIF document.
 * @param reference_tag GPS tag for the hemisphere letter.
 * @param coordinate_tag GPS tag for the three unsigned rationals.
 * @param coordinate Signed coordinate in units of 10^-7 degrees.
 * @param limit Inclusive absolute bound in the same units.
 * @param positive Hemisphere for nonnegative values.
 * @param negative Hemisphere for negative values.
 * @return Zero on success, or -1 on invalid range or insufficient tag storage.
 * @details Split the magnitude with integer arithmetic so coordinate
 * conversion does not introduce floating-point rollover between DMS fields.
 * A failure after setting the hemisphere can leave the document modified. */
static int set_coordinate(struct exif_document *exif, uint16_t reference_tag, uint16_t coordinate_tag,
                          int32_t coordinate, int32_t limit, char positive, char negative)
{
  if (coordinate < -limit || coordinate > limit) {
    return -1;
  }

  char reference[2] = {coordinate < 0 ? negative : positive, '\0'};
  if (exif_document_ascii(exif, EXIF_GPS, reference_tag, reference) != 0) {
    return -1;
  }

  unsigned char *entry = exif_document_reserve(exif, EXIF_GPS, coordinate_tag, EXIF_RATIONAL, 3);
  if (entry == NULL) {
    return -1;
  }

  uint64_t absolute = coordinate < 0 ? (uint64_t)(-(int64_t)coordinate) : (uint64_t)coordinate;
  uint32_t degrees = (uint32_t)(absolute / 10000000U);
  uint64_t remainder = absolute % 10000000U;
  uint32_t minutes = (uint32_t)((remainder * 60U) / 10000000U);
  remainder = remainder * 60U - (uint64_t)minutes * 10000000U;
  uint32_t seconds_millionths = (uint32_t)((remainder * 60U * 1000000U) / 10000000U);

  exif_encode_long(entry, degrees);
  exif_encode_long(entry + 4, 1);
  exif_encode_long(entry + 8, minutes);
  exif_encode_long(entry + 12, 1);
  exif_encode_long(entry + 16, seconds_millionths);
  exif_encode_long(entry + 20, 1000000U);
  return 0;
}

/** @brief Convert a flight-controller heading to the EXIF angular range.
 * @param angle_bfp Angle in flight-controller fixed-point radians.
 * @return Degrees in [0, 360).
 * @details Wrap negative headings before encoding unsigned EXIF rationals. */
static double normalized_degrees(int32_t angle_bfp)
{
  double degrees = angle_bfp / ANGLE_BFP_SCALE * RAD_TO_DEG;
  degrees = fmod(degrees, 360.0);
  return degrees < 0.0 ? degrees + 360.0 : degrees;
}

/** @brief Form a nonnegative numerator for an EXIF rational.
 * @param value Quantity to encode.
 * @param scale Positive finite denominator scale.
 * @return Rounded numerator, saturated at UINT32_MAX; zero for nonpositive or nonfinite input.
 * @details Saturate before converting so an out-of-range value cannot wrap. */
static uint32_t scaled_unsigned(double value, double scale)
{
  if (!isfinite(value) || value <= 0.0) {
    return 0;
  }
  double scaled = value * scale;
  return scaled >= UINT32_MAX ? UINT32_MAX : (uint32_t)llround(scaled);
}

/** @brief Add GPS fields and the flight-pose description to an editable document.
 * @param[in,out] exif Document owned by the active workspace.
 * @param original Pose supplied by the caller; never modified.
 * @param delay Estimated capture delay in seconds.
 * @param compensate Nonzero to attempt ground-motion compensation.
 * @param timing Optional capture delivery evidence.
 * @return Zero on success, or -1 if a tag or the bounded description cannot be written.
 * @details Apply optional compensation to a local copy. Keep standard EXIF
 * fields for generic viewers and record fixed-point and physical units in
 * the description for consumers that need the full flight pose. */
static int add_shot_tags(struct exif_document *exif, const union dc_shot_union *original, double delay, int compensate,
                         const struct capture_timing *timing)
{
  union dc_shot_union adjusted = *original;
  int32_t latitude = original->data.lat;
  int32_t longitude = original->data.lon;
  const int applied = compensate && compensate_ground_position(&latitude, &longitude,
                      original->data.vground / SPEED_BFP_SCALE, original->data.course / ANGLE_BFP_SCALE, delay);
  adjusted.data.lat = latitude;
  adjusted.data.lon = longitude;
  const union dc_shot_union *shot = &adjusted;
  unsigned char *version = exif_document_reserve(exif, EXIF_GPS, EXIF_GPS_VERSION, EXIF_BYTE, 4);
  if (version == NULL) {
    return -1;
  }
  version[0] = 2;
  version[1] = 3;

  if (set_coordinate(exif, EXIF_GPS_LATITUDE_REF, EXIF_GPS_LATITUDE,
                     shot->data.lat, 900000000, 'N', 'S') != 0
      || set_coordinate(exif, EXIF_GPS_LONGITUDE_REF, EXIF_GPS_LONGITUDE,
                        shot->data.lon, 1800000000, 'E', 'W') != 0) {
    return -1;
  }

  unsigned char *altitude_reference = exif_document_reserve(exif, EXIF_GPS, EXIF_GPS_ALTITUDE_REF,
                                  EXIF_BYTE, 1);
  if (altitude_reference == NULL) {
    return -1;
  }
  altitude_reference[0] = shot->data.alt < 0 ? 1 : 0;
  uint64_t altitude_mm = shot->data.alt < 0 ? (uint64_t)(-(int64_t)shot->data.alt) : (uint64_t)shot->data.alt;
  if (altitude_mm > UINT32_MAX
      || exif_document_rational(exif, EXIF_GPS, EXIF_GPS_ALTITUDE, (uint32_t)altitude_mm, 1000) != 0) {
    return -1;
  }

  double speed_m_s = shot->data.vground / SPEED_BFP_SCALE;
  double course_deg = normalized_degrees(shot->data.course);
  double yaw_deg = normalized_degrees(shot->data.psi);
    if (exif_document_ascii(exif, EXIF_GPS, EXIF_GPS_SPEED_REF, "K") != 0
      || exif_document_rational(exif, EXIF_GPS, EXIF_GPS_SPEED,
                          scaled_unsigned(fabs(speed_m_s) * 3.6, 1000.0), 1000) != 0
      || exif_document_ascii(exif, EXIF_GPS, EXIF_GPS_TRACK_REF, "T") != 0
      || exif_document_rational(exif, EXIF_GPS, EXIF_GPS_TRACK,
                          scaled_unsigned(course_deg, 10000.0), 10000) != 0
      || exif_document_ascii(exif, EXIF_GPS, EXIF_GPS_DIRECTION_REF, "T") != 0
      || exif_document_rational(exif, EXIF_GPS, EXIF_GPS_DIRECTION,
                          scaled_unsigned(yaw_deg, 10000.0), 10000) != 0) {
    return -1;
  }

  char *comment = (char *)active_workspace->scratch;
  int comment_length = snprintf(comment, SHOT_COMMENT_CAPACITY,
                                "Paparazzi USHER MORA; nr=%" PRId32 "; lat_e7deg=%" PRId32
                                "; lon_e7deg=%" PRId32 "; alt_mm=%" PRId32
                                "; phi_bfp=%" PRId32 "; theta_bfp=%" PRId32
                                "; psi_bfp=%" PRId32 "; vground_bfp=%" PRId32
                                "; course_bfp=%" PRId32 "; groundalt_bfp=%" PRId32
                                "; lat_deg=%.7f; lon_deg=%.7f; alt_m=%.3f"
                                "; roll_deg=%.4f; pitch_deg=%.4f; yaw_deg=%.4f"
                                "; speed_m_s=%.4f; course_deg=%.4f; groundalt_m=%.3f",
                                shot->data.nr, shot->data.lat, shot->data.lon, shot->data.alt,
                                shot->data.phi, shot->data.theta, shot->data.psi,
                                shot->data.vground, shot->data.course, shot->data.groundalt,
                                shot->data.lat / 1e7, shot->data.lon / 1e7, shot->data.alt / 1000.0,
                                shot->data.phi / ANGLE_BFP_SCALE * RAD_TO_DEG,
                                shot->data.theta / ANGLE_BFP_SCALE * RAD_TO_DEG,
                                shot->data.psi / ANGLE_BFP_SCALE * RAD_TO_DEG,
                                speed_m_s, course_deg, shot->data.groundalt / POSITION_BFP_SCALE);
  if (comment_length < 0 || (size_t)comment_length >= SHOT_COMMENT_CAPACITY) {
    return -1;
  }
  if (isfinite(delay) && delay >= 0) {
    int extra = snprintf(comment + comment_length, SHOT_COMMENT_CAPACITY - (size_t)comment_length,
                         "; original_lat_deg=%.7f; original_lon_deg=%.7f; request_to_frame_arrival_s=%.9f"
                         "; position_compensation=%s; timing_reference=%s"
                         "; exposure_latency_known=false; attitude_compensated=false; altitude_compensated=false",
                         original->data.lat / 1e7, original->data.lon / 1e7, delay,
                         applied ? "constant_ground_velocity_estimate" : "not_applied",
                         timing && capture_timing_valid(timing)
                         && timing->callback_arrival ? "server_request_to_callback" : "server_request_to_uvc_return");
    if (extra < 0 || (size_t)extra >= SHOT_COMMENT_CAPACITY - (size_t)comment_length) return -1;
    comment_length += extra;
  }
  if (timing && capture_timing_valid(timing)) {
    char boot_id[37];
    usher_boot_id(boot_id);
    int extra = snprintf(comment + comment_length, SHOT_COMMENT_CAPACITY - (size_t)comment_length,
                         "; mora_boot_id=%s; camera_request_monotonic_us=%" PRIu64
                         "; frame_arrival_monotonic_us=%" PRIu64 "; capture_time_kind=%s",
                         boot_id, timing->request_monotonic_us, timing->arrival_monotonic_us,
                         timing->callback_arrival ? "sdk_callback_not_exposure" : "uvc_return_not_exposure");
    if (extra < 0 || (size_t)extra >= SHOT_COMMENT_CAPACITY - (size_t)comment_length) return -1;
    comment_length += extra;
    if (timing->callback_arrival) {
      extra = snprintf(comment + comment_length, SHOT_COMMENT_CAPACITY - (size_t)comment_length,
                       "; callback_sequence=%" PRIu64 "; callback_drops=%" PRIu64,
                       timing->callback_sequence, timing->callback_drops);
      if (extra < 0 || (size_t)extra >= SHOT_COMMENT_CAPACITY - (size_t)comment_length) return -1;
    }
  }
    if (exif_document_ascii(exif, EXIF_PRIMARY, EXIF_IMAGE_DESCRIPTION, comment) != 0
      || exif_document_ascii(exif, EXIF_PRIMARY, EXIF_SOFTWARE, "Paparazzi USHER") != 0) {
    return -1;
  }
  return 0;
}

static int save_exif(const char *filename, struct exif_document *exif);

/** @brief Write an EXIF UserComment composed of flight and analysis records.
 * @param[in,out] exif Document already carrying the flight description.
 * @param tag Optional section label as an empty or nonempty string, never NULL.
 * @param information Null-terminated analysis text, at most 8192 bytes.
 * @return Zero on success, or -1 on invalid input or insufficient storage.
 * @details Snapshot the description before reserving another entry so
 * rebuilding the tag storage cannot invalidate the bytes being copied. */
static int set_tagged_comment(struct exif_document *exif, const char *tag, const char *information)
{
  if (!exif || !tag || !information) return -1;
  const size_t information_length = strnlen(information, 8193);
  if (information_length > 8192) return -1;
  struct exif_value description;
  if (!exif_document_get(exif, EXIF_PRIMARY, EXIF_IMAGE_DESCRIPTION, &description)
      || description.format != EXIF_ASCII || description.size > COMPLETE_RECORD_CAPACITY) return -1;
  const size_t description_length = strnlen((const char *)description.data, description.size);
  unsigned char saved_description[COMPLETE_RECORD_CAPACITY];
  for (size_t index = 0; index < description_length; ++index) saved_description[index] = description.data[index];
  char separator[64];
  const int separator_length = tag[0] ? snprintf(separator, sizeof(separator), "\n%s\n", tag) : 0;
  if (separator_length < 0 || (size_t)separator_length >= sizeof(separator)) return -1;
  const size_t length = ASCII_COMMENT_PREFIX_SIZE + description_length + (size_t)separator_length
                        + information_length + 1;
  unsigned char *entry = exif_document_reserve(exif, EXIF_CAMERA, EXIF_USER_COMMENT,
                                              EXIF_UNDEFINED, (uint32_t)length);
  if (!entry) return -1;
  const unsigned char prefix[8] = {'A', 'S', 'C', 'I', 'I', 0, 0, 0};
  size_t offset = 0;
  for (size_t index = 0; index < sizeof(prefix); ++index) entry[offset++] = prefix[index];
  for (size_t index = 0; index < description_length; ++index) entry[offset++] = saved_description[index];
  for (size_t index = 0; index < (size_t)separator_length; ++index) entry[offset++] = (unsigned char)separator[index];
  for (size_t index = 0; index < information_length; ++index) entry[offset++] = (unsigned char)information[index];
  return 0;
}

/** @brief Compatibility wrapper for analysis that becomes available after the flight EXIF write.
 * @details Prefer the combined writer when the result is already available, so
 * the original need not be streamed to a replacement file twice.
 */
static int write_tagged_comment(const char *filename, const char *tag, const char *information)
{
  if (!filename) return -1;
  struct exif_workspace *workspace = workspace_acquire();
  if (workspace == NULL) return -1;
  struct exif_document *exif = load_exif(filename);
  if (!exif) {
    workspace_release(workspace);
    return -1;
  }
  if (set_tagged_comment(exif, tag, information) != 0) {
    workspace_release(workspace);
    return -1;
  }
  int result = save_exif(filename, exif);
  workspace_release(workspace);
  return result;
}

int image_exif_write_hotspots(const char *filename, const char *information)
{
  return write_tagged_comment(filename, "LWIR_HOTSPOTS_V1", information);
}

int image_exif_write_object_detections(const char *filename, const char *information)
{
  return write_tagged_comment(filename, "AICAM_OBJECTS_V1", information);
}

int image_exif_write_mock_map(const char *filename, double horizontal_fov, double vertical_fov)
{
  if (filename == NULL || !isfinite(horizontal_fov) || !isfinite(vertical_fov)
      || horizontal_fov <= 0 || horizontal_fov >= 179 || vertical_fov <= 0 || vertical_fov >= 179) return -1;
  struct exif_workspace *workspace = workspace_acquire();
  if (workspace == NULL) return -1;
  struct exif_document *exif = load_exif(filename);
  if (exif == NULL) {
    workspace_release(workspace);
    return -1;
  }
  char record[256];
  int length = snprintf(record, sizeof(record),
                        "{\"schema\":\"USHER_MOCK_MAP_V1\",\"tile_scheme\":\"XYZ\",\"zoom\":18,"
                        "\"projection\":\"pinhole_flat_ground\",\"horizontal_fov_deg\":%.9g,"
                        "\"vertical_fov_deg\":%.9g}", horizontal_fov, vertical_fov);
  if (length < 0 || (size_t)length >= sizeof(record)
      || exif_document_ascii(exif, EXIF_PRIMARY, EXIF_DOCUMENT_NAME, record) != 0) {
    workspace_release(workspace);
    return -1;
  }
  int result = save_exif(filename, exif);
  workspace_release(workspace);
  return result;
}

int image_exif_write(const char *filename, const union dc_shot_union *shot)
{
  return image_exif_write_timed(filename, shot, -1, 0);
}

int image_exif_write_timed(const char *filename, const union dc_shot_union *shot,
                           double capture_delay_s, int compensate)
{
  return image_exif_write_capture(filename, shot, capture_delay_s, compensate, NULL);
}

int image_exif_write_capture(const char *filename, const union dc_shot_union *shot,
                             double capture_delay_s, int compensate, const struct capture_timing *timing)
{
  return image_exif_write_complete(filename, shot, capture_delay_s, compensate, timing, NULL, 0, NULL);
}

/** @brief Extend the flight description with observation provenance.
 * @param[in,out] exif Active document containing the existing description.
 * @param timing Optional timing, frame-identity and pose evidence.
 * @param aligned Whether the primary capture pose came from synchronized evidence.
 * @return Zero on success, or -1 on malformed metadata or insufficient storage.
 * @details Record frame identity separately from timing and pose alignment.
 * A bound frame may lack a valid exposure interval; navigation uncertainty
 * is recorded only when a pose is marked valid. */
static int add_observation_tags(struct exif_document *exif, const struct capture_timing *timing, bool aligned)
{
  const struct capture_observation empty = {0};
  const struct capture_observation *observation = timing ? &timing->observation : &empty;
  if (!capture_observation_valid(observation) && !observation->frame_bound) observation = &empty;
    struct exif_value description;
    if (!exif_document_get(exif, EXIF_PRIMARY, EXIF_IMAGE_DESCRIPTION, &description)
      || description.format != EXIF_ASCII || description.size > COMPLETE_RECORD_CAPACITY) return -1;
  char boot[37];
  usher_boot_id(boot);
  char *record = (char *)active_workspace->scratch;
  int length = snprintf(record, COMPLETE_RECORD_CAPACITY,
                        "%.*s; altitude_datum=flight_controller_navigation; capture_pose_synchronized=%s"
                        "; observation_time_kind=%s; observation_pose_aligned=%s; frame_identity_bound=%s"
                        "; observation_earliest_monotonic_us=%" PRIu64 "; observation_latest_monotonic_us=%" PRIu64
                        "; observation_sequence=%" PRIu64 "; sensor_timestamp_ns=%" PRIu64 "; observation_boot_id=%s",
                        (int)description.size, description.data, aligned ? "true" : "false",
                        capture_time_name(observation->kind), observation->pose_valid ? "true" : "false",
                        observation->frame_bound ? "true" : "false",
                        observation->earliest_us, observation->latest_us, observation->sequence, observation->sensor_timestamp_ns, boot);
  if (length < 0 || (size_t)length >= COMPLETE_RECORD_CAPACITY) return -1;
  if (observation->pose_valid) {
    const union dc_shot_union *pose = &observation->pose;
    if (!isfinite(observation->pose_uncertainty_m) || observation->pose_uncertainty_m < 0) return -1;
    int extra = snprintf(record + length, COMPLETE_RECORD_CAPACITY - (size_t)length,
                         "; observation_lat_deg=%.7f; observation_lon_deg=%.7f; observation_alt_m=%.3f"
                         "; observation_agl_m=%.3f; observation_roll_deg=%.4f; observation_pitch_deg=%.4f"
                         "; observation_yaw_deg=%.4f; observation_pose_uncertainty_m=%.3f"
                         "; navigation_position_method=%s; navigation_position_age_ms=%" PRIu32,
                         pose->data.lat / 1e7, pose->data.lon / 1e7, pose->data.alt / 1000.0,
                         pose->data.groundalt / POSITION_BFP_SCALE, pose->data.phi / ANGLE_BFP_SCALE * RAD_TO_DEG,
                         pose->data.theta / ANGLE_BFP_SCALE * RAD_TO_DEG, pose->data.psi / ANGLE_BFP_SCALE * RAD_TO_DEG,
                         observation->pose_uncertainty_m,
                         observation->navigation_projected ? "gps_receipt_velocity_projection" : "fc_state_sample",
                         observation->navigation_age_ms);
    if (extra < 0 || (size_t)extra >= COMPLETE_RECORD_CAPACITY - (size_t)length) return -1;
  }
  return exif_document_ascii(exif, EXIF_PRIMARY, EXIF_IMAGE_DESCRIPTION, record);
}

int image_exif_write_complete(const char *filename, const union dc_shot_union *shot,
                              double capture_delay_s, int compensate, const struct capture_timing *timing,
                              const union dc_shot_union *request, double uncertainty_m, const char *objects)
{
  if (filename == NULL || shot == NULL) {
    errno = EINVAL;
    return -1;
  }

  struct exif_workspace *workspace = workspace_acquire();
  if (workspace == NULL) {
    return -1;
  }
  struct exif_document *exif = &workspace->document;
  size_t capture_size = 0;
  struct exif_view view;
  if (exif_read_jpeg(filename, workspace->payload, sizeof(workspace->payload), &capture_size) == 0
      && exif_parse(&view, workspace->payload, capture_size) == 0) {
    struct exif_value capture;
    for (size_t field = 0; field < sizeof(image_exif_capture_tags) / sizeof(image_exif_capture_tags[0]); ++field) {
      if (!exif_get(&view, EXIF_CAMERA, image_exif_capture_tags[field], &capture)) continue;
      unsigned char *stored = exif_document_reserve(exif, EXIF_CAMERA, image_exif_capture_tags[field],
                                                   (enum exif_format)capture.format, capture.count);
      if (!stored) {
        workspace_release(workspace);
        return -1;
      }
      for (size_t index = 0; index < capture.size; ++index) stored[index] = capture.data[index];
    }
  }
  if (timing && capture_timing_valid(timing)) {
    capture_delay_s = (timing->arrival_monotonic_us - timing->request_monotonic_us) / 1e6;
  }
  if (add_shot_tags(exif, shot, capture_delay_s, request ? 0 : compensate, timing) != 0) {
    fprintf(stderr, "EXIF:\tfailed to create shot metadata for %s\n", filename);
    workspace_release(workspace);
    return -1;
  }

  if (request) {
    /* Exposure-aligned coordinates improve the geotag, but the original FC
     * request remains essential when investigating latency or pose errors later. */
    struct exif_value description;
    char *record = (char *)active_workspace->scratch;
    if (!exif_document_get(exif, EXIF_PRIMARY, EXIF_IMAGE_DESCRIPTION, &description)
      || description.format != EXIF_ASCII || description.size > 1536
      || !isfinite(uncertainty_m) || uncertainty_m < 0) {
      workspace_release(workspace);
      return -1;
    }
    const int length = snprintf(record, COMPLETE_RECORD_CAPACITY,
                                "%.*s; pose_reference=bounded_sensor_exposure_estimate; pose_uncertainty_m=%.3f"
                                "; request_lat_e7deg=%" PRId32 "; request_lon_e7deg=%" PRId32 "; request_alt_mm=%" PRId32
                                "; request_phi_bfp=%" PRId32 "; request_theta_bfp=%" PRId32 "; request_psi_bfp=%" PRId32
                                "; request_groundalt_bfp=%" PRId32 "; request_vground_bfp=%" PRId32 "; request_course_bfp=%" PRId32,
                                (int)description.size, description.data, uncertainty_m, request->data.lat, request->data.lon, request->data.alt,
                                request->data.phi, request->data.theta, request->data.psi, request->data.groundalt, request->data.vground,
                                request->data.course);
    if (length < 0 || (size_t)length >= COMPLETE_RECORD_CAPACITY
        || exif_document_ascii(exif, EXIF_PRIMARY, EXIF_IMAGE_DESCRIPTION, record) != 0) {
      workspace_release(workspace);
      return -1;
    }
  }
  if (add_observation_tags(exif, timing, request != NULL) != 0
      || set_tagged_comment(exif, objects ? "AICAM_OBJECTS_V1" : "", objects ? objects : "") != 0) {
    workspace_release(workspace);
    return -1;
  }

  int result = save_exif(filename, exif);
  workspace_release(workspace);
  return result;
}

/** @brief Replace metadata while streaming the original compressed payload unchanged.
 * @param filename Existing image, exclusively owned by the caller during publication.
 * @param exif Metadata ownership is consumed on every exit.
 * @return Zero after replacement, -1 on failure with the original path retained.
 * @details The temporary sits beside the original so rename stays on one filesystem.
 * Flush and file sync happen before rename. This helper does not sync the parent
 * directory, so do not treat success as a complete power-loss durability guarantee.
 */
static int save_exif(const char *filename, struct exif_document *exif)
{
  if (active_workspace == NULL) {
    errno = EINVAL;
    return -1;
  }
  struct stat file_status, path_status;
  FILE *input = fopen(filename, IMAGE_EXIF_ATOMIC_REPLACE ? "rb" : "r+b");
  int source_result = input == NULL ? -1 : fstat(fileno(input), &file_status);
  if (source_result == 0) source_result = lstat(filename, &path_status);
  if (source_result == 0 && (!S_ISREG(file_status.st_mode) || !S_ISREG(path_status.st_mode)
      || file_status.st_dev != path_status.st_dev || file_status.st_ino != path_status.st_ino)) {
    errno = EINVAL;
    source_result = -1;
  }
  if (source_result != 0) {
    fprintf(stderr, "EXIF:\tfailed to open %s: %s\n", filename, strerror(errno));
    if (input != NULL) {
      const int saved_errno = errno;
      fclose(input);
      errno = saved_errno;
    }
    return -1;
  }

  uint32_t width, height;
  unsigned components;
  int metadata_result = exif_jpeg_dimensions(input, &width, &height, &components);
  if (!metadata_result && components != 1 && components != 3) {
    errno = EINVAL;
    metadata_result = -1;
  }
  if (!metadata_result) {
    unsigned char *configuration = exif_document_reserve(exif, EXIF_CAMERA, EXIF_COMPONENTS_CONFIGURATION,
                                                        EXIF_UNDEFINED, 4);
    if (!configuration) metadata_result = -1;
    else {
      configuration[0] = 1;
      if (components == 3) { configuration[1] = 2; configuration[2] = 3; }
    }
    unsigned char *dimension = exif_document_reserve(exif, EXIF_CAMERA, EXIF_PIXEL_X_DIMENSION, EXIF_LONG, 1);
    if (!dimension) metadata_result = -1;
    else exif_encode_long(dimension, width);
    dimension = exif_document_reserve(exif, EXIF_CAMERA, EXIF_PIXEL_Y_DIMENSION, EXIF_LONG, 1);
    if (!dimension) metadata_result = -1;
    else exif_encode_long(dimension, height);
  }
  unsigned char *exif_data = active_workspace->payload;
  size_t exif_size = 0;
  if (metadata_result != 0 || fseek(input, 0, SEEK_SET) != 0
      || exif_serialize(exif, exif_data, EXIF_MAX_PAYLOAD, &exif_size) != 0) {
    const int saved_errno = errno;
    fclose(input);
    errno = saved_errno;
    return -1;
  }

#if IMAGE_EXIF_ATOMIC_REPLACE
  int name_length = snprintf(active_workspace->temporary_name, sizeof(active_workspace->temporary_name),
                             "%s.exif.XXXXXX", filename);
  if (name_length < 0 || (size_t)name_length >= sizeof(active_workspace->temporary_name)) {
    fclose(input);
    errno = ENAMETOOLONG;
    return -1;
  }
  char *temporary_name = active_workspace->temporary_name;
  int temporary_fd = mkstemp(temporary_name);
  FILE *output = temporary_fd >= 0 ? fdopen(temporary_fd, "wb") : NULL;
  int result = 0;
  if (output == NULL || fchmod(temporary_fd, file_status.st_mode & 0777) != 0
      || exif_rewrite_jpeg(input, output, exif_data, exif_size) != 0
      || fflush(output) != 0 || fsync(temporary_fd) != 0) {
    result = -1;
  }
  if (fclose(input) != 0) {
    result = -1;
  }
  if (output != NULL && fclose(output) != 0) {
    result = -1;
  } else if (output == NULL && temporary_fd >= 0) {
    close(temporary_fd);
  }

  if (result == 0) {
    if (lstat(filename, &path_status) != 0 || !S_ISREG(path_status.st_mode)
        || file_status.st_dev != path_status.st_dev || file_status.st_ino != path_status.st_ino
        || file_status.st_size != path_status.st_size
        || file_status.st_mtim.tv_sec != path_status.st_mtim.tv_sec
        || file_status.st_mtim.tv_nsec != path_status.st_mtim.tv_nsec) {
      errno = ESTALE;
      result = -1;
    } else if (rename(temporary_name, filename) != 0) {
      result = -1;
    }
  }
  if (result != 0) {
    int saved_errno = errno;
    unlink(temporary_name);
    fprintf(stderr, "EXIF:\tfailed to update %s: %s\n", filename, strerror(saved_errno));
    errno = saved_errno;
  }

  return result;
#else
  int result = 0;
  if (lstat(filename, &path_status) != 0 || !S_ISREG(path_status.st_mode) ||
      file_status.st_dev != path_status.st_dev || file_status.st_ino != path_status.st_ino ||
      file_status.st_size != path_status.st_size ||
      file_status.st_mtim.tv_sec != path_status.st_mtim.tv_sec ||
      file_status.st_mtim.tv_nsec != path_status.st_mtim.tv_nsec) {
    errno = ESTALE;
    result = -1;
  } else if (exif_update_jpeg(input, exif_data, exif_size) != 0 || fsync(fileno(input)) != 0) {
    result = -1;
  } else if (lstat(filename, &path_status) != 0 || !S_ISREG(path_status.st_mode) ||
             file_status.st_dev != path_status.st_dev || file_status.st_ino != path_status.st_ino) {
    errno = ESTALE;
    result = -1;
  }
  int saved_errno = errno;
  if (fclose(input) != 0 && result == 0) {
    saved_errno = errno;
    result = -1;
  }
  if (result != 0) {
    fprintf(stderr, "EXIF:\tfailed to update %s in place: %s\n", filename, strerror(saved_errno));
    errno = saved_errno;
  }
  return result;
#endif
}