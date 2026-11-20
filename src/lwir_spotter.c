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
 * @file lwir_spotter.c
 * @brief Associate detections with the native thermal samples of a saved LWIR frame.
 * @details Reuse the Spottercam analysis worker on already-published captures.
 * Keep display coordinates, native sensor coordinates, and optional geographic
 * estimates distinct. The image is checked again before the sidecar is published
 * so a replaced capture is not silently paired with another frame's detections.
 */
#include "lwir_spotter.h"
#include "spotter_cam_pipe.h"
#include "imx500_pipe.h"
#include "camera_profile.h"
#include "bounded_ini.h"
#include "lwir_container.h"
#include "lwir_render.h"

#include <errno.h>
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#ifndef USHER_SPOTTERCAM_COMMAND
#define USHER_SPOTTERCAM_COMMAND "./bin/spottercam"
#endif

static struct imx500_pipe analysis = IMX500_PIPE_INITIALIZER;
static const char *selected_settings;
static struct usher_camera_profile profile;
static bool profile_loaded;
static unsigned request_sequence;
static uint64_t maximum_interval_us = 5000;
static double maximum_uncertainty_m = 1.0;
static uint8_t combined[256U * 192U * 4U];
static struct spotter_detections detections;

bool lwir_spotter_settings(const char *settings)
{
  if (!settings || !settings[0] || analysis.pid > 0) return false;
  selected_settings = settings;
  return true;
}

bool lwir_spotter_profile(const char *filename)
{
  struct usher_camera_profile candidate;
  if (analysis.pid > 0 || !usher_camera_profile_read(filename, &candidate)
      || candidate.camera_id != USHER_CAMERA_LWIRCAM || candidate.calibration.width != 256
      || candidate.calibration.height != 192) return false;
  profile = candidate;
  profile_loaded = true;
  return true;
}

bool lwir_spotter_limits(const char *limits)
{
  if (!limits || strlen(limits) >= 96 || analysis.pid > 0) return false;
  char text[96];
  for (size_t index = 0; index <= strlen(limits); ++index) text[index] = limits[index];
  char *comma = strchr(text, ',');
  if (!comma) return false;
  *comma++ = '\0';
  uint64_t interval;
  double uncertainty;
  if (!ini_uint(text, &interval) || !interval || interval > 200000
      || !ini_real(comma, &uncertainty) || uncertainty <= 0 || uncertainty > 100) return false;
  maximum_interval_us = interval;
  maximum_uncertainty_m = uncertainty;
  return true;
}

bool lwir_spotter_enabled(void)
{
  return selected_settings != NULL;
}

void lwir_spotter_stop(void)
{
  imx500_pipe_stop(&analysis);
}

/**
 * @brief Check whether an image still matches its pre-analysis metadata.
 * @param before Metadata captured before reading the image.
 * @param after Metadata captured after a later processing step.
 * @return True if the later file is regular and the identity and metadata match.
 * @details Device and inode catch replacement; size, modification time and
 * change time detect ordinary in-place edits without rereading the image.
 * This check is not a content hash.
 */
static bool same_file(const struct stat *before, const struct stat *after)
{
  return S_ISREG(after->st_mode) && before->st_dev == after->st_dev && before->st_ino == after->st_ino
    && before->st_size == after->st_size && before->st_mtim.tv_sec == after->st_mtim.tv_sec
    && before->st_mtim.tv_nsec == after->st_mtim.tv_nsec && before->st_ctim.tv_sec == after->st_ctim.tv_sec
    && before->st_ctim.tv_nsec == after->st_ctim.tv_nsec;
}

int lwir_spotter_analyze(const char *filename, const union dc_shot_union *shot,
                         const struct capture_timing *timing, const volatile sig_atomic_t *running)
{
  if (!selected_settings) return 0;
  if (!filename || !shot || !timing || shot->data.nr <= 0 || !capture_timing_valid(timing)) return -1;
  struct stat before, after;
  if (lstat(filename, &before) || !S_ISREG(before.st_mode)) return -1;
  struct lwir_saved_geometry geometry;
  /* Match the embedded frame to delivery evidence before invoking inference. */
  if (lwir_read_temperature_evidence(filename, combined + sizeof(combined) / 2,
      sizeof(combined) / 2, &geometry) != 1 || !geometry.camera || geometry.width != 256
      || geometry.height != 192 || geometry.legend || !geometry.sequence
      || geometry.arrival_us != timing->arrival_monotonic_us
      || (timing->callback_arrival && geometry.sequence != timing->callback_sequence)) return -1;
  if (analysis.pid <= 0 || request_sequence == UINT_MAX) {
    imx500_pipe_stop(&analysis);
    analysis.command = USHER_SPOTTERCAM_COMMAND;
    analysis.settings = selected_settings;
    analysis.analyze = true;
    analysis.running = running;
    if (imx500_pipe_start(&analysis)) return -1;
    request_sequence = 0;
  }
  const unsigned request = ++request_sequence;
  if (imx500_pipe_analyze(&analysis, filename, request)
      || !spotter_cam_pipe_read_detections(filename, &detections) || detections.sequence != request
      || lstat(filename, &after) || !same_file(&before, &after)) return -1;
  const struct capture_observation *observation = &timing->observation;
  /* Unqualified pose evidence still permits pixel and temperature results. */
  const bool aligned = profile_loaded && capture_observation_exposure(observation) && observation->pose_valid
    && observation->sequence == geometry.sequence && isfinite(observation->pose_uncertainty_m)
    && observation->pose_uncertainty_m >= 0 && observation->pose_uncertainty_m <= maximum_uncertainty_m
    && observation->latest_us - observation->earliest_us <= maximum_interval_us;
  const union dc_shot_union *pose_shot = &observation->pose;
  const double radians_to_degrees = 57.29577951308232;
  const struct coverage_pose pose = {
    .latitude_deg = pose_shot->data.lat / 1e7, .longitude_deg = pose_shot->data.lon / 1e7,
    .agl_m = pose_shot->data.groundalt / 256.0,
    .roll_deg = pose_shot->data.phi / 4096.0 * radians_to_degrees,
    .pitch_deg = pose_shot->data.theta / 4096.0 * radians_to_degrees,
    .yaw_deg = pose_shot->data.psi / 4096.0 * radians_to_degrees,
    .uncertainty_m = observation->pose_uncertainty_m
  };
  char output_path[PATH_MAX], pending_path[PATH_MAX];
  const size_t length = strnlen(filename, PATH_MAX);
  if (length < 4 || length >= PATH_MAX || strcmp(filename + length - 4, ".jpg")) return -1;
  int count = snprintf(output_path, sizeof(output_path), "%.*s.lwir-objects.ini", (int)length - 4, filename);
  if (count < 0 || (size_t)count >= sizeof(output_path)) return -1;
  count = snprintf(pending_path, sizeof(pending_path), "%s.pending", output_path);
  if (count < 0 || (size_t)count >= sizeof(pending_path)) return -1;
  FILE *output = fopen(pending_path, "wx");
  if (!output) return -1;
  fprintf(output, "schema=USHER_LWIR_OBJECTS_V1\nphoto_number=%d\nanalysis_sequence=%u\n"
    "frame_sequence=%llu\nframe_arrival_us=%llu\nboot_id=%s\nframe_sha256=%s\nimage_sha256=%s\n"
    "count=%u\ncoordinate_space=native_sensor\ncalibration=unverified\n"
    "projection_qualified=%u\ntiming_kind=%s\nexposure_earliest_us=%llu\nexposure_latest_us=%llu\n"
    "pose_valid=%u\npose_uncertainty_m=%.9g\nmaximum_interval_us=%llu\nmaximum_uncertainty_m=%.9g\n"
    "latitude_deg=%.9f\nlongitude_deg=%.9f\nagl_m=%.9g\nroll_deg=%.9g\npitch_deg=%.9g\nyaw_deg=%.9g\n"
    "projection_model=local_flat_ground_pinhole\n",
    shot->data.nr, request, (unsigned long long)geometry.sequence, (unsigned long long)geometry.arrival_us,
    geometry.boot_id, geometry.frame_sha256, detections.image_sha256, detections.count, aligned ? 1U : 0U,
    capture_time_name(observation->kind), (unsigned long long)observation->earliest_us,
    (unsigned long long)observation->latest_us, observation->pose_valid ? 1U : 0U, observation->pose_uncertainty_m,
    (unsigned long long)maximum_interval_us, maximum_uncertainty_m,
    pose.latitude_deg, pose.longitude_deg, pose.agl_m, pose.roll_deg, pose.pitch_deg, pose.yaw_deg);
  fprintf(output, "model_sha256=%s\nmodel_descriptor_sha256=%s\n", detections.model_sha256, detections.descriptor_sha256);
  struct lwir_render_options orientation = {.rotation = geometry.rotation, .mirror = geometry.mirror, .flip = geometry.flip};
  bool valid = true;
  for (unsigned index = 0; index < detections.count && valid; ++index) {
    const struct spotter_detection *box = &detections.objects[index];
    unsigned first_x, first_y, last_x, last_y;
    if (!lwir_render_native_coordinate(256, 192, &orientation, box->x, box->y, &first_x, &first_y)
        || !lwir_render_native_coordinate(256, 192, &orientation, box->x + box->width - 1,
          box->y + box->height - 1, &last_x, &last_y)) { valid = false; break; }
    double center_x = (first_x + last_x) / 2.0, center_y = (first_y + last_y) / 2.0;
    if (box->precise_center) {
      /* Map the display basis into sensor space to retain the subpixel center
       * through rotation and reflection; round only for the temperature sample. */
      unsigned origin_x, origin_y, right_x, right_y, down_x, down_y;
      if (!lwir_render_native_coordinate(256, 192, &orientation, 0, 0, &origin_x, &origin_y)
          || !lwir_render_native_coordinate(256, 192, &orientation, 1, 0, &right_x, &right_y)
          || !lwir_render_native_coordinate(256, 192, &orientation, 0, 1, &down_x, &down_y)) { valid = false; break; }
      center_x = origin_x + box->center_x * ((double)right_x - origin_x) + box->center_y * ((double)down_x - origin_x);
      center_y = origin_y + box->center_x * ((double)right_y - origin_y) + box->center_y * ((double)down_y - origin_y);
    }
    const unsigned native_x = (unsigned)lround(center_x), native_y = (unsigned)lround(center_y);
    struct lwir_measurement measurement = {.kind = LWIR_POINT, .roi = {.x = native_x, .y = native_y}};
    struct lwir_temperature_stats temperature;
    if (!lwir_measure_temperature(combined, sizeof(combined), 256, 192, &measurement, NULL, &temperature)) { valid = false; break; }
    fprintf(output, "detection.%u.label=%s\ndetection.%u.confidence=%.9g\ndetection.%u.native_x=%u\n"
      "detection.%u.native_y=%u\ndetection.%u.temperature_valid=%u\ndetection.%u.raw_kelvin64=%u\n",
      index, box->label, index, box->confidence, index, native_x, index, native_y,
      index, temperature.endpoint_codes ? 0U : 1U, index, temperature.min_raw);
    fprintf(output, "detection.%u.center_native_x=%.9g\ndetection.%u.center_native_y=%.9g\n"
      "detection.%u.box_display=%u,%u,%u,%u\ndetection.%u.temperature_sample_method=nearest_native_pixel\n",
      index, center_x, index, center_y, index, box->x, box->y, box->width, box->height, index);
    if (!temperature.endpoint_codes) fprintf(output, "detection.%u.reported_celsius=%.9g\n", index, temperature.min_raw / 64.0 - 273.15);
    struct coverage_point offset;
    const bool projected = aligned && coverage_project_pixel(&profile.calibration, &pose, center_x, center_y, &offset);
    fprintf(output, "detection.%u.position_valid=%u\n", index, projected ? 1U : 0U);
    if (projected) {
      const double degrees_per_metre = 180.0 / (3.14159265358979323846 * 6378137.0);
      fprintf(output, "detection.%u.latitude_deg=%.10f\ndetection.%u.longitude_deg=%.10f\n",
        index, pose.latitude_deg + offset.north * degrees_per_metre,
        index, pose.longitude_deg + offset.east * degrees_per_metre / cos(pose.latitude_deg / radians_to_degrees));
    }
  }
  if (ferror(output) || fflush(output) || fsync(fileno(output))) valid = false;
  if (fclose(output)) valid = false;
  if (lstat(filename, &after) || !same_file(&before, &after)) valid = false;
  /* Publish the complete sidecar without replacing an earlier result. */
  if (valid && link(pending_path, output_path)) valid = false;
  if (unlink(pending_path)) valid = false;
  return valid ? 0 : -1;
}