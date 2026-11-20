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
 * @file image_mock_transform.c
 * @brief Deterministic attitude-based JPEG transform used by local USHER simulation. */
#include "image_mock_transform.h"

#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <math.h>
#include <pthread.h>
#include <setjmp.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include <jpeglib.h>
#include "jpeg_marker_guard.h"

#define ANGLE_BFP_SCALE 4096.0
#define SPEED_BFP_SCALE 524288.0
#define POSITION_BFP_SCALE 256.0
#define RAD_TO_DEG 57.29577951308232
#define JPEG_COMPONENTS 3U
#define JPEG_QUALITY 95
#define FOCAL_LENGTH_FACTOR 0.9
#define PROJECTION_EPSILON 1e-9
#define PROJECTION_MIN_DEPTH 1e-4
#define PROJECTION_FOV_SAMPLES 192
#define PROJECTION_REFINE_STEPS 48
#define PROJECTION_MIN_RAY_SCALE 1e-4
#define COVER_SAFETY_FACTOR 0.999
#define IMAGE_MOCK_MAX_DIMENSION 8192U
#define IMAGE_MOCK_MAX_PIXELS (2028U * 1520U)
#define IMAGE_MOCK_RGB_CAPACITY ((size_t)IMAGE_MOCK_MAX_PIXELS * JPEG_COMPONENTS)
#define MAP_TILE_SIZE 256U
#define MAP_MOSAIC_DIMENSION (3U * MAP_TILE_SIZE)
#define MAP_MOSAIC_CAPACITY ((size_t)MAP_MOSAIC_DIMENSION * MAP_MOSAIC_DIMENSION * JPEG_COMPONENTS)

#ifndef near_horizon_case_black
  #define near_horizon_case_black 0
#endif

#ifndef near_horizon_roll_deg
  #define near_horizon_roll_deg 75.0
#endif

#ifndef near_horizon_pitch_deg
  #define near_horizon_pitch_deg 75.0
#endif

#ifndef near_horizon_combined_tilt_deg
  #define near_horizon_combined_tilt_deg 70.0
#endif

#if near_horizon_case_black != 0 && near_horizon_case_black != 1
  #error "near_horizon_case_black must be 0 or 1"
#endif

struct projection_fit {
  double ray_scale;
  double image_scale;
  double offset_x;
  double offset_y;
};

struct transform_jpeg_error {
  struct jpeg_error_mgr manager;
  jmp_buf recovery;
  void (*emit)(j_common_ptr, int);
};

struct decoded_image {
  unsigned char *pixels;
  size_t width;
  size_t height;
};

struct image_mock_workspace {
  unsigned char source[IMAGE_MOCK_RGB_CAPACITY];
  unsigned char destination[IMAGE_MOCK_RGB_CAPACITY];
  unsigned char mosaic[MAP_MOSAIC_CAPACITY];
  struct jpeg_decompress_struct decoder;
  struct jpeg_compress_struct encoder;
  struct transform_jpeg_error decode_error;
  struct transform_jpeg_error encode_error;
};

static struct image_mock_workspace workspace;
static pthread_mutex_t workspace_mutex = PTHREAD_MUTEX_INITIALIZER;

static void transform_jpeg_error_exit(j_common_ptr jpeg_info);
static bool is_near_horizon(double roll_rad, double pitch_rad);
static bool dimensions_fit(size_t width, size_t height);
static int read_jpeg_dimensions(FILE *input, struct decoded_image *image);
static int decode_jpeg(FILE *input, struct decoded_image *image);
static int encode_jpeg(FILE *output, const struct decoded_image *image);
static int transform_pixels(const struct decoded_image *source, struct decoded_image *destination,
                            double roll_rad, double pitch_rad, double yaw_rad,
                            bool synthesize_terrain);
static void build_rotation(double roll_rad, double pitch_rad, double yaw_rad,
                           double rotation[3][3]);
static bool project_point(double rotation[3][3], double focal_length,
                          double center_x, double center_y, double ray_scale,
                          double destination_x, double destination_y,
                          double *source_x, double *source_y);
static bool projection_corners_valid(const struct decoded_image *source,
                                     double rotation[3][3], double focal_length,
                                     double center_x, double center_y, double ray_scale);
static int compute_projection_fit_at_scale(const struct decoded_image *source,
    double rotation[3][3], double focal_length,
    double center_x, double center_y, double ray_scale,
    struct projection_fit *fit);
static int compute_projection_fit(const struct decoded_image *source,
                                  double rotation[3][3], double focal_length,
                                  double center_x, double center_y,
                                  struct projection_fit *fit);
static void bilinear_sample_rgb(const struct decoded_image *image, double source_x,
                                double source_y, unsigned char output[JPEG_COMPONENTS]);
static double mirror_coordinate(double coordinate, size_t extent);
static int create_black_frame(const char *filename);
static int replace_image_atomically(const char *filename, const struct decoded_image *image);
static int image_mock_map_locked(const char *filename, const char *cache_directory,
                                 const union dc_shot_union *shot, size_t width, size_t height,
                                 double horizontal_fov_deg, double vertical_fov_deg);
static int image_mock_transform_locked(const char *filename, const union dc_shot_union *shot);

/**
 * @brief Intersect a camera ray with a local flat ground plane.
 * @param rotation Body-to-north/east/down rotation.
 * @param forward Forward component of the body ray, with unit down component.
 * @param right Rightward component of that ray.
 * @param altitude_pixels Height above the plane at the local map scale.
 * @param center_x World pixel column below the aircraft.
 * @param center_y World pixel row below the aircraft.
 * @param[out] map_x Projected world pixel column on success.
 * @param[out] map_y Projected world pixel row on success.
 * @return True for a finite intersection in front of the camera.
 * @details Divide by the ray's downward component to intersect the plane.
 * Negate north when converting to image rows, which increase southward.
 */
static bool map_project(double rotation[3][3], double forward, double right,
                        double altitude_pixels, double center_x, double center_y,
                        double *map_x, double *map_y)
{
  double north = rotation[0][0] * forward + rotation[0][1] * right + rotation[0][2];
  double east = rotation[1][0] * forward + rotation[1][1] * right + rotation[1][2];
  double down = rotation[2][0] * forward + rotation[2][1] * right + rotation[2][2];
  if (down <= PROJECTION_MIN_DEPTH) return false;
  *map_x = center_x + altitude_pixels * east / down;
  *map_y = center_y - altitude_pixels * north / down;
  return isfinite(*map_x) && isfinite(*map_y);
}

/** @brief Render a map-backed simulated exposure.
 * @param filename Destination JPEG path.
 * @param cache_directory Local map tile cache root.
 * @param shot Trigger containing position, attitude and AGL.
 * @param width Output columns.
 * @param height Output rows.
 * @param horizontal_fov_deg Horizontal field of view in degrees.
 * @param vertical_fov_deg Vertical field of view in degrees.
 * @return Zero on publication, or -1 on lock, geometry, cache, or I/O failure.
 * @details Serialize rendering with fixture transforms because both reuse
 * the same codec and pixel workspace. */
int image_mock_map(const char *filename, const char *cache_directory,
                   const union dc_shot_union *shot, size_t width, size_t height,
                   double horizontal_fov_deg, double vertical_fov_deg)
{
  int lock_result = pthread_mutex_lock(&workspace_mutex);
  if (lock_result != 0) {
    errno = lock_result;
    return -1;
  }
  int result = image_mock_map_locked(filename, cache_directory, shot, width, height,
                                     horizontal_fov_deg, vertical_fov_deg);
  pthread_mutex_unlock(&workspace_mutex);
  return result;
}

/**
 * @brief Render a simulated camera view from cached map tiles.
 * @param filename Destination JPEG path.
 * @param cache_directory Root of the local map tile cache.
 * @param shot Trigger containing the position, attitude, and height above ground.
 * @param width Output width in pixels.
 * @param height Output height in pixels.
 * @param horizontal_fov_deg Horizontal field of view in degrees.
 * @param vertical_fov_deg Vertical field of view in degrees.
 * @return Zero on publication, or -1 on invalid geometry, missing data, or I/O failure.
 * @details With the workspace mutex held, project the image footprint onto
 * the zoom-18 map, load its required tiles, and inverse-sample the footprint.
 * Reject views outside the bounded tile workspace rather than filling gaps.
 */
static int image_mock_map_locked(const char *filename, const char *cache_directory,
                                 const union dc_shot_union *shot, size_t width, size_t height,
                                 double horizontal_fov_deg, double vertical_fov_deg)
{
  const int tile_size = MAP_TILE_SIZE;
  const int tile_count = 1 << 18;
  const double earth_radius = 6378137.0;
  const double pi = 3.14159265358979323846;
  if (filename == NULL || cache_directory == NULL || cache_directory[0] == '\0'
      || shot == NULL || !dimensions_fit(width, height)
      || !isfinite(horizontal_fov_deg) || !isfinite(vertical_fov_deg)
      || horizontal_fov_deg <= 0.0 || horizontal_fov_deg >= 179.0
      || vertical_fov_deg <= 0.0 || vertical_fov_deg >= 179.0
      || shot->data.groundalt <= 0 || fabs(shot->data.lat / 1e7) > 85.05112878
      || fabs(shot->data.lon / 1e7) > 180.0) {
    fprintf(stderr, "MOCK_MAP:\tinvalid pose, positive AGL, image size, or FOV required\n");
    errno = EINVAL;
    return -1;
  }
  double latitude = shot->data.lat / 1e7 / RAD_TO_DEG;
  double world_pixels = (double)tile_size * tile_count;
  double center_x = (shot->data.lon / 1e7 + 180.0) / 360.0 * world_pixels;
  double center_y = (1.0 - asinh(tan(latitude)) / pi) * 0.5 * world_pixels;
  /* Use the scale at the capture latitude: Mercator world pixels do not span
   * the same ground distance at the equator and near the poles. */
  double meters_per_pixel = 2.0 * pi * earth_radius * cos(latitude) / world_pixels;
  double altitude_pixels = shot->data.groundalt / POSITION_BFP_SCALE / meters_per_pixel;
  double half_horizontal = tan(horizontal_fov_deg / RAD_TO_DEG * 0.5);
  double half_vertical = tan(vertical_fov_deg / RAD_TO_DEG * 0.5);
  double rotation[3][3];
  build_rotation(shot->data.phi / ANGLE_BFP_SCALE, shot->data.theta / ANGLE_BFP_SCALE,
                 shot->data.psi / ANGLE_BFP_SCALE, rotation);
  double minimum_x = INFINITY, minimum_y = INFINITY;
  double maximum_x = -INFINITY, maximum_y = -INFINITY;
  for (int row = -1; row <= 1; row += 2) {
    for (int column = -1; column <= 1; column += 2) {
      double map_x, map_y;
      if (!map_project(rotation, -row * half_vertical, column * half_horizontal,
                       altitude_pixels, center_x, center_y, &map_x, &map_y)) {
        fprintf(stderr, "MOCK_MAP:\tcamera footprint reaches the horizon\n");
        errno = ERANGE;
        return -1;
      }
      minimum_x = fmin(minimum_x, map_x);
      minimum_y = fmin(minimum_y, map_y);
      maximum_x = fmax(maximum_x, map_x);
      maximum_y = fmax(maximum_y, map_y);
    }
  }
  int center_tile_x = (int)floor(center_x / tile_size);
  int center_tile_y = (int)floor(center_y / tile_size);
  if (minimum_x < (center_tile_x - 1.0) * tile_size
      || minimum_y < (center_tile_y - 1.0) * tile_size
      || ceil(maximum_x) >= (center_tile_x + 2.0) * tile_size
      || ceil(maximum_y) >= (center_tile_y + 2.0) * tile_size
      || minimum_y < 0.0 || ceil(maximum_y) >= world_pixels) {
    fprintf(stderr, "MOCK_MAP:\tfootprint exceeds the zoom-18 XYZ tile 3x3 neighborhood\n");
    errno = ERANGE;
    return -1;
  }
  int first_x = (int)floor(minimum_x / tile_size);
  int first_y = (int)floor(minimum_y / tile_size);
  int last_x = (int)floor(ceil(maximum_x) / tile_size);
  int last_y = (int)floor(ceil(maximum_y) / tile_size);
  struct decoded_image mosaic = {
    workspace.mosaic, (size_t)(last_x - first_x + 1) * tile_size,
    (size_t)(last_y - first_y + 1) * tile_size
  };
  for (int tile_y = first_y; tile_y <= last_y; ++tile_y) {
    for (int tile_x = first_x; tile_x <= last_x; ++tile_x) {
      char path[PATH_MAX];
      int wrapped_x = (tile_x % tile_count + tile_count) % tile_count;
      int length = snprintf(path, sizeof(path), "%s/GOOGLE/18/%d/%d.jpeg",
                            cache_directory, wrapped_x, tile_y);
      if (length < 0 || (size_t)length >= sizeof(path)) {
        errno = ENAMETOOLONG;
        return -1;
      }
      FILE *input = fopen(path, "rb");
      struct decoded_image tile = {0};
      int result = input == NULL ? -1 : read_jpeg_dimensions(input, &tile);
      if (result == 0 && (tile.width != (size_t)tile_size || tile.height != (size_t)tile_size)) {
        result = -1;
      }
      if (result == 0) {
        rewind(input);
        result = decode_jpeg(input, &tile);
      }
      if (input != NULL && fclose(input) != 0) result = -1;
      if (result != 0) {
        fprintf(stderr, "MOCK_MAP:\tmissing or invalid 256x256 JPEG tile: %s\n", path);
        return -1;
      }
      size_t offset_x = (size_t)(tile_x - first_x) * tile_size;
      size_t offset_y = (size_t)(tile_y - first_y) * tile_size;
      for (size_t row = 0; row < tile.height && row + offset_y < mosaic.height; ++row) {
        for (size_t column = 0; column < tile.width && column + offset_x < mosaic.width; ++column) {
          for (size_t component = 0; component < JPEG_COMPONENTS; ++component) {
            mosaic.pixels[((row + offset_y) * mosaic.width + column + offset_x) * JPEG_COMPONENTS + component]
              = tile.pixels[(row * tile.width + column) * JPEG_COMPONENTS + component];
          }
        }
      }
    }
  }
  struct decoded_image image = {workspace.destination, width, height};
  for (size_t row = 0; row < height; ++row) {
    double forward = (1.0 - 2.0 * row / (height - 1)) * half_vertical;
    for (size_t column = 0; column < width; ++column) {
      double right = (2.0 * column / (width - 1) - 1.0) * half_horizontal;
      double map_x, map_y;
      if (!map_project(rotation, forward, right, altitude_pixels, center_x, center_y, &map_x, &map_y)) {
        return -1;
      }
      map_x -= (double)first_x * tile_size;
      map_y -= (double)first_y * tile_size;
      if (map_x < 0.0 || map_y < 0.0 || map_x > mosaic.width - 1 || map_y > mosaic.height - 1) {
        return -1;
      }
      bilinear_sample_rgb(&mosaic, map_x, map_y, &image.pixels[(row * width + column) * JPEG_COMPONENTS]);
    }
  }
  int result = replace_image_atomically(filename, &image);
  if (result == 0) {
    printf("MOCK_MAP:\tXYZ z18 tiles %d..%d/%d..%d | %.7f,%.7f AGL %.2fm FOV %.2fx%.2f | %s\n",
           first_x, last_x, first_y, last_y, shot->data.lat / 1e7, shot->data.lon / 1e7,
           shot->data.groundalt / POSITION_BFP_SCALE, horizontal_fov_deg, vertical_fov_deg, filename);
  }
  return result;
}

int image_mock_transform(const char *filename, const union dc_shot_union *shot)
{
  int lock_result = pthread_mutex_lock(&workspace_mutex);
  if (lock_result != 0) {
    errno = lock_result;
    return -1;
  }
  int result = image_mock_transform_locked(filename, shot);
  pthread_mutex_unlock(&workspace_mutex);
  return result;
}

/**
 * @brief Apply simulated attitude to the existing fixture JPEG.
 * @param filename Fixture JPEG to replace if a transform is applied.
 * @param shot Trigger containing fixed-point aircraft attitude.
 * @return Applied or skipped status, or -1 on failure.
 * @details With the workspace mutex held, select the configured horizon policy
 * before full decode. Skip an effectively identity transform to avoid an
 * unnecessary lossy JPEG re-encode.
 */
static int image_mock_transform_locked(const char *filename, const union dc_shot_union *shot)
{
  if (filename == NULL || shot == NULL) {
    errno = EINVAL;
    return -1;
  }
  if (!isfinite(near_horizon_roll_deg) || near_horizon_roll_deg < 0.0
      || near_horizon_roll_deg > 180.0
      || !isfinite(near_horizon_pitch_deg) || near_horizon_pitch_deg < 0.0
      || near_horizon_pitch_deg > 180.0
      || !isfinite(near_horizon_combined_tilt_deg)
      || near_horizon_combined_tilt_deg < 0.0
      || near_horizon_combined_tilt_deg > 180.0) {
    fprintf(stderr, "MOCK_TRANSFORM:\tnear-horizon thresholds must be between 0 and 180 degrees\n");
    errno = EINVAL;
    return -1;
  }

  double altitude_m = shot->data.alt / 1000.0;
  double ground_altitude_m = shot->data.groundalt / POSITION_BFP_SCALE;
  double roll_rad = shot->data.phi / ANGLE_BFP_SCALE;
  double pitch_rad = shot->data.theta / ANGLE_BFP_SCALE;
  double yaw_rad = shot->data.psi / ANGLE_BFP_SCALE;
  double speed_m_s = shot->data.vground / SPEED_BFP_SCALE;
  double course_deg = shot->data.course / ANGLE_BFP_SCALE * RAD_TO_DEG;
  double blur_factor = speed_m_s / 100.0;
  bool near_horizon = is_near_horizon(roll_rad, pitch_rad);

  printf("MOCK_TRANSFORM:\talt %.3f m AGL %.3f m | roll %.3f pitch %.3f yaw %.3f deg | "
         "speed %.3f m/s course %.3f deg | blur factor %.6f (not applied)\n",
         altitude_m, ground_altitude_m,
         roll_rad * RAD_TO_DEG, pitch_rad * RAD_TO_DEG, yaw_rad * RAD_TO_DEG,
         speed_m_s, course_deg, blur_factor);

  if (near_horizon && near_horizon_case_black) {
    printf("MOCK_TRANSFORM:\tnear-horizon attitude: generating black frame\n");
    return create_black_frame(filename);
  }

  if (!near_horizon && fabs(roll_rad) < PROJECTION_EPSILON
      && fabs(pitch_rad) < PROJECTION_EPSILON && fabs(yaw_rad) < PROJECTION_EPSILON) {
    return IMAGE_MOCK_TRANSFORM_SKIPPED;
  }

  FILE *input = fopen(filename, "rb");
  if (input == NULL) {
    fprintf(stderr, "MOCK_TRANSFORM:\tfailed to open %s: %s\n", filename, strerror(errno));
    return -1;
  }

  struct decoded_image source = {workspace.source, 0, 0};
  if (decode_jpeg(input, &source) != 0) {
    fprintf(stderr, "MOCK_TRANSFORM:\tfailed to decode JPEG %s\n", filename);
    fclose(input);
    return -1;
  }
  if (fclose(input) != 0) {
    return -1;
  }

  struct decoded_image transformed = {workspace.destination, source.width, source.height};
  int result = transform_pixels(&source, &transformed, roll_rad, pitch_rad, yaw_rad,
                                near_horizon);
  if (result < 0) {
    return -1;
  }
  if (result == IMAGE_MOCK_TRANSFORM_SKIPPED) {
    return result;
  }

  result = replace_image_atomically(filename, &transformed);
  return result;
}

/**
 * @brief Redirect a fatal libjpeg error to the active codec operation.
 * @param jpeg_info Codec state with this module's installed error manager.
 * @details The codec caller owns cleanup at its recovery point; the callback
 * does not return and must not run without an active recovery frame.
 */
static void transform_jpeg_error_exit(j_common_ptr jpeg_info)
{
  struct transform_jpeg_error *error = (struct transform_jpeg_error *)jpeg_info->err;
  longjmp(error->recovery, 1);
}

/**
 * @brief Check marker bounds before forwarding a libjpeg diagnostic.
 * @param state Codec state with this module's installed error manager.
 * @param level libjpeg diagnostic level.
 * @details Preserve the installed diagnostic behavior for accepted input.
 * A marker rejected by the guard takes the codec's fatal recovery path.
 */
static void transform_jpeg_message(j_common_ptr state, int level)
{
  struct transform_jpeg_error *error = (struct transform_jpeg_error *)state->err;
  usher_jpeg_validate_marker(state);
  error->emit(state, level);
}

/**
 * @brief Apply both axis-specific and combined near-horizon limits.
 * @param roll_rad Aircraft roll in radians.
 * @param pitch_rad Aircraft pitch in radians.
 * @return True if any configured tilt threshold is reached.
 * @details The down-axis cosine combines roll and pitch, catching attitudes
 * where neither axis alone reaches its limit. Clamp it before inverse cosine.
 */
static bool is_near_horizon(double roll_rad, double pitch_rad)
{
  double absolute_roll_deg = fabs(roll_rad * RAD_TO_DEG);
  double absolute_pitch_deg = fabs(pitch_rad * RAD_TO_DEG);
  double optical_axis_cosine = cos(roll_rad) * cos(pitch_rad);
  optical_axis_cosine = fmax(-1.0, fmin(1.0, optical_axis_cosine));
  double combined_tilt_deg = acos(optical_axis_cosine) * RAD_TO_DEG;
  return absolute_roll_deg >= near_horizon_roll_deg
         || absolute_pitch_deg >= near_horizon_pitch_deg
         || combined_tilt_deg >= near_horizon_combined_tilt_deg;
}

/**
 * @brief Check that an image can fit the bounded RGB workspace.
 * @param width Pixel columns.
 * @param height Pixel rows.
 * @return True for dimensions of at least two pixels within axis and area limits.
 * @details Test area by division so a malformed header cannot overflow the check.
 */
static bool dimensions_fit(size_t width, size_t height)
{
  return width >= 2 && height >= 2
         && width <= IMAGE_MOCK_MAX_DIMENSION && height <= IMAGE_MOCK_MAX_DIMENSION
         && width <= IMAGE_MOCK_MAX_PIXELS / height;
}

/**
 * @brief Read bounded image dimensions without decoding pixels.
 * @param input Open JPEG stream positioned at the start of the image.
 * @param[out] image Descriptor whose width and height are set on success.
 * @return Zero on success, or -1 for a codec or dimension error.
 * @details The black-frame fallback needs the original size only. Reuse the
 * locked decoder workspace and leave stream ownership with the caller.
 */
static int read_jpeg_dimensions(FILE *input, struct decoded_image *image)
{
  struct jpeg_decompress_struct *decoder = &workspace.decoder;
  struct transform_jpeg_error *error = &workspace.decode_error;
  memset(decoder, 0, sizeof(*decoder));
  decoder->err = jpeg_std_error(&error->manager);
  error->emit = error->manager.emit_message;
  error->manager.emit_message = transform_jpeg_message;
  error->manager.error_exit = transform_jpeg_error_exit;
  if (setjmp(error->recovery) != 0) {
    jpeg_destroy_decompress(decoder);
    return -1;
  }

  jpeg_create_decompress(decoder);
  jpeg_stdio_src(decoder, input);
  if (jpeg_read_header(decoder, TRUE) != JPEG_HEADER_OK
      || !dimensions_fit(decoder->image_width, decoder->image_height)) {
    jpeg_destroy_decompress(decoder);
    errno = EINVAL;
    return -1;
  }
  image->width = decoder->image_width;
  image->height = decoder->image_height;
  jpeg_destroy_decompress(decoder);
  return 0;
}

/**
 * @brief Decode bounded RGB scanlines from an open JPEG stream.
 * @param input JPEG stream positioned at the start of the image.
 * @param[in,out] image Destination descriptor; a null pixel buffer selects the
 *                      shared source buffer. A supplied buffer must fit the image.
 * @return Zero for a complete decode, or -1 on failure; discard partial output.
 * @details Validate both header and output dimensions before indexing the RGB
 * buffer. The caller holds the workspace mutex and owns the input stream.
 */
static int decode_jpeg(FILE *input, struct decoded_image *image)
{
  struct jpeg_decompress_struct *decoder = &workspace.decoder;
  struct transform_jpeg_error *error = &workspace.decode_error;
  memset(decoder, 0, sizeof(*decoder));
  decoder->err = jpeg_std_error(&error->manager);
  error->emit = error->manager.emit_message;
  error->manager.emit_message = transform_jpeg_message;
  error->manager.error_exit = transform_jpeg_error_exit;
  if (setjmp(error->recovery) != 0) {
    jpeg_destroy_decompress(decoder);
    return -1;
  }

  jpeg_create_decompress(decoder);
  jpeg_stdio_src(decoder, input);
  if (jpeg_read_header(decoder, TRUE) != JPEG_HEADER_OK
      || !dimensions_fit(decoder->image_width, decoder->image_height)) {
    jpeg_destroy_decompress(decoder);
    errno = EOVERFLOW;
    return -1;
  }
  decoder->out_color_space = JCS_RGB;
  jpeg_start_decompress(decoder);

  image->width = decoder->output_width;
  image->height = decoder->output_height;
  if (decoder->output_components != JPEG_COMPONENTS || image->width < 2 || image->height < 2
      || !dimensions_fit(image->width, image->height)) {
    jpeg_destroy_decompress(decoder);
    errno = EOVERFLOW;
    return -1;
  }

  size_t row_size = image->width * JPEG_COMPONENTS;
  if (image->pixels == NULL) {
    image->pixels = workspace.source;
  }

  while (decoder->output_scanline < decoder->output_height) {
    JSAMPROW row[1] = {&image->pixels[(size_t)decoder->output_scanline * row_size]};
    if (jpeg_read_scanlines(decoder, row, 1) != 1) {
      jpeg_destroy_decompress(decoder);
      return -1;
    }
  }

  jpeg_finish_decompress(decoder);
  jpeg_destroy_decompress(decoder);
  return 0;
}

/**
 * @brief Encode a bounded interleaved RGB image at the simulation quality setting.
 * @param output Open output stream, closed by the caller.
 * @param image Populated RGB image descriptor.
 * @return Zero on completion, or -1 for an invalid image or codec failure.
 * @details Reuse the locked encoder workspace. Publication is the caller's
 * responsibility: a failed encode may leave partial output on the stream.
 */
static int encode_jpeg(FILE *output, const struct decoded_image *image)
{
  if (!dimensions_fit(image->width, image->height) || image->pixels == NULL) {
    errno = EOVERFLOW;
    return -1;
  }
  struct jpeg_compress_struct *encoder = &workspace.encoder;
  struct transform_jpeg_error *error = &workspace.encode_error;
  memset(encoder, 0, sizeof(*encoder));
  encoder->err = jpeg_std_error(&error->manager);
  error->manager.error_exit = transform_jpeg_error_exit;
  if (setjmp(error->recovery) != 0) {
    jpeg_destroy_compress(encoder);
    return -1;
  }

  jpeg_create_compress(encoder);
  jpeg_stdio_dest(encoder, output);
  encoder->image_width = (JDIMENSION)image->width;
  encoder->image_height = (JDIMENSION)image->height;
  encoder->input_components = JPEG_COMPONENTS;
  encoder->in_color_space = JCS_RGB;
  jpeg_set_defaults(encoder);
  jpeg_set_quality(encoder, JPEG_QUALITY, TRUE);
  jpeg_start_compress(encoder, TRUE);

  size_t row_size = image->width * JPEG_COMPONENTS;
  while (encoder->next_scanline < encoder->image_height) {
    JSAMPROW row[1] = {(JSAMPROW) &image->pixels[(size_t)encoder->next_scanline * row_size]};
    if (jpeg_write_scanlines(encoder, row, 1) != 1) {
      jpeg_destroy_compress(encoder);
      return -1;
    }
  }

  jpeg_finish_compress(encoder);
  jpeg_destroy_compress(encoder);
  return 0;
}

/**
 * @brief Resample a fixture into a camera view at the supplied attitude.
 * @param source Decoded fixture RGB raster.
 * @param[out] destination Separate output raster with caller-provided storage.
 * @param roll_rad Aircraft roll in radians.
 * @param pitch_rad Aircraft pitch in radians.
 * @param yaw_rad Aircraft yaw in radians.
 * @param synthesize_terrain Initially allow mirrored terrain beyond the fixture.
 * @return @c IMAGE_MOCK_TRANSFORM_APPLIED on success, or -1 on failure.
 * @details Fit a borderless crop when possible. For the near-horizon fallback,
 * reflect out-of-bounds samples instead of claiming the fixture covers the view.
 * In both cases inverse-map the destination and sample the fixture bilinearly.
 */
static int transform_pixels(const struct decoded_image *source, struct decoded_image *destination,
                            double roll_rad, double pitch_rad, double yaw_rad,
                            bool synthesize_terrain)
{
  double center_x = ((double)source->width - 1.0) / 2.0;
  double center_y = ((double)source->height - 1.0) / 2.0;
  double largest_dimension = source->width > source->height ? source->width : source->height;
  double focal_length = largest_dimension * FOCAL_LENGTH_FACTOR;
  double rotation[3][3];
  build_rotation(roll_rad, pitch_rad, yaw_rad, rotation);
  if (rotation[2][2] <= PROJECTION_MIN_DEPTH) {
    synthesize_terrain = true;
  }
  struct projection_fit fit;
  if (synthesize_terrain) {
    fit.ray_scale = 1.0;
    fit.image_scale = 1.0;
    fit.offset_x = 0.0;
    fit.offset_y = 0.0;
    printf("MOCK_TRANSFORM:\tnear-horizon attitude: synthesizing extended terrain\n");
  } else if (compute_projection_fit(source, rotation, focal_length, center_x, center_y, &fit) != 0) {
    fprintf(stderr, "MOCK_TRANSFORM:\tattitude is outside the usable single-image camera model\n");
    return -1;
  }

  if (!dimensions_fit(source->width, source->height)
      || destination->pixels == NULL) {
    errno = EOVERFLOW;
    return -1;
  }
  if (!synthesize_terrain) {
    double cover_zoom = 1.0 / (fit.ray_scale * fit.image_scale);
    double pan_x = fit.offset_x + (fit.image_scale - 1.0) * center_x;
    double pan_y = fit.offset_y + (fit.image_scale - 1.0) * center_y;
    printf("MOCK_TRANSFORM:\tborderless cover crop: %.3fx zoom, pan %.1f px x %.1f px\n",
           cover_zoom, pan_x, pan_y);
  }

  /* The unnormalized ray is affine across a scanline; advance it by a
   * constant step rather than multiplying by the rotation for every pixel. */
  double ray_x_start = -fit.ray_scale * center_x / focal_length;
  double ray_x_increment = fit.ray_scale / focal_length;
  double source_ray_x_increment = rotation[0][0] * ray_x_increment;
  double source_ray_y_increment = rotation[0][1] * ray_x_increment;
  double source_ray_z_increment = rotation[0][2] * ray_x_increment;

  for (size_t destination_y = 0; destination_y < destination->height; destination_y++) {
    double ray_y = fit.ray_scale * ((double)destination_y - center_y) / focal_length;
    double source_ray_x = rotation[0][0] * ray_x_start + rotation[1][0] * ray_y + rotation[2][0];
    double source_ray_y = rotation[0][1] * ray_x_start + rotation[1][1] * ray_y + rotation[2][1];
    double source_ray_z = rotation[0][2] * ray_x_start + rotation[1][2] * ray_y + rotation[2][2];

    for (size_t destination_x = 0; destination_x < destination->width; destination_x++) {
      double projection_depth = source_ray_z;
      if (projection_depth <= PROJECTION_MIN_DEPTH) {
        if (!synthesize_terrain) {
          return -1;
        }
        projection_depth = projection_depth < 0.0
                           ? fmin(projection_depth, -PROJECTION_MIN_DEPTH)
                           : PROJECTION_MIN_DEPTH;
      }
      double inverse_depth = focal_length / projection_depth;
      double source_x = source_ray_x * inverse_depth + center_x;
      double source_y = source_ray_y * inverse_depth + center_y;

      source_x = fit.image_scale * source_x + fit.offset_x;
      source_y = fit.image_scale * source_y + fit.offset_y;
      if (synthesize_terrain) {
        source_x = mirror_coordinate(source_x, source->width);
        source_y = mirror_coordinate(source_y, source->height);
      } else {
        source_x = fmax(0.0, fmin((double)(source->width - 1), source_x));
        source_y = fmax(0.0, fmin((double)(source->height - 1), source_y));
      }

      size_t destination_offset = (destination_y * destination->width + destination_x) * JPEG_COMPONENTS;
      bilinear_sample_rgb(source, source_x, source_y,
                          &destination->pixels[destination_offset]);
      source_ray_x += source_ray_x_increment;
      source_ray_y += source_ray_y_increment;
      source_ray_z += source_ray_z_increment;
    }
  }
  return IMAGE_MOCK_TRANSFORM_APPLIED;
}

/**
 * @brief Reflect a sample coordinate into the source image.
 * @param coordinate Finite source position, possibly outside the image.
 * @param extent Axis length, at least two pixels.
 * @return A reflected coordinate in [0, extent - 1].
 * @details Use a triangle wave across repeated reflections so the fallback
 * has no hard seam caused by wrapping from one edge to the opposite edge.
 */
static double mirror_coordinate(double coordinate, size_t extent)
{
  double maximum = (double)(extent - 1);
  double period = 2.0 * maximum;
  double mirrored = fmod(coordinate, period);
  if (mirrored < 0.0) {
    mirrored += period;
  }
  return mirrored <= maximum ? mirrored : period - mirrored;
}

/**
 * @brief Replace the fixture with a black JPEG of the same dimensions.
 * @param filename Fixture JPEG path.
 * @return Zero on replacement, or -1 on header, codec, or I/O failure.
 * @details Read only the original header: near-horizon black frames do not
 * need any source pixels.
 */
static int create_black_frame(const char *filename)
{
  FILE *input = fopen(filename, "rb");
  if (input == NULL) {
    return -1;
  }
  struct decoded_image black = {NULL, 0, 0};
  int result = read_jpeg_dimensions(input, &black);
  if (fclose(input) != 0) {
    result = -1;
  }
  if (result != 0 || black.width > SIZE_MAX / JPEG_COMPONENTS
      || !dimensions_fit(black.width, black.height)) {
    return -1;
  }
  black.pixels = workspace.destination;
  memset(black.pixels, 0, black.width * black.height * JPEG_COMPONENTS);
  result = replace_image_atomically(filename, &black);
  return result;
}

/**
 * @brief Construct the body-to-north/east/down attitude rotation.
 * @param roll_rad Roll about the forward axis, in radians.
 * @param pitch_rad Pitch about the right axis, in radians.
 * @param yaw_rad Heading rotation about the down axis, in radians.
 * @param[out] rotation Row-major 3 by 3 rotation.
 * @details Compose yaw, pitch, and roll in that order. The map renderer
 * uses the forward mapping; the fixture warp uses its transpose.
 */
static void build_rotation(double roll_rad, double pitch_rad, double yaw_rad,
                           double rotation[3][3])
{
  double cosine_roll = cos(roll_rad);
  double sine_roll = sin(roll_rad);
  double cosine_pitch = cos(pitch_rad);
  double sine_pitch = sin(pitch_rad);
  double cosine_yaw = cos(yaw_rad);
  double sine_yaw = sin(yaw_rad);

  rotation[0][0] = cosine_yaw * cosine_pitch;
  rotation[0][1] = cosine_yaw * sine_pitch * sine_roll - sine_yaw * cosine_roll;
  rotation[0][2] = cosine_yaw * sine_pitch * cosine_roll + sine_yaw * sine_roll;
  rotation[1][0] = sine_yaw * cosine_pitch;
  rotation[1][1] = sine_yaw * sine_pitch * sine_roll + cosine_yaw * cosine_roll;
  rotation[1][2] = sine_yaw * sine_pitch * cosine_roll - cosine_yaw * sine_roll;
  rotation[2][0] = -sine_pitch;
  rotation[2][1] = cosine_pitch * sine_roll;
  rotation[2][2] = cosine_pitch * cosine_roll;
}

/**
 * @brief Map a destination pixel back into the fixture JPEG.
 * @param rotation Attitude rotation, applied here by its transpose.
 * @param focal_length Pinhole focal length in pixels.
 * @param center_x Principal point column.
 * @param center_y Principal point row.
 * @param ray_scale Field-of-view contraction applied before rotation.
 * @param destination_x Output pixel column.
 * @param destination_y Output pixel row.
 * @param[out] source_x Projected source column on success.
 * @param[out] source_y Projected source row on success.
 * @return True for a finite projection with sufficient positive depth.
 * @details Use inverse mapping to avoid holes between forward-projected
 * pixels. The caller still needs to check source raster bounds.
 */
static bool project_point(double rotation[3][3], double focal_length,
                          double center_x, double center_y, double ray_scale,
                          double destination_x, double destination_y,
                          double *source_x, double *source_y)
{
  double ray_x = ray_scale * (destination_x - center_x) / focal_length;
  double ray_y = ray_scale * (destination_y - center_y) / focal_length;
  double source_ray_x = rotation[0][0] * ray_x + rotation[1][0] * ray_y + rotation[2][0];
  double source_ray_y = rotation[0][1] * ray_x + rotation[1][1] * ray_y + rotation[2][1];
  double source_ray_z = rotation[0][2] * ray_x + rotation[1][2] * ray_y + rotation[2][2];
  if (source_ray_z <= PROJECTION_MIN_DEPTH) {
    return false;
  }

  *source_x = focal_length * source_ray_x / source_ray_z + center_x;
  *source_y = focal_length * source_ray_y / source_ray_z + center_y;
  return isfinite(*source_x) && isfinite(*source_y);
}

/**
 * @brief Check for a horizon crossing in the candidate projection.
 * @param source Image descriptor defining the candidate raster dimensions.
 * @param rotation Attitude rotation.
 * @param focal_length Focal length in pixels.
 * @param center_x Principal point column.
 * @param center_y Principal point row.
 * @param ray_scale Candidate field-of-view contraction.
 * @return True if each corner has a finite, positive-depth projection.
 * @details Depth is affine in destination coordinates, so corner checks
 * bound the depth over the whole rectangle. They do not check image coverage.
 */
static bool projection_corners_valid(const struct decoded_image *source,
                                     double rotation[3][3], double focal_length,
                                     double center_x, double center_y, double ray_scale)
{
  const double corners[4][2] = {
    {0.0, 0.0},
    {(double)(source->width - 1), 0.0},
    {(double)(source->width - 1), (double)(source->height - 1)},
    {0.0, (double)(source->height - 1)}
  };
  for (size_t index = 0; index < 4; index++) {
    double source_x;
    double source_y;
    if (!project_point(rotation, focal_length, center_x, center_y, ray_scale,
                       corners[index][0], corners[index][1], &source_x, &source_y)) {
      return false;
    }
  }
  return true;
}

/**
 * @brief Find a fitted projection with low effective zoom.
 * @param source Source raster descriptor.
 * @param rotation Attitude rotation.
 * @param focal_length Focal length in pixels.
 * @param center_x Principal point column.
 * @param center_y Principal point row.
 * @param[out] fit Best candidate found by the bounded search.
 * @return Zero if a candidate was found, or -1 if none was usable.
 * @details Sample ray scales on a logarithmic grid, then refine around
 * the best candidate. This bounds search work without claiming a global optimum.
 */
static int compute_projection_fit(const struct decoded_image *source,
                                  double rotation[3][3], double focal_length,
                                  double center_x, double center_y,
                                  struct projection_fit *fit)
{
  if (rotation[2][2] <= PROJECTION_MIN_DEPTH) {
    errno = ERANGE;
    return -1;
  }

  double minimum_log_scale = log(PROJECTION_MIN_RAY_SCALE);
  double best_log_scale = minimum_log_scale;
  double best_zoom = INFINITY;
  bool fit_found = false;
  for (int sample = 0; sample <= PROJECTION_FOV_SAMPLES; sample++) {
    double fraction = (double)sample / PROJECTION_FOV_SAMPLES;
    double log_scale = minimum_log_scale * (1.0 - fraction);
    double ray_scale = exp(log_scale);
    struct projection_fit candidate;
    if (compute_projection_fit_at_scale(source, rotation, focal_length,
                                        center_x, center_y, ray_scale, &candidate) != 0) {
      continue;
    }
    double zoom = 1.0 / (candidate.ray_scale * candidate.image_scale);
    if (zoom < best_zoom) {
      best_zoom = zoom;
      best_log_scale = log_scale;
      *fit = candidate;
      fit_found = true;
    }
  }
  if (!fit_found) {
    errno = ERANGE;
    return -1;
  }

  double coarse_step = -minimum_log_scale / PROJECTION_FOV_SAMPLES;
  double lower_log_scale = fmax(minimum_log_scale, best_log_scale - coarse_step);
  double upper_log_scale = fmin(0.0, best_log_scale + coarse_step);
  for (int step = 0; step < PROJECTION_REFINE_STEPS; step++) {
    double first_log_scale = (2.0 * lower_log_scale + upper_log_scale) / 3.0;
    double second_log_scale = (lower_log_scale + 2.0 * upper_log_scale) / 3.0;
    struct projection_fit first_fit;
    struct projection_fit second_fit;
    bool first_valid = compute_projection_fit_at_scale(source, rotation, focal_length,
                       center_x, center_y, exp(first_log_scale),
                       &first_fit) == 0;
    bool second_valid = compute_projection_fit_at_scale(source, rotation, focal_length,
                        center_x, center_y, exp(second_log_scale),
                        &second_fit) == 0;
    double first_zoom = first_valid ? 1.0 / (first_fit.ray_scale * first_fit.image_scale) : INFINITY;
    double second_zoom = second_valid ? 1.0 / (second_fit.ray_scale * second_fit.image_scale) : INFINITY;
    if (first_zoom < best_zoom) {
      best_zoom = first_zoom;
      *fit = first_fit;
    }
    if (second_zoom < best_zoom) {
      best_zoom = second_zoom;
      *fit = second_fit;
    }
    if (first_zoom <= second_zoom) {
      upper_log_scale = second_log_scale;
    } else {
      lower_log_scale = first_log_scale;
    }
  }
  return 0;
}

/**
 * @brief Fit one projected destination rectangle into the source raster.
 * @param source Source raster descriptor.
 * @param rotation Attitude rotation.
 * @param focal_length Focal length in pixels.
 * @param center_x Principal point column.
 * @param center_y Principal point row.
 * @param ray_scale Candidate field-of-view contraction.
 * @param[out] fit Sampling scale and offsets; use only on success.
 * @return Zero for a usable fit, or -1 for a singular or degenerate projection.
 * @details Measure the projected corner bounds and fit both axes with a
 * common scale to avoid stretching. Preserve a full-scale fit that is
 * already covered; otherwise center the bounds with a numerical margin.
 */
static int compute_projection_fit_at_scale(const struct decoded_image *source,
    double rotation[3][3], double focal_length,
    double center_x, double center_y, double ray_scale,
    struct projection_fit *fit)
{
  if (!projection_corners_valid(source, rotation, focal_length, center_x, center_y,
                                ray_scale)) {
    return -1;
  }
  fit->ray_scale = ray_scale;

  const double corners[4][2] = {
    {0.0, 0.0},
    {(double)(source->width - 1), 0.0},
    {(double)(source->width - 1), (double)(source->height - 1)},
    {0.0, (double)(source->height - 1)}
  };
  double minimum_x = INFINITY;
  double minimum_y = INFINITY;
  double maximum_x = -INFINITY;
  double maximum_y = -INFINITY;
  for (size_t index = 0; index < 4; index++) {
    double source_x;
    double source_y;
    if (!project_point(rotation, focal_length, center_x, center_y, fit->ray_scale,
                       corners[index][0], corners[index][1], &source_x, &source_y)) {
      errno = ERANGE;
      return -1;
    }
    minimum_x = fmin(minimum_x, source_x);
    minimum_y = fmin(minimum_y, source_y);
    maximum_x = fmax(maximum_x, source_x);
    maximum_y = fmax(maximum_y, source_y);
  }

  double span_x = maximum_x - minimum_x;
  double span_y = maximum_y - minimum_y;
  if (span_x <= PROJECTION_EPSILON || span_y <= PROJECTION_EPSILON) {
    errno = ERANGE;
    return -1;
  }

  double horizontal_fit = (double)(source->width - 1) / span_x;
  double vertical_fit = (double)(source->height - 1) / span_y;
  bool already_covered = minimum_x >= 0.0 && minimum_y >= 0.0
                         && maximum_x <= (double)(source->width - 1)
                         && maximum_y <= (double)(source->height - 1);
  if (already_covered && fit->ray_scale == 1.0) {
    fit->image_scale = 1.0;
    fit->offset_x = 0.0;
    fit->offset_y = 0.0;
    return 0;
  }

  fit->image_scale = fmin(1.0, fmin(horizontal_fit, vertical_fit)) * COVER_SAFETY_FACTOR;
  fit->offset_x = center_x - fit->image_scale * (minimum_x + maximum_x) / 2.0;
  fit->offset_y = center_y - fit->image_scale * (minimum_y + maximum_y) / 2.0;
  return 0;
}

/**
 * @brief Sample the RGB raster at an in-bounds fractional position.
 * @param image Source RGB raster.
 * @param source_x Fractional pixel column in the valid raster range.
 * @param source_y Fractional pixel row in the valid raster range.
 * @param[out] output Three-byte RGB sample.
 * @details Interpolate channel values along each axis and collapse the
 * neighbor to the edge pixel on the last row or column.
 */
static void bilinear_sample_rgb(const struct decoded_image *image, double source_x,
                                double source_y, unsigned char output[JPEG_COMPONENTS])
{
  size_t left = (size_t)floor(source_x);
  size_t top = (size_t)floor(source_y);
  size_t right = left + 1 < image->width ? left + 1 : left;
  size_t bottom = top + 1 < image->height ? top + 1 : top;
  double horizontal = source_x - (double)left;
  double vertical = source_y - (double)top;

  size_t top_left = (top * image->width + left) * JPEG_COMPONENTS;
  size_t top_right = (top * image->width + right) * JPEG_COMPONENTS;
  size_t bottom_left = (bottom * image->width + left) * JPEG_COMPONENTS;
  size_t bottom_right = (bottom * image->width + right) * JPEG_COMPONENTS;
  for (size_t component = 0; component < JPEG_COMPONENTS; component++) {
    double top_value = image->pixels[top_left + component] * (1.0 - horizontal)
                       + image->pixels[top_right + component] * horizontal;
    double bottom_value = image->pixels[bottom_left + component] * (1.0 - horizontal)
                          + image->pixels[bottom_right + component] * horizontal;
    double value = top_value * (1.0 - vertical) + bottom_value * vertical;
    output[component] = (unsigned char)(value + 0.5);
  }
}

/**
 * @brief Publish a complete simulated JPEG in place of its fixture.
 * @param filename Final JPEG path.
 * @param image RGB raster to encode.
 * @return Zero on replacement, or -1 on an encode or filesystem error.
 * @details Encode under a temporary sibling name so readers of the final
 * path do not see a partially written JPEG.
 */
static int replace_image_atomically(const char *filename, const struct decoded_image *image)
{
  struct stat original_status = {.st_mode = 0644};
  if (stat(filename, &original_status) != 0 && errno != ENOENT) {
    return -1;
  }

  size_t filename_length = strlen(filename);
  const char suffix[] = ".transform.XXXXXX";
  char temporary_name[PATH_MAX];
  if (filename_length >= sizeof(temporary_name)
      || sizeof(suffix) > sizeof(temporary_name) - filename_length) {
    errno = ENAMETOOLONG;
    return -1;
  }
  int name_length = snprintf(temporary_name, sizeof(temporary_name), "%s%s", filename, suffix);
  if (name_length < 0 || (size_t)name_length >= sizeof(temporary_name)) {
    errno = ENAMETOOLONG;
    return -1;
  }

  int temporary_fd = mkstemp(temporary_name);
  if (temporary_fd < 0) {
    return -1;
  }
  FILE *output = fdopen(temporary_fd, "wb");
  if (output == NULL) {
    close(temporary_fd);
    unlink(temporary_name);
    return -1;
  }

  int result = encode_jpeg(output, image);
  if (result == 0 && fflush(output) != 0) {
    result = -1;
  }
  if (result == 0 && fsync(temporary_fd) != 0) {
    result = -1;
  }
  if (result == 0 && fchmod(temporary_fd, original_status.st_mode & 0777) != 0) {
    result = -1;
  }
  if (fclose(output) != 0) {
    result = -1;
  }
  if (result == 0 && rename(temporary_name, filename) != 0) {
    result = -1;
  }
  if (result != 0) {
    unlink(temporary_name);
    fprintf(stderr, "MOCK_TRANSFORM:\tfailed to replace %s: %s\n", filename, strerror(errno));
  }
  return result;
}