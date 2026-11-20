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
 * @file ear_heatmap.c
 * @brief Bounded, north-up acoustic evidence rendering and JPEG georeference sidecars.
 * @details Deposit measured linear power into a height-dependent Gaussian grid,
 * normalize by deposited weight, then display the loud end of the dB range.
 * Smoothing is a visualization heuristic, not a measured microphone beam model.
 * The decorated image explains the track; the undecorated field supports overlays.
 */
#include "ear_heatmap.h"
 
#include <ctype.h>
#include <errno.h>
#include <float.h>
#include <math.h>
#include <pthread.h>
#include <setjmp.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
 
#include <jpeglib.h>
 
#define HEATMAP_MIN_CELL_M 1.0
#define HEATMAP_MAX_CELLS 96U
#define HEATMAP_MAX_OUTPUT_PX 768U
#define HEATMAP_QUALITY 90
#define HEATMAP_MIN_SAMPLES 3U
#define EARTH_RADIUS_M 6378137.0
#define DEG_TO_RAD 0.017453292519943295
 
/** Policy values below retain existing output geometry, colors and byte values. */
#define HEATMAP_RGB_CHANNELS 3
#define HEATMAP_RGB_CHANNELS_U 3U
#define HEATMAP_RGB_MAX 255
#define HEATMAP_RGB_MAX_F 255.0
#define HEATMAP_PATH_CAPACITY 512
#define HEATMAP_SUFFIX_LENGTH 4
#define HEATMAP_SUFFIX_STORAGE 5
#define HEATMAP_BOUND_INITIALIZER 1e9
#define HEATMAP_POSE_LATITUDE_EPSILON_DEG 1e-6
#define HEATMAP_MIN_USABLE_AGL_M 0.5
#define HEATMAP_FALLBACK_AGL_M 10.0
#define HEATMAP_CELLS_PER_AGL 4.0
#define HEATMAP_FOOTPRINT_SIGMA_DIVISOR 2.0
#define HEATMAP_MIN_HALF_EXTENT_CELLS 3.0
#define HEATMAP_GAUSSIAN_REACH_SIGMAS 3.0
#define HEATMAP_MIN_DEPOSIT_WEIGHT 1e-3
#define HEATMAP_MIN_INTERPOLATION_WEIGHT 1e-6
#define HEATMAP_DB_LOG_BASE 10.0
#define HEATMAP_POWER_DB_PER_DECADE 10.0
#define HEATMAP_FLAT_RANGE_DB 1.0
#define HEATMAP_DISPLAY_RANGE_DB 20.0
/** Piecewise palette: dark blue -> cyan -> yellow -> red; channels are normalized. */
#define HEATMAP_PALETTE_FIRST_END 0.33
#define HEATMAP_PALETTE_SECOND_END 0.66
#define HEATMAP_PALETTE_FIRST_WIDTH 0.33
#define HEATMAP_PALETTE_SECOND_WIDTH 0.33
#define HEATMAP_PALETTE_LAST_WIDTH 0.34
#define HEATMAP_PALETTE_CHANNEL_LOW 0.05
#define HEATMAP_PALETTE_CHANNEL_HIGH 0.95
#define HEATMAP_PALETTE_BLUE_START 0.35
#define HEATMAP_PALETTE_GREEN_START 0.10
#define HEATMAP_PALETTE_GREEN_FIRST_RISE 0.70
#define HEATMAP_PALETTE_BLUE_FIRST_RISE 0.55
#define HEATMAP_PALETTE_RED_SECOND_RISE 0.90
#define HEATMAP_PALETTE_GREEN_MIDDLE 0.80
#define HEATMAP_PALETTE_GREEN_SECOND_RISE 0.15
#define HEATMAP_PALETTE_BLUE_MIDDLE 0.90
#define HEATMAP_PALETTE_BLUE_SECOND_FALL 0.85
#define HEATMAP_PALETTE_GREEN_LAST_FALL 0.80
/** Marker and bar dimensions are output pixels, not localization error bounds. */
#define HEATMAP_RING_STROKE_PX 2.0
#define HEATMAP_DOT_REACH_PX 2.5
#define HEATMAP_DOT_OUTER_RADIUS_PX 2.0
#define HEATMAP_SPOT_GREEN 40
#define HEATMAP_SPOT_RING_RADIUS_PX 9.0
#define HEATMAP_SPOT_ARM_LENGTH_PX 22.0
#define HEATMAP_SPOT_ARM_GAP_PX 4.0
#define HEATMAP_SPOT_OUTLINE_RADIUS_PX 2.4
#define HEATMAP_SPOT_ARM_RADIUS_PX 1.2
#define HEATMAP_SPOT_CENTER_RADIUS_PX 1.5
#define HEATMAP_SCALE_LENGTH_M 50.0
#define HEATMAP_SCALE_MAX_WIDTH_FRACTION 0.6
#define HEATMAP_SCALE_LEFT_FRACTION 0.04
#define HEATMAP_SCALE_BOTTOM_FRACTION 0.06
#define HEATMAP_SCALE_OUTLINE_RADIUS_PX 2.5
#define HEATMAP_SCALE_STROKE_RADIUS_PX 1.2
#define HEATMAP_SCALE_TICK_HALF_HEIGHT_PX 6
 
/* Serialize the shared grids and RGB workspace through encoding. A second
 * caller must not overwrite pixels while libjpeg still reads the first image. */
static pthread_mutex_t heatmap_mutex = PTHREAD_MUTEX_INITIALIZER;
static double heatmap_field[HEATMAP_MAX_CELLS * HEATMAP_MAX_CELLS];
static double heatmap_weight[HEATMAP_MAX_CELLS * HEATMAP_MAX_CELLS];
static uint8_t heatmap_pixels[HEATMAP_MAX_OUTPUT_PX * HEATMAP_MAX_OUTPUT_PX * HEATMAP_RGB_CHANNELS_U];
 
/**
 * @brief Per-encode libjpeg callback state and recovery point.
 * @details Kept on the active stack frame so one failed JPEG cannot poison a
 * subsequent render or escape into the host service.
 */
struct heatmap_jpeg_error {
 struct jpeg_error_mgr manager;
  jmp_buf recovery;
};
 
/**
 * @brief Turn a fatal libjpeg callback into a local encoder failure.
 * @param info Codec state whose error manager stores the recovery target.
 * @details Default libjpeg handling terminates the process; USHER must instead
 * report a failed visualization and remain available for camera/UART work.
 */
static void heatmap_jpeg_error_exit(j_common_ptr info)
{
 struct heatmap_jpeg_error *error = (struct heatmap_jpeg_error *)info->err;
  longjmp(error->recovery, 1);
}
 
/**
 * @brief Map a normalized relative level to the preserved three-color ramp.
 * @param t Display position; out-of-range values are clamped to [0, 1].
 * @param rgb Receives red, green and blue 8-bit channels.
 * @details The palette represents relative levels within this image; it is not
 * a calibrated sound-pressure scale and its breakpoints remain fixed for compatibility.
 */
static void colormap(double t, uint8_t *rgb)
{
 if (t < 0.0) { t = 0.0; }
 if (t > 1.0) { t = 1.0; }
 double r, g, b;
 if (t < HEATMAP_PALETTE_FIRST_END) {
 double u = t / HEATMAP_PALETTE_FIRST_WIDTH;
    r = HEATMAP_PALETTE_CHANNEL_LOW;
    g = HEATMAP_PALETTE_GREEN_START + HEATMAP_PALETTE_GREEN_FIRST_RISE * u;
    b = HEATMAP_PALETTE_BLUE_START + HEATMAP_PALETTE_BLUE_FIRST_RISE * u;
  } else if (t < HEATMAP_PALETTE_SECOND_END) {
 double u = (t - HEATMAP_PALETTE_FIRST_END) / HEATMAP_PALETTE_SECOND_WIDTH;
    r = HEATMAP_PALETTE_CHANNEL_LOW + HEATMAP_PALETTE_RED_SECOND_RISE * u;
    g = HEATMAP_PALETTE_GREEN_MIDDLE + HEATMAP_PALETTE_GREEN_SECOND_RISE * u;
    b = HEATMAP_PALETTE_BLUE_MIDDLE - HEATMAP_PALETTE_BLUE_SECOND_FALL * u;
  } else {
 double u = (t - HEATMAP_PALETTE_SECOND_END) / HEATMAP_PALETTE_LAST_WIDTH;
    r = HEATMAP_PALETTE_CHANNEL_HIGH;
    g = HEATMAP_PALETTE_CHANNEL_HIGH - HEATMAP_PALETTE_GREEN_LAST_FALL * u;
    b = HEATMAP_PALETTE_CHANNEL_LOW;
  }
  rgb[0] = (uint8_t)(r * HEATMAP_RGB_MAX_F + 0.5);
  rgb[1] = (uint8_t)(g * HEATMAP_RGB_MAX_F + 0.5);
  rgb[2] = (uint8_t)(b * HEATMAP_RGB_MAX_F + 0.5);
}
 
/**
 * @brief Rasterize a clipped filled disc or annular stroke.
 * @param pixels Interleaved RGB destination.
 * @param width Destination width in pixels.
 * @param height Destination height in pixels.
 * @param cx Marker center X, including fractional-pixel position.
 * @param cy Marker center Y, including fractional-pixel position.
 * @param radius Outer marker radius.
 * @param rgb Opaque marker color.
 * @param ring Draw a ring instead of a filled disc when true.
 * @details Pixel-center distances keep fractional marker placement symmetric;
 * explicit clipping prevents edge markers from indexing outside the fixed raster.
 */
static void draw_disc(uint8_t *pixels, unsigned int width, unsigned int height,
 double cx, double cy, double radius, const uint8_t *rgb, bool ring)
{
 if (!isfinite(cx) || !isfinite(cy) || !isfinite(radius) || radius < 0 ||
   cx < -radius - 1 || cy < -radius - 1 || cx > width + radius + 1 || cy > height + radius + 1) return;
 int x0 = (int)floor(cx - radius - 1.0);
 int x1 = (int)ceil(cx + radius + 1.0);
 int y0 = (int)floor(cy - radius - 1.0);
 int y1 = (int)ceil(cy + radius + 1.0);
 for (int y = y0; y <= y1; y++) {
 if (y < 0 || (unsigned int)y >= height) { continue; }
 for (int x = x0; x <= x1; x++) {
 if (x < 0 || (unsigned int)x >= width) { continue; }
 double d = hypot(x + 0.5 - cx, y + 0.5 - cy);
 bool inside = ring ? (d <= radius && d >= radius - HEATMAP_RING_STROKE_PX) : d <= radius;
 if (inside) {
        uint8_t *p = &pixels[((size_t)y * width + x) * HEATMAP_RGB_CHANNELS];
        p[0] = rgb[0]; p[1] = rgb[1]; p[2] = rgb[2];
      }
    }
  }
}
 
/**
 * @brief Clamp a normalized coverage/intensity value to its legal blend interval.
 * @param v Candidate weight or palette position.
 * @return `v` constrained to [0, 1].
 */
static double clamp01(double v)
{
 return v < 0.0 ? 0.0 : (v > 1.0 ? 1.0 : v);
}
 
/**
 * @brief Draw one observation as a white subpixel core with a dark outline.
 * @param pixels Interleaved RGB destination.
 * @param width Destination width in pixels.
 * @param height Destination height in pixels.
 * @param cx Observation center X in fractional output pixels.
 * @param cy Observation center Y in fractional output pixels.
 * @details Fractional coverage preserves the measured pose's subpixel placement;
 * the outline separates a location mark from the relative heat palette.
 */
static void draw_dot(uint8_t *pixels, unsigned int width, unsigned int height, double cx, double cy)
{
 if (!isfinite(cx) || !isfinite(cy) || cx < -HEATMAP_DOT_REACH_PX || cy < -HEATMAP_DOT_REACH_PX ||
   cx > width + HEATMAP_DOT_REACH_PX || cy > height + HEATMAP_DOT_REACH_PX) return;
 int x0 = (int)floor(cx - HEATMAP_DOT_REACH_PX), x1 = (int)ceil(cx + HEATMAP_DOT_REACH_PX);
 int y0 = (int)floor(cy - HEATMAP_DOT_REACH_PX), y1 = (int)ceil(cy + HEATMAP_DOT_REACH_PX);
 for (int y = y0; y <= y1; y++) {
 if (y < 0 || (unsigned int)y >= height) { continue; }
 for (int x = x0; x <= x1; x++) {
 if (x < 0 || (unsigned int)x >= width) { continue; }
 double d = hypot(x + 0.5 - cx, y + 0.5 - cy);
 double white = clamp01(1.0 - d);          // core: radius 0.5 px, soft edge
 double black = clamp01(HEATMAP_DOT_OUTER_RADIUS_PX - d) - white;  // ring: 1 px wide around the core
 if (white + black <= 0.0) { continue; }
      uint8_t *p = &pixels[((size_t)y * width + x) * HEATMAP_RGB_CHANNELS];
 for (int c = 0; c < HEATMAP_RGB_CHANNELS; c++) {
 double v = p[c] * (1.0 - white - black) + HEATMAP_RGB_MAX_F * white;
        p[c] = (uint8_t)(v + 0.5);
      }
    }
  }
}
 
/**
 * @brief Write the georeference sidecar for a successful north-up JPEG.
 * @param filename JPEG path whose `.jpg` suffix becomes `.geo`.
 * @param width Encoded output width in pixels.
 * @param height Encoded output height in pixels.
 * @param lat_c Local projection center latitude, degrees.
 * @param lon_c Local projection center longitude, degrees.
 * @param cos_lat Longitude-to-metre scale at the center latitude.
 * @param half_e East half-extent of the raster, metres.
 * @param half_n North half-extent of the raster, metres.
 * @param m_per_px Local metres per encoded output pixel.
 * @param spot Optional solved spot metadata; null/invalid estimates add no spot keys.
 * @details The sidecar uses the same local equirectangular frame as rasterization.
 * Its write is best-effort so a valid JPEG is retained if metadata storage fails;
 * map consumers requiring both artifacts validate the sidecar separately.
 */
static void write_georef(const char *filename, unsigned int width, unsigned int height,
 double lat_c, double lon_c, double cos_lat, double half_e, double half_n,
 double m_per_px, const struct ear_loudest_spot *spot)
{
 char path[HEATMAP_PATH_CAPACITY];
 size_t n = strlen(filename);
 if (n + HEATMAP_SUFFIX_STORAGE > sizeof(path)) {
 return;
  }
 size_t stem = (n > HEATMAP_SUFFIX_LENGTH && strcmp(filename + n - HEATMAP_SUFFIX_LENGTH, ".jpg") == 0)
                ? n - HEATMAP_SUFFIX_LENGTH : n;
 for (size_t i = 0; i < stem; i++) {
    path[i] = filename[i];
  }
  path[stem] = '\0';
  strcat(path, ".geo");
  FILE *f = fopen(path, "w");
 if (f == NULL) {
 return;
  }
 double m_per_deg_lat = DEG_TO_RAD * EARTH_RADIUS_M;
 double m_per_deg_lon = m_per_deg_lat * cos_lat;
  fprintf(f, "# ear heatmap georeference: equirectangular about the centre, north up\n");
  fprintf(f, "width_px=%u\nheight_px=%u\nm_per_px=%.6f\n", width, height, m_per_px);
  fprintf(f, "lat_center=%.8f\nlon_center=%.8f\n", lat_c, lon_c);
  fprintf(f, "lat_north=%.8f\nlon_west=%.8f\n", lat_c + half_n / m_per_deg_lat, lon_c - half_e / m_per_deg_lon);
  fprintf(f, "lat_south=%.8f\nlon_east=%.8f\n", lat_c + (half_n - height * m_per_px) / m_per_deg_lat,
          lon_c + (width * m_per_px - half_e) / m_per_deg_lon);
 if (spot != NULL && spot->valid) {
    fprintf(f, "spot_lat=%.8f\nspot_lon=%.8f\nspot_level_db=%.2f\nspot_confidence=%.3f\n",
            spot->lat_deg, spot->lon_deg, spot->level_db, spot->confidence);
  }
  fclose(f);
}
 
/**
 * @brief Encode caller-owned RGB pixels as a JPEG.
 * @param filename Output path.
 * @param pixels Borrowed row-major interleaved RGB raster.
 * @param width Raster width in pixels.
 * @param height Raster height in pixels.
 * @return 0 after encoding and close succeed; -1 for open/codec/close failures.
 * @details Scanlines are passed directly to libjpeg, so the bounded renderer
 * does not need another image-sized copy.
 */
static int encode_jpeg(const char *filename, const uint8_t *pixels,
 unsigned int width, unsigned int height)
{
  FILE *output = fopen(filename, "wb");
 if (output == NULL) {
 return -1;
  }
 struct jpeg_compress_struct encoder;
 struct heatmap_jpeg_error error;
 encoder.err = jpeg_std_error(&error.manager);
 error.manager.error_exit = heatmap_jpeg_error_exit;
 if (setjmp(error.recovery) != 0) {
    jpeg_destroy_compress(&encoder);
    fclose(output);
 return -1;
  }
  jpeg_create_compress(&encoder);
  jpeg_stdio_dest(&encoder, output);
 encoder.image_width = width;
 encoder.image_height = height;
 encoder.input_components = HEATMAP_RGB_CHANNELS;
 encoder.in_color_space = JCS_RGB;
  jpeg_set_defaults(&encoder);
  jpeg_set_quality(&encoder, HEATMAP_QUALITY, TRUE);
  jpeg_start_compress(&encoder, TRUE);
 while (encoder.next_scanline < encoder.image_height) {
    JSAMPROW row[1] = {(JSAMPROW)&pixels[(size_t)encoder.next_scanline * width * HEATMAP_RGB_CHANNELS]};
    jpeg_write_scanlines(&encoder, row, 1);
  }
  jpeg_finish_compress(&encoder);
  jpeg_destroy_compress(&encoder);
 return fclose(output) == 0 ? 0 : -1;
}
 
/**
 * @brief Render the shared measured field with optional human-facing markers.
 * @param filename Destination JPEG.
 * @param samples Borrowed, immutable geotagged EAR windows.
 * @param count Supplied window count.
 * @param spot Optional fused loudest-spot estimate.
 * @param decorate Draw observation dots, estimate crosshair and scale bar when true.
 * @return 0 on successful JPEG encode; -1 for insufficient usable samples or I/O failure.
 * @details One mutex protects the shared grids and pixels until libjpeg finishes reading
 * them. The AGL-derived grid is bounded; bilinear upsampling smooths display pixels but
 * does not increase the microphone's spatial resolution.
 */
static int heatmap_write(const char *filename, const struct ear_sample *samples, size_t count,
 const struct ear_loudest_spot *spot, bool decorate)
{
 if (filename == NULL || samples == NULL || count < HEATMAP_MIN_SAMPLES || count > EAR_MAX_SAMPLES) {
 return -1;
  }
 
 // Bounding box and typical AGL over the usable samples.
 double lat_min = HEATMAP_BOUND_INITIALIZER, lat_max = -HEATMAP_BOUND_INITIALIZER,
      lon_min = HEATMAP_BOUND_INITIALIZER, lon_max = -HEATMAP_BOUND_INITIALIZER;
 double agl_sum = 0.0;
 size_t usable = 0;
 for (size_t index = 0; index < count; index++) {
 const struct ear_sample *s = &samples[index];
 if ((isfinite(s->lat_deg) && fabs(s->lat_deg) > 90) ||
   (isfinite(s->lon_deg) && fabs(s->lon_deg) > 180) ||
   (isfinite(s->agl_m) && s->agl_m > INT32_MAX)) return -1;
 if (s->clipped || !isfinite(s->lat_deg) || !isfinite(s->lon_deg)
        || !isfinite(s->level_db) || fabs(s->lat_deg) <= HEATMAP_POSE_LATITUDE_EPSILON_DEG) {
 continue;
    }
 const double power = pow(HEATMAP_DB_LOG_BASE, s->level_db / HEATMAP_POWER_DB_PER_DECADE);
 if (!isfinite(power) || power < DBL_MIN || power > DBL_MAX / EAR_MAX_SAMPLES) return -1;
 if (s->lat_deg < lat_min) { lat_min = s->lat_deg; }
 if (s->lat_deg > lat_max) { lat_max = s->lat_deg; }
 if (s->lon_deg < lon_min) { lon_min = s->lon_deg; }
 if (s->lon_deg > lon_max) { lon_max = s->lon_deg; }
    agl_sum += (isfinite(s->agl_m) && s->agl_m > HEATMAP_MIN_USABLE_AGL_M)
           ? s->agl_m : HEATMAP_FALLBACK_AGL_M;
    usable++;
  }
 if (usable < HEATMAP_MIN_SAMPLES) {
 return -1;
  }
 double agl = agl_sum / (double)usable;
 double cell_m = fmax(HEATMAP_MIN_CELL_M, agl / HEATMAP_CELLS_PER_AGL);
 double sigma_m = fmax(cell_m, agl / HEATMAP_FOOTPRINT_SIGMA_DIVISOR);
 
 // Local ENU about the box centre, with one footprint of margin.
 double lat_c = 0.5 * (lat_min + lat_max);
 double lon_c = 0.5 * (lon_min + lon_max);
 double cos_lat = cos(lat_c * DEG_TO_RAD);
 if (!isfinite(cos_lat) || cos_lat <= 0) return -1;
 double half_e = (lon_max - lon_min) * 0.5 * DEG_TO_RAD * EARTH_RADIUS_M * cos_lat + agl;
 double half_n = (lat_max - lat_min) * 0.5 * DEG_TO_RAD * EARTH_RADIUS_M + agl;
 if (half_e < HEATMAP_MIN_HALF_EXTENT_CELLS * cell_m) { half_e = HEATMAP_MIN_HALF_EXTENT_CELLS * cell_m; }
 if (half_n < HEATMAP_MIN_HALF_EXTENT_CELLS * cell_m) { half_n = HEATMAP_MIN_HALF_EXTENT_CELLS * cell_m; }
 double grid_x = ceil(2.0 * half_e / cell_m), grid_y = ceil(2.0 * half_n / cell_m);
 if (!isfinite(grid_x) || !isfinite(grid_y) || grid_x < 1 || grid_y < 1) return -1;
 if (grid_x > HEATMAP_MAX_CELLS || grid_y > HEATMAP_MAX_CELLS) {
 double scale = fmax(grid_x, grid_y) / HEATMAP_MAX_CELLS;
    cell_m *= scale;
   grid_x = ceil(2.0 * half_e / cell_m);
   grid_y = ceil(2.0 * half_n / cell_m);
  }
 if (!isfinite(grid_x) || !isfinite(grid_y) || grid_x < 1 || grid_y < 1 ||
    grid_x > HEATMAP_MAX_CELLS || grid_y > HEATMAP_MAX_CELLS) return -1;
 const unsigned int cells_x = (unsigned int)grid_x, cells_y = (unsigned int)grid_y;
 
  pthread_mutex_lock(&heatmap_mutex);
 double *field = heatmap_field;
 double *weight = heatmap_weight;
  memset(field, 0, sizeof(heatmap_field));
  memset(weight, 0, sizeof(heatmap_weight));
 
 // Gaussian footprint deposit of each sample's level (in linear power).
 double level_min = HEATMAP_BOUND_INITIALIZER, level_max = -HEATMAP_BOUND_INITIALIZER;
 double inv_two_sigma2 = 1.0 / (2.0 * sigma_m * sigma_m);
 int reach = (int)ceil(HEATMAP_GAUSSIAN_REACH_SIGMAS * sigma_m / cell_m);
 for (size_t index = 0; index < count; index++) {
 const struct ear_sample *s = &samples[index];
 if (s->clipped || !isfinite(s->lat_deg) || !isfinite(s->lon_deg)
        || !isfinite(s->level_db) || fabs(s->lat_deg) <= HEATMAP_POSE_LATITUDE_EPSILON_DEG) {
 continue;
    }
 double e = (s->lon_deg - lon_c) * DEG_TO_RAD * EARTH_RADIUS_M * cos_lat;
 double n = (s->lat_deg - lat_c) * DEG_TO_RAD * EARTH_RADIUS_M;
 double gx = (e + half_e) / cell_m;
 double gy = (half_n - n) / cell_m;   // north up
 double power = pow(HEATMAP_DB_LOG_BASE, s->level_db / HEATMAP_POWER_DB_PER_DECADE);
 int cx0 = (int)floor(gx), cy0 = (int)floor(gy);
 for (int y = cy0 - reach; y <= cy0 + reach; y++) {
 if (y < 0 || (unsigned int)y >= cells_y) { continue; }
 for (int x = cx0 - reach; x <= cx0 + reach; x++) {
 if (x < 0 || (unsigned int)x >= cells_x) { continue; }
 double dx = (x + 0.5 - gx) * cell_m;
 double dy = (y + 0.5 - gy) * cell_m;
 double w = exp(-(dx * dx + dy * dy) * inv_two_sigma2);
 size_t k = (size_t)y * cells_x + x;
        field[k] += w * power;
        weight[k] += w;
      }
    }
  }
 for (size_t k = 0; k < (size_t)cells_x * cells_y; k++) {
 if (weight[k] > HEATMAP_MIN_DEPOSIT_WEIGHT) {
      field[k] = HEATMAP_POWER_DB_PER_DECADE * log10(field[k] / weight[k]);
 if (field[k] < level_min) { level_min = field[k]; }
 if (field[k] > level_max) { level_max = field[k]; }
    } else {
      field[k] = NAN;
    }
  }
 if (!(level_max > level_min)) {
    level_max = level_min + HEATMAP_FLAT_RANGE_DB;
  }
 // Emphasize the top of the dynamic range; the quiet floor is not informative.
 double span = fmin(level_max - level_min, HEATMAP_DISPLAY_RANGE_DB);
 double floor_db = level_max - span;
 
 // Output image: upsample cells bilinearly so the viewer sees smooth blobs.
 // The grid (not the pixel count) carries the microphone's resolution limit.
 unsigned int cells_max = cells_x > cells_y ? cells_x : cells_y;
 double scale = (double)HEATMAP_MAX_OUTPUT_PX / (double)cells_max;
 unsigned int width = (unsigned int)ceil(cells_x * scale);
 unsigned int height = (unsigned int)ceil(cells_y * scale);
 if (width < 1U) { width = 1U; }
 if (height < 1U) { height = 1U; }
 // The long side is HEATMAP_MAX_OUTPUT_PX by construction; bound both anyway.
 if (width > HEATMAP_MAX_OUTPUT_PX) { width = HEATMAP_MAX_OUTPUT_PX; }
 if (height > HEATMAP_MAX_OUTPUT_PX) { height = HEATMAP_MAX_OUTPUT_PX; }
  uint8_t *pixels = heatmap_pixels;
 for (unsigned int y = 0; y < height; y++) {
 double fy = (y + 0.5) / scale - 0.5;
 int y0 = (int)floor(fy); double ty = fy - y0;
 for (unsigned int x = 0; x < width; x++) {
 double fx = (x + 0.5) / scale - 0.5;
 int x0 = (int)floor(fx); double tx = fx - x0;
 double sum = 0.0, wsum = 0.0;
 for (int j = 0; j < 2; j++) {
 int yy = y0 + j;
 if (yy < 0 || (unsigned int)yy >= cells_y) { continue; }
 for (int i = 0; i < 2; i++) {
 int xx = x0 + i;
 if (xx < 0 || (unsigned int)xx >= cells_x) { continue; }
 double v = field[(size_t)yy * cells_x + xx];
 if (isnan(v)) { continue; }
 double w = (i ? tx : 1.0 - tx) * (j ? ty : 1.0 - ty);
          sum += w * v;
          wsum += w;
        }
      }
      uint8_t *p = &pixels[((size_t)y * width + x) * HEATMAP_RGB_CHANNELS];
 if (wsum > HEATMAP_MIN_INTERPOLATION_WEIGHT) {
 colormap((sum / wsum - floor_db) / span, p);
      } else {
        p[0] = EAR_HEATMAP_NO_DATA_RED; p[1] = EAR_HEATMAP_NO_DATA_GREEN; p[2] = EAR_HEATMAP_NO_DATA_BLUE;
      }
    }
  }
 
 // Track dots and the fused loudest-spot marker.
 for (size_t index = 0; decorate && index < count; index++) {
 const struct ear_sample *s = &samples[index];
 if (!isfinite(s->lat_deg) || !isfinite(s->lon_deg) || fabs(s->lat_deg) <= HEATMAP_POSE_LATITUDE_EPSILON_DEG) {
 continue;
    }
 double e = (s->lon_deg - lon_c) * DEG_TO_RAD * EARTH_RADIUS_M * cos_lat;
 double n = (s->lat_deg - lat_c) * DEG_TO_RAD * EARTH_RADIUS_M;
 double px = (e + half_e) / cell_m * scale;
 double py = (half_n - n) / cell_m * scale;
 draw_dot(pixels, width, height, px, py);
  }
 if (decorate && spot != NULL && spot->valid) {
 // Loudest spot: magenta crosshair with a small ring, black casing.
 const uint8_t magenta[HEATMAP_RGB_CHANNELS] = {HEATMAP_RGB_MAX, HEATMAP_SPOT_GREEN, HEATMAP_RGB_MAX};
 const uint8_t black[HEATMAP_RGB_CHANNELS] = {0, 0, 0};
 double e = (spot->lon_deg - lon_c) * DEG_TO_RAD * EARTH_RADIUS_M * cos_lat;
 double n = (spot->lat_deg - lat_c) * DEG_TO_RAD * EARTH_RADIUS_M;
 double px = (e + half_e) / cell_m * scale;
 double py = (half_n - n) / cell_m * scale;
 const double ring = HEATMAP_SPOT_RING_RADIUS_PX,
           arm = HEATMAP_SPOT_ARM_LENGTH_PX, gap = HEATMAP_SPOT_ARM_GAP_PX;
 for (int pass = 0; pass < 2; pass++) {
 const uint8_t *rgb = pass == 0 ? black : magenta;
 double r = pass == 0 ? HEATMAP_SPOT_OUTLINE_RADIUS_PX : HEATMAP_SPOT_ARM_RADIUS_PX;
 for (int t = (int)gap; t <= (int)arm; t++) {
 draw_disc(pixels, width, height, px + t, py, r, rgb, false);
 draw_disc(pixels, width, height, px - t, py, r, rgb, false);
 draw_disc(pixels, width, height, px, py + t, r, rgb, false);
 draw_disc(pixels, width, height, px, py - t, r, rgb, false);
      }
    }
 draw_disc(pixels, width, height, px, py, ring + HEATMAP_RING_STROKE_PX, black, true);
 draw_disc(pixels, width, height, px, py, ring, magenta, true);
 draw_disc(pixels, width, height, px, py, HEATMAP_SPOT_CENTER_RADIUS_PX, magenta, false);
  }
 
 /* The nominal bar is capped to fit small maps. When capped it is shorter
   * than the nominal length; .geo m_per_px is the authoritative map scale. */
 if (decorate) {
 const uint8_t white[HEATMAP_RGB_CHANNELS] = {HEATMAP_RGB_MAX, HEATMAP_RGB_MAX, HEATMAP_RGB_MAX};
 const uint8_t black[HEATMAP_RGB_CHANNELS] = {0, 0, 0};
 double bar_px = HEATMAP_SCALE_LENGTH_M / cell_m * scale;
 if (bar_px > HEATMAP_SCALE_MAX_WIDTH_FRACTION * width) { bar_px = HEATMAP_SCALE_MAX_WIDTH_FRACTION * width; }
 int bar_steps = (int)floor(bar_px);
 double x0 = HEATMAP_SCALE_LEFT_FRACTION * width;
 double y0 = height - HEATMAP_SCALE_BOTTOM_FRACTION * height;
 for (int t = 0; t <= bar_steps; t++) {
 draw_disc(pixels, width, height, x0 + t, y0, HEATMAP_SCALE_OUTLINE_RADIUS_PX, black, false);
    }
 for (int t = 0; t <= bar_steps; t++) {
 draw_disc(pixels, width, height, x0 + t, y0, HEATMAP_SCALE_STROKE_RADIUS_PX, white, false);
    }
 for (int dy = -HEATMAP_SCALE_TICK_HALF_HEIGHT_PX; dy <= HEATMAP_SCALE_TICK_HALF_HEIGHT_PX; dy++) {
 draw_disc(pixels, width, height, x0, y0 + dy, HEATMAP_SCALE_STROKE_RADIUS_PX, white, false);
 draw_disc(pixels, width, height, x0 + bar_px, y0 + dy, HEATMAP_SCALE_STROKE_RADIUS_PX, white, false);
    }
  }
 
 int status = encode_jpeg(filename, pixels, width, height);
 if (status == 0) {
 write_georef(filename, width, height, lat_c, lon_c, cos_lat, half_e, half_n, cell_m / scale, spot);
  }
  pthread_mutex_unlock(&heatmap_mutex);
 return status;
}
 
int ear_heatmap_write(const char *filename, const struct ear_sample *samples, size_t count,
 const struct ear_loudest_spot *spot)
{
 return heatmap_write(filename, samples, count, spot, true);
}
 
int ear_heatmap_write_field(const char *filename, const struct ear_sample *samples, size_t count,
 const struct ear_loudest_spot *spot)
{
 return heatmap_write(filename, samples, count, spot, false);
}
 
/**
 * @brief Trim leading/trailing whitespace in a mutable sidecar token.
 * @param text NUL-terminated buffer; trailing whitespace is replaced by NUL.
 * @return Pointer to the first non-whitespace character in the same storage.
 * @details In-place trimming keeps the sidecar reader bounded and validates the
 * unconsumed suffix left by numeric conversion.
 */
static char *trim_whitespace(char *text)
{
 while (isspace((unsigned char)*text)) { text++; }
 char *end = text + strlen(text);
 while (end > text && isspace((unsigned char)end[-1])) { *--end = '\0'; }
 return text;
}
 
/**
 * @brief Parse one finite number from a bounded sidecar value token.
 * @param text Token after key/value splitting.
 * @param value Receives the value only if the complete trimmed token is valid.
 * @return false for empty, non-finite, range-error or partially consumed input.
 * @details Rejecting trailing text prevents plausible prefixes such as `48junk`
 * from silently changing the overlay's geographic placement.
 */
static bool parse_geo_double(const char *text, double *value)
{
  errno = 0;
 char *end = NULL;
 double parsed = strtod(text, &end);
 if (end == text || errno == ERANGE || !isfinite(parsed) || *trim_whitespace(end) != '\0') {
 return false;
  }
  *value = parsed;
 return true;
}
 
/**
 * @brief Parse a strictly positive image dimension from a sidecar field.
 * @param text Decimal token.
 * @param value Receives the dimension on complete conversion.
 * @return true when nonzero and representable as unsigned int; false otherwise.
 * @details Rejecting zero/overflow dimensions before indexing protects consumers
 * that use the sidecar to size geographic pixel transforms.
 */
static bool parse_geo_dimension(const char *text, unsigned int *value)
{
  errno = 0;
 char *end = NULL;
 unsigned long parsed = strtoul(text, &end, 10);
 if (end == text || errno == ERANGE || parsed == 0 || parsed > UINT32_MAX
      || *trim_whitespace(end) != '\0') {
 return false;
  }
  *value = (unsigned int)parsed;
 return true;
}
 
int ear_heatmap_read_georef(const char *filename, struct ear_heatmap_georef *georef)
{
 if (filename == NULL || georef == NULL) { return -1; }
  FILE *input = fopen(filename, "r");
 if (input == NULL) { return -1; }
 
 struct ear_heatmap_georef parsed = {0};
 unsigned int seen = 0;
 bool spot_lat_seen = false;
 bool spot_lon_seen = false;
 /* Bound each input line independently of the size of the sidecar. */
 char line[256];
 bool valid = true;
 while (fgets(line, sizeof(line), input) != NULL) {
 size_t length = strlen(line);
 if (length > 0 && line[length - 1] != '\n' && !feof(input)) {
 valid = false;
 break;
    }
 char *entry = trim_whitespace(line);
 if (*entry == '\0' || *entry == '#') { continue; }
 char *separator = strchr(entry, '=');
 if (separator == NULL) { valid = false; break; }
    *separator = '\0';
 char *key = trim_whitespace(entry);
 char *value = trim_whitespace(separator + 1);
 unsigned int bit = 0;
 bool field_valid = true;
 if (strcmp(key, "width_px") == 0) {
      bit = 1U << 0;
      field_valid = parse_geo_dimension(value, &parsed.width_px);
    } else if (strcmp(key, "height_px") == 0) {
      bit = 1U << 1;
      field_valid = parse_geo_dimension(value, &parsed.height_px);
    } else if (strcmp(key, "m_per_px") == 0) {
      bit = 1U << 2;
      field_valid = parse_geo_double(value, &parsed.m_per_px) && parsed.m_per_px > 0.0;
    } else if (strcmp(key, "lat_center") == 0) {
      bit = 1U << 3;
      field_valid = parse_geo_double(value, &parsed.lat_center);
    } else if (strcmp(key, "lon_center") == 0) {
      bit = 1U << 4;
      field_valid = parse_geo_double(value, &parsed.lon_center);
    } else if (strcmp(key, "lat_north") == 0) {
      bit = 1U << 5;
      field_valid = parse_geo_double(value, &parsed.lat_north);
    } else if (strcmp(key, "lon_west") == 0) {
      bit = 1U << 6;
      field_valid = parse_geo_double(value, &parsed.lon_west);
    } else if (strcmp(key, "spot_lat") == 0) {
      field_valid = !spot_lat_seen && parse_geo_double(value, &parsed.spot_lat);
 if (field_valid) { spot_lat_seen = true; }
    } else if (strcmp(key, "spot_lon") == 0) {
      field_valid = !spot_lon_seen && parse_geo_double(value, &parsed.spot_lon);
 if (field_valid) { spot_lon_seen = true; }
    } else if (strcmp(key, "spot_level_db") == 0 || strcmp(key, "spot_confidence") == 0) {
 double ignored = 0.0;
      field_valid = parse_geo_double(value, &ignored);
    }
 if (!field_valid || (bit != 0 && (seen & bit) != 0)) {
 valid = false;
 break;
    }
    seen |= bit;
  }
 if (ferror(input) != 0) { valid = false; }
 if (fclose(input) != 0) { valid = false; }
 const unsigned int required = (1U << 7) - 1U;
 if (!valid || seen != required || spot_lat_seen != spot_lon_seen
      || fabs(parsed.lat_center) > 90.0 || fabs(parsed.lat_north) > 90.0
      || fabs(parsed.lon_center) > 180.0 || fabs(parsed.lon_west) > 180.0
      || (spot_lat_seen && (fabs(parsed.spot_lat) > 90.0 || fabs(parsed.spot_lon) > 180.0))) {
 return -1;
  }
  parsed.has_spot = spot_lat_seen;
  *georef = parsed;
 return 0;
}
