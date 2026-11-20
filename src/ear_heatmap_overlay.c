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

#define _POSIX_C_SOURCE 200809L
 
/**
 * @file ear_heatmap_overlay.c
 * @brief Render EARcam acoustic fields over cached or downloaded map tiles.
 * @details Host-side review utility. It does not run in the MORA service or
 * change acoustic samples, source estimates or flight-control decisions.
 */
#include "version.h"
#include "modules/ear_cam/ear_heatmap.h"
 
#include <ctype.h>
#include <errno.h>
#include <math.h>
#include <setjmp.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <jpeglib.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>
 
/** Web Mercator XYZ tile geometry and local projection constants. */
#define TILE_SIZE 256U
#define PI 3.14159265358979323846
#define EARTH_RADIUS_M 6378137.0
#define DEG_TO_RAD 0.017453292519943295
/** Explicit I/O/image/CSV ceilings keep malformed inputs from growing memory unboundedly. */
#define MAX_PATH_LENGTH 4096U
#define MAX_TILE_COUNT 4096U
#define MAX_OUTPUT_PIXELS 16000000U
#define MAX_INPUT_PIXELS 1048576U
#define MAX_CSV_POINTS 200000U
#define MAX_CSV_LINE 8192U
/** Review defaults: visual framing only; none of these imply spatial accuracy. */
#define DEFAULT_MAX_ZOOM 20U
#define DEFAULT_MARGIN_M 15.0
#define DEFAULT_ALPHA 0.55
#define DEFAULT_TRACK_GAP_S 1.5
#define JPEG_QUALITY 92
 
/**
 * @struct rgb_image
 * @brief Non-owning view of a bounded interleaved RGB workspace.
 * @details Caller-provided capacity is checked before libjpeg writes scanlines;
 * keeping ownership outside this view makes scratch-buffer reuse explicit.
 */
struct rgb_image {
 unsigned int width; /**< Decoded or composed width in pixels. */
 unsigned int height; /**< Decoded or composed height in pixels. */
  uint8_t *pixels; /**< Borrowed storage; this descriptor never frees it. */
 size_t capacity_bytes; /**< Maximum writable size of @c pixels. */
};
 
/**
 * @struct point
 * @brief One timestamped geographic position for a replay annotation.
 */
struct point {
 double time_s; /**< Timestamp normalized to seconds for gap handling. */
 double lat; /**< Latitude in degrees. */
 double lon; /**< Longitude in degrees. */
};
 
/**
 * @struct point_list
 * @brief Fixed-capacity, non-owning view over parsed session/track points.
 */
struct point_list {
 struct point *items; /**< Caller-owned record array. */
 size_t count; /**< Number of initialized points. */
 size_t capacity; /**< Hard record limit; excess valid rows fail the read. */
};
 
/**
 * @struct obstacle
 * @brief One optional terrain-height cell supplied for visual context.
 */
struct obstacle {
 double east_m; /**< Projected east coordinate used to infer grid pitch. */
 double lat; /**< Cell-center latitude in degrees. */
 double lon; /**< Cell-center longitude in degrees. */
 double height_m; /**< Height above terrain in metres. */
};
 
/**
 * @struct obstacle_list
 * @brief Bounded view over LiDAR cells and their inferred regular spacing.
 */
struct obstacle_list {
 struct obstacle *items; /**< Caller-owned fixed cell array. */
 size_t count; /**< Initialized cells. */
 size_t capacity; /**< Maximum accepted grid cells. */
};
 
/** @brief Reused input, tile and final RGB workspaces; avoid per-image allocator churn. */
static uint8_t input_pixels[(size_t)MAX_INPUT_PIXELS * 3U];
static uint8_t tile_pixels[(size_t)TILE_SIZE * TILE_SIZE * 3U];
static uint8_t output_pixels[(size_t)MAX_OUTPUT_PIXELS * 3U];
static struct point session_points[MAX_CSV_POINTS];
static struct point track_points[MAX_CSV_POINTS];
static struct obstacle obstacle_cells[MAX_CSV_POINTS];
static double obstacle_eastings[MAX_CSV_POINTS];
 
/**
 * @struct options
 * @brief Parsed overlay policy; all strings are borrowed from the process argument vector.
 */
struct options {
 const char *input; /**< Replay JPEG used to locate matching field and geometry files. */
 const char *geo_path; /**< Explicit geometry sidecar, or NULL for stem-derived naming. */
 const char *log_path; /**< Optional EAR session log for observation dots. */
 const char *track_path; /**< Optional timestamped aircraft track. */
 const char *truth_text; /**< Optional known simulation position. */
 const char *search_text; /**< Optional search center and radius. */
 const char *obstacles_path; /**< Optional LiDAR grid with heights above terrain. */
 const char *output_path; /**< Output override, or NULL for the @c _sat.jpg default. */
 const char *source; /**< Selected satellite source and cache namespace. */
 const char *cache_path; /**< Paparazzi map-cache root. */
 unsigned int zoom; /**< Explicit XYZ zoom; zero requests automatic selection. */
 unsigned int max_zoom; /**< Upper bound for automatic zoom selection. */
 double alpha; /**< Opacity for valid heat-field pixels. */
 double margin_m; /**< Ground extent around the heatmap. */
 double track_gap_s; /**< Largest timestamp gap connected by a track segment. */
 bool quiet; /**< Suppress progress messages. */
 bool offline; /**< Forbid downloads when a cache tile is missing. */
};
 
struct jpeg_failure {
 struct jpeg_error_mgr manager;
  jmp_buf recovery;
};
 
/**
 * @brief Convert libjpeg's fatal callback into a jump to the active operation.
 * @param info Codec state carrying the recovery record in its error manager.
 * @details Default libjpeg behavior exits the process; keeping recovery local lets
 * corrupt cached tiles fail this overlay without terminating other host tools.
 */
static void jpeg_error_exit(j_common_ptr info)
{
 struct jpeg_failure *failure = (struct jpeg_failure *)info->err;
  longjmp(failure->recovery, 1);
}
 
/**
 * @brief Invalidate an RGB view before its shared workspace is reused.
 * @param image Descriptor to clear; its backing pixels are caller-owned static storage.
 */
static void image_free(struct rgb_image *image)
{
  image->pixels = NULL;
  image->width = 0;
  image->height = 0;
  image->capacity_bytes = 0;
}
 
/**
 * @brief Decode one JPEG into the caller's bounded RGB workspace.
 * @param path JPEG file path.
 * @param image Supplies pixel pointer/capacity and receives decoded dimensions.
 * @return 0 after complete decode, or -1 for I/O, codec, size or capacity failure.
 * @details The pixel cap is checked before scanlines are decoded; image dimensions
 * from a file header never trigger allocation or out-of-bounds writes.
 */
static int image_read_jpeg(const char *path, struct rgb_image *image)
{
  FILE *input = fopen(path, "rb");
 if (input == NULL) { return -1; }
 struct jpeg_decompress_struct decoder;
 struct jpeg_failure failure;
 decoder.err = jpeg_std_error(&failure.manager);
  failure.manager.error_exit = jpeg_error_exit;
 if (setjmp(failure.recovery) != 0) {
    jpeg_destroy_decompress(&decoder);
    fclose(input);
 image_free(image);
 return -1;
  }
  jpeg_create_decompress(&decoder);
  jpeg_stdio_src(&decoder, input);
  jpeg_read_header(&decoder, TRUE);
 decoder.out_color_space = JCS_RGB;
  jpeg_start_decompress(&decoder);
 if (decoder.output_components != 3 || decoder.output_width == 0 || decoder.output_height == 0
      || (uint64_t)decoder.output_width * decoder.output_height > MAX_INPUT_PIXELS) {
    jpeg_destroy_decompress(&decoder);
    fclose(input);
 return -1;
  }
 unsigned int width = decoder.output_width;
 unsigned int height = decoder.output_height;
 size_t bytes = (size_t)width * height * 3U;
 if (image->pixels == NULL || image->capacity_bytes < bytes) {
    jpeg_destroy_decompress(&decoder);
    fclose(input);
 return -1;
  }
  image->width = width;
  image->height = height;
 while (decoder.output_scanline < decoder.output_height) {
    JSAMPROW row = &image->pixels[(size_t)decoder.output_scanline * image->width * 3U];
    jpeg_read_scanlines(&decoder, &row, 1);
  }
  jpeg_finish_decompress(&decoder);
  jpeg_destroy_decompress(&decoder);
 return fclose(input) == 0 ? 0 : -1;
}
 
/**
 * @brief Encode a bounded RGB view to a JPEG file.
 * @param path Output JPEG path.
 * @param image Image dimensions and borrowed RGB pixels.
 * @return 0 on successful encode/close, or -1 on open, codec or close failure.
 * @details Scanlines are passed directly from the single output workspace so no
 * image-sized staging copy is required.
 */
static int image_write_jpeg(const char *path, const struct rgb_image *image)
{
  FILE *output = fopen(path, "wb");
 if (output == NULL) { return -1; }
 struct jpeg_compress_struct encoder;
 struct jpeg_failure failure;
 encoder.err = jpeg_std_error(&failure.manager);
  failure.manager.error_exit = jpeg_error_exit;
 if (setjmp(failure.recovery) != 0) {
    jpeg_destroy_compress(&encoder);
    fclose(output);
 return -1;
  }
  jpeg_create_compress(&encoder);
  jpeg_stdio_dest(&encoder, output);
 encoder.image_width = image->width;
 encoder.image_height = image->height;
 encoder.input_components = 3;
 encoder.in_color_space = JCS_RGB;
  jpeg_set_defaults(&encoder);
  jpeg_set_quality(&encoder, JPEG_QUALITY, TRUE);
  jpeg_start_compress(&encoder, TRUE);
 while (encoder.next_scanline < encoder.image_height) {
    JSAMPROW row = (JSAMPROW)&image->pixels[(size_t)encoder.next_scanline * image->width * 3U];
    jpeg_write_scanlines(&encoder, &row, 1);
  }
  jpeg_finish_compress(&encoder);
  jpeg_destroy_compress(&encoder);
 return fclose(output) == 0 ? 0 : -1;
}
 
/**
 * @brief Parse a finite floating-point token in full.
 * @param text NUL-terminated command-line value.
 * @param value Receives the result only on success.
 * @return false for empty, non-finite, out-of-range or partially numeric strings.
 * @details Rejecting trailing characters prevents a typo like `0.5junk` from
 * silently changing image opacity or another numeric option.
 */
static bool parse_double(const char *text, double *value)
{
  errno = 0;
 char *end = NULL;
 double parsed = strtod(text, &end);
 if (end == text || errno == ERANGE || !isfinite(parsed) || *end != '\0') { return false; }
  *value = parsed;
 return true;
}
 
/**
 * @brief Parse an unsigned integer option without accepting suffix characters.
 * @param text NUL-terminated decimal token.
 * @param value Receives the value only when it fits `unsigned int`.
 * @return true for a complete conversion within the destination range.
 */
static bool parse_unsigned(const char *text, unsigned int *value)
{
  errno = 0;
 char *end = NULL;
 unsigned long parsed = strtoul(text, &end, 10);
 if (end == text || errno == ERANGE || *end != '\0' || parsed > UINT32_MAX) { return false; }
  *value = (unsigned int)parsed;
 return true;
}
 
/**
 * @brief Print command synopsis and defaults from the same option vocabulary parsed below.
 * @param program Invocation name displayed to the user.
 */
static void print_help(const char *program)
{
  printf("%s\nUsage: %s HEATMAP.jpg [options]\n"
 "Overlay a USHER EARcam heatmap on cached/downloaded satellite tiles.\n\n"
 "  --geo FILE             Sidecar path (default: HEATMAP.geo)\n"
 "  -o, --output FILE      Output JPEG (default: HEATMAP_sat.jpg)\n"
 "  --source google|esri   Tile source (default: google)\n"
 "  --cache DIR            Shared tile cache (default: XDG_DATA_HOME/paparazzi/pprzgcs/map)\n"
 "  --zoom N               Tile zoom (otherwise choose from heatmap resolution)\n"
 "  --max-zoom N           Maximum automatic zoom (default: 20)\n"
 "  --alpha N              Heatmap opacity from 0 to 1 (default: 0.55)\n"
 "  --margin-m N           Imagery margin (default: 15)\n"
 "  --log FILE             Session CSV; redraw sample positions\n"
 "  --track FILE           Track CSV: t_s,lat_deg,lon_deg,...\n"
 "  --track-gap-s N        Break track after this gap (default: 1.5)\n"
 "  --search LAT,LON[,M]   Search point and radius (default radius: 25 m)\n"
 "  --truth LAT,LON        Known source position for simulation comparison\n"
 "  --obstacles FILE       LiDAR CSV with x_east_m,lat,lon,mnh_m columns\n"
 "  --offline              Use cached tiles only; fail if any tile is missing\n"
 "  --quiet                Suppress tile progress messages\n"
 "  -h, --help             Show this help\n"
 "  -V, --version          Show build version\n", EAR_HEATMAP_OVERLAY_BUILD_VERSION, program);
}
 
/**
 * @brief Parse one positional heatmap and the supported optional controls.
 * @param argc Argument count.
 * @param argv Process argument vector.
 * @param options Receives validated values; strings borrow `argv` storage.
 * @return true when exactly one positional input and all recognized values are present.
 * @details Options are accepted before or after the image to avoid shell-order
 * surprises; unknown switches and duplicate positional images are rejected.
 */
static bool parse_options(int argc, char **argv, struct options *options)
{
  memset(options, 0, sizeof(*options));
 options->source = "google";
 options->max_zoom = DEFAULT_MAX_ZOOM;
 options->alpha = DEFAULT_ALPHA;
 options->margin_m = DEFAULT_MARGIN_M;
 options->track_gap_s = DEFAULT_TRACK_GAP_S;
 for (int index = 1; index < argc; index++) {
 const char *arg = argv[index];
 if (arg[0] != '-') {
 if (options->input != NULL) { return false; }
 options->input = arg;
 continue;
    }
 if (strcmp(arg, "--help") == 0 || strcmp(arg, "-h") == 0) {
 print_help(argv[0]);
      exit(0);
    }
 if (strcmp(arg, "--version") == 0 || strcmp(arg, "-V") == 0) {
      puts(EAR_HEATMAP_OVERLAY_BUILD_VERSION);
      exit(0);
    }
 if (strcmp(arg, "--quiet") == 0) { options->quiet = true; continue; }
 if (strcmp(arg, "--offline") == 0) { options->offline = true; continue; }
 if (index + 1 >= argc) { return false; }
 const char *value = argv[++index];
 if (strcmp(arg, "--geo") == 0) { options->geo_path = value; }
 else if (strcmp(arg, "--log") == 0) { options->log_path = value; }
 else if (strcmp(arg, "--track") == 0) { options->track_path = value; }
 else if (strcmp(arg, "--truth") == 0) { options->truth_text = value; }
 else if (strcmp(arg, "--search") == 0) { options->search_text = value; }
 else if (strcmp(arg, "--obstacles") == 0) { options->obstacles_path = value; }
 else if (strcmp(arg, "--output") == 0 || strcmp(arg, "-o") == 0) { options->output_path = value; }
 else if (strcmp(arg, "--source") == 0) { options->source = value; }
 else if (strcmp(arg, "--cache") == 0) { options->cache_path = value; }
 else if (strcmp(arg, "--zoom") == 0) {
 if (!parse_unsigned(value, &options->zoom)) { return false; }
    } else if (strcmp(arg, "--max-zoom") == 0) {
 if (!parse_unsigned(value, &options->max_zoom)) { return false; }
    } else if (strcmp(arg, "--alpha") == 0) {
 if (!parse_double(value, &options->alpha)) { return false; }
    } else if (strcmp(arg, "--margin-m") == 0) {
 if (!parse_double(value, &options->margin_m)) { return false; }
    } else if (strcmp(arg, "--track-gap-s") == 0) {
 if (!parse_double(value, &options->track_gap_s)) { return false; }
    } else { return false; }
  }
 return options->input != NULL;
}
 
/**
 * @brief Project latitude/longitude into fractional Web Mercator tile coordinates.
 * @param lat Latitude in degrees.
 * @param lon Longitude in degrees.
 * @param zoom XYZ zoom level.
 * @param x Receives fractional tile X, increasing eastward.
 * @param y Receives fractional tile Y, increasing southward.
 * @details Clamp to the finite square-tile Mercator latitude limit; callers separately
 * reject overlays whose geographic bounds are not supported by this projection.
 */
static void mercator_xy(double lat, double lon, unsigned int zoom, double *x, double *y)
{
 const double n = ldexp(1.0, (int)zoom);
 double bounded_lat = fmax(-85.05112878, fmin(85.05112878, lat));
 double lat_rad = bounded_lat * DEG_TO_RAD;
  *x = (lon + 180.0) / 360.0 * n;
  *y = (1.0 - asinh(tan(lat_rad)) / PI) * 0.5 * n;
}
 
/**
 * @brief Estimate ground metres per tile pixel at a latitude.
 * @param lat Latitude in degrees.
 * @param zoom XYZ zoom level.
 * @return Local ground resolution in metres per 256-pixel tile pixel.
 * @details Mercator scale varies by latitude, so zoom selection and the scale
 * annotation consistently use the heatmap's center latitude.
 */
static double mercator_m_per_pixel(double lat, unsigned int zoom)
{
 return 2.0 * PI * EARTH_RADIUS_M * cos(lat * DEG_TO_RAD) / (TILE_SIZE * ldexp(1.0, (int)zoom));
}
 
/**
 * @brief Select the first basemap zoom at least as detailed as the heatmap.
 * @param geo Heatmap pixel scale and center latitude.
 * @param max_zoom Highest permitted map zoom.
 * @return Selected zoom, capped at `max_zoom` when the map remains coarser.
 * @details More detailed imagery cannot increase acoustic measurement resolution;
 * this default avoids needless tile downloads at higher-than-useful zoom.
 */
static unsigned int choose_zoom(const struct ear_heatmap_georef *geo, unsigned int max_zoom)
{
 for (unsigned int zoom = 12; zoom <= max_zoom; zoom++) {
 if (mercator_m_per_pixel(geo->lat_center, zoom) <= geo->m_per_px) { return zoom; }
  }
 return max_zoom;
}
 
/**
 * @brief Parse `LAT,LON` or `LAT,LON,RADIUS_M` into bounded geographic values.
 * @param text Comma-separated command-line token.
 * @param with_radius Allow/require interpretation of a third optional field.
 * @param lat Receives latitude in degrees.
 * @param lon Receives longitude in degrees.
 * @param radius Receives the parsed radius or the 25 m default when enabled.
 * @return true only when every field is consumed and within its geographic/radius range.
 */
static bool parse_location(const char *text, bool with_radius, double *lat, double *lon, double *radius)
{
 if (text == NULL) { return false; }
 char buffer[128];
 size_t length = strlen(text);
 if (length == 0 || length >= sizeof(buffer)) { return false; }
 for (size_t index = 0; index <= length; index++) { buffer[index] = text[index]; }
 char *first = strchr(buffer, ',');
 if (first == NULL) { return false; }
  *first++ = '\0';
 char *second = strchr(first, ',');
 if (second != NULL) { *second++ = '\0'; }
 if (!parse_double(buffer, lat) || !parse_double(first, lon)) { return false; }
 if (*lat < -90.0 || *lat > 90.0 || *lon < -180.0 || *lon > 180.0) { return false; }
 if (with_radius) {
    *radius = 25.0;
 if (second != NULL && (!parse_double(second, radius) || *radius <= 0.0 || *radius > 100000.0)) { return false; }
 return second == NULL || strchr(second, ',') == NULL;
  }
 return second == NULL;
}
 
/**
 * @brief Derive a sibling `.geo`, `_field.jpg` or `_sat.jpg` path from an image.
 * @param path Input JPEG path.
 * @param suffix Replacement suffix.
 * @param output Destination path buffer.
 * @param capacity Capacity in bytes.
 * @return 0 when the path fits, -1 rather than returning a truncated filename.
 * @details Stem-derived names keep replay artifacts paired without asking users
 * to type a second path that can silently refer to another session.
 */
static int make_stem_path(const char *path, const char *suffix, char *output, size_t capacity)
{
 size_t length = strlen(path);
 size_t stem = length;
 if (length >= 4 && (strcmp(path + length - 4, ".jpg") == 0 || strcmp(path + length - 4, ".JPG") == 0)) {
    stem = length - 4;
  } else if (length >= 5 && (strcmp(path + length - 5, ".jpeg") == 0 || strcmp(path + length - 5, ".JPEG") == 0)) {
    stem = length - 5;
  }
 if (stem + strlen(suffix) + 1 > capacity) { return -1; }
 for (size_t index = 0; index < stem; index++) { output[index] = path[index]; }
 output[stem] = '\0';
 return snprintf(output + stem, capacity - stem, "%s", suffix) < 0 ? -1 : 0;
}
 
/**
 * @brief Resolve the Paparazzi GCS map-cache root from XDG environment settings.
 * @param path Receives `$XDG_DATA_HOME/paparazzi/pprzgcs/map` or the HOME fallback.
 * @param capacity Destination capacity in bytes.
 * @return 0 for a usable environment/path, -1 if HOME is absent or the result cannot fit.
 * @details GCS imagery belongs to its user-data cache, not the source checkout;
 * using this shared location is what makes local tiles the fast/default path.
 */
static int default_cache_path(char *path, size_t capacity)
{
 const char *xdg_data_home = getenv("XDG_DATA_HOME");
 const char *home = getenv("HOME");
 if (xdg_data_home != NULL && *xdg_data_home != '\0') {
 if (snprintf(path, capacity, "%s/paparazzi/pprzgcs/map", xdg_data_home) >= (int)capacity) { return -1; }
  } else {
 if (home == NULL || *home == '\0') { return -1; }
 if (snprintf(path, capacity, "%s/.local/share/paparazzi/pprzgcs/map", home) >= (int)capacity) { return -1; }
  }
 return 0;
}
 
/**
 * @brief Create parent directories for a tile file without treating the file as a directory.
 * @param path Full destination filename.
 * @return 0 if all parents exist or are created, otherwise -1.
 * @details Downloads go to a sibling temporary path and are renamed on success;
 * partial transfers therefore cannot masquerade as cache hits.
 */
static int make_directories(const char *path)
{
 char buffer[MAX_PATH_LENGTH] = {0};
 if (!path) { return -1; }
 size_t length = strnlen(path, sizeof(buffer));
 if (length == 0 || length >= sizeof(buffer)) { return -1; }
 for (size_t index = 0; index <= length; index++) { buffer[index] = path[index]; }
 char *last_separator = strrchr(buffer, '/');
 if (last_separator == NULL) { return 0; }
  *last_separator = '\0';
  length = strlen(buffer);
 for (size_t index = 1; index < length; index++) {
 if (buffer[index] != '/') { continue; }
    buffer[index] = '\0';
 if (mkdir(buffer, 0775) != 0 && errno != EEXIST) { return -1; }
    buffer[index] = '/';
  }
 if (mkdir(buffer, 0775) != 0 && errno != EEXIST) { return -1; }
 return 0;
}
 
/**
 * @brief Encode an XYZ address using Paparazzi's legacy Google quadkey alphabet.
 * @param x Tile column.
 * @param y Tile row, increasing southward.
 * @param zoom Number of quadkey digits.
 * @param key Caller-owned string buffer.
 * @param capacity Buffer capacity including the terminator.
 * @details This is a compatibility fallback; current Paparazzi cache entries
 * are searched in the uppercase XYZ layout first.
 */
static void gcs_key(unsigned int x, unsigned int y, unsigned int zoom, char *key, size_t capacity)
{
 static const char quadrants[2][2] = {{'q', 't'}, {'r', 's'}};
 size_t position = 0;
  key[position++] = 't';
 for (int bit = (int)zoom - 1; bit >= 0 && position + 1 < capacity; bit--) {
    key[position++] = quadrants[(x >> bit) & 1U][(y >> bit) & 1U];
  }
  key[position] = '\0';
}
 
/**
 * @brief Select the first known local filename for an XYZ source tile.
 * @param options Source namespace and cache root.
 * @param x Tile column.
 * @param y Tile row.
 * @param zoom Tile zoom.
 * @param path Receives an existing tile path when found, otherwise preferred download path.
 * @param capacity Destination buffer size.
 * @return 0 when a candidate fits the buffer; -1 if path formatting would truncate.
 * @details Current `GOOGLE/{z}/{x}/{y}.jpeg` and Esri layouts are checked before
 * historical spellings so existing locally cached imagery is reused without network.
 */
static int tile_path(const struct options *options, unsigned int x, unsigned int y,
 unsigned int zoom, char *path, size_t capacity)
{
 if (strcmp(options->source, "google") == 0) {
 if (snprintf(path, capacity, "%s/GOOGLE/%u/%u/%u.jpeg", options->cache_path, zoom, x, y) >= (int)capacity) { return -1; }
 if (access(path, R_OK) == 0) { return 0; }
 if (snprintf(path, capacity, "%s/GOOGLE/%u/%u/%u.jpg", options->cache_path, zoom, x, y) >= (int)capacity) { return -1; }
 if (access(path, R_OK) == 0) { return 0; }
 char key[32];
 gcs_key(x, y, zoom, key, sizeof(key));
 if (snprintf(path, capacity, "%s/Google/%s.jpg", options->cache_path, key) >= (int)capacity) { return -1; }
 if (access(path, R_OK) == 0) { return 0; }
 return snprintf(path, capacity, "%s/Google/%s.jpeg", options->cache_path, key) >= (int)capacity ? -1 : 0;
  }
 if (snprintf(path, capacity, "%s/ESRI/%u/%u/%u.jpeg", options->cache_path, zoom, x, y) >= (int)capacity) { return -1; }
 if (access(path, R_OK) == 0) { return 0; }
 if (snprintf(path, capacity, "%s/esri/%u/%u_%u.jpg", options->cache_path, zoom, x, y) >= (int)capacity) { return -1; }
 if (access(path, R_OK) == 0) { return 0; }
 return snprintf(path, capacity, "%s/esri/%u/%u_%u.jpeg", options->cache_path, zoom, x, y) >= (int)capacity ? -1 : 0;
}
 
/**
 * @brief Download one cache miss with the existing curl executable.
 * @param options Source/cache policy.
 * @param x Tile column.
 * @param y Tile row.
 * @param zoom Tile zoom level.
 * @param path Final cache filename.
 * @return 0 after an atomic install, otherwise -1 on process, network or file failure.
 * @details Called only after every local cache spelling misses. `execvp` avoids
 * shell interpretation; the temporary file is renamed only after curl exits cleanly.
 */
static int download_tile(const struct options *options, unsigned int x, unsigned int y,
 unsigned int zoom, const char *path)
{
 if (make_directories(path) != 0) { return -1; }
 char temporary[MAX_PATH_LENGTH];
 if (snprintf(temporary, sizeof(temporary), "%s.part.%ld", path, (long)getpid()) >= (int)sizeof(temporary)) { return -1; }
 char url[512];
 if (strcmp(options->source, "google") == 0) {
    snprintf(url, sizeof(url), "https://mt1.google.com/vt/lyrs=s&x=%u&y=%u&z=%u", x, y, zoom);
  } else {
    snprintf(url, sizeof(url), "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/%u/%u/%u",
             zoom, y, x);
  }
 char *const arguments[] = {"curl", "--fail", "--location", "--silent", "--show-error",
 "--connect-timeout", "15", "--max-time", "30", "--retry", "2",
 "--retry-delay", "1", "--user-agent", "sensoraia-ear-heatmap-overlay/1",
 "--output", temporary, url, NULL};
  pid_t child = fork();
 if (child < 0) { return -1; }
 if (child == 0) {
    execvp(arguments[0], arguments);
    _exit(127);
  }
 int child_status = 0;
 while (waitpid(child, &child_status, 0) < 0) {
 if (errno != EINTR) { unlink(temporary); return -1; }
  }
 if (!WIFEXITED(child_status) || WEXITSTATUS(child_status) != 0
      || rename(temporary, path) != 0) {
    unlink(temporary);
 return -1;
  }
 return 0;
}
 
/**
 * @brief Read a local hit, fetch a miss if permitted, and decode one 256x256 tile.
 * @param options Source/cache/offline policy.
 * @param x Tile column.
 * @param y Tile row.
 * @param zoom Tile zoom level.
 * @param tile Receives RGB pixels in the fixed tile workspace.
 * @return 0 for a readable tile/placeholder; -1 for offline misses or invalid cached JPEGs.
 * @details Offline mode is deliberately strict: replacing missing terrain with blank
 * pixels would make incomplete geographic context look like a successful map review.
 */
static int fetch_tile(const struct options *options, unsigned int x, unsigned int y,
 unsigned int zoom, struct rgb_image *tile)
{
 char path[MAX_PATH_LENGTH];
 if (tile_path(options, x, y, zoom, path, sizeof(path)) != 0) { return -1; }
 if (access(path, R_OK) != 0 && options->offline) {
    fprintf(stderr, "offline mode: cached tile is missing: %s\n", path);
 return -1;
  }
 if (access(path, R_OK) != 0 && download_tile(options, x, y, zoom, path) != 0) {
 if (!options->quiet) { fprintf(stderr, "warning: tile z%u %u/%u unavailable; using dark placeholder\n", zoom, x, y); }
    tile->width = TILE_SIZE;
    tile->height = TILE_SIZE;
    tile->pixels = tile_pixels;
    tile->capacity_bytes = sizeof(tile_pixels);
 for (size_t index = 0; index < (size_t)TILE_SIZE * TILE_SIZE; index++) {
      tile->pixels[index * 3] = 40;
      tile->pixels[index * 3 + 1] = 40;
      tile->pixels[index * 3 + 2] = 40;
    }
 return 0;
  }
 if (image_read_jpeg(path, tile) != 0 || tile->width != TILE_SIZE || tile->height != TILE_SIZE) {
 image_free(tile);
 return -1;
  }
 return 0;
}
 
/**
 * @brief Copy only the part of a source tile intersecting the cropped output.
 * @param tile Decoded fixed-size map tile.
 * @param output Bounded destination mosaic.
 * @param tile_x Absolute XYZ column.
 * @param tile_y Absolute XYZ row.
 * @param crop_x Crop origin in the assembled tile-grid pixels.
 * @param crop_y Crop origin in the assembled tile-grid pixels.
 * @return 0 after a clipped copy; nonintersecting edges need no work.
 * @details Direct clipped copies avoid constructing a larger full-tile mosaic
 * that would immediately be cropped and discarded.
 */
static int copy_tile_to_output(const struct rgb_image *tile, struct rgb_image *output,
 int tile_x, int tile_y, int crop_x, int crop_y)
{
 int left = tile_x * (int)TILE_SIZE - crop_x;
 int top = tile_y * (int)TILE_SIZE - crop_y;
 int x0 = left < 0 ? 0 : left;
 int y0 = top < 0 ? 0 : top;
 int x1 = left + (int)TILE_SIZE > (int)output->width ? (int)output->width : left + (int)TILE_SIZE;
 int y1 = top + (int)TILE_SIZE > (int)output->height ? (int)output->height : top + (int)TILE_SIZE;
 if (x0 >= x1 || y0 >= y1) { return 0; }
 for (int y = y0; y < y1; y++) {
 size_t source_offset = ((size_t)(y - top) * TILE_SIZE + (unsigned int)(x0 - left)) * 3U;
 size_t target_offset = ((size_t)y * output->width + (unsigned int)x0) * 3U;
 size_t bytes = (size_t)(x1 - x0) * 3U;
 for (size_t index = 0; index < bytes; index++) { output->pixels[target_offset + index] = tile->pixels[source_offset + index]; }
  }
 return 0;
}
 
/**
 * @brief Invert fractional Web Mercator tile Y into a geographic latitude.
 * @param tile_y Fractional tile row, increasing southward.
 * @param zoom XYZ zoom level.
 * @return Latitude in degrees.
 */
static double mercator_latitude(double tile_y, unsigned int zoom)
{
 double n = ldexp(1.0, (int)zoom);
 return atan(sinh(PI * (1.0 - 2.0 * tile_y / n))) / DEG_TO_RAD;
}
 
/**
 * @brief Resample the local equirectangular heat field onto the Mercator tile mosaic.
 * @param base In/out satellite RGB image.
 * @param field Undecorated EAR heatmap raster.
 * @param geo Validated sidecar footprint and scale.
 * @param zoom Basemap XYZ zoom shared by tile placement and output projection.
 * @param crop_x Cropped image origin within the tile grid, pixels.
 * @param crop_y Cropped image origin within the tile grid, pixels.
 * @param alpha Opacity for covered heatmap pixels.
 * @details Every basemap pixel is transformed back into the heatmap's local frame.
 * This avoids stretching the field to fit a differently projected map and preserves
 * the renderer's no-data color as transparent coverage rather than false evidence.
 */
static void composite_heatmap(struct rgb_image *base, const struct rgb_image *field,
 const struct ear_heatmap_georef *geo, unsigned int zoom,
 int crop_x, int crop_y, double alpha)
{
 double m_per_deg_lat = DEG_TO_RAD * EARTH_RADIUS_M;
 double cos_lat = cos(geo->lat_center * DEG_TO_RAD);
 double m_per_deg_lon = m_per_deg_lat * cos_lat;
 double east0 = (geo->lon_west - geo->lon_center) * m_per_deg_lon;
 double north0 = (geo->lat_north - geo->lat_center) * m_per_deg_lat;
 double tile_count = ldexp(1.0, (int)zoom);
 for (unsigned int y = 0; y < base->height; y++) {
 double tile_y = ((double)(crop_y + (int)y) + 0.5) / TILE_SIZE;
 double lat = mercator_latitude(tile_y, zoom);
 double hy = (north0 - (lat - geo->lat_center) * m_per_deg_lat) / geo->m_per_px;
 if (hy < 0.0 || hy >= field->height) { continue; }
 unsigned int fy = (unsigned int)hy;
 for (unsigned int x = 0; x < base->width; x++) {
 double tile_x = ((double)(crop_x + (int)x) + 0.5) / TILE_SIZE;
 double lon = tile_x / tile_count * 360.0 - 180.0;
 double hx = ((lon - geo->lon_center) * m_per_deg_lon - east0) / geo->m_per_px;
 if (hx < 0.0 || hx >= field->width) { continue; }
 unsigned int fx = (unsigned int)hx;
 size_t source = ((size_t)fy * field->width + fx) * 3U;
 int dr = (int)field->pixels[source] - (int)EAR_HEATMAP_NO_DATA_RED;
 int dg = (int)field->pixels[source + 1] - (int)EAR_HEATMAP_NO_DATA_GREEN;
 int db = (int)field->pixels[source + 2] - (int)EAR_HEATMAP_NO_DATA_BLUE;
 if (dr * dr + dg * dg + db * db < 24 * 24) { continue; }
 size_t target = ((size_t)y * base->width + x) * 3U;
 for (unsigned int channel = 0; channel < 3U; channel++) {
 double value = base->pixels[target + channel] * (1.0 - alpha) + field->pixels[source + channel] * alpha;
        base->pixels[target + channel] = (uint8_t)(value + 0.5);
      }
    }
  }
}
 
/**
 * @brief Blend one annotation pixel while clipping out-of-image coordinates.
 * @param image Destination RGB image.
 * @param x Image-space X.
 * @param y Image-space Y.
 * @param red Annotation red channel.
 * @param green Annotation green channel.
 * @param blue Annotation blue channel.
 * @param alpha Blend amount from transparent to opaque.
 * @details Central clipping lets each simple raster primitive draw across map
 * boundaries without a separate bounds check for every caller.
 */
static void put_blended_pixel(struct rgb_image *image, int x, int y,
                              uint8_t red, uint8_t green, uint8_t blue, double alpha)
{
 if (x < 0 || y < 0 || (unsigned int)x >= image->width || (unsigned int)y >= image->height) { return; }
 size_t offset = ((size_t)y * image->width + (unsigned int)x) * 3U;
 const uint8_t color[3] = {red, green, blue};
 for (unsigned int channel = 0; channel < 3U; channel++) {
 double value = image->pixels[offset + channel] * (1.0 - alpha) + color[channel] * alpha;
    image->pixels[offset + channel] = (uint8_t)(value + 0.5);
  }
}
 
/**
 * @brief Stamp a square brush centered at an integer pixel.
 * @param image Destination RGB image.
 * @param x Center X.
 * @param y Center Y.
 * @param width Nominal stroke width.
 * @param red Stroke red channel.
 * @param green Stroke green channel.
 * @param blue Stroke blue channel.
 * @param alpha Stroke opacity.
 */
static void draw_brush(struct rgb_image *image, int x, int y, int width,
                       uint8_t red, uint8_t green, uint8_t blue, double alpha)
{
 int radius = width / 2;
 for (int dy = -radius; dy <= radius; dy++) {
 for (int dx = -radius; dx <= radius; dx++) {
 put_blended_pixel(image, x + dx, y + dy, red, green, blue, alpha);
    }
  }
}
 
/**
 * @brief Draw a raster line with an integer Bresenham walk.
 * @param image Destination RGB image.
 * @param x0 Start X.
 * @param y0 Start Y.
 * @param x1 End X.
 * @param y1 End Y.
 * @param width Brush width.
 * @param red Stroke red channel.
 * @param green Stroke green channel.
 * @param blue Stroke blue channel.
 * @param alpha Stroke opacity.
 * @details A compact local rasterizer avoids a drawing library for a few crisp
 * lines; per-pixel clipping remains centralized in `put_blended_pixel`.
 */
static void draw_line(struct rgb_image *image, int x0, int y0, int x1, int y1,
 int width, uint8_t red, uint8_t green, uint8_t blue, double alpha)
{
 int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
 int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
 int error = dx + dy;
 for (;;) {
 draw_brush(image, x0, y0, width, red, green, blue, alpha);
 if (x0 == x1 && y0 == y1) { break; }
 int twice = 2 * error;
 if (twice >= dy) { error += dy; x0 += sx; }
 if (twice <= dx) { error += dx; y0 += sy; }
  }
}
 
/**
 * @brief Draw a filled disc or clipped ring with a fixed-width stroke.
 * @param image Destination RGB image.
 * @param cx Center X in output pixels.
 * @param cy Center Y in output pixels.
 * @param radius Outer radius in pixels.
 * @param width Ring thickness, ignored for a filled disc.
 * @param red Annotation red channel.
 * @param green Annotation green channel.
 * @param blue Annotation blue channel.
 * @param alpha Annotation opacity.
 * @details Reject circles vastly larger than the image before squaring radii;
 * clipping the remaining scan bounds avoids visiting pixels outside the output.
 */
static void draw_circle(struct rgb_image *image, int cx, int cy, int radius,
 int width, uint8_t red, uint8_t green, uint8_t blue, double alpha)
{
 if (radius <= 0 || radius > (int)(image->width + image->height)) { return; }
 int inner = radius - width;
 int x0 = cx - radius - 1, x1 = cx + radius + 1;
 int y0 = cy - radius - 1, y1 = cy + radius + 1;
 if (x0 < 0) { x0 = 0; }
 if (y0 < 0) { y0 = 0; }
 if (x1 >= (int)image->width) { x1 = (int)image->width - 1; }
 if (y1 >= (int)image->height) { y1 = (int)image->height - 1; }
 for (int y = y0; y <= y1; y++) {
 for (int x = x0; x <= x1; x++) {
 int dx = x - cx, dy = y - cy;
 int distance2 = dx * dx + dy * dy;
 if (distance2 <= radius * radius && (inner <= 0 || distance2 >= inner * inner)) {
 put_blended_pixel(image, x, y, red, green, blue, alpha);
      }
    }
  }
}
 
/**
 * @brief Draw a distinct, high-contrast marker for the estimated loudest spot.
 * @param image Destination image.
 * @param x Marker center X.
 * @param y Marker center Y.
 * @param red Foreground red channel.
 * @param green Foreground green channel.
 * @param blue Foreground blue channel.
 * @details A dark casing pass precedes the colored stroke so the estimate remains
 * visible over both bright and dark map/heat pixels.
 */
static void draw_crosshair(struct rgb_image *image, int x, int y,
                           uint8_t red, uint8_t green, uint8_t blue)
{
 const int radius = 9, arm = 24, gap = 4;
 for (int pass = 0; pass < 2; pass++) {
 int width = pass == 0 ? 5 : 2;
    uint8_t r = pass == 0 ? 0 : red;
    uint8_t g = pass == 0 ? 0 : green;
    uint8_t b = pass == 0 ? 0 : blue;
 for (int axis = 0; axis < 2; axis++) {
 int dx = axis == 0 ? 1 : 0;
 int dy = axis == 0 ? 0 : 1;
 draw_line(image, x + dx * gap, y + dy * gap, x + dx * arm, y + dy * arm, width, r, g, b, 1.0);
 draw_line(image, x - dx * gap, y - dy * gap, x - dx * arm, y - dy * arm, width, r, g, b, 1.0);
 draw_circle(image, x, y, radius, width, r, g, b, 1.0);
    }
  }
}
 
/**
 * @brief Draw the known simulation source as a shape distinct from the estimate.
 * @param image Destination image.
 * @param x Diamond center X.
 * @param y Diamond center Y.
 * @param radius Diamond half-width and half-height.
 * @param red Foreground red channel.
 * @param green Foreground green channel.
 * @param blue Foreground blue channel.
 * @details Truth is a separate simulation reference, not another sensor reading;
 * the black outline keeps that distinction visible on any background.
 */
static void draw_diamond(struct rgb_image *image, int x, int y, int radius,
                         uint8_t red, uint8_t green, uint8_t blue)
{
 const int offsets[4][2] = {{0, -1}, {1, 0}, {0, 1}, {-1, 0}};
 for (int pass = 0; pass < 2; pass++) {
 int width = pass == 0 ? 5 : 2;
    uint8_t r = pass == 0 ? 0 : red;
    uint8_t g = pass == 0 ? 0 : green;
    uint8_t b = pass == 0 ? 0 : blue;
 for (int side = 0; side < 4; side++) {
 int next = (side + 1) % 4;
 draw_line(image, x + offsets[side][0] * radius, y + offsets[side][1] * radius,
                x + offsets[next][0] * radius, y + offsets[next][1] * radius,
                width, r, g, b, 1.0);
    }
  }
}
 
/**
 * @brief Return one embedded 5x7 uppercase bitmap glyph.
 * @param character ASCII label character; unsupported characters become blank.
 * @return Seven row bitmasks with five meaningful bits each.
 * @details This tiny font keeps annotation text independent of system font files
 * and avoids adding a font-rendering library to the host utility.
 */
static const uint8_t *glyph_rows(char character)
{
 static const uint8_t font[59][7] = {
    ['(' - 32] = {2, 4, 8, 8, 8, 4, 2}, [')' - 32] = {8, 4, 2, 2, 2, 4, 8},
    [',' - 32] = {0, 0, 0, 0, 0, 4, 8}, ['-' - 32] = {0, 0, 0, 31, 0, 0, 0},
    ['.' - 32] = {0, 0, 0, 0, 0, 12, 12}, ['0' - 32] = {14, 17, 19, 21, 25, 17, 14},
    ['1' - 32] = {4, 12, 4, 4, 4, 4, 14}, ['2' - 32] = {14, 17, 1, 2, 4, 8, 31},
    ['3' - 32] = {30, 1, 1, 14, 1, 1, 30}, ['4' - 32] = {2, 6, 10, 18, 31, 2, 2},
    ['5' - 32] = {31, 16, 16, 30, 1, 1, 30}, ['6' - 32] = {14, 16, 16, 30, 17, 17, 14},
    ['7' - 32] = {31, 1, 2, 4, 8, 8, 8}, ['8' - 32] = {14, 17, 17, 14, 17, 17, 14},
    ['9' - 32] = {14, 17, 17, 15, 1, 1, 14}, [':' - 32] = {0, 12, 12, 0, 12, 12, 0},
    ['=' - 32] = {0, 31, 0, 31, 0, 0, 0}, ['>' - 32] = {16, 8, 4, 2, 4, 8, 16},
    ['A' - 32] = {14, 17, 17, 31, 17, 17, 17}, ['B' - 32] = {30, 17, 17, 30, 17, 17, 30},
    ['C' - 32] = {14, 17, 16, 16, 16, 17, 14}, ['D' - 32] = {30, 17, 17, 17, 17, 17, 30},
    ['E' - 32] = {31, 16, 16, 30, 16, 16, 31}, ['F' - 32] = {31, 16, 16, 30, 16, 16, 16},
    ['G' - 32] = {14, 17, 16, 23, 17, 17, 15}, ['H' - 32] = {17, 17, 17, 31, 17, 17, 17},
    ['I' - 32] = {14, 4, 4, 4, 4, 4, 14}, ['J' - 32] = {7, 2, 2, 2, 18, 18, 12},
    ['K' - 32] = {17, 18, 20, 24, 20, 18, 17}, ['L' - 32] = {16, 16, 16, 16, 16, 16, 31},
    ['M' - 32] = {17, 27, 21, 21, 17, 17, 17}, ['N' - 32] = {17, 25, 21, 19, 17, 17, 17},
    ['O' - 32] = {14, 17, 17, 17, 17, 17, 14}, ['P' - 32] = {30, 17, 17, 30, 16, 16, 16},
    ['Q' - 32] = {14, 17, 17, 17, 21, 18, 13}, ['R' - 32] = {30, 17, 17, 30, 20, 18, 17},
    ['S' - 32] = {15, 16, 16, 14, 1, 1, 30}, ['T' - 32] = {31, 4, 4, 4, 4, 4, 4},
    ['U' - 32] = {17, 17, 17, 17, 17, 17, 14}, ['V' - 32] = {17, 17, 17, 17, 17, 10, 4},
    ['W' - 32] = {17, 17, 17, 21, 21, 21, 10}, ['X' - 32] = {17, 17, 10, 4, 10, 17, 17},
    ['Y' - 32] = {17, 17, 10, 4, 4, 4, 4}, ['Z' - 32] = {31, 1, 2, 4, 8, 16, 31}
  };
 unsigned char upper = (unsigned char)toupper((unsigned char)character);
 return upper >= 32U && upper <= 90U ? font[upper - 32U] : font[' ' - 32];
}
 
/**
 * @brief Paint one foreground or outline pass of bitmap text.
 * @param image Destination image.
 * @param x Left coordinate.
 * @param y Top coordinate.
 * @param text NUL-terminated label.
 * @param scale Integer expansion per glyph pixel.
 * @param red Text red channel.
 * @param green Text green channel.
 * @param blue Text blue channel.
 */
static void draw_text_pass(struct rgb_image *image, int x, int y, const char *text,
 unsigned int scale, uint8_t red, uint8_t green, uint8_t blue)
{
 for (const unsigned char *cursor = (const unsigned char *)text; *cursor != '\0'; cursor++) {
 const uint8_t *glyph = glyph_rows((char)*cursor);
 for (unsigned int row = 0; row < 7U; row++) {
 for (unsigned int column = 0; column < 5U; column++) {
 if ((glyph[row] & (1U << (4U - column))) == 0) { continue; }
 for (unsigned int dy = 0; dy < scale; dy++) {
 for (unsigned int dx = 0; dx < scale; dx++) {
 put_blended_pixel(image, x + (int)(column * scale + dx), y + (int)(row * scale + dy),
                              red, green, blue, 1.0);
          }
        }
      }
    }
    x += (int)(6U * scale);
  }
}
 
/**
 * @brief Draw a small outlined label over arbitrary imagery.
 * @param image Destination image.
 * @param x Left coordinate.
 * @param y Top coordinate.
 * @param text NUL-terminated label.
 * @param red Foreground red channel.
 * @param green Foreground green channel.
 * @param blue Foreground blue channel.
 * @details The eight-neighbor dark casing is a lightweight contrast aid; no
 * external font or graphics library is needed for these short map labels.
 */
static void draw_text(struct rgb_image *image, int x, int y, const char *text,
                      uint8_t red, uint8_t green, uint8_t blue)
{
 const unsigned int scale = 2U;
 for (int dy = -1; dy <= 1; dy++) {
 for (int dx = -1; dx <= 1; dx++) {
 if (dx != 0 || dy != 0) { draw_text_pass(image, x + dx, y + dy, text, scale, 0, 0, 0); }
    }
  }
 draw_text_pass(image, x, y, text, scale, red, green, blue);
}
 
/**
 * @brief Convert a geographic annotation to pixels in the cropped tile mosaic.
 * @param lat Latitude in degrees.
 * @param lon Longitude in degrees.
 * @param origin_x Fractional XYZ X coordinate of output origin.
 * @param origin_y Fractional XYZ Y coordinate of output origin.
 * @param zoom Shared Web Mercator zoom.
 * @param x Receives image-space X.
 * @param y Receives image-space Y.
 */
static void pixel_location(double lat, double lon, double origin_x, double origin_y,
 unsigned int zoom, int *x, int *y)
{
 double tile_x, tile_y;
 mercator_xy(lat, lon, zoom, &tile_x, &tile_y);
  *x = (int)floor((tile_x - origin_x) * TILE_SIZE);
  *y = (int)floor((tile_y - origin_y) * TILE_SIZE);
}
 
/**
 * @brief Append a point into caller-owned bounded storage.
 * @param list Point list view.
 * @param point Validated point to copy.
 * @return 0 on append, -1 when its fixed capacity is exhausted.
 */
static int append_point(struct point_list *list, const struct point *point)
{
 if (list->count >= list->capacity) { return -1; }
  list->items[list->count++] = *point;
 return 0;
}
 
/**
 * @brief Parse one complete numeric field from a bounded CSV row.
 * @param line CSV record buffer.
 * @param column Zero-based field index.
 * @param value Receives the finite number on success.
 * @return true only when the selected field exists and contains a complete number.
 * @details A short stack token buffer rejects overlong cells rather than sizing
 * memory from untrusted file contents.
 */
static bool csv_number(const char *line, unsigned int column, double *value)
{
 const char *field = line;
 for (unsigned int index = 0; index < column; index++) {
    field = strchr(field, ',');
 if (field == NULL) { return false; }
    field++;
  }
 char buffer[128];
 size_t length = strcspn(field, ",\r\n");
 if (length == 0 || length >= sizeof(buffer)) { return false; }
 for (size_t index = 0; index < length; index++) { buffer[index] = field[index]; }
  buffer[length] = '\0';
 return parse_double(buffer, value);
}
 
/**
 * @brief Read geographic points from an EAR session or aircraft-track CSV.
 * @param path CSV path.
 * @param session_log Select session columns/timestamps instead of track schema.
 * @param points Bounded destination list view.
 * @return 0 after a complete read, -1 for I/O errors or record-cap overflow.
 * @details Session time is milliseconds and positions are columns 2/3; track time
 * is seconds and positions are columns 1/2. Invalid poses are skipped, not plotted.
 */
static int read_points(const char *path, bool session_log, struct point_list *points)
{
  FILE *input = fopen(path, "r");
 if (input == NULL) { perror(path); return -1; }
 char line[MAX_CSV_LINE];
 int status = 0;
 while (fgets(line, sizeof(line), input) != NULL) {
 if (line[0] == '\0' || line[0] == '#' || line[0] == 't') { continue; }
 struct point point = {0};
 double stamp = 0.0;
 bool valid = csv_number(line, 0, &stamp)
                 && csv_number(line, session_log ? 2U : 1U, &point.lat)
                 && csv_number(line, session_log ? 3U : 2U, &point.lon);
 if (!valid || !isfinite(point.lat) || !isfinite(point.lon) || fabs(point.lat) <= 1e-6
        || fabs(point.lat) > 90.0 || fabs(point.lon) > 180.0) { continue; }
 point.time_s = session_log ? stamp / 1000.0 : stamp;
 if (append_point(points, &point) != 0) { status = -1; break; }
  }
 if (ferror(input) != 0) { status = -1; }
 if (fclose(input) != 0) { status = -1; }
 if (status != 0) { fprintf(stderr, "%s: invalid or excessive CSV input\n", path); }
 return status;
}
 
/**
 * @brief Locate an exact field name in a CSV header without mutating the source.
 * @param header Bounded header row.
 * @param name Required column name.
 * @return Zero-based field index, or -1 when absent or overlong.
 * @details Each lookup tokenizes a stack copy so querying several required fields
 * does not consume or corrupt the shared header.
 */
static int header_column(const char *header, const char *name)
{
 char copy[MAX_CSV_LINE];
 size_t length = strnlen(header, sizeof(copy));
 if (length >= sizeof(copy)) { return -1; }
 for (size_t index = 0; index <= length; index++) { copy[index] = header[index]; }
 int column = 0;
 char *save = NULL;
 for (char *field = strtok_r(copy, ",\r\n", &save); field != NULL; field = strtok_r(NULL, ",\r\n", &save), column++) {
 if (strcmp(field, name) == 0) { return column; }
  }
 return -1;
}
 
/** @brief Read one required numeric column from a LiDAR CSV row. */
static bool csv_column(const char *line, unsigned int column, double *value)
{
 return csv_number(line, column, value);
}
 
/**
 * @brief Append one obstacle cell when fixed storage has room.
 * @param list Bounded obstacle list view.
 * @param obstacle Validated cell to copy.
 * @return 0 on append, -1 when the configured cell cap is reached.
 */
static int append_obstacle(struct obstacle_list *list, const struct obstacle *obstacle)
{
 if (list->count >= list->capacity) { return -1; }
  list->items[list->count++] = *obstacle;
 return 0;
}
 
/** @brief Order projected eastings so adjacent positive deltas reveal cell pitch. */
static int compare_double(const void *left, const void *right)
{
 double a = *(const double *)left;
 double b = *(const double *)right;
 return (a > b) - (a < b);
}
 
/**
 * @brief Read the optional regular LiDAR height grid and infer its cell width.
 * @param path CSV with `x_east_m`, `lat`, `lon`, and `mnh_m` header columns.
 * @param obstacles Receives validated cells in fixed-capacity storage.
 * @param cell_size Receives the smallest positive spacing between eastings.
 * @return 0 for a usable grid, -1 for I/O, schema, capacity or spacing failures.
 * @details Sorting the projected eastings distinguishes grid pitch from geographic
 * map scale; height colors remain visual context, not a clearance/navigation result.
 */
static int read_obstacles(const char *path, struct obstacle_list *obstacles, double *cell_size)
{
  FILE *input = fopen(path, "r");
 if (input == NULL) { perror(path); return -1; }
 char line[MAX_CSV_LINE];
 int east_column = -1, lat_column = -1, lon_column = -1, height_column = -1;
 if (fgets(line, sizeof(line), input) != NULL) {
    east_column = header_column(line, "x_east_m");
    lat_column = header_column(line, "lat");
    lon_column = header_column(line, "lon");
    height_column = header_column(line, "mnh_m");
  }
 if (east_column < 0 || lat_column < 0 || lon_column < 0 || height_column < 0) {
    fclose(input);
    fprintf(stderr, "%s: expected x_east_m,lat,lon,mnh_m columns\n", path);
 return -1;
  }
 double *eastings = obstacle_eastings;
 size_t easting_count = 0;
 int status = 0;
 while (fgets(line, sizeof(line), input) != NULL) {
 double values[4];
 int columns[4] = {east_column, lat_column, lon_column, height_column};
 bool valid = true;
 for (int index = 0; index < 4; index++) {
 if (!csv_column(line, (unsigned int)columns[index], &values[index])) { valid = false; break; }
    }
 if (!valid || fabs(values[1]) > 90.0 || fabs(values[2]) > 180.0) { continue; }
 struct obstacle obstacle = {values[0], values[1], values[2], values[3]};
 if (append_obstacle(obstacles, &obstacle) != 0) { status = -1; break; }
 if (easting_count >= MAX_CSV_POINTS) { status = -1; break; }
    eastings[easting_count++] = values[0];
  }
 if (ferror(input) != 0) { status = -1; }
  fclose(input);
 if (status == 0 && easting_count > 1) {
    qsort(eastings, easting_count, sizeof(*eastings), compare_double);
    *cell_size = INFINITY;
 for (size_t index = 1; index < easting_count; index++) {
 double difference = eastings[index] - eastings[index - 1];
 if (difference > 0.0 && difference < *cell_size) { *cell_size = difference; }
    }
 if (!isfinite(*cell_size)) { status = -1; }
  } else { status = -1; }
 if (status != 0) { fprintf(stderr, "%s: no usable obstacle-grid spacing\n", path); }
 return status;
}
 
/**
 * @brief Compute great-circle distance between two latitude/longitude points.
 * @param lat_a First latitude in degrees.
 * @param lon_a First longitude in degrees.
 * @param lat_b Second latitude in degrees.
 * @param lon_b Second longitude in degrees.
 * @return Spherical-Earth separation in metres.
 * @details Used to annotate known simulation truth versus the estimate; it does
 * not correct the estimate or certify field accuracy.
 */
static double haversine_m(double lat_a, double lon_a, double lat_b, double lon_b)
{
 double d_lat = (lat_b - lat_a) * DEG_TO_RAD;
 double d_lon = (lon_b - lon_a) * DEG_TO_RAD;
 double a = sin(d_lat / 2.0) * sin(d_lat / 2.0)
             + cos(lat_a * DEG_TO_RAD) * cos(lat_b * DEG_TO_RAD)
             * sin(d_lon / 2.0) * sin(d_lon / 2.0);
 return 2.0 * EARTH_RADIUS_M * asin(sqrt(fmax(0.0, fmin(1.0, a))));
}
 
/**
 * @brief Redraw sample locations as crisp dots over the blended sound field.
 * @param image Output map.
 * @param points Parsed EAR session observations.
 * @param origin_x Fractional XYZ X coordinate of output origin.
 * @param origin_y Fractional XYZ Y coordinate of output origin.
 * @param zoom Map tile zoom.
 * @details A black edge and white core keep individual observations visible on
 * both satellite imagery and the heat palette.
 */
static void draw_log_points(struct rgb_image *image, const struct point_list *points,
 double origin_x, double origin_y, unsigned int zoom)
{
 for (size_t index = 0; index < points->count; index++) {
 int x, y;
 pixel_location(points->items[index].lat, points->items[index].lon, origin_x, origin_y, zoom, &x, &y);
 draw_circle(image, x, y, 2, 2, 0, 0, 0, 1.0);
 draw_circle(image, x, y, 1, 1, 255, 255, 255, 1.0);
  }
}
 
/**
 * @brief Draw aircraft-track segments, breaking where fixes are too far apart.
 * @param image Output map.
 * @param points Ordered fixes with normalized second timestamps.
 * @param origin_x Fractional XYZ X coordinate of output origin.
 * @param origin_y Fractional XYZ Y coordinate of output origin.
 * @param zoom Map tile zoom.
 * @param gap_s Largest allowed timestamp gap within a connected segment.
 * @details Breaking stale intervals avoids drawing a fabricated straight path
 * through periods where no aircraft positions were recorded.
 */
static void draw_track(struct rgb_image *image, const struct point_list *points,
 double origin_x, double origin_y, unsigned int zoom, double gap_s)
{
 for (size_t index = 1; index < points->count; index++) {
 if (points->items[index].time_s - points->items[index - 1].time_s > gap_s) { continue; }
 int x0, y0, x1, y1;
 pixel_location(points->items[index - 1].lat, points->items[index - 1].lon, origin_x, origin_y, zoom, &x0, &y0);
 pixel_location(points->items[index].lat, points->items[index].lon, origin_x, origin_y, zoom, &x1, &y1);
 draw_line(image, x0, y0, x1, y1, 3, 0, 0, 0, 0.9);
 draw_line(image, x0, y0, x1, y1, 1, 255, 235, 120, 1.0);
  }
}
 
/**
 * @brief Paint optional height-above-terrain cells over the output map.
 * @param image Output map.
 * @param obstacles Parsed regular LiDAR cells.
 * @param cell_size Projected cell pitch in metres.
 * @param m_per_deg_lat Local metres per latitude degree.
 * @param m_per_deg_lon Local metres per longitude degree.
 * @param origin_x Fractional XYZ X coordinate of output origin.
 * @param origin_y Fractional XYZ Y coordinate of output origin.
 * @param zoom Map tile zoom.
 * @details Cell bounds are clipped before painting, and color bands are only
 * qualitative review aids; no collision or flight-control logic consumes them.
 */
static void draw_obstacles(struct rgb_image *image, const struct obstacle_list *obstacles,
 double cell_size, double m_per_deg_lat, double m_per_deg_lon,
 double origin_x, double origin_y, unsigned int zoom)
{
 for (size_t index = 0; index < obstacles->count; index++) {
 const struct obstacle *cell = &obstacles->items[index];
 if (cell->height_m < 3.0) { continue; }
 int x0, y0, x1, y1;
 pixel_location(cell->lat + cell_size / (2.0 * m_per_deg_lat),
                   cell->lon - cell_size / (2.0 * m_per_deg_lon), origin_x, origin_y, zoom, &x0, &y0);
 pixel_location(cell->lat - cell_size / (2.0 * m_per_deg_lat),
                   cell->lon + cell_size / (2.0 * m_per_deg_lon), origin_x, origin_y, zoom, &x1, &y1);
 if (x0 > x1) { int swap = x0; x0 = x1; x1 = swap; }
 if (y0 > y1) { int swap = y0; y0 = y1; y1 = swap; }
 if (x1 < 0 || y1 < 0 || x0 >= (int)image->width || y0 >= (int)image->height) { continue; }
 if (x0 < 0) { x0 = 0; }
 if (y0 < 0) { y0 = 0; }
 if (x1 >= (int)image->width) { x1 = (int)image->width - 1; }
 if (y1 >= (int)image->height) { y1 = (int)image->height - 1; }
 double alpha = cell->height_m > 6.0 ? 0.48 : 0.35;
 for (int y = y0; y <= y1; y++) {
 for (int x = x0; x <= x1; x++) { put_blended_pixel(image, x, y, 255, cell->height_m > 6.0 ? 40 : 150, 40, alpha); }
    }
  }
}
 
/**
 * @brief Assemble basemap tiles, heatfield and optional annotations into one output raster.
 * @param options Parsed cache, projection and annotation policy.
 * @param geo Validated local-projection footprint from the `.geo` sidecar.
 * @param heat Undecorated `_field.jpg` when present, otherwise the input JPEG.
 * @param output Receives the final cropped raster in fixed output storage.
 * @return 0 after composition, or -1 for malformed annotations, unsupported bounds,
 * cache failures or capacity limits.
 * @details Bounds are computed before tile access; only intersecting tiles are
 * assembled, heat pixels are projected individually, then crisp evidence layers
 * are drawn on top. No step feeds back into acoustic measurements or navigation.
 */
static int build_output(const struct options *options, const struct ear_heatmap_georef *geo,
 const struct rgb_image *heat, struct rgb_image *output)
{
 double search_lat = 0.0, search_lon = 0.0, search_radius = 25.0;
 bool has_search = options->search_text != NULL;
 if (has_search && !parse_location(options->search_text, true, &search_lat, &search_lon, &search_radius)) {
    fprintf(stderr, "--search expects LAT,LON[,RADIUS_M]\n"); return -1;
  }
 double truth_lat = 0.0, truth_lon = 0.0, unused_radius = 0.0;
 bool has_truth = options->truth_text != NULL;
 if (has_truth && !parse_location(options->truth_text, false, &truth_lat, &truth_lon, &unused_radius)) {
    fprintf(stderr, "--truth expects LAT,LON\n"); return -1;
  }
 double m_per_deg_lat = DEG_TO_RAD * EARTH_RADIUS_M;
 double m_per_deg_lon = m_per_deg_lat * cos(geo->lat_center * DEG_TO_RAD);
 if (m_per_deg_lon <= 1e-6) { fprintf(stderr, "Heatmap is too close to a pole.\n"); return -1; }
   // Keep the field's local equirectangular origin explicit; basemap pixels are Mercator.
   double east0 = (geo->lon_west - geo->lon_center) * m_per_deg_lon;
 double north0 = (geo->lat_north - geo->lat_center) * m_per_deg_lat;
 double width_m = geo->width_px * geo->m_per_px;
 double height_m = geo->height_px * geo->m_per_px;
   // Expand map bounds far enough to include the full optional search circle.
   double margin = options->margin_m;
 if (has_search) {
 double east = (search_lon - geo->lon_center) * m_per_deg_lon;
 double north = (search_lat - geo->lat_center) * m_per_deg_lat;
 double needed[4] = {east + search_radius - (east0 + width_m) + 5.0,
                        east0 - (east - search_radius) + 5.0,
                        north + search_radius - north0 + 5.0,
                        (north0 - height_m) - (north - search_radius) + 5.0};
 for (unsigned int index = 0; index < 4; index++) { if (needed[index] > margin) { margin = needed[index]; } }
  }
 double lat_north = geo->lat_center + (north0 + margin) / m_per_deg_lat;
 double lat_south = geo->lat_center + (north0 - height_m - margin) / m_per_deg_lat;
 double lon_west = geo->lon_center + (east0 - margin) / m_per_deg_lon;
 double lon_east = geo->lon_center + (east0 + width_m + margin) / m_per_deg_lon;
 if (lat_north > 85.0 || lat_south < -85.0 || lon_west < -180.0 || lon_east > 180.0) {
    fprintf(stderr, "Requested overlay crosses supported map bounds.\n"); return -1;
  }
 unsigned int zoom = options->zoom == 0 ? choose_zoom(geo, options->max_zoom) : options->zoom;
 double x_w, y_n, x_e, y_s;
 mercator_xy(lat_north, lon_west, zoom, &x_w, &y_n);
 mercator_xy(lat_south, lon_east, zoom, &x_e, &y_s);
 if (!(x_e > x_w && y_s > y_n)) { return -1; }
 int tx0 = (int)floor(x_w), ty0 = (int)floor(y_n);
 int tx1 = (int)floor(x_e), ty1 = (int)floor(y_s);
 if (tx0 < 0 || ty0 < 0 || tx1 < tx0 || ty1 < ty0
      || tx1 >= (int)ldexp(1.0, (int)zoom) || ty1 >= (int)ldexp(1.0, (int)zoom)) {
    fprintf(stderr, "Tile bounds are outside Web Mercator.\n"); return -1;
  }
   // Reject oversized requests before touching cache/network or output storage.
   uint64_t tile_count = (uint64_t)(tx1 - tx0 + 1) * (uint64_t)(ty1 - ty0 + 1);
 if (tile_count > MAX_TILE_COUNT) { fprintf(stderr, "Overlay requires too many map tiles (%llu).\n", (unsigned long long)tile_count); return -1; }
 int crop_x = (int)floor((x_w - tx0) * TILE_SIZE);
 int crop_y = (int)floor((y_n - ty0) * TILE_SIZE);
 int end_x = (int)ceil((x_e - tx0) * TILE_SIZE);
 int end_y = (int)ceil((y_s - ty0) * TILE_SIZE);
 if (end_x <= crop_x || end_y <= crop_y) { return -1; }
 output->width = (unsigned int)(end_x - crop_x);
 output->height = (unsigned int)(end_y - crop_y);
  uint64_t pixels = (uint64_t)output->width * output->height;
 if (pixels > MAX_OUTPUT_PIXELS) {
    fprintf(stderr, "Overlay output exceeds the %u-pixel memory limit. Increase margin less or choose a lower zoom.\n", MAX_OUTPUT_PIXELS);
 return -1;
  }
   // One bounded mosaic buffer avoids a second full-resolution assembly surface.
   output->pixels = output_pixels;
 output->capacity_bytes = sizeof(output_pixels);
 for (size_t index = 0; index < (size_t)pixels * 3U; index++) { output->pixels[index] = 0; }
 for (int ty = ty0; ty <= ty1; ty++) {
 for (int tx = tx0; tx <= tx1; tx++) {
 struct rgb_image tile = {.pixels = tile_pixels, .capacity_bytes = sizeof(tile_pixels)};
 if (fetch_tile(options, (unsigned int)tx, (unsigned int)ty, zoom, &tile) != 0) { return -1; }
 copy_tile_to_output(&tile, output, tx, ty, crop_x, crop_y);
 image_free(&tile);
    }
  }
 int crop_end_x = crop_x + (int)output->width;
 int crop_end_y = crop_y + (int)output->height;
   // All later field/marker transforms share the crop's pixel-aligned XYZ origin.
   double origin_x = tx0 + (double)crop_x / TILE_SIZE;
 double origin_y = ty0 + (double)crop_y / TILE_SIZE;
  (void)crop_end_x;
  (void)crop_end_y;
 composite_heatmap(output, heat, geo, zoom, crop_x, crop_y, options->alpha);
 
   // Draw evidence after the translucent field so sample/track colors stay legible.
   struct point_list samples = {.items = session_points, .capacity = MAX_CSV_POINTS};
 struct point_list track = {.items = track_points, .capacity = MAX_CSV_POINTS};
 if (options->log_path != NULL && read_points(options->log_path, true, &samples) != 0) { return -1; }
 if (options->track_path != NULL && read_points(options->track_path, false, &track) != 0) { return -1; }
 draw_log_points(output, &samples, origin_x, origin_y, zoom);
 struct obstacle_list obstacles = {.items = obstacle_cells, .capacity = MAX_CSV_POINTS};
 if (options->obstacles_path != NULL) {
 double cell_size = 0.0;
 if (read_obstacles(options->obstacles_path, &obstacles, &cell_size) != 0) {
 return -1;
    }
 draw_obstacles(output, &obstacles, cell_size, m_per_deg_lat, m_per_deg_lon, origin_x, origin_y, zoom);
 draw_text(output, 4, 4, "TREES 3-6M ORANGE 6M+ RED", 255, 170, 90);
  }
 double output_m_per_px = mercator_m_per_pixel(geo->lat_center, zoom);
 int bar_length = (int)fmin(50.0 / output_m_per_px, 0.6 * output->width);
 int bar_x = (int)(0.04 * output->width);
 int bar_y = (int)(0.94 * output->height);
 draw_line(output, bar_x, bar_y, bar_x + bar_length, bar_y, 5, 0, 0, 0, 1.0);
 draw_line(output, bar_x, bar_y, bar_x + bar_length, bar_y, 2, 255, 255, 255, 1.0);
 draw_line(output, bar_x, bar_y - 6, bar_x, bar_y + 6, 2, 255, 255, 255, 1.0);
 draw_line(output, bar_x + bar_length, bar_y - 6, bar_x + bar_length, bar_y + 6, 2, 255, 255, 255, 1.0);
 draw_text(output, bar_x, bar_y - 19, "50 M", 255, 255, 255);
 draw_track(output, &track, origin_x, origin_y, zoom, options->track_gap_s);
 if (geo->has_spot) {
 int x, y;
 pixel_location(geo->spot_lat, geo->spot_lon, origin_x, origin_y, zoom, &x, &y);
 draw_crosshair(output, x, y, 255, 40, 255);
 draw_text(output, x + 12, y + 10, "LOUDEST SPOT", 255, 40, 255);
  }
 if (has_search) {
 int x, y;
 pixel_location(search_lat, search_lon, origin_x, origin_y, zoom, &x, &y);
 int radius = (int)(search_radius / output_m_per_px + 0.5);
 draw_circle(output, x, y, radius, 2, 255, 255, 255, 1.0);
 draw_line(output, x - 6, y, x + 6, y, 2, 255, 255, 255, 1.0);
 draw_line(output, x, y - 6, x, y + 6, 2, 255, 255, 255, 1.0);
 char radius_label[48];
    snprintf(radius_label, sizeof(radius_label), "R=%.0f M", search_radius);
 draw_text(output, x + 8, y - radius - 20, "GIVEN POINT", 255, 255, 255);
 draw_text(output, x - 12, y - radius - 4, radius_label, 255, 255, 255);
  }
 if (has_truth) {
 int x, y;
 pixel_location(truth_lat, truth_lon, origin_x, origin_y, zoom, &x, &y);
 draw_diamond(output, x, y, 9, 255, 40, 40);
 draw_text(output, x + 12, y - 24, "SOUND SOURCE", 255, 70, 70);
 if (geo->has_spot) {
 double offset_m = haversine_m(truth_lat, truth_lon, geo->spot_lat, geo->spot_lon);
 char offset_label[48];
      snprintf(offset_label, sizeof(offset_label), "%.1f M OFF", offset_m);
 draw_text(output, x + 12, y - 8, offset_label, 255, 70, 70);
      fprintf(stderr, "truth-to-estimated-spot=%.1f m\n", offset_m);
    }
  }
 if (!options->quiet) {
    printf("overlay %ux%u, %s tiles z%u, %.3f m/px\n", output->width, output->height,
 options->source, zoom, output_m_per_px);
  }
 return 0;
}
 
/**
 * @brief Validate replay artifacts, compose the map context and write a JPEG.
 * @param argc Command-line argument count.
 * @param argv Process arguments; option strings are borrowed for this invocation.
 * @return 0 on success, 2 for usage/policy errors, or 1 for I/O/data failure.
 * @details Prefer the matching undecorated field because reusing the decorated
 * heatmap would blend its original dots/marker into the basemap a second time.
 * The sidecar dimensions are checked before any map data is read.
 */
int main(int argc, char **argv)
{
 if (argc == 2 && (strcmp(argv[1], "--version") == 0 || strcmp(argv[1], "-V") == 0)) {
    puts(EAR_HEATMAP_OVERLAY_BUILD_VERSION);
 return 0;
  }
 if (argc == 2 && (strcmp(argv[1], "--help") == 0 || strcmp(argv[1], "-h") == 0)) {
 print_help(argv[0]);
 return 0;
  }
 struct options options;
 if (!parse_options(argc, argv, &options)) {
 print_help(argv[0]);
 return 2;
  }
 if (options.source == NULL || (strcmp(options.source, "google") != 0 && strcmp(options.source, "esri") != 0)
      || options.max_zoom < 12 || options.max_zoom > 22 || options.zoom > 22
      || options.alpha < 0.0 || options.alpha > 1.0 || options.margin_m < 0.0
      || options.track_gap_s < 0.0) {
    fprintf(stderr, "Invalid source, zoom, alpha, margin or track gap.\n");
 return 2;
  }
 char geo_path[MAX_PATH_LENGTH], field_path[MAX_PATH_LENGTH], output_path[MAX_PATH_LENGTH];
 char cache_path[MAX_PATH_LENGTH];
 if (options.geo_path == NULL) {
 if (make_stem_path(options.input, ".geo", geo_path, sizeof(geo_path)) != 0) { return 2; }
 options.geo_path = geo_path;
  }
 if (make_stem_path(options.input, "_field.jpg", field_path, sizeof(field_path)) != 0) { return 2; }
 if (options.output_path == NULL) {
 if (make_stem_path(options.input, "_sat.jpg", output_path, sizeof(output_path)) != 0) { return 2; }
 options.output_path = output_path;
  }
 if (options.cache_path == NULL) {
 if (default_cache_path(cache_path, sizeof(cache_path)) != 0) {
      fprintf(stderr, "Set HOME or pass --cache.\n"); return 2;
    }
 options.cache_path = cache_path;
  }
 struct ear_heatmap_georef geo = {0};
 if (ear_heatmap_read_georef(options.geo_path, &geo) != 0) {
    fprintf(stderr, "%s: cannot read valid EAR heatmap georeference\n", options.geo_path); return 1;
  }
   // Decode exactly one raster into the fixed input buffer, avoiding a duplicate image copy.
   struct rgb_image field = {.pixels = input_pixels, .capacity_bytes = sizeof(input_pixels)};
 struct rgb_image output = {0};
 const char *field_input = access(field_path, R_OK) == 0 ? field_path : options.input;
 if (image_read_jpeg(field_input, &field) != 0) {
    fprintf(stderr, "Cannot read heatmap JPEG: %s\n", field_input); return 1;
  }
 if (field.width != geo.width_px || field.height != geo.height_px
      || field.width > MAX_INPUT_PIXELS || field.height > MAX_INPUT_PIXELS) {
    fprintf(stderr, "Heatmap dimensions do not match its .geo sidecar.\n");
 image_free(&field);
 return 1;
  }
 int status = build_output(&options, &geo, &field, &output);
 if (status == 0 && image_write_jpeg(options.output_path, &output) != 0) {
    perror(options.output_path);
    status = -1;
  }
 if (status == 0) { printf("wrote %s\n", options.output_path); }
 image_free(&output);
 image_free(&field);
 return status == 0 ? 0 : 1;
}
