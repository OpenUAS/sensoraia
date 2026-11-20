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
 * @file lwir_capture.c
 * @brief Command-line entry point of the `lwircam` executable.
 *
 * @details
 * One binary serves several mutually exclusive roles, selected by options:
 * - **One-shot capture** (default): `main()` forks; the child runs
 *   save_capture() while the parent supervise()s it against a monotonic
 *   deadline, so a wedged USB/UVC call cannot outlive `timeout_ms`.
 * - **Persistent capture server** (`--capture-server`): serve_captures() keeps
 *   the camera streaming and answers one absolute output path per stdin line.
 *   This is the interface USHER drives.
 * - **Raw replay** (`--raw FRAME --size W H`): renders a recorded frame through
 *   the same statistics/JPEG/publication path, marked as simulated.
 * - **Standalone camera controls** (`--get-control`, `--set-gain`,
 *   `--set-emissivity-128`, `--set-agc-mode`, `--set-agc`, `--ffc`): exactly one
 *   control per invocation, no image output.
 * - **Optional builds**: analysis builds add `--analyze`, `--check-model`,
 *   `--calibration` and `--geolocate`; mock builds add `--mock-image` and
 *   `--mock-layer`. Those options are rejected in builds without the feature.
 *
 * Capture data flow:
 * 1. Acquire a frame (camera via lwir_device, or replay file) into the static
 *    ::captured_frame buffer. Freshness is measured from the start of the
 *    request, so a published photo is never older than the request.
 * 2. Optionally read device state (`--device-state`): opt-in, read-only,
 *    after the frame (not frame-atomic) and bounded to 250 ms.
 * 3. Optionally compute temperature statistics/threshold coverage.
 * 4. Hash the frame bytes (SHA-256) into the evidence record.
 * 5. Optionally write a raw companion (`--record-raw` or `--native-raw`, which
 *    derives `PATH.raw`) for byte-exact verification.
 * 6. Encode the JPEG with evidence metadata; the temperature plane is stored
 *    exactly in APP15 markers. Capture defaults to quality 95 with 4:4:4
 *    sampling (0 in the library means legacy quality 100).
 * 7. Publish durably without overwriting: the image library writes a temporary
 *    file, fsyncs it, link()s it to the final name (fails if it exists) and
 *    fsyncs the directory. `main()` also refuses an existing output up front.
 *
 * Configuration precedence: the settings INI (`--settings`/`--config`, or the
 * discovered default `lwircam` settings file) is loaded first, then command-line
 * options override or extend it. Each INI key may appear only once.
 *
 * Exit codes: 0 success; 1 runtime failure; 2 usage/configuration error
 * (including capture-server protocol violations); 124 supervisor timeout;
 * 128+signal when interrupted by SIGINT/SIGTERM.
 *
 * Memory policy: no heap in this file and no memcpy (project policy); strings
 * are copied with bounded loops into fixed static storage.
 */

#include "lwir_frame.h"
#include "lwir_image.h"
#include "lwir_features.h"
#if LWIR_ENABLE_ANALYSIS
#include "lwir_analysis.h"
#include "lwir_inference.h"
#endif
#if LWIR_ENABLE_MOCK
#include "lwir_mock.h"
#endif
#include "boot_id.h"
#include "sha256fst.h"
#include "lwir_render.h"
#include "bounded_ini.h"
#include "inference_settings.h"
#include "version.h"

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <poll.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

#include "lwir_device.h"

/**
 * @brief Effective run policy: INI settings first, then command-line overrides.
 *
 * String members point either into `argv` or into the static `configured_*`
 * buffers, so they stay valid for the whole process lifetime.
 */
struct capture_options {
    const char *output;          /**< Final JPEG path; rewritten per request in server mode. */
    const char *raw;             /**< Replay input frame (`--raw`); selects replay instead of the camera. */
    const char *serial;          /**< Camera serial to select; must be < 64 bytes. */
    const char *device_path;     /**< Explicit `/dev/...` node; V4L2 builds only. */
    const char *record_raw;      /**< Raw companion path; explicit or derived `OUTPUT.raw` from native_raw. */
    const char *gain_label;      /**< Metadata label matched against the correction profile; physical gain is `--set-gain`. */
    const char *corrections;     /**< Correction profile path; must match serial/gain label/geometry. */
    const char *replay_serial;   /**< Serial recorded in replay evidence; replay only. */
    const char *config;          /**< Settings INI path, loaded before argument parsing. */
    const char *calibration;     /**< Geolocation calibration file (analysis builds). */
    const char *geolocate_input; /**< JPEG to geolocate offline (analysis builds). */
    const char *mock_input;      /**< Mock source image (mock builds). */
    bool mock_layer_only;        /**< `--mock-layer`: update the input in place, no separate output. */
    unsigned width;              /**< Image/temperature plane width; 0 = negotiate with the camera. */
    unsigned height;             /**< Plane height, not the doubled UVC height of combined frames. */
    unsigned warmup_ms;          /**< Startup qualification time; forced to 0 for controls and server requests. */
    unsigned timeout_ms;         /**< Supervisor deadline, server watchdog and idle-frame timeout. */
    bool report_stats;           /**< Print statistics; implied by measurements and thresholds. */
    bool scene_check;
    bool settle;
    bool device_state;           /**< Opt-in read-only state query after the frame. */
    bool control_selected;       /**< A standalone control was requested; excludes every capture mode. */
    bool control_read;           /**< The selected control is `--get-control` rather than a write/trigger. */
    struct lwir_control_request control;
    bool server;
    bool capture_requested;      /**< Explicit `--capture`; only valid together with an output. */
    bool warmup_only;            /**< `--warmup`: qualify the camera once, publish nothing. */
    bool native_raw;             /**< Derive the raw companion as `OUTPUT.raw`. */
    bool warn_percent_set;       /**< Distinguishes an explicit `--warn-percent` from the default of 1. */
    bool jpeg_quality_selected;  /**< Rejects a repeated `--jpeg-quality`; INI may still set the base value. */
    unsigned warn_percent;
    struct lwir_measurement measurement;
    struct lwir_thresholds thresholds;
    struct lwir_render_options render;
    enum lwir_frame_layout layout; /**< Filled from the opened device; image-only cameras halve the frame size. */
    const char *inference_input;
    bool check_model;
    double score_threshold;
    unsigned class_index;
};

static struct inference_settings model_settings;

/* Static to keep the ~196 KB frame off the stack. */
static uint8_t captured_frame[LWIR_MAX_FRAME_BYTES];
/* Holds the signal number (0 = none) so exit codes can report 128+signal. */
static volatile sig_atomic_t interrupted;
static struct lwir_evidence capture_evidence;
static char device_serial[64];
static struct lwir_profile correction_profile;
static struct lwir_temperature_stats captured_statistics;
static struct lwir_device_state captured_device_state;
static char device_state_text[4096];
static char configured_serial[64], configured_gain[32], configured_corrections[1024];
static char configured_device[1024];
static bool parse_unsigned(const char *text, unsigned maximum, unsigned *value);
static bool select_measurement(struct capture_options *options, enum lwir_measurement_kind kind, const unsigned *coordinates);
static bool parse_coordinates(const char *text, unsigned count, unsigned *coordinates);

/**
 * @brief Convert a Celsius band to device raw codes (Kelvin * 64).
 *
 * @details Rounds to nearest and re-checks ordering after rounding, because two
 * distinct close temperatures can collapse onto the same raw code.
 *
 * @param low_c Lower bound in degrees Celsius.
 * @param high_c Upper bound in degrees Celsius.
 * @param[out] low_raw Lower raw code; written only when both bounds are representable.
 * @param[out] high_raw Upper raw code.
 * @return true when both bounds are finite, within 0..65535 and strictly ordered.
 */
static bool temperature_band(double low_c, double high_c, uint16_t *low_raw, uint16_t *high_raw)
{
    const double low = (low_c + 273.15) * 64, high = (high_c + 273.15) * 64;
    if (!isfinite(low) || !isfinite(high) || low < 0 || high > 65535 || low >= high) return false;
    *low_raw = (uint16_t)(low + 0.5); *high_raw = (uint16_t)(high + 0.5);
    return *low_raw < *high_raw;
}

/**
 * @brief Configuration identities shared by key lookup and duplicate/pair masks.
 *
 * @details Each enumerator is both an index into the key table in
 * config_field() and a bit position in config_parser::seen. Setting a bit on
 * first sight lets a repeated key be rejected, and load_config() tests groups of
 * bits to require complete pairs (window, percentiles, thresholds) and to
 * forbid window and percentiles together.
 */
enum config_field {
    CONFIG_FORMAT,
    CONFIG_WIDTH,
    CONFIG_HEIGHT,
    CONFIG_WARMUP_MS,
    CONFIG_TIMEOUT_MS,
    CONFIG_WINDOW_LOW_C,
    CONFIG_WINDOW_HIGH_C,
    CONFIG_PERCENTILE_LOW,
    CONFIG_PERCENTILE_HIGH,
    CONFIG_PALETTE,
    CONFIG_LEGEND,
    CONFIG_DENOISE_K,
    CONFIG_SERIAL,
    CONFIG_GAIN_LABEL,
    CONFIG_CORRECTIONS,
    CONFIG_SCENE_CHECK,
    CONFIG_SETTLE,
    CONFIG_THRESHOLD_LOW_C,
    CONFIG_THRESHOLD_HIGH_C,
    CONFIG_WARN_PERCENT,
    CONFIG_STATS,
    CONFIG_POINT,
    CONFIG_LINE,
    CONFIG_AVERAGE_3X3,
    CONFIG_ROI,
    CONFIG_ROTATE,
    CONFIG_MIRROR,
    CONFIG_FLIP,
    CONFIG_DEVICE_STATE,
    CONFIG_SCORE_THRESHOLD,
    CONFIG_CLASS_INDEX,
    CONFIG_DEVICE,
    CONFIG_JPEG_QUALITY,
    CONFIG_FIELD_COUNT
};

_Static_assert(CONFIG_FIELD_COUNT <= 64, "Configuration identities must fit the duplicate mask");

/**
 * @brief Callback context while reading the settings INI.
 *
 * Paired Celsius values are staged here and validated together in
 * load_config(), since either half may arrive first.
 */
struct config_parser {
    struct capture_options *options;
    uint64_t seen;  /**< Bit per ::config_field already accepted. */
    double low_c;
    double high_c;
    double threshold_low_c;
    double threshold_high_c;
};

/**
 * @brief Accept one `key = value` pair from the settings INI.
 *
 * @details Maps the key to its ::config_field identity, rejects unknown or
 * repeated keys, validates the value and stores it in the options. Strings are
 * copied with bounded loops into static buffers because the INI reader's value
 * storage does not outlive the callback. Paired Celsius values are only staged;
 * load_config() checks their completeness and ordering.
 *
 * @param context A struct config_parser.
 * @param key INI key.
 * @param value INI value text.
 * @return false for an unknown/duplicate key or an invalid value, which makes
 *         the whole settings file invalid.
 */
static bool config_field(void *context, const char *key, const char *value)
{
    struct config_parser *parser = context;
    struct capture_options *options = parser->options;
    static const char *const keys[CONFIG_FIELD_COUNT] = {
        [CONFIG_FORMAT] = "format",
        [CONFIG_WIDTH] = "width",
        [CONFIG_HEIGHT] = "height",
        [CONFIG_WARMUP_MS] = "warmup_ms",
        [CONFIG_TIMEOUT_MS] = "timeout_ms",
        [CONFIG_WINDOW_LOW_C] = "window_low_c",
        [CONFIG_WINDOW_HIGH_C] = "window_high_c",
        [CONFIG_PERCENTILE_LOW] = "percentile_low",
        [CONFIG_PERCENTILE_HIGH] = "percentile_high",
        [CONFIG_PALETTE] = "palette",
        [CONFIG_LEGEND] = "legend",
        [CONFIG_DENOISE_K] = "denoise_k",
        [CONFIG_SERIAL] = "serial",
        [CONFIG_GAIN_LABEL] = "gain_label",
        [CONFIG_CORRECTIONS] = "corrections",
        [CONFIG_SCENE_CHECK] = "scene_check",
        [CONFIG_SETTLE] = "settle",
        [CONFIG_THRESHOLD_LOW_C] = "threshold_low_c",
        [CONFIG_THRESHOLD_HIGH_C] = "threshold_high_c",
        [CONFIG_WARN_PERCENT] = "warn_percent",
        [CONFIG_STATS] = "stats",
        [CONFIG_POINT] = "point",
        [CONFIG_LINE] = "line",
        [CONFIG_AVERAGE_3X3] = "average_3x3",
        [CONFIG_ROI] = "roi",
        [CONFIG_ROTATE] = "rotate",
        [CONFIG_MIRROR] = "mirror",
        [CONFIG_FLIP] = "flip",
        [CONFIG_DEVICE_STATE] = "device_state",
        [CONFIG_SCORE_THRESHOLD] = "score_threshold",
        [CONFIG_CLASS_INDEX] = "class_index",
        [CONFIG_DEVICE] = "device",
        [CONFIG_JPEG_QUALITY] = "jpeg_quality"
    };
    const unsigned fields = sizeof(keys) / sizeof(keys[0]);
    unsigned field = 0;
    while (field < fields && strcmp(key, keys[field])) ++field;
    if (field == fields || (parser->seen & (UINT64_C(1) << field))) return false;
    parser->seen |= UINT64_C(1) << field;
    if (field == CONFIG_DEVICE) {
        /* The index <= strlen bound also copies the terminator; control characters are refused. */
        if (strncmp(value, "/dev/", 5) || strlen(value) >= sizeof(configured_device)) return false;
        for (size_t index = 0; index <= strlen(value); ++index) {
            if (((unsigned char)value[index] < 32 && value[index]) || value[index] == 127) return false;
            configured_device[index] = value[index];
        }
        options->device_path = configured_device;
        return true;
    }
    if (field == CONFIG_FORMAT) return !strcmp(value, "LWIR_CONFIG_V1");
    if (field == CONFIG_SCORE_THRESHOLD)
        return ini_real(value, &options->score_threshold) && options->score_threshold >= 0 && options->score_threshold <= 1;
    if (field == CONFIG_CLASS_INDEX) return parse_unsigned(value, 4095, &options->class_index);
    /* Relies on SERIAL, GAIN_LABEL and CORRECTIONS being contiguous in the enum. */
    if (field >= CONFIG_SERIAL && field <= CONFIG_CORRECTIONS) {
        char *destination = field == CONFIG_SERIAL ? configured_serial : field == CONFIG_GAIN_LABEL ? configured_gain : configured_corrections;
        const size_t capacity = field == CONFIG_SERIAL ? sizeof(configured_serial) : field == CONFIG_GAIN_LABEL ? sizeof(configured_gain) : sizeof(configured_corrections);
        if (strlen(value) >= capacity) return false;
        for (size_t index = 0; index <= strlen(value); ++index) { if (((unsigned char)value[index] < 32 && value[index]) || value[index] == 127) return false; destination[index] = value[index]; }
        if (field == CONFIG_SERIAL) options->serial = destination;
        else if (field == CONFIG_GAIN_LABEL) options->gain_label = destination;
        else options->corrections = destination;
        return true;
    }
    if (field == CONFIG_PALETTE) {
        if (!strcmp(value, "white-hot")) options->render.palette = LWIR_WHITE_HOT;
        else if (!strcmp(value, "black-hot")) options->render.palette = LWIR_BLACK_HOT;
        else if (!strcmp(value, "iron")) options->render.palette = LWIR_IRON;
        else return false;
        return true;
    }
    if (field == CONFIG_POINT || field == CONFIG_LINE || field == CONFIG_ROI) {
        unsigned coordinates[4];
        return parse_coordinates(value, field == CONFIG_POINT ? 2 : 4, coordinates) &&
            select_measurement(options, field == CONFIG_POINT ? LWIR_POINT : field == CONFIG_LINE ? LWIR_LINE : LWIR_RECTANGLE, coordinates);
    }
    if (field == CONFIG_WINDOW_LOW_C || field == CONFIG_WINDOW_HIGH_C || field == CONFIG_DENOISE_K ||
        field == CONFIG_THRESHOLD_LOW_C || field == CONFIG_THRESHOLD_HIGH_C) {
        double number;
        if (!ini_real(value, &number)) return false;
        if (field == CONFIG_WINDOW_LOW_C) parser->low_c = number;
        else if (field == CONFIG_WINDOW_HIGH_C) parser->high_c = number;
        else if (field == CONFIG_THRESHOLD_LOW_C) parser->threshold_low_c = number;
        else if (field == CONFIG_THRESHOLD_HIGH_C) parser->threshold_high_c = number;
        /* Stored in raw-code units (x64, like temperatures); must not round down to 0. */
        else { if (number <= 0 || number > 10) return false; options->render.denoise_raw = (unsigned)(number * 64 + 0.5); if (!options->render.denoise_raw) return false; }
        return true;
    }
    /* Remaining keys are unsigned; 120000 is the shared ceiling, tightened per key below. */
    unsigned number;
    if (!parse_unsigned(value, 120000, &number)) return false;
    switch (field) {
    case CONFIG_WIDTH: options->width = number; break;
    case CONFIG_HEIGHT: options->height = number; break;
    case CONFIG_WARMUP_MS: options->warmup_ms = number; break;
    case CONFIG_TIMEOUT_MS: options->timeout_ms = number; break;
    case CONFIG_JPEG_QUALITY: if (!number || number > 100) return false; options->render.jpeg_quality = number; break;
    case CONFIG_PERCENTILE_LOW: options->render.low_percentile = number; break;
    case CONFIG_PERCENTILE_HIGH: options->render.high_percentile = number; break;
    case CONFIG_LEGEND: if (number > 1) return false; options->render.legend = number != 0; break;
    case CONFIG_SCENE_CHECK: if (number > 1) return false; options->scene_check = number != 0; break;
    case CONFIG_SETTLE: if (number > 1) return false; options->settle = number != 0; break;
    case CONFIG_WARN_PERCENT: if (!number || number > 100) return false; options->warn_percent = number; options->warn_percent_set = true; break;
    case CONFIG_STATS: if (number > 1) return false; options->report_stats |= number != 0; break;
    case CONFIG_AVERAGE_3X3: if (number > 1) return false; options->measurement.average_3x3 = number != 0; options->report_stats |= number != 0; break;
    case CONFIG_ROTATE: if (number > 270 || number % 90) return false; options->render.rotation = number; break;
    case CONFIG_MIRROR: if (number > 1) return false; options->render.mirror = number != 0; break;
    case CONFIG_FLIP: if (number > 1) return false; options->render.flip = number != 0; break;
    case CONFIG_DEVICE_STATE: if (number > 1) return false; options->device_state = number != 0; break;
    default: return false;
    }
    return true;
}

/**
 * @brief Load the settings INI into @p options before command-line parsing.
 *
 * @details Resets ::model_settings to the native backend, lets the shared
 * inference-settings reader dispatch each key to config_field(), then applies
 * cross-key rules: `format = LWIR_CONFIG_V1` is mandatory, thresholds and
 * window values must come in complete pairs, and a fixed window excludes
 * percentiles. A configured size must form a valid frame geometry.
 *
 * @param path Settings file path.
 * @param[in,out] options Defaults on entry; INI values on return.
 * @return true when the file is readable and every key is valid; on false the
 *         caller exits with code 2. The reader's error text is not printed here.
 * @note The pair masks use 32-bit constants; this is sufficient only because
 *       every identity they test is below 32 (::CONFIG_JPEG_QUALITY is not).
 */
static bool load_config(const char *path, struct capture_options *options)
{
    char error[1024];
    inference_settings_init(&model_settings, INFERENCE_NATIVE);
    struct config_parser parser = {.options = options};
    if (!inference_settings_read(&model_settings, path, config_field, &parser, error, sizeof(error)) ||
        !(parser.seen & (UINT32_C(1) << CONFIG_FORMAT))) return false;
    const uint32_t window = (UINT32_C(1) << CONFIG_WINDOW_LOW_C) | (UINT32_C(1) << CONFIG_WINDOW_HIGH_C);
    const uint32_t percentiles = (UINT32_C(1) << CONFIG_PERCENTILE_LOW) | (UINT32_C(1) << CONFIG_PERCENTILE_HIGH);
    const uint32_t thresholds = (UINT32_C(1) << CONFIG_THRESHOLD_LOW_C) | (UINT32_C(1) << CONFIG_THRESHOLD_HIGH_C);
    if (parser.seen & thresholds) {
        if ((parser.seen & thresholds) != thresholds ||
            !temperature_band(parser.threshold_low_c, parser.threshold_high_c, &options->thresholds.low, &options->thresholds.high)) return false;
        options->thresholds.enabled = true; options->report_stats = true;
    }
    if ((parser.seen & window) && (parser.seen & percentiles)) return false;
    if (parser.seen & window) {
        if ((parser.seen & window) != window) return false;
        const double low = (parser.low_c + 273.15) * 64, high = (parser.high_c + 273.15) * 64;
        if (low < 0 || high > 65535 || low >= high) return false;
        options->render.low = (int32_t)(low + 0.5); options->render.high = (int32_t)(high + 0.5);
        if (options->render.low >= options->render.high) return false;
        options->render.view = LWIR_FIXED;
    } else if (parser.seen & percentiles) {
        if ((parser.seen & percentiles) != percentiles || options->render.low_percentile >= options->render.high_percentile || options->render.high_percentile > 100) return false;
        options->render.view = LWIR_PERCENTILE;
    }
    return (!options->width && !options->height) || lwir_frame_bytes(options->width, options->height) != 0;
}

/**
 * @brief SIGINT/SIGTERM handler: record the signal for cooperative cancellation.
 *
 * @details Only stores into a sig_atomic_t; capture loops poll ::interrupted
 * (and the device layer receives it as its cancel flag) so cleanup runs in
 * normal context.
 *
 * @param signal_number Delivered signal, later reported as exit code 128+signal.
 */
static void interrupt_capture(int signal_number)
{
    interrupted = signal_number;
}

/**
 * @brief Print the build version and command-line synopsis.
 *
 * @details Lines for optional features are compiled in only when the matching
 * build profile is enabled, so help never advertises unavailable options.
 *
 * @param output stdout for `--help`, stderr after a usage error.
 */
static void usage(FILE *output)
{
    fprintf(output, "%s\nUsage: lwircam [--capture] [--output FILE] [--serial SERIAL] [--size WIDTH HEIGHT]\n",
                    LWIRCAM_BUILD_VERSION);
    fprintf(output, "Model settings: --settings FILE (--config alias); analysis builds: --check-model or --analyze FILE.jpg\n");
#if LWIR_ENABLE_ANALYSIS
    fprintf(output, "                     [--calibration FILE] [--geolocate FILE]\n");
#endif
#if LWIR_ENABLE_MOCK
    fprintf(output, "                     [--mock-image FILE | --mock-layer FILE]\n");
#endif
    fprintf(output, "                     [--warmup-ms MS] [--timeout-ms MS]\n"
                    "                     [--device /dev/videoN]\n"
                    "                     [--jpeg-quality 1..100] (capture default: 95; sampling: 4:4:4)\n"
                    "                     [--stats] [--roi X Y WIDTH HEIGHT] OUTPUT.jpg\n"
                    "                     [--point X Y | --line X0 Y0 X1 Y1] [--average-3x3]\n"
                    "                     [--thresholds LOW_C HIGH_C] [--warn-percent 1..100]\n"
                    "                     [--rotate 0|90|180|270] [--mirror] [--flip]\n"
                    "                     [--window LOW_C HIGH_C | --percentiles LOW HIGH]\n"
                    "                     [--palette white-hot|black-hot|iron] [--legend]\n"
                    "                     [--corrections FILE] [--denoise-k K] [--gain-label LABEL]\n"
                    "                     [--replay-serial SERIAL] [--bare]\n"
                    "                     [--record-raw FILE | --native-raw] [--config FILE]\n"
                    "                     [--device-state]\n"
                    "                     [--scene-check] [--settle] [--capture-server | --warmup]\n"
                    "       lwircam --raw FRAME.yuyt --size WIDTH HEIGHT OUTPUT.jpg\n"
                    "WIDTH and HEIGHT describe the image/temperature planes, not the doubled UVC height.\n"
                    "Tiny1C 0bda:5840: combined image/temperature; TDK 5474:5411: native 640x512 image only.\n"
                    "TDK supports JPEG and orientation, not thermal measurements/rendering/raw/state controls.\n"
                    "Statistics report device values to stderr, not calibrated surface temperatures.\n"
                    "All measurement coordinates are native; coverage uses original centers, excluding 0/65535.\n"
                    "Thresholds are an inclusive diagnostic band, not a display window or gain command.\n"
                    "Orientation is clockwise rotate, then mirror, then flip; raw thermal bytes stay native.\n"
                    "Device-state queries are opt-in, read-only, bounded to 250 ms and bench-unverified.\n"
                    "Thermal replay is marked LWIRSIM1; Tiny1C temperatures use LWIRRAW1; TDK has no thermal markers.\n");
    fprintf(output, "Standalone Tiny1C controls (one per invocation, no output image):\n"
                    "       lwircam --get-control gain|emissivity-128|agc-mode|agc\n"
                    "       lwircam [--device /dev/videoN | --serial SERIAL] --set-gain low|high\n"
                    "       lwircam --set-emissivity-128 1..128 | --set-agc-mode 0..5 | --set-agc 0|1 | --ffc\n"
                    "Setters require matching readback; FFC completion is not calibration validation. Never auto-retry.\n"
                    "Output: LWIR_CONTROL_OK control=NAME operation=read|write|trigger ...; failures report submitted=0|1.\n");
}

/**
 * @brief Strictly parse a decimal unsigned integer with an upper bound.
 *
 * @details Pre-scans for digits only, because strtoul() would otherwise accept
 * leading whitespace, a sign (wrapping "-1" to a huge value) or a `0x` prefix.
 *
 * @param text NUL-terminated input; NULL or empty is rejected.
 * @param maximum Largest accepted value.
 * @param[out] value Written only on success.
 * @return true for a pure digit string within range.
 */
static bool parse_unsigned(const char *text, unsigned maximum, unsigned *value)
{
    if (text == NULL || text[0] == '\0') return false;
    for (const char *cursor = text; *cursor != '\0'; ++cursor) {
        if (*cursor < '0' || *cursor > '9') return false;
    }
    errno = 0;
    char *end = NULL;
    const unsigned long parsed = strtoul(text, &end, 10);
    if (errno != 0 || end == NULL || *end != '\0' || parsed > maximum) return false;
    *value = (unsigned)parsed;
    return true;
}

/**
 * @brief Select the single point, line or rectangle measurement.
 *
 * @details The zero-initialised measurement (kind ::LWIR_RECTANGLE, width 0)
 * means "none chosen yet"; any prior selection makes a second one fail, so the
 * INI and command line together can name at most one measurement. Bounds
 * against the frame are checked later, once the geometry is known.
 *
 * @param[in,out] options Receives the measurement and enables statistics.
 * @param kind Point, line or rectangle.
 * @param coordinates `x y` for a point; `x0 y0 x1 y1` for a line; `x y w h` for a rectangle.
 * @return false if a measurement was already selected or a rectangle is empty.
 */
static bool select_measurement(struct capture_options *options, enum lwir_measurement_kind kind, const unsigned *coordinates)
{
    if (options->measurement.roi.width || options->measurement.kind != LWIR_RECTANGLE) return false;
    options->measurement.kind = kind;
    options->measurement.roi = (struct lwir_roi){.x = coordinates[0], .y = coordinates[1], .width = 1, .height = 1};
    if (kind == LWIR_RECTANGLE) {
        if (!coordinates[2] || !coordinates[3]) return false;
        options->measurement.roi.width = coordinates[2]; options->measurement.roi.height = coordinates[3];
    } else if (kind == LWIR_LINE) {
        options->measurement.end_x = coordinates[2]; options->measurement.end_y = coordinates[3];
    }
    options->report_stats = true;
    return true;
}

/**
 * @brief Parse an INI coordinate list such as `x,y` or `x,y,w,h`.
 *
 * @details Each field must start with a digit (blocking strtoul's sign and
 * whitespace acceptance) and be followed by exactly `,` or, for the last one,
 * the end of the string.
 *
 * @param text Comma-separated decimal values.
 * @param count Number of fields expected (2 or 4).
 * @param[out] coordinates At least @p count elements.
 * @return true when exactly @p count values, each <= LWIR_MAX_WIDTH, are present.
 */
static bool parse_coordinates(const char *text, unsigned count, unsigned *coordinates)
{
    for (unsigned index = 0; index < count; ++index) {
        if (*text < '0' || *text > '9') return false;
        errno = 0; char *end;
        const unsigned long value = strtoul(text, &end, 10);
        if (errno || value > LWIR_MAX_WIDTH || (index + 1 < count ? *end != ',' : *end != '\0')) return false;
        coordinates[index] = (unsigned)value;
        text = end + (index + 1 < count ? 1 : 0);
    }
    return true;
}

/**
 * @brief Apply command-line options on top of the INI settings and validate the mode.
 *
 * @details Three passes:
 * 1. Parse each option, rejecting repeats where a second value would be
 *    ambiguous, and feature options missing from this build profile.
 * 2. Reject any argument of 1024+ bytes or containing control characters, so
 *    paths and labels cannot inject lines into logs or metadata.
 * 3. Enforce mode rules: a standalone control admits only its selector,
 *    `--device`, `--serial` and `--timeout-ms`; analysis-only runs exclude
 *    capture; everything else must satisfy the mutual-exclusion expression at
 *    the end.
 *
 * @param count argc.
 * @param arguments argv; strings are referenced, not copied.
 * @param[in,out] options INI-derived values on entry; final policy on return.
 * @return false for any usage error; the caller prints usage and exits 2.
 * @note `--settings`/`--config` was already consumed by main(); here it only
 *       has to repeat the same path.
 */
static bool parse_options(int count, char **arguments, struct capture_options *options)
{
    for (int index = 1; index < count; ++index) {
        const char *argument = arguments[index];
        if ((!LWIR_ENABLE_ANALYSIS && (!strcmp(argument, "--geolocate") || !strcmp(argument, "--calibration"))) ||
            (!LWIR_ENABLE_MOCK && (!strcmp(argument, "--mock-image") || !strcmp(argument, "--mock-layer")))) {
            fprintf(stderr, "LWIR: option unavailable in this build profile: %s\n", argument);
            return false;
        }
        if (!strcmp(argument, "--get-control")) {
            if (options->control_selected || ++index >= count) return false;
            const char *name = arguments[index];
            if (strcmp(name, "gain") && strcmp(name, "emissivity-128") && strcmp(name, "agc-mode") && strcmp(name, "agc")) return false;
            options->control_selected = true; options->control_read = true;
            /* Placeholder value only keeps the request valid: emissivity must be 1..128 even for a read. */
            options->control = (struct lwir_control_request){.kind = !strcmp(name, "gain") ? LWIR_CONTROL_GAIN :
                !strcmp(name, "emissivity-128") ? LWIR_CONTROL_EMISSIVITY : !strcmp(name, "agc-mode") ? LWIR_CONTROL_AGC_MODE : LWIR_CONTROL_AGC_ENABLE,
                .value = !strcmp(name, "emissivity-128") ? 1 : 0};
        } else if (!strcmp(argument, "--set-gain") || !strcmp(argument, "--set-emissivity-128") ||
            !strcmp(argument, "--set-agc-mode") || !strcmp(argument, "--set-agc") || !strcmp(argument, "--ffc")) {
            if (options->control_selected) return false;
            options->control_selected = true;
            if (!strcmp(argument, "--ffc")) options->control = (struct lwir_control_request){.kind = LWIR_CONTROL_FFC};
            else {
                if (++index >= count) return false;
                if (!strcmp(argument, "--set-gain")) {
                    if (strcmp(arguments[index], "low") && strcmp(arguments[index], "high")) return false;
                    options->control = (struct lwir_control_request){.kind = LWIR_CONTROL_GAIN, .value = !strcmp(arguments[index], "high")};
                } else {
                    unsigned value;
                    if (!parse_unsigned(arguments[index], 128, &value)) return false;
                    options->control = (struct lwir_control_request){.kind = !strcmp(argument, "--set-emissivity-128") ? LWIR_CONTROL_EMISSIVITY :
                        !strcmp(argument, "--set-agc-mode") ? LWIR_CONTROL_AGC_MODE : LWIR_CONTROL_AGC_ENABLE, .value = (uint16_t)value};
                }
            }
            if (!lwir_control_valid(&options->control)) return false;
        } else if (strcmp(argument, "--config") == 0 || strcmp(argument, "--settings") == 0) {
            /* Already loaded by main(); only the identical path is accepted here. */
            if (++index >= count || !options->config || strcmp(options->config, arguments[index])) return false;
        } else if (!strcmp(argument, "--analyze")) {
            if (!LWIR_ENABLE_ANALYSIS || options->inference_input || ++index >= count) return false;
            options->inference_input = arguments[index];
        } else if (!strcmp(argument, "--check-model")) {
            if (!LWIR_ENABLE_ANALYSIS || options->check_model) return false;
            options->check_model = true;
        } else if (strcmp(argument, "--capture-server") == 0) {
            options->server = true;
        } else if (strcmp(argument, "--device-state") == 0) {
            options->device_state = true;
        } else if (!strcmp(argument, "--jpeg-quality")) {
            /* Repeats are rejected, but one CLI value may override the INI value. */
            if (options->jpeg_quality_selected || ++index >= count || !parse_unsigned(arguments[index], 100, &options->render.jpeg_quality)
                || !options->render.jpeg_quality) return false;
            options->jpeg_quality_selected = true;
        } else if (!strcmp(argument, "--device")) {
            if (options->device_path || ++index >= count || strncmp(arguments[index], "/dev/", 5)) return false;
            options->device_path = arguments[index];
        } else if (strcmp(argument, "--capture") == 0) {
            if (options->capture_requested) return false;
            options->capture_requested = true;
        } else if (strcmp(argument, "--mock-image") == 0 || strcmp(argument, "--mock-layer") == 0) {
            if (options->mock_input || ++index >= count || !arguments[index][0]) return false;
            options->mock_input = arguments[index];
            options->mock_layer_only = strcmp(argument, "--mock-layer") == 0;
        } else if (strcmp(argument, "--output") == 0 || strcmp(argument, "--calibration") == 0 ||
                   strcmp(argument, "--geolocate") == 0) {
            const char **destination = strcmp(argument, "--output") == 0 ? &options->output :
                strcmp(argument, "--calibration") == 0 ? &options->calibration : &options->geolocate_input;
            if (*destination || ++index >= count || !arguments[index][0]) return false;
            *destination = arguments[index];
        } else if (strcmp(argument, "--warmup") == 0) {
            options->warmup_only = true;
        } else if (strcmp(argument, "--native-raw") == 0) {
            options->native_raw = true;
        } else if (strcmp(argument, "--bare") == 0) {
            /* Accepted for compatibility; it has no effect in this file. */
            continue;
        } else if (strcmp(argument, "--raw") == 0 || strcmp(argument, "--serial") == 0 ||
            strcmp(argument, "--record-raw") == 0 || strcmp(argument, "--gain-label") == 0) {
            if (++index >= count || arguments[index][0] == '\0') return false;
            const char **destination = strcmp(argument, "--raw") == 0 ? &options->raw :
                strcmp(argument, "--serial") == 0 ? &options->serial :
                strcmp(argument, "--record-raw") == 0 ? &options->record_raw : &options->gain_label;
            if (*destination != NULL) return false;
            *destination = arguments[index];
        } else if (strcmp(argument, "--corrections") == 0 || strcmp(argument, "--replay-serial") == 0) {
            /* An INI value counts as already set, so these cannot be overridden from the CLI. */
            const char **destination = !strcmp(argument, "--corrections") ? &options->corrections : &options->replay_serial;
            if (*destination || ++index >= count || !arguments[index][0]) return false;
            *destination = arguments[index];
        } else if (strcmp(argument, "--denoise-k") == 0) {
            double threshold;
            if (++index >= count || !ini_real(arguments[index], &threshold) || threshold <= 0 || threshold > 10) return false;
            options->render.denoise_raw = (unsigned)(threshold * 64 + 0.5);
            if (!options->render.denoise_raw) return false;
        } else if (strcmp(argument, "--size") == 0) {
            if (options->width != 0 || index + 2 >= count ||
                !parse_unsigned(arguments[index + 1], LWIR_MAX_WIDTH, &options->width) ||
                !parse_unsigned(arguments[index + 2], LWIR_MAX_HEIGHT, &options->height) ||
                lwir_frame_bytes(options->width, options->height) == 0) return false;
            index += 2;
        } else if (strcmp(argument, "--warmup-ms") == 0 || strcmp(argument, "--timeout-ms") == 0) {
            if (++index >= count) return false;
            unsigned *destination = strcmp(argument, "--warmup-ms") == 0 ? &options->warmup_ms : &options->timeout_ms;
            if (!parse_unsigned(arguments[index], 120000, destination)) return false;
        } else if (strcmp(argument, "--window") == 0) {
            /* LWIR_SENSOR means no display view chosen yet; window and percentiles are exclusive. */
            if (options->render.view != LWIR_SENSOR || index + 2 >= count) return false;
            int32_t *limits[2] = {&options->render.low, &options->render.high};
            for (unsigned bound = 0; bound < 2; ++bound) {
                char *end;
                errno = 0;
                const double celsius = strtod(arguments[++index], &end);
                const double scaled = (celsius + 273.15) * 64;
                if (errno || end == arguments[index] || *end || !isfinite(scaled) || scaled < 0 || scaled > 65535) return false;
                *limits[bound] = (int32_t)(scaled + 0.5);
            }
            if (options->render.low >= options->render.high) return false;
            options->render.view = LWIR_FIXED;
        } else if (strcmp(argument, "--percentiles") == 0) {
            if (options->render.view != LWIR_SENSOR || index + 2 >= count ||
                !parse_unsigned(arguments[index + 1], 100, &options->render.low_percentile) ||
                !parse_unsigned(arguments[index + 2], 100, &options->render.high_percentile) ||
                options->render.low_percentile >= options->render.high_percentile) return false;
            options->render.view = LWIR_PERCENTILE;
            index += 2;
        } else if (strcmp(argument, "--palette") == 0) {
            if (++index >= count) return false;
            if (!strcmp(arguments[index], "white-hot")) options->render.palette = LWIR_WHITE_HOT;
            else if (!strcmp(arguments[index], "black-hot")) options->render.palette = LWIR_BLACK_HOT;
            else if (!strcmp(arguments[index], "iron")) options->render.palette = LWIR_IRON;
            else return false;
        } else if (strcmp(argument, "--legend") == 0) {
            options->render.legend = true;
        } else if (strcmp(argument, "--stats") == 0) {
            options->report_stats = true;
        } else if (strcmp(argument, "--rotate") == 0) {
            if (++index >= count || !parse_unsigned(arguments[index], 270, &options->render.rotation) || options->render.rotation % 90) return false;
        } else if (strcmp(argument, "--mirror") == 0) {
            options->render.mirror = true;
        } else if (strcmp(argument, "--flip") == 0) {
            options->render.flip = true;
        } else if (strcmp(argument, "--thresholds") == 0) {
            double low, high;
            if (options->thresholds.enabled || index + 2 >= count ||
                !ini_real(arguments[index + 1], &low) || !ini_real(arguments[index + 2], &high) ||
                !temperature_band(low, high, &options->thresholds.low, &options->thresholds.high)) return false;
            options->thresholds.enabled = true; options->report_stats = true; index += 2;
        } else if (strcmp(argument, "--warn-percent") == 0) {
            if (options->warn_percent_set || ++index >= count || !parse_unsigned(arguments[index], 100, &options->warn_percent) || !options->warn_percent) return false;
            options->warn_percent_set = true;
        } else if (strcmp(argument, "--average-3x3") == 0) {
            options->measurement.average_3x3 = true; options->report_stats = true;
        } else if (strcmp(argument, "--scene-check") == 0) {
            options->scene_check = true;
        } else if (strcmp(argument, "--settle") == 0) {
            options->settle = true;
        } else if (!strcmp(argument, "--roi") || !strcmp(argument, "--point") || !strcmp(argument, "--line")) {
            const enum lwir_measurement_kind kind = !strcmp(argument, "--point") ? LWIR_POINT : !strcmp(argument, "--line") ? LWIR_LINE : LWIR_RECTANGLE;
            const unsigned fields = kind == LWIR_POINT ? 2U : 4U;
            unsigned coordinates[4] = {0};
            if (index + (int)fields >= count) return false;
            for (unsigned field = 0; field < fields; ++field) {
                if (!parse_unsigned(arguments[++index], LWIR_MAX_WIDTH, &coordinates[field])) return false;
            }
            if (!select_measurement(options, kind, coordinates)) return false;
        } else {
            /* The only positional argument is the output path, accepted once. */
            if (argument[0] == '-' || argument[0] == '\0' || options->output != NULL) return false;
            options->output = argument;
        }
    }
    for (int index = 1; index < count; ++index) {
        if (strlen(arguments[index]) >= 1024) return false;
        for (const char *cursor = arguments[index]; *cursor; ++cursor) if ((unsigned char)*cursor < 32 || *cursor == 127) return false;
    }
    if (options->control_selected) {
        /* Allow-list: every token must be a control selector or one of its permitted companions; others abort. */
        for (int index = 1; index < count; ++index) {
            const char *argument = arguments[index];
            if (!strcmp(argument, "--ffc")) continue;
            if (!strcmp(argument, "--set-gain") || !strcmp(argument, "--set-emissivity-128") || !strcmp(argument, "--set-agc-mode") ||
                !strcmp(argument, "--set-agc") || !strcmp(argument, "--get-control") || !strcmp(argument, "--device") || !strcmp(argument, "--serial") || !strcmp(argument, "--timeout-ms")) {
                ++index;
            } else return false;
        }
        /* Controls need one fresh frame, not startup qualification. */
        options->warmup_ms = 0;
        return options->timeout_ms > 0 && (!options->serial || strlen(options->serial) < 64);
    }
    if (options->inference_input || options->check_model) {
        /* Offline model checks/analysis never touch the camera and need the ONNX backend from settings. */
        return LWIR_ENABLE_ANALYSIS && model_settings.backend == INFERENCE_ONNX && !options->output &&
            !options->server && !options->capture_requested && !options->raw && !options->geolocate_input &&
            !options->mock_input && !options->device_path && !(options->inference_input && options->check_model);
    }
    /*
     * Capture-mode rules, one conjunct each:
     * - some product is requested (output, server, warmup, geolocation or mock);
     * - mock runs exclude camera/replay/raw modes, and --mock-layer has no separate output;
     * - --capture requires an output, and the deadline must exceed warmup;
     * - geolocation, warmup and server are mutually exclusive with each other and
     *   with a fixed output or raw companion (the server derives per-request paths);
     * - --native-raw and --record-raw are alternative raw companion sources;
     * - the sensor view has no palette/legend, and corrections/denoise need a thermal view;
     * - replay serial applies only to replay, and labels/serials fit their buffers;
     * - a raw companion may not alias the output or the replay input;
     * - replay needs an explicit size and cannot select a camera serial;
     * - device state and device path need a live camera;
     * - --warn-percent only applies with thresholds;
     * - a known geometry must contain the measurement (else the device validates it later).
     */
    return (options->output != NULL || options->server || options->warmup_only || options->geolocate_input != NULL || options->mock_input != NULL) &&
            (!options->mock_input || (!options->capture_requested && !options->server && !options->warmup_only &&
                !options->raw && !options->geolocate_input && !options->record_raw && !options->native_raw &&
                (!options->mock_layer_only || !options->output))) &&
            (!options->capture_requested || options->output != NULL) && options->timeout_ms > options->warmup_ms &&
            (!options->geolocate_input || (!options->output && !options->server && !options->warmup_only && !options->native_raw && !options->record_raw)) &&
            (!options->warmup_only || (!options->output && !options->server && !options->record_raw && !options->native_raw)) &&
            (!options->server || (!options->output && !options->record_raw && !options->warmup_only)) &&
            (!options->native_raw || !options->record_raw) &&
            (options->render.view != LWIR_SENSOR || (!options->render.legend && options->render.palette == LWIR_WHITE_HOT)) &&
            (!options->replay_serial || (options->raw && strlen(options->replay_serial) < 64)) &&
            (!options->gain_label || strlen(options->gain_label) < 32) &&
            (!options->serial || strlen(options->serial) < 64) &&
            ((!options->corrections && !options->render.denoise_raw) || options->render.view != LWIR_SENSOR) &&
           (!options->record_raw || (options->output && strcmp(options->record_raw, options->output) &&
             (!options->raw || strcmp(options->record_raw, options->raw)))) &&
           (options->raw == NULL || (options->width != 0 && options->serial == NULL)) &&
           (!options->device_state || (!options->raw && !options->mock_input && !options->geolocate_input)) &&
           (!options->device_path || (!options->raw && !options->mock_input && !options->geolocate_input)) &&
           (!options->warn_percent_set || options->thresholds.enabled) &&
           (options->width == 0 || lwir_measurement_valid(options->width, options->height, &options->measurement));
}

/**
 * @brief Read CLOCK_MONOTONIC in milliseconds.
 *
 * @details Monotonic time is immune to wall-clock steps (NTP/GPS), which would
 * otherwise falsely trigger or postpone deadlines.
 *
 * @param[out] milliseconds Current monotonic time.
 * @return false if clock_gettime() fails (errno set by it).
 */
static bool monotonic_ms(uint64_t *milliseconds)
{
    struct timespec now;
    if (clock_gettime(CLOCK_MONOTONIC, &now) != 0) return false;
    *milliseconds = (uint64_t)now.tv_sec * 1000 + (uint64_t)now.tv_nsec / 1000000;
    return true;
}

/**
 * @brief Load a replay frame into ::captured_frame.
 *
 * @details Opens with O_NOFOLLOW and O_NONBLOCK and requires a regular file of
 * exactly the expected size, so symlinks, FIFOs and devices cannot block or
 * redirect the read. A trailing-byte probe catches files that grew after
 * fstat(). The frame is then structurally validated and, with `--scene-check`,
 * checked for plausibility.
 *
 * @param options Supplies the replay path and geometry.
 * @return true on success. On failure errno is EINVAL (wrong type/size/content),
 *         EIO (read or close error) or the open()/fdopen() error.
 */
static bool read_raw(const struct capture_options *options)
{
    const size_t bytes = lwir_frame_bytes(options->width, options->height);
    const int descriptor = open(options->raw, O_RDONLY | O_NONBLOCK | O_CLOEXEC | O_NOFOLLOW);
    if (descriptor < 0) return false;
    struct stat status;
    if (fstat(descriptor, &status) != 0 || !S_ISREG(status.st_mode) || status.st_size != (off_t)bytes) {
        close(descriptor);
        errno = EINVAL;
        return false;
    }
    FILE *input = fdopen(descriptor, "rb");
    if (input == NULL) { const int saved_errno = errno; close(descriptor); errno = saved_errno; return false; }
    if (input == NULL) return false;
    const size_t received = fread(captured_frame, 1, bytes, input);
    const int extra = received == bytes && !ferror(input) && !feof(input) ? fgetc(input) : EOF;
    const bool io_error = ferror(input) != 0;
    const int close_result = fclose(input);
    if (io_error || close_result != 0) {
        errno = EIO;
        return false;
    }
    if (received != bytes || extra != EOF ||
        !lwir_frame_valid(captured_frame, bytes, options->width, options->height) ||
        (options->scene_check && !lwir_frame_plausible(captured_frame, bytes, options->width, options->height))) {
        errno = EINVAL;
        return false;
    }
    return true;
}

/**
 * @brief Translate capture options into the device-layer acquisition policy.
 *
 * @details `requires_thermal` is true whenever any requested feature needs the
 * temperature plane, letting the device layer refuse image-only cameras
 * up front instead of producing incomplete evidence.
 *
 * @param options Current policy; pointers inside the result alias it.
 * @return Device options by value; ::interrupted is passed as the cancel flag.
 */
static struct lwir_device_options device_policy(const struct capture_options *options)
{
    return (struct lwir_device_options){.serial = options->serial, .device_path = options->device_path,
        .width = options->width, .height = options->height,
        .warmup_ms = options->warmup_ms, .timeout_ms = options->timeout_ms,
        .scene_check = options->scene_check, .settle = options->settle,
        .measurement = &options->measurement, .cancelled = &interrupted, .layout = options->layout,
        .requires_thermal = options->control_selected || options->report_stats || options->thresholds.enabled || options->render.view != LWIR_SENSOR ||
            options->corrections || options->render.denoise_raw || options->device_state || options->record_raw ||
            options->native_raw || options->calibration || options->gain_label || options->scene_check};
}

/**
 * @brief Open the camera and adopt its negotiated geometry, layout and serial.
 *
 * @details The serial is copied into static storage so the evidence pointer
 * remains valid after the device session is closed.
 *
 * @param[in,out] options Width, height and layout are updated from the device.
 * @param[out] device Session to close with lwir_device_close().
 * @return false on open failure, errno set by the device layer.
 */
static bool open_camera(struct capture_options *options, struct lwir_device *device)
{
    struct lwir_device_options policy = device_policy(options);
    if (!lwir_device_open(&policy, device)) return false;
    options->width = policy.width;
    options->height = policy.height;
    options->layout = device->layout;
    for (size_t index = 0; index < sizeof(device_serial); ++index) device_serial[index] = device->serial[index];
    capture_evidence.serial = device_serial[0] ? device_serial : "unknown";
    return true;
}

/**
 * @brief Acquire one frame into ::captured_frame and record its timing evidence.
 *
 * @details The device layer applies warmup, qualification and the deadline,
 * and rejects frames timestamped before the acquisition started. Because that
 * start follows the request time stamped by initialize_evidence(), photos are
 * never older than the request.
 *
 * @param device Open session.
 * @param options Acquisition policy.
 * @return false on timeout, cancellation or invalid frame, errno set by the device layer.
 */
static bool acquire_frame(struct lwir_device *device, const struct capture_options *options)
{
    const struct lwir_device_options policy = device_policy(options);
    struct lwir_device_observation observation;
    if (!lwir_device_acquire(device, &policy, captured_frame, sizeof(captured_frame), &observation)) return false;
    capture_evidence.capture.sequence = observation.sequence;
    capture_evidence.arrival_us = observation.arrival_us;
    capture_evidence.kernel_monotonic_us = observation.kernel_monotonic_us;
    capture_evidence.kernel_timestamp_flags = observation.kernel_timestamp_flags;
    return true;
}

/**
 * @brief Optionally attach read-only device state to the evidence.
 *
 * @details Runs only with `--device-state`, after the frame, so the state is
 * not frame-atomic. The device layer bounds the query to 250 ms. State is
 * best effort: failure never fails the capture, and errno is preserved so a
 * later failure report is not masked by a state-query error.
 *
 * @param device Open session.
 * @param options Supplies the device_state flag.
 */
static void read_device_state(struct lwir_device *device, const struct capture_options *options)
{
    if (!options->device_state) return;
    const int saved_errno = errno;
    lwir_device_read_state(device, &captured_device_state);
    capture_evidence.device_state = &captured_device_state;
    if (lwir_state_format(&captured_device_state, device_state_text, sizeof(device_state_text)))
        fprintf(stderr, "LWIR: read-only device state (after frame; not exposure/FFC history)\n%s", device_state_text);
    errno = saved_errno;
}

/**
 * @brief One-shot camera capture: open, acquire, optional state, close.
 *
 * @details The session is closed before publication so encoding and fsync do
 * not hold the camera.
 *
 * @param[in,out] options Geometry and layout are filled from the device.
 * @return true when a frame is in ::captured_frame; errno describes failures.
 */
static bool capture_camera(struct capture_options *options)
{
    struct lwir_device session = {0};
    if (!open_camera(options, &session)) return false;
    const bool acquired = acquire_frame(&session, options);
    if (acquired) read_device_state(&session, options);
    lwir_device_close(&session);
    return acquired;
}

/**
 * @brief Measure the selected region, print statistics and attach them to the evidence.
 *
 * @details Values are device-reported temperatures (raw / 64 - 273.15), not
 * calibrated surface temperatures. Standard deviation uses the single-pass
 * moment formula, clamped at zero against rounding. Threshold coverage
 * excludes endpoint codes (0/65535), and the range warning is purely
 * diagnostic: it never changes camera settings.
 *
 * @param options Measurement, thresholds and warning percentage.
 * @return false with errno EINVAL if the measurement cannot be computed.
 */
static bool report_statistics(const struct capture_options *options)
{
    struct lwir_temperature_stats *statistics = &captured_statistics;
    if (!lwir_measure_temperature(captured_frame, lwir_frame_bytes(options->width, options->height),
                                  options->width, options->height, &options->measurement, &options->thresholds, statistics)) {
        errno = EINVAL;
        return false;
    }
    const double mean_raw = (double)statistics->sum_raw / statistics->pixels;
    const double variance = (double)statistics->sum_squared_raw / statistics->pixels - mean_raw * mean_raw;
    const double deviation = sqrt(variance > 0 ? variance : 0) / 64.0;
    fprintf(stderr, "LWIR: %s %s (%zu native centers, %s): reported C min=%.3f at %u,%u max=%.3f at %u,%u mean=%.3f"
                    " spatial-stdev=%.3f endpoint-codes=%zu (calibration unverified)\n",
            options->raw ? "replay" : "camera", options->measurement.kind == LWIR_POINT ? "point" : options->measurement.kind == LWIR_LINE ? "line" : "ROI",
            statistics->pixels, options->measurement.average_3x3 ? "3x3 averaged" : "original pixels",
            statistics->min_raw / 64.0 - 273.15, statistics->min_x, statistics->min_y,
            statistics->max_raw / 64.0 - 273.15, statistics->max_x, statistics->max_y,
            mean_raw / 64.0 - 273.15, deviation, statistics->endpoint_codes);
    if (options->thresholds.enabled) {
        const size_t valid = statistics->pixels - statistics->endpoint_codes;
        const double fraction = valid ? 100.0 / valid : 0;
        fprintf(stderr, "LWIR: raw coverage [%.6f, %.6f] C valid=%zu below=%zu (%.3f%%) within=%zu (%.3f%%) above=%zu (%.3f%%) status=%s\n",
            options->thresholds.low / 64.0 - 273.15, options->thresholds.high / 64.0 - 273.15, valid,
            statistics->below, statistics->below * fraction, statistics->within, statistics->within * fraction,
            statistics->above, statistics->above * fraction, valid ? "available" : "unavailable");
        /* Inclusive warning percentage, calculated exactly in integers; never change camera settings. */
        if (valid && (statistics->below * 100 >= valid * options->warn_percent || statistics->above * 100 >= valid * options->warn_percent)) {
            fprintf(stderr, "LWIR: RANGE WARNING below=%u above=%u limit=%u%% (diagnostic only; gain/saturation/calibration unverified)\n",
                statistics->below * 100 >= valid * options->warn_percent ? 1U : 0U,
                statistics->above * 100 >= valid * options->warn_percent ? 1U : 0U, options->warn_percent);
        }
    }
    capture_evidence.measurement = &options->measurement;
    capture_evidence.thresholds = &options->thresholds;
    capture_evidence.statistics = statistics;
    capture_evidence.warn_percent = options->warn_percent;
    return true;
}

/**
 * @brief Reset the evidence record and stamp the request start time.
 *
 * @details Called before acquisition: `request_us` is the freshness floor for
 * the frame. Exposure and frame duration are -1 and the time error is
 * UINT64_MAX because this camera path does not measure them; they must read as
 * unknown rather than zero.
 *
 * @param options Supplies the gain label, replay serial, view and raw path.
 * @return false if the monotonic clock is unavailable.
 */
static bool initialize_evidence(const struct capture_options *options)
{
    capture_evidence = (struct lwir_evidence){0};
    usher_boot_id(capture_evidence.capture.boot_id);
    capture_evidence.capture.process_id = (uint32_t)getpid();
    capture_evidence.capture.exposure_us = -1;
    capture_evidence.capture.frame_duration_us = -1;
    capture_evidence.capture.time_error_us = UINT64_MAX;
    capture_evidence.gain = options->gain_label;
    capture_evidence.serial = options->raw ? options->replay_serial : device_serial[0] ? device_serial : "unknown";
    capture_evidence.rendering = options->render.view == LWIR_SENSOR ? "sensor" : "thermal";
    capture_evidence.raw_path = options->record_raw;
    uint64_t requested;
    if (!monotonic_ms(&requested)) return false;
    capture_evidence.request_us = requested * 1000;
    return true;
}

/**
 * @brief Validate, hash and durably publish ::captured_frame.
 *
 * @details Refuses publication when a correction profile disagrees with the
 * device gain readback or with the serial/gain label/geometry, because
 * applying the wrong profile would silently corrupt temperatures. Then hashes
 * the frame, optionally reports statistics, writes the raw companion and the
 * JPEG. Each writer uses the no-overwrite publication of the image library.
 * In analysis builds an ONNX failure is only reported: the photograph stays.
 *
 * @param[in,out] options Output paths and rendering policy.
 * @return true when the JPEG (and requested companion) were published.
 *         false with errno EINVAL for profile conflicts, or the writer's errno.
 * @details Publication is all-or-nothing: when the JPEG cannot be published
 * after the raw companion was written, that companion is removed again, so a
 * failed request never leaves an orphaned raw file or blocks a later request
 * for the same name (link() refuses existing names).
 */
static bool publish_frame(struct capture_options *options)
{
    uint16_t selected_gain;
    if (options->render.profile && options->gain_label &&
        lwir_state_u16(capture_evidence.device_state, LWIR_STATE_GAIN, &selected_gain) &&
        ((!strcmp(options->gain_label, "high") && selected_gain != 1) ||
         (!strcmp(options->gain_label, "low") && selected_gain != 0))) {
        fprintf(stderr, "LWIR: device gain selection conflicts with correction profile label; no publication\n");
        errno = EINVAL; return false;
    }
    if (options->render.profile && !lwir_profile_matches(options->render.profile, options->width, options->height,
                                                         capture_evidence.serial, options->gain_label)) {
        fprintf(stderr, "LWIR: correction profile serial/gain/geometry mismatch; no publication\n");
        errno = EINVAL; return false;
    }
    capture_evidence.capture.width = options->width;
    capture_evidence.capture.height = options->height;
    capture_evidence.layout = options->layout;
    /* Image-only cameras deliver half of the combined frame size (no temperature plane). */
    const size_t bytes = lwir_frame_bytes(options->width, options->height) / (options->layout == LWIR_YUYV_IMAGE ? 2U : 1U);
    sha256_hex(captured_frame, bytes, capture_evidence.frame_sha256);
    const bool prepared = !interrupted && (!options->report_stats || report_statistics(options));
    const bool raw_published = prepared && options->record_raw && lwir_write_raw(options->record_raw, captured_frame,
        lwir_frame_bytes(options->width, options->height), options->width, options->height);
    const bool published = prepared && (!options->record_raw || raw_published) &&
        lwir_write_jpeg_rendered(options->output, captured_frame, bytes,
                                  options->width, options->height, options->raw != NULL, &capture_evidence, &options->render);
    if (!published && raw_published) {
        const int saved_errno = errno;
        if (!lwir_remove_published(options->record_raw))
            fprintf(stderr, "LWIR: JPEG failed and raw companion %s could not be removed: %s\n", options->record_raw, strerror(errno));
        errno = saved_errno;
    }
#if LWIR_ENABLE_ANALYSIS
    if (published && model_settings.backend == INFERENCE_ONNX) {
        char error[2048];
        if (!lwir_inference_image(options->output, capture_evidence.request_id, error, sizeof(error)))
            fprintf(stderr, "LWIR: analysis_failed; photograph retained: %s\n", error);
    }
#endif
    return published;
}

/**
 * @brief Open the configured inference backend, if any.
 *
 * @details Opening before the camera means a bad model fails fast as a
 * configuration error instead of after a capture.
 *
 * @param options Score threshold and class index for ONNX.
 * @return true for the native backend or a successfully opened ONNX model;
 *         false (callers exit 2) otherwise, with the reason on stderr.
 */
static bool begin_inference(const struct capture_options *options)
{
    if (model_settings.backend != INFERENCE_ONNX) return model_settings.backend == INFERENCE_NATIVE;
#if LWIR_ENABLE_ANALYSIS
    char error[2048];
    if (lwir_inference_open(&model_settings, options->score_threshold, options->class_index, error, sizeof(error))) return true;
    fprintf(stderr, "LWIR: %s\n", error);
#else
    (void)options;
    fprintf(stderr, "LWIR: ONNX requires an analysis-enabled build\n");
#endif
    return false;
}

/**
 * @brief Child-process body of one-shot mode: run a control or a capture.
 *
 * @details For a standalone control, a fresh valid frame is acquired first,
 * because the camera's control mailbox is not reliably responsive right after
 * stream start. Writes are submitted once and never replayed; setters are
 * verified by readback, while FFC only reports acknowledgement. Failure lines
 * report `submitted=0|1` so the operator knows whether the camera may have
 * changed. For a capture, the frame is acquired (camera or replay) and
 * published; the output path is printed on success.
 *
 * @param[in,out] options Run policy.
 * @return 0 success; 1 runtime failure; 2 inference configuration error;
 *         128+signal when interrupted.
 */
static int save_capture(struct capture_options *options)
{
    if (options->control_selected) {
        struct lwir_device device = {0};
        if (!open_camera(options, &device) || !acquire_frame(&device, options)) {
            lwir_device_close(&device);
            fprintf(stderr, "LWIR: control setup failed: %s\n", strerror(errno));
            return 1;
        }
        static const char *const names[] = {"gain", "emissivity-128", "agc-mode", "agc", "ffc"};
        /* Indexed by reply status; out-of-range values print "unknown" below. */
        static const char *const statuses[] = {"not_requested", "ok", "unsupported", "timeout", "io_error",
            "invalid", "busy", "skipped", "cancelled"};
        const char *name = names[options->control.kind];
        struct lwir_control_result result;
        const bool success = options->control_read ? lwir_device_control_read(&device, options->control.kind, &result) :
            lwir_device_control(&device, &options->control, &result);
        lwir_device_close(&device);
        if (!success) {
            const unsigned status = (unsigned)result.reply.status;
            fprintf(stderr, "LWIR: control %s failed status=%s transport=%d device=%u submitted=%u verified=%u; %s; never retry automatically\n",
                name, status < sizeof(statuses) / sizeof(statuses[0]) ? statuses[status] : "unknown",
                result.reply.transport_error, result.reply.device_status, result.submitted ? 1U : 0U, result.verified ? 1U : 0U,
                result.submitted ? "camera state may have changed" : "no command submitted");
            return 1;
        }
        if (options->control_read) printf("LWIR_CONTROL_OK control=%s operation=read value=%u\n", name, result.readback);
        else if (options->control.kind == LWIR_CONTROL_FFC) printf("LWIR_CONTROL_OK control=ffc operation=trigger acknowledged=1 verified=0\n");
        else printf("LWIR_CONTROL_OK control=%s operation=write value=%u readback=%u verified=1\n", name,
            options->control.value, result.readback);
        return fflush(stdout) == 0 ? 0 : 1;
    }
    if (!begin_inference(options)) return 2;
    if (!initialize_evidence(options)) return 1;
    errno = 0;
    const bool acquired = options->raw != NULL ? read_raw(options) : capture_camera(options);
    /* --warmup qualifies the camera only; nothing is published. */
    if (options->warmup_only && acquired && !interrupted) return 0;
    if (!acquired || !publish_frame(options)) {
        if (interrupted) errno = EINTR;
        fprintf(stderr, "LWIR: capture failed: %s\n", strerror(errno));
        return interrupted ? 128 + interrupted : 1;
    }
    puts(options->output);
    return fflush(stdout) == 0 ? 0 : 1;
}

/**
 * @brief Persistent `--capture-server` loop driven by USHER over stdin/stdout.
 *
 * @details Protocol:
 * - After the camera (or replay frame) is ready, prints `LWIR_SERVER_READY`.
 * - Each stdin line is an absolute output path, or `GEO:path` to geolocate an
 *   existing JPEG in analysis builds.
 * - A capture replies `LWIR_SERVER_TIMING request_us arrival_us` then
 *   `LWIR_SERVER_OK`; a geolocation replies `LWIR_SERVER_OK` only.
 * - Strictly one request at a time: bytes after the newline (pipelining),
 *   embedded NULs, control characters, relative or overlong paths are
 *   protocol errors (result 2).
 * - Any failure prints `LWIR_SERVER_ERROR` and ends the server. There is no
 *   automatic replay; USHER's supervisor must explicitly restart it.
 *
 * While idle, stdin is polled every 10 ms and camera frames are discarded so
 * the 4-buffer driver queue never hands a stale frame to the next request; if
 * no frame arrives for `timeout_ms` the server fails. Each request has an
 * alarm() hang watchdog of `timeout_ms` rounded up to seconds: SIGALRM is not
 * handled, so a wedged request terminates the process.
 *
 * `warmup_ms` is forced to 0 per request: startup qualification happens once
 * per USB power instance (tracked by the device layer's driver marker), never
 * per request.
 *
 * @param[in,out] options Policy; `output` and `record_raw` are rewritten per request.
 * @return 0 when stdin reaches EOF cleanly; 1 runtime failure; 2 inference
 *         configuration, correction-profile mismatch (before READY) or protocol
 *         error; 128+signal when interrupted.
 */
static int serve_captures(struct capture_options *options)
{
    if (!begin_inference(options)) return 2;
    /* Bounds camera startup; the default SIGALRM action kills a hung open. */
    alarm((options->timeout_ms + 999) / 1000);
    struct lwir_device session = {0};
    if (!options->raw && (!open_camera(options, &session) || !acquire_frame(&session, options))) {
        lwir_device_close(&session); return 1;
    }
    if (options->raw && !read_raw(options)) return 1;
    /* Same check publish_frame() applies per request, done once now: a profile that
     * can never match would otherwise pass startup and then fail every capture in flight. */
    const char *profile_serial = options->raw ? options->replay_serial : device_serial[0] ? device_serial : "unknown";
    if (options->render.profile && !lwir_profile_matches(options->render.profile, options->width, options->height,
                                                         profile_serial, options->gain_label)) {
        fprintf(stderr, "LWIR: correction profile does not match this camera's serial/gain label/geometry; not ready\n");
        lwir_device_close(&session); return 2;
    }
    const int flags = fcntl(STDIN_FILENO, F_GETFL);
    /* Non-blocking stdin lets the loop keep draining camera frames between requests. */
    if (flags < 0 || fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK) != 0) { lwir_device_close(&session); return 1; }
    uint64_t last_frame;
    if (!monotonic_ms(&last_frame)) { lwir_device_close(&session); return 1; }
    puts("LWIR_SERVER_READY");
    if (fflush(stdout)) { lwir_device_close(&session); return 1; }
    alarm(0);
    /* 1100 bytes: a 1023-byte path plus the optional "GEO:" prefix, newline and slack. */
    char command[1100], raw_path[1100];
    size_t used = 0;
    uint64_t last_request = 0, replay_sequence = 0;
    bool have_request = false;
    int result = 0;
    while (!interrupted) {
        struct pollfd input = {.fd = STDIN_FILENO, .events = POLLIN};
        const int ready = poll(&input, 1, 10);
        if (ready < 0 && errno == EINTR) continue;
        if (ready < 0 || (input.revents & (POLLERR | POLLNVAL))) { result = 1; break; }
        if (input.revents & (POLLIN | POLLHUP)) {
            /* Reserve one byte for the terminator so the buffer is always a valid string. */
            const ssize_t received = read(STDIN_FILENO, command + used, sizeof(command) - used - 1);
            if (received < 0 && (errno == EINTR || errno == EAGAIN)) continue;
            /* EOF means USHER closed the pipe: a clean shutdown, not an error. */
            if (received <= 0) { result = received < 0 ? 1 : 0; break; }
            /* The watchdog starts at the first byte of a request, covering slow partial lines too. */
            if (!used) alarm((options->timeout_ms + 999) / 1000);
            used += (size_t)received;
            command[used] = '\0';
            char *newline = memchr(command, '\n', used);
            /* Partial line: keep accumulating unless the buffer is already full. */
            if (!newline) { if (used + 1 == sizeof(command)) { result = 2; break; } continue; }
            /* The newline must be the last byte read: anything after it is a pipelined request. */
            if (newline != command + used - 1 || memchr(command, '\0', used)) { result = 2; break; }
            *newline = '\0';
            for (size_t index = 0; index + 1 < used; ++index) if ((unsigned char)command[index] < 32 || command[index] == 127) { result = 2; break; }
            if (result) break;
            const bool geolocate = strncmp(command, "GEO:", 4) == 0;
            char *output_path = geolocate ? command + 4 : command;
            /* Request ids are never reused: a capture after id UINT64_MAX is refused. */
            if (output_path[0] != '/' || strlen(output_path) >= 1024 ||
                (!geolocate && have_request && last_request == UINT64_MAX)) { result = 2; break; }
            for (size_t index = 0; command[index]; ++index) {
                /* Defensive repeat of the control-character check on the terminated string. */
                if ((unsigned char)command[index] < 32 || command[index] == 127) { result = 2; break; }
            }
            if (result) break;
            if (geolocate) {
#if LWIR_ENABLE_ANALYSIS
                if (lwir_geolocate_image(output_path, options->calibration) != 0) { result = 1; break; }
                puts("LWIR_SERVER_OK");
                if (fflush(stdout) != 0) { result = 1; break; }
                alarm(0);
                used = 0;
                continue;
#else
                fprintf(stderr, "LWIR: geographic analysis unavailable in this build profile\n");
                result = 2;
                break;
#endif
            }
            const uint64_t request = have_request ? last_request + 1 : 1;
            last_request = request; have_request = true;
            /* Startup qualification already happened once for this USB power instance. */
            options->output = command; options->warmup_ms = 0;
            options->record_raw = NULL;
            if (options->native_raw) {
                const int length = snprintf(raw_path, sizeof(raw_path), "%s.raw", command);
                if (length < 0 || (size_t)length >= sizeof(raw_path)) { result = 2; break; }
                options->record_raw = raw_path;
            }
            if (!initialize_evidence(options)) { result = 1; break; }
            capture_evidence.request_id = request;
            if (!options->raw) {
                bool discarded = false;
                /* Drop queued frames so the published frame was exposed after this request. */
                if (!lwir_device_discard(&session, &discarded) || !acquire_frame(&session, options)) { result = 1; break; }
                read_device_state(&session, options);
            } else {
                /* Replay re-reads the file per request and synthesises arrival time and sequence. */
                if (!read_raw(options) || !monotonic_ms(&last_frame)) { result = 1; break; }
                capture_evidence.arrival_us = last_frame * 1000;
                capture_evidence.capture.sequence = ++replay_sequence;
            }
            if (!publish_frame(options)) { result = 1; break; }
            printf("LWIR_SERVER_TIMING %llu %llu\nLWIR_SERVER_OK\n",
                (unsigned long long)capture_evidence.request_us,
                (unsigned long long)capture_evidence.arrival_us);
            if (fflush(stdout) || !monotonic_ms(&last_frame)) { result = 1; break; }
            alarm(0);
            used = 0;
        } else if (!options->raw) {
            /* Idle: keep the driver queue empty and detect a camera that stopped streaming. */
            bool discarded = false;
            uint64_t now;
            if (!lwir_device_discard(&session, &discarded) || !monotonic_ms(&now)) { result = 1; break; }
            if (discarded) last_frame = now;
            if (now - last_frame >= options->timeout_ms) { result = 1; break; }
        }
    }
    if (result) { puts("LWIR_SERVER_ERROR request failed; no automatic replay"); fflush(stdout); }
    /* Even closing the device may hang on USB; give it one second before SIGALRM kills the process. */
    alarm(1);
    lwir_device_close(&session);
    alarm(0);
    return interrupted ? 128 + interrupted : result;
}

/**
 * @brief Parent side of one-shot mode: wait for the capture child under a deadline.
 *
 * @details Polls waitpid() every 10 ms against a monotonic deadline. On timeout
 * or SIGINT/SIGTERM it sends SIGTERM, then SIGKILL after 250 ms. Publication may
 * already have happened, or be half-done, when the child is stopped, so the
 * outcome is reported as ambiguous and nothing is retried automatically. A
 * failing clock is treated as an expired deadline (fail safe).
 *
 * @param child Capture child process id.
 * @param timeout_ms Deadline from start of supervision.
 * @return The child's exit status; 1 if it died by a signal or waiting failed;
 *         124 on timeout; 128+signal when this process was interrupted.
 */
static int supervise(pid_t child, unsigned timeout_ms)
{
    uint64_t start = 0;
    const bool clock_valid = monotonic_ms(&start);
    bool terminating = false;
    uint64_t stop_started = 0;
    for (;;) {
        int status;
        const pid_t reaped = waitpid(child, &status, WNOHANG);
        if (reaped == child) {
            /* Exiting during termination still counts as cancelled, whatever its own status. */
            if (terminating) return interrupted ? 128 + interrupted : 124;
            return WIFEXITED(status) ? WEXITSTATUS(status) : 1;
        }
        if (reaped < 0 && errno != EINTR) return 1;
        uint64_t now = start;
        const bool now_valid = clock_valid && monotonic_ms(&now);
        const bool timed_out = !now_valid || now - start >= timeout_ms;
        if (!terminating && (interrupted || timed_out)) {
            terminating = true;
            stop_started = now;
            kill(child, SIGTERM);
            fprintf(stderr, "LWIR: lifecycle cancelled/timed out; publication may be ambiguous, never retry automatically\n");
        }
        if (terminating && (!now_valid || now - stop_started >= 250)) {
            kill(child, SIGKILL);
            do { } while (waitpid(child, &status, 0) < 0 && errno == EINTR);
            return interrupted ? 128 + interrupted : 124;
        }
        const struct timespec pause = {.tv_nsec = 10000000};
        if (nanosleep(&pause, NULL) != 0 && errno != EINTR) { kill(child, SIGKILL); waitpid(child, &status, 0); return 1; }
    }
}

/**
 * @brief Program entry: select the mode, load policy and run it.
 *
 * @details Order matters: `--features`, `--version` and `--help` short-circuit;
 * the settings file is located and loaded before parse_options() so CLI values
 * override INI values; analysis/mock modes return before the camera is
 * touched; the correction profile is loaded and an existing output is refused
 * before signal handlers are installed and the capture starts. Server mode runs
 * in-process; one-shot mode forks so the parent can enforce the deadline.
 *
 * @param count argc.
 * @param arguments argv.
 * @return 0 success; 1 runtime failure; 2 usage/configuration error;
 *         124 timeout; 128+signal when interrupted.
 */
int main(int count, char **arguments)
{
    if (count == 2 && !strcmp(arguments[1], "--features")) {
        printf("analysis=%d mock=%d jpeg=%s capture=v4l2\n", LWIR_ENABLE_ANALYSIS, LWIR_ENABLE_MOCK,
            lwir_jpeg_provider());
        return 0;
    }
    if (count == 2 && strcmp(arguments[1], "--version") == 0) {
        puts(LWIRCAM_BUILD_VERSION);
        return 0;
    }
    if (count == 2 && strcmp(arguments[1], "--help") == 0) {
        usage(stdout);
        return 0;
    }
    struct capture_options options = {.warmup_ms = 5000, .timeout_ms = 20000, .warn_percent = 1, .score_threshold = 0.6};
    options.render.jpeg_quality = 95;
    inference_settings_init(&model_settings, INFERENCE_NATIVE);
    /* Pre-scan for the settings path: the INI must be applied before CLI options override it. */
    for (int index = 1; index < count; ++index) {
        if (!strcmp(arguments[index], "--config") || !strcmp(arguments[index], "--settings")) {
            if (options.config || ++index >= count || !arguments[index][0]) return 2;
            options.config = arguments[index];
        }
    }
    char default_settings[INFERENCE_PATH_CAPACITY];
    if (!options.config && inference_settings_find("lwircam", "lwir_cam", default_settings, sizeof(default_settings)))
        options.config = default_settings;
    if (options.config && !load_config(options.config, &options)) {
        fprintf(stderr, "LWIR: invalid runtime policy: %s\n", options.config); return 2;
    }
    if (!parse_options(count, arguments, &options)) {
        usage(stderr);
        return 2;
    }
#if LWIR_ENABLE_ANALYSIS
    if (options.inference_input || options.check_model) {
        if (!begin_inference(&options)) return 2;
        char error[2048];
        const bool valid = options.check_model || lwir_inference_image(options.inference_input, 1, error, sizeof(error));
        if (!valid) fprintf(stderr, "LWIR: %s\n", error);
        else puts(options.check_model ? "lwircam model ready adapter=lwir_classifier_v1" : "OK");
        lwir_inference_close();
        return valid ? 0 : 1;
    }
    if (options.geolocate_input)
        return lwir_geolocate_image(options.geolocate_input, options.calibration) == 0 ? 0 : 1;
#endif
#if LWIR_ENABLE_MOCK
    if (options.mock_input)
        return lwir_process_mock(options.mock_input, options.output ? options.output : options.mock_input,
            options.mock_layer_only) == 0 ? 0 : 1;
#endif
    if (options.corrections) {
        if (!lwir_profile_load(options.corrections, &correction_profile)) {
            fprintf(stderr, "LWIR: invalid correction profile: %s\n", options.corrections); return 2;
        }
        options.render.profile = &correction_profile;
    }
    struct stat existing;
    /* Early, friendly refusal; the link()-based publication is the authoritative no-overwrite guard. */
    if (options.output && lstat(options.output, &existing) == 0) {
        fprintf(stderr, "LWIR: refusing to overwrite %s\n", options.output);
        return 1;
    }
    if (options.output && errno != ENOENT) {
        fprintf(stderr, "LWIR: cannot inspect %s: %s\n", options.output, strerror(errno));
        return 1;
    }
    char raw_path[1100];
    if (options.native_raw && options.output) {
        const int length = snprintf(raw_path, sizeof(raw_path), "%s.raw", options.output);
        if (length < 0 || (size_t)length >= sizeof(raw_path)) return 2;
        options.record_raw = raw_path;
    }
    struct sigaction action = {.sa_handler = interrupt_capture};
    if (sigemptyset(&action.sa_mask) != 0 || sigaction(SIGINT, &action, NULL) != 0 ||
        sigaction(SIGTERM, &action, NULL) != 0) {
        fprintf(stderr, "LWIR: cannot install cancellation handlers: %s\n", strerror(errno));
        return 1;
    }
    if (options.server) return serve_captures(&options);
    /* The child inherits the handlers, so both observe SIGINT/SIGTERM sent to the process group. */
    const pid_t child = fork();
    if (child < 0) { fprintf(stderr, "LWIR: cannot fork capture: %s\n", strerror(errno)); return 1; }
    if (child == 0) _exit(save_capture(&options));
    return supervise(child, options.timeout_ms);
}