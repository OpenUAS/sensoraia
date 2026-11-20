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

/** @file
 * @brief Strict loader for measured LWIR display-correction profiles.
 *
 * @details Parses a flat INI file whose format key is "LWIR_CORRECTIONS_V1":
 * serial, gain label, width, height, scale_ppm (500000..1500000), offset_mk
 * (-50000..50000), range_min_raw/range_max_raw (supported raw code range,
 * excluding the endpoint codes 0 and 65535), bad_pixel_count and
 * bad_pixel_0..bad_pixel_N-1 as "column,row" pairs.
 *
 * The renderer applies these corrections to display pixels only; native
 * temperatures, measurements and stored APP15 data are never altered. The
 * SHA-256 of the exact file bytes becomes the profile identity recorded in
 * each JPEG's provenance text.
 *
 * Invariants: the whole file is bounded to 8192 bytes; all fields are
 * mandatory and unique; bad pixels are bounded by LWIR_MAX_BAD_PIXELS,
 * inside the profile geometry, sorted ascending and duplicate-free. The
 * caller's struct is written only after every check passed (all-or-nothing),
 * using a byte loop rather than memcpy per project policy.
 */
#include "lwir_render.h"
#include "bounded_ini.h"
#include "sha256fst.h"

/** @brief Parse state; kept apart from the caller's profile until fully validated. */
struct profile_parser {
    struct lwir_profile profile;               /**< Candidate profile under construction. */
    bool fields[10];                           /**< Seen flags for the ten mandatory keys, indexed as in profile_field(). */
    bool bad_seen[LWIR_MAX_BAD_PIXELS];        /**< Seen flags per bad_pixel_N index; detects duplicates and gaps. */
    uint16_t bad_x[LWIR_MAX_BAD_PIXELS];       /**< Bad-pixel columns; linearized only once width is known. */
    uint16_t bad_y[LWIR_MAX_BAD_PIXELS];       /**< Bad-pixel rows. */
};

/** @brief Copy a serial or gain label after validating it.
 * @param destination Label buffer in the candidate profile.
 * @param capacity Buffer size including the terminator.
 * @param value NUL-terminated label text.
 * @return False for empty, too long, control characters or DEL, or "unknown".
 * @details "unknown" is the placeholder written to evidence when a label is
 * missing, so a profile must not match a camera whose label was never given.
 */
static bool copy_label(char *destination, size_t capacity, const char *value)
{
    const size_t length = strlen(value);
    if (!length || length >= capacity || !strcmp(value, "unknown")) return false;
    for (size_t index = 0; index < length; ++index) if ((unsigned char)value[index] < 32 || value[index] == 127) return false;
    for (size_t index = 0; index <= length; ++index) destination[index] = value[index];
    return true;
}

/** @brief ini_parse() callback for one profile key.
 * @param context struct profile_parser.
 * @param key INI key.
 * @param value NUL-terminated value.
 * @return False aborts the load: unknown or repeated key, wrong format string,
 * invalid label, malformed number or a value out of range.
 * @details Unlike the provenance reader, unknown keys are errors: a profile
 * changes rendered output, so a misspelled key must not be silently ignored.
 * Bad-pixel coordinates are only bounded by LWIR_MAX_WIDTH/HEIGHT here because
 * width and height may appear later in the file; lwir_profile_load() rechecks
 * them against the declared geometry.
 */
static bool profile_field(void *context, const char *key, const char *value)
{
    struct profile_parser *parser = context;
    static const char *const keys[] = {"format", "serial", "gain", "width", "height", "scale_ppm", "offset_mk", "range_min_raw", "range_max_raw", "bad_pixel_count"};
    unsigned field = 0;
    while (field < 10 && strcmp(key, keys[field])) ++field;
    if (field == 10) {
        /* Not a fixed key: must be bad_pixel_N with N below the list capacity. */
        const char prefix[] = "bad_pixel_";
        uint64_t index;
        if (strncmp(key, prefix, sizeof(prefix) - 1) || !ini_uint(key + sizeof(prefix) - 1, &index) ||
            index >= LWIR_MAX_BAD_PIXELS || parser->bad_seen[index] || strlen(value) >= 32) return false;
        char coordinates[32];
        /* Local copy so the comma can be split without touching the parser's buffer. */
        for (size_t position = 0; position <= strlen(value); ++position) coordinates[position] = value[position];
        char *separator = strchr(coordinates, ',');
        if (!separator) return false;
        *separator++ = '\0';
        uint64_t column, row;
        if (!ini_uint(coordinates, &column) || !ini_uint(separator, &row) || column >= LWIR_MAX_WIDTH || row >= LWIR_MAX_HEIGHT) return false;
        parser->bad_seen[index] = true;
        parser->bad_x[index] = (uint16_t)column; parser->bad_y[index] = (uint16_t)row;
        return true;
    }
    if (parser->fields[field]) return false;
    parser->fields[field] = true;
    if (field == 0) return !strcmp(value, "LWIR_CORRECTIONS_V1");
    if (field == 1) return copy_label(parser->profile.serial, sizeof(parser->profile.serial), value);
    if (field == 2) return copy_label(parser->profile.gain, sizeof(parser->profile.gain), value);
    uint64_t number;
    /* Only offset_mk may be negative; ini_uint() itself accepts unsigned text only. */
    const bool negative = field == 6 && value[0] == '-';
    if (!ini_uint(value + (negative ? 1 : 0), &number)) return false;
    switch (field) {
    /* Width must be even and at least 2, matching the native frame width rule. */
    case 3: if (number < 2 || number > LWIR_MAX_WIDTH || number % 2) return false; parser->profile.width = (unsigned)number; break;
    case 4: if (number == 0 || number > LWIR_MAX_HEIGHT) return false; parser->profile.height = (unsigned)number; break;
    case 5: if (number < 500000 || number > 1500000) return false; parser->profile.scale_ppm = (unsigned)number; break;
    case 6: if (number > 50000) return false; parser->profile.offset_mk = negative ? -(int)number : (int)number; break;
    /* Raw codes 0 and 65535 are reserved invalid endpoints and cannot be supported. */
    case 7: if (number == 0 || number >= UINT16_MAX) return false; parser->profile.range_min = (uint16_t)number; break;
    case 8: if (number == 0 || number >= UINT16_MAX) return false; parser->profile.range_max = (uint16_t)number; break;
    case 9: if (number > LWIR_MAX_BAD_PIXELS) return false; parser->profile.bad_count = (unsigned)number; break;
    default: return false;
    }
    return true;
}

/** @brief Load and fully validate a profile; see lwir_render.h.
 * @details Steps: bounded read (8192-byte buffer) -> SHA-256 of the raw bytes ->
 * strict parse -> completeness and range checks -> bad-pixel geometry check
 * and linearization -> sort -> duplicate rejection -> byte copy to caller.
 * On any failure errno is EINVAL, including read failures, whose original
 * errno is not preserved; @p profile is then left untouched.
 * @note The digest covers comments and whitespace too, so any byte change
 * yields a new profile identity.
 */
bool lwir_profile_load(const char *path, struct lwir_profile *profile)
{
    if (!profile) { errno = EINVAL; return false; }
    char data[8192];
    size_t length;
    struct profile_parser parser = {0};
    if (!ini_read_file(path, data, sizeof(data), &length)) { errno = EINVAL; return false; }
    /* Hash before parsing: the identity is the file as stored, not as interpreted. */
    sha256_hex(data, length, parser.profile.sha256);
    if (!ini_parse(data, length, profile_field, &parser)) { errno = EINVAL; return false; }
    for (unsigned field = 0; field < 10; ++field) if (!parser.fields[field]) { errno = EINVAL; return false; }
    if (parser.profile.range_min >= parser.profile.range_max) { errno = EINVAL; return false; }
    for (unsigned index = 0; index < LWIR_MAX_BAD_PIXELS; ++index) {
        /* Indices 0..bad_pixel_count-1 must all be present and nothing beyond: no gaps or extras. */
        if (parser.bad_seen[index] != (index < parser.profile.bad_count)) { errno = EINVAL; return false; }
        if (index < parser.profile.bad_count) {
            if (parser.bad_x[index] >= parser.profile.width || parser.bad_y[index] >= parser.profile.height) { errno = EINVAL; return false; }
            parser.profile.bad_pixels[index] = (uint32_t)parser.bad_y[index] * parser.profile.width + parser.bad_x[index];
        }
    }
    /* Insertion sort: at most LWIR_MAX_BAD_PIXELS entries, no heap. The renderer
     * binary-searches the ascending list, and sorting turns duplicate detection
     * into a neighbour comparison. */
    for (unsigned index = 1; index < parser.profile.bad_count; ++index) {
        const uint32_t value = parser.profile.bad_pixels[index];
        unsigned position = index;
        while (position && parser.profile.bad_pixels[position - 1] > value) { parser.profile.bad_pixels[position] = parser.profile.bad_pixels[position - 1]; --position; }
        parser.profile.bad_pixels[position] = value;
    }
    for (unsigned index = 1; index < parser.profile.bad_count; ++index) if (parser.profile.bad_pixels[index] == parser.profile.bad_pixels[index - 1]) { errno = EINVAL; return false; }
    /* Publish only now, after every check: callers never see a half-valid profile. */
    unsigned char *destination = (unsigned char *)profile;
    const unsigned char *source = (const unsigned char *)&parser.profile;
    for (size_t index = 0; index < sizeof(*profile); ++index) destination[index] = source[index];
    return true;
}

/** @brief Exact match of geometry, serial and gain label; see lwir_render.h.
 * @details Plain string equality: no case folding or trimming, so a profile
 * never silently applies to a different camera or gain setting.
 */
bool lwir_profile_matches(const struct lwir_profile *profile, unsigned width, unsigned height,
                            const char *serial, const char *gain)
{
    return profile && serial && gain && profile->width == width && profile->height == height &&
        !strcmp(profile->serial, serial) && !strcmp(profile->gain, gain);
}
