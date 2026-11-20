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
 * @brief Bounded thermal-marker reader; JPEG pixels are deliberately left alone.
 *
 * @details Walks JPEG marker segments from SOI up to the first SOS or EOI and
 * reassembles the APP15 temperature plane written by lwir_image.c. Optionally
 * parses the APP14 "LWIREV02" text for native geometry and identity. The
 * walk uses one fixed 65534-byte segment buffer on the stack and copies
 * samples straight into the caller's destination, so no heap or intermediate
 * plane is needed.
 *
 * Defensive rules: regular files of at most 256 MiB only; every chunk is
 * checked for signature, dimensions, offset continuity and capacity; and the
 * file's identity, size and timestamps must be identical before and after the
 * walk, so a file replaced or rewritten mid-read is rejected.
 */
#include "lwir_container.h"
#include "lwir_frame.h"
#include "exifite.h"
#include "bounded_ini.h"

#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

/** @brief Decode a little-endian uint32 from four bytes.
 * @param bytes At least four readable bytes.
 * @return Decoded value, independent of host endianness.
 */
static uint32_t read_u32_le(const uint8_t *bytes)
{
    return (uint32_t)bytes[0] | ((uint32_t)bytes[1] << 8)
        | ((uint32_t)bytes[2] << 16) | ((uint32_t)bytes[3] << 24);
}

/** @brief Accumulator for the INI callback over the APP14 provenance text. */
struct geometry_fields {
    struct lwir_saved_geometry value;  /**< Parsed values, copied out only on full success. */
    unsigned seen;                     /**< Bit per key in geometry_field()'s name table; detects duplicates and completeness. */
};

/** @brief ini_parse() callback that extracts the eleven geometry keys.
 * @param context struct geometry_fields accumulator.
 * @param key Key from the LWIREV02 text.
 * @param value Value text, NUL-terminated.
 * @return False aborts the parse: duplicate key, malformed number, or a value
 * out of range. Unrelated keys return true and are ignored, so newer writers
 * can add fields without breaking older readers.
 * @details Table index drives both validation and the @c seen bit. Ranges:
 * width/height and flags at most 65535, rotation a multiple of 90 up to 270,
 * mirror/flip/legend 0 or 1, source "camera" or "replay", boot_id non-empty
 * and shorter than its buffer, frame_sha256 exactly 64 lowercase hex digits.
 */
static bool geometry_field(void *context, const char *key, const char *value)
{
    struct geometry_fields *fields = context;
    const char *names[] = {"width", "height", "rotation_cw", "mirror", "flip", "legend",
        "source", "sequence", "arrival_us", "boot_id", "frame_sha256"};
    unsigned index = 0;
    while (index < 11 && strcmp(key, names[index])) ++index;
    if (index == 11) return true;
    /* A repeated key could otherwise silently override an earlier value. */
    if (fields->seen & (1U << index)) return false;
    fields->seen |= 1U << index;
    if (index == 6) {
        fields->value.camera = !strcmp(value, "camera");
        return fields->value.camera || !strcmp(value, "replay");
    }
    if (index >= 9) {
        /* Bounded copy including the terminator; length was checked against the buffer. */
        const size_t length = strlen(value);
        char *output = index == 9 ? fields->value.boot_id : fields->value.frame_sha256;
        const size_t limit = index == 9 ? sizeof(fields->value.boot_id) : sizeof(fields->value.frame_sha256);
        if (!length || length >= limit || (index == 10 && (length != 64 || strspn(value, "0123456789abcdef") != 64))) return false;
        for (size_t character = 0; character <= length; ++character) output[character] = value[character];
        return true;
    }
    uint64_t number;
    if (!ini_uint(value, &number)) return false;
    if (index < 6 && number > 65535) return false;
    if (index == 0) fields->value.width = (unsigned)number;
    if (index == 1) fields->value.height = (unsigned)number;
    if (index == 2) { if (number > 270 || number % 90) return false; fields->value.rotation = (unsigned)number; }
    if (index >= 3 && index <= 5 && number > 1) return false;
    if (index == 3) fields->value.mirror = number != 0;
    if (index == 4) fields->value.flip = number != 0;
    if (index == 5) fields->value.legend = number != 0;
    if (index == 7) fields->value.sequence = number;
    if (index == 8) fields->value.arrival_us = number;
    return true;
}

/** @brief Walk JPEG markers, reassemble the APP15 plane and optionally check APP14.
 * @param input Borrowed stream; rewound to the start, never closed here.
 * @param destination Caller-owned sample storage; may be partially written on failure.
 * @param capacity Writable bytes in destination.
 * @param width Native width, written only on result 1.
 * @param height Native height, written only on result 1.
 * @param synthetic True for LWIRSIM1, written only on result 1.
 * @param geometry Optional; when non-NULL, LWIREV02 evidence becomes mandatory
 * and is cross-checked against the layer and the JPEG dimensions.
 * @return 1 complete layer, 0 no layer (only possible with @p geometry NULL),
 * -1 invalid input, inconsistency or I/O failure. errno is not normalized.
 * @details Stops at SOS or EOI because the custom markers are always written
 * before the scan; entropy-coded data is never inspected. APP15 segments
 * with foreign signatures are skipped, since other software may also use
 * APP15. A file stability check (device, inode, size, mtime, ctime) runs
 * last, even after failures, and overrides any earlier success.
 */
static int read_temperature_stream(FILE *input, uint8_t *destination, size_t capacity,
                                    unsigned *width, unsigned *height, bool *synthetic,
                                    struct lwir_saved_geometry *geometry)
{
    if (!input || !destination || !width || !height || !synthetic) return -1;
    struct stat before = {0}, after = {0};
    /* 256 MiB bounds the walk; real LWIR photos are far smaller. */
    bool valid = fstat(fileno(input), &before) == 0 && S_ISREG(before.st_mode)
        && before.st_size >= 4 && before.st_size <= (off_t)(256U * 1024U * 1024U);
    if (valid) valid = fseeko(input, 0, SEEK_SET) == 0;
    uint32_t jpeg_width = 0, jpeg_height = 0;
    unsigned components = 0;
    /* Quarter-turn rotation swaps display axes, so either native maximum may be the display width. */
    const unsigned display_limit = LWIR_MAX_WIDTH > LWIR_MAX_HEIGHT ? LWIR_MAX_WIDTH : LWIR_MAX_HEIGHT;
    if (valid) valid = exif_jpeg_dimensions(input, &jpeg_width, &jpeg_height, &components) == 0
        && jpeg_width <= display_limit && jpeg_height <= display_limit
        && jpeg_width > 0 && jpeg_height > 0;
    if (valid) valid = fseeko(input, 0, SEEK_SET) == 0;
    uint8_t soi[2] = {0};
    if (valid) valid = fread(soi, 1, sizeof(soi), input) == sizeof(soi) && soi[0] == 0xff && soi[1] == 0xd8;
    /* Largest possible APPn payload: 16-bit length minus its own two bytes. */
    uint8_t marker_payload[65534];
    size_t expected = 0, copied = 0;
    uint32_t layer_width = 0, layer_height = 0;
    bool found_layer = false, layer_synthetic = false;
    struct geometry_fields fields = {0};
    bool found_geometry = false;
    while (valid) {
        int byte = fgetc(input);
        if (byte != 0xff) { valid = false; break; }
        /* Any number of 0xff fill bytes may precede a marker code. */
        do { byte = fgetc(input); } while (byte == 0xff);
        if (byte == EOF) { valid = false; break; }
        const unsigned marker = (unsigned)byte;
        /* Custom markers always precede the scan, so stop before entropy-coded data. */
        if (marker == 0xda || marker == 0xd9) break;
        /* TEM and RSTn are standalone markers without a length field. */
        if (marker == 0x01 || (marker >= 0xd0 && marker <= 0xd7)) continue;
        const int length_high = fgetc(input), length_low = fgetc(input);
        if (length_high == EOF || length_low == EOF) { valid = false; break; }
        const size_t segment_length = ((size_t)length_high << 8) | (size_t)length_low;
        if (segment_length < 2 || segment_length - 2 > sizeof(marker_payload)) { valid = false; break; }
        const size_t payload_size = segment_length - 2;
        if (marker == LWIR_THERMAL_APP_MARKER) {
            if (fread(marker_payload, 1, payload_size, input) != payload_size) { valid = false; break; }
            if (payload_size < LWIR_THERMAL_SIGNATURE_BYTES) continue;
            /* Other software may use APP15 too; only our two signatures are claimed. */
            const bool is_synthetic = !memcmp(marker_payload, LWIR_THERMAL_SYNTHETIC_SIGNATURE, LWIR_THERMAL_SIGNATURE_BYTES);
            const bool is_sensor = !memcmp(marker_payload, LWIR_THERMAL_SENSOR_SIGNATURE, LWIR_THERMAL_SIGNATURE_BYTES);
            if (!is_synthetic && !is_sensor) continue;
            /* A signed chunk without samples is malformed, not foreign. */
            if (payload_size <= LWIR_THERMAL_HEADER_BYTES) { valid = false; break; }
            const uint32_t marker_width = read_u32_le(marker_payload + LWIR_THERMAL_WIDTH_OFFSET);
            const uint32_t marker_height = read_u32_le(marker_payload + LWIR_THERMAL_HEIGHT_OFFSET);
            if (!found_layer) {
                /* The first chunk fixes the layer geometry and the capacity requirement. */
                if (!marker_width || !marker_height || marker_width > LWIR_MAX_WIDTH || marker_height > LWIR_MAX_HEIGHT) {
                    valid = false;
                    break;
                }
                layer_width = marker_width;
                layer_height = marker_height;
                expected = (size_t)layer_width * layer_height * 2;
                if (expected > capacity) { valid = false; break; }
            }
            /* Later chunks must agree on size and kind, continue exactly at the
             * bytes copied so far (no gaps, reordering or duplicates) and not
             * overrun the plane; this also guarantees the destination copy fits. */
            if (marker_width != layer_width || marker_height != layer_height
                || read_u32_le(marker_payload + LWIR_THERMAL_DATA_OFFSET) != copied
                || payload_size - LWIR_THERMAL_HEADER_BYTES > expected - copied
                || (found_layer && layer_synthetic != is_synthetic)) { valid = false; break; }
            found_layer = true;
            layer_synthetic = is_synthetic;
            for (size_t index = LWIR_THERMAL_HEADER_BYTES; index < payload_size; ++index)
                destination[copied++] = marker_payload[index];
        } else if (geometry && marker == 0xee) {
            /* APP14 is also used by Adobe; only the LWIREV02 record is parsed, and only once. */
            if (fread(marker_payload, 1, payload_size, input) != payload_size) { valid = false; break; }
            if (payload_size >= 9 && !memcmp(marker_payload, "LWIREV02\n", 9)) {
                if (found_geometry || !ini_parse((char *)marker_payload + 9, payload_size - 9, geometry_field, &fields)) {
                    valid = false; break;
                }
                found_geometry = true;
            }
        } else if (fseeko(input, (off_t)payload_size, SEEK_CUR) != 0) {
            valid = false;
        }
    }
    if (found_layer && copied != expected) valid = false;
    /* 2047 = all eleven key bits. A camera source must pair with LWIRRAW1 and
     * replay with LWIRSIM1; JPEG size must equal native size after rotation. */
    if (geometry && (!found_geometry || fields.seen != 2047 || fields.value.width != layer_width
        || fields.value.height != layer_height || fields.value.camera == layer_synthetic
        || jpeg_width != (fields.value.rotation % 180 ? layer_height : layer_width)
        || jpeg_height != (fields.value.rotation % 180 ? layer_width : layer_height))) valid = false;
    /* Reject files rewritten or replaced during the walk, even if parsing looked fine. */
    if (fstat(fileno(input), &after) != 0 || before.st_dev != after.st_dev || before.st_ino != after.st_ino
        || before.st_size != after.st_size || before.st_mtim.tv_sec != after.st_mtim.tv_sec
        || before.st_mtim.tv_nsec != after.st_mtim.tv_nsec || before.st_ctim.tv_sec != after.st_ctim.tv_sec
        || before.st_ctim.tv_nsec != after.st_ctim.tv_nsec) valid = false;
    if (!valid) return -1;
    if (!found_layer) return 0;
    *width = layer_width;
    *height = layer_height;
    *synthetic = layer_synthetic;
    if (geometry) *geometry = fields.value;
    return 1;
}

/** @brief Public stream reader without geometry; see lwir_container.h. */
int lwir_read_temperature_stream(FILE *input, uint8_t *destination, size_t capacity,
                                 unsigned *width, unsigned *height, bool *synthetic)
{
    return read_temperature_stream(input, destination, capacity, width, height, synthetic, NULL);
}

/** @brief Open @p filename safely and read layer plus geometry; see lwir_container.h.
 * @details O_NOFOLLOW refuses a symlinked final component; O_NONBLOCK keeps
 * a FIFO from blocking the open so the regular-file check can reject it.
 * Results go through locals and are copied out only on success.
 */
int lwir_read_temperature_evidence(const char *filename, uint8_t *destination, size_t capacity,
                                    struct lwir_saved_geometry *geometry)
{
    if (!filename || !geometry) return -1;
    const int descriptor = open(filename, O_RDONLY | O_NONBLOCK | O_CLOEXEC | O_NOFOLLOW);
    if (descriptor < 0) return -1;
    FILE *input = fdopen(descriptor, "rb");
    if (!input) { close(descriptor); return -1; }
    unsigned width = 0, height = 0;
    bool synthetic = false;
    struct lwir_saved_geometry result = {0};
    int status = read_temperature_stream(input, destination, capacity, &width, &height, &synthetic, &result);
    /* A close failure invalidates an otherwise successful read. */
    if (fclose(input)) status = -1;
    if (status == 1) *geometry = result;
    return status;
}

/** @brief Open @p filename safely and read the layer only; see lwir_container.h.
 * @details Same open flags as lwir_read_temperature_evidence(). Outputs are
 * written only when the result is 1 and the stream closed cleanly.
 */
int lwir_read_temperature_layer(const char *filename, uint8_t *destination, size_t capacity,
                                unsigned *width, unsigned *height, bool *synthetic)
{
    if (!filename || !destination || !width || !height || !synthetic) return -1;
    const int descriptor = open(filename, O_RDONLY | O_NONBLOCK | O_CLOEXEC | O_NOFOLLOW);
    if (descriptor < 0) return -1;
    FILE *input = fdopen(descriptor, "rb");
    if (!input) { close(descriptor); return -1; }
    unsigned native_width = 0, native_height = 0;
    bool native_synthetic = false;
    const int result = lwir_read_temperature_stream(input, destination, capacity,
        &native_width, &native_height, &native_synthetic);
    if (fclose(input) != 0) return -1;
    if (result == 1) { *width = native_width; *height = native_height; *synthetic = native_synthetic; }
    return result;
}