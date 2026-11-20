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
 * @file lwir_mock.c
 * @brief Development replay: convert an ordinary JPEG into LWIR evidence (LWIR_ENABLE_MOCK).
 * @details Two modes:
 * - layer only: keep the source's compressed bytes and insert LWIRSIM1 APP15
 *   temperature chunks derived from luminance;
 * - full: decode, build a combined image+temperature frame, re-render it with the
 *   normal LWIR renderer, and carry the source EXIF across.
 * An existing embedded temperature layer is always reused rather than synthesized.
 * Output goes to a same-directory temporary file, is fsynced, then published
 * without overwriting unrelated files. All buffers are static (no heap).
 */
#include "lwir_mock.h"
#include "lwir_container.h"
#include "lwir_frame.h"
#include "lwir_image.h"
#include "lwir_render.h"
#include "exifite.h"

#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <setjmp.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include <jpeglib.h>

/** @brief Static workspace: maximum-size buffers would be too large for the stack and the heap is avoided.
 * @details frame holds the combined layout (image plane of Y/neutral-chroma pairs,
 * then little-endian Kelvin*64 temperatures); thermal receives an embedded layer;
 * marker is one APP15 segment body (20-byte header + 65000 data) and doubles as
 * the copy buffer. Makes the module non-reentrant. */
static struct {
    struct jpeg_decompress_struct decoder;
    struct jpeg_error_mgr error;
    jmp_buf recovery;
    uint8_t frame[LWIR_MAX_FRAME_BYTES];
    uint8_t thermal[LWIR_MAX_WIDTH * LWIR_MAX_HEIGHT * 2];
    uint8_t row[LWIR_MAX_WIDTH * 3];
    uint8_t marker[65020];
    uint8_t exif[EXIF_MAX_PAYLOAD];
} mock_workspace;

/**
 * @brief libjpeg fatal-error hook.
 * @param decoder Unused; the workspace is global.
 * @details libjpeg's default handler calls exit(); jumping back to decode_mock()
 * turns corrupt input into an ordinary false return.
 */
static void mock_jpeg_error(j_common_ptr decoder)
{
    (void)decoder;
    longjmp(mock_workspace.recovery, 1);
}

/**
 * @brief Map display luminance to a synthetic Kelvin*64 sample.
 * @param luminance 0..255 grey level.
 * @return round((celsius + 273.15) * 64).
 * @details Piecewise: 0..128 spans 15..20 degC (background), 129..229 rises
 * steeply to about 594 degC (hot objects), and >= 230 saturates at 600 degC.
 * Only gives tests a plausible scene; it is not a calibration.
 */
static uint16_t synthetic_raw(uint8_t luminance)
{
    const double celsius = luminance <= 128 ? 15.0 + 5.0 * luminance / 128.0 :
        luminance < 230 ? 20.0 + 580.0 * (luminance - 128) / 102.0 : 600.0;
    return (uint16_t)((celsius + 273.15) * 64.0 + 0.5);
}

/**
 * @brief Decode a JPEG into the combined workspace frame.
 * @param input Open stream positioned at the start of the JPEG.
 * @param width In: required width when @p embedded; out: decoded width.
 * @param height In: required height when @p embedded; out: decoded height.
 * @param embedded True to take temperatures from the workspace thermal layer
 * instead of synthesizing them.
 * @return True only for a complete decode with no libjpeg warnings and a size
 * accepted by lwir_frame_bytes(); false otherwise (frame contents undefined).
 * @details Converts RGB to integer BT.601-style luma (77/150/29 >> 8) and stores
 * Y with neutral chroma 128 so the image plane matches the camera's layout.
 */
static bool decode_mock(FILE *input, unsigned *width, unsigned *height, bool embedded)
{
    struct jpeg_decompress_struct *decoder = &mock_workspace.decoder;
    *decoder = (struct jpeg_decompress_struct){0};
    decoder->err = jpeg_std_error(&mock_workspace.error);
    mock_workspace.error.error_exit = mock_jpeg_error;
    /* Every libjpeg failure lands here; only static state is touched afterwards, so no volatile is needed. */
    if (setjmp(mock_workspace.recovery)) {
        jpeg_destroy_decompress(decoder);
        return false;
    }
    jpeg_create_decompress(decoder);
    jpeg_stdio_src(decoder, input);
    bool valid = jpeg_read_header(decoder, TRUE) == JPEG_HEADER_OK &&
        lwir_frame_bytes(decoder->image_width, decoder->image_height) != 0;
    if (embedded && (decoder->image_width != *width || decoder->image_height != *height)) valid = false;
    decoder->out_color_space = JCS_RGB;
    if (valid) valid = jpeg_start_decompress(decoder) && decoder->output_components == 3;
    if (valid) {
        *width = decoder->output_width;
        *height = decoder->output_height;
        const size_t pixels = (size_t)*width * *height;
        while (valid && decoder->output_scanline < decoder->output_height) {
            const size_t row_number = decoder->output_scanline;
            JSAMPROW row = mock_workspace.row;
            valid = jpeg_read_scanlines(decoder, &row, 1) == 1;
            for (unsigned column = 0; valid && column < *width; ++column) {
                const size_t pixel = row_number * *width + column;
                const uint8_t *rgb = row + column * 3;
                const uint8_t luminance = (uint8_t)((77U * rgb[0] + 150U * rgb[1] + 29U * rgb[2]) >> 8);
                mock_workspace.frame[pixel * 2] = luminance;
                mock_workspace.frame[pixel * 2 + 1] = 128;
                const uint16_t raw = embedded ? (uint16_t)mock_workspace.thermal[pixel * 2] |
                    (uint16_t)((uint16_t)mock_workspace.thermal[pixel * 2 + 1] << 8) : synthetic_raw(luminance);
                mock_workspace.frame[(pixels + pixel) * 2] = (uint8_t)raw;
                mock_workspace.frame[(pixels + pixel) * 2 + 1] = (uint8_t)(raw >> 8);
            }
        }
        /* Warnings mean libjpeg patched over damage; refuse rather than build evidence from guessed pixels. */
        if (valid) valid = jpeg_finish_decompress(decoder) && mock_workspace.error.num_warnings == 0;
    }
    jpeg_destroy_decompress(decoder);
    return valid;
}

/**
 * @brief Store a 32-bit value little-endian.
 * @param output Destination of at least 4 bytes.
 * @param number Value to store.
 */
static void write_u32(uint8_t *output, uint32_t number)
{
    for (unsigned index = 0; index < 4; ++index) output[index] = (uint8_t)(number >> (index * 8));
}

/**
 * @brief Copy a JPEG while inserting the workspace temperature plane as LWIRSIM1 APP15 chunks.
 * @param input Source JPEG; rewound here.
 * @param output Destination stream.
 * @param width Temperature width.
 * @param height Temperature height.
 * @return True if everything was copied; false on a non-JPEG source or any I/O error.
 * @details Each chunk is "LWIRSIM1", width, height, byte offset (all u32 LE) and
 * up to 65000 data bytes, keeping the segment under JPEG's 65535-byte limit.
 * Chunks go right after SOI; the remaining source bytes, including its own
 * segments and compressed pixels, are copied unchanged.
 */
static bool attach_layer(FILE *input, FILE *output, unsigned width, unsigned height)
{
    uint8_t header[2];
    if (fseeko(input, 0, SEEK_SET) != 0 || fread(header, 1, 2, input) != 2 ||
        header[0] != 0xff || header[1] != 0xd8 || fwrite(header, 1, 2, output) != 2) return false;
    const size_t pixels = (size_t)width * height;
    for (size_t offset = 0; offset < pixels * 2;) {
        const size_t chunk = pixels * 2 - offset < 65000 ? pixels * 2 - offset : 65000;
        uint8_t *marker = mock_workspace.marker;
        for (size_t index = 0; index < 8; ++index) marker[index] = (uint8_t)"LWIRSIM1"[index];
        write_u32(marker + 8, width); write_u32(marker + 12, height); write_u32(marker + 16, (uint32_t)offset);
        for (size_t index = 0; index < chunk; ++index)
            marker[20 + index] = mock_workspace.frame[pixels * 2 + offset + index];
        /* The JPEG length field counts itself (2 bytes) plus the 20-byte header and data. */
        const size_t length = chunk + 22;
        const uint8_t prefix[4] = {0xff, 0xef, (uint8_t)(length >> 8), (uint8_t)length};
        if (fwrite(prefix, 1, sizeof(prefix), output) != sizeof(prefix) ||
            fwrite(marker, 1, chunk + 20, output) != chunk + 20) return false;
        offset += chunk;
    }
    size_t received;
    while ((received = fread(mock_workspace.marker, 1, sizeof(mock_workspace.marker), input)) != 0)
        if (fwrite(mock_workspace.marker, 1, received, output) != received) return false;
    return !ferror(input);
}

/**
 * @brief Produce mock LWIR evidence from @p input_path at @p output_path.
 * @return 0 on success, -1 on any failure.
 * @details Steps: open and identify the source; read any embedded layer; decode
 * if needed; write a mode-matched temporary file next to the output; check that
 * the source did not change meanwhile; publish with link() (fails if the name
 * exists) or, in place, rename() after confirming the path still names the same
 * inode; finally fsync the directory.
 * @warning -1 is also returned if only the final directory fsync fails, after the
 * file has already been published.
 */
int lwir_process_mock(const char *input_path, const char *output_path, bool layer_only)
{
    if (!input_path || !output_path) return -1;
    const int input_fd = open(input_path, O_RDONLY | O_CLOEXEC | O_NOFOLLOW);
    if (input_fd < 0) return -1;
    /* Identity snapshot: used to detect concurrent modification and an in-place path swap. */
    struct stat original;
    if (fstat(input_fd, &original) != 0 || !S_ISREG(original.st_mode) ||
        original.st_size <= 0 || original.st_size > 256 * 1024 * 1024) { close(input_fd); return -1; }
    FILE *input = fdopen(input_fd, "rb");
    if (!input) { close(input_fd); return -1; }
    unsigned width = 0, height = 0;
    /* Stays true unless a sensor (LWIRRAW1) layer is found, so absent provenance defaults to synthetic. */
    bool synthetic = true;
    const int layer = lwir_read_temperature_stream(input, mock_workspace.thermal,
        sizeof(mock_workspace.thermal), &width, &height, &synthetic);
    /* Existing evidence needs no visual decode, including an oriented display JPEG. */
    if (layer == 1 && layer_only) return fclose(input) == 0 ? 0 : -1;
    bool success = layer >= 0 && fseeko(input, 0, SEEK_SET) == 0 && decode_mock(input, &width, &height, layer == 1);
    const bool in_place = strcmp(input_path, output_path) == 0;
    struct stat existing;
    /* Out-of-place output must not exist yet; any lstat error other than ENOENT is also a refusal. */
    if (!in_place && (lstat(output_path, &existing) == 0 || errno != ENOENT)) success = false;
    char temporary[PATH_MAX];
    /* Same directory as the output so link()/rename() stay on one filesystem. */
    const int length = snprintf(temporary, sizeof(temporary), "%s.mock.XXXXXX", output_path);
    if (length < 0 || (size_t)length >= sizeof(temporary)) success = false;
    int output_fd = -1;
    if (success) output_fd = mkstemp(temporary);
    const bool temporary_created = output_fd >= 0;
    if (success && output_fd < 0) success = false;
    if (success && fchmod(output_fd, original.st_mode & 0777) != 0) success = false;
    FILE *output = NULL;
    if (success && layer_only) {
        output = fdopen(output_fd, "wb");
        if (!output) success = false;
        else {
            output_fd = -1;
            success = attach_layer(input, output, width, height) && fflush(output) == 0 && fsync(fileno(output)) == 0;
        }
    } else if (success) {
        /* The renderer creates its file by path, so release the reserved name; it is briefly unclaimed. */
        success = close(output_fd) == 0 && unlink(temporary) == 0;
        output_fd = -1;
        struct lwir_render_options render = {.view = LWIR_PERCENTILE, .palette = LWIR_WHITE_HOT,
            .low_percentile = 2, .high_percentile = 98};
        /* NULL device state: synthetic or replayed imagery must never claim live camera provenance. */
        if (success) success = lwir_write_jpeg_rendered(temporary, mock_workspace.frame,
            lwir_frame_bytes(width, height), width, height, synthetic, NULL, &render);
        if (success) success = chmod(temporary, original.st_mode & 0777) == 0;
        size_t exif_size = 0;
        /* ENODATA means the source simply has no EXIF; any other read failure aborts. */
        if (success && exif_read_jpeg(input_path, mock_workspace.exif, sizeof(mock_workspace.exif), &exif_size) == 0) {
            output = fopen(temporary, "r+b");
            success = output && exif_update_jpeg(output, mock_workspace.exif, exif_size) == 0 && fsync(fileno(output)) == 0;
        } else if (success && errno != ENODATA) success = false;
    }
    if (output) { if (fclose(output) != 0) success = false; }
    if (output_fd >= 0) close(output_fd);
    struct stat current;
    /* A source rewritten mid-run would make the output inconsistent with it. */
    if (fstat(input_fd, &current) != 0 || original.st_size != current.st_size ||
        original.st_mtim.tv_sec != current.st_mtim.tv_sec || original.st_mtim.tv_nsec != current.st_mtim.tv_nsec) success = false;
    if (fclose(input) != 0) success = false;
    if (success) {
        if (in_place) {
            /* Replace only if the path still names the file we read, not a substitute. */
            struct stat named;
            success = lstat(input_path, &named) == 0 && named.st_dev == original.st_dev && named.st_ino == original.st_ino &&
                rename(temporary, output_path) == 0;
        } else success = link(temporary, output_path) == 0;
    }
    /* After link() this drops the extra name; after rename() it fails harmlessly. */
    if (temporary_created) unlink(temporary);
    if (success) {
        /* Persist the new directory entry, not just the file data. */
        char parent[PATH_MAX];
        const size_t path_size = strlen(output_path);
        if (path_size >= sizeof(parent)) return -1;
        for (size_t index = 0; index <= path_size; ++index) parent[index] = output_path[index];
        char *separator = strrchr(parent, '/');
        if (!separator) { parent[0] = '.'; parent[1] = '\0'; }
        else if (separator == parent) separator[1] = '\0';
        else *separator = '\0';
        const int directory = open(parent, O_RDONLY | O_DIRECTORY | O_CLOEXEC);
        if (directory < 0) return -1;
        success = fsync(directory) == 0;
        if (close(directory) != 0) success = false;
    }
    return success ? 0 : -1;
}