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
 * @brief LWIR JPEG encoder and durable no-overwrite file publisher.
 *
 * @details Implements lwir_image.h. Every photo goes through one static libjpeg
 * workspace so no heap allocation is made per photo by this module (libjpeg
 * still allocates internally). Consequently the encoder is single-threaded
 * and not re-entrant.
 *
 * Marker order inside each JPEG, all before the first scanline:
 * 1. APP14 "LWIREV02" provenance text (only with evidence).
 * 2. APP1 EXIF with the sensor-capture tag (only with evidence, live frames).
 * 3. APP15 thermal chunks (combined frames only).
 * libjpeg may emit its own JFIF APP0 before these, so readers must scan
 * markers rather than expect APP1 directly after SOI.
 *
 * Codec errors longjmp from jpeg_failure() back to encode_jpeg() with
 * errno = EIO. Publication writes a temporary, fsyncs, link()s and fsyncs
 * the directory; see publish().
 */

#include "lwir_image.h"
#include "lwir_container.h"
#include "exifite.h"
#include "lwir_render.h"

#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <setjmp.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <jpeglib.h>

/** @brief Static encoder state shared by every photo.
 * @details Static storage avoids per-photo heap use and keeps worst-case
 * memory fixed. Shared state is why the encoder is not re-entrant.
 */
static struct {
    struct jpeg_compress_struct encoder;  /**< libjpeg compressor, zeroed and recreated per photo. */
    struct jpeg_error_mgr error;          /**< Standard error manager with error_exit redirected to jpeg_failure(). */
    jmp_buf recovery;                     /**< Target of jpeg_failure(); armed in encode_jpeg(). */
    uint8_t marker[LWIR_THERMAL_HEADER_BYTES + LWIR_THERMAL_CHUNK_CAPACITY]; /**< One APP15 chunk being assembled. */
    uint8_t row[LWIR_MAX_WIDTH * 3U];     /**< One interleaved YCbCr display row; the renderer is given its capacity. */
    struct exif_document exif;            /**< EXIF builder for the sensor-capture tag. */
    uint8_t exif_payload[EXIF_MAX_PAYLOAD]; /**< Serialized EXIF APP1 payload. */
    char evidence[8192];                  /**< APP14 text; well below the 65533-byte APPn limit, so one marker suffices. */
} jpeg_workspace;

/** @brief Store @p value as four little-endian bytes.
 * @param destination At least four writable bytes.
 * @param value Value to encode.
 * @details Explicit byte order keeps the APP15 header independent of host endianness.
 */
static void write_u32(uint8_t *destination, uint32_t value)
{
    for (size_t index = 0; index < 4; ++index) destination[index] = (uint8_t)(value >> (index * 8));
}

/** @brief libjpeg error_exit replacement: abandon the photo instead of exiting.
 * @param encoder Unused; the single static workspace already identifies the photo.
 * @note Does not return; jumps to the recovery point armed by the encoder.
 * @details The default handler calls exit(), which would kill the capture
 * worker. errno is set to EIO before jumping so the caller sees an I/O-class
 * failure regardless of the libjpeg message code.
 */
static void jpeg_failure(j_common_ptr encoder)
{
    (void)encoder;
    errno = EIO;
    longjmp(jpeg_workspace.recovery, 1);
}

/** @brief Write application provenance without taking ownership of the encoder.
 * @param encoder Started compressor; markers must precede the first scanline.
 * @param width Native frame width.
 * @param height Native frame height.
 * @param display_width Encoded width after orientation.
 * @param display_height Encoded height after orientation.
 * @param synthetic True for replayed/simulated frames.
 * @param evidence Optional provenance; NULL writes no APP14 or EXIF at all.
 * @param render Resolved display policy (never NULL here).
 * @param range Window and clipping counts from lwir_render_prepare().
 * @return Zero on success, otherwise the errno value the encoder owner must publish:
 * ENOSPC when the text exceeds the fixed evidence buffer, EINVAL when the
 * sensor-capture record cannot be encoded or serialized.
 * @details Teardown stays with the owner so marker failures retain cleanup/error ordering.
 * Builds the APP14 text in several bounded snprintf steps; each step checks
 * truncation so a partial key=value record is never emitted.
 * @warning jpeg_write_marker() can still longjmp on codec failure; that unwinds
 * straight to encode_jpeg() and bypasses this function's return value.
 */
static int write_provenance_markers(j_compress_ptr encoder, unsigned width, unsigned height,
                                    unsigned display_width, unsigned display_height, bool synthetic,
                                    const struct lwir_evidence *evidence, const struct lwir_render_options *render,
                                    const struct lwir_render_range *range)
{
    if (evidence != NULL) {
        /* Unavailable facts are written as "unknown"/"unverified" rather than omitted,
         * so a reader never mistakes absence for a default or a measured value. */
        const int length = snprintf(jpeg_workspace.evidence, sizeof(jpeg_workspace.evidence),
            "LWIREV02\nsource=%s\nserial=%s\ngain=%s\nfirmware=unknown\nffc=unknown\nenvironment=unknown\nlens_focus=unknown\npower_on_age=unknown\nboot_id=%s\n"
            "sequence=%llu\nrequest_us=%llu\narrival_us=%llu\nrequest_id=%llu\nexposure=unknown\ncalibration=unverified\n"
            "width=%u\nheight=%u\nframe_sha256=%s\nraw_file=%s\nrendering=%s\n"
            "display_width=%u\ndisplay_height=%u\nrotation_cw=%u\nmirror=%u\nflip=%u\nraw_coordinates=native\norientation_order=rotate_mirror_flip\n"
            "view=%u\npalette=%u\nlegend=%u\nwindow_low_kelvin64=%d\nwindow_high_kelvin64=%d\nclipped_low=%zu\nclipped_high=%zu\ninvalid_codes=%zu\n"
            "profile_sha256=%s\nscale_ppm=%u\noffset_mk=%d\nsupport_min_raw=%u\nsupport_max_raw=%u\nbad_pixel_count=%u\ndenoise_kelvin64=%u\ngain_source=user_or_unknown\n",
            synthetic ? "replay" : "camera", evidence->serial ? evidence->serial : "unknown",
            evidence->gain ? evidence->gain : "unknown", evidence->capture.boot_id,
            (unsigned long long)evidence->capture.sequence, (unsigned long long)evidence->request_us,
            (unsigned long long)evidence->arrival_us, (unsigned long long)evidence->request_id, width, height, evidence->frame_sha256,
            evidence->raw_path ? evidence->raw_path : "none", evidence->rendering ? evidence->rendering : "sensor",
            display_width, display_height, render->rotation, render->mirror ? 1U : 0U, render->flip ? 1U : 0U,
            (unsigned)render->view, (unsigned)render->palette, render->legend ? 1U : 0U, range->low, range->high, range->below, range->above, range->invalid,
            render->profile ? render->profile->sha256 : "none", render->profile ? render->profile->scale_ppm : 1000000U,
            render->profile ? render->profile->offset_mk : 0, render->profile ? render->profile->range_min : 1U,
            render->profile ? render->profile->range_max : 65534U, render->profile ? render->profile->bad_count : 0U, render->denoise_raw);
        if (length < 0 || (size_t)length >= sizeof(jpeg_workspace.evidence)) {
            return ENOSPC;
        }
        size_t evidence_length = (size_t)length;
        /* Mirrors the encoder's mapping of quality 0 to the legacy fixed 100. */
        const int quality_length = snprintf(jpeg_workspace.evidence + evidence_length, sizeof(jpeg_workspace.evidence) - evidence_length,
            "jpeg_quality=%u\njpeg_sampling=444\n", render->jpeg_quality ? render->jpeg_quality : 100U);
        if (quality_length < 0 || (size_t)quality_length >= sizeof(jpeg_workspace.evidence) - evidence_length) return ENOSPC;
        evidence_length += (size_t)quality_length;
        /* Kernel buffer time is kept as evidence only; it is not proven to mark exposure. */
        const int timestamp_length = snprintf(jpeg_workspace.evidence + evidence_length, sizeof(jpeg_workspace.evidence) - evidence_length,
            "kernel_monotonic_us=%llu\nkernel_timestamp_flags=%u\nkernel_timestamp_is_qualified_exposure=false\n",
            (unsigned long long)evidence->kernel_monotonic_us, evidence->kernel_timestamp_flags);
        if (timestamp_length < 0 || (size_t)timestamp_length >= sizeof(jpeg_workspace.evidence) - evidence_length) return ENOSPC;
        evidence_length += (size_t)timestamp_length;
        if (evidence->measurement && evidence->statistics) {
            const struct lwir_measurement *measurement = evidence->measurement;
            const struct lwir_temperature_stats *statistics = evidence->statistics;
            const struct lwir_thresholds *thresholds = evidence->thresholds;
            /* A point reports a 1x1 ROI; a zero ROI width means the whole frame.
             * Gotcha: the reported height is also selected by roi.width, not roi.height. */
            const int added = snprintf(jpeg_workspace.evidence + evidence_length, sizeof(jpeg_workspace.evidence) - evidence_length,
                "measurement=%s\nmeasurement_coordinates=native\nmeasurement_roi=%u,%u,%u,%u\nmeasurement_line_end=%u,%u\naverage_3x3=%u\n"
                "measurement_pixels=%zu\nmeasurement_endpoint_codes=%zu\nmeasurement_min_raw=%u\nmeasurement_min_xy=%u,%u\n"
                "measurement_max_raw=%u\nmeasurement_max_xy=%u,%u\nmeasurement_sum_raw=%llu\nmeasurement_sum_squared_raw=%llu\n"
                "coverage_enabled=%u\ncoverage_source=original_centers\ncoverage_low_kelvin64=%u\ncoverage_high_kelvin64=%u\n"
                "coverage_below=%zu\ncoverage_within=%zu\ncoverage_above=%zu\nwarn_percent=%u\n",
                measurement->kind == LWIR_POINT ? "point" : measurement->kind == LWIR_LINE ? "line" : "roi",
                measurement->roi.x, measurement->roi.y, measurement->kind == LWIR_POINT ? 1U : measurement->roi.width ? measurement->roi.width : width,
                measurement->kind == LWIR_POINT ? 1U : measurement->roi.width ? measurement->roi.height : height,
                measurement->end_x, measurement->end_y, measurement->average_3x3 ? 1U : 0U, statistics->pixels, statistics->endpoint_codes,
                statistics->min_raw, statistics->min_x, statistics->min_y, statistics->max_raw, statistics->max_x, statistics->max_y,
                (unsigned long long)statistics->sum_raw, (unsigned long long)statistics->sum_squared_raw,
                thresholds && thresholds->enabled ? 1U : 0U, thresholds ? thresholds->low : 0U, thresholds ? thresholds->high : 0U,
                statistics->below, statistics->within, statistics->above, evidence->warn_percent);
            if (added < 0 || (size_t)added >= sizeof(jpeg_workspace.evidence) - evidence_length) {
                return ENOSPC;
            }
            evidence_length += (size_t)added;
        }
        if (evidence->layout == LWIR_YUYV_IMAGE) {
            /* Tells readers why no APP15 layer exists, instead of leaving it to guesswork. */
            const int added = snprintf(jpeg_workspace.evidence + evidence_length, sizeof(jpeg_workspace.evidence) - evidence_length,
                "device=tdk_5474_5411\nframe_layout=yuyv_image\ntemperature_data=unavailable\nyuv_range=limited\n");
            if (added < 0 || (size_t)added >= sizeof(jpeg_workspace.evidence) - evidence_length) return ENOSPC;
            evidence_length += (size_t)added;
        }
        /* Synthetic output passes NULL so replayed files never carry live device state.
         * A zero length means the state text did not fit the remaining buffer. */
        const size_t state_length = lwir_state_format(synthetic ? NULL : evidence->device_state,
            jpeg_workspace.evidence + evidence_length, sizeof(jpeg_workspace.evidence) - evidence_length);
        if (!state_length) return ENOSPC;
        evidence_length += state_length;
        jpeg_write_marker(encoder, JPEG_APP0 + 14, (const JOCTET *)jpeg_workspace.evidence, (unsigned)evidence_length);
        /* Only live frames get the sensor-capture tag, so replays cannot pose as timed exposures. */
        if (!synthetic) {
            exif_document_init(&jpeg_workspace.exif);
            unsigned char *capture = exif_document_reserve(&jpeg_workspace.exif, EXIF_CAMERA,
                SENSOR_CAPTURE_TAG, EXIF_UNDEFINED, SENSOR_CAPTURE_BYTES);
            size_t size = 0;
            if (!sensor_capture_encode(&evidence->capture, capture, SENSOR_CAPTURE_BYTES) ||
                exif_serialize(&jpeg_workspace.exif, jpeg_workspace.exif_payload,
                               sizeof(jpeg_workspace.exif_payload), &size) != 0) {
                return EINVAL;
            }
            jpeg_write_marker(encoder, JPEG_APP0 + 1, jpeg_workspace.exif_payload, (unsigned)size);
        }
    }
    return 0;
}

/** @brief Preserve exact native temperatures independently of display rendering.
 * @param encoder Started compressor; called before the first scanline.
 * @param frame Combined frame; the temperature plane starts after the YUYV plane.
 * @param width Native width.
 * @param height Native height.
 * @param synthetic Selects the LWIRSIM1 instead of the LWIRRAW1 signature.
 * @details Splits the width*height*2-byte plane into APP15 chunks of at most
 * LWIR_THERMAL_CHUNK_CAPACITY bytes, each with the full header. Offsets are
 * relative to the plane start and strictly sequential, which the reader
 * enforces. Bytes are copied verbatim, so JPEG quality never touches them.
 * Codec failures longjmp to encode_jpeg().
 */
static void write_thermal_markers(j_compress_ptr encoder, const uint8_t *frame,
                                  unsigned width, unsigned height, bool synthetic)
{
    const size_t plane_bytes = (size_t)width * height * 2;
    /* The temperature plane follows the equally sized YUYV plane in the combined frame. */
    const char *signature = synthetic ? LWIR_THERMAL_SYNTHETIC_SIGNATURE : LWIR_THERMAL_SENSOR_SIGNATURE;
    for (size_t offset = 0; offset < plane_bytes;) {
        const size_t remaining = plane_bytes - offset;
        const size_t chunk = remaining < LWIR_THERMAL_CHUNK_CAPACITY ? remaining : LWIR_THERMAL_CHUNK_CAPACITY;
        for (size_t index = 0; index < LWIR_THERMAL_SIGNATURE_BYTES; ++index) jpeg_workspace.marker[index] = (uint8_t)signature[index];
        write_u32(jpeg_workspace.marker + LWIR_THERMAL_WIDTH_OFFSET, width);
        write_u32(jpeg_workspace.marker + LWIR_THERMAL_HEIGHT_OFFSET, height);
        write_u32(jpeg_workspace.marker + LWIR_THERMAL_DATA_OFFSET, (uint32_t)offset);
        for (size_t index = 0; index < chunk; ++index) {
            jpeg_workspace.marker[LWIR_THERMAL_HEADER_BYTES + index] = frame[plane_bytes + offset + index];
        }
        jpeg_write_marker(encoder, LWIR_THERMAL_APP_MARKER, jpeg_workspace.marker,
                          (unsigned)(LWIR_THERMAL_HEADER_BYTES + chunk));
        offset += chunk;
    }
}

/** @brief Expand one limited-range video component to full range.
 * @param value Component sample from the limited-range source.
 * @param low Lowest legal code (16 for luma).
 * @param span Legal code span: 219 for luma (16..235), 224 for chroma (16..240).
 * @return Full-range 0..255 component.
 * @details JPEG/JFIF assumes full-range YCbCr, so passing TDK video through
 * unchanged would look washed out. Out-of-range inputs clamp. Chroma scales
 * symmetrically around the neutral 128 and truncates toward zero; luma rounds.
 * @note The chroma branch is selected by span == 224 and ignores @p low,
 * implicitly assuming low == 16.
 */
static uint8_t expand_video_component(uint8_t value, unsigned low, unsigned span)
{
    if (value <= low) return 0;
    if (value >= low + span) return 255;
    if (span == 224) return (uint8_t)(128 + ((int)value - 128) * 255 / 224);
    return (uint8_t)((((unsigned)value - low) * 255U + span / 2U) / span);
}

/** @brief Own the serial codec lifecycle and the ordered marker/scanline stages.
 * @param output Caller-owned stream; flush/sync/close remain with the caller.
 * @param frame Frame already size-checked by lwir_encode_jpeg().
 * @param bytes Exact frame size.
 * @param width Native width.
 * @param height Native height.
 * @param synthetic True for replayed/simulated frames.
 * @param evidence Optional provenance; required for any display orientation.
 * @param render Display policy (never NULL here).
 * @return True when the full JPEG was handed to the stream. On false, errno is
 * EINVAL, ENOTSUP, ENOSPC, EIO, or the renderer's errno; discard the output.
 * @details Validates the image-only and orientation rules before touching the
 * codec, writes markers, then renders and compresses one row at a time so the
 * only image buffer is the single-row workspace.
 */
static bool encode_jpeg(FILE *output, const uint8_t *frame, size_t bytes, unsigned width, unsigned height,
                         bool synthetic, const struct lwir_evidence *evidence, const struct lwir_render_options *render)
{
    if (!render || render->jpeg_quality > 100) { errno = EINVAL; return false; }
    struct lwir_render_range range;
    unsigned display_width, display_height;
    /* volatile and never modified: stays well defined when read after the setjmp below. */
    const volatile bool image_only = evidence && evidence->layout == LWIR_YUYV_IMAGE;
    /* Image-only frames have no temperature plane, so anything thermal is unsupported. */
    if (image_only && (synthetic || render->view != LWIR_SENSOR || render->legend || render->profile || render->denoise_raw ||
        render->palette != LWIR_WHITE_HOT || evidence->statistics || evidence->device_state || evidence->raw_path)) {
        errno = ENOTSUP;
        return false;
    }
    /* Oriented display pixels require native/display provenance; never publish an ambiguous mapping. */
    if (!evidence && (render->rotation || render->mirror || render->flip)) { errno = EINVAL; return false; }
    if (!lwir_render_dimensions(width, height, render, &display_width, &display_height) ||
        !lwir_render_prepare(frame, bytes, width, height, render, &range)) return false;
    struct jpeg_compress_struct *encoder = &jpeg_workspace.encoder;
    /* Zeroing first makes jpeg_destroy_compress() safe even if a failure jumps
     * back before jpeg_create_compress() finished. */
    *encoder = (struct jpeg_compress_struct){0};
    encoder->err = jpeg_std_error(&jpeg_workspace.error);
    jpeg_workspace.error.error_exit = jpeg_failure;
    /* Codec failures land here with errno = EIO. Only state not modified after
     * this point (the encoder pointer) or declared volatile may be relied on. */
    if (setjmp(jpeg_workspace.recovery) != 0) {
        jpeg_destroy_compress(encoder);
        return false;
    }
    jpeg_create_compress(encoder);
    jpeg_stdio_dest(encoder, output);
    encoder->image_width = display_width;
    encoder->image_height = display_height;
    encoder->input_components = 3;
    /* The renderer already emits YCbCr rows, so libjpeg skips any RGB conversion pass. */
    encoder->in_color_space = JCS_YCbCr;
    jpeg_set_defaults(encoder);
    /* force_baseline keeps quantization tables 8-bit; quality 0 means legacy 100. */
    jpeg_set_quality(encoder, render->jpeg_quality ? (int)render->jpeg_quality : 100, TRUE);
    /* Standard Huffman tables avoid an extra statistics pass, trading slightly larger files for speed. */
    encoder->optimize_coding = FALSE;
    /* Force 4:4:4: chroma subsampling would smear one- or two-pixel thermal edges. */
    for (size_t component = 0; component < 3; ++component) {
        encoder->comp_info[component].h_samp_factor = 1;
        encoder->comp_info[component].v_samp_factor = 1;
    }
    jpeg_start_compress(encoder, TRUE);
    /* All markers must be written after start and before the first scanline. */
    const int provenance_error = write_provenance_markers(encoder, width, height, display_width, display_height,
                                                         synthetic, evidence, render, &range);
    if (provenance_error != 0) {
        /* Not a codec error, so no longjmp happened: tear down here and keep the specific errno. */
        jpeg_destroy_compress(encoder);
        errno = provenance_error;
        return false;
    }
    if (!image_only) write_thermal_markers(encoder, frame, width, height, synthetic);
    while (encoder->next_scanline < encoder->image_height) {
        if (!lwir_render_row(frame, bytes, width, height, encoder->next_scanline,
                              render, &range, jpeg_workspace.row, sizeof(jpeg_workspace.row))) {
            jpeg_destroy_compress(encoder); return false;
        }
        if (image_only) {
            /* TDK video is limited range (Y 16..235, Cb/Cr 16..240); JPEG expects full range. */
            for (size_t column = 0; column < display_width; ++column) {
                jpeg_workspace.row[column * 3] = expand_video_component(jpeg_workspace.row[column * 3], 16, 219);
                for (size_t component = 1; component < 3; ++component)
                    jpeg_workspace.row[column * 3 + component] = expand_video_component(jpeg_workspace.row[column * 3 + component], 16, 224);
            }
        }
        JSAMPROW row = jpeg_workspace.row;
        /* Defensive: a short write is treated as I/O failure rather than retried. */
        if (jpeg_write_scanlines(encoder, &row, 1) != 1) {
            jpeg_destroy_compress(encoder);
            errno = EIO;
            return false;
        }
    }
    /* Write errors while flushing the codec also longjmp to the recovery point. */
    jpeg_finish_compress(encoder);
    jpeg_destroy_compress(encoder);
    return true;
}

/** @brief Open the directory that will contain @p path, for fsync after linking.
 * @param path Destination path; a name without '/' resolves to ".", "/name" to "/".
 * @return Read-only directory descriptor (close-on-exec), or -1 with errno set;
 * ENAMETOOLONG when the directory part exceeds PATH_MAX.
 * @details Copies the directory part with a bounded loop instead of memcpy.
 */
static int open_parent(const char *path)
{
    char directory[PATH_MAX];
    const char *separator = strrchr(path, '/');
    const size_t length = separator ? (size_t)(separator - path) : 1;
    if (length >= sizeof(directory)) { errno = ENAMETOOLONG; return -1; }
    if (!separator) { directory[0] = '.'; directory[1] = '\0'; }
    else if (length == 0) { directory[0] = '/'; directory[1] = '\0'; }
    else { for (size_t index = 0; index < length; ++index) directory[index] = path[index]; directory[length] = '\0'; }
    return open(directory, O_RDONLY | O_DIRECTORY | O_CLOEXEC);
}

/** @brief Report the bundled codec; see lwir_image.h. */
const char *lwir_jpeg_provider(void)
{
    return "ijg-9f";
}

/** @brief Validate the frame size for its layout, then encode; see lwir_image.h.
 * @details Image-only frames are half the combined size. Layout values beyond
 * the known enum are rejected rather than guessed. A NULL render becomes the
 * plain sensor view with quality 0, i.e. legacy 100.
 */
bool lwir_encode_jpeg(FILE *output, const uint8_t *frame, size_t bytes, unsigned width, unsigned height,
                      bool synthetic, const struct lwir_evidence *evidence, const struct lwir_render_options *render)
{
    const size_t expected = lwir_frame_bytes(width, height) / (evidence && evidence->layout == LWIR_YUYV_IMAGE ? 2U : 1U);
    if (!output || !frame || !expected || bytes != expected || (evidence && evidence->layout > LWIR_YUYV_IMAGE)) {
        errno = EINVAL; return false;
    }
    const struct lwir_render_options sensor = {.view = LWIR_SENSOR};
    return encode_jpeg(output, frame, bytes, width, height, synthetic, evidence, render ? render : &sensor);
}

/** @brief Durably publish a JPEG or raw companion under a new name.
 * @param path Final destination; must not exist.
 * @param frame Frame to encode or copy.
 * @param bytes Exact frame size (half size for image-only JPEG output).
 * @param width Native width.
 * @param height Native height.
 * @param synthetic True for replayed/simulated frames (JPEG only).
 * @param raw True writes @p frame verbatim instead of encoding a JPEG.
 * @param evidence Optional provenance (JPEG only).
 * @param render Optional display policy (JPEG only).
 * @return True when the file is complete, named and the directory synced.
 * On false, errno holds the first failure (later cleanup does not overwrite it)
 * and @p path does not exist: a name this call already linked is removed again.
 * @details Sequence: mkstemp "PATH.part.XXXXXX" -> write -> fflush + fsync ->
 * link() to @p path -> fsync directory -> unlink temporary -> fsync directory.
 * link() fails with EEXIST instead of replacing an existing file (unlike
 * rename()), so publication is atomic and never overwrites. The temporary is
 * removed on every path. Because link() refuses existing names, a name removed
 * on rollback is always one this call created, never someone else's file.
 * @note mkstemp creates the file owner-only (0600); the linked name keeps that mode.
 */
static bool publish(const char *path, const uint8_t *frame, size_t bytes,
                     unsigned width, unsigned height, bool synthetic, bool raw,
                     const struct lwir_evidence *evidence, const struct lwir_render_options *render)
{
    const size_t expected = lwir_frame_bytes(width, height) / (!raw && evidence && evidence->layout == LWIR_YUYV_IMAGE ? 2U : 1U);
    if (path == NULL || path[0] == '\0' || !frame || !expected || bytes != expected) {
        errno = EINVAL;
        return false;
    }
    char temporary[PATH_MAX];
    const int length = snprintf(temporary, sizeof(temporary), "%s.part.XXXXXX", path);
    if (length < 0 || (size_t)length >= sizeof(temporary)) {
        errno = ENAMETOOLONG;
        return false;
    }
    /* Sibling of the destination: same filesystem, so link() cannot fail with EXDEV. */
    const int descriptor = mkstemp(temporary);
    if (descriptor < 0) return false;
    /* Open the directory before writing anything, so this failure costs no I/O. */
    const int directory = open_parent(path);
    if (directory < 0) { const int saved_errno = errno; close(descriptor); unlink(temporary); errno = saved_errno; return false; }
    FILE *output = fdopen(descriptor, "wb");
    if (output == NULL) {
        const int saved_errno = errno;
        close(descriptor);
        close(directory);
        unlink(temporary);
        errno = saved_errno;
        return false;
    }
    bool success = raw ? fwrite(frame, 1, bytes, output) == bytes :
        lwir_encode_jpeg(output, frame, bytes, width, height, synthetic, evidence, render);
    /* Data must be durable before the name becomes visible. */
    if (success && fflush(output) != 0) success = false;
    if (success && fsync(descriptor) != 0) success = false;
    /* Capture the first failure now; later cleanup calls may overwrite errno. */
    int saved_errno = errno;
    if (fclose(output) != 0 && success) {
        success = false;
        saved_errno = errno;
    }
    /* No-overwrite atomic visibility: link() refuses an existing destination. */
    bool linked = false;
    if (success && link(temporary, path) != 0) {
        success = false;
        saved_errno = errno;
    } else if (success) {
        linked = true;
    }
    /* Persist the new directory entry before reporting success. */
    if (success && fsync(directory) != 0) { success = false; saved_errno = errno; }
    /* The final name already holds the complete file, so a leftover temporary is
     * only a warning. */
    if (unlink(temporary) != 0 && success) {
        fprintf(stderr, "LWIR: published %s, but could not remove %s: %s\n", path, temporary, strerror(errno));
    }
    /* Second sync persists removal of the temporary entry. */
    if (success && fsync(directory) != 0) { success = false; saved_errno = errno; }
    /* A failure report must not leave an unconfirmed file under the final name. */
    if (!success && linked && unlink(path) == 0) fsync(directory);
    if (close(directory) != 0 && success) { success = false; saved_errno = errno; unlink(path); }
    errno = saved_errno;
    return success;
}

/** @brief Remove a file this process published and persist the removal; see lwir_image.h. */
bool lwir_remove_published(const char *path)
{
    if (!path || !path[0]) { errno = EINVAL; return false; }
    const int directory = open_parent(path);
    if (directory < 0) return false;
    bool success = unlink(path) == 0 && fsync(directory) == 0;
    const int saved_errno = errno;
    if (close(directory) != 0) success = false;
    errno = saved_errno;
    return success;
}

/** @brief Publish the sensor view without evidence; see lwir_image.h. */
bool lwir_write_jpeg(const char *path, const uint8_t *frame, size_t bytes,
                      unsigned width, unsigned height, bool synthetic)
{
    return publish(path, frame, bytes, width, height, synthetic, false, NULL, NULL);
}

/** @brief Publish the sensor view with provenance; see lwir_image.h. */
bool lwir_write_jpeg_evidence(const char *path, const uint8_t *frame, size_t bytes,
                               unsigned width, unsigned height, bool synthetic,
                               const struct lwir_evidence *evidence)
{
    return publish(path, frame, bytes, width, height, synthetic, false, evidence, NULL);
}

/** @brief Publish the combined frame verbatim; see lwir_image.h. */
bool lwir_write_raw(const char *path, const uint8_t *frame, size_t bytes, unsigned width, unsigned height)
{
    return publish(path, frame, bytes, width, height, false, true, NULL, NULL);
}

/** @brief Publish a rendered view with provenance; see lwir_image.h. */
bool lwir_write_jpeg_rendered(const char *path, const uint8_t *frame, size_t bytes,
                               unsigned width, unsigned height, bool synthetic,
                               const struct lwir_evidence *evidence, const struct lwir_render_options *render)
{
    return publish(path, frame, bytes, width, height, synthetic, false, evidence, render);
}