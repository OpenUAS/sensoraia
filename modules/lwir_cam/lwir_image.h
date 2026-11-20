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

#ifndef SENSORAIA_LWIR_IMAGE_H
#define SENSORAIA_LWIR_IMAGE_H
/** @file
 * @brief JPEG encoding and durable no-overwrite publication of LWIR frames.
 *
 * @details Final stage of the LWIR capture pipeline. Acquisition and frame
 * arithmetic (lwir_frame.h) yield an immutable frame, the renderer
 * (lwir_render.h) converts it into display rows, and this module wraps those
 * rows in a JPEG together with the evidence needed to interpret it later.
 * Frame arithmetic never depends on this module.
 *
 * Input frames:
 * - Combined (LWIR_COMBINED): packed YUYV display plane followed by an equally
 *   sized little-endian Kelvin*64 temperature plane; see lwir_frame_bytes().
 * - Image-only (LWIR_YUYV_IMAGE, TDK device): the limited-range YUYV plane
 *   alone, half the combined size, with no temperatures.
 *
 * JPEG output, produced by the bundled IJG libjpeg 9f (lwir_jpeg_provider()):
 * - APP14 "LWIREV02" key=value provenance text, when evidence is supplied.
 * - APP1 EXIF holding the sensor-capture record, for live frames only.
 * - APP15 chunks carrying the exact native temperature plane, signed
 *   "LWIRRAW1" (live) or "LWIRSIM1" (synthetic replay); see lwir_container.h.
 * - Sequential scan, 4:4:4 sampling, quality 1..100 (0 means legacy 100).
 *
 * Invariants:
 * - Temperatures survive bit-exact at any JPEG quality; only the display is lossy.
 * - Image-only frames carry no thermal markers and are expanded from limited
 *   to full-range YCbCr before encoding.
 * - Publication never replaces an existing path and is durable on success.
 * - Encoding uses one static workspace: single-threaded and not re-entrant.
 */
#include "lwir_frame.h"
#include "sensor_capture.h"
#include "lwir_state.h"
#include <stdio.h>
#ifdef __cplusplus
extern "C" {
#endif

/** @brief Borrowed provenance for one publication; strings must outlive encoding.
 * @details User labels, post-frame state and unknown exposure timing stay distinct.
 * Every field is copied into the APP14 text as-is; nothing here is validated
 * against the device. NULL strings are recorded as fixed placeholders so a
 * reader can tell "not supplied" from an empty value.
 */
struct lwir_evidence {
    struct sensor_capture capture;  /**< Boot id and sequence go to APP14; the whole record becomes the EXIF sensor-capture tag on live frames. */
    uint64_t request_us;            /**< Host time the capture was requested, microseconds. */
    uint64_t arrival_us;            /**< Host time the frame arrived, microseconds; arrival is not exposure time. */
    uint64_t request_id;            /**< Caller request identifier, recorded verbatim. */
    uint64_t kernel_monotonic_us;   /**< Kernel buffer timestamp; recorded but explicitly not a qualified exposure time. */
    unsigned kernel_timestamp_flags; /**< Kernel buffer timestamp flags, recorded verbatim. */
    const char *serial;             /**< Camera serial; NULL is recorded as "unknown". */
    const char *gain;               /**< Operator gain label, not a device readback; NULL is recorded as "unknown". */
    const char *raw_path;           /**< Raw companion path; NULL is recorded as "none". Must be NULL for image-only frames. */
    const char *rendering;          /**< Rendering description; NULL is recorded as "sensor". */
    const struct lwir_measurement *measurement;     /**< Optional measurement geometry; written only together with statistics. */
    const struct lwir_thresholds *thresholds;       /**< Optional coverage band; NULL is recorded as disabled. */
    const struct lwir_temperature_stats *statistics; /**< Optional measurement results; must be NULL for image-only frames. */
    const struct lwir_device_state *device_state;   /**< Optional post-frame device snapshot; dropped for synthetic output, must be NULL for image-only frames. */
    unsigned warn_percent;          /**< Advisory coverage warning percentage, recorded verbatim. */
    char frame_sha256[65];          /**< NUL-terminated hex SHA-256 of the frame; readers require 64 lowercase hex digits. */
    enum lwir_frame_layout layout;  /**< LWIR_YUYV_IMAGE selects image-only encoding and validation. */
};
/* Defined in lwir_render.h; only pointers are used here. */
struct lwir_render_options;

/** @brief Encode into an existing stream without owning its publication policy.
 * @param output Caller-owned writable stream; caller handles flush/sync/close.
 * @param frame Immutable combined frame, or packed YUYV when evidence selects LWIR_YUYV_IMAGE.
 * @param bytes Exact frame size.
 * @param width Native width.
 * @param height Native height.
 * @param synthetic Whether thermal evidence is simulated.
 * @param evidence Optional capture provenance; required for display orientation.
 * @param render Optional display policy; NULL keeps the sensor image.
 * @return True on successful encoding. Discard partial output on failure.
 * @details Serial encoder, bounded scanline workspace. The codec may
 * allocate internally. Image-only evidence permits sensor-view rendering only,
 * expands limited-range YUYV for JPEG, and emits no thermal markers. Otherwise
 * JPEG display is lossy; embedded native temperatures are exact.
 *
 * Failures set errno: EINVAL for bad arguments, quality above 100 or display
 * orientation without evidence; ENOTSUP for image-only frames combined with
 * thermal rendering, synthetic output or thermal evidence; ENOSPC when the
 * provenance text does not fit its fixed buffer; EIO for codec failures.
 * Renderer failures keep the renderer's errno.
 * @note A NULL render, or jpeg_quality 0, encodes at the legacy quality 100.
 * @warning Uses process-wide static encoder state: never call concurrently or
 * from a signal handler.
 */
bool lwir_encode_jpeg(FILE *output, const uint8_t *frame, size_t bytes, unsigned width, unsigned height,
                      bool synthetic, const struct lwir_evidence *evidence, const struct lwir_render_options *render);
/** @brief Identify the compiled codec provider for diagnostics and benchmark logs.
 * @return Static string "ijg-9f" (bundled IJG libjpeg 9f); never NULL, never freed.
 */
const char *lwir_jpeg_provider(void);
/** @brief Remove a file previously published by this process and sync its directory.
 * @param path File name returned as published by one of the writers below.
 * @return True when the name was removed and the removal persisted; errno otherwise.
 * @details Used to undo a raw companion when the matching JPEG could not be
 * published, so a failed capture leaves neither file behind.
 * @warning Only pass names this process published; it removes whatever the path names.
 */
bool lwir_remove_published(const char *path);

/** @brief Publish a sensor-view JPEG with exact native thermal markers.
 * @param path New destination, never an existing file.
 * @param frame Immutable combined frame.
 * @param bytes Exact combined size.
 * @param width Native width.
 * @param height Native height.
 * @param synthetic Select simulated versus live thermal evidence.
 * @return True after exclusive publication and directory sync. On failure
 * errno holds the first error; EEXIST means the destination already existed.
 * @details Writes a private temporary "PATH.part.XXXXXX", syncs it, then
 * hard-links it to @p path so the name appears atomically and only when
 * complete. Without evidence no APP14/EXIF is written, but the thermal
 * markers still are.
 * @warning A failure after linking may leave a final file. Inspect; never auto-retry.
 */
bool lwir_write_jpeg(const char *path, const uint8_t *frame, size_t bytes,
                      unsigned width, unsigned height, bool synthetic);
/** @brief Publish the sensor view plus borrowed capture provenance.
 * @param path New destination, never an existing file.
 * @param frame Immutable combined frame or image-only YUYV.
 * @param bytes Exact byte count for the selected layout.
 * @param width Native width.
 * @param height Native height.
 * @param synthetic True for simulated rather than live evidence.
 * @param evidence Optional capture record; simulated output strips live state.
 * @return True after exclusive publication and directory sync.
 * @details Uses the same temporary-file and no-overwrite publication
 * contract as lwir_write_jpeg().
 * @warning A failure after linking may leave a final file; inspect before retrying.
 * @note Image-only evidence (LWIR_YUYV_IMAGE) expects the half-size YUYV frame.
 */
bool lwir_write_jpeg_evidence(const char *path, const uint8_t *frame, size_t bytes,
                               unsigned width, unsigned height, bool synthetic,
                               const struct lwir_evidence *evidence);
/** @brief Publish an exact combined frame with the same no-overwrite contract.
 * @param path New raw destination.
 * @param frame Immutable combined frame.
 * @param bytes Exact combined size.
 * @param width Native width.
 * @param height Native height.
 * @return True after publication; ambiguous failures must not be retried blindly.
 * @details Bytes are written verbatim through the same temporary/link/sync
 * sequence as the JPEG writers, so the companion is equally durable and never
 * overwrites. Always expects the full combined size.
 */
bool lwir_write_raw(const char *path, const uint8_t *frame, size_t bytes,
                     unsigned width, unsigned height);
/** @brief Publish a rendered view without altering original temperature samples.
 * @param path New destination, never an existing file.
 * @param frame Immutable native frame.
 * @param bytes Exact frame byte count for its layout.
 * @param width Native width before orientation.
 * @param height Native height before orientation.
 * @param synthetic True for simulated rather than live evidence.
 * @param evidence Optional capture record, required when orientation changes.
 * @param render Display policy; orientation requires evidence for its native mapping.
 * @return True after exclusive publication and directory sync.
 * @details Use the same no-overwrite publication as lwir_write_jpeg_evidence().
 * @warning A failure after linking may leave a final file; inspect before retrying.
 * @note Rendering changes only display pixels; the APP15 temperature plane is
 * always the unmodified native data.
 */
bool lwir_write_jpeg_rendered(const char *path, const uint8_t *frame, size_t bytes,
                               unsigned width, unsigned height, bool synthetic,
                               const struct lwir_evidence *evidence, const struct lwir_render_options *render);

#ifdef __cplusplus
}
#endif
#endif
