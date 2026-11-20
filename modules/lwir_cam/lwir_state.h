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
 * @file lwir_state.h
 * @brief Tiny1-C vendor mailbox: bounded device-state snapshots and verified setting controls.
 * @details The Tiny1-C exposes a command mailbox through a small register window:
 * - status byte at 0x0200: 0 = idle/done, bit0 = busy, bits 0xfc = error class
 *   (0x08/0x10 unsupported, 0x14 invalid, anything else I/O error);
 * - standard 8-byte command header written at 0x1d00 (the write executes it);
 * - extended 16-byte header: bytes 0..7 written to 0x9d00 are only held,
 *   bytes 8..15 written to 0x1d08 trigger execution;
 * - reply read at 0x1d00 + header length (0x1d08 standard, 0x1d10 extended).
 *
 * This module implements only that protocol. Transport is injected through
 * struct lwir_state_io: the native driver backs it with Linux USBFS control
 * transfers, host tests back it with fakes. No heap, no memcpy, no global state.
 *
 * Two independent entry families exist:
 * - lwir_state_read(): read-only evidence snapshot taken after a capture,
 *   bounded by #LWIR_STATE_BUDGET_MS and explicitly not frame-atomic;
 * - lwir_state_control() / lwir_state_control_read(): one targeted write or
 *   read with read-back verification and a submitted-versus-verified result.
 *
 * Value ranges and units come from the vendor headers; they describe what the
 * device accepts, not measured radiometric accuracy.
 */
#ifndef LWIRCAM_LWIR_STATE_H
#define LWIRCAM_LWIR_STATE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Wall-time budget for one whole lwir_state_read() snapshot, in milliseconds.
 * @details Keeps post-capture latency bounded. On real hardware all fields do not
 * always fit: later fields (AGC, mirror/flip) may time out or stay skipped.
 */
#define LWIR_STATE_BUDGET_MS 250U

/**
 * @brief Index of each queried device-state field; also the read order.
 * @details Fields are read in this order, so earlier entries are the most likely
 * to complete within #LWIR_STATE_BUDGET_MS. Two-byte values are big-endian;
 * ranges below are those accepted by lwir_state_read() validation.
 * - #LWIR_STATE_GAIN: gain select, 0 = low, 1 = high (TPD params, cmd 0x14/0x85 #5).
 * - #LWIR_STATE_FW_BUILD: firmware build, 64-byte device-info string (0x05/0x84 #5).
 * - #LWIR_STATE_FW_DATE: firmware date, 64-byte device-info string (0x05/0x84 #1).
 * - #LWIR_STATE_CHIP: chip identification, 64-byte device-info string (0x05/0x84 #0).
 * - #LWIR_STATE_SHUTTER: shutter status, two bytes each 0 or 1 (0x0c/0x83).
 * - #LWIR_STATE_DISTANCE: target distance, 128 units per metre, 0..32767 (TPD #0).
 * - #LWIR_STATE_REFLECTED_K: reflected temperature in kelvin, 230..900 (TPD #1).
 * - #LWIR_STATE_AMBIENT_K: ambient temperature in kelvin, 230..900 (TPD #2).
 * - #LWIR_STATE_EMISSIVITY: emissivity in 1/128, 1..128 (TPD #3).
 * - #LWIR_STATE_TRANSMITTANCE: atmospheric transmittance in 1/128, 1..128 (TPD #4).
 * - #LWIR_STATE_TNR: temporal noise reduction level, 0..3 (image params 0x14/0x83 #0).
 * - #LWIR_STATE_SNR: spatial noise reduction level, 0..3 (image #1).
 * - #LWIR_STATE_DDE: digital detail enhancement level, 0..6 (image #2).
 * - #LWIR_STATE_BRIGHTNESS: display brightness, 0..255 (image #3).
 * - #LWIR_STATE_CONTRAST: display contrast, 0..255 (image #4).
 * - #LWIR_STATE_AGC_MODE: automatic gain control mode, 0..5 (image #5).
 * - #LWIR_STATE_AGC_MAX_GAIN: AGC maximum gain, 0..255 (image #6).
 * - #LWIR_STATE_AGC_BOS: AGC brightness offset, 0..255 (image #7).
 * - #LWIR_STATE_AGC_ENABLE: AGC enable, 0 or 1 (image #8).
 * - #LWIR_STATE_MIRROR_FLIP: mirror/flip combination, 0..3 (image #9).
 * - #LWIR_STATE_FIELDS: number of fields, not a field.
 */
enum lwir_state_field {
    LWIR_STATE_GAIN, LWIR_STATE_FW_BUILD, LWIR_STATE_FW_DATE, LWIR_STATE_CHIP,
    LWIR_STATE_SHUTTER, LWIR_STATE_DISTANCE, LWIR_STATE_REFLECTED_K,
    LWIR_STATE_AMBIENT_K, LWIR_STATE_EMISSIVITY, LWIR_STATE_TRANSMITTANCE,
    LWIR_STATE_TNR, LWIR_STATE_SNR, LWIR_STATE_DDE, LWIR_STATE_BRIGHTNESS,
    LWIR_STATE_CONTRAST, LWIR_STATE_AGC_MODE, LWIR_STATE_AGC_MAX_GAIN,
    LWIR_STATE_AGC_BOS, LWIR_STATE_AGC_ENABLE, LWIR_STATE_MIRROR_FLIP,
    LWIR_STATE_FIELDS
};

/**
 * @brief Outcome of one field read or one control operation.
 * @details The numeric order is part of the evidence format: lwir_state_format()
 * maps it to text by index.
 * - #LWIR_STATE_NOT_REQUESTED: zero value; nothing was attempted.
 * - #LWIR_STATE_OK: reply received and validated (or setting verified).
 * - #LWIR_STATE_UNSUPPORTED: device status error class 0x08 or 0x10.
 * - #LWIR_STATE_TIMEOUT: transfer returned -7, the budget ran out, or polling
 *   never saw the mailbox go idle.
 * - #LWIR_STATE_IO_ERROR: other negative or short transfer, or any other device
 *   error class.
 * - #LWIR_STATE_INVALID: device error class 0x14, implausible/out-of-range reply,
 *   unusable arguments/callbacks, or a control read-back mismatch.
 * - #LWIR_STATE_BUSY: the mailbox was not idle when a snapshot started; reported
 *   on the first field only, nothing was sent.
 * - #LWIR_STATE_SKIPPED: the snapshot stopped before reaching this field.
 * - #LWIR_STATE_CANCELLED: transfer returned -10 or the pause callback refused.
 */
enum lwir_state_status {
    LWIR_STATE_NOT_REQUESTED, LWIR_STATE_OK, LWIR_STATE_UNSUPPORTED,
    LWIR_STATE_TIMEOUT, LWIR_STATE_IO_ERROR, LWIR_STATE_INVALID,
    LWIR_STATE_BUSY, LWIR_STATE_SKIPPED, LWIR_STATE_CANCELLED
};

/** @brief Raw result of one mailbox query, kept verbatim as evidence. */
struct lwir_state_value {
    enum lwir_state_status status; ///< Outcome; data is meaningful only for #LWIR_STATE_OK.
    /** Reply bytes exactly as received: big-endian for two-byte values; 64-byte
     * device-info strings are not guaranteed to be NUL-terminated. */
    uint8_t data[64];
    uint8_t length; ///< Valid bytes in data (2 or 64); 0 unless a reply was accepted.
    uint8_t device_status; ///< Last status byte read from 0x0200; 0 if none was read.
    /** Return value of the last failing transfer callback (negative code or short
     * byte count); 0 when no transfer failed. */
    int transport_error;
};

/** @brief One post-capture snapshot of all #lwir_state_field values. */
struct lwir_device_state {
    struct lwir_state_value values[LWIR_STATE_FIELDS]; ///< Indexed by enum lwir_state_field.
    uint64_t started_us; ///< now_us() before the first transfer; clock domain is the io provider's.
    uint64_t finished_us; ///< now_us() after the last attempt; 0 if the io table was unusable.
    bool requested; ///< True once lwir_state_read() ran; false means no device state exists.
};

/**
 * @brief Transport and clock callbacks driving the mailbox protocol.
 * @details All callbacks receive @c context unchanged. All three function
 * pointers are required; a table with any NULL is rejected as invalid.
 */
struct lwir_state_io {
    void *context; ///< Opaque provider state (USBFS session, test fake).
    /**
     * @brief Move @p length bytes between @p data and mailbox @p address.
     * @param input True reads from the device into @p data; false writes @p data.
     * @param timeout_ms Per-transfer limit, never 0 and at most 50 ms.
     * @return Bytes transferred. Any value other than @p length is a failure:
     * -7 means timeout, -10 means cancelled, anything else is an I/O error.
     * @warning A failed or late write may still have reached the device.
     */
    int (*transfer)(void *context, bool input, uint16_t address, uint8_t *data,
                    unsigned length, unsigned timeout_ms);
    /** @brief Current time in microseconds; must be monotonic for budgets to hold. */
    uint64_t (*now_us)(void *context);
    /** @brief Sleep about @p milliseconds; return false to cancel the whole operation. */
    bool (*pause)(void *context, unsigned milliseconds);
};

/**
 * @brief Settings that lwir_state_control() may change.
 * - #LWIR_CONTROL_GAIN: gain select, value 0 (low) or 1 (high); 5 s budget because
 *   the camera may run internal correction after switching.
 * - #LWIR_CONTROL_EMISSIVITY: emissivity in 1/128, value 1..128.
 * - #LWIR_CONTROL_AGC_MODE: AGC mode, value 0..5.
 * - #LWIR_CONTROL_AGC_ENABLE: AGC enable, value 0 or 1.
 * - #LWIR_CONTROL_FFC: flat-field correction (vendor OOC+B combined update, not a
 *   shutter open/close); value must be 0; acknowledgement only, never verified.
 */
enum lwir_control_kind {
    LWIR_CONTROL_GAIN, LWIR_CONTROL_EMISSIVITY, LWIR_CONTROL_AGC_MODE,
    LWIR_CONTROL_AGC_ENABLE, LWIR_CONTROL_FFC
};

/** @brief One requested setting change; validate with lwir_control_valid(). */
struct lwir_control_request {
    enum lwir_control_kind kind; ///< Setting to change.
    uint16_t value; ///< New value in the units of the matching #lwir_state_field.
};

/**
 * @brief Outcome of lwir_state_control() or lwir_state_control_read().
 * @details @c submitted and @c verified are deliberately separate:
 * - @c submitted is set just before the executing (trigger) write. Once true, the
 *   device may have applied the command even if the call failed, so callers must
 *   never replay automatically; read the setting back instead.
 * - @c verified is true only when an exact read-back matched (setters) or the
 *   read-back succeeded (control_read). FFC never sets it.
 */
struct lwir_control_result {
    struct lwir_state_value reply; ///< Status/transport detail of the last step reached.
    bool submitted, verified; ///< See struct description; both false after early rejection.
    uint16_t readback; ///< Value read back; meaningful only once a read-back reply arrived.
};

/**
 * @brief Check a control request against the vendor value ranges.
 * @param request Request to check; NULL is rejected.
 * @return True if the kind is known and the value is in range (FFC requires 0).
 */
bool lwir_control_valid(const struct lwir_control_request *request);
/**
 * @brief Read one controllable setting without writing anything.
 * @details Waits for idle, sends one extended getter and reads the 2-byte reply,
 * all within a 1 s budget.
 * @param io Transport; all callbacks required.
 * @param kind Setting to read; #LWIR_CONTROL_FFC has no value and is rejected.
 * @param result Always reset when non-NULL; on success readback holds the value.
 * @return True with verified set on success; false with reply.status describing
 * the failure (#LWIR_STATE_INVALID for bad arguments or out-of-range values).
 */
bool lwir_state_control_read(const struct lwir_state_io *io, enum lwir_control_kind kind,
                              struct lwir_control_result *result);
/**
 * @brief Apply one setting or trigger FFC, then confirm it.
 * @details Setters write the value and verify it by exact read-back within 1 s
 * (gain: 5 s). FFC waits 3 s and an idle mailbox within 5 s, and reports
 * acknowledgement only.
 * @param io Transport; all callbacks required.
 * @param request Validated with lwir_control_valid().
 * @param result Always reset when non-NULL; see struct lwir_control_result.
 * @return True when completed (and verified for setters). False otherwise;
 * check result->submitted before deciding anything about device state.
 * @warning Never retry blindly after a false return with submitted set.
 */
bool lwir_state_control(const struct lwir_state_io *io, const struct lwir_control_request *request,
                         struct lwir_control_result *result);
/**
 * @brief Take a best-effort, read-only snapshot of all device-state fields.
 * @details Total time is bounded by #LWIR_STATE_BUDGET_MS. Unsupported or invalid
 * fields are recorded and the next field is tried; timeout, I/O error or
 * cancellation stops the snapshot and leaves the rest #LWIR_STATE_SKIPPED.
 * @param io Transport; an unusable table marks field 0 #LWIR_STATE_INVALID.
 * @param state Fully overwritten; NULL is ignored.
 * @note Not frame-atomic: values are read after the frame and may differ from
 * those in effect at exposure.
 * @warning An aborted command may leave the mailbox busy, so the next snapshot
 * can report #LWIR_STATE_BUSY.
 */
void lwir_state_read(const struct lwir_state_io *io, struct lwir_device_state *state);
/**
 * @brief Decode a validated two-byte big-endian field.
 * @param state Snapshot to read.
 * @param field Field index.
 * @param value Output, written only on success.
 * @return False for NULL arguments, an out-of-range field, a non-OK status or a
 * non-two-byte (string) field.
 */
bool lwir_state_u16(const struct lwir_device_state *state, enum lwir_state_field field, uint16_t *value);
/**
 * @brief Render a snapshot as key=value evidence lines for the JPEG APP14 segment.
 * @param state Snapshot; NULL (e.g. synthetic imagery) yields a not_requested header only.
 * @param output Destination; NUL-terminated on success.
 * @param capacity Bytes available in output.
 * @return Characters written excluding the NUL, or 0 on overflow, NULL/zero-sized
 * output or a corrupt snapshot; output may then hold a truncated prefix.
 */
size_t lwir_state_format(const struct lwir_device_state *state, char *output, size_t capacity);

#ifdef __cplusplus
}
#endif
#endif