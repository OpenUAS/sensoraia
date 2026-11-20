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
 * @file lwir_state.c
 * @brief Tiny1-C vendor mailbox protocol: snapshot reads, controls and evidence text.
 * @details Transport-agnostic implementation of the protocol summarised in
 * lwir_state.h. Every operation is driven by an absolute deadline: each transfer
 * timeout is the remaining budget capped at 50 ms, and every poll loop is bounded
 * both by an iteration cap and by that deadline, so a stuck device cannot stall
 * the capture path. Storage is caller-owned or on the stack; no heap, no memcpy.
 */
#include "lwir_state.h"

#include <stdio.h>

/**
 * @brief One mailbox getter, shared by snapshot reads and control read-back.
 * @details @c command 0x14 selects the extended 16-byte header; others use the
 * 8-byte standard header. @c parameter selects the item within the command,
 * @c length is the reply size requested in the last header byte, and
 * @c minimum/@c maximum bound two-byte replies (unused for strings/shutter).
 */
struct state_query {
    const char *name;
    uint8_t command;
    uint8_t subcommand;
    uint8_t parameter;
    uint8_t length;
    uint16_t minimum;
    uint16_t maximum;
};

/** @brief Getter table indexed by enum lwir_state_field; names become evidence keys.
 * @details Ranges come from the vendor headers. Order is read order, so the most
 * important evidence (gain, firmware identity) is fetched while budget remains. */
static const struct state_query queries[LWIR_STATE_FIELDS] = {
    {"gain_select", 0x14, 0x85, 5, 2, 0, 1},
    {"fw_build", 0x05, 0x84, 5, 64, 0, 0},
    {"fw_date", 0x05, 0x84, 1, 64, 0, 0},
    {"chip", 0x05, 0x84, 0, 64, 0, 0},
    {"shutter", 0x0c, 0x83, 0, 2, 0, 0},
    {"distance_128_per_m", 0x14, 0x85, 0, 2, 0, 32767},
    {"reflected_k", 0x14, 0x85, 1, 2, 230, 900},
    {"ambient_k", 0x14, 0x85, 2, 2, 230, 900},
    {"emissivity_128", 0x14, 0x85, 3, 2, 1, 128},
    {"transmittance_128", 0x14, 0x85, 4, 2, 1, 128},
    {"tnr", 0x14, 0x83, 0, 2, 0, 3},
    {"snr", 0x14, 0x83, 1, 2, 0, 3},
    {"dde", 0x14, 0x83, 2, 2, 0, 6},
    {"brightness", 0x14, 0x83, 3, 2, 0, 255},
    {"contrast", 0x14, 0x83, 4, 2, 0, 255},
    {"agc_mode", 0x14, 0x83, 5, 2, 0, 5},
    {"agc_max_gain", 0x14, 0x83, 6, 2, 0, 255},
    {"agc_bos", 0x14, 0x83, 7, 2, 0, 255},
    {"agc_enable", 0x14, 0x83, 8, 2, 0, 1},
    {"mirror_flip", 0x14, 0x83, 9, 2, 0, 3}
};

/**
 * @brief Time left before @p deadline, as a per-transfer timeout.
 * @param io Clock provider.
 * @param deadline Absolute deadline in now_us() microseconds.
 * @return Whole milliseconds remaining, capped at 50; 0 when less than 1 ms remains.
 * @note The 50 ms cap keeps one slow transfer from consuming the whole budget.
 */
static unsigned remaining_ms(const struct lwir_state_io *io, uint64_t deadline)
{
    const uint64_t now = io->now_us(io->context);
    if (now >= deadline) return 0;
    const uint64_t remaining = (deadline - now) / 1000U;
    return (unsigned)(remaining > 50U ? 50U : remaining);
}

/**
 * @brief Perform one deadline-bounded mailbox transfer and classify failure.
 * @param io Transport.
 * @param deadline Absolute deadline in now_us() microseconds.
 * @param input True to read from the device, false to write.
 * @param address Mailbox register address.
 * @param data Source or destination buffer of at least @p length bytes.
 * @param length Exact number of bytes expected.
 * @param value Receives status (and transport_error) on failure; untouched on success.
 * @return True only for a complete transfer that also finished before the deadline.
 * @warning A complete transfer that ends after the deadline is reported as
 * #LWIR_STATE_TIMEOUT even though a write may have executed.
 */
static bool transfer(const struct lwir_state_io *io, uint64_t deadline, bool input,
                     uint16_t address, uint8_t *data, unsigned length, struct lwir_state_value *value)
{
    const unsigned timeout = remaining_ms(io, deadline);
    if (!timeout) { value->status = LWIR_STATE_TIMEOUT; return false; }
    const int result = io->transfer(io->context, input, address, data, length, timeout);
    if (result != (int)length) {
        value->transport_error = result;
        value->status = result == -7 ? LWIR_STATE_TIMEOUT : result == -10 ? LWIR_STATE_CANCELLED : LWIR_STATE_IO_ERROR;
        return false;
    }
    /* Re-check after the call: a transfer may block up to its timeout, and the budget must stay hard. */
    if (!remaining_ms(io, deadline)) { value->status = LWIR_STATE_TIMEOUT; return false; }
    return true;
}

/**
 * @brief Plausibility check of one snapshot reply.
 * @param field Field the reply belongs to.
 * @param data Reply bytes, at least the field's query length.
 * @return True if acceptable as evidence.
 * @details 64-byte strings must be neither all zero (no data) nor all 0xff
 * (erased/unprogrammed); shutter bytes must each be 0 or 1; other fields are
 * big-endian and must fall inside the vendor range.
 */
static bool valid_value(enum lwir_state_field field, const uint8_t *data)
{
    const struct state_query *query = &queries[field];
    if (query->length == 64) {
        bool nonempty = false, nonerased = false;
        for (unsigned index = 0; index < 64; ++index) {
            nonempty |= data[index] != 0;
            nonerased |= data[index] != 255;
        }
        return nonempty && nonerased;
    }
    if (field == LWIR_STATE_SHUTTER) return data[0] <= 1 && data[1] <= 1;
    const uint16_t number = (uint16_t)((unsigned)data[0] * 256U + data[1]);
    return number >= query->minimum && number <= query->maximum;
}

/**
 * @brief Read all fields of struct lwir_device_state within #LWIR_STATE_BUDGET_MS.
 * @details Algorithm:
 * 1. Zero the state, mark it requested and every field skipped.
 * 2. Refuse to start unless the mailbox status is idle (0), so a command still
 *    running from an earlier aborted snapshot is not clobbered.
 * 3. For each field in table order: write the command header (standard at
 *    0x1d00, or extended as held half at 0x9d00 plus trigger half at 0x1d08),
 *    poll 0x0200 up to 32 times at <= 10 ms intervals until idle, read the reply
 *    at 0x1d00 + header length and validate it.
 * Unsupported/invalid fields continue with the next field because the device is
 * still responsive; timeout, I/O error and cancellation stop the snapshot since
 * further transfers would only burn budget or hit a busy mailbox.
 * @note Failures are reported only through @p state; field 0 carries
 * #LWIR_STATE_INVALID for an unusable @p io or #LWIR_STATE_BUSY for a busy mailbox.
 */
void lwir_state_read(const struct lwir_state_io *io, struct lwir_device_state *state)
{
    if (!state) return;
    unsigned char *storage = (unsigned char *)state;
    /* Byte loop instead of memset/memcpy per project policy; also clears padding so evidence is deterministic. */
    for (size_t index = 0; index < sizeof(*state); ++index) storage[index] = 0;
    state->requested = true;
    for (unsigned field = 0; field < LWIR_STATE_FIELDS; ++field) state->values[field].status = LWIR_STATE_SKIPPED;
    if (!io || !io->transfer || !io->now_us || !io->pause) { state->values[0].status = LWIR_STATE_INVALID; return; }
    state->started_us = io->now_us(io->context);
    if (state->started_us > UINT64_MAX - LWIR_STATE_BUDGET_MS * 1000U) { state->values[0].status = LWIR_STATE_INVALID; return; }
    const uint64_t deadline = state->started_us + LWIR_STATE_BUDGET_MS * 1000U;
    uint8_t status = 0;
    struct lwir_state_value *first = &state->values[0];
    /* Step 1: only start from an idle mailbox; never queue behind someone else's command. */
    if (!transfer(io, deadline, true, 0x0200, &status, 1, first)) goto finished;
    if (status != 0) { first->status = LWIR_STATE_BUSY; first->device_status = status; goto finished; }
    for (unsigned field = 0; field < LWIR_STATE_FIELDS; ++field) {
        const struct state_query *query = &queries[field];
        struct lwir_state_value *value = &state->values[field];
        uint8_t header[16] = {0}, reply[64] = {0};
        const bool extended = query->command == 0x14;
        const unsigned header_length = extended ? 16U : 8U;
        /* Step 2: the item selector sits at byte 2 (standard) or 3 (extended); the last byte requests the reply length. */
        header[0] = query->command; header[1] = query->subcommand;
        header[extended ? 3 : 2] = query->parameter;
        header[header_length - 1] = query->length;
        if (extended) {
            /* 0x9d00 only holds the first half; the write to 0x1d08 starts execution, so order is mandatory. */
            if (!transfer(io, deadline, false, 0x9d00, header, 8, value) ||
                !transfer(io, deadline, false, 0x1d08, header + 8, 8, value)) break;
        } else if (!transfer(io, deadline, false, 0x1d00, header, 8, value)) break;
        bool complete = false;
        /* Step 3: poll for completion; 32 x 10 ms pauses alone exceed the budget, so the deadline usually ends a hung poll. */
        for (unsigned attempt = 0; attempt < 32; ++attempt) {
            if (!transfer(io, deadline, true, 0x0200, &status, 1, value)) break;
            value->device_status = status;
            if (status == 0) { complete = true; break; }
            /* Non-zero but not busy (e.g. 0x02 alone) is unexpected and treated as an I/O error. */
            if ((status & 0xfcU) || !(status & 1U)) {
                const unsigned error = status & 0xfcU;
                value->status = error == 0x08 || error == 0x10 ? LWIR_STATE_UNSUPPORTED :
                    error == 0x14 ? LWIR_STATE_INVALID : LWIR_STATE_IO_ERROR;
                break;
            }
            unsigned delay = remaining_ms(io, deadline);
            if (!delay) { value->status = LWIR_STATE_TIMEOUT; break; }
            if (delay > 10) delay = 10;
            if (!io->pause(io->context, delay)) { value->status = LWIR_STATE_CANCELLED; break; }
        }
        if (!complete) {
            /* The device rejected this item but answered cleanly, so the mailbox is usable for the next one. */
            if (value->status == LWIR_STATE_UNSUPPORTED || value->status == LWIR_STATE_INVALID) continue;
            /* Still SKIPPED means all 32 polls saw busy. */
            if (value->status == LWIR_STATE_SKIPPED) value->status = LWIR_STATE_TIMEOUT;
            break;
        }
        /* Step 4: the reply follows the header, so its address depends on the header form. */
        if (!transfer(io, deadline, true, (uint16_t)(0x1d00U + header_length), reply, query->length, value)) break;
        if (!valid_value((enum lwir_state_field)field, reply)) { value->status = LWIR_STATE_INVALID; continue; }
        value->status = LWIR_STATE_OK;
        value->length = query->length;
        for (unsigned index = 0; index < query->length; ++index) value->data[index] = reply[index];
    }
finished:
    state->finished_us = io->now_us(io->context);
}

/**
 * @brief Range-check a control request (vendor-header limits).
 * @return True for a known kind with an in-range value; FFC accepts only 0.
 */
bool lwir_control_valid(const struct lwir_control_request *request)
{
    if (!request) return false;
    switch (request->kind) {
    case LWIR_CONTROL_GAIN: case LWIR_CONTROL_AGC_ENABLE: return request->value <= 1;
    case LWIR_CONTROL_EMISSIVITY: return request->value >= 1 && request->value <= 128;
    case LWIR_CONTROL_AGC_MODE: return request->value <= 5;
    case LWIR_CONTROL_FFC: return request->value == 0;
    default: return false;
    }
}

/**
 * @brief Poll the mailbox status until idle, an error class, or the deadline.
 * @param io Transport.
 * @param deadline Absolute deadline in now_us() microseconds.
 * @param reply Receives device_status and the failure status.
 * @return True once status reads 0. False with #LWIR_STATE_UNSUPPORTED,
 * #LWIR_STATE_INVALID or #LWIR_STATE_IO_ERROR for an error class,
 * #LWIR_STATE_CANCELLED, or #LWIR_STATE_TIMEOUT when the deadline or the
 * 512-poll cap is reached; transfer failures keep transfer()'s status.
 * @details Pauses at most 10 ms between polls. The larger cap than the snapshot's
 * 32 polls fits the 1-5 s control budgets.
 */
static bool control_idle(const struct lwir_state_io *io, uint64_t deadline, struct lwir_state_value *reply)
{
    for (unsigned attempt = 0; attempt < 512; ++attempt) {
        uint8_t status = 0;
        if (!transfer(io, deadline, true, 0x0200, &status, 1, reply)) return false;
        reply->device_status = status;
        if (!status) return true;
        if ((status & 0xfcU) || !(status & 1U)) {
            const unsigned error = status & 0xfcU;
            reply->status = error == 0x08 || error == 0x10 ? LWIR_STATE_UNSUPPORTED :
                error == 0x14 ? LWIR_STATE_INVALID : LWIR_STATE_IO_ERROR;
            return false;
        }
        unsigned delay = remaining_ms(io, deadline);
        if (!delay) break;
        if (delay > 10) delay = 10;
        if (!io->pause(io->context, delay)) { reply->status = LWIR_STATE_CANCELLED; return false; }
    }
    reply->status = LWIR_STATE_TIMEOUT;
    return false;
}

/**
 * @brief Map a control to the getter row whose subcommand/parameter it writes.
 * @param kind Control kind.
 * @return Table row, or NULL for FFC/unknown kinds (no readable value).
 * @note Reusing the snapshot table keeps getter, setter and range limits in one place.
 */
static const struct state_query *control_query(enum lwir_control_kind kind)
{
    switch (kind) {
    case LWIR_CONTROL_GAIN: return &queries[LWIR_STATE_GAIN];
    case LWIR_CONTROL_EMISSIVITY: return &queries[LWIR_STATE_EMISSIVITY];
    case LWIR_CONTROL_AGC_MODE: return &queries[LWIR_STATE_AGC_MODE];
    case LWIR_CONTROL_AGC_ENABLE: return &queries[LWIR_STATE_AGC_ENABLE];
    default: return NULL;
    }
}

/**
 * @brief Read one extended two-byte setting back from the device.
 * @param io Transport.
 * @param deadline Absolute deadline in now_us() microseconds.
 * @param query Getter row (extended command 0x14).
 * @param result Receives reply bytes, length 2 and the decoded readback.
 * @return True for an in-range value. False with the failing step's status, or
 * #LWIR_STATE_INVALID when the value is outside the vendor range (readback is
 * still stored for diagnostics).
 * @details Writes the held half (0x9d00), triggers with the second half (0x1d08,
 * byte 15 = reply length 2), waits idle, then reads the big-endian reply at 0x1d10.
 * @pre The mailbox is idle.
 */
static bool control_readback(const struct lwir_state_io *io, uint64_t deadline, const struct state_query *query,
                              struct lwir_control_result *result)
{
    uint8_t command[16] = {0x14, query->subcommand, 0, query->parameter, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2};
    struct lwir_state_value *reply = &result->reply;
    if (!transfer(io, deadline, false, 0x9d00, command, 8, reply)
        || !transfer(io, deadline, false, 0x1d08, command + 8, 8, reply)
        || !control_idle(io, deadline, reply)
        || !transfer(io, deadline, true, 0x1d10, reply->data, 2, reply)) return false;
    reply->length = 2;
    result->readback = (uint16_t)((unsigned)reply->data[0] * 256U + reply->data[1]);
    if (result->readback < query->minimum || result->readback > query->maximum) {
        reply->status = LWIR_STATE_INVALID; return false;
    }
    return true;
}

/**
 * @brief Targeted read of one controllable setting; performs no setting writes.
 * @return True with verified and readback set; false on bad arguments or any
 * failed step (see result->reply).
 * @details Budget 1 s: wait idle, then control_readback(). Only the getter header
 * is written, so submitted stays false.
 */
bool lwir_state_control_read(const struct lwir_state_io *io, enum lwir_control_kind kind,
                              struct lwir_control_result *result)
{
    if (!result) return false;
    *result = (struct lwir_control_result){.reply = {.status = LWIR_STATE_INVALID}};
    const struct state_query *query = control_query(kind);
    if (!query || !io || !io->transfer || !io->now_us || !io->pause) return false;
    const uint64_t start = io->now_us(io->context);
    if (start > UINT64_MAX - 1000000U) return false;
    const uint64_t deadline = start + 1000000U;
    if (!control_idle(io, deadline, &result->reply) || !control_readback(io, deadline, query, result)) return false;
    result->verified = true;
    result->reply.status = LWIR_STATE_OK;
    return true;
}

/**
 * @brief Apply one setting with read-back verification, or trigger FFC.
 * @return True when the setter was verified or FFC was acknowledged. False on
 * invalid input, budget overflow or any failed step; result->submitted tells
 * whether the device may already have acted.
 * @details
 * - Budget: 1 s; gain and FFC 5 s (gain may involve internal correction).
 * - Always wait for an idle mailbox first so a pending command is not overwritten.
 * - FFC: standard command 0d c1 02 (OOC+B combined update, not shutter
 *   open/close) at 0x1d00, then a fixed 3 s wait (300 x 10 ms) mirroring the
 *   vendor's special 3000 ms poll delay, then an idle check. Result is an
 *   acknowledgement only; verified stays false.
 * - Setters: SET subcommand = getter + 0x40 (0x83->0xc3 image, 0x85->0xc5 TPD),
 *   value big-endian in header bytes 6..7 (verified against vendor reference
 *   source). Held half to 0x9d00, trigger half to 0x1d08, wait idle, read back,
 *   and mark verified only on an exact match.
 * @warning After a false return with submitted set, never replay the command;
 * read the setting back to learn the device state.
 */
bool lwir_state_control(const struct lwir_state_io *io, const struct lwir_control_request *request,
                         struct lwir_control_result *result)
{
    if (!result) return false;
    *result = (struct lwir_control_result){.reply = {.status = LWIR_STATE_INVALID}};
    if (!lwir_control_valid(request) || !io || !io->transfer || !io->now_us || !io->pause) return false;
    const uint64_t start = io->now_us(io->context);
    const uint64_t budget = request->kind == LWIR_CONTROL_FFC || request->kind == LWIR_CONTROL_GAIN ? 5000000U : 1000000U;
    if (start > UINT64_MAX - budget) return false;
    const uint64_t deadline = start + budget;
    struct lwir_state_value *reply = &result->reply;
    if (!control_idle(io, deadline, reply)) return false;
    if (request->kind == LWIR_CONTROL_FFC) {
        uint8_t command[8] = {0x0d, 0xc1, 2, 0, 0, 0, 0, 0};
        /* Set before the write: a failed or short single-transfer trigger may still have executed. */
        result->submitted = true;
        if (!transfer(io, deadline, false, 0x1d00, command, sizeof(command), reply)) return false;
        /* Fixed wait, not polling: mirrors the vendor's special 3000 ms poll delay for this command. */
        for (unsigned pause = 0; pause < 300; ++pause) {
            if (!remaining_ms(io, deadline)) { reply->status = LWIR_STATE_TIMEOUT; return false; }
            if (!io->pause(io->context, 10)) { reply->status = LWIR_STATE_CANCELLED; return false; }
        }
        if (!control_idle(io, deadline, reply)) return false;
    } else {
        const struct state_query *query = control_query(request->kind);
        /* Byte 15 (reply length) stays 0: a setter returns no payload. */
        uint8_t command[16] = {0x14, (uint8_t)(query->subcommand + 0x40U), 0, query->parameter,
            0, 0, (uint8_t)(request->value >> 8), (uint8_t)request->value};
        /* The held half does not execute, so a failure here leaves submitted false. */
        if (!transfer(io, deadline, false, 0x9d00, command, 8, reply)) return false;
        /* From here the trigger may have executed even if the transfer reports failure. */
        result->submitted = true;
        if (!transfer(io, deadline, false, 0x1d08, command + 8, 8, reply)
            || !control_idle(io, deadline, reply)) return false;
        if (!control_readback(io, deadline, query, result)) return false;
        if (result->readback != request->value) { reply->status = LWIR_STATE_INVALID; return false; }
        result->verified = true;
    }
    reply->status = LWIR_STATE_OK;
    return true;
}

/**
 * @brief Decode a validated big-endian two-byte snapshot field.
 * @return False for NULL arguments, invalid index, non-OK status or a string field.
 */
bool lwir_state_u16(const struct lwir_device_state *state, enum lwir_state_field field, uint16_t *value)
{
    if (!state || !value || (unsigned)field >= LWIR_STATE_FIELDS ||
        state->values[field].status != LWIR_STATE_OK || state->values[field].length != 2) return false;
    *value = (uint16_t)((unsigned)state->values[field].data[0] * 256U + state->values[field].data[1]);
    return true;
}

/**
 * @brief Render snapshot evidence as key=value lines for the JPEG APP14 segment.
 * @return Bytes written excluding the NUL; 0 on NULL/zero-sized output, overflow,
 * or a corrupt status/length (output may then hold a truncated prefix).
 * @details A fixed header records provenance and two constant caveats:
 * device_state_bench=unverified and device_state_frame_atomic=0. For a requested
 * snapshot, each field adds three lines: status name, reply hex ("unknown"
 * unless OK) and "transport_error,device_status" so failures stay diagnosable.
 * @note Returning 0 rather than truncating lets the caller refuse incomplete evidence.
 */
size_t lwir_state_format(const struct lwir_device_state *state, char *output, size_t capacity)
{
    /* Indexed by enum lwir_state_status; keep both in the same order. */
    static const char *const statuses[] = {"not_requested", "ok", "unsupported", "timeout", "io_error", "invalid", "busy", "skipped", "cancelled"};
    if (!output || !capacity) return 0;
    const int prefix = snprintf(output, capacity, "device_state=%s\ndevice_state_source=%s\ndevice_state_bench=unverified\n"
        "device_state_frame_atomic=0\ndevice_state_started_us=%llu\ndevice_state_finished_us=%llu\n",
        state && state->requested ? "requested" : "not_requested", state && state->requested ? "tiny1c_usb_i2c" : "none",
        (unsigned long long)(state ? state->started_us : 0),
        (unsigned long long)(state ? state->finished_us : 0));
    if (prefix < 0 || (size_t)prefix >= capacity) return 0;
    size_t used = (size_t)prefix;
    if (!state || !state->requested) return used;
    for (unsigned field = 0; field < LWIR_STATE_FIELDS; ++field) {
        const struct lwir_state_value *value = &state->values[field];
        char hex[129];
        /* Guards both the table lookup and hex[] against a corrupted caller-supplied snapshot. */
        if ((unsigned)value->status >= sizeof(statuses) / sizeof(statuses[0]) || value->length > 64) return 0;
        for (unsigned index = 0; index < value->length; ++index) {
            hex[index * 2] = "0123456789abcdef"[value->data[index] >> 4];
            hex[index * 2 + 1] = "0123456789abcdef"[value->data[index] & 15];
        }
        hex[value->length * 2] = '\0';
        const int added = snprintf(output + used, capacity - used, "device_%s_status=%s\ndevice_%s_hex=%s\n"
            "device_%s_result=%d,%u\n", queries[field].name, statuses[value->status], queries[field].name,
            value->status == LWIR_STATE_OK ? hex : "unknown", queries[field].name, value->transport_error, value->device_status);
        if (added < 0 || (size_t)added >= capacity - used) return 0;
        used += (size_t)added;
    }
    return used;
}