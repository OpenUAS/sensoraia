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
 * @file chdkbridge.c
 * @brief Standalone USB PTP client for CHDK JPEG capture and normalized zoom.
 * @details One invocation owns one USB interface and PTP session. Standard PTP
 * framing carries CHDK vendor commands; small Lua scripts execute on the camera,
 * not on the host. USB access uses Linux usbfs directly (usbfs.h); no USB
 * library is loaded. This standalone C client implements camera control
 * directly over USB PTP.
 * @warning A failed transaction is not retried: capture or lens movement may
 * already have occurred. Fake-USB tests are not physical-camera validation.
 */
#define _POSIX_C_SOURCE 200809L
#define _DEFAULT_SOURCE
#include <dlfcn.h>
#include <errno.h>
#include <math.h>
#include <fcntl.h>
#include <limits.h>
#include <poll.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>
#include "usbfs.h"
#include "version.h"

/** @brief USB operations used by the application; tests substitute fakes. */
static struct {
  int (*devices_open)(struct usbfs_devices *);
  int (*devices_next)(struct usbfs_devices *, struct usbfs_device *);
  void (*devices_close)(struct usbfs_devices *);
  int (*active_config)(const struct usbfs_device *, uint8_t *, size_t, const uint8_t **, size_t *);
  int (*open)(const struct usbfs_device *, struct usbfs_handle *);
  void (*close)(struct usbfs_handle *);
  int (*claim_interface)(struct usbfs_handle *, int);
  int (*release_interface)(struct usbfs_handle *, int);
  int (*set_alternate)(struct usbfs_handle *, int, int);
  int (*bulk_transfer)(struct usbfs_handle *, unsigned char, unsigned char *, int, int *, unsigned);
  int (*control_transfer)(struct usbfs_handle *, uint8_t, uint8_t, uint16_t, uint16_t,
                          unsigned char *, uint16_t, unsigned);
  int (*clear_halt)(struct usbfs_handle *, unsigned char);
} usb = {
  usbfs_devices_open, usbfs_devices_next, usbfs_devices_close, usbfs_active_config,
  usbfs_open, usbfs_close, usbfs_claim_interface, usbfs_release_interface,
  usbfs_set_alternate, usbfs_bulk_transfer, usbfs_control_transfer, usbfs_clear_halt
};

/** @brief Wire constants and resource limits, not an exhaustive PTP API. */
enum {
  PTP_COMMAND = 1, PTP_DATA = 2, PTP_RESPONSE = 3,
  PTP_OPEN_SESSION = 0x1002, PTP_CLOSE_SESSION = 0x1003,
  PTP_OK = 0x2001, PTP_CHDK = 0x9999,
  CHDK_VERSION = 0, CHDK_TEMP_DATA = 4, CHDK_DOWNLOAD = 6,
  CHDK_EXECUTE = 7, CHDK_STATUS = 8, CHDK_SUPPORT = 9, CHDK_MESSAGE = 10,
  CHDK_WRITE_MESSAGE = 11,
  CHDK_CAPTURE_READY = 13, CHDK_CAPTURE_DATA = 14,
  CHDK_CAPTURE_JPEG = 1, CHDK_CAPTURE_NOTSET = 0x10000000,
  CHDK_MESSAGE_NONE = 0, CHDK_MESSAGE_ERROR = 1, CHDK_MESSAGE_RETURN = 2,
  CHDK_MESSAGE_USER = 3,
  CHDK_UNSUPPORTED = 0, CHDK_NIL = 1, CHDK_BOOLEAN = 2, CHDK_INTEGER = 3,
  CHDK_STRING = 4, CHDK_TABLE = 5,
  CHDK_ERROR_NONE = 0, CHDK_ERROR_COMPILE = 1, CHDK_ERROR_RUNTIME = 2,
  CHDK_ERROR_RUNNING = 0x1000,
  /* 0x100 rejects an existing script; 0x200/0x400 flush old message queues. */
  CHDK_SCRIPT_FLAGS = 0x700, TRANSFER_SIZE = 65536, SCRIPT_CAPACITY = 12288,
  SCRIPT_POLL_MIN_MS = 10, SCRIPT_POLL_MAX_MS = 50,
  DESKTOP_VOLUME_LIMIT = 128, DESKTOP_RELEASE_MS = 5000,
  DESKTOP_DISCOVERY_POLLS = 50, DESKTOP_DISCOVERY_POLL_MS = 10,
  DESKTOP_RECLAIM_MS = 2000, DESKTOP_RECLAIM_POLL_MS = 20,
  MAX_DOWNLOAD = 256 * 1024 * 1024
};

/**
 * @brief Single-session USB transport with bounded input staging.
 * @details Unread bytes occupy buffered[offset..available). They may belong to
 * the next container: USB transfer boundaries need not match PTP containers.
 * The caller owns the handle and maintains one overall operation deadline.
 */
struct transport {
  struct usbfs_handle *handle; /**< Borrowed handle for the claimed interface. */
  uint8_t input_endpoint, output_endpoint; /**< Discovered bulk endpoint addresses. */
  unsigned output_packet_size; /**< Bulk-OUT wMaxPacketSize; zero disables ZLP termination. */
  uint32_t transaction_id; /**< Next command ID; OpenSession starts with zero. */
  uint8_t buffered[TRANSFER_SIZE]; /**< Fixed 64 KiB receive buffer, never image-sized. */
  size_t offset, available; /**< Consumed offset and valid-byte count in buffered. */
  uint64_t deadline_ms; /**< Absolute CLOCK_MONOTONIC deadline in milliseconds. */
  unsigned usb_timeout_ms;
  size_t max_download;
  bool closing_session;
};

/** @brief Validated successful PTP response and optional data-phase length. */
struct response {
  uint32_t parameters[5]; /**< Decoded little-endian response parameters. */
  size_t count, data_size; /**< Parameter count and bytes in the received data phase. */
};

struct direct_download {
  size_t cursor, extent, received;
  uint32_t image_number;
  bool armed, complete;
};

static bool download_direct_chunk(struct transport *transport, FILE *file,
                                  struct direct_download *download);

/** @brief Signal-safe cancellation flag checked before blocking USB calls. */
static volatile sig_atomic_t interrupted;

/** @brief Request cancellation without allocation, I/O, or USB calls in a signal handler. */
static void interrupt_handler(int signal_number)
{
  (void)signal_number;
  interrupted = 1;
}

/** @brief Read a wall-clock-independent time base; return zero if the clock fails. */
static uint64_t monotonic_ms(void)
{
  struct timespec now;
  if (clock_gettime(CLOCK_MONOTONIC, &now) != 0) return 0;
  return (uint64_t)now.tv_sec * 1000 + (uint64_t)now.tv_nsec / 1000000;
}

/**
 * @brief Compute the next USB call's timeout within the operation deadline.
 * @param transport Active session and its absolute deadline.
 * @return 1..10000 ms, or zero for cancellation, expiry, or clock failure.
 * @note A zero result means "do not call"; USB calls always receive a nonzero
 * timeout. Calls default to a 1 s cap; --usb-timeout
 * permits 50..10000 ms without extending the overall operation deadline.
 */
static unsigned remaining_ms(const struct transport *transport)
{
  uint64_t now = monotonic_ms();
  if ((interrupted && !transport->closing_session) || !now || now >= transport->deadline_ms) return 0;
  uint64_t remaining = transport->deadline_ms - now;
  unsigned limit = transport->usb_timeout_ms ? transport->usb_timeout_ms : 1000;
  return remaining > limit ? limit : (unsigned)remaining;
}

/** @brief Select a recovery hint for a USB transport failure.
 * @param error usbfs wrapper status.
 * @return Borrowed static diagnostic.
 * @details Distinguish device loss, access and ownership from timeouts that
 * may leave a completed shot on the camera. Do not prescribe blind replay. */
static const char *usb_failure_hint(int error)
{
  switch (error) {
    case USBFS_ERROR_NO_DEVICE: return "camera disconnected; check camera power, data cable and hub; rediscover with --list";
    case USBFS_ERROR_ACCESS: return "grant the USHER service user read/write access to the camera USB device (plugdev); do not run the service as root";
    case USBFS_ERROR_BUSY: return "selected desktop camera mount could not be released or another PTP client still owns the camera; no USB reset is attempted";
    case USBFS_ERROR_TIMEOUT: return "check camera power, USB cable/hub, SD-card readiness and active scripts; a shot may already exist on SD";
    case USBFS_ERROR_IO: return "check USB power and cable/hub integrity; other cameras are not reset";
    default: return "inspect the USB error and camera state; failed commands are not replayed";
  }
}

/** @brief Decode a 16-bit little-endian value without alignment assumptions.
 * @param bytes At least two readable bytes. */
static uint16_t decode16(const uint8_t *bytes)
{
  return (uint16_t)((uint16_t)bytes[0] | (uint16_t)bytes[1] << 8);
}

/** @brief Decode a 32-bit little-endian value without native-endian casts.
 * @param bytes At least four readable bytes. */
static uint32_t decode32(const uint8_t *bytes)
{
  return (uint32_t)bytes[0] | (uint32_t)bytes[1] << 8 |
         (uint32_t)bytes[2] << 16 | (uint32_t)bytes[3] << 24;
}

/** @brief Encode a wire value into two writable bytes.
 * @param bytes Destination of at least two bytes.
 * @param value Host-order value to encode little-endian. */
static void encode16(uint8_t *bytes, uint16_t value)
{
  bytes[0] = (uint8_t)value;
  bytes[1] = (uint8_t)(value >> 8);
}

/** @brief Encode a wire value into four writable bytes.
 * @param bytes Destination of at least four bytes.
 * @param value Host-order value to encode little-endian. */
static void encode32(uint8_t *bytes, uint32_t value)
{
  for (size_t index = 0; index < 4; ++index)
    bytes[index] = (uint8_t)(value >> (index * 8));
}

/**
 * @brief Submit one complete, small outbound PTP container.
 * @param transport Claimed USB transport.
 * @param bytes Encoded container; the URB API requires a non-const buffer.
 * @param length Number of bytes to transmit, representable by an int.
 * @return True only when the complete container was transferred.
 * @warning Partial writes abort rather than resend a possibly executed command.
 */
static bool send_packet(struct transport *transport, uint8_t *bytes, size_t length)
{
  int transferred = 0;
  unsigned timeout = remaining_ms(transport);
  if (!timeout || length > INT_MAX) return false;
  int result = usb.bulk_transfer(transport->handle, transport->output_endpoint,
                                  bytes, (int)length, &transferred, timeout);
  if (result != 0 || transferred != (int)length) {
        fprintf(stderr, "USB send failed: %s (%d/%zu bytes); %s\n",
          usbfs_error_name(result), transferred, length, usb_failure_hint(result));
    return false;
  }
  return true;
}

/**
 * @brief Refill empty bounded USB storage without replaying a command.
 * @param transport Input stream whose previous bytes have been consumed.
 * @return False on cancellation, exhausted deadline, repeated empty reads, or a USB error.
 * @details Read-ahead is retained for subsequent headers or payloads. Failed
 * calls may already have consumed part of the stream; do not retry parsing.
 */
static bool refill_buffer(struct transport *transport)
{
  unsigned empty_reads = 0;
  for (;;) {
      unsigned timeout = remaining_ms(transport);
      int transferred = 0;
      if (!timeout) return false;
      int result = usb.bulk_transfer(transport->handle, transport->input_endpoint,
                                       transport->buffered, TRANSFER_SIZE,
                                       &transferred, timeout);
      if (transferred < 0 || transferred > TRANSFER_SIZE) return false;
      /* A ZLP can terminate a data phase at a packet-size boundary. Retrying a
       * read is safe; the corresponding command is never retransmitted. */
      if (!transferred && (result == 0 || result == USBFS_ERROR_INTERRUPTED)) {
        if (++empty_reads <= 3) continue;
        fprintf(stderr, "USB receive made no progress after repeated empty/interrupted reads\n");
        return false;
      }
      if (!transferred || (result != 0 && result != USBFS_ERROR_TIMEOUT &&
                            result != USBFS_ERROR_INTERRUPTED)) {
        fprintf(stderr, "USB receive failed: %s; %s\n", usbfs_error_name(result), usb_failure_hint(result));
        return false;
      }
      /* usbfs may return useful bytes with timeout/interruption. Preserve them
       * instead of shifting the stream by discarding a partially read header. */
      transport->offset = 0;
      transport->available = (size_t)transferred;
      return true;
  }
}

/** @brief Read an exact bounded payload across arbitrary USB transfer splits.
 * @param transport Input stream, including retained read-ahead.
 * @param destination Writable region of at least length bytes.
 * @param length Exact number of bytes required by the parser.
 * @return True only when the requested bytes have been copied. */
static bool receive_bytes(struct transport *transport, uint8_t *destination, size_t length)
{
  while (length) {
    if (!remaining_ms(transport)) return false;
    if (transport->offset == transport->available && !refill_buffer(transport)) return false;
    size_t amount = transport->available - transport->offset;
    if (amount > length) amount = length;
    for (size_t index = 0; index < amount; ++index)
      destination[index] = transport->buffered[transport->offset + index];
    transport->offset += amount;
    destination += amount;
    length -= amount;
  }
  return true;
}

/** @brief Describe standard PTP response failures without triggering recovery actions. */
static const char *ptp_response_name(uint16_t code)
{
  switch (code) {
    case 0x2002: return "general camera error";
    case 0x2003: return "session not open";
    case 0x2005: return "operation not supported";
    case 0x200f: return "access denied";
    case 0x2019: return "camera busy; stop competing clients or wait for the current action";
    case 0x201d: return "invalid parameter";
    case 0x201e: return "session already open; another client may own the camera";
    default: return "unrecognized PTP response";
  }
}

/**
 * @brief Perform command, optional data, and response phases for one PTP operation.
 * @param transport Single-session transport; transaction_id advances once.
 * @param operation Standard PTP or vendor operation code.
 * @param parameters Host-order command parameters, or NULL for zero parameters.
 * @param parameter_count Number of command parameters, at most five.
 * @param outgoing Optional host-to-camera data; NULL means no outgoing data phase.
 * @param outgoing_size Payload size, limited to SCRIPT_CAPACITY bytes.
 * @param incoming Optional destination for a small camera-to-host payload.
 * @param capacity Writable size of incoming; ignored when file is supplied.
 * @param file Optional stream for image data; takes precedence over incoming.
 * @param response Successful response parameters and data size; reset on entry.
 * @return True only after a matching PTP_OK response and any required data phase.
 * @details A data phase is bounded before it is consumed. Response bodies contain
 * at most five 32-bit parameters. This minimal client does not recover the USB
 * stream after malformed input or retry non-idempotent operations.
 */
static bool transaction(struct transport *transport, uint16_t operation,
                        const uint32_t *parameters, size_t parameter_count,
                        const uint8_t *outgoing, size_t outgoing_size,
                        uint8_t *incoming, size_t capacity, FILE *file,
                        struct response *response)
{
  uint8_t command[32] = {0}, header[12], payload[SCRIPT_CAPACITY + 12];
  if (parameter_count > 5 || outgoing_size > sizeof(payload) - 12) return false;
  *response = (struct response){0};
  uint32_t transaction_id = transport->transaction_id++;
  encode32(command, (uint32_t)(12 + parameter_count * 4));
  encode16(command + 4, PTP_COMMAND);
  encode16(command + 6, operation);
  encode32(command + 8, transaction_id);
  for (size_t index = 0; index < parameter_count; ++index)
    encode32(command + 12 + index * 4, parameters[index]);
  if (!send_packet(transport, command, 12 + parameter_count * 4)) return false;
  if (outgoing) {
    encode32(payload, (uint32_t)(12 + outgoing_size));
    encode16(payload + 4, PTP_DATA);
    encode16(payload + 6, operation);
    encode32(payload + 8, transaction_id);
    for (size_t index = 0; index < outgoing_size; ++index)
      payload[12 + index] = outgoing[index];
    if (!send_packet(transport, payload, 12 + outgoing_size)) return false;
    /* A container ending exactly on a packet boundary needs a ZLP, as chdkptp sends. */
    if (transport->output_packet_size && (12 + outgoing_size) % transport->output_packet_size == 0 &&
        !send_packet(transport, payload, 0)) return false;
  }
  /* Parse framing independently of USB packet boundaries. A transaction may
   * carry one incoming data container followed by its response container. */
  bool received_data = false;
  for (;;) {
    if (!receive_bytes(transport, header, sizeof(header))) return false;
    uint32_t length = decode32(header);
    uint16_t type = decode16(header + 4), code = decode16(header + 6);
    if (length < 12 || decode32(header + 8) != transaction_id) {
      fprintf(stderr, "Invalid PTP container or transaction ID\n");
      return false;
    }
    size_t body_size = length - 12;
    if (type == PTP_DATA) {
      size_t maximum = file ? (transport->max_download ? transport->max_download : MAX_DOWNLOAD) : capacity;
      if (received_data || code != operation || (!incoming && !file) ||
          body_size > maximum) {
        fprintf(stderr, "Rejected unexpected or oversized PTP data (%zu bytes)\n", body_size);
        return false;
      }
      received_data = true;
      response->data_size = body_size;
      /* Stream images directly from receive storage; small replies use bounded scratch storage. */
      size_t position = 0;
      while (position < body_size) {
        if (!remaining_ms(transport)) return false;
        size_t amount = body_size - position;
        if (file) {
          if (transport->offset == transport->available && !refill_buffer(transport)) return false;
          size_t available = transport->available - transport->offset;
          if (amount > available) amount = available;
          if (fwrite(transport->buffered + transport->offset, 1, amount, file) != amount) return false;
          transport->offset += amount;
        } else {
          if (amount > sizeof(payload)) amount = sizeof(payload);
          if (!receive_bytes(transport, payload, amount)) return false;
          for (size_t index = 0; index < amount; ++index)
            incoming[position + index] = payload[index];
        }
        position += amount;
      }
      continue;
    }
    if (type != PTP_RESPONSE || body_size > 20 || body_size % 4) return false;
    if (!receive_bytes(transport, payload, body_size)) return false;
    if (code != PTP_OK) {
            fprintf(stderr, "PTP operation 0x%04x failed: response 0x%04x (%s)\n",
              operation, code, ptp_response_name(code));
      return false;
    }
    if ((incoming || file) && !received_data) return false;
    response->count = body_size / 4;
    for (size_t index = 0; index < response->count; ++index)
      response->parameters[index] = decode32(payload + index * 4);
    return true;
  }
}

/**
 * @brief Place a CHDK subcommand and argument in vendor operation 0x9999.
 * @details Payloads and response validation are delegated to transaction().
 */
static bool chdk_call(struct transport *transport, uint32_t command, uint32_t argument,
                      const uint8_t *outgoing, size_t size, uint8_t *incoming,
                      size_t capacity, FILE *file, struct response *response)
{
  uint32_t parameters[2] = {command, argument};
  return transaction(transport, PTP_CHDK, parameters, 2, outgoing, size,
                     incoming, capacity, file, response);
}

/** @brief Best-effort close of an owned session, even after cancellation, within one second. */
static void close_session(struct transport *transport)
{
  transport->closing_session = true;
  transport->deadline_ms = monotonic_ms() + 1000;
  if (transport->handle) transport->handle->cancel = NULL;
  struct response response;
  if (!transaction(transport, PTP_CLOSE_SESSION, NULL, 0, NULL, 0, NULL, 0, NULL, &response))
    fprintf(stderr, "Owned PTP session close was not confirmed; no reset or takeover attempted\n");
  transport->closing_session = false;
}

enum {
  PTP_CLASS_OUT = 0x21, PTP_CLASS_IN = 0xa1,
  PTP_REQUEST_DEVICE_RESET = 0x66, PTP_REQUEST_DEVICE_STATUS = 0x67,
  RECOVERY_CONTROL_MS = 3000, RECOVERY_DRAIN_MS = 100, RECOVERY_DRAIN_READS = 64
};

/**
 * @brief Clear a session or stale data left by an interrupted earlier run.
 * @param transport Claimed interface whose OpenSession just failed.
 * @param interface_number Claimed still-image interface.
 * @return True when the device is present and OpenSession may be retried once.
 * @details Uses the still-image class requests and endpoint halt clearing, as
 * chdkptp does; never resets the USB port. Stale input is drained with a bound.
 */
static bool recover_ptp(struct transport *transport, int interface_number)
{
  if (interface_number < 0 || interface_number > 255 || !remaining_ms(transport)) return false;
  fprintf(stderr, "Recovering camera PTP state left by an earlier run (class reset, no USB port reset)\n");
  unsigned char status[4] = {0};
  const uint16_t index = (uint16_t)interface_number;
  unsigned timeout = remaining_ms(transport);
  if (!timeout) return false;
  if (timeout > RECOVERY_CONTROL_MS) timeout = RECOVERY_CONTROL_MS;
  int result = usb.control_transfer(transport->handle, PTP_CLASS_IN, PTP_REQUEST_DEVICE_STATUS,
                                    0, index, status, sizeof(status), timeout);
  if (result == USBFS_ERROR_NO_DEVICE) return false;
  (void)usb.clear_halt(transport->handle, transport->input_endpoint);
  (void)usb.clear_halt(transport->handle, transport->output_endpoint);
  timeout = remaining_ms(transport);
  if (!timeout) return false;
  if (timeout > RECOVERY_CONTROL_MS) timeout = RECOVERY_CONTROL_MS;
  result = usb.control_transfer(transport->handle, PTP_CLASS_OUT, PTP_REQUEST_DEVICE_RESET,
                                0, index, NULL, 0, timeout);
  if (result == USBFS_ERROR_NO_DEVICE) return false;
  if (result < 0) fprintf(stderr, "PTP device reset request failed: %s\n", usbfs_error_name(result));
  transport->offset = transport->available = 0;
  for (unsigned reads = 0; reads < RECOVERY_DRAIN_READS; ++reads) {
    timeout = remaining_ms(transport);
    if (!timeout) return false;
    if (timeout > RECOVERY_DRAIN_MS) timeout = RECOVERY_DRAIN_MS;
    int transferred = 0;
    result = usb.bulk_transfer(transport->handle, transport->input_endpoint, transport->buffered,
                               TRANSFER_SIZE, &transferred, timeout);
    if (result == USBFS_ERROR_NO_DEVICE) return false;
    if (transferred <= 0 || (result != 0 && result != USBFS_ERROR_TIMEOUT)) break;
  }
  timeout = remaining_ms(transport);
  if (!timeout) return false;
  if (timeout > RECOVERY_CONTROL_MS) timeout = RECOVERY_CONTROL_MS;
  (void)usb.control_transfer(transport->handle, PTP_CLASS_IN, PTP_REQUEST_DEVICE_STATUS,
                             0, index, status, sizeof(status), timeout);
  transport->offset = transport->available = 0;
  transport->transaction_id = 0;
  return transport->handle->fd >= 0;
}

/**
 * @brief Report CHDK protocol and script capabilities without executing a script.
 * @param transport Open session on one selected camera.
 * @return True when version, scripting support, and status queries succeed.
 * @note Does not enter record mode, drain messages, move the lens, or touch files.
 */
static bool camera_info(struct transport *transport)
{
  struct response response;
  if (!chdk_call(transport, CHDK_VERSION, 0, NULL, 0, NULL, 0, NULL, &response) ||
    response.count < 2) return false;
  printf("CHDK PTP protocol: %u.%u\n", response.parameters[0], response.parameters[1]);
  if (!chdk_call(transport, CHDK_SUPPORT, 0, NULL, 0, NULL, 0, NULL, &response) ||
    response.count < 1) return false;
  printf("Camera-side Lua: %s\n", response.parameters[0] & 1 ? "supported" : "not supported");
  if (!chdk_call(transport, CHDK_STATUS, 0, NULL, 0, NULL, 0, NULL, &response) ||
    response.count < 1) return false;
  printf("Script running: %s; messages waiting: %s\n",
     response.parameters[0] & 1 ? "yes" : "no", response.parameters[0] & 2 ? "yes" : "no");
  return true;
}

/** @brief Translate a CHDK script status for diagnostics.
 * @param error Camera-reported script error code.
 * @return Borrowed static description, including an unknown-code fallback. */
static const char *script_error_name(uint32_t error)
{
  switch (error) {
    case CHDK_ERROR_NONE: return "unspecified script error";
    case CHDK_ERROR_COMPILE: return "compilation error";
    case CHDK_ERROR_RUNTIME: return "runtime error";
    case CHDK_ERROR_RUNNING: return "another script is running; it was not interrupted";
    default: return "unknown script error";
  }
}

/** @brief Validate known message types and payload lengths, including empty-message zero padding. */
static bool valid_script_message(const struct response *response, const uint8_t *message)
{
  if (response->count < 4) return false;
  size_t length = response->parameters[3];
  if (length) {
    if (length != response->data_size) return false;
  } else {
    if (!response->data_size) return false;
    for (size_t index = 0; index < response->data_size; ++index)
      if (message[index]) return false;
  }
  uint32_t subtype = response->parameters[1];
  switch (response->parameters[0]) {
    case CHDK_MESSAGE_NONE:
      return subtype == 0 && length == 0;
    case CHDK_MESSAGE_ERROR:
      return subtype <= CHDK_ERROR_RUNTIME;
    case CHDK_MESSAGE_RETURN:
    case CHDK_MESSAGE_USER:
      switch (subtype) {
        case CHDK_UNSUPPORTED:
        case CHDK_STRING:
        case CHDK_TABLE: return true;
        case CHDK_NIL: return length == 0;
        case CHDK_BOOLEAN:
        case CHDK_INTEGER: return length == sizeof(uint32_t);
        default: return false;
      }
    default: return false;
  }
}

/** @brief Sleep to a monotonic poll target without extending it on EINTR or exceeding the deadline. */
static bool wait_script_poll(const struct transport *transport, unsigned interval_ms)
{
  uint64_t wake_ms = monotonic_ms() + interval_ms;
  for (;;) {
    unsigned remaining = remaining_ms(transport);
    uint64_t now = monotonic_ms();
    if (!remaining || !now || now >= transport->deadline_ms) return false;
    if (now >= wake_ms) return true;
    uint64_t delay_ms = wake_ms - now;
    if (delay_ms > remaining) delay_ms = remaining;
    struct timespec delay = {.tv_sec = (time_t)(delay_ms / 1000),
                            .tv_nsec = (long)(delay_ms % 1000) * 1000000};
    if (nanosleep(&delay, NULL) != 0 && errno != EINTR) {
      fprintf(stderr, "Camera script polling sleep failed: %s\n", strerror(errno));
      return false;
    }
  }
}

/** @brief Wait within the deadline until no camera script is running; never interrupts one. */
static bool wait_script_idle(struct transport *transport)
{
  fprintf(stderr, "Waiting for a camera script left by an earlier run to finish\n");
  unsigned poll_ms = SCRIPT_POLL_MIN_MS;
  struct response response;
  for (;;) {
    if (!chdk_call(transport, CHDK_STATUS, 0, NULL, 0, NULL, 0, NULL, &response) ||
        response.count < 1) return false;
    if (!(response.parameters[0] & 1)) return true;
    if (!wait_script_poll(transport, poll_ms)) return false;
    poll_ms = poll_ms * 2 > SCRIPT_POLL_MAX_MS ? SCRIPT_POLL_MAX_MS : poll_ms * 2;
  }
}

static const char record_mode_script[] =
  "if not get_mode() then set_record(true) end\n"
  "local started=get_tick_count()\n"
  "while not get_mode() and bitand(get_tick_count()-started,2147483647)<5000 do sleep(100) end\n"
  "if not get_mode() then error('cannot enter record mode') end\n";
#define SCRIPT_BODY_CAPACITY (SCRIPT_CAPACITY - sizeof(record_mode_script))

/**
 * @brief Run camera-side Lua and wait for its correlated string return value.
 * @param transport Open CHDK PTP session and overall operation deadline.
 * @param body Independently authored script body.
 * @param result Destination for one nonempty, NUL-terminated return string.
 * @param result_capacity Total writable capacity, including the terminator.
 * @param direct_file Optional JPEG stream serviced while the script runs.
 * @param enter_record Prepend record-mode setup for existing capture/configuration operations.
 * @return True when this script has returned a string and is no longer running.
 * @details Requires CHDK protocol 2.6+ within major version 2. Existing scripts
 * are not killed. Messages belonging to another script ID are ignored, camera
 * errors fail the operation, and an oversized or embedded-NUL result is rejected.
 * @warning Host cancellation/timeout does not stop a script already on the camera.
 */
static bool run_script_operation(struct transport *transport, const char *body,
                                 char *result, size_t result_capacity, FILE *direct_file,
                                 bool enter_record)
{
  char script[SCRIPT_CAPACITY];
  int script_length = snprintf(script, sizeof(script), "%s%s", enter_record ? record_mode_script : "", body);
  if (!result_capacity || script_length < 0 || (size_t)script_length + 1 > sizeof(script))
    return false;
  result[0] = '\0';
  struct response response;
  if (!chdk_call(transport, CHDK_VERSION, 0, NULL, 0, NULL, 0, NULL, &response) ||
      response.count < 2 || response.parameters[0] != 2 || response.parameters[1] < 6) {
    fprintf(stderr, "CHDK PTP protocol 2.6 or later (major 2) is required\n");
    return false;
  }
  if (!chdk_call(transport, CHDK_SUPPORT, 0, NULL, 0, NULL, 0, NULL, &response) ||
      response.count < 1 || !(response.parameters[0] & 1)) {
    fprintf(stderr, "Camera does not report CHDK Lua support; no script was started\n");
    return false;
  }
  if (!chdk_call(transport, CHDK_EXECUTE, CHDK_SCRIPT_FLAGS,
                 (const uint8_t *)script, (size_t)script_length + 1, NULL, 0, NULL, &response)) {
    fprintf(stderr, "Camera script startup transaction failed; execution state is unknown, not retried\n");
    return false;
  }
  /* A bounded script left by an interrupted earlier run is never killed; wait once for it. */
  if (response.count >= 2 && response.parameters[1] == CHDK_ERROR_RUNNING && wait_script_idle(transport) &&
      !chdk_call(transport, CHDK_EXECUTE, CHDK_SCRIPT_FLAGS,
                 (const uint8_t *)script, (size_t)script_length + 1, NULL, 0, NULL, &response)) {
    fprintf(stderr, "Camera script startup transaction failed; execution state is unknown, not retried\n");
    return false;
  }
  if (response.count < 2) {
    fprintf(stderr, "Malformed camera script startup response; execution state is unknown\n");
    return false;
  }
  if (response.parameters[1] != CHDK_ERROR_NONE) {
    fprintf(stderr, "Camera rejected script: %s (0x%08x)\n",
            script_error_name(response.parameters[1]), response.parameters[1]);
    return false;
  }
  uint32_t script_id = response.parameters[0];
  bool have_result = false;
  struct direct_download download = {0};
  unsigned poll_ms = SCRIPT_POLL_MIN_MS;
  while (remaining_ms(transport)) {
    if (!chdk_call(transport, CHDK_STATUS, 0, NULL, 0, NULL, 0, NULL, &response) ||
        response.count < 1) return false;
    uint32_t status = response.parameters[0];
    /* Drain messages even after RUN clears; a script's final return may still
     * be queued. Accept it only if the script ID matches ExecuteScript's ID. */
    if (status & 2) {
      uint8_t message[4096];
      if (!chdk_call(transport, CHDK_MESSAGE, 0, NULL, 0, message,
                     sizeof(message), NULL, &response)) return false;
      if (!valid_script_message(&response, message)) {
        fprintf(stderr, "Malformed camera script message: %zu parameters, %zu payload bytes\n",
                response.count, response.data_size);
        return false;
      }
      if (response.parameters[0] != CHDK_MESSAGE_NONE) {
        if (response.parameters[2] != script_id) continue;
        size_t length = response.parameters[3];
        if (response.parameters[0] == CHDK_MESSAGE_ERROR) {
          fprintf(stderr, "Camera script %u %s (0x%08x): %.*s\n", script_id,
                  script_error_name(response.parameters[1]), response.parameters[1],
                  (int)length, (char *)message);
          return false;
        }
        if (response.parameters[0] == CHDK_MESSAGE_RETURN) {
          if (response.parameters[1] != CHDK_STRING || have_result ||
              !length || length >= result_capacity) {
            fprintf(stderr, "Invalid camera script return; expected one nonempty bounded string\n");
            return false;
          }
          for (size_t index = 0; index < length; ++index) {
            if (!message[index]) {
              result[0] = '\0';
              fprintf(stderr, "Invalid embedded NUL in camera script return\n");
              return false;
            }
            result[index] = (char)message[index];
          }
          result[length] = '\0';
          have_result = true;
        }
        if (direct_file && response.parameters[0] == CHDK_MESSAGE_USER) {
          static const char ready[] = "Direct ready";
          if (download.armed || response.parameters[1] != CHDK_STRING ||
              length != sizeof(ready) - 1 || memcmp(message, ready, sizeof(ready) - 1)) {
            fprintf(stderr, "Unexpected direct capture script message\n");
            return false;
          }
          download.armed = true;
        }
        poll_ms = SCRIPT_POLL_MIN_MS;
        continue;
      }
    }
    if (direct_file && download.armed && !download.complete) {
      if (!chdk_call(transport, CHDK_CAPTURE_READY, 0, NULL, 0, NULL, 0, NULL, &response) ||
          response.count != 2) return false;
      uint32_t ready = response.parameters[0];
      if (ready != 0 && ready != CHDK_CAPTURE_JPEG) {
        fprintf(stderr, "Direct JPEG capture unavailable or cancelled (0x%08x)\n", ready);
        return false;
      }
      if (ready == CHDK_CAPTURE_JPEG) {
        if (!response.parameters[1] ||
            (download.image_number && download.image_number != response.parameters[1])) {
          fprintf(stderr, "Direct JPEG image identity changed during transfer\n");
          return false;
        }
        download.image_number = response.parameters[1];
        if (!download_direct_chunk(transport, direct_file, &download)) return false;
        if (download.complete) {
          static const uint8_t done[] = "Direct done";
          if (!chdk_call(transport, CHDK_WRITE_MESSAGE, script_id, done, sizeof(done) - 1,
                         NULL, 0, NULL, &response) || response.count != 1 || response.parameters[0] != 0)
            return false;
        }
        poll_ms = SCRIPT_POLL_MIN_MS;
        continue;
      }
    }
    if (!(status & 1)) {
      if (!have_result || (direct_file && !download.complete)) {
        fprintf(stderr, "Camera script ended without its expected return value\n");
        return false;
      }
      break;
    }
    if (!wait_script_poll(transport, poll_ms)) return false;
    if (poll_ms < SCRIPT_POLL_MAX_MS) {
      poll_ms *= 2;
      if (poll_ms > SCRIPT_POLL_MAX_MS) poll_ms = SCRIPT_POLL_MAX_MS;
    }
  }
  if (!have_result || !remaining_ms(transport)) return false;
  if (direct_file) {
    char expected[64];
    int length = snprintf(expected, sizeof(expected), "Direct JPEG %u", download.image_number);
    if (!download.complete || length < 0 || (size_t)length >= sizeof(expected) || strcmp(result, expected)) {
      fprintf(stderr, "Direct JPEG and script image identities do not match\n");
      return false;
    }
    fprintf(stderr, "Direct JPEG image=%u bytes=%zu transferred=%zu; no SD JPEG retained\n",
            download.image_number, download.extent, download.received);
  }
  return true;
}

/** @brief Execute a capture-path script without direct-download output.
 * @param transport Active PTP session and deadline.
 * @param body Camera-side Lua source.
 * @param[out] result Caller buffer for the validated script result.
 * @param result_capacity Buffer size including the terminator.
 * @return True after the script operation succeeds.
 * @details Reuse the common script runner with the capture-path policy so
 * ordinary SD capture and direct transfer share result validation. */
static bool run_script(struct transport *transport, const char *body,
                       char *result, size_t result_capacity)
{
  return run_script_operation(transport, body, result, result_capacity, NULL, true);
}

static const char retract_lens_script[] =
  "if type(get_mode)~='function' or type(get_shooting)~='function' or "
    "type(get_tick_count)~='function' or type(sleep)~='function' then error('missing playback API') end\n"
  "if get_shooting() then error('camera is shooting; lens retract rejected') end\n"
  "local record=get_mode()\n"
  "if type(record)~='boolean' then error('invalid camera mode') end\n"
  "if record then\n"
  " if type(switch_mode_usb)=='function' then switch_mode_usb(0)\n"
  " elseif type(set_record)=='function' then set_record(false)\n"
  " else error('missing playback transition API') end\n"
  " local started=get_tick_count()\n"
  " while get_mode() and bitand(get_tick_count()-started,2147483647)<5000 do sleep(50) end\n"
  " if get_mode()~=false then error('cannot enter playback mode') end\n"
  "end\n"
  "return 'Playback confirmed; physical lens retraction not verified'\n";

/** @brief Ask the camera to enter playback and confirm the resulting mode.
 * @param transport Active PTP session.
 * @return True if playback was confirmed.
 * @warning Success does not establish physical lens position. */
static bool retract_lens(struct transport *transport)
{
  char result[256];
  if (!run_script_operation(transport, retract_lens_script, result, sizeof(result), NULL, false) ||
      strcmp(result, "Playback confirmed; physical lens retraction not verified")) return false;
  printf("%s\n", result);
  return true;
}

/* Read-only: reports CHDK "Disable LCD off" policy (config 1221; 3 holds main
 * power through LockMainPower) with battery voltage and temperatures when the
 * firmware exposes them. No record mode, shooting or configuration change. */
static const char power_status_script[] =
  "if type(get_config_value)~='function' then error('missing power status API') end\n"
  "local policy=get_config_value(1221)\n"
  "if type(policy)~='number' or policy<0 or policy>3 or policy%1~=0 then error('power policy unavailable') end\n"
  "local report='idle_shutdown_policy='..tostring(policy)..'; keep_awake='..(policy==3 and '1' or '0')\n"
  "if type(get_vbatt)=='function' then\n"
  " local millivolts=get_vbatt()\n"
  " if type(millivolts)=='number' then report=report..'; battery_mv='..tostring(millivolts) end\n"
  "end\n"
  "if type(get_temperature)=='function' then\n"
  " local names={'optics','sensor','battery'}\n"
  " for which=0,2 do\n"
  "  local celsius=get_temperature(which)\n"
  "  if type(celsius)=='number' and celsius>-100 then report=report..'; '..names[which+1]..'_c='..tostring(celsius) end\n"
  " end\n"
  "end\n"
  "for _,entry in ipairs{{'zoom_step',get_zoom},{'focus_mm',get_focus}} do\n"
  " if type(entry[2])=='function' then\n"
  "  local ok,value=pcall(entry[2])\n"
  "  if ok and type(value)=='number' then report=report..'; '..entry[1]..'='..tostring(value) end\n"
  " end\n"
  "end\n"
  "if type(require)=='function' and type(get_prop)=='function' then\n"
  " local ok,props=pcall(require,'propcase')\n"
  " if ok and type(props)=='table' then\n"
  "  for _,entry in ipairs{{'focus_mode',props.FOCUS_MODE},{'af_lock',props.AF_LOCK},{'canon_mode',props.SHOOTING_MODE},{'iso_mode',props.ISO_MODE}} do\n"
  "   if type(entry[2])=='number' then\n"
  "    local valid,value=pcall(get_prop,entry[2])\n"
  "    if valid and type(value)=='number' then report=report..'; '..entry[1]..'='..tostring(value) end\n"
  "   end\n"
  "  end\n"
  " end\n"
  "end\n"
  "if type(get_mode)=='function' and type(bitand)=='function' then\n"
  " local valid,_,_,mode=pcall(get_mode)\n"
  " if valid and type(mode)=='number' then report=report..'; mode='..tostring(bitand(mode,255)) end\n"
  "end\n"
  "for _,entry in ipairs{{'overrides_off',2003},{'tv_ovr',1110},{'iso_ovr',1130},{'iso_ovr_koef',1131},{'sd_ovr_koef',1141},{'autoiso',1160}} do\n"
  " local valid,value=pcall(get_config_value,entry[2])\n"
  " if valid and type(value)=='number' then report=report..'; '..entry[1]..'='..tostring(value) end\n"
  "end\n"
  "return report\n";

/** @brief Print the camera's idle power-off policy and available telemetry.
 * @param transport Active PTP session.
 * @return True only for a valid reply indicating the Always keep-awake policy.
 * @details The query is read-only. Report the observed setting even if it
 * does not satisfy the operational policy. */
static bool power_status(struct transport *transport)
{
  static const char prefix[] = "idle_shutdown_policy=";
  char result[512];
  if (!run_script_operation(transport, power_status_script, result, sizeof(result), NULL, false) ||
      strncmp(result, prefix, sizeof(prefix) - 1)) return false;
  printf("%s\n", result);
  if (!strstr(result, "; keep_awake=1")) {
    fprintf(stderr, "Camera idle power-off prevention is not Always; run --configure --keep-awake 1\n");
    return false;
  }
  return true;
}

/**
 * @brief Validate a bounded printable camera pathname with no traversal components.
 * @param path Absolute camera path beginning with A/; no script interpolation occurs.
 * @return True for a nonempty file path shorter than 512 bytes.
 */
static bool valid_camera_path(const char *path)
{
  if (!path || path[0] != 'A' || path[1] != '/') return false;
  size_t length = strlen(path);
  if (length < 4 || length >= 512) return false;
  const char *component = path + 2;
  for (const char *position = component;; ++position) {
    unsigned char character = (unsigned char)*position;
    if (character && (character < 32 || character > 126 || character == '\\')) return false;
    if (character == '/' || !character) {
      size_t component_length = (size_t)(position - component);
      if (!component_length || (component_length == 1 && component[0] == '.') ||
          (component_length == 2 && component[0] == '.' && component[1] == '.')) return false;
      if (!character) return true;
      component = position + 1;
    }
  }
}

/**
 * @brief Stream an existing SD-card file without executing a camera script.
 * @param transport Open session on a selected camera.
 * @param remote_path Valid A/ pathname; original is never deleted.
 * @param file Writable caller-owned temporary file.
 * @return True after bounded file transfer and a successful PTP response.
 * @note main() still requires JPEG markers before publishing a download.
 */
static bool download_file(struct transport *transport, const char *remote_path, FILE *file)
{
  struct response response;
  if (!valid_camera_path(remote_path)) {
    fprintf(stderr, "Invalid camera file path; use a printable A/ path without . or .. components\n");
    return false;
  }
  fprintf(stderr, "Downloading %s\n", remote_path);
  if (!chdk_call(transport, CHDK_TEMP_DATA, 0, (const uint8_t *)remote_path,
                 strlen(remote_path), NULL, 0, NULL, &response)) return false;
  return chdk_call(transport, CHDK_DOWNLOAD, 0, NULL, 0, NULL, 0, file, &response);
}

static const char romlog_script[] =
  "if type(call_event_proc)~='function' then error('native event procedures unavailable') end\n"
  "if type(os)~='table' or type(os.stat)~='function' or type(os.remove)~='function' then error('log filesystem API unavailable') end\n"
  "if type(get_config_value)~='function' or get_config_value(1999)~=1 then error('Enable Lua native calls in CHDK menu, or use Save ROM crash log') end\n"
  "if call_event_proc('SystemEventInit')==-1 and call_event_proc('System.Create')==-1 then error('log export initialization failed') end\n"
  "for _,path in ipairs{'A/ROMLOG.LOG','A/RomLogErr.txt'} do\n"
  " if os.stat(path) and not os.remove(path) then error('cannot replace old log '..path) end\n"
  "end\n"
  "if call_event_proc('GetLogToFile','A/ROMLOG.LOG',1)~=0 then error('GetLogToFile failed') end\n"
  "if os.stat('A/RomLogErr.txt') then error('camera reported RomLogErr.txt') end\n"
  "local info=os.stat('A/ROMLOG.LOG')\n"
  "if type(info)~='table' or type(info.size)~='number' or info.size<1 or info.size>65536 then error('missing or oversized ROM log') end\n"
  "return 'A/ROMLOG.LOG'\n";

/** @brief Export and download the camera's bounded ROM crash log.
 * @param transport Active PTP session.
 * @param file Caller-owned writable output stream.
 * @return True after a successful export and download.
 * @details Require the expected fixed camera pathname before reading it
 * instead of treating arbitrary script output as a destination selector. */
static bool download_romlog(struct transport *transport, FILE *file)
{
  char path[64];
  return run_script_operation(transport, romlog_script, path, sizeof(path), NULL, false) &&
         !strcmp(path, "A/ROMLOG.LOG") && download_file(transport, path, file);
}

/** @brief Receive one direct JPEG chunk into the private output stream.
 * @param transport Active PTP session with the global download limit.
 * @param[in,out] file Seekable temporary stream open for reading and writing.
 * @param[in,out] download Transfer cursor, file extent and completion state.
 * @return True for an accepted chunk; discard the temporary image after failure.
 * @details Stage at the end until the reply supplies the actual offset.
 * Permit backpatching existing bytes but reject gaps, and bound total
 * transferred bytes separately from the resulting extent. */
static bool download_direct_chunk(struct transport *transport, FILE *file,
                                  struct direct_download *download)
{
  size_t maximum = transport->max_download ? transport->max_download : MAX_DOWNLOAD;
  if (download->complete || download->received >= maximum ||
      fseeko(file, (off_t)download->extent, SEEK_SET) != 0) return false;
  struct response response;
  size_t saved_limit = transport->max_download;
  transport->max_download = maximum - download->received;
  bool transferred = chdk_call(transport, CHDK_CAPTURE_DATA, CHDK_CAPTURE_JPEG,
                               NULL, 0, NULL, 0, file, &response);
  transport->max_download = saved_limit;
  if (!transferred) return false;
  if (response.count != 3 || !response.data_size ||
      response.parameters[0] != response.data_size || response.parameters[1] > 1) {
    fprintf(stderr, "Invalid direct JPEG chunk length or completion response\n");
    return false;
  }
  size_t position = response.parameters[2] == UINT32_MAX ? download->cursor : response.parameters[2];
  if (position > download->extent || response.data_size > maximum - position) {
    fprintf(stderr, "Rejected direct JPEG seek gap or oversized image\n");
    return false;
  }
  size_t next_cursor = position + response.data_size;
  size_t next_extent = next_cursor > download->extent ? next_cursor : download->extent;
  if (position != download->extent) {
    uint8_t block[TRANSFER_SIZE];
    size_t moved = 0;
    while (moved < response.data_size) {
      if (!remaining_ms(transport)) return false;
      size_t amount = response.data_size - moved;
      if (amount > sizeof(block)) amount = sizeof(block);
      if (fseeko(file, (off_t)(download->extent + moved), SEEK_SET) != 0 ||
          fread(block, 1, amount, file) != amount ||
          fseeko(file, (off_t)(position + moved), SEEK_SET) != 0 ||
          fwrite(block, 1, amount, file) != amount) return false;
      moved += amount;
    }
    if (fflush(file) != 0 || ftruncate(fileno(file), (off_t)next_extent) != 0) return false;
  }
  if (fseeko(file, (off_t)next_cursor, SEEK_SET) != 0) return false;
  download->cursor = next_cursor;
  download->extent = next_extent;
  download->received += response.data_size;
  download->complete = response.parameters[1] == 0;
  return true;
}

/**
 * @brief Shoot to the camera SD card and stream the resulting JPEG to a local file.
 * @param transport Open session on one selected Canon camera.
 * @param file Caller-owned writable temporary stream, positioned at its start.
 * @return True when the download data phase and final response both succeed.
 * @details TempData carries the remote pathname before DownloadFile. File flush,
 * JPEG-marker checks, publication, and cleanup belong to main(). The SD original
 * is retained; conventional IMG_####.JPG naming and JPEG-capable settings are assumed.
 */
struct camera_setting {
  const char *option;
  const char *property;
  unsigned maximum;
};

static const struct camera_setting camera_settings[] = {
  {"--iso-mode", "ISO_MODE", 32767},
  {"--flash", "FLASH_MODE", 2},
  {"--resolution", "RESOLUTION", 8},
  {"--quality", "QUALITY", 2},
  {"--date-stamp", "DATE_STAMP", 3},
  {"--af-assist", "AF_ASSIST_BEAM", 1},
  {"--raw", NULL, 1}
};
#define CAMERA_SETTING_COUNT (sizeof(camera_settings) / sizeof(camera_settings[0]))

enum focus_mode { FOCUS_UNCHANGED, FOCUS_AF, FOCUS_AFLOCK, FOCUS_MANUAL, FOCUS_AUTO };

struct focus_policy {
  enum focus_mode mode;
  bool distance_present;
  int distance_mm;
  unsigned timeout_ms, tolerance_mm;
};

enum exposure_parameter { EXPOSURE_TV, EXPOSURE_SV, EXPOSURE_AV, EXPOSURE_ND, EXPOSURE_COUNT };
enum exposure_iso_kind { EXPOSURE_ISO_SV96, EXPOSURE_ISO_REAL, EXPOSURE_ISO_MARKET };

struct exposure_policy {
  bool present[EXPOSURE_COUNT];
  int values[EXPOSURE_COUNT];
  unsigned timeout_ms;
  unsigned tolerance96, shutter_numerator, shutter_denominator, aperture_milli;
  enum exposure_iso_kind iso_kind;
};

struct capture_settings {
  bool present[CAMERA_SETTING_COUNT];
  unsigned values[CAMERA_SETTING_COUNT];
  bool keep_awake_present;
  unsigned keep_awake;
  struct focus_policy focus;
  struct exposure_policy exposure;
};

/** @brief Check whether an explicit exposure override is present.
 * @param exposure Requested exposure policy.
 * @return True if at least one override was supplied.
 * @details Presence, not the numeric value, determines whether a control was requested. */
static bool exposure_requested(const struct exposure_policy *exposure)
{
  for (size_t index = 0; index < EXPOSURE_COUNT; ++index)
    if (exposure->present[index]) return true;
  return false;
}

/** @brief Check whether a capture needs camera configuration before shooting.
 * @param settings Parsed optional settings.
 * @return True if a focus, exposure, power or camera setting was requested.
 * @details Skip the configuration script when the user requested no changes. */
static bool settings_requested(const struct capture_settings *settings)
{
  if (settings->keep_awake_present || settings->focus.mode != FOCUS_UNCHANGED ||
      exposure_requested(&settings->exposure)) return true;
  for (size_t index = 0; index < CAMERA_SETTING_COUNT; ++index)
    if (settings->present[index]) return true;
  return false;
}

/** @brief Build the focus setup and readback portion of a CHDK script.
 * @param focus Requested mode, distance and verification bounds.
 * @param[out] body Script buffer; discard on failure.
 * @param capacity Available bytes including the terminator.
 * @param exposure_cleanup Include the exposure-abort hook in focus failure handling.
 * @return True if the policy is supported locally and the script fits.
 * @details Validate mode combinations before formatting. The camera script
 * checks model capabilities and bounds waits for mode and distance readback. */
static bool focus_script(const struct focus_policy *focus, char *body, size_t capacity, bool exposure_cleanup)
{
  if (focus->mode == FOCUS_UNCHANGED) { if (capacity) body[0] = '\0'; return capacity != 0; }
  if (focus->mode > FOCUS_AUTO || !focus->timeout_ms || focus->timeout_ms > 60000 ||
      (focus->distance_present && focus->distance_mm != -1 && focus->distance_mm < 1) ||
      (focus->mode == FOCUS_AF && focus->distance_present) ||
      ((focus->mode == FOCUS_AUTO || focus->mode == FOCUS_MANUAL) && !focus->distance_present)) return false;
  char distance[24] = "nil";
  if (focus->distance_present && snprintf(distance, sizeof(distance), "%d", focus->distance_mm) < 0) return false;
  int length = snprintf(body, capacity,
    "local props=require('propcase')\n"
    "local mode=%u local distance=%s local timeout=%u local tolerance=%u\n"
    "for _,name in ipairs{'set_mf','set_aflock','get_focus','get_focus_ok','get_shooting','press','release'} do\n"
    " if type(_G[name])~='function' then error('missing focus API: '..name) end\n"
    "end\n"
    "if type(props.AF_LOCK)~='number' or type(props.FOCUS_MODE)~='number' then error('missing focus properties') end\n"
    "local _,video=get_mode() if video then error('still-photo mode required') end\n"
    "if distance then\n"
    " if type(get_sd_over_modes)~='function' or type(set_focus)~='function' then error('missing distance API') end\n"
    " local modes=get_sd_over_modes()\n"
    " if type(modes)~='number' or modes<0 or modes%%1~=0 then error('invalid capabilities') end\n"
    " local function supported(flag) return modes%%(flag*2)>=flag end\n"
    " if mode==4 then\n"
    "  if supported(4) then mode=3 elseif supported(2) then mode=2 else error('no MF/AFL distance support') end\n"
    " elseif not supported(mode==3 and 4 or 2) then error('distance mode unsupported') end\n"
    "end\n"
    "if mode==1 or (mode==2 and not distance and get_prop(props.AF_LOCK)~=1) then\n"
    " for _,name in ipairs{'CONTINUOUS_AF','SERVO_AF'} do\n"
    "  if props[name] and get_prop(props[name])~=0 then error('disable '..name) end\n"
    " end\n"
    "end\n"
    "local held=false\n"
    "local started=0\n"
    "local function focus_release() if held then release('shoot_half') held=false end end\n"
    "local function fail(message) focus_release() %serror('focus: '..message) end\n"
    "local function wait(check,message)\n"
    " while true do\n"
    "  local elapsed=bitand(get_tick_count()-started,2147483647)\n"
    "  if elapsed>=timeout then fail(message..'; actual='..tostring(get_focus())) end\n"
    "  if check()==true then return end sleep(20)\n"
    " end\n"
    "end\n"
    "local function selected()\n"
    " return get_prop(props.AF_LOCK)==(mode==2 and 1 or 0) and get_prop(props.FOCUS_MODE)==(mode==3 and 1 or 0)\n"
    "end\n"
    "local function halfpress()\n"
    " if not held then held=true press('shoot_half') end\n"
    " wait(get_shooting,'half-press timeout')\n"
    "end\n"
    "local function focus_prepare()\n"
    " started=get_tick_count()\n"
    " if mode~=2 and get_prop(props.AF_LOCK)==1 then set_aflock(false) end\n"
    " if mode~=3 and get_prop(props.FOCUS_MODE)==1 then set_mf(false) end\n"
    " wait(function() return (mode==2 or get_prop(props.AF_LOCK)==0) and (mode==3 or get_prop(props.FOCUS_MODE)==0) end,'unlock rejected')\n"
    " if mode==3 and not selected() then set_mf(true) end\n"
    " if mode==2 and not selected() then\n"
    "  if not distance then halfpress() wait(get_focus_ok,'AF timeout') end\n"
    "  set_aflock(true)\n"
    " end\n"
    " wait(selected,'mode rejected')\n"
    " if distance and set_focus(distance)~=true then fail('override rejected') end\n"
    " halfpress()\n"
    " if mode==1 then wait(get_focus_ok,'AF timeout') end\n"
    " wait(function()\n"
    "  local actual=get_focus() if type(actual)~='number' then return false end\n"
    "  if distance==-1 then return actual==-1 end\n"
    "  if distance then return actual>0 and actual-distance>=-tolerance and actual-distance<=tolerance end\n"
    "  return actual==-1 or actual>0\n"
    " end,'distance timeout')\n"
    "end\n",
    (unsigned)focus->mode, distance, focus->timeout_ms, focus->tolerance_mm,
    exposure_cleanup ? "if exposure_abort then exposure_abort() end " : "");
  return length >= 0 && (size_t)length < capacity;
}

/** @brief Assemble bounded Lua for applying the requested camera settings.
 * @param settings Parsed settings and focus/exposure policies.
 * @param[out] body Script buffer; discard on failure.
 * @param capacity Available bytes including the terminator.
 * @return True when the script fits and local policy checks pass.
 * @details Share the setup and cleanup fragments between configuration and
 * capture so both paths apply the same camera-side checks. */
static bool settings_script(const struct capture_settings *settings, char *body, size_t capacity)
{
  if (!focus_script(&settings->focus, body, capacity, exposure_requested(&settings->exposure))) return false;
  size_t used = strlen(body);
  if (exposure_requested(&settings->exposure)) {
    const struct exposure_policy *exposure = &settings->exposure;
    int length = snprintf(body + used, capacity - used,
      "local overrides={}\n"
      "local exp_timeout=%u local exp_held=false\n"
      "for _,name in ipairs{'get_shooting','press','release'} do\n"
      " if type(_G[name])~='function' then error('missing exposure API: '..name) end\n"
      "end\n"
      "%s",
      exposure->timeout_ms ? exposure->timeout_ms : 5000,
      settings->focus.mode == FOCUS_UNCHANGED ?
      "local _,video=get_mode() if video then error('still-photo mode required') end\n" : "");
    if (length < 0 || (size_t)length >= capacity - used) return false;
    used += (size_t)length;
    static const char *const names[EXPOSURE_COUNT][4] = {
      {"TV96", "set_tv96_direct", "get_tv96", "set_tv96_direct"}, {"SV96", "set_sv96", "get_sv96", "set_sv96"},
      {"AV96", "set_av96_direct", "get_av96", "set_av96_direct"}, {"ND", "set_nd_filter", "get_nd_current_ev96", "set_nd_filter"}
    };
    for (size_t index = 0; index < EXPOSURE_COUNT; ++index) {
      if (!exposure->present[index]) continue;
      const char *setter = names[index][1];
      char argument[32], expected[64];
      int argument_length = snprintf(argument, sizeof(argument), "%d", exposure->values[index]);
      int expected_length = snprintf(expected, sizeof(expected), "%d", exposure->values[index]);
      if (argument_length < 0 || (size_t)argument_length >= sizeof(argument) ||
          expected_length < 0 || (size_t)expected_length >= sizeof(expected)) return false;
      if (index == EXPOSURE_TV && exposure->shutter_numerator) {
        length = snprintf(body + used, capacity - used,
          "if type(seconds_to_tv96)~='function' then error('missing shutter conversion API') end\n"
          "local target_tv=seconds_to_tv96(%u,%u)\n",
          exposure->shutter_numerator, exposure->shutter_denominator);
        if (length < 0 || (size_t)length >= capacity - used) return false;
        used += (size_t)length;
        snprintf(argument, sizeof(argument), "target_tv");
        snprintf(expected, sizeof(expected), "target_tv");
      } else if (index == EXPOSURE_SV && exposure->iso_kind != EXPOSURE_ISO_SV96) {
        length = snprintf(body + used, capacity - used,
          "if type(iso_to_sv96)~='function' %s then error('missing ISO conversion API') end\n"
          "local real_iso=%s(%d)\n"
          "if type(real_iso)~='number' or real_iso%%1~=0 or real_iso<=0 or real_iso>32767 then error('invalid real ISO conversion') end\n",
          exposure->iso_kind == EXPOSURE_ISO_MARKET ? "or type(iso_market_to_real)~='function'" : "",
          exposure->iso_kind == EXPOSURE_ISO_MARKET ? "iso_market_to_real" : "", exposure->values[index]);
        if (length < 0 || (size_t)length >= capacity - used) return false;
        used += (size_t)length;
        snprintf(argument, sizeof(argument), "real_iso");
        snprintf(expected, sizeof(expected), "iso_to_sv96(real_iso)");
        setter = "set_iso_real";
      } else if (index == EXPOSURE_AV) {
        length = snprintf(body + used, capacity - used,
          "if type(get_nd_present)~='function' or type(get_min_av96)~='function' or type(get_max_av96)~='function' then error('iris/range unavailable') end\n"
          "local iris_type=get_nd_present() if iris_type~=0 and iris_type~=2 then error('no verified iris capability') end\n");
        if (length < 0 || (size_t)length >= capacity - used) return false;
        used += (size_t)length;
        if (exposure->aperture_milli) {
          length = snprintf(body + used, capacity - used,
            "if type(aperture_to_av96)~='function' then error('missing aperture conversion API') end\n"
            "local target_av=aperture_to_av96(%u)\n", exposure->aperture_milli);
          if (length < 0 || (size_t)length >= capacity - used) return false;
          used += (size_t)length;
          snprintf(argument, sizeof(argument), "target_av");
          snprintf(expected, sizeof(expected), "target_av");
          setter = "set_av96";
        }
      } else if (index == EXPOSURE_ND) {
        length = snprintf(body + used, capacity - used,
          "if type(get_nd_present)~='function' or type(get_nd_value_ev96)~='function' then error('ND filter unavailable') end\n"
          "local nd_type=get_nd_present() if nd_type~=1 and nd_type~=2 then error('ND filter unavailable') end\n"
          "local nd_value=get_nd_value_ev96() if type(nd_value)~='number' or nd_value<=0 then error('unknown ND attenuation') end\n");
        if (length < 0 || (size_t)length >= capacity - used) return false;
        used += (size_t)length;
        snprintf(expected, sizeof(expected), "%s", exposure->values[index] == 1 ? "nd_value" : "0");
      }
      length = snprintf(body + used, capacity - used,
        "overrides[#overrides+1]={'%s','%s','%s',%s,%s,%u,'%s',%d,%d,%d}\n",
        names[index][0],
        setter, names[index][2], argument, expected, index == EXPOSURE_ND ? 0 : exposure->tolerance96,
        names[index][3], index == EXPOSURE_TV ? 32767 : 0,
        index == EXPOSURE_TV ? -576 : index == EXPOSURE_ND ? 0 : 1,
        index == EXPOSURE_TV ? 1920 : index == EXPOSURE_AV ? 1152 : 1536);
      if (length < 0 || (size_t)length >= capacity - used) return false;
      used += (size_t)length;
    }
    length = snprintf(body + used, capacity - used,
      "for _,entry in ipairs(overrides) do\n"
      " for _,slot in ipairs{2,3,7} do entry[slot]=_G[entry[slot]] if type(entry[slot])~='function' then error('missing '..entry[1]..' API') end end\n"
      " if type(entry[4])~='number' or type(entry[5])~='number' or entry[4]%%1~=0 or entry[5]%%1~=0 then error('invalid exposure conversion: '..entry[1]) end\n"
      " if entry[5]<entry[9] or entry[5]>entry[10] then error('exposure bounds: '..entry[1]) end\n"
      "end\n"
      "local function exp_release() if exp_held then release('shoot_half') exp_held=false end end\n"
      "function restore() if get_shooting()==false then for _,entry in ipairs(overrides) do entry[7](entry[8]) end end end\n"
      "function exposure_abort()\n"
      " exp_release() %s local stopped=get_tick_count()\n"
      " while get_shooting()~=false do if bitand(get_tick_count()-stopped,2147483647)>=1000 then error('exposure cleanup: camera still shooting') end sleep(20) end restore()\n"
      "end\n"
      "local function exp_fail(message) exposure_abort() error('exposure: '..message) end\n"
      "local function exp_stage() for _,entry in ipairs(overrides) do entry[2](entry[4]) end end\n"
      "local function exp_prepare()\n"
      " local started=get_tick_count()\n"
      " if get_shooting()~=true then exp_held=true press('shoot_half') end\n"
      " while get_shooting()~=true do\n"
      "  if bitand(get_tick_count()-started,2147483647)>=exp_timeout then exp_fail('half-press timeout') end sleep(20)\n"
      " end\n"
      " for _,entry in ipairs(overrides) do\n"
      "  if entry[1]=='AV96' then\n"
      "   local low,high=get_min_av96(),get_max_av96()\n"
      "   if type(low)~='number' or type(high)~='number' or low<=0 or high<=low or entry[5]<low or entry[5]>high then exp_fail('aperture outside verified iris range') end\n"
      "  end\n"
      "  if entry[3]()~=entry[5] then entry[2](entry[4]) end\n"
      " end\n"
      " while true do\n"
      "  if get_shooting()~=true then exp_fail('half-press lost before exposure') end\n"
      "  local bad=nil\n"
      "  for _,entry in ipairs(overrides) do\n"
      "   local actual=entry[3]()\n"
      "   if type(actual)~='number' or actual~=actual or actual-entry[5]>entry[6] or actual-entry[5]<-entry[6] then bad=entry break end\n"
      "  end\n"
      "  if bitand(get_tick_count()-started,2147483647)>=exp_timeout then exp_fail(bad and (bad[1]..' rejected; actual='..tostring(bad[3]())..'; expected='..bad[5]) or 'verification timeout') end\n"
      "  if not bad then return end sleep(20)\n"
      " end\n"
      "end\n",
      settings->focus.mode == FOCUS_UNCHANGED ? "" : "focus_release()");
    if (length < 0 || (size_t)length >= capacity - used) return false;
    used += (size_t)length;
  }
  int length = snprintf(body + used, capacity - used,
    "local props=require('propcase')\n"
    "local requested={");
  if (length < 0 || (size_t)length >= capacity - used) return false;
  used += (size_t)length;
  for (size_t index = 0; index < CAMERA_SETTING_COUNT; ++index) {
    if (!settings->present[index] || !camera_settings[index].property) continue;
    length = snprintf(body + used, capacity - used, "{'%s',%u},",
                      camera_settings[index].property, settings->values[index]);
    if (length < 0 || (size_t)length >= capacity - used) return false;
    used += (size_t)length;
  }
  length = snprintf(body + used, capacity - used,
    "}\n"
    "for _,entry in ipairs(requested) do\n"
    " if type(props[entry[1]])~='number' then error('unsupported property '..entry[1]) end\n"
    " if entry[1]=='QUALITY' then\n"
    "  if type(set_prop_str)~='function' or type(get_prop_str)~='function' then error('missing JPEG quality property API') end\n"
    "  entry[3]=string.char(entry[2],0,0,0)\n"
    " end\n"
    "end\n"
    "local capture_mode=0\n"
    "if type(get_mode)=='function' then local _,_,mode=get_mode() if type(mode)=='number' then capture_mode=bitand(mode,255) end end\n"
    "if capture_mode<2 or capture_mode>5 then\n"
    " for index=#requested,1,-1 do if requested[index][1]=='ISO_MODE' then table.remove(requested,index) end end\n"
    "end\n");
  if (length < 0 || (size_t)length >= capacity - used) return false;
  used += (size_t)length;
  if (settings->keep_awake_present) {
    if (settings->keep_awake > 1) return false;
    length = snprintf(body + used, capacity - used,
      "if type(get_config_value)~='function' or type(set_config_value)~='function' then error('missing keep-awake API') end\n"
      "local shutdown=get_config_value(1221)\n"
      "if type(shutdown)~='number' or shutdown<0 or shutdown>3 or shutdown%%1~=0 then error('keep-awake setting unavailable') end\n"
      "if shutdown~=%u then set_config_value(1221,%u) end\n"
      "if get_config_value(1221)~=%u then error('keep-awake setting rejected') end\n",
      settings->keep_awake * 3, settings->keep_awake * 3, settings->keep_awake * 3);
    if (length < 0 || (size_t)length >= capacity - used) return false;
    used += (size_t)length;
  }
  length = snprintf(body + used, capacity - used,
    "local function setting(entry) return entry[3] and get_prop_str(props[entry[1]],4) or get_prop(props[entry[1]]) end\n"
    "local changed=false\n"
    "for _,entry in ipairs(requested) do\n"
    " if setting(entry)~=(entry[3] or entry[2]) then\n"
    "  if entry[3] then\n"
    "   if set_prop_str(props[entry[1]],entry[3])~=true then error('property rejected QUALITY') end\n"
    "  else set_prop(props[entry[1]],entry[2]) end\n"
    "  changed=true\n"
    " end\n"
    "end\n"
    "if changed then sleep(100) end\n"
    "for _,entry in ipairs(requested) do\n"
    " if setting(entry)~=(entry[3] or entry[2]) then error('property rejected '..entry[1]) end\n"
    "end\n");
  if (length < 0 || (size_t)length >= capacity - used) return false;
  used += (size_t)length;
  if (settings->present[CAMERA_SETTING_COUNT - 1]) {
    length = snprintf(body + used, capacity - used,
      "if get_raw()~=(%u~=0) then set_raw(%u) end\nif get_raw()~=(%u~=0) then error('raw setting rejected') end\n",
      settings->values[CAMERA_SETTING_COUNT - 1], settings->values[CAMERA_SETTING_COUNT - 1], settings->values[CAMERA_SETTING_COUNT - 1]);
    if (length < 0 || (size_t)length >= capacity - used) return false;
    used += (size_t)length;
  }
  if (exposure_requested(&settings->exposure)) {
    length = snprintf(body + used, capacity - used, "exp_stage()\n");
    if (length < 0 || (size_t)length >= capacity - used) return false;
    used += (size_t)length;
  }
  if (settings->focus.mode != FOCUS_UNCHANGED) {
    length = snprintf(body + used, capacity - used, "focus_prepare()\n");
    if (length < 0 || (size_t)length >= capacity - used) return false;
    used += (size_t)length;
  }
  if (exposure_requested(&settings->exposure)) {
    length = snprintf(body + used, capacity - used, "exp_prepare()\n");
    if (length < 0 || (size_t)length >= capacity - used) return false;
  }
  return true;
}

/** @brief Build a configuration-only script with an observed-state report.
 * @param settings Requested settings.
 * @param[out] body Destination script buffer.
 * @param capacity Available bytes including the terminator.
 * @return True if setup and the report fit.
 * @details Append readback and release temporary focus/exposure state
 * without taking a picture. */
static bool configure_script(const struct capture_settings *settings, char *body, size_t capacity)
{
  if (!settings_script(settings, body, capacity)) return false;
  size_t used = strlen(body);
  int length = snprintf(body + used, capacity - used, "local report='Configured'\n%s%s%s%sreturn report\n",
    settings->keep_awake_present ? "report=report..'; keep_awake='..(get_config_value(1221)==3 and 1 or 0)\n" : "",
    settings->focus.mode == FOCUS_UNCHANGED ? "" :
    "report=report..'; mode='..({'af','aflock','manual'})[mode]..'; distance_mm='..get_focus()\n",
    exposure_requested(&settings->exposure) ?
    "for _,entry in ipairs(overrides) do report=report..'; '..entry[1]..'='..entry[3]() end\nexposure_abort()\n" : "",
    settings->focus.mode == FOCUS_UNCHANGED ? "" : "focus_release()\n");
  return length >= 0 && (size_t)length < capacity - used;
}

/** @brief Apply settings without capturing a picture.
 * @param transport Active PTP session.
 * @param settings Requested settings and optional readback checks.
 * @return True when a configuration report was accepted.
 * @details Require the expected report shape before displaying an apparent success. */
static bool configure_camera(struct transport *transport, const struct capture_settings *settings)
{
  char body[SCRIPT_BODY_CAPACITY], result[256];
  if (!configure_script(settings, body, sizeof(body)) ||
      !run_script(transport, body, result, sizeof(result)) ||
      (!settings->keep_awake_present && settings->focus.mode == FOCUS_UNCHANGED && !exposure_requested(&settings->exposure) ? strcmp(result, "Configured") :
       strncmp(result, "Configured; ", 12))) return false;
  puts(result);
  return true;
}

/** @brief Build the ordinary SD-backed still-capture script.
 * @param settings Optional settings to apply before exposing.
 * @param[out] body Script buffer.
 * @param capacity Available bytes including the terminator.
 * @return True when the capture program fits.
 * @details Require the exposure counter to advance, then allow the new
 * JPEG to settle before the host attempts to download its path. */
static bool capture_script(const struct capture_settings *settings, char *body, size_t capacity)
{
  static const char capture_body[] =
    "if get_exp_count()==before then error('capture did not advance file counter') end\n"
    "local path=get_image_dir()..string.format('/IMG_%04d.JPG',get_exp_count())\n"
    "local stat=type(os)=='table' and type(os.stat)=='function' and os.stat or nil\n"
    "if not stat then sleep(1500) else\n"
    " local last,still,began=-1,0,get_tick_count()\n"
    " while still<3 and bitand(get_tick_count()-began,2147483647)<5000 do\n"
    "  local info=stat(path) local size=type(info)=='table' and info.size or -1\n"
    "  if size>0 and size==last then still=still+1 else still=0 end\n"
    "  last=size sleep(100)\n"
    " end\n"
    "end\n"
    "return path\n";
  if (!capacity) return false;
  body[0] = '\0';
  if (settings_requested(settings) && !settings_script(settings, body, capacity)) return false;
  size_t used = strlen(body);
  int length = snprintf(body + used, capacity - used,
                        "local before=get_exp_count()\nlocal shot_status=shoot(0)\n%s%sif shot_status~=0 then error('shoot failed; status='..tostring(shot_status)) end\n%s",
                        exposure_requested(&settings->exposure) ? "exposure_abort()\n" : "",
                        settings->focus.mode == FOCUS_UNCHANGED ? "" : "focus_release()\n", capture_body);
  return length >= 0 && (size_t)length < capacity - used;
}

/** @brief Capture to the camera's SD card and download the resulting JPEG.
 * @param transport Active PTP session and deadline.
 * @param file Caller-owned temporary destination stream.
 * @param settings Optional settings for this shot.
 * @return True if the script and file download succeeded.
 * @warning A failed download does not imply the exposure failed. */
static bool capture(struct transport *transport, FILE *file, const struct capture_settings *settings)
{
  char body[SCRIPT_BODY_CAPACITY];
  if (!capture_script(settings, body, sizeof(body))) return false;
  char remote_path[512];
  if (!run_script(transport, body, remote_path, sizeof(remote_path))) return false;
  return download_file(transport, remote_path, file);
}

/** @brief Build the direct JPEG handshake and its camera-side cleanup.
 * @param settings Optional settings for this shot.
 * @param timeout_ms Remaining operation budget supplied to the camera.
 * @param[out] body Script buffer.
 * @param capacity Available bytes including the terminator.
 * @return True when the complete script fits.
 * @details Require single-still support and an unused direct-capture slot.
 * Own cleanup only after enabling the slot; require the host's completion
 * reply before reporting the final image identity. */
static bool direct_capture_script(const struct capture_settings *settings, unsigned timeout_ms,
                                   char *body, size_t capacity)
{
  int length = snprintf(body, capacity,
    "for _,name in ipairs{'get_usb_capture_support','get_usb_capture_target','init_usb_capture','set_usb_capture_timeout','write_usb_msg','read_usb_msg','get_drive_mode'} do\n"
    " if type(_G[name])~='function' then error('missing direct capture API: '..name) end\nend\n"
    "local support=get_usb_capture_support()\n"
    "if type(support)~='number' or support%%2~=1 then error('direct JPEG unsupported; USB capture mask='..tostring(support)) end\n"
    "local _,video=get_mode() if video or get_drive_mode()~=0 then error('single still-photo mode required') end\n"
    "if get_usb_capture_target()~=0 then error('remote capture already enabled; not replaced') end\n");
  if (length < 0 || (size_t)length >= capacity) return false;
  size_t used = (size_t)length;
  if (settings_requested(settings)) {
    if (!settings_script(settings, body + used, capacity - used)) return false;
    used += strlen(body + used);
  }
  length = snprintf(body + used, capacity - used,
    "local owned=false local previous_restore=restore\n"
    "local function direct_release() if owned then init_usb_capture(0) set_usb_capture_timeout() owned=false end end\n"
    "function restore() direct_release() if previous_restore then previous_restore() end end\n"
    "if not init_usb_capture(1) then error('direct JPEG initialization failed') end owned=true\n"
    "set_usb_capture_timeout(%u)\n"
    "if not write_usb_msg('Direct ready') then direct_release() error('direct readiness message failed') end\n"
    "local before=get_exp_count() local started=get_tick_count()\nlocal shot_status=shoot(0)\n"
    "if shot_status~=0 then direct_release() error('shoot failed; status='..tostring(shot_status)) end\n"
    "local remaining=%u-bitand(get_tick_count()-started,2147483647)\n"
    "local done=remaining>0 and read_usb_msg(remaining)=='Direct done'\n"
    "local active=get_usb_capture_target()==1\ndirect_release()\n%s%s"
    "if not done or not active then error('direct capture incomplete; not retried') end\n"
    "if get_exp_count()==before then error('direct capture counter did not advance') end\n"
    "return 'Direct JPEG '..get_exp_count()\n",
    timeout_ms, timeout_ms,
    exposure_requested(&settings->exposure) ? "exposure_abort()\n" : "",
    settings->focus.mode == FOCUS_UNCHANGED ? "" : "focus_release()\n");
  return length >= 0 && (size_t)length < capacity - used;
}

/** @brief Capture and stream a JPEG via CHDK's direct-download protocol.
 * @param transport Active session with an absolute operation deadline.
 * @param file Seekable temporary stream for JPEG chunks.
 * @param settings Optional capture settings.
 * @return True when script completion and transfer agree on the image.
 * @details Pass the remaining budget to the camera rather than extending
 * the original host deadline with a new per-shot timeout. */
static bool capture_direct(struct transport *transport, FILE *file, const struct capture_settings *settings)
{
  char body[SCRIPT_BODY_CAPACITY], result[64];
  uint64_t now_ms = monotonic_ms();
  if (!remaining_ms(transport) || !now_ms || now_ms >= transport->deadline_ms ||
      transport->deadline_ms - now_ms > UINT_MAX ||
      !direct_capture_script(settings, (unsigned)(transport->deadline_ms - now_ms), body, sizeof(body))) {
    fprintf(stderr, "Direct capture script exceeds storage or operation deadline\n");
    return false;
  }
  return run_script_operation(transport, body, result, sizeof(result), file, true);
}

static bool parse_range(const char *text, unsigned minimum, unsigned maximum, unsigned *value);

/**
 * @brief Set normalized optical zoom without taking or downloading a photo.
 * @param transport Open camera session.
 * @param level 0 selects the first zoom step; 255 selects the last.
 * @param observed Optional confirmed normalized lens-step position.
 * @return True after camera-side get_zoom() confirms the requested step.
 * @details The camera computes floor((level * (steps - 1) + 127) / 255), using
 * get_zoom_steps() as a count. This rounds to a lens step, not a focal length.
 * A lens with one step maps every valid level to zero. Digital zoom is not set.
 */
static bool set_zoom(struct transport *transport, unsigned level, uint8_t *observed)
{
  if (level > 255) return false;
  char body[1024], result[128];
  int length = snprintf(body, sizeof(body),
    "local level=%u\n"
    "local steps=get_zoom_steps()\n"
    "if steps<1 then error('camera reports no zoom steps') end\n"
    "local scaled=level*(steps-1)+127\n"
    "local target=(scaled-scaled%%255)/255\n"
    "set_zoom(target)\n"
    "local started=get_tick_count()\n"
    "while get_zoom()~=target and bitand(get_tick_count()-started,2147483647)<5000 do sleep(50) end\n"
    "if get_zoom()~=target then error('zoom did not reach target') end\n"
    "return string.format('%%d/%%d',get_zoom(),steps-1)\n", level);
  if (length < 0 || (size_t)length >= sizeof(body) ||
      !run_script(transport, body, result, sizeof(result))) return false;
  char *separator = strchr(result, '/');
  unsigned step, last;
  if (!separator) return false;
  *separator = '\0';
  if (!parse_range(result, 0, 65535, &step) || !parse_range(separator + 1, 0, 65535, &last)
      || step > last || step != (level * last + 127) / 255) return false;
  if (observed) *observed = last ? (uint8_t)((step * 255 + last / 2) / last) : 0;
  printf("Zoom %u: camera step %u/%u\n", level, step, last);
  return true;
}

/**
 * @brief Validate one USB still-image interface without opening the device.
 * @param descriptor Candidate alternate-setting descriptor.
 * @param input Bulk-IN address, modified only on successful validation.
 * @param output Bulk-OUT address, modified only on successful validation.
 * @return True for PTP 6/1/1 with one valid bulk endpoint in each direction.
 * @details Zero/reserved endpoint addresses, missing packet sizes, duplicate
 * bulk directions, and unsupported endpoint types are rejected. One optional
 * interrupt-IN endpoint is allowed for events, although this client does not
 * consume events. Packet-size checks cover full/high/super-speed bulk limits.
 */
static bool valid_ptp_interface(const struct usbfs_interface *descriptor,
                                 uint8_t *input, uint8_t *output)
{
  if (!descriptor || !input || !output || descriptor->bInterfaceClass != 6 ||
      descriptor->bInterfaceSubClass != 1 || descriptor->bInterfaceProtocol != 1 ||
      descriptor->bNumEndpoints < 2 || descriptor->bNumEndpoints > 3) return false;
  uint8_t bulk_input = 0, bulk_output = 0;
  bool interrupt_input = false;
  for (uint8_t index = 0; index < descriptor->bNumEndpoints; ++index) {
    const struct usbfs_endpoint *endpoint = &descriptor->endpoint[index];
    uint8_t address = endpoint->bEndpointAddress;
    unsigned packet_size = endpoint->wMaxPacketSize & 0x7ffU;
    unsigned type = endpoint->bmAttributes & USBFS_TRANSFER_TYPE_MASK;
    if (!(address & 0x0fU) || (address & 0x70U) || !packet_size || packet_size > 1024)
      return false;
    for (uint8_t previous = 0; previous < index; ++previous)
      if (descriptor->endpoint[previous].bEndpointAddress == address) return false;
    if (type == USBFS_TRANSFER_TYPE_BULK) {
      if ((endpoint->wMaxPacketSize & 0xf800U) ||
          (packet_size != 8 && packet_size != 16 && packet_size != 32 &&
           packet_size != 64 && packet_size != 512 && packet_size != 1024)) return false;
      if (address & USBFS_ENDPOINT_IN) {
        if (bulk_input) return false;
        bulk_input = address;
      } else {
        if (bulk_output) return false;
        bulk_output = address;
      }
    } else if (type == USBFS_TRANSFER_TYPE_INTERRUPT &&
               (address & USBFS_ENDPOINT_IN) && !interrupt_input) {
      interrupt_input = true;
    } else return false;
  }
  if (!bulk_input || !bulk_output) return false;
  *input = bulk_input;
  *output = bulk_output;
  return true;
}

/**
 * @brief Find a still-image PTP interface and its bulk endpoints.
 * @param device Enumerated USB device; ownership stays with the device list.
 * @param interface_number Output interface number, not its array index.
 * @param alternate Output alternate-setting number, not its array index.
 * @param input Output bulk-IN endpoint address on success.
 * @param output Output bulk-OUT endpoint address on success.
 * @param output_packet_size Output bulk-OUT maximum packet size in bytes.
 * @return True for a class/subclass/protocol 6/1/1 interface with both endpoints.
 * @note This examines descriptors, not CHDK support. CHDK is checked after opening
 * the selected device; listing never claims an interface or triggers capture.
 */
static bool find_interface(const struct usbfs_device *device, int *interface_number, int *alternate,
                           uint8_t *input, uint8_t *output, unsigned *output_packet_size)
{
  uint8_t buffer[USBFS_DESCRIPTOR_CAPACITY];
  const uint8_t *config;
  size_t length, offset = 0;
  if (usb.active_config(device, buffer, sizeof(buffer), &config, &length) != 0) return false;
  struct usbfs_interface descriptor;
  while (usbfs_next_interface(config, length, &offset, &descriptor)) {
    if (valid_ptp_interface(&descriptor, input, output)) {
      *interface_number = descriptor.bInterfaceNumber;
      *alternate = descriptor.bAlternateSetting;
      *output_packet_size = 0;
      for (uint8_t index = 0; index < descriptor.bNumEndpoints; ++index)
        if (descriptor.endpoint[index].bEndpointAddress == *output)
          *output_packet_size = descriptor.endpoint[index].wMaxPacketSize & 0x7ffU;
      return true;
    }
  }
  return false;
}

/** @brief Parse unsigned decimal CLI input with inclusive bounds.
 * @param text Nonempty digits; signs, whitespace, and trailing text are rejected.
 * @param minimum Lowest accepted number, inclusive.
 * @param maximum Highest accepted number, inclusive.
 * @param value Output destination, modified only on success.
 * @return True for a representable value in the specified range. */
static bool parse_range(const char *text, unsigned minimum, unsigned maximum, unsigned *value)
{
  char *end;
  errno = 0;
  unsigned long parsed = strtoul(text, &end, 10);
  if (errno || !*text || *text < '0' || *text > '9' || *end ||
      parsed < minimum || parsed > maximum) return false;
  *value = (unsigned)parsed;
  return true;
}

/** @brief Parse a positive CLI number; zero is reserved for unspecified selection. */
static bool parse_number(const char *text, unsigned maximum, unsigned *value)
{
  return parse_range(text, 1, maximum, value);
}

/** @brief Parse a bounded signed decimal command-line value.
 * @param text Null-terminated token, optionally prefixed by minus.
 * @param minimum Inclusive lower bound.
 * @param maximum Inclusive upper bound.
 * @param[out] value Result written on success.
 * @return True for a complete in-range token.
 * @details Check both syntax and range before narrowing the conversion. */
static bool parse_signed(const char *text, int minimum, int maximum, int *value)
{
  const char *digits = *text == '-' ? text + 1 : text;
  char *end;
  errno = 0;
  long parsed = strtol(text, &end, 10);
  if (*digits < '0' || *digits > '9' || errno || *end || parsed < minimum || parsed > maximum) return false;
  *value = (int)parsed;
  return true;
}

/** @brief Parse a decimal setting and convert it to rounded integer units.
 * @param text Null-terminated numeric token beginning with a decimal digit.
 * @param scale Multiplier applied before checking bounds.
 * @param minimum Inclusive minimum in scaled units.
 * @param maximum Inclusive maximum in scaled units.
 * @param[out] value Rounded result written on success.
 * @return True when syntax and scaled range are accepted.
 * @details Reject hexadecimal and non-finite inputs rather than passing unusual
 * spellings through to camera-side option handling. */
static bool parse_scaled(const char *text, unsigned scale, unsigned minimum, unsigned maximum, unsigned *value)
{
  char *end;
  errno = 0;
  double parsed = strtod(text, &end) * scale;
  if (*text < '0' || *text > '9' || (text[0] == '0' && (text[1] == 'x' || text[1] == 'X')) ||
      errno || *end || !(parsed >= minimum && parsed <= maximum)) return false;
  *value = (unsigned)(parsed + 0.5);
  return true;
}

/** @brief Convert a decimal or rational shutter time to an exposure ratio.
 * @param text Exposure in seconds, as a decimal or numerator/denominator.
 * @param[in,out] exposure Policy receiving the checked ratio.
 * @return True for an exposure from one microsecond through 64 seconds.
 * @details Keep fractions exact and convert decimal seconds to microseconds;
 * validate rational bounds with widened integers before updating the policy. */
static bool parse_shutter(const char *text, struct exposure_policy *exposure)
{
  unsigned numerator, denominator;
  const char *slash = strchr(text, '/');
  if (slash) {
    char *end;
    errno = 0;
    unsigned long parsed = strtoul(text, &end, 10);
    if (*text < '0' || *text > '9' || errno || end != slash || parsed < 1 || parsed > 64000000 ||
        !parse_number(slash + 1, 1000000, &denominator) || parsed > (uint64_t)64 * denominator ||
        parsed * UINT64_C(1000000) < denominator) return false;
    numerator = (unsigned)parsed;
  } else {
    if (!parse_scaled(text, 1000000, 1, 64000000, &numerator)) return false;
    denominator = 1000000;
  }
  exposure->shutter_numerator = numerator;
  exposure->shutter_denominator = denominator;
  return true;
}

struct usb_port_selection {
  unsigned bus;
  uint8_t ports[7];
  size_t depth;
};

/** @brief Parse a stable USB bus and port-chain selector.
 * @param text USB path in BUS-PORT[.PORT...] form.
 * @param[out] selection Parsed selector; discard on failure.
 * @return True when the complete nonempty chain fits.
 * @details Select the physical port independently of changing device addresses. */
static bool parse_usb_port(const char *text, struct usb_port_selection *selection)
{
  *selection = (struct usb_port_selection){0};
  char *end;
  errno = 0;
  unsigned long bus = strtoul(text, &end, 10);
  if (errno || *text < '0' || *text > '9' || bus < 1 || bus > 255 || *end != '-') return false;
  selection->bus = (unsigned)bus;
  text = end + 1;
  for (;;) {
    errno = 0;
    unsigned long port = strtoul(text, &end, 10);
    if (errno || *text < '0' || *text > '9' || port < 1 || port > 255 || selection->depth == sizeof(selection->ports)) return false;
    selection->ports[selection->depth++] = (uint8_t)port;
    if (!*end) return true;
    if (*end != '.') return false;
    text = end + 1;
  }
}

/** @brief Compare a discovered device with the optional port selector.
 * @param selection Parsed selector; empty depth disables the filter.
 * @param bus Discovered bus number.
 * @param ports Discovered port chain with @p depth entries when valid.
 * @param depth Discovered chain length, or a negative unavailable value.
 * @return True for an exact match or an unconfigured selector.
 * @details Do not accept a prefix match: another device may be farther
 * downstream on the same hub. */
static bool usb_port_matches(const struct usb_port_selection *selection, unsigned bus,
                             const uint8_t *ports, int depth)
{
  if (!selection->depth) return true;
  if (bus != selection->bus || depth < 0 || (size_t)depth != selection->depth) return false;
  for (size_t index = 0; index < selection->depth; ++index)
    if (ports[index] != selection->ports[index]) return false;
  return true;
}

struct desktop_api {
  void *(*monitor_get)(void);
  int (*context_iteration)(void *, int);
  void *(*volumes_get)(void *);
  void *(*list_nth)(void *, unsigned);
  char *(*identifier_get)(void *, const char *);
  void *(*mount_get)(void *);
  void *(*root_get)(void *);
  char *(*uri_get)(void *);
  void (*unref)(void *);
  void (*free)(void *);
  void (*list_free_full)(void *, void (*)(void *));
};

/** @brief Find the unique desktop mount attached to the selected USB node.
 * @param api Loaded desktop volume-monitor functions.
 * @param device Exact USB device path to match.
 * @param[out] uri Buffer for a matching gphoto2 URI.
 * @param capacity Available bytes including the terminator.
 * @return True only for a unique matching mount within the scan bound.
 * @details Match the desktop volume by device identity before considering its
 * URI; never choose an unrelated camera by mount order. */
static bool desktop_camera_uri(const struct desktop_api *api, const char *device,
                               char *uri, size_t capacity)
{
  if (!capacity) return false;
  uri[0] = '\0';
  void *monitor = api->monitor_get();
  if (!monitor) return false;
  void *volumes = api->volumes_get(monitor);
  bool valid = true;
  for (unsigned index = 0; index < DESKTOP_VOLUME_LIMIT; ++index) {
    void *volume = api->list_nth(volumes, index);
    if (!volume) break;
    char *identifier = api->identifier_get(volume, "unix-device");
    bool selected = identifier && !strcmp(identifier, device);
    api->free(identifier);
    if (!selected) continue;
    void *mount = api->mount_get(volume);
    if (!mount) continue;
    void *root = api->root_get(mount);
    char *location = root ? api->uri_get(root) : NULL;
    size_t length = location ? strnlen(location, capacity) : 0;
    if (!location || length < 10 || length >= capacity ||
        strncmp(location, "gphoto2://", 10) || uri[0]) valid = false;
    if (valid) {
      for (size_t byte = 0; byte <= length; ++byte) uri[byte] = location[byte];
    }
    api->free(location);
    if (root) api->unref(root);
    api->unref(mount);
    if (!valid) break;
  }
  if (api->list_nth(volumes, DESKTOP_VOLUME_LIMIT)) valid = false;
  api->list_free_full(volumes, api->unref);
  api->unref(monitor);
  if (!valid) uri[0] = '\0';
  return valid && uri[0];
}

/** @brief Resolve one optional desktop camera mount and invoke its unmount.
 * @param bus Selected USB bus.
 * @param address Selected device address.
 * @return One on failure; successful exec replaces this helper process.
 * @details Load desktop dependencies only in the helper so headless
 * operation does not require the GIO runtime. */
static int unmount_desktop_camera(unsigned bus, unsigned address)
{
  void *library = dlopen("libgio-2.0.so.0", RTLD_NOW | RTLD_LOCAL);
  if (!library) {
    fprintf(stderr, "chdkbridge: optional desktop GIO runtime unavailable; release the selected camera mount manually\n");
    return 1;
  }
  struct desktop_api api = {0};
#define DESKTOP_SYMBOL(FIELD, NAME) \
  api.FIELD = (__typeof__(api.FIELD))dlsym(library, NAME); \
  if (!api.FIELD) { fprintf(stderr, "chdkbridge: missing desktop API %s\n", NAME); return 1; }
  DESKTOP_SYMBOL(monitor_get, "g_volume_monitor_get")
  DESKTOP_SYMBOL(context_iteration, "g_main_context_iteration")
  DESKTOP_SYMBOL(volumes_get, "g_volume_monitor_get_volumes")
  DESKTOP_SYMBOL(list_nth, "g_list_nth_data")
  DESKTOP_SYMBOL(identifier_get, "g_volume_get_identifier")
  DESKTOP_SYMBOL(mount_get, "g_volume_get_mount")
  DESKTOP_SYMBOL(root_get, "g_mount_get_root")
  DESKTOP_SYMBOL(uri_get, "g_file_get_uri")
  DESKTOP_SYMBOL(unref, "g_object_unref")
  DESKTOP_SYMBOL(free, "g_free")
  DESKTOP_SYMBOL(list_free_full, "g_list_free_full")
#undef DESKTOP_SYMBOL
  void *monitor = api.monitor_get();
  if (!monitor) return 1;
  for (unsigned iteration = 0; iteration < DESKTOP_DISCOVERY_POLLS; ++iteration) {
    for (unsigned event = 0; event < 64 && api.context_iteration(NULL, 0); ++event) {}
    if (poll(NULL, 0, DESKTOP_DISCOVERY_POLL_MS) < 0 && errno != EINTR) {
      api.unref(monitor);
      return 1;
    }
  }
  char device[32], uri[PATH_MAX];
  int length = snprintf(device, sizeof(device), "/dev/bus/usb/%03u/%03u", bus, address);
  bool found = length >= 0 && (size_t)length < sizeof(device) &&
               desktop_camera_uri(&api, device, uri, sizeof(uri));
  api.unref(monitor);
  if (!found) {
    fprintf(stderr, "chdkbridge: no unique mounted desktop camera for USB %u:%u\n", bus, address);
    return 1;
  }
  fprintf(stderr, "chdkbridge: releasing selected desktop camera mount %s\n", uri);
  execlp("gio", "gio", "mount", "--unmount", uri, (char *)NULL);
  fprintf(stderr, "chdkbridge: cannot run optional desktop gio tool: %s\n", strerror(errno));
  return 1;
}

/** @brief Supervise a targeted desktop unmount helper.
 * @param transport Operation whose deadline bounds the helper.
 * @param bus Selected USB bus.
 * @param address Selected device address.
 * @return True if the helper exited successfully.
 * @details Run the helper in its own process group and terminate it on
 * expiry or cancellation. Success permits a claim attempt, not assumed ownership. */
static bool release_desktop_camera(struct transport *transport, unsigned bus, unsigned address)
{
  uint64_t started = monotonic_ms();
  if (!started || !remaining_ms(transport)) return false;
  uint64_t deadline = started + DESKTOP_RELEASE_MS;
  if (deadline > transport->deadline_ms) deadline = transport->deadline_ms;
  char bus_text[4], address_text[4];
  if (snprintf(bus_text, sizeof(bus_text), "%u", bus) < 0 ||
      snprintf(address_text, sizeof(address_text), "%u", address) < 0) return false;
  pid_t child = fork();
  if (child < 0) { perror("Starting desktop camera release"); return false; }
  if (child == 0) {
    if (setpgid(0, 0) != 0) _exit(127);
    signal(SIGINT, SIG_DFL);
    signal(SIGTERM, SIG_DFL);
    int input = open("/dev/null", O_RDONLY);
    if (input < 0 || dup2(input, STDIN_FILENO) < 0 ||
        dup2(STDERR_FILENO, STDOUT_FILENO) < 0) _exit(127);
    if (input != STDIN_FILENO) close(input);
    execl("/proc/self/exe", "chdkbridge", "--internal-unmount", bus_text, address_text, (char *)NULL);
    _exit(127);
  }
  (void)setpgid(child, child);
  int status = 0;
  while (true) {
    pid_t result = waitpid(child, &status, WNOHANG);
    if (result == child) return WIFEXITED(status) && WEXITSTATUS(status) == 0;
    if (result < 0 && errno != EINTR) return false;
    uint64_t now = monotonic_ms();
    if (interrupted || !now || now >= deadline) break;
    int delay = deadline - now < 10 ? (int)(deadline - now) : 10;
    if (poll(NULL, 0, delay) < 0 && errno != EINTR) break;
  }
  (void)kill(-child, SIGKILL);
  (void)kill(child, SIGKILL);
  while (waitpid(child, &status, 0) < 0 && errno == EINTR) {}
  fprintf(stderr, "chdkbridge: selected desktop camera release cancelled or timed out\n");
  return false;
}

/** @brief Claim an interface, optionally releasing its selected desktop mount.
 * @param transport Active device transport and deadline.
 * @param interface_number USB interface number to claim.
 * @param bus Selected USB bus.
 * @param address Selected device address.
 * @param release_mount Targeted desktop release callback.
 * @return USBFS_SUCCESS or a negative USBFS status.
 * @details Only a busy claim triggers desktop release. Use bounded polling
 * to allow that owner to close the descriptor; do not reset the USB device. */
static int claim_camera_interface(struct transport *transport, int interface_number,
                                  unsigned bus, unsigned address,
                                  bool (*release_mount)(struct transport *, unsigned, unsigned))
{
  if (!remaining_ms(transport)) return interrupted ? USBFS_ERROR_INTERRUPTED : USBFS_ERROR_TIMEOUT;
  int result = usb.claim_interface(transport->handle, interface_number);
  if (result != USBFS_ERROR_BUSY || !release_mount(transport, bus, address)) return result;
  uint64_t now = monotonic_ms();
  if (!now) return USBFS_ERROR_IO;
  uint64_t deadline = now + DESKTOP_RECLAIM_MS;
  if (deadline > transport->deadline_ms) deadline = transport->deadline_ms;
  while (remaining_ms(transport)) {
    result = usb.claim_interface(transport->handle, interface_number);
    if (result != USBFS_ERROR_BUSY) return result;
    now = monotonic_ms();
    if (!now) return USBFS_ERROR_IO;
    if (now >= deadline) return result;
    int delay = deadline - now < DESKTOP_RECLAIM_POLL_MS ? (int)(deadline - now) : DESKTOP_RECLAIM_POLL_MS;
    if (poll(NULL, 0, delay) < 0 && errno != EINTR) return USBFS_ERROR_IO;
  }
  return interrupted ? USBFS_ERROR_INTERRUPTED : USBFS_ERROR_TIMEOUT;
}

/** @brief Print build identity and CLI usage without initializing USB.
 * @param program Program name from argv[0]. */
static void print_help(const char *program)
{
  puts(CHDKBRIDGE_BUILD_VERSION);
  printf("Usage: %s [options] OUTPUT.jpg\n"
         "       %s [options] zoom LEVEL\n"
         "       %s [options] --info\n"
      "       %s [options] --retract-lens\n"
      "       %s [options] --power-status\n"
         "       %s [options] --download A/DCIM/.../IMAGE.JPG OUTPUT.jpg\n"
         "       %s --list\n\n"
         "Options:\n"
         "  --help, -h        Show this help.\n"
         "  --version, -V     Show application version and build Git revision.\n"
         "  --list            List Canon USB PTP devices without claiming them.\n"
         "  --info            Query CHDK protocol, Lua support, and script status.\n"
         "  --retract-lens    Request playback/lens retraction without powering off or shooting.\n"
         "  --power-status    Read-only: report idle power-off prevention, battery and temperatures; fails unless prevention is Always.\n"
         "  --romlog OUTPUT   Export and download the Canon ROM crash log (max 64 KiB); no record mode, shoot, zoom or power-off.\n"
         "  --check-runtime   Check Linux USB device access without opening a camera.\n"
         "  --download PATH   Download an existing camera JPEG without shooting.\n"
         "  --direct-download Capture JPEG directly over USB; no JPEG saved on SD.\n"
         "  --bus N           Select USB bus (requires --device).\n"
         "  --device N        Select USB device address (requires --bus).\n"
         "  --port BUS-P[.P]  Select physical USB port path from --list (survives device readdressing).\n"
         "  --timeout SECONDS Operation timeout, 1..3600 (default: 60).\n\n"
         "  --usb-timeout MS  Individual USB-call timeout, 50..10000 (default: 1000).\n"
         "  --max-download-mib N  File-size limit, 1..256 MiB (default: 64); streaming RAM is fixed.\n\n"
         "  --configure       Apply settings without shooting (requires settings).\n"
         "  --iso-mode N      Canon ISO_MODE property, 0..32767; 0 is auto.\n"
         "  --flash N         0 auto, 1 on, 2 off.\n"
         "  --resolution N    Camera resolution code, 0..8 (model-dependent).\n"
         "  --quality N       0 superfine, 1 fine, 2 normal (model-dependent).\n"
         "  --date-stamp N    Camera date-stamp code, 0..3 (model-dependent).\n"
         "  --af-assist N     AF assist beam: 0 off, 1 on.\n"
         "  --raw N           CHDK raw saving: 0 off, 1 on.\n"
         "  --keep-awake N    CHDK idle-shutdown prevention: 0 Never, 1 Always; saved by default.\n\n"
         "  --focus-mode MODE af: refocus; aflock: preserve/acquire lock; manual: fixed distance; auto: supported MF/AFL distance.\n"
         "  --focus-distance MM|inf  Distance in millimeters, 1..1000000, or infinity; requires manual/auto/aflock.\n"
         "  --focus-timeout MS Total focus-wait budget, 50..60000 (default: 5000); less than --timeout.\n"
         "  --focus-tolerance MM  Distance readback tolerance, 0..100000 (default: 100); infinity requires -1 readback.\n\n"
         "  --shutter SECONDS Shutter override: decimal seconds or fraction (e.g. 0.001 or 1/1000), 1 us..64 s.\n"
         "  --tv96 N          Direct shutter APEX96, -576..1920; alternative to --shutter.\n"
         "  --iso-real N      Real sensor ISO, 1..32767; model limits/readback apply.\n"
         "  --iso-market N    Camera-calibrated conversion of displayed ISO, 1..32767.\n"
         "  --sv96 N          Direct real-sensitivity APEX96, 1..1536; alternative ISO override.\n"
         "  --aperture F      F-number, 1..64; requires verified adjustable-iris range.\n"
         "  --av96 N          Direct aperture APEX96, 1..1152; alternative to --aperture.\n"
         "  --nd in|out       Explicit ND insertion/removal; requires ND capability and readback.\n"
         "  --exposure-timeout MS  Shared exposure-wait budget, 50..60000 (default: 5000).\n"
         "  --exposure-tolerance N APEX96 tolerance, 0..32 (default: 4); ND readback is exact.\n\n"
         "Capture saves a JPEG to a new file; existing files are never overwritten.\n"
         "zoom/Zoom LEVEL accepts 0..255: 0 is minimum optical zoom, 255 maximum.\n"
         "Lens retraction timing is camera-dependent; set Canon Lens Retraction to 0 sec. for immediate retraction.\n"
         "--retract-lens confirms playback, not physical lens position; later capture/configuration enters record mode.\n"
         "Exactly one camera must match; --port cannot be combined with --bus/--device.\n",
         program, program, program, program, program, program, program);
}

/** @brief Emit the control operation's fixed-size binary receipt.
 * @param descriptor Caller-provided descriptor, or zero to omit the receipt.
 * @param command Echoed command ID.
 * @param observed Observed camera value when applicable.
 * @param evidence Kind of confirmation supplied with the value.
 * @return True for a complete write or when no receipt was requested.
 * @details Keep this receipt separate from human-readable stdout so the
 * parent can distinguish camera evidence from process success. */
static bool publish_control_reply(unsigned descriptor, uint8_t command, uint8_t observed, uint8_t evidence)
{
  if (!descriptor) return true;
  const uint8_t reply[] = {0xc1, command, observed, evidence};
  ssize_t written;
  do { written = write((int)descriptor, reply, sizeof(reply)); } while (written < 0 && errno == EINTR);
  return written == (ssize_t)sizeof(reply);
}

/**
 * @brief Select one camera, execute one operation, and release owned resources.
 * @param argc Argument count.
 * @param argv Program name and CLI arguments.
 * @return 0 for success, 1 for dependency/device/operation failure, 2 for invalid CLI.
 * @details Capture publishes a complete temporary file with link(), which fails
 * if the destination exists, including a file created after the initial check.
 * Zoom and info bypass local-file creation. Cleanup closes only a session opened
 * here and removes only a temporary file successfully created by this invocation.
 */
int main(int argc, char **argv)
{
  if (argc == 4 && !strcmp(argv[1], "--internal-unmount")) {
    unsigned helper_bus, helper_address;
    if (!parse_number(argv[2], 255, &helper_bus) || !parse_number(argv[3], 255, &helper_address)) return 2;
    return unmount_desktop_camera(helper_bus, helper_address);
  }
  unsigned bus = 0, address = 0, timeout_seconds = 60, zoom_level = 0;
  unsigned usb_timeout_ms = 1000, max_download_mib = 64;
  unsigned control_result_fd = 0;
  struct usb_port_selection port_selection = {0};
  struct capture_settings settings = {.focus = {.timeout_ms = 5000, .tolerance_mm = 100},
    .exposure = {.timeout_ms = 5000, .tolerance96 = 4}};
  const struct {
    const char *name;
    unsigned *value;
    unsigned minimum, maximum;
  } numeric_options[] = {
    {"--bus", &bus, 1, 255}, {"--device", &address, 1, 255},
    {"--timeout", &timeout_seconds, 1, 3600},
    {"--usb-timeout", &usb_timeout_ms, 50, 10000},
    {"--max-download-mib", &max_download_mib, 1, 256},
    {"--focus-timeout", &settings.focus.timeout_ms, 50, 60000},
    {"--focus-tolerance", &settings.focus.tolerance_mm, 0, 100000},
    {"--exposure-timeout", &settings.exposure.timeout_ms, 50, 60000},
    {"--exposure-tolerance", &settings.exposure.tolerance96, 0, 32}
  };
  const struct {
    const char *name;
    enum exposure_parameter parameter;
    int minimum, maximum;
  } apex_options[] = {
    {"--tv96", EXPOSURE_TV, -576, 1920}, {"--sv96", EXPOSURE_SV, 1, 1536},
    {"--av96", EXPOSURE_AV, 1, 1152}
  };
  bool numeric_seen[sizeof(numeric_options) / sizeof(numeric_options[0])] = {0};
  const char *output = NULL;
  const char *remote_download = NULL;
  bool list_only = false, zoom_requested = false, info_requested = false;
  bool configure_requested = false, retract_requested = false;
  bool runtime_requested = false, power_requested = false;
  bool direct_requested = false, romlog_requested = false;
  for (int index = 1; index < argc; ++index) {
    bool setting_option = false;
    for (size_t setting = 0; setting < CAMERA_SETTING_COUNT; ++setting) {
      if (strcmp(argv[index], camera_settings[setting].option)) continue;
      if (settings.present[setting] || ++index >= argc ||
          !parse_range(argv[index], 0, camera_settings[setting].maximum, &settings.values[setting])) {
        fprintf(stderr, "Invalid or duplicate value for %s\n", camera_settings[setting].option);
        return 2;
      }
      settings.present[setting] = true;
      setting_option = true;
      break;
    }
    if (setting_option) continue;
    for (size_t option = 0; option < sizeof(apex_options) / sizeof(apex_options[0]); ++option) {
      if (strcmp(argv[index], apex_options[option].name)) continue;
      enum exposure_parameter parameter = apex_options[option].parameter;
      if (settings.exposure.present[parameter] || ++index >= argc ||
          !parse_signed(argv[index], apex_options[option].minimum, apex_options[option].maximum, &settings.exposure.values[parameter])) {
        fprintf(stderr, "Invalid, duplicate or conflicting value for %s\n", apex_options[option].name);
        return 2;
      }
      settings.exposure.present[parameter] = true;
      setting_option = true;
      break;
    }
    if (setting_option) continue;
    for (size_t option = 0; option < sizeof(numeric_options) / sizeof(numeric_options[0]); ++option) {
      if (strcmp(argv[index], numeric_options[option].name)) continue;
        if (numeric_seen[option] || ++index >= argc || !(numeric_options[option].minimum == 1
          ? parse_number(argv[index], numeric_options[option].maximum, numeric_options[option].value)
          : parse_range(argv[index], numeric_options[option].minimum, numeric_options[option].maximum, numeric_options[option].value))) {
        fprintf(stderr, "Invalid or duplicate value for %s\n", numeric_options[option].name);
        return 2;
      }
      numeric_seen[option] = true;
      setting_option = true;
      break;
    }
    if (setting_option) continue;
    if (!strcmp(argv[index], "--help") || !strcmp(argv[index], "-h")) {
      print_help(argv[0]);
      return 0;
    } else if (!strcmp(argv[index], "--version") || !strcmp(argv[index], "-V")) {
      puts(CHDKBRIDGE_BUILD_VERSION);
      return 0;
    } else if (!strcmp(argv[index], "--list")) list_only = true;
    else if (!strcmp(argv[index], "--control-result-fd")) {
      if (control_result_fd || ++index >= argc || !parse_range(argv[index], 3, 31, &control_result_fd)
          || fcntl((int)control_result_fd, F_GETFD) < 0) return 2;
        if (fcntl((int)control_result_fd, F_SETFD, FD_CLOEXEC) < 0) return 2;
    }
    else if (!strcmp(argv[index], "--info")) info_requested = true;
    else if (!strcmp(argv[index], "--retract-lens")) {
      if (retract_requested) {
        fprintf(stderr, "Duplicate --retract-lens option\n");
        return 2;
      }
      retract_requested = true;
    }
    else if (!strcmp(argv[index], "--configure")) configure_requested = true;
    else if (!strcmp(argv[index], "--check-runtime")) runtime_requested = true;
    else if (!strcmp(argv[index], "--power-status")) power_requested = true;
    else if (!strcmp(argv[index], "--romlog")) {
      if (romlog_requested) return 2;
      romlog_requested = true;
    }
    else if (!strcmp(argv[index], "--keep-awake")) {
      if (settings.keep_awake_present || ++index >= argc ||
          !parse_range(argv[index], 0, 1, &settings.keep_awake)) {
        fprintf(stderr, "--keep-awake requires one value: 0 Never or 1 Always\n");
        return 2;
      }
      settings.keep_awake_present = true;
    }
    else if (!strcmp(argv[index], "--direct-download")) {
      if (direct_requested) {
        fprintf(stderr, "Duplicate --direct-download option\n");
        return 2;
      }
      direct_requested = true;
    }
    else if (!strcmp(argv[index], "--shutter")) {
      if (settings.exposure.present[EXPOSURE_TV] || ++index >= argc || !parse_shutter(argv[index], &settings.exposure)) {
        fprintf(stderr, "--shutter requires one decimal/fraction from 1 us to 64 seconds; do not combine with --tv96\n");
        return 2;
      }
      settings.exposure.present[EXPOSURE_TV] = true;
    } else if (!strcmp(argv[index], "--iso-real") || !strcmp(argv[index], "--iso-market")) {
      bool market = !strcmp(argv[index], "--iso-market");
      unsigned iso;
      if (settings.exposure.present[EXPOSURE_SV] || ++index >= argc || !parse_number(argv[index], 32767, &iso)) {
        fprintf(stderr, "Choose exactly one --iso-real, --iso-market or --sv96 override; ISO requires 1..32767\n");
        return 2;
      }
      settings.exposure.iso_kind = market ? EXPOSURE_ISO_MARKET : EXPOSURE_ISO_REAL;
      settings.exposure.values[EXPOSURE_SV] = (int)iso;
      settings.exposure.present[EXPOSURE_SV] = true;
    } else if (!strcmp(argv[index], "--aperture")) {
      if (settings.exposure.present[EXPOSURE_AV] || ++index >= argc ||
          !parse_scaled(argv[index], 1000, 1000, 64000, &settings.exposure.aperture_milli)) {
        fprintf(stderr, "--aperture requires one f-number from 1 to 64; do not combine with --av96\n");
        return 2;
      }
      settings.exposure.present[EXPOSURE_AV] = true;
    } else if (!strcmp(argv[index], "--nd")) {
      if (settings.exposure.present[EXPOSURE_ND] || ++index >= argc ||
          (strcmp(argv[index], "in") && strcmp(argv[index], "out"))) {
        fprintf(stderr, "--nd requires one explicit in or out state\n");
        return 2;
      }
      settings.exposure.values[EXPOSURE_ND] = !strcmp(argv[index], "in") ? 1 : 2;
      settings.exposure.present[EXPOSURE_ND] = true;
    }
    else if (!strcmp(argv[index], "--focus-mode")) {
      const char *modes[] = {"af", "aflock", "manual", "auto"};
      if (settings.focus.mode != FOCUS_UNCHANGED || ++index >= argc) {
        fprintf(stderr, "--focus-mode requires one af, aflock, manual or auto policy\n");
        return 2;
      }
      for (unsigned mode = 0; mode < sizeof(modes) / sizeof(modes[0]); ++mode)
        if (!strcmp(argv[index], modes[mode])) settings.focus.mode = (enum focus_mode)(mode + 1);
      if (settings.focus.mode == FOCUS_UNCHANGED) {
        fprintf(stderr, "Unknown focus mode; use af, aflock, manual or auto\n");
        return 2;
      }
    } else if (!strcmp(argv[index], "--focus-distance")) {
      unsigned distance = 0;
      if (settings.focus.distance_present || ++index >= argc ||
          (strcmp(argv[index], "inf") && !parse_number(argv[index], 1000000, &distance))) {
        fprintf(stderr, "--focus-distance requires one distance: 1..1000000 mm or inf\n");
        return 2;
      }
      settings.focus.distance_present = true;
      settings.focus.distance_mm = distance ? (int)distance : -1;
    }
    else if (!strcmp(argv[index], "--download")) {
      if (remote_download || ++index >= argc || !valid_camera_path(argv[index])) {
        fprintf(stderr, "--download requires a valid A/ camera file path\n");
        return 2;
      }
      remote_download = argv[index];
    }
    else if (!strcmp(argv[index], "--port")) {
      if (port_selection.depth || ++index >= argc || !parse_usb_port(argv[index], &port_selection)) {
        fprintf(stderr, "--port requires one BUS-PORT[.PORT...] path from --list\n");
        return 2;
      }
    } else if ((!strcmp(argv[index], "zoom") || !strcmp(argv[index], "Zoom")) &&
               !output && !zoom_requested) {
      if (++index >= argc || !parse_range(argv[index], 0, 255, &zoom_level)) {
        fprintf(stderr, "Zoom requires an integer from 0 to 255\n");
        return 2;
      }
      zoom_requested = true;
    } else if (argv[index][0] == '-' || output || zoom_requested) {
      fprintf(stderr, "Unexpected argument: %s\n", argv[index]);
      return 2;
    } else output = argv[index];
  }
  unsigned action_count = (unsigned)list_only + (unsigned)zoom_requested +
                          (unsigned)info_requested + (unsigned)configure_requested +
                          (unsigned)runtime_requested + (unsigned)retract_requested +
                          (unsigned)power_requested + (unsigned)(output != NULL);
  if ((settings.focus.mode == FOCUS_UNCHANGED &&
       (settings.focus.distance_present || numeric_seen[5] || numeric_seen[6])) ||
      (settings.focus.mode == FOCUS_AF && settings.focus.distance_present) ||
      ((settings.focus.mode == FOCUS_MANUAL || settings.focus.mode == FOCUS_AUTO) && !settings.focus.distance_present) ||
      (numeric_seen[6] && !settings.focus.distance_present) ||
      (settings.focus.mode != FOCUS_UNCHANGED && settings.focus.timeout_ms >= timeout_seconds * 1000)) {
    fprintf(stderr, "Focus requires an explicit compatible mode/distance and a focus timeout shorter than --timeout\n");
    return 2;
  }
  if ((!exposure_requested(&settings.exposure) && (numeric_seen[7] || numeric_seen[8])) ||
      (exposure_requested(&settings.exposure) && settings.exposure.timeout_ms >= timeout_seconds * 1000) ||
      (settings.exposure.present[EXPOSURE_SV] && settings.present[0] && settings.values[0] != 0)) {
    fprintf(stderr, "Exposure limits require overrides and a shorter wait than --timeout; real ISO overrides conflict with nonzero --iso-mode\n");
    return 2;
  }
  uint64_t requested_ms = settings.focus.mode == FOCUS_UNCHANGED ? 0 : settings.focus.timeout_ms;
  if (exposure_requested(&settings.exposure)) requested_ms += settings.exposure.timeout_ms + 1000U;
  if (settings.exposure.shutter_numerator)
    requested_ms += ((uint64_t)settings.exposure.shutter_numerator * 1000 +
                     settings.exposure.shutter_denominator - 1) / settings.exposure.shutter_denominator;
  else if (settings.exposure.present[EXPOSURE_TV])
    requested_ms += (uint64_t)ceil(exp2(-(double)settings.exposure.values[EXPOSURE_TV] / 96.0) * 1000.0);
  if (requested_ms >= (uint64_t)timeout_seconds * 1000) {
    fprintf(stderr, "--timeout must exceed combined focus/exposure waits, exposure cleanup and the requested shutter duration; leave time for record mode and download\n");
    return 2;
  }
    if ((control_result_fd && !zoom_requested && !retract_requested) ||
      (bus == 0) != (address == 0) || (port_selection.depth && bus) || action_count != 1 || (remote_download && !output) ||
      (romlog_requested && (!output || remote_download || direct_requested || settings_requested(&settings))) ||
      (direct_requested && (!output || remote_download || list_only || info_requested || zoom_requested || configure_requested || runtime_requested || power_requested)) ||
      (configure_requested && !settings_requested(&settings)) ||
      (settings_requested(&settings) && (list_only || info_requested || zoom_requested || remote_download || runtime_requested || retract_requested || power_requested))) {
    fprintf(stderr, "Select one action; specify both --bus and --device OR --port. Use --help for usage\n");
    return 2;
  }
  if (runtime_requested) {
    if (usbfs_available()) return 0;
    fprintf(stderr, "chdkbridge: Linux USB device access unavailable (%s and %s)\n",
            USBFS_SYSFS_ROOT, USBFS_DEVICE_ROOT);
    return 1;
  }
  struct usbfs_devices devices = {0};
  struct usbfs_device selected = {0};
  struct usbfs_handle handle = {.fd = -1};
  bool opened = false;
  FILE *file = NULL;
  int result = 1, interface_number = -1, alternate = 0;
  bool claimed = false, session = false, temporary_created = false;
  char temporary[PATH_MAX] = {0};
  struct transport transport = {0};
  uint64_t started_ms = monotonic_ms();
  if (!started_ms) { fprintf(stderr, "Cannot read monotonic clock\n"); return 1; }
  transport.deadline_ms = started_ms + (uint64_t)timeout_seconds * 1000;
  transport.usb_timeout_ms = usb_timeout_ms;
  transport.max_download = romlog_requested ? 65536U : (size_t)max_download_mib * 1024U * 1024U;
  struct sigaction cancellation = {.sa_handler = interrupt_handler};
  sigemptyset(&cancellation.sa_mask);
  sigaction(SIGINT, &cancellation, NULL);
  sigaction(SIGTERM, &cancellation, NULL);
  signal(SIGPIPE, SIG_IGN);
  int usb_result = usb.devices_open(&devices);
  if (usb_result != 0) {
    fprintf(stderr, "chdkbridge: USB device enumeration failed: %s\n", usbfs_error_name(usb_result));
    goto cleanup;
  }
  unsigned matches = 0;
  struct usbfs_device candidate;
  while ((usb_result = usb.devices_next(&devices, &candidate)) > 0) {
    if (!remaining_ms(&transport)) { fprintf(stderr, "USB discovery cancelled or operation deadline reached\n"); goto cleanup; }
    int candidate_interface, candidate_alternate;
    uint8_t input = 0, output_endpoint = 0;
    unsigned packet_size = 0;
    if (candidate.vendor != 0x04a9 ||
        (bus && (bus != candidate.bus || address != candidate.address)) ||
        !usb_port_matches(&port_selection, candidate.bus, candidate.ports, candidate.depth) ||
        !find_interface(&candidate, &candidate_interface, &candidate_alternate,
                         &input, &output_endpoint, &packet_size)) continue;
    if (list_only) {
      printf("bus=%u device=%u Canon %04x:%04x", candidate.bus, candidate.address, candidate.vendor, candidate.product);
      if (candidate.depth > 0) {
        printf(" port=%u-", candidate.bus);
        for (int port = 0; port < candidate.depth; ++port) printf("%s%u", port ? "." : "", candidate.ports[port]);
      }
      putchar('\n');
    }
    selected = candidate;
    interface_number = candidate_interface;
    alternate = candidate_alternate;
    transport.input_endpoint = input;
    transport.output_endpoint = output_endpoint;
    transport.output_packet_size = packet_size;
    ++matches;
  }
  if (usb_result < 0) {
    fprintf(stderr, "chdkbridge: USB device enumeration failed: %s\n", usbfs_error_name(usb_result));
    goto cleanup;
  }
  if (list_only) {
    if (!matches) fprintf(stderr, "No matching Canon PTP camera on USB; check power, CHDK firmware, USB mode, data cable and selected port\n");
    result = 0; goto cleanup;
  }
  if (matches != 1) {
    if (!matches) fprintf(stderr, "No matching Canon PTP camera on USB; check power, CHDK firmware, USB mode, data cable and selected port; run --list\n");
    else fprintf(stderr, "Found %u matching Canon PTP cameras; use --list and select --port or --bus/--device\n", matches);
    goto cleanup;
  }
  /* Check output access before touching the camera. mkstemp reserves a private
   * same-directory file so publication cannot expose a partial download. */
  if (output) {
    struct stat existing;
    if (lstat(output, &existing) == 0 || errno != ENOENT) {
      fprintf(stderr, "Output already exists or cannot be accessed: %s\n", output);
      goto cleanup;
    }
    int length = snprintf(temporary, sizeof(temporary), "%s.part.XXXXXX", output);
    if (length < 0 || (size_t)length >= sizeof(temporary)) goto cleanup;
    int descriptor = mkstemp(temporary);
    if (descriptor < 0) { perror("Creating output"); temporary[0] = '\0'; goto cleanup; }
    temporary_created = true;
    if (fcntl(descriptor, F_SETFD, FD_CLOEXEC) < 0) { close(descriptor); goto cleanup; }
    file = fdopen(descriptor, "w+b");
    if (!file) { close(descriptor); goto cleanup; }
  }
  usb_result = usb.open(&selected, &handle);
  if (usb_result != 0) {
    fprintf(stderr, "Cannot open camera: %s; %s\n", usbfs_error_name(usb_result), usb_failure_hint(usb_result));
    goto cleanup;
  }
  opened = true;
  handle.cancel = &interrupted;
  transport.handle = &handle;
  usb_result = claim_camera_interface(&transport, interface_number,
    selected.bus, selected.address, release_desktop_camera);
  if (usb_result != 0) {
    fprintf(stderr, "Cannot claim camera: %s; %s\n", usbfs_error_name(usb_result), usb_failure_hint(usb_result));
    goto cleanup;
  }
  claimed = true;
  if (alternate && (usb_result = usb.set_alternate(&handle, interface_number, alternate)) != 0) {
    fprintf(stderr, "Cannot select camera interface setting: %s; %s\n", usbfs_error_name(usb_result), usb_failure_hint(usb_result));
    goto cleanup;
  }
  struct response response;
  /* OpenSession uses transaction zero; subsequent commands increment the ID.
   * The claimed interface excludes other host clients, so a session or data
   * left by an interrupted earlier run is ours; recover it once, as chdkptp does. */
  uint32_t session_id = 1;
  if (!transaction(&transport, PTP_OPEN_SESSION, &session_id, 1,
                    NULL, 0, NULL, 0, NULL, &response) &&
      (!recover_ptp(&transport, interface_number) ||
       !transaction(&transport, PTP_OPEN_SESSION, &session_id, 1,
                    NULL, 0, NULL, 0, NULL, &response))) goto cleanup;
  session = true;
  if (retract_requested) {
    if (retract_lens(&transport) && publish_control_reply(control_result_fd, 2, 0, 2)) result = 0;
    else fprintf(stderr, "Lens retract request failed or timed out; camera state may have changed, no automatic retry\n");
    goto cleanup;
  }
  if (power_requested) {
    if (power_status(&transport)) result = 0;
    else fprintf(stderr, "Power status query failed or prevention is not Always\n");
    goto cleanup;
  }
  if (info_requested) {
    if (camera_info(&transport)) result = 0;
    else fprintf(stderr, "Camera information query failed\n");
    goto cleanup;
  }
  if (zoom_requested) {
    uint8_t observed = 0;
    if (set_zoom(&transport, zoom_level, &observed)
        && publish_control_reply(control_result_fd, 1, observed, 1)) result = 0;
    else fprintf(stderr, "Zoom failed or timed out; camera zoom may have changed\n");
    goto cleanup;
  }
  if (configure_requested) {
    if (configure_camera(&transport, &settings)) result = 0;
    else fprintf(stderr, "Configuration failed; camera settings may have partially changed\n");
    goto cleanup;
  }
    if (!(romlog_requested ? download_romlog(&transport, file) : remote_download ? download_file(&transport, remote_download, file) :
      direct_requested ? capture_direct(&transport, file, &settings) : capture(&transport, file, &settings))) {
    fprintf(stderr, "Capture/download failed or timed out; no automatic shot retry\n");
    goto cleanup;
  }
  /* Flush storage before checking only the JPEG boundary markers. This does
   * not decode the image or establish full filesystem crash durability. */
  if (fflush(file) != 0 || fsync(fileno(file)) != 0 || fseek(file, 0, SEEK_SET) != 0)
    goto cleanup;
  uint8_t signature[2];
  if (!romlog_requested && (fread(signature, 1, 2, file) != 2 || signature[0] != 0xff || signature[1] != 0xd8 ||
      fseek(file, -2, SEEK_END) != 0 || fread(signature, 1, 2, file) != 2 ||
      signature[0] != 0xff || signature[1] != 0xd9)) {
    fprintf(stderr, "Downloaded file is not a complete JPEG\n");
    goto cleanup;
  }
  if (romlog_requested) {
    struct stat status;
    if (fstat(fileno(file), &status) != 0 || status.st_size < 1 || status.st_size > 65536) goto cleanup;
  }
  if (fclose(file) != 0) { file = NULL; goto cleanup; }
  file = NULL;
  /* link() atomically creates a new name without replacing an existing one;
   * cleanup removes the temporary name, leaving the published inode intact. */
  if (link(temporary, output) != 0) { perror("Publishing output without overwrite"); goto cleanup; }
  printf("Saved %s\n", output);
  result = 0;
cleanup:
  if (session) close_session(&transport);
  if (file) fclose(file);
  if (temporary_created) unlink(temporary);
  if (claimed) usb.release_interface(&handle, interface_number);
  if (opened) usb.close(&handle);
  usb.devices_close(&devices);
  return result;
}