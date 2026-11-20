/*
 * Copyright (C) 2026 The Paparazzi Team
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/**
 * @file modules/digital_cam/uart_cam_ctrl.c
 * @brief USHER camera control and payload exchange over a serial link.
 *
 * @details This module translates Paparazzi digital-camera commands into USHER
 * frames, attaches the aircraft pose to capture requests, parses camera status
 * and payload replies, and optionally streams pose samples. On flight hardware,
 * `CAMERA_LINK` is the ordinary UART selected by `CAMERA_PORT`.
 *
 * Linux simulation targets use a private, nonblocking ::link_device adapter.
 * The adapter prefers the local USHER pseudo-terminal (normally
 * `/tmp/usher-sim`) and falls back to a desk FTDI device (normally
 * `/dev/ttyUSB0`). Keeping this policy here is intentional: endpoint priority
 * is camera-module behavior, not a property of Paparazzi's generic UART
 * architecture. It also avoids changing a generic UART peripheral while its
 * Linux receive thread may be using it.
 *
 * @par Endpoint boundary invariant
 * Each endpoint is a different USHER byte stream and may represent a different
 * peer and clock domain. Switching therefore discards queued RX/TX bytes,
 * partial parser state, and pose-clock alignment. Carrying any of that state
 * across the boundary could splice frames from two peers or timestamp samples
 * against the wrong clock. Commands queued at the instant of a switch are
 * intentionally not replayed; callers may issue a later command after
 * reconnection.
 *
 * @par Scheduling model
 * The simulation adapter has no worker thread. The module event and periodic
 * callbacks perform zero-timeout I/O service, while endpoint reconciliation is
 * rate-limited. This preserves Paparazzi's single-threaded module semantics and
 * keeps parser and ring-buffer state free from concurrent access.
 */

#include "uart_cam_ctrl.h"
#include "generated/airframe.h"

// Include Standard Camera Control Interface
#include "modules/digital_cam/dc.h"

//Needed if we have thumbnail relay
#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
#endif

#include BOARD_CONFIG

// Communication
#include "usher_protocol.h"

#include "state.h"

#ifdef DIGITAL_CAM_UART_SIM_BACKEND
#include "arch/linux/serial_port.h"
#include "mcu_periph/sys_time.h"

#include <errno.h>
#include <poll.h>
#include <stdio.h>
#include <unistd.h>

/** Preferred USHER endpoint created by the local simulation launcher. */
#ifndef DIGITAL_CAM_UART_LOCAL_DEVICE
#define DIGITAL_CAM_UART_LOCAL_DEVICE /tmp/usher-sim
#endif

/** Physical desk UART used while local USHER is unavailable. */
#ifndef DIGITAL_CAM_UART_FALLBACK_DEVICE
#define DIGITAL_CAM_UART_FALLBACK_DEVICE /dev/ttyUSB0
#endif

/** Interval between endpoint-priority checks, in microseconds. */
#ifndef DIGITAL_CAM_UART_RECONNECT_INTERVAL_USEC
#define DIGITAL_CAM_UART_RECONNECT_INTERVAL_USEC 200000
#endif
/** RX ring capacity; one slot remains unused to distinguish full from empty. */
#ifndef DIGITAL_CAM_UART_SIM_RX_BUFFER_SIZE
#define DIGITAL_CAM_UART_SIM_RX_BUFFER_SIZE 1024
#endif
/** TX ring capacity; complete USHER frames must fit before they are enqueued. */
#ifndef DIGITAL_CAM_UART_SIM_TX_BUFFER_SIZE
#define DIGITAL_CAM_UART_SIM_TX_BUFFER_SIZE 1024
#endif

/** Maximum received bytes processed by one service pass. */
#ifndef DIGITAL_CAM_UART_SIM_RX_BATCH_SIZE
#define DIGITAL_CAM_UART_SIM_RX_BATCH_SIZE 256
#endif

static void digital_cam_uart_reset_protocol_state(void);

/** Serial endpoint currently owned by the simulation adapter. */
enum digital_cam_uart_endpoint {
  DIGITAL_CAM_UART_ENDPOINT_NONE,     /**< No usable endpoint is open. */
  DIGITAL_CAM_UART_ENDPOINT_LOCAL,    /**< Preferred local USHER pseudo-terminal. */
  DIGITAL_CAM_UART_ENDPOINT_FALLBACK  /**< Physical desk serial adapter. */
};

/** Private state implementing the simulation target's ::link_device contract. */
struct digital_cam_uart_sim_periph {
  struct link_device device;                /**< Transport-neutral USHER interface. */
  struct SerialPort port;                   /**< Currently open Linux serial port. */
  enum digital_cam_uart_endpoint endpoint;  /**< Identity of `port`, or NONE. */
  uint8_t rx_buf[DIGITAL_CAM_UART_SIM_RX_BUFFER_SIZE]; /**< Bytes ready for the parser. */
  uint16_t rx_insert_idx;                   /**< Producer position in `rx_buf`. */
  uint16_t rx_extract_idx;                  /**< Consumer position in `rx_buf`. */
  uint8_t tx_buf[DIGITAL_CAM_UART_SIM_TX_BUFFER_SIZE]; /**< Bytes awaiting OS writes. */
  uint16_t tx_insert_idx;                   /**< Producer position in `tx_buf`. */
  uint16_t tx_extract_idx;                  /**< Consumer position in `tx_buf`. */
  uint32_t last_reconnect_us;               /**< Wrap-safe reconciliation timestamp. */
};

static struct digital_cam_uart_sim_periph digital_cam_uart_sim_link;

static int  digital_cam_uart_sim_check_free_space(void *periph, long *fd, uint16_t len);
static void digital_cam_uart_sim_put_byte(void *periph, long fd, uint8_t data);
static void digital_cam_uart_sim_put_buffer(void *periph, long fd, const uint8_t *data, uint16_t len);
static void digital_cam_uart_sim_send_message(void *periph, long fd);
static int  digital_cam_uart_sim_char_available(void *periph);
static uint8_t digital_cam_uart_sim_get_byte(void *periph);
static void digital_cam_uart_sim_set_baudrate(void *periph, uint32_t baudrate);
static void digital_cam_uart_sim_init(void);
static void digital_cam_uart_sim_periodic(void);
#endif

#ifndef DIGITAL_CAM_UART_POSE_STREAM
#define DIGITAL_CAM_UART_POSE_STREAM 0
#endif

#include "mcu_periph/sys_time.h"
#include "mcu_periph/uart.h"
#if USE_GPS
#include "modules/gps/gps.h"
#endif
static uint32_t pose_sequence;
static union usher_clock_request_union pose_clock_token;
static uint32_t last_clock_reply_us;
static bool clock_reply_sent;
static uint32_t last_pose_sample_us;
static struct usher_control_client control_client;
static uint32_t last_control_negotiate_us;
static void control_periodic(void);
_Static_assert(sizeof(control_client) + sizeof(last_control_negotiate_us) <= 256, "FC controls exceed 256 bytes");
#define SHOT_REQUEST_CAPACITY 32
static struct {
  uint32_t request_id, sent_us;
  int32_t number;
  uint8_t pending_mask, point_mask;
} shot_requests[SHOT_REQUEST_CAPACITY];
static uint8_t next_shot_request_slot;
static uint32_t point_stream_number[8];
static bool point_from_stream[8];
#define CAMERA_POINT_STREAM_MAX_AGE_US 15000000U
uint32_t digital_cam_uart_camera_result_status[8];
int32_t digital_cam_uart_camera_result_number[8];
struct digital_cam_uart_point digital_cam_uart_camera_point[8];
static uint32_t next_camera_session_id;
static struct {
  uint32_t id;
  bool awaiting_point;
} camera_sessions[8];
static void send_pose_sample(void);
static void reply_clock_request(void);
/** @brief Borrowed TX reservation valid only during this module's dc_periodic() call.
 * @details Shared DC code stays unchanged. A non-NULL pointer carries the
 * already-reserved ordinary shot frame for an autoshoot call.
 * dc_send_command() consumes it before reporting callbacks; a no-shot tick
 * releases it locally. This relies on the existing serialized main-loop contract,
 * not on thread-local or ISR-safe dispatch. Never retain the pointer after the call.
 */
static long *reserved_autoshoot_fd;
static uint8_t reserved_autoshoot_mask;

static bool autoshoot_can_send_shot(void)
{
  return dc_autoshoot == DC_AUTOSHOOT_PERIODIC || dc_autoshoot == DC_AUTOSHOOT_DISTANCE
         || dc_autoshoot == DC_AUTOSHOOT_SURVEY
         || dc_autoshoot == DC_AUTOSHOOT_CIRCLE;
}

/**
 * @brief Reset state that belongs to one USHER peer connection.
 *
 * @details Parser progress and clock tokens cannot survive a simulation
 * endpoint switch: the replacement endpoint may be a different process with a
 * different byte stream and monotonic clock. Hardware also calls this during
 * initialization to start from a known protocol boundary.
 */
static void digital_cam_uart_reset_protocol_state(void)
{
  usher_protocol = (struct usher_transport){0};
  usher_control_client_reset(&control_client, control_client.nonce
                             ? control_client.nonce + 1 : get_sys_time_usec() | 1U);
  last_control_negotiate_us = 0;
  for (size_t index = 0; index < SHOT_REQUEST_CAPACITY; ++index) {
    shot_requests[index].request_id = 0;
    shot_requests[index].point_mask = 0;
  }
  for (size_t index = 0; index < 8; ++index) digital_cam_uart_camera_result_status[index] = 0;
  for (size_t index = 0; index < 8; ++index) digital_cam_uart_camera_point[index] = (struct digital_cam_uart_point){0};
  for (size_t index = 0; index < 8; ++index) {
    camera_sessions[index].id = 0;
    camera_sessions[index].awaiting_point = false;
    point_stream_number[index] = 0;
    point_from_stream[index] = false;
  }
  pose_clock_token = (union usher_clock_request_union){0};
  last_clock_reply_us = 0;
  last_pose_sample_us = 0;
  clock_reply_sent = false;
}

#if FIXEDWING_FIRMWARE
#include "modules/nav/common_nav.h"
#endif


#define CameraLinkDev (&((CAMERA_LINK).device))
/**
 * @brief Each frame producer owns a local camera_fd, initialized to zero for stream backends.
 * OS sets it when reserving the UART transmit mutex; preserve it
 * for every byte and finish the frame with CameraLinkSendMessage(). */
#define CameraLinkTransmit(c) CameraLinkDev->put_byte(CameraLinkDev->periph, camera_fd, c)
#define CameraLinkChAvailable() CameraLinkDev->char_available(CameraLinkDev->periph)
#define CameraLinkGetch() CameraLinkDev->get_byte(CameraLinkDev->periph)
#define CameraLinkCheckFreeSpace(len) CameraLinkDev->check_free_space(CameraLinkDev->periph, &camera_fd, len)
#define CameraLinkSendMessage() CameraLinkDev->send_message(CameraLinkDev->periph, camera_fd)

#ifdef DIGITAL_CAM_UART_SIM_BACKEND
/**
 * @brief Return the configured path for an endpoint.
 * @param endpoint LOCAL for the preferred pseudo-terminal, otherwise FALLBACK.
 * @return Static, stringified build-time device path.
 */
static const char *digital_cam_uart_sim_endpoint_path(enum digital_cam_uart_endpoint endpoint)
{
  return endpoint == DIGITAL_CAM_UART_ENDPOINT_LOCAL
         ? STRINGIFY(DIGITAL_CAM_UART_LOCAL_DEVICE) : STRINGIFY(DIGITAL_CAM_UART_FALLBACK_DEVICE);
}

/**
 * @brief Discard all buffered transport and peer-specific protocol state.
 * @param periph Simulation transport instance.
 *
 * @details RX and TX are cleared together because they describe one ordered
 * conversation. Retaining either side while changing peers could complete a
 * partial frame with bytes from the wrong endpoint.
 */
static void digital_cam_uart_sim_clear_queues(struct digital_cam_uart_sim_periph *periph)
{
  periph->rx_insert_idx = 0;
  periph->rx_extract_idx = 0;
  periph->tx_insert_idx = 0;
  periph->tx_extract_idx = 0;
  digital_cam_uart_reset_protocol_state();
}

/**
 * @brief Close the active endpoint and establish a clean disconnected state.
 * @param periph Simulation transport instance.
 */
static void digital_cam_uart_sim_close(struct digital_cam_uart_sim_periph *periph)
{
  if (periph->port.fd >= 0) {
    serial_port_close(&periph->port);
    periph->port.fd = -1;
  }
  periph->endpoint = DIGITAL_CAM_UART_ENDPOINT_NONE;
  digital_cam_uart_sim_clear_queues(periph);
}

/**
 * @brief Open an endpoint as a nonblocking raw serial candidate.
 * @param[out] candidate Receives the opened descriptor on success.
 * @param endpoint Endpoint whose configured path is opened.
 * @return `true` when the candidate is ready for use, otherwise `false`.
 */
static bool digital_cam_uart_sim_open_candidate(struct SerialPort *candidate,
                                                 enum digital_cam_uart_endpoint endpoint)
{
  candidate->fd = -1;
  if (serial_port_open_raw(candidate, digital_cam_uart_sim_endpoint_path(endpoint), B115200) == 0) {
    return true;
  }
  candidate->fd = -1;
  return false;
}

/**
 * @brief Atomically replace the active endpoint with an open candidate.
 * @param periph Simulation transport instance.
 * @param endpoint Desired endpoint.
 * @return `true` if the new endpoint was opened and selected.
 *
 * @details The candidate is opened before the old endpoint is closed. A failed
 * preference probe must not interrupt a working fallback connection. Once the
 * candidate is known to work, closing the old endpoint intentionally resets all
 * connection-bound queues and protocol state before the candidate is installed.
 */
static bool digital_cam_uart_sim_switch(struct digital_cam_uart_sim_periph *periph,
                                        enum digital_cam_uart_endpoint endpoint)
{
  struct SerialPort candidate;
  if (!digital_cam_uart_sim_open_candidate(&candidate, endpoint)) {
    return false;
  }

  digital_cam_uart_sim_close(periph);
  periph->port = candidate;
  periph->endpoint = endpoint;
  fprintf(stderr, "Digital camera UART: connected to %s\n", digital_cam_uart_sim_endpoint_path(endpoint));
  return true;
}

/**
 * @brief Record a link failure and request immediate endpoint reconciliation.
 * @param periph Simulation transport instance.
 */
static void digital_cam_uart_sim_disconnect(struct digital_cam_uart_sim_periph *periph)
{
  if (periph->endpoint != DIGITAL_CAM_UART_ENDPOINT_NONE) {
    fprintf(stderr, "Digital camera UART: disconnected from %s\n",
            digital_cam_uart_sim_endpoint_path(periph->endpoint));
  }
  digital_cam_uart_sim_close(periph);
  periph->last_reconnect_us = 0;
}

/**
 * @brief Count occupied bytes in a wrapping single-producer ring.
 * @param insert_idx Producer index.
 * @param extract_idx Consumer index.
 * @param buffer_size Ring capacity.
 * @return Number of occupied slots.
 */
static uint16_t digital_cam_uart_sim_ring_count(uint16_t insert_idx, uint16_t extract_idx,
                                                uint16_t buffer_size)
{
  return insert_idx >= extract_idx ? insert_idx - extract_idx : buffer_size - extract_idx + insert_idx;
}

/**
 * @brief Move a bounded batch of operating-system bytes into the RX ring.
 * @param periph Simulation transport instance with an open endpoint.
 *
 * @details Bounding each pass prevents a busy camera stream from monopolizing
 * the flight loop. Overflow drops only the newly received byte and increments
 * the standard ::link_device overrun counter.
 */
static void digital_cam_uart_sim_receive(struct digital_cam_uart_sim_periph *periph)
{
  for (uint16_t count = 0; count < DIGITAL_CAM_UART_SIM_RX_BATCH_SIZE; count++) {
    uint8_t byte;
    ssize_t received = read(periph->port.fd, &byte, 1);
    if (received == 1) {
      uint16_t next_idx = (periph->rx_insert_idx + 1) % DIGITAL_CAM_UART_SIM_RX_BUFFER_SIZE;
      if (next_idx != periph->rx_extract_idx) {
        periph->rx_buf[periph->rx_insert_idx] = byte;
        periph->rx_insert_idx = next_idx;
      } else {
        periph->device.nb_ovrn++;
      }
      continue;
    }
    if (received < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
      digital_cam_uart_sim_disconnect(periph);
    }
    return;
  }
}

/**
 * @brief Flush queued TX bytes without blocking the module scheduler.
 * @param periph Simulation transport instance with an open endpoint.
 *
 * @details Partial writes advance the consumer index by exactly the accepted
 * count. `EAGAIN`, `EWOULDBLOCK`, and `EINTR` retain the unsent suffix for the
 * next service pass; permanent errors disconnect the endpoint.
 */
static void digital_cam_uart_sim_transmit(struct digital_cam_uart_sim_periph *periph)
{
  while (periph->tx_extract_idx != periph->tx_insert_idx) {
    uint16_t available = periph->tx_insert_idx > periph->tx_extract_idx
                         ? periph->tx_insert_idx - periph->tx_extract_idx
                         : DIGITAL_CAM_UART_SIM_TX_BUFFER_SIZE - periph->tx_extract_idx;
    ssize_t sent = write(periph->port.fd, &periph->tx_buf[periph->tx_extract_idx], available);
    if (sent > 0) {
      periph->tx_extract_idx = (periph->tx_extract_idx + sent) % DIGITAL_CAM_UART_SIM_TX_BUFFER_SIZE;
      periph->device.nb_bytes += sent;
      continue;
    }
    if (sent < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
      digital_cam_uart_sim_disconnect(periph);
    }
    return;
  }
}

/**
 * @brief Perform one zero-timeout RX/TX service pass.
 * @param periph Simulation transport instance.
 *
 * @details I/O happens here rather than in `char_available`, so querying the
 * ::link_device never changes connection state or performs hidden system calls.
 * The event and periodic callbacks invoke this function explicitly, avoiding a
 * transport thread and the locking that shared rings and parser state would need.
 */
static void digital_cam_uart_sim_service(struct digital_cam_uart_sim_periph *periph)
{
  if (periph->endpoint == DIGITAL_CAM_UART_ENDPOINT_NONE) {
    return;
  }

  struct pollfd poll_fd = {
    .fd = periph->port.fd,
    .events = POLLIN | (periph->tx_extract_idx != periph->tx_insert_idx ? POLLOUT : 0),
    .revents = 0
  };
  int result = poll(&poll_fd, 1, 0);
  if (result < 0) {
    if (errno != EINTR) {
      digital_cam_uart_sim_disconnect(periph);
    }
    return;
  }
  if ((poll_fd.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
    digital_cam_uart_sim_disconnect(periph);
    return;
  }
  if ((poll_fd.revents & POLLIN) != 0) {
    digital_cam_uart_sim_receive(periph);
  }
  if (periph->endpoint != DIGITAL_CAM_UART_ENDPOINT_NONE && (poll_fd.revents & POLLOUT) != 0) {
    digital_cam_uart_sim_transmit(periph);
  }
}

/**
 * @brief Enforce local-USHER-first endpoint priority.
 * @param periph Simulation transport instance.
 *
 * @details A present and openable local pseudo-terminal always wins. If it is
 * absent or disappears, the physical fallback is attempted. Presence alone is
 * not enough to displace a working fallback: ::digital_cam_uart_sim_switch
 * first proves that the preferred endpoint can be opened.
 */
static void digital_cam_uart_sim_reconcile(struct digital_cam_uart_sim_periph *periph)
{
  bool local_available = access(STRINGIFY(DIGITAL_CAM_UART_LOCAL_DEVICE), F_OK) == 0;
  if (periph->endpoint == DIGITAL_CAM_UART_ENDPOINT_LOCAL && !local_available) {
    digital_cam_uart_sim_disconnect(periph);
  }
  if (local_available && periph->endpoint != DIGITAL_CAM_UART_ENDPOINT_LOCAL
      && digital_cam_uart_sim_switch(periph, DIGITAL_CAM_UART_ENDPOINT_LOCAL)) {
    return;
  }
  if (periph->endpoint == DIGITAL_CAM_UART_ENDPOINT_NONE) {
    digital_cam_uart_sim_switch(periph, DIGITAL_CAM_UART_ENDPOINT_FALLBACK);
  }
}

/**
 * @brief Check whether one complete caller-requested frame can be queued.
 * @param parent Simulation transport instance passed through ::link_device.
 * @param fd Unused multi-link compatibility argument.
 * @param len Required frame size in bytes.
 * @return Available byte count when at least `len` bytes fit, otherwise zero.
 *
 * @details Reporting zero for insufficient space lets USHER producers reject a
 * whole frame before writing its header. This is essential because a truncated
 * frame would desynchronize the receiver until its next valid start marker.
 */
static int digital_cam_uart_sim_check_free_space(void *parent, long *fd __attribute__((unused)), uint16_t len)
{
  struct digital_cam_uart_sim_periph *periph = parent;
  if (periph->endpoint == DIGITAL_CAM_UART_ENDPOINT_NONE) {
    return 0;
  }
  uint16_t used = digital_cam_uart_sim_ring_count(periph->tx_insert_idx, periph->tx_extract_idx,
                                                  DIGITAL_CAM_UART_SIM_TX_BUFFER_SIZE);
  uint16_t space = DIGITAL_CAM_UART_SIM_TX_BUFFER_SIZE - used - 1;
  return space >= len ? space : 0;
}

/**
 * @brief Append one byte to the simulation TX ring.
 * @param parent Simulation transport instance passed through ::link_device.
 * @param fd Unused multi-link compatibility argument.
 * @param data Byte to enqueue.
 *
 * @warning Frame producers must call `check_free_space` for the complete USHER
 * frame first. The overrun guard protects memory but cannot make a partial frame
 * valid if a producer violates that contract.
 */
static void digital_cam_uart_sim_put_byte(void *parent, long fd __attribute__((unused)), uint8_t data)
{
  struct digital_cam_uart_sim_periph *periph = parent;
  if (periph->endpoint == DIGITAL_CAM_UART_ENDPOINT_NONE) {
    return;
  }
  uint16_t next_idx = (periph->tx_insert_idx + 1) % DIGITAL_CAM_UART_SIM_TX_BUFFER_SIZE;
  if (next_idx == periph->tx_extract_idx) {
    periph->device.nb_ovrn++;
    return;
  }
  periph->tx_buf[periph->tx_insert_idx] = data;
  periph->tx_insert_idx = next_idx;
}

/**
 * @brief Append a byte array to the simulation TX ring.
 * @param parent Simulation transport instance passed through ::link_device.
 * @param fd Unused multi-link compatibility argument.
 * @param data Bytes to enqueue.
 * @param len Number of bytes in `data`.
 * @note Callers must reserve space for the complete frame before insertion.
 */
static void digital_cam_uart_sim_put_buffer(void *parent, long fd, const uint8_t *data, uint16_t len)
{
  for (uint16_t index = 0; index < len; index++) {
    digital_cam_uart_sim_put_byte(parent, fd, data[index]);
  }
}

/**
 * @brief Give the nonblocking transport an immediate chance to flush a frame.
 * @param parent Simulation transport instance passed through ::link_device.
 * @param fd Unused multi-link compatibility argument.
 */
static void digital_cam_uart_sim_send_message(void *parent, long fd __attribute__((unused)))
{
  digital_cam_uart_sim_service(parent);
}

/**
 * @brief Query bytes already buffered for USHER parsing.
 * @param parent Simulation transport instance passed through ::link_device.
 * @return Number of bytes currently in the RX ring.
 * @note This callback is deliberately side-effect free; I/O and connection
 * management remain visible in explicit service calls.
 */
static int digital_cam_uart_sim_char_available(void *parent)
{
  struct digital_cam_uart_sim_periph *periph = parent;
  return digital_cam_uart_sim_ring_count(periph->rx_insert_idx, periph->rx_extract_idx,
                                         DIGITAL_CAM_UART_SIM_RX_BUFFER_SIZE);
}

/**
 * @brief Consume one buffered RX byte.
 * @param parent Simulation transport instance passed through ::link_device.
 * @return Oldest buffered byte, or zero when called on an empty ring.
 */
static uint8_t digital_cam_uart_sim_get_byte(void *parent)
{
  struct digital_cam_uart_sim_periph *periph = parent;
  if (periph->rx_extract_idx == periph->rx_insert_idx) {
    return 0;
  }
  uint8_t byte = periph->rx_buf[periph->rx_extract_idx];
  periph->rx_extract_idx = (periph->rx_extract_idx + 1) % DIGITAL_CAM_UART_SIM_RX_BUFFER_SIZE;
  return byte;
}

/**
 * @brief Apply a baud rate to the active simulation serial endpoint.
 * @param parent Simulation transport instance passed through ::link_device.
 * @param baudrate Platform baud-rate constant.
 */
static void digital_cam_uart_sim_set_baudrate(void *parent, uint32_t baudrate)
{
  struct digital_cam_uart_sim_periph *periph = parent;
  if (periph->endpoint != DIGITAL_CAM_UART_ENDPOINT_NONE
      && serial_port_set_baudrate(&periph->port, baudrate) != 0) {
    digital_cam_uart_sim_disconnect(periph);
  }
}

/**
 * @brief Initialize the private ::link_device and select the best endpoint.
 *
 * @details Hardware builds keep using `CAMERA_PORT`; this adapter gives only
 * simulation builds dynamic selection without changing generic UART ownership.
 */
static void digital_cam_uart_sim_init(void)
{
  struct digital_cam_uart_sim_periph *periph = &digital_cam_uart_sim_link;
  periph->port.fd = -1;
  periph->endpoint = DIGITAL_CAM_UART_ENDPOINT_NONE;
  periph->last_reconnect_us = 0;
  digital_cam_uart_sim_clear_queues(periph);
  periph->device = (struct link_device) {
    .check_free_space = digital_cam_uart_sim_check_free_space,
    .put_byte = digital_cam_uart_sim_put_byte,
    .put_buffer = digital_cam_uart_sim_put_buffer,
    .send_message = digital_cam_uart_sim_send_message,
    .char_available = digital_cam_uart_sim_char_available,
    .get_byte = digital_cam_uart_sim_get_byte,
    .set_baudrate = digital_cam_uart_sim_set_baudrate,
    .periph = periph
  };
  digital_cam_uart_sim_reconcile(periph);
}

/**
 * @brief Service transport I/O and periodically reconsider endpoint priority.
 *
 * @details Service runs before and after reconciliation so a newly selected
 * endpoint participates immediately. Unsigned timestamp subtraction keeps the
 * interval test valid across the microsecond counter wrapping.
 */
static void digital_cam_uart_sim_periodic(void)
{
  struct digital_cam_uart_sim_periph *periph = &digital_cam_uart_sim_link;
  digital_cam_uart_sim_service(periph);
  uint32_t now = get_sys_time_usec();
  if (periph->last_reconnect_us == 0
      || (uint32_t)(now - periph->last_reconnect_us) >= DIGITAL_CAM_UART_RECONNECT_INTERVAL_USEC) {
    periph->last_reconnect_us = now;
    digital_cam_uart_sim_reconcile(periph);
  }
  digital_cam_uart_sim_service(periph);
}
#endif

union usher_status_union usher_status_msg;
int digital_cam_uart_status = 0;
#ifdef DIGITAL_CAM_UART_CAMERA_ID
#error "DIGITAL_CAM_UART_CAMERA_ID was removed; configure DIGITAL_CAM_UART_CAMERA_MASK"
#endif
#ifndef DIGITAL_CAM_UART_CAMERA_MASK
#define DIGITAL_CAM_UART_CAMERA_MASK USHER_CAMERA_MASK_ALL
#endif
#if DIGITAL_CAM_UART_CAMERA_MASK < 0 || DIGITAL_CAM_UART_CAMERA_MASK > USHER_CAMERA_MASK_ALL
#error "DIGITAL_CAM_UART_CAMERA_MASK must fit in eight bits"
#endif
uint8_t digital_cam_uart_camera_mask = DIGITAL_CAM_UART_CAMERA_MASK;
#ifndef DIGITAL_CAM_UART_SHOT_REPORT_MASK
#define DIGITAL_CAM_UART_SHOT_REPORT_MASK USHER_CAMERA_MASK_ALL
#endif
#if DIGITAL_CAM_UART_SHOT_REPORT_MASK < 0 || DIGITAL_CAM_UART_SHOT_REPORT_MASK > USHER_CAMERA_MASK_ALL
#error "DIGITAL_CAM_UART_SHOT_REPORT_MASK must fit in eight bits"
#endif
uint8_t digital_cam_uart_shot_report_mask = DIGITAL_CAM_UART_SHOT_REPORT_MASK;
#define DIGITAL_CAM_UART_RX_HANDLERS 4
static struct {
  uint8_t message_id;
  digital_cam_uart_rx_handler_t handler;
} rx_handlers[DIGITAL_CAM_UART_RX_HANDLERS];

int digital_cam_uart_thumbnails = 0;
#define THUMB_MSG_SIZE  USHER_PAYLOAD_MSG_SIZE
#define THUMB_COUNT     10
static uint8_t thumbs[THUMB_COUNT][THUMB_MSG_SIZE];
static uint8_t thumb_pointer = 0;
static uint32_t next_shot_request_id;

/** Populate a USHER shot payload from the aircraft state estimator. */
static void fill_shot_message(union dc_shot_union *msg);


bool uart_cam_ctrl_set_camera(float camera_id)
{
  if (!(camera_id >= USHER_CAMERA_ALL && camera_id <= 8)) {
    return false;
  }
  uint8_t selection = (uint8_t)camera_id;
  if (camera_id != (float)selection) {
    return false;
  }
  return uart_cam_ctrl_set_camera_mask(selection == USHER_CAMERA_ALL
                                      ? USHER_CAMERA_MASK_ALL : 1U << (selection - 1));
}

bool uart_cam_ctrl_set_camera_mask(float camera_mask)
{
  if (!(camera_mask >= 0 && camera_mask <= USHER_CAMERA_MASK_ALL)) {
    return false;
  }
  uint8_t selection = (uint8_t)camera_mask;
  if (camera_mask != (float)selection) {
    return false;
  }
  digital_cam_uart_camera_mask = selection;
  return true;
}

bool uart_cam_ctrl_set_shot_report_mask(float camera_mask)
{
  if (!(camera_mask >= 0 && camera_mask <= USHER_CAMERA_MASK_ALL)) {
    return false;
  }
  uint8_t selection = (uint8_t)camera_mask;
  if (camera_mask != (float)selection) {
    return false;
  }
  digital_cam_uart_shot_report_mask = selection;
  return true;
}

bool uart_cam_ctrl_control_ready(void)
{
  return usher_control_client_ready(&control_client, get_sys_time_usec());
}

bool uart_cam_ctrl_supports(uint32_t camera_mask, uint32_t command)
{
  if (!camera_mask || camera_mask > 255 || command >= 32 || !uart_cam_ctrl_control_ready()) return false;
  for (size_t index = 0; index < 8; ++index)
    if ((camera_mask & (1U << index)) && !(control_client.capabilities[index] & CAM_CONTROL_CAP(command))) return false;
  return true;
}

uint32_t uart_cam_ctrl_command(uint32_t camera_mask, uint32_t command, uint32_t value)
{
  if (camera_mask > 255 || command > 255 || value > 255) return 0;
  const uint32_t now = get_sys_time_usec();
  struct usher_control_request request;
  uint8_t payload[USHER_CONTROL_COMMAND_SIZE];
  if (!usher_control_client_prepare(&control_client, (uint8_t)camera_mask, (uint8_t)command,
                                   (uint8_t)value, now, &request)
      || !usher_control_encode_request(payload, sizeof(payload), &request)) return 0;
  long camera_fd = 0;
  if (!CameraLinkCheckFreeSpace(UsherSizeOf(sizeof(payload)))) return 0;
  UsherHeader(USHER_CONTROL_COMMAND, sizeof(payload));
  for (size_t index = 0; index < sizeof(payload); ++index) UsherPutUint8(payload[index]);
  UsherTrailer();
  usher_control_client_commit(&control_client, &request, now);
  CameraLinkSendMessage();
  return request.request_id;
}

uint32_t uart_cam_ctrl_zoom(uint32_t camera_mask, uint32_t value)
{
  return uart_cam_ctrl_command(camera_mask, CAM_CMD_ZOOM_ABSOLUTE, value);
}

uint32_t uart_cam_ctrl_retract(uint32_t camera_mask)
{
  return uart_cam_ctrl_command(camera_mask, CAM_CMD_LENS_RETRACT, 0);
}

uint8_t uart_cam_ctrl_command_result(uint32_t ticket)
{
  usher_control_client_expire(&control_client, get_sys_time_usec());
  return usher_control_client_result(&control_client, ticket);
}

bool uart_cam_ctrl_command_pending(uint32_t ticket)
{
  return uart_cam_ctrl_command_result(ticket) == CAM_CONTROL_PENDING;
}

uint8_t uart_cam_ctrl_command_camera_result(uint32_t ticket, uint8_t camera_id)
{
  usher_control_client_expire(&control_client, get_sys_time_usec());
  return usher_control_client_camera_result(&control_client, ticket, camera_id);
}

bool uart_cam_ctrl_command_value(uint32_t ticket, uint8_t camera_id, uint8_t *value)
{
  if (!value || uart_cam_ctrl_command_camera_result(ticket, camera_id) != CAM_CONTROL_APPLIED
      || control_client.slots[camera_id - 1].evidence != CAM_CONTROL_EVIDENCE_VALUE) return false;
  *value = control_client.slots[camera_id - 1].observed;
  return true;
}

uint8_t uart_cam_ctrl_command_evidence(uint32_t ticket, uint8_t camera_id)
{
  if (uart_cam_ctrl_command_camera_result(ticket, camera_id) != CAM_CONTROL_APPLIED)
    return CAM_CONTROL_EVIDENCE_NONE;
  return control_client.slots[camera_id - 1].evidence;
}

static void control_periodic(void)
{
  const uint32_t now = get_sys_time_usec();
  usher_control_client_expire(&control_client, now);
  if (last_control_negotiate_us && (uint32_t)(now - last_control_negotiate_us) < 1000000U) return;
  if (control_client.session && (uint32_t)(now - control_client.hello_us) >= USHER_CONTROL_LEASE_US)
    usher_control_client_reset(&control_client, control_client.nonce + 1);
  long camera_fd = 0;
  if (!CameraLinkCheckFreeSpace(UsherSizeOf(USHER_CONTROL_NEGOTIATE_SIZE)
                               + UsherSizeOf(USHER_SHOOT_MASK_MSG_SIZE))) return;
  uint8_t payload[USHER_CONTROL_NEGOTIATE_SIZE] = {USHER_CONTROL_VERSION};
  usher_control_put_u32(payload, sizeof(payload), 1, control_client.nonce);
  usher_control_put_u32(payload, sizeof(payload), 5, control_client.session);
  control_client.negotiating_session = control_client.session;
  control_client.negotiating = true;
  UsherHeader(USHER_CONTROL_NEGOTIATE, sizeof(payload));
  for (size_t index = 0; index < sizeof(payload); ++index) UsherPutUint8(payload[index]);
  UsherTrailer();
  CameraLinkSendMessage();
  last_control_negotiate_us = now;
}

static void receive_shot_result(const struct usher_transport *frame)
{
  if (frame->payload_len != USHER_SHOT_RESULT_MSG_SIZE) return;
  union usher_shot_result_union result;
  for (size_t index = 0; index < sizeof(result.bin); ++index) result.bin[index] = frame->payload[index];
  if (!result.data.request_id || result.data.camera_id < 1 || result.data.camera_id > 8
      || result.data.status < USHER_SHOT_SAVED || result.data.status > USHER_SHOT_UNSUPPORTED) return;
  const uint8_t camera_bit = 1U << (result.data.camera_id - 1);
  const uint32_t now_us = get_sys_time_usec();
  for (size_t index = 0; index < SHOT_REQUEST_CAPACITY; ++index) {
    if (shot_requests[index].request_id != result.data.request_id
        || shot_requests[index].number != result.data.number
        || !(shot_requests[index].pending_mask & camera_bit)
        || (uint32_t)(now_us - shot_requests[index].sent_us) > 120000000U) continue;
    shot_requests[index].pending_mask &= ~camera_bit;
    if (result.data.status != USHER_SHOT_SAVED && result.data.status != USHER_SHOT_RECORDED)
      shot_requests[index].point_mask &= ~camera_bit;
    if (!shot_requests[index].pending_mask) shot_requests[index].request_id = 0;
    digital_cam_uart_camera_result_number[result.data.camera_id - 1] = result.data.number;
    digital_cam_uart_camera_result_status[result.data.camera_id - 1] = result.data.status;
    return;
  }
}

static void receive_cam_point(const struct usher_transport *frame)
{
  if (frame->payload_len != USHER_CAM_POINT_RESULT_MSG_SIZE) return;
  union usher_cam_point_result_union result;
  for (size_t index = 0; index < sizeof(result.bin); ++index) result.bin[index] = frame->payload[index];
  if (result.data.camera_id < 1 || result.data.camera_id > 8) return;
  const size_t slot = result.data.camera_id - 1;
  if (!result.data.session_id || result.data.session_id != camera_sessions[slot].id
      || !camera_sessions[slot].awaiting_point
      || (result.data.status != USHER_CAM_POINT_VALID && result.data.status != USHER_CAM_POINT_INVALID)) return;
  struct digital_cam_uart_point *point = &digital_cam_uart_camera_point[slot];
  if (result.data.status == USHER_CAM_POINT_INVALID) {
    // An answer without an estimate is still an answer; keep the last good point.
    point->status = USHER_CAM_POINT_INVALID;
    point_from_stream[slot] = false;
    point->session_id = result.data.session_id;
    ++point->sequence;
    camera_sessions[slot].awaiting_point = false;
    return;
  }
  if (result.data.lat < -900000000 || result.data.lat > 900000000
      || result.data.lon < -1800000000 || result.data.lon > 1800000000
      || (result.data.lat == 0 && result.data.lon == 0)
      || result.data.agl_mm < 0 || result.data.quality < 0 || result.data.quality > 1000) return;
  point->lat = result.data.lat;
  point_from_stream[slot] = false;
  point->lon = result.data.lon;
  point->alt_mm = result.data.alt_mm;
  point->agl_mm = result.data.agl_mm;
  point->quality = result.data.quality;
  point->metric = result.data.metric;
  point->status = USHER_CAM_POINT_VALID;
  point->session_id = result.data.session_id;
  ++point->sequence;
  camera_sessions[slot].awaiting_point = false;
}

/** Accept recurring points independently of stopped sessions and JPEG acknowledgements. */
static void receive_cam_point_update(const struct usher_transport *frame)
{
  if (frame->payload_len != USHER_CAM_POINT_UPDATE_MSG_SIZE) return;
  union usher_cam_point_update_union result;
  for (size_t index = 0; index < sizeof(result.bin); ++index) result.bin[index] = frame->payload[index];
  if (result.data.camera_id < 1 || result.data.camera_id > 8 || !result.data.frame_number
      || result.data.frame_number > INT32_MAX
      || (result.data.status != USHER_CAM_POINT_VALID && result.data.status != USHER_CAM_POINT_INVALID)) return;
  const size_t slot = result.data.camera_id - 1;
  const uint8_t camera_bit = 1U << slot;
  if (result.data.frame_number <= point_stream_number[slot]) return;
  if (result.data.status == USHER_CAM_POINT_VALID
      && (result.data.lat < -900000000 || result.data.lat > 900000000
          || result.data.lon < -1800000000 || result.data.lon > 1800000000
          || (!result.data.lat && !result.data.lon) || result.data.agl_mm < 0
          || result.data.quality < 0 || result.data.quality > 1000)) return;
  const uint32_t now = get_sys_time_usec();
  for (size_t index = 0; index < SHOT_REQUEST_CAPACITY; ++index) {
    if (shot_requests[index].number != (int32_t)result.data.frame_number
        || !(shot_requests[index].point_mask & camera_bit)
        || (uint32_t)(now - shot_requests[index].sent_us) > CAMERA_POINT_STREAM_MAX_AGE_US) continue;
    struct digital_cam_uart_point *point = &digital_cam_uart_camera_point[slot];
    shot_requests[index].point_mask &= ~camera_bit;
    point_stream_number[slot] = result.data.frame_number;
    point_from_stream[slot] = true;
    point->status = result.data.status;
    point->session_id = result.data.frame_number;
    ++point->sequence;
    if (result.data.status == USHER_CAM_POINT_VALID) {
      point->lat = result.data.lat;
      point->lon = result.data.lon;
      point->alt_mm = result.data.alt_mm;
      point->agl_mm = result.data.agl_mm;
      point->quality = result.data.quality;
      point->metric = result.data.metric;
    }
    return;
  }
}

bool digital_cam_uart_register_rx_handler(uint8_t message_id, digital_cam_uart_rx_handler_t handler)
{
  if (!handler || message_id == USHER_SHOT_RESULT || message_id == USHER_CLOCK_REQUEST || message_id == USHER_STATUS
      || message_id == USHER_PAYLOAD || message_id == USHER_CAPTURE_ARBITER_HELLO
      || message_id == USHER_CONTROL_HELLO || message_id == USHER_CONTROL_RESULT
      || message_id == USHER_CAM_POINT_RESULT || message_id == USHER_CAM_POINT_UPDATE
      || message_id == 0) return false;
  for (size_t index = 0; index < DIGITAL_CAM_UART_RX_HANDLERS; ++index) {
    if (rx_handlers[index].handler && rx_handlers[index].message_id == message_id)
      return rx_handlers[index].handler == handler;
  }
  for (size_t index = 0; index < DIGITAL_CAM_UART_RX_HANDLERS; ++index) {
    if (!rx_handlers[index].handler) {
      rx_handlers[index].message_id = message_id;
      rx_handlers[index].handler = handler;
      return true;
    }
  }
  return false;
}

static void dispatch_application_reply(const struct usher_transport *frame)
{
  for (size_t index = 0; index < DIGITAL_CAM_UART_RX_HANDLERS; ++index) {
    if (rx_handlers[index].handler && rx_handlers[index].message_id == frame->msg_id) {
      rx_handlers[index].handler(frame);
      return;
    }
  }
}

/** @brief Service available UART input and dispatch complete USHER replies.
 * @details Parses buffered bytes, updates transport-owned status/results, and
 * invokes registered handlers for application messages that this module does
 * not consume. It returns no value; malformed or unsupported frames are ignored.
 */
void digital_cam_uart_event(void)
{
#ifdef DIGITAL_CAM_UART_SIM_BACKEND
  digital_cam_uart_sim_service(&digital_cam_uart_sim_link);
#endif
  while (CameraLinkChAvailable()) {
    parse_usher(&usher_protocol, CameraLinkGetch());
    if (usher_protocol.msg_received) {
      switch (usher_protocol.msg_id) {
        case USHER_CONTROL_HELLO:
          usher_control_client_hello(&control_client, usher_protocol.payload, usher_protocol.payload_len,
                                     get_sys_time_usec());
          break;
        case USHER_CONTROL_RESULT:
          usher_control_client_receive(&control_client, usher_protocol.payload, usher_protocol.payload_len,
                                       get_sys_time_usec());
          break;
        case USHER_SHOT_RESULT:
          receive_shot_result(&usher_protocol);
          break;
        case USHER_CAM_POINT_RESULT:
          receive_cam_point(&usher_protocol);
          break;
        case USHER_CAM_POINT_UPDATE:
          receive_cam_point_update(&usher_protocol);
          break;
        case USHER_CLOCK_REQUEST:
          reply_clock_request();
          break;
        case USHER_STATUS:
          if (usher_protocol.payload_len != USHER_STATUS_MSG_SIZE) break;
          for (int i = 0; i < USHER_STATUS_MSG_SIZE; i++) {
            usher_status_msg.bin[i] = usher_protocol.payload[i];
          }
          digital_cam_uart_status = usher_status_msg.data.shots;
          break;
        case USHER_PAYLOAD:
          if (usher_protocol.payload_len != USHER_PAYLOAD_MSG_SIZE) break;
          for (int i = 0; i < USHER_PAYLOAD_MSG_SIZE; i++) {
            thumbs[thumb_pointer][i] = usher_protocol.payload[i];
          }
          break;
        default:
          dispatch_application_reply(&usher_protocol);
          break;
      }
      usher_protocol.msg_received = 0;
    }
  }
}

//There is an option to send a tiny thumbnail image over the telemetry link
//This is controlled by the DIGITAL_CAM_UART_THUMBNAILS parameter
#if PERIODIC_TELEMETRY
static void send_thumbnails(struct transport_tx *trans, struct link_device *dev)
{
  long camera_fd = 0;
  static int cnt = 0;
  if (digital_cam_uart_thumbnails > 0) {
    if (digital_cam_uart_thumbnails == 1) {
      cnt++;
      if (cnt > 1) {
        cnt = 0;
        return;
      }
    }
    pprz_msg_send_PAYLOAD(trans, dev, AC_ID, THUMB_MSG_SIZE, thumbs[thumb_pointer]);

    // Update the write/read pointer: if we receive a new thumb part, that will be sent, otherwise the oldest infor is repeated
    thumb_pointer++;
    if (thumb_pointer >= THUMB_COUNT) {
      thumb_pointer = 0;
    }

    if (CameraLinkCheckFreeSpace(UsherSizeOf(0))) {
      UsherHeader(USHER_BUFFER_EMPTY, 0);
      UsherTrailer();
      CameraLinkSendMessage();
    }
  }
}
#endif

void digital_cam_uart_init(void)
{
  digital_cam_uart_reset_protocol_state();
#ifdef DIGITAL_CAM_UART_SIM_BACKEND
  digital_cam_uart_sim_init();
#endif
  pose_sequence = 0;
  digital_cam_uart_thumbnails = 0;
  for (int t = 0; t < THUMB_COUNT; t++) {
    for (int i = 0; i < THUMB_MSG_SIZE; i++) {
      thumbs[t][i] = 0;
    }
  }
#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_PAYLOAD, send_thumbnails);
#endif
}

/** @brief Run camera housekeeping and the shared DC periodic algorithm.
 * @details Reserving a complete ordinary shot frame before evaluating autoshoot
 * keeps TX backpressure from advancing an unsent timed or spatial shot.
 * Flight plans own decisions; optional pose traffic follows shot commands.
 */
void digital_cam_uart_periodic(void)
{
#ifdef DIGITAL_CAM_UART_SIM_BACKEND
  digital_cam_uart_sim_periodic();
#endif
  control_periodic();
  if (autoshoot_can_send_shot() && digital_cam_uart_camera_mask != USHER_CAMERA_MASK_NONE) {
    long camera_fd = 0;
    if (digital_cam_uart_camera_mask <= USHER_CAMERA_MASK_SUPPORTED
        && CameraLinkCheckFreeSpace(UsherSizeOf(USHER_SHOOT_MASK_MSG_SIZE))) {
      /* On ChibiOS this owns the TX mutex as well as space. The nested sender
       * must consume this reservation, not reserve again and deadlock itself. */
      reserved_autoshoot_mask = digital_cam_uart_camera_mask;
      reserved_autoshoot_fd = &camera_fd;
      dc_periodic();
      /* No photo may have been due. Release the unused reservation without
       * emitting a frame, or the next producer could inherit a locked UART. */
      if (reserved_autoshoot_fd) CameraLinkSendMessage();
      reserved_autoshoot_fd = NULL;
      reserved_autoshoot_mask = USHER_CAMERA_MASK_NONE;
    }
  } else dc_periodic();
#if DIGITAL_CAM_UART_POSE_STREAM
  send_pose_sample();
#else
  if (clock_reply_sent && (uint32_t)(get_sys_time_usec() - last_clock_reply_us) < 2000000U) {
    const uint32_t now = get_sys_time_usec();
    if ((uint32_t)(now - last_pose_sample_us) >= 100000U) {
      last_pose_sample_us = now;
      send_pose_sample();
    }
  }
#endif
}

/**
 * @brief Snapshot navigation and attitude state into a shot record.
 * @param[out] msg Shot record populated with the next photo number and pose.
 */
static void fill_shot_message(union dc_shot_union *msg)
{
  msg->data.nr = dc_photo_nr + 1;
  msg->data.lat = stateGetPositionLla_i()->lat;
  msg->data.lon = stateGetPositionLla_i()->lon;
  msg->data.alt = stateGetPositionLla_i()->alt;
  msg->data.phi = stateGetNedToBodyEulers_i()->phi;
  msg->data.theta = stateGetNedToBodyEulers_i()->theta;
  msg->data.psi = stateGetNedToBodyEulers_i()->psi;
  msg->data.vground = stateGetHorizontalSpeedNorm_i();
  msg->data.course = stateGetHorizontalSpeedDir_i();
#if FIXEDWING_FIRMWARE
  msg->data.groundalt = POS_BFP_OF_REAL(stateGetPositionUtm_f()->alt - ground_alt);
#else
  msg->data.groundalt = POS_BFP_OF_REAL(state.alt_agl_f);
#endif
}

/**
 * @brief Reply to a valid USHER clock-alignment request.
 * @details Reply and shot-frame capacity is reserved together so diagnostic
 * clock traffic cannot consume the space needed for a capture command.
 */
static void reply_clock_request(void)
{
  long camera_fd = 0;
  if (usher_protocol.payload_len != USHER_CLOCK_REQUEST_MSG_SIZE) return;
  union usher_clock_reply_union reply = {0};
  reply.data.receive_us = get_sys_time_usec();
  for (size_t index = 0; index < sizeof(reply.data.request.bin); ++index) {
    reply.data.request.bin[index] = usher_protocol.payload[index];
  }
  if ((reply.data.request.data.token_low == 0 && reply.data.request.data.token_high == 0)
      || (clock_reply_sent && (uint32_t)(reply.data.receive_us - last_clock_reply_us) < 500000)
      || !CameraLinkCheckFreeSpace(UsherSizeOf(USHER_CLOCK_REPLY_MSG_SIZE)
             + UsherSizeOf(USHER_SHOOT_MASK_MSG_SIZE))) return;
  reply.data.transmit_us = get_sys_time_usec();
  UsherHeader(USHER_CLOCK_REPLY, USHER_CLOCK_REPLY_MSG_SIZE);
  for (size_t index = 0; index < sizeof(reply.bin); ++index) UsherPutUint8(reply.bin[index]);
  UsherTrailer();
  CameraLinkSendMessage();
  pose_clock_token = reply.data.request;
  last_clock_reply_us = reply.data.transmit_us;
  clock_reply_sent = true;
}

/**
 * @brief Send one optional diagnostic pose sample when a complete frame fits.
 * @details Pose samples are best-effort telemetry. Dropping one under TX
 * pressure is preferable to emitting a partial frame or delaying camera
 * commands. Endpoint changes clear the peer-specific clock token.
 */
static void send_pose_sample(void)
{
  long camera_fd = 0;
  const uint32_t sequence = pose_sequence++;
  const bool clocked = pose_clock_token.data.token_low != 0 || pose_clock_token.data.token_high != 0;
  const uint8_t payload_size = clocked ? USHER_POSE_CLOCKED_MSG_SIZE : USHER_POSE_SAMPLE_MSG_SIZE;
  const uint16_t required_space = UsherSizeOf(payload_size)
                                  + UsherSizeOf(USHER_SHOOT_MASK_MSG_SIZE);
  if (!CameraLinkCheckFreeSpace(required_space)) {
    return;
  }
  union usher_pose_sample_union sample = {0};
  sample.data.sequence = sequence;
  sample.data.sample_begin_us = get_sys_time_usec();
  fill_shot_message(&sample.data.shot);
  const struct NedCoor_i *velocity = stateGetSpeedNed_i();
  sample.data.velocity_north_bfp = velocity->x;
  sample.data.velocity_east_bfp = velocity->y;
  sample.data.velocity_down_bfp = velocity->z;
#if USE_GPS
  sample.data.gps_tow_ms = gps.tow;
  sample.data.gps_week = gps.week;
  sample.data.gps_hacc_cm = gps.hacc;
  sample.data.gps_vacc_cm = gps.vacc;
  sample.data.gps_sacc_cm_s = gps.sacc;
  sample.data.gps_fix = gps.fix;
  sample.data.gps_num_sv = gps.num_sv;
  sample.data.gps_valid_fields = gps.valid_fields;
  sample.data.flags = USHER_POSE_SAMPLE_GPS_PRESENT;
#if defined(MODULE_INS_ALT_FLOAT_ID) && !defined(INS_ALT_GPS_ID) && !defined(NPS)
  const uint32_t estimated_tow = gps_tow_from_sys_ticks(sys_time.nb_tick);
  uint32_t age_ms = (estimated_tow + 604800000U - gps.tow) % 604800000U;
  if (age_ms > (USHER_POSE_SAMPLE_GPS_AGE_MASK >> USHER_POSE_SAMPLE_GPS_AGE_SHIFT))
    age_ms = USHER_POSE_SAMPLE_GPS_AGE_MASK >> USHER_POSE_SAMPLE_GPS_AGE_SHIFT;
  sample.data.flags |= USHER_POSE_SAMPLE_GPS_HELD | USHER_POSE_SAMPLE_GPS_AGE_VALID
                    | (age_ms << USHER_POSE_SAMPLE_GPS_AGE_SHIFT);
#endif
#endif
  sample.data.sample_end_us = get_sys_time_usec();
  UsherHeader(clocked ? USHER_POSE_CLOCKED : USHER_POSE_SAMPLE, payload_size);
  for (size_t index = 0; index < USHER_POSE_SAMPLE_MSG_SIZE; ++index) {
    UsherPutUint8(sample.bin[index]);
  }
  if (clocked) {
    for (size_t index = 0; index < sizeof(pose_clock_token.bin); ++index) UsherPutUint8(pose_clock_token.bin[index]);
  }
  UsherTrailer();
  CameraLinkSendMessage();
}
static bool send_masked_shot(uint32_t camera_mask, long *reservation)
{
  long camera_fd = 0;
  if (camera_mask == USHER_CAMERA_MASK_NONE || camera_mask > USHER_CAMERA_MASK_ALL) {
    if (reservation) {
      camera_fd = *reservation;
      CameraLinkSendMessage();
    }
    return false;
  }
  if (reservation) camera_fd = *reservation;
  else if (!CameraLinkCheckFreeSpace(UsherSizeOf(USHER_SHOOT_MASK_MSG_SIZE))) return false;
  union dc_shot_mask_union msg;
  fill_shot_message(&msg.data.shot);
  msg.data.camera_mask = camera_mask;
  if (!next_shot_request_id) next_shot_request_id = get_sys_time_usec() | 1U;
  msg.data.request_id = next_shot_request_id++;
  if (!next_shot_request_id) ++next_shot_request_id;
  UsherHeader(USHER_SHOOT_MASK, USHER_SHOOT_MASK_MSG_SIZE);
  for (size_t i = 0; i < sizeof(msg.bin); i++) {
    UsherPutUint8(msg.bin[i]);
  }
  UsherTrailer();
  CameraLinkSendMessage();
  const uint8_t slot = next_shot_request_slot++ % SHOT_REQUEST_CAPACITY;
  shot_requests[slot].request_id = msg.data.request_id;
  shot_requests[slot].sent_us = get_sys_time_usec();
  shot_requests[slot].number = msg.data.shot.data.nr;
  shot_requests[slot].pending_mask = (uint8_t)camera_mask;
  shot_requests[slot].point_mask = (uint8_t)camera_mask;
  return true;
}

static void finish_admitted_shot(uint32_t camera_mask, bool report)
{
  if (report && (camera_mask & digital_cam_uart_shot_report_mask)) {
    dc_send_shot_position();
  } else if (dc_photo_nr < DC_IMAGE_BUFFER) {
    dc_photo_nr++;
  }
}

uint8_t digital_cam_uart_shoot(uint8_t camera_id, bool report)
{
  if (camera_id > 8) return 1;
  const uint32_t mask = camera_id == USHER_CAMERA_ALL ? USHER_CAMERA_MASK_ALL : 1U << (camera_id - 1);
  if (!send_masked_shot(mask, NULL)) return 1;
  finish_admitted_shot(mask, report);
  return 0;
}

static void put_session_frame(uint8_t message_id, uint8_t camera_id, uint32_t session_id, long camera_fd)
{
  union usher_camera_session_union command = {.data = {camera_id, session_id}};
  UsherHeader(message_id, sizeof(command.bin));
  for (size_t index = 0; index < sizeof(command.bin); ++index) {
    UsherPutUint8(command.bin[index]);
  }
  UsherTrailer();
}

uint8_t digital_cam_uart_start(uint8_t camera_id)
{
  long camera_fd = 0;
  if (camera_id > 8 || next_camera_session_id == UINT32_MAX
      || !CameraLinkCheckFreeSpace(UsherSizeOf(USHER_START_TARGETED_MSG_SIZE))) return 1;
  const uint32_t session_id = ++next_camera_session_id;
  put_session_frame(USHER_START_TARGETED, camera_id, session_id, camera_fd);
  CameraLinkSendMessage();
  for (size_t slot = 0; slot < 8; ++slot) {
    if (camera_id != USHER_CAMERA_ALL && slot != (size_t)(camera_id - 1)) continue;
    camera_sessions[slot].id = session_id;
    camera_sessions[slot].awaiting_point = false;
    point_from_stream[slot] = false;
    for (size_t request = 0; request < SHOT_REQUEST_CAPACITY; ++request)
      shot_requests[request].point_mask &= ~(1U << slot);
    digital_cam_uart_camera_point[slot].session_id = 0;
    digital_cam_uart_camera_point[slot].status = USHER_CAM_POINT_INVALID;
  }
  return 0;
}

uint8_t digital_cam_uart_stop(uint8_t camera_id)
{
  if (camera_id > 8) return 1;
  uint8_t pending_mask = 0;
  size_t frame_count = 0;
  bool has_session = false;
  for (size_t slot = 0; slot < 8; ++slot) {
    if (camera_id != USHER_CAMERA_ALL && slot != (size_t)(camera_id - 1)) continue;
    if (!camera_sessions[slot].id) continue;
    has_session = true;
    if (!point_from_stream[slot] && digital_cam_uart_camera_point[slot].session_id == camera_sessions[slot].id) continue;
    pending_mask |= (uint8_t)(1U << slot);
    ++frame_count;
  }
  if (!has_session) return 1;
  if (!frame_count) return 0;
  long camera_fd = 0;
  if (!CameraLinkCheckFreeSpace(frame_count * UsherSizeOf(USHER_STOP_TARGETED_MSG_SIZE))) return 1;
  for (size_t slot = 0; slot < 8; ++slot) {
    if (!(pending_mask & (1U << slot))) continue;
    put_session_frame(USHER_STOP_TARGETED, (uint8_t)(slot + 1), camera_sessions[slot].id, camera_fd);
    camera_sessions[slot].awaiting_point = true;
    for (size_t request = 0; request < SHOT_REQUEST_CAPACITY; ++request)
      shot_requests[request].point_mask &= ~(1U << slot);
  }
  CameraLinkSendMessage();
  return 0;
}


/** @brief Implement the shared DC command interface using the UART camera transport.
 * @param cmd Existing DC command code.
 * @details An autoshoot command consumes its pre-reserved frame. Other shots
 * reserve independently. No capture policy or survey session is applied here.
 */
void dc_send_command(uint8_t cmd)
{
  switch (cmd) {
    case DC_SHOOT:
      if (reserved_autoshoot_fd || digital_cam_uart_camera_mask != USHER_CAMERA_MASK_NONE) {
        long *reservation = reserved_autoshoot_fd;
        const uint8_t mask = reservation ? reserved_autoshoot_mask : digital_cam_uart_camera_mask;
        reserved_autoshoot_fd = NULL;
        if (send_masked_shot(mask, reservation)) finish_admitted_shot(mask, true);
      }
      break;
    case DC_TALLER:
      break;
    case DC_WIDER:
      break;
    case DC_ON:
      break;
    case DC_OFF:
      break;
    default:
      break;
  }

  // call command send_command function
  dc_send_command_common(cmd);
}
