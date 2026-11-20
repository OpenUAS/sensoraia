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
 * @file modules/digital_cam/uart_cam_ctrl.h
 * @brief Public API for USHER digital-camera control over UART.
 *
 * @details The flight controller sends pose-tagged capture commands to an
 * external camera computer and receives status, thumbnails, or application
 * replies. The API is transport-independent: hardware uses `CAMERA_PORT`, while
 * simulation targets may use the module-owned local/fallback serial adapter.
 *
 * @see modules/digital_cam/uart_cam_ctrl.c for transport ownership and endpoint
 * switching rationale.
 */

#ifndef DIGITAL_CAM_UART_H
#define DIGITAL_CAM_UART_H

#include "modules/digital_cam/dc.h"
#include "usher_protocol.h"
#include "camera_control.h"

/**
 * @defgroup digital_cam_uart Digital camera UART control
 * @brief Pose-tagged USHER camera commands, replies, and thumbnail relay.
 * @{
 */

/** Initialize USHER protocol state, camera buffers, and the selected transport. */
extern void digital_cam_uart_init(void);

/** Run camera housekeeping, optional pose streaming, and simulation link failover. */
extern void digital_cam_uart_periodic(void);
/** Service transport input and dispatch complete USHER frames. */
extern void digital_cam_uart_event(void);

/** Thumbnail downlink mode: zero disables, one throttles, and two sends every opportunity. */
extern int digital_cam_uart_thumbnails;
/** Shot count reported by the most recent USHER status frame. */
extern int digital_cam_uart_status;
/** Validated terminal outcome for each camera ID (array index is ID - 1).
 * Status zero means no result since the last peer reset. An absent result is
 * unknown, not proof of failure: feedback can be lost under UART pressure.
 * DC_SHOT is the optional single GCS trigger-number report for a masked request.
 */
extern uint32_t digital_cam_uart_camera_result_status[8];
extern int32_t digital_cam_uart_camera_result_number[8];

/** A geographic point deduced by one camera, indexed by camera ID - 1.
 * A camera reports one on ::digital_cam_uart_stop. Only one matching answer is
 * accepted for a stopped session. session_id zero means no current answer;
 * start and peer reset invalidate it. sequence counts accepted replies.
 */
struct digital_cam_uart_point {
  int32_t lat;       /**< 1e7 deg. */
  int32_t lon;       /**< 1e7 deg. */
  int32_t alt_mm;    /**< Ellipsoid altitude of the point. */
  int32_t agl_mm;    /**< Observation height above the point. */
  int32_t quality;   /**< 0..1000, camera-defined; not comparable across camera types. */
  int32_t metric;    /**< Camera-defined scalar. */
  uint32_t status;   /**< ::USHER_CAM_POINT_VALID, ::USHER_CAM_POINT_INVALID, or zero. */
  uint32_t session_id; /**< Nonzero FC token echoed by the accepted reply. */
  uint32_t sequence; /**< Increments on every accepted reply for this camera. */
};
extern struct digital_cam_uart_point digital_cam_uart_camera_point[8];

/** Bit mask of cameras triggered by the standard ::DC_SHOOT command. */
extern uint8_t digital_cam_uart_camera_mask;

/** Cameras whose admitted triggers permit DC_SHOT telemetry; independent of capture selection.
 * A mixed trigger reports once if any selected bit is enabled. Zero silences
 * reports without suppressing captures or shot numbering. Defaults to all bits
 * through DIGITAL_CAM_UART_SHOT_REPORT_MASK and survives peer reconnection.
 */
extern uint8_t digital_cam_uart_shot_report_mask;

/**
 * @brief Select one camera for standard capture commands.
 * @param camera_id Camera ID in [1, 8], or zero for all eight slots.
 * @return `true` when valid and applied; `false` leaves the selection unchanged.
 * @note This compatibility helper converts the ID to a one-bit camera mask.
 */
extern bool uart_cam_ctrl_set_camera(float camera_id);

/**
 * @brief Select any combination of cameras for standard capture commands.
 * @param camera_mask Integer-valued mask in [0, 255]; zero suppresses captures.
 * @return `true` when valid and applied; `false` leaves the selection unchanged.
 */
extern bool uart_cam_ctrl_set_camera_mask(float camera_mask);

/**
 * @brief Set the cameras eligible for DC_SHOT reports without changing captures.
 * @param camera_mask Integer-valued mask in [0, 255]; zero silences every camera.
 * @return `true` when valid and applied; `false` leaves the policy unchanged.
 */
extern bool uart_cam_ctrl_set_shot_report_mask(float camera_mask);

#ifdef __cplusplus
extern "C" {
#endif
extern uint32_t uart_cam_ctrl_command(uint32_t camera_mask, uint32_t command, uint32_t value);
extern uint32_t uart_cam_ctrl_zoom(uint32_t camera_mask, uint32_t value);
extern uint32_t uart_cam_ctrl_retract(uint32_t camera_mask);
extern bool uart_cam_ctrl_control_ready(void);
extern bool uart_cam_ctrl_supports(uint32_t camera_mask, uint32_t command);
extern bool uart_cam_ctrl_command_pending(uint32_t ticket);
extern uint8_t uart_cam_ctrl_command_result(uint32_t ticket);
extern uint8_t uart_cam_ctrl_command_camera_result(uint32_t ticket, uint8_t camera_id);
extern bool uart_cam_ctrl_command_value(uint32_t ticket, uint8_t camera_id, uint8_t *value);
extern uint8_t uart_cam_ctrl_command_evidence(uint32_t ticket, uint8_t camera_id);
#ifdef __cplusplus
}
#endif

/**
 * @brief Request a pose-tagged capture from one camera or all cameras.
 * @param camera_id USHER camera ID, or ::USHER_CAMERA_ALL.
 * @param report Permit DC_SHOT telemetry subject to the report mask; false always silences it.
 * @return Zero when the complete USHER frame was queued, nonzero otherwise.
 * @note The photo number and optional DC_SHOT report advance only after the
 * complete frame was admitted to the UART, not on a failed request.
 */
extern uint8_t digital_cam_uart_shoot(uint8_t camera_id, bool report);

/**
 * @brief Start a capture session on one camera or all cameras.
 * @param camera_id USHER camera ID, or ::USHER_CAMERA_ALL.
 * @return Zero when the complete USHER frame was queued, nonzero otherwise.
 * @note Allocates a nonzero token and invalidates prior answers only on admission.
 * Tokens are not reused during this FC runtime, including peer resets. A camera
 * samples until matching stop; queuing is not evidence that the camera started.
 */
extern uint8_t digital_cam_uart_start(uint8_t camera_id);

/**
 * @brief Stop a capture session and ask the camera for its deduced point.
 * @param camera_id USHER camera ID, or ::USHER_CAMERA_ALL.
 * @return Zero when all required frames were admitted (or already answered),
 * nonzero for an invalid ID, no started session, or insufficient TX capacity.
 * @note Replies must echo the started session token. All-camera stop reserves
 * the whole batch; no partial stop is emitted. The answer arrives asynchronously
 * in ::digital_cam_uart_camera_point. Repeated stop reuses the same token.
 */
extern uint8_t digital_cam_uart_stop(uint8_t camera_id);

/**
 * @brief Callback for a reply assigned to one message ID.
 * @param frame Parsed frame; valid only for the duration of the callback.
 * @return `true` if accepted; rejected replies are not retried or forwarded.
 */
typedef bool (*digital_cam_uart_rx_handler_t)(const struct usher_transport *frame);

/** @brief Register one application reply consumer for a wire message ID.
 * @return True when the ID was registered (or already belongs to this handler).
 * False for a null handler, duplicate ID owned by another consumer, or full table.
 * @note Registration is startup-only; handlers persist across endpoint switches.
 * Transport-owned messages are not delivered; application handlers validate
 * their own payloads and any session/shot identity before changing flight state.
 */
extern bool digital_cam_uart_register_rx_handler(uint8_t message_id, digital_cam_uart_rx_handler_t handler);

/** @} */

#endif // DIGITAL_CAM_UART_H
