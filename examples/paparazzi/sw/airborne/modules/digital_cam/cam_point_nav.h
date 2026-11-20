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

/** @file modules/digital_cam/cam_point_nav.h
 *  @brief Apply a point deduced by any USHER camera to a flight plan waypoint.
 *
 * A specialized camera runs between digital_cam_uart_start() and
 * digital_cam_uart_stop() and answers with one geographic point. This module is
 * camera-agnostic: the camera ID selects whose answer is used, so a future
 * camera needs no new flight controller code.
 */

#ifndef DIGITAL_CAM_CAM_POINT_NAV_H
#define DIGITAL_CAM_CAM_POINT_NAV_H

#include "std.h"

/** True when the current session has a usable accepted answer.
 * Start, peer reset, or an INVALID answer makes this false.
 * @param camera_id The ID of the camera to check for a valid point.
 */
extern bool cam_point_valid(uint8_t camera_id);

/** True for a current session answer not yet explicitly consumed, even if INVALID.
 * Start and peer reset make this false. Waypoint application does not consume it.
 * @param camera_id The ID of the camera to check for a fresh point.
 */
extern bool cam_point_fresh(uint8_t camera_id);

/** Drop the fresh flag without using the point, so the next answer can be awaited.
 * Returns true when a current answer was consumed, false when none is available.
 * @param camera_id The ID of the camera whose point is being consumed.
 */
extern bool cam_point_consume(uint8_t camera_id);

/** Camera-defined confidence in [0, 1]; not comparable between camera types.
 * Returns a float in the range [0, 1], where 1 indicates maximum confidence.
 */
extern float cam_point_quality(uint8_t camera_id);

/** True when a valid point lies within radius_m of wp_ref; a plausibility gate, not proof.
 * Returns true if the camera's point is within the specified radius of the waypoint reference.
 * Otherwise, returns false.
 */
extern bool cam_point_within(uint8_t camera_id, uint8_t wp_ref, float radius_m);

/**
 * Move wp_id onto the camera's point, keeping the waypoint altitude. This does
 * not consume the answer; the flight plan calls cam_point_consume() explicitly.
 * @param camera_id The ID of the camera providing the point.
 * @param wp_id The ID of the waypoint to move onto the camera's point.
 * @return true when moved; false for an invalid camera point or waypoint,
 * failed coordinate conversion, or a position outside fixed-wing navigation limits.
 */
extern bool cam_point_to_waypoint(uint8_t camera_id, uint8_t wp_id);

#endif // DIGITAL_CAM_CAM_POINT_NAV_H
