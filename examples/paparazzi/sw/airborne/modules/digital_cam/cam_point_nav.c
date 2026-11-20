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

/** @file modules/digital_cam/cam_point_nav.c
 *  @brief Apply a point deduced by any USHER camera to a flight plan waypoint.
 */

#include "cam_point_nav.h"

#include "modules/digital_cam/uart_cam_ctrl.h"
#include "usher_protocol.h"
#include "math/pprz_geodetic_float.h"
#include "math/pprz_geodetic_int.h"
#include "state.h"
#include <math.h>

#if FIXEDWING_FIRMWARE
#include "modules/nav/common_nav.h"
#include "firmwares/fixedwing/nav.h"
#include "generated/flight_plan.h"
#else
#include "modules/nav/waypoints.h"
#endif

/** Session answer already acknowledged or discarded, per camera. */
static uint32_t consumed_session_id[8];

static const struct digital_cam_uart_point *point_of(uint8_t camera_id)
{
  if (camera_id < 1 || camera_id > 8) {
    return NULL;
  }
  return &digital_cam_uart_camera_point[camera_id - 1];
}

/** Convert the stored point to flight plan local coordinates in metres. */
static bool point_to_local_xy(const struct digital_cam_uart_point *point, float *x, float *y)
{
  struct LlaCoor_i lla_i = { .lat = point->lat, .lon = point->lon, .alt = point->alt_mm };
#if FIXEDWING_FIRMWARE
  struct LlaCoor_f lla;
  struct UtmCoor_f utm;
  LLA_FLOAT_OF_BFP(lla, lla_i);
  utm.zone = nav_utm_zone0;
  utm_of_lla_f(&utm, &lla);
  *x = utm.east - nav_utm_east0;
  *y = utm.north - nav_utm_north0;
#else
  if (!stateIsLocalCoordinateValid()) {
    return false;
  }
  struct EnuCoor_i enu;
  enu_of_lla_point_i(&enu, stateGetNedOrigin_i(), &lla_i);
  *x = enu.x / 100.f;   // enu_of_lla_point_i returns centimetres
  *y = enu.y / 100.f;
#endif
  return isfinite(*x) && isfinite(*y);
}

static bool waypoint_local_xy(uint8_t wp_id, float *x, float *y)
{
#if FIXEDWING_FIRMWARE
  if (wp_id >= nb_waypoint) {
    return false;
  }
  *x = WaypointX(wp_id);
  *y = WaypointY(wp_id);
#else
  if (wp_id >= nb_waypoint) {
    return false;
  }
  *x = waypoint_get_x(wp_id);
  *y = waypoint_get_y(wp_id);
#endif
  return isfinite(*x) && isfinite(*y);
}

bool cam_point_valid(uint8_t camera_id)
{
  const struct digital_cam_uart_point *point = point_of(camera_id);
  return point && point->session_id && point->status == USHER_CAM_POINT_VALID;
}

bool cam_point_fresh(uint8_t camera_id)
{
  const struct digital_cam_uart_point *point = point_of(camera_id);
  return point && point->session_id && point->session_id != consumed_session_id[camera_id - 1];
}

bool cam_point_consume(uint8_t camera_id)
{
  const struct digital_cam_uart_point *point = point_of(camera_id);
  if (!point || !point->session_id) return false;
  consumed_session_id[camera_id - 1] = point->session_id;
  return true;
}

float cam_point_quality(uint8_t camera_id)
{
  const struct digital_cam_uart_point *point = point_of(camera_id);
  if (!point || point->status != USHER_CAM_POINT_VALID) {
    return 0.f;
  }
  if (point->quality <= 0) {
    return 0.f;
  }
  return point->quality >= 1000 ? 1.f : point->quality / 1000.f;
}

bool cam_point_within(uint8_t camera_id, uint8_t wp_ref, float radius_m)
{
  if (!cam_point_valid(camera_id) || !isfinite(radius_m) || radius_m <= 0.f) {
    return false;
  }
  float x, y, wx, wy;
  if (!point_to_local_xy(point_of(camera_id), &x, &y) || !waypoint_local_xy(wp_ref, &wx, &wy)) {
    return false;
  }
  const float dx = x - wx;
  const float dy = y - wy;
  return hypotf(dx, dy) <= radius_m;
}

bool cam_point_to_waypoint(uint8_t camera_id, uint8_t wp_id)
{
  if (!cam_point_valid(camera_id) || wp_id >= nb_waypoint) {
    return false;
  }
  float x, y;
  if (!point_to_local_xy(point_of(camera_id), &x, &y)) {
    return false;
  }
#if FIXEDWING_FIRMWARE
  if (WP_HOME >= nb_waypoint || !isfinite(max_dist_from_home) || max_dist_from_home < 0.f
      || !isfinite(WaypointX(WP_HOME)) || !isfinite(WaypointY(WP_HOME))
      || !isfinite(WaypointAlt(wp_id))
      || fabsf(x - WaypointX(WP_HOME)) > max_dist_from_home
      || fabsf(y - WaypointY(WP_HOME)) > max_dist_from_home) {
    return false;
  }
  WaypointX(wp_id) = x;
  WaypointY(wp_id) = y;
#else
  struct EnuCoor_f enu = { x, y, waypoint_get_alt(wp_id) };
  if (!isfinite(enu.z)) return false;
  waypoint_set_enu(wp_id, &enu);
#endif
  return true;
}
