/*
 * Copyright (C) 2026
 *
 * This file is part of paparazzi.
 */

#include "modules/cyberzoo_navigation/cyberzoo_perimeter_waypoints.h"

#include "firmwares/rotorcraft/navigation.h"
#include "generated/flight_plan.h"
#include "modules/cyberzoo_navigation/waypoint_navigation.h"
#include "state.h"
#include <math.h>

float cz_perimeter_lookahead = 1.2f;
float cz_perimeter_corner_radius = 1.f;

static uint8_t current_edge;
static struct FloatVect2 current_dir;
static bool dir_valid;
static bool path_hold_active;
static struct FloatVect2 path_hold_point;

static inline void get_corner(uint8_t idx, struct FloatVect2 *p);

static bool get_ray_edge_intersection(float px, float py, float dx, float dy,
                                      struct FloatVect2 *hit_point, uint8_t *hit_edge)
{
  if (hit_point == NULL || hit_edge == NULL) {
    return false;
  }

  float best_t = -1.f;
  struct FloatVect2 best_point = {0.f, 0.f};
  uint8_t best_edge = 0;

  for (uint8_t i = 0; i < 4; i++) {
    struct FloatVect2 a, b;
    get_corner(i, &a);
    get_corner(i + 1, &b);

    float sx = b.x - a.x;
    float sy = b.y - a.y;
    float rxs = dx * sy - dy * sx;
    if (fabsf(rxs) < 1e-6f) {
      continue;
    }

    float apx = a.x - px;
    float apy = a.y - py;
    float t = (apx * sy - apy * sx) / rxs;
    float u = (apx * dy - apy * dx) / rxs;

    if (t >= 0.f && u >= 0.f && u <= 1.f) {
      if (best_t < 0.f || t < best_t) {
        best_t = t;
        best_point.x = px + t * dx;
        best_point.y = py + t * dy;
        best_edge = i;
      }
    }
  }

  if (best_t < 0.f) {
    return false;
  }

  *hit_point = best_point;
  *hit_edge = best_edge;
  return true;
}

static bool get_closest_edge_point(float px, float py, struct FloatVect2 *closest_point, float *closest_dist)
{
  if (closest_point == NULL || closest_dist == NULL) {
    return false;
  }

  float best_dist_sq = -1.f;
  struct FloatVect2 best_point = {0.f, 0.f};

  for (uint8_t i = 0; i < 4; i++) {
    struct FloatVect2 a, b;
    get_corner(i, &a);
    get_corner(i + 1, &b);

    float abx = b.x - a.x;
    float aby = b.y - a.y;
    float len_sq = abx * abx + aby * aby;
    if (len_sq < 1e-6f) {
      continue;
    }

    float apx = px - a.x;
    float apy = py - a.y;
    float t = (apx * abx + apy * aby) / len_sq;
    Bound(t, 0.f, 1.f);

    float cx = a.x + t * abx;
    float cy = a.y + t * aby;
    float dx = px - cx;
    float dy = py - cy;
    float dist_sq = dx * dx + dy * dy;

    if (best_dist_sq < 0.f || dist_sq < best_dist_sq) {
      best_dist_sq = dist_sq;
      best_point.x = cx;
      best_point.y = cy;
    }
  }

  if (best_dist_sq < 0.f) {
    return false;
  }

  *closest_point = best_point;
  *closest_dist = sqrtf(best_dist_sq);
  return true;
}

// define the position of the four corners of the inner geofence
static inline void get_corner(uint8_t idx, struct FloatVect2 *p)
{
  switch (idx & 0x3) {
    case 0:
      p->x = WaypointX(WP__OZ1);
      p->y = WaypointY(WP__OZ1);
      break;
    case 1:
      p->x = WaypointX(WP__OZ2);
      p->y = WaypointY(WP__OZ2);
      break;
    case 2:
      p->x = WaypointX(WP__OZ3);
      p->y = WaypointY(WP__OZ3);
      break;
    default:
      p->x = WaypointX(WP__OZ4);
      p->y = WaypointY(WP__OZ4);
      break;
  }
}

void cyberzoo_perimeter_nav_init(void)
{
  current_edge = 0;
  current_dir.x = 0.f;
  current_dir.y = 1.f;
  dir_valid = false;
  path_hold_active = false;
  path_hold_point.x = 0.f;
  path_hold_point.y = 0.f;
}

bool cyberzoo_perimeter_get_path_direction(struct FloatVect2 *dir)
{
  if (!dir_valid || dir == NULL) {
    return false;
  }
  *dir = current_dir;
  return true;
}

void ProjectPathToEdge(void)
{
  float heading = nav.heading;
  float dx = sinf(heading);
  float dy = cosf(heading);

  struct FloatVect2 hit_point;
  uint8_t hit_edge = 0;
  if (!get_ray_edge_intersection(GetPosX(), GetPosY(), dx, dy, &hit_point, &hit_edge)) {
    return;
  }

  path_hold_point = hit_point;
  path_hold_active = true;

  current_edge = hit_edge & 0x3;
  struct FloatVect2 start, end;
  get_corner(current_edge, &start);
  get_corner(current_edge + 1, &end);
  float ex = end.x - start.x;
  float ey = end.y - start.y;
  float len = sqrtf(ex * ex + ey * ey);
  if (len >= 0.1f) {
    current_dir.x = ex / len;
    current_dir.y = ey / len;
    dir_valid = true;
  }

  waypoint_move_xy_i(WP_PATH, POS_BFP_OF_REAL(hit_point.x), POS_BFP_OF_REAL(hit_point.y));
}

void cyberzoo_perimeter_nav_periodic(void)
{
  if (path_hold_active) {
    float dx_hold = GetPosX() - path_hold_point.x;
    float dy_hold = GetPosY() - path_hold_point.y;
    float dist_hold = sqrtf(dx_hold * dx_hold + dy_hold * dy_hold);
    if (dist_hold > cz_perimeter_corner_radius) {
      waypoint_move_xy_i(WP_PATH, POS_BFP_OF_REAL(path_hold_point.x), POS_BFP_OF_REAL(path_hold_point.y));
      return;
    }
    path_hold_active = false;
  }

  struct FloatVect2 start, end;
  get_corner(current_edge, &start);
  get_corner(current_edge + 1, &end);

  float ex = end.x - start.x;
  float ey = end.y - start.y;
  float len = sqrtf(ex * ex + ey * ey);
  if (len < 0.1f) {
    dir_valid = false;
    return;
  }

  float ux = ex / len;
  float uy = ey / len;

  float px = GetPosX() - start.x;
  float py = GetPosY() - start.y;
  float s = px * ux + py * uy;

  // Advance edge either when close to the corner, or when we are within a threshold
  // of the edge end along track. This avoids getting stuck if an obstacle blocks the corner.
  float dx_end = end.x - GetPosX();
  float dy_end = end.y - GetPosY();
  float dist_end = sqrtf(dx_end * dx_end + dy_end * dy_end);
  bool close_to_corner = (dist_end < cz_perimeter_corner_radius);
  bool near_end_along_track = (s > (len - cz_perimeter_corner_radius));
  if (close_to_corner || near_end_along_track) {
    current_edge = (current_edge + 1) & 0x3;
    get_corner(current_edge, &start);
    get_corner(current_edge + 1, &end);
    ex = end.x - start.x;
    ey = end.y - start.y;
    len = sqrtf(ex * ex + ey * ey);
    if (len < 0.1f) {
      dir_valid = false;
      return;
    }
    ux = ex / len;
    uy = ey / len;
    px = GetPosX() - start.x;
    py = GetPosY() - start.y;
    s = px * ux + py * uy;
  }

  current_dir.x = ux;
  current_dir.y = uy;
  dir_valid = true;

  // Project vehicle on current edge and move lookahead point along that edge.
  float s_look = s + cz_perimeter_lookahead;
  Bound(s_look, 0.f, len);

  float tx = start.x + s_look * ux;
  float ty = start.y + s_look * uy;

  float dx_wp = WaypointX(WP_PATH) - GetPosX();
  float dy_wp = WaypointY(WP_PATH) - GetPosY();
  float dist_to_wp_path = sqrtf(dx_wp * dx_wp + dy_wp * dy_wp);

  struct FloatVect2 closest_point;
  float closest_dist = 0.f;
  if (navigation_state == REJOIN_PATH &&
      get_closest_edge_point(GetPosX(), GetPosY(), &closest_point, &closest_dist) &&
      dist_to_wp_path > closest_dist) {
    waypoint_move_xy_i(WP_PATH, POS_BFP_OF_REAL(closest_point.x), POS_BFP_OF_REAL(closest_point.y));
    return;
  }

  waypoint_move_xy_i(WP_PATH, POS_BFP_OF_REAL(tx), POS_BFP_OF_REAL(ty));
}
