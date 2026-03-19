/*
 * Copyright (C) 2026
 *
 * This file is part of paparazzi.
 */

#include "modules/cyberzoo_navigation/cyberzoo_perimeter_waypoints.h"

#include "firmwares/rotorcraft/navigation.h"
#include "generated/flight_plan.h"
#include "state.h"
#include <math.h>

float cz_perimeter_lookahead = 1.2f;
float cz_perimeter_corner_radius = 1.f;

static uint8_t current_edge;
static struct FloatVect2 current_dir;
static bool dir_valid;

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
}

bool cyberzoo_perimeter_get_path_direction(struct FloatVect2 *dir)
{
  if (!dir_valid || dir == NULL) {
    return false;
  }
  *dir = current_dir;
  return true;
}

void cyberzoo_perimeter_nav_periodic(void)
{
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
  waypoint_move_xy_i(WP_PATH, POS_BFP_OF_REAL(tx), POS_BFP_OF_REAL(ty));
}
