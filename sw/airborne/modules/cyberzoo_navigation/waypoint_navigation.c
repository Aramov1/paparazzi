/*
 * Copyright (C) Roland Meertens
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/orange_avoider/waypoint_navigation.c"
 * @author Roland Meertens
 * Example on how to use the colours detected to avoid orange pole in the cyberzoo
 * This module is an example module for the course AE4317 Autonomous Flight of Micro Air Vehicles at the TU Delft.
 * This module is used in combination with a color filter (cv_detect_color_object) and the navigation mode of the autopilot.
 * The avoidance strategy is to simply count the total number of orange pixels. When above a certain percentage threshold,
 * (given by color_count_frac) we assume that there is an obstacle and we turn.
 *
 * The color filter settings are set using the cv_detect_color_object. This module can run multiple filters simultaneously
 * so you have to define which filter to use with the ORANGE_AVOIDER_VISUAL_DETECTION_ID setting.
 */

#include "modules/cyberzoo_navigation/waypoint_navigation.h"
#include "firmwares/rotorcraft/navigation.h"
#include "generated/airframe.h"
#include "state.h"
#include "modules/core/abi.h"
#include <time.h>
#include <stdio.h>
#include <math.h>

#include "generated/flight_plan.h"

#define ORANGE_AVOIDER_VERBOSE TRUE

#define PRINT(string,...) fprintf(stderr, "[orange_avoider->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if ORANGE_AVOIDER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

static uint8_t moveWaypointAlongHeading(uint8_t waypoint, float distanceMeters, float _heading);
static uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters);
static uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters);
static uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor);
static uint8_t increase_nav_heading(float incrementDegrees);
static uint8_t chooseAvoidanceHeadingIncrement(void);
static uint8_t setGoalToPathWaypoint(void);
static uint8_t getHeadingToPathWaypoint(float *heading_to_path);
static uint8_t setHeadingToPathWaypointLimited(float max_delta_deg);
static uint8_t isHeadingBlocked(float heading);
static uint8_t isRejoinGeometrySatisfied(void);
static uint8_t chooseEdgeAwareIncrement(float *increment_deg);
static uint8_t getClosestInnerEdgeInward(float *inward_x, float *inward_y, float *distance_to_edge);
static float angle_diff(float a, float b);
static float clampf(float v, float lo, float hi);

enum navigation_state_t {
  SAFE,
  OBSTACLE_FOUND,
  SEARCH_FOR_SAFE_HEADING,
  REJOIN_PATH,
  OUT_OF_BOUNDS
};

/* currently, using waypoint_navigation.c, the drone avoids abstacles at the same time as following the perimeter of the cyberzoo, however, when it reaches the SEARCH_FOR_SAFE_HEADING state, it keeps moving slightly forward at the same time as it turn to find a safe heading, and will bump into the obstacle, make it such that it will move faster out of the way of the obstacle */

// define settings
float oa_color_count_frac = 0.18f;       // detect threshold (orange fraction)
float oa_clear_color_count_frac = 0.18f; // clear threshold (hysteresis)
float oa_safe_max_speed = 0.3f;          // low cruise speed [m/s] for cautious testing
float oa_stop_max_speed = 0.0f;          // commanded max speed on obstacle detection [m/s]
float oa_heading_slew_deg = 20.f;         // max heading change per cycle when tracking path [deg]
float oa_rejoin_hold_time_s = 1.5f;      // minimum time in rejoin mode before SAFE [s]
float oa_rejoin_corridor_width_m = 0.8f; // max cross-track error to rejoin line [m]
float oa_rejoin_heading_error_deg = 20.f; // max heading error to path before SAFE [deg]
float oa_rejoin_probe_distance_m = 1.2f; // short probe ahead toward path for arena safety [m]
float oa_blocked_sector_half_angle_deg = 35.f; // forbidden return sector half-angle [deg]
float oa_blocked_sector_time_s = 4.0f;   // blocked-sector memory duration [s]
int16_t oa_rejoin_clear_confidence = 8;  // clear samples needed before safe rejoin
float oa_inner_edge_margin_m = 0.4f;     // apply edge-aware turn selection when closer than this to inner geofence edge [m]
int16_t oa_rejoin_forward_cycles = 4;    // keep following safe WP_TRAJECTORY this many REJOIN_PATH cycles before path targetting

// define and initialise global variables
enum navigation_state_t navigation_state = SEARCH_FOR_SAFE_HEADING;
int32_t color_count = 0;               // orange color count from color filter for obstacle detection
int16_t obstacle_free_confidence = 0;  // certainty that forward direction is safe
float heading_increment = 15.f;         // heading angle increment [deg]
int16_t rejoin_counter = 0;            // cycles spent in REJOIN_PATH
float rejoin_start_x = 0.f;            // position where avoidance started
float rejoin_start_y = 0.f;
float blocked_heading_center = 0.f;    // center of recently blocked heading sector
int16_t blocked_heading_counter = 0;   // cycles left in blocked heading memory
uint8_t edge_turn_bias_active = false; // when true, keep turning away from geofence edge
float edge_turn_bias_sign = 1.f;       // +1 or -1 turn sign bias when edge_turn_bias_active

/*
 * This next section defines an ABI messaging event (http://wiki.paparazziuav.org/wiki/ABI), necessary
 * any time data calculated in another module needs to be accessed. Including the file where this external
 * data is defined is not enough, since modules are executed parallel to each other, at different frequencies,
 * in different threads. The ABI event is triggered every time new data is sent out, and as such the function
 * defined in this file does not need to be explicitly called, only bound in the init function
 */
#ifndef ORANGE_AVOIDER_VISUAL_DETECTION_ID
#define ORANGE_AVOIDER_VISUAL_DETECTION_ID ABI_BROADCAST
#endif
static int16_t sensor_turn_vote = 0;  // turn hint from obstacle sensor fusion

static abi_event color_detection_ev;
static void color_detection_cb(uint8_t __attribute__((unused)) sender_id,
                               int16_t pixel_x, int16_t __attribute__((unused)) pixel_y,
                               int16_t __attribute__((unused)) pixel_width, int16_t __attribute__((unused)) pixel_height,
                               int32_t quality, int16_t __attribute__((unused)) extra)
{
  color_count = quality;
  sensor_turn_vote = pixel_x;
}

/*
 * Initialisation function, setting the colour filter, random seed and heading_increment
 */
void orange_avoider_init(void)
{
  srand(time(NULL));
  chooseAvoidanceHeadingIncrement();
  NavSetMaxSpeed(oa_safe_max_speed);

  // bind our colorfilter callbacks to receive the color filter outputs
  AbiBindMsgVISUAL_DETECTION(ORANGE_AVOIDER_VISUAL_DETECTION_ID, &color_detection_ev, color_detection_cb);
}

/*
 * Function that checks it is safe to move forwards, and then moves a waypoint forward or changes the heading
 */
void orange_avoider_periodic(void)
{
  const float periodic_hz = 4.0f; // module configured at 4 Hz
  const int16_t rejoin_hold_cycles = (int16_t)(oa_rejoin_hold_time_s * periodic_hz);
  const int16_t blocked_sector_cycles = (int16_t)(oa_blocked_sector_time_s * periodic_hz);

  // only evaluate our state machine if we are flying
  if (!autopilot_in_flight()) {
    return;
  }

  int32_t color_count_threshold = oa_color_count_frac * front_camera.output_size.w * front_camera.output_size.h;
  int32_t clear_color_count_threshold = oa_clear_color_count_frac * front_camera.output_size.w * front_camera.output_size.h;

  VERBOSE_PRINT("Color_count: %d detect_thr: %d clear_thr: %d state: %d\n",
                color_count, color_count_threshold, clear_color_count_threshold, navigation_state);

  // Hysteresis for obstacle confidence
  if (color_count <= clear_color_count_threshold) {
    obstacle_free_confidence++;
  } else if (color_count >= color_count_threshold) {
    obstacle_free_confidence -= 2;
  } else {
    obstacle_free_confidence--;
  }
  Bound(obstacle_free_confidence, 0, oa_rejoin_clear_confidence);

  if (blocked_heading_counter > 0) {
    blocked_heading_counter--;
  }

  NavSetMaxSpeed(oa_safe_max_speed);  // set maximum speed of bebop

  switch (navigation_state) {
    case SAFE:
      setGoalToPathWaypoint();
      setHeadingToPathWaypointLimited(oa_heading_slew_deg);

      if (color_count >= color_count_threshold || obstacle_free_confidence == 0) {
        navigation_state = OBSTACLE_FOUND;
      }
      break;

    case OBSTACLE_FOUND:
      waypoint_move_here_2d(WP_TRAJECTORY);
      waypoint_move_here_2d(WP_GOAL);
      chooseAvoidanceHeadingIncrement();

      rejoin_start_x = stateGetPositionEnu_f()->x;
      rejoin_start_y = stateGetPositionEnu_f()->y;
      rejoin_counter = 0;

      blocked_heading_center = stateGetNedToBodyEulers_f()->psi;
      FLOAT_ANGLE_NORMALIZE(blocked_heading_center);
      blocked_heading_counter = blocked_sector_cycles;

      obstacle_free_confidence = 0;
      navigation_state = SEARCH_FOR_SAFE_HEADING;
      break;

    case SEARCH_FOR_SAFE_HEADING:
      // Once a candidate safe heading appears (confidence > 0), stop turning and keep testing straight ahead.
      // chooseAvoidanceHeadingIncrement();
      if (obstacle_free_confidence == 0) {
        increase_nav_heading(heading_increment);
      }

      // move test waypoint (WP_TRAJECTORY) ahead and check if it falls out of bounds
      moveWaypointForward(WP_TRAJECTORY, 0.7f);
      if (!InsideObstacleZone(WaypointX(WP_TRAJECTORY), WaypointY(WP_TRAJECTORY))) {
        navigation_state = OUT_OF_BOUNDS;
        break;
      }

      // move in direction of test waypoint if it does not fall out of bounds
      waypoint_move_xy_i(WP_GOAL, POS_BFP_OF_REAL(WaypointX(WP_TRAJECTORY)),
                         POS_BFP_OF_REAL(WaypointY(WP_TRAJECTORY)));

      if (obstacle_free_confidence >= oa_rejoin_clear_confidence) {
        navigation_state = REJOIN_PATH;
        rejoin_counter = 0;
      }
      break;

    case REJOIN_PATH: {
      float path_heading;
      float safe_heading;
      rejoin_counter++;

      // First keep moving along already-safe local trajectory for a few cycles before steering to WP_PATH.
      if (rejoin_counter <= oa_rejoin_forward_cycles) {
        VERBOSE_PRINT("keeping on same trajectory");
        moveWaypointForward(WP_TRAJECTORY, 0.7f);
        waypoint_move_xy_i(WP_GOAL, POS_BFP_OF_REAL(WaypointX(WP_TRAJECTORY)),
                           POS_BFP_OF_REAL(WaypointY(WP_TRAJECTORY)));

        safe_heading = nav.heading;

        // check that 'safe' local trajectory won't go out of bounds
        if (!InsideObstacleZone(WaypointX(WP_TRAJECTORY), WaypointY(WP_TRAJECTORY))) {
          // navigation_state = OUT_OF_BOUNDS;
          break;
        }

        if (color_count >= color_count_threshold) {
          navigation_state = OBSTACLE_FOUND;
          break;
        }
        break;
      }

      // getHeadingToPathWaypoint(&path_heading);
      /*if (isHeadingBlocked(path_heading)) {
        increase_nav_heading(heading_increment);
        moveWaypointForward(WP_TRAJECTORY, 0.7f);
        waypoint_move_xy_i(WP_GOAL, POS_BFP_OF_REAL(WaypointX(WP_TRAJECTORY)),
                           POS_BFP_OF_REAL(WaypointY(WP_TRAJECTORY)));
        if (!InsideObstacleZone(WaypointX(WP_TRAJECTORY), WaypointY(WP_TRAJECTORY))) {
          // navigation_state = OUT_OF_BOUNDS;
        }
        break;
      }*/

      setGoalToPathWaypoint();
      setHeadingToPathWaypointLimited(oa_heading_slew_deg);

      if (color_count >= color_count_threshold) {
        VERBOSE_PRINT("detected obstacle and returning to last safe heading");
        // return to original heading
        moveWaypointAlongHeading(WP_TRAJECTORY, 0.7f, safe_heading);
        waypoint_move_xy_i(WP_GOAL, POS_BFP_OF_REAL(WaypointX(WP_TRAJECTORY)),
                           POS_BFP_OF_REAL(WaypointY(WP_TRAJECTORY)));


        // navigation_state = OBSTACLE_FOUND;
        break;
      }

      if (rejoin_counter >= rejoin_hold_cycles &&
          obstacle_free_confidence >= oa_rejoin_clear_confidence &&
          isRejoinGeometrySatisfied()) {
        navigation_state = SAFE;
      }
      break;
    }

    case OUT_OF_BOUNDS:
      increase_nav_heading(heading_increment);
      moveWaypointForward(WP_TRAJECTORY, 0.7f);
      waypoint_move_xy_i(WP_GOAL, POS_BFP_OF_REAL(WaypointX(WP_TRAJECTORY)),
                         POS_BFP_OF_REAL(WaypointY(WP_TRAJECTORY)));

      if (InsideObstacleZone(WaypointX(WP_TRAJECTORY), WaypointY(WP_TRAJECTORY))) {
        increase_nav_heading(heading_increment);
        obstacle_free_confidence = 0;
        navigation_state = SEARCH_FOR_SAFE_HEADING;
      }
      break;

    default:
      break;
  }
}

/*
 * Increases the NAV heading. Assumes heading is an INT32_ANGLE. It is bound in this function.
 */
uint8_t increase_nav_heading(float incrementDegrees)
{
  float signed_increment_deg = incrementDegrees;

  // Keep the turn away from inner-geofence edge while we remain near that edge.
  if (edge_turn_bias_active) {
    float inward_x, inward_y, edge_dist;
    if (!getClosestInnerEdgeInward(&inward_x, &inward_y, &edge_dist) || edge_dist > oa_inner_edge_margin_m * 1.5f) {
      edge_turn_bias_active = false;
    } else {
      signed_increment_deg = fabsf(incrementDegrees) * edge_turn_bias_sign;
    }
  }

  float new_heading = nav.heading + RadOfDeg(signed_increment_deg);
  FLOAT_ANGLE_NORMALIZE(new_heading);
  nav.heading = new_heading;

  VERBOSE_PRINT("Increasing heading to %f\n", DegOfRad(new_heading));
  return false;
}

/*
 * Calculates coordinates of distance forward and sets waypoint 'waypoint' to those coordinates
 */
uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters)
{
  struct EnuCoor_i new_coor;
  calculateForwards(&new_coor, distanceMeters);
  moveWaypoint(waypoint, &new_coor);
  return false;
}

/*
 * Calculates coordinates of a distance of 'distanceMeters' forward w.r.t. current position and heading
 */
uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters)
{
  float heading = nav.heading;

  new_coor->x = stateGetPositionEnu_i()->x + POS_BFP_OF_REAL(sinf(heading) * distanceMeters);
  new_coor->y = stateGetPositionEnu_i()->y + POS_BFP_OF_REAL(cosf(heading) * distanceMeters);
  return false;
}

uint8_t moveWaypointAlongHeading(uint8_t waypoint, float distanceMeters, float _heading) {
  struct EnuCoor_i new_coor;
  new_coor.x = stateGetPositionEnu_i()->x + POS_BFP_OF_REAL(sinf(_heading) * distanceMeters);
  new_coor.y = stateGetPositionEnu_i()->y + POS_BFP_OF_REAL(cosf(_heading) * distanceMeters);
  moveWaypoint(waypoint, &new_coor);
  return false;
}

/*
 * Sets WP_GOAL to the current nominal WP_PATH target.
 */
uint8_t setGoalToPathWaypoint(void)
{
  waypoint_move_xy_i(WP_GOAL, POS_BFP_OF_REAL(WaypointX(WP_PATH)), POS_BFP_OF_REAL(WaypointY(WP_PATH)));
  return false;
}

/*
 * Compute desired heading toward WP_PATH from current position.
 */
uint8_t getHeadingToPathWaypoint(float *heading_to_path)
{
  float dx = WaypointX(WP_PATH) - stateGetPositionEnu_f()->x;
  float dy = WaypointY(WP_PATH) - stateGetPositionEnu_f()->y;

  if ((dx * dx + dy * dy) < 0.01f) {
    *heading_to_path = nav.heading;
    return false;
  }

  *heading_to_path = atan2f(dx, dy);
  FLOAT_ANGLE_NORMALIZE(*heading_to_path);
  return false;
}

/*
 * Align heading toward WP_PATH with a slew-rate limit and blocked-sector check.
 */
uint8_t setHeadingToPathWaypointLimited(float max_delta_deg)
{
  float desired_heading;
  getHeadingToPathWaypoint(&desired_heading);

  if (isHeadingBlocked(desired_heading)) {
    float bypass = blocked_heading_center +
                   copysignf(RadOfDeg(oa_blocked_sector_half_angle_deg + 10.f), heading_increment);
    FLOAT_ANGLE_NORMALIZE(bypass);
    desired_heading = bypass;
  }

  float heading_error = angle_diff(desired_heading, nav.heading);
  float max_delta = RadOfDeg(max_delta_deg);
  float limited_delta = clampf(heading_error, -max_delta, max_delta);

  float new_heading = nav.heading + limited_delta;
  FLOAT_ANGLE_NORMALIZE(new_heading);
  nav.heading = new_heading;
  return false;
}

/*
 * Check whether heading lies inside the remembered blocked return sector.
 */
uint8_t isHeadingBlocked(float heading)
{
  if (blocked_heading_counter <= 0) {
    return false;
  }

  float d = fabsf(angle_diff(heading, blocked_heading_center));
  return d <= RadOfDeg(oa_blocked_sector_half_angle_deg);
}

/*
 * Geometric gate for path rejoin:
 * 1) Current position must be close to the line from avoidance start to WP_PATH.
 * 2) Heading error to path target must be small.
 * 3) Short forward probe toward WP_PATH must remain inside arena bounds.
 */
uint8_t isRejoinGeometrySatisfied(void)
{
  float cx = stateGetPositionEnu_f()->x;
  float cy = stateGetPositionEnu_f()->y;
  float ax = rejoin_start_x;
  float ay = rejoin_start_y;
  float bx = WaypointX(WP_PATH);
  float by = WaypointY(WP_PATH);

  float abx = bx - ax;
  float aby = by - ay;
  float acx = cx - ax;
  float acy = cy - ay;
  float ab_norm_sq = abx * abx + aby * aby;

  float cross_track_dist = 0.f;
  if (ab_norm_sq > 0.04f) {
    float t = clampf((acx * abx + acy * aby) / ab_norm_sq, 0.f, 1.f);
    float px = ax + t * abx;
    float py = ay + t * aby;
    float ex = cx - px;
    float ey = cy - py;
    cross_track_dist = sqrtf(ex * ex + ey * ey);
  }

  float path_heading;
  getHeadingToPathWaypoint(&path_heading);
  float heading_err = fabsf(angle_diff(path_heading, nav.heading));

  float probe_x = cx + sinf(path_heading) * oa_rejoin_probe_distance_m;
  float probe_y = cy + cosf(path_heading) * oa_rejoin_probe_distance_m;

  return (cross_track_dist <= oa_rejoin_corridor_width_m) &&
         (heading_err <= RadOfDeg(oa_rejoin_heading_error_deg)) &&
         InsideObstacleZone(probe_x, probe_y);
}

float angle_diff(float a, float b)
{
  float d = a - b;
  FLOAT_ANGLE_NORMALIZE(d);
  return d;
}

float clampf(float v, float lo, float hi)
{
  if (v < lo) {
    return lo;
  }
  if (v > hi) {
    return hi;
  }
  return v;
}

/*
 * Sets waypoint 'waypoint' to the coordinates of 'new_coor'
 */
uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor)
{
  VERBOSE_PRINT("Moving waypoint %d to x:%f y:%f\n", waypoint, POS_FLOAT_OF_BFP(new_coor->x),
                POS_FLOAT_OF_BFP(new_coor->y));
  waypoint_move_xy_i(waypoint, new_coor->x, new_coor->y);
  return false;
}

/*
 * Sets the variable 'heading_increment'
 */
uint8_t chooseAvoidanceHeadingIncrement(void)
{
  float selected_increment = 0.f;

  if (chooseEdgeAwareIncrement(&selected_increment)) {
    heading_increment = selected_increment;
    edge_turn_bias_active = true;
    edge_turn_bias_sign = (heading_increment >= 0.f) ? 1.f : -1.f;
    VERBOSE_PRINT("Edge-aware avoidance increment: %f\n", heading_increment);
    return false;
  }

  edge_turn_bias_active = false;

  if (sensor_turn_vote != 0) {
    // Sensor has a directional opinion — obstacle safety takes priority over path preference
    heading_increment = (sensor_turn_vote > 0) ? fabsf(heading_increment) : -fabsf(heading_increment);
    VERBOSE_PRINT("Sensor-guided turn: vote=%d increment=%f\n", sensor_turn_vote, heading_increment);
  } else {
    // No sensor directional signal — fall back to turning toward path
    float path_heading;
    getHeadingToPathWaypoint(&path_heading);
    float diff = angle_diff(path_heading, nav.heading);
    if (diff > 0.f) {
      heading_increment = fabsf(heading_increment);
    } else {
      heading_increment = -fabsf(heading_increment);
    }
    VERBOSE_PRINT("Turn closest to PATH with increment: %f\n", heading_increment);
  }
  return false;
}

/*
 * If close to inner geofence edge, choose turn direction away from that edge.
 * Returns true when edge-aware increment has been selected.
 */
uint8_t chooseEdgeAwareIncrement(float *increment_deg)
{
  float inward_x, inward_y, edge_dist;
  if (!getClosestInnerEdgeInward(&inward_x, &inward_y, &edge_dist)) {
    return false;
  }

  if (edge_dist > oa_inner_edge_margin_m) {
    return false;
  }

  // Forward vector in ENU from current heading convention.
  float fx = sinf(nav.heading);
  float fy = cosf(nav.heading);

  // If inward normal is on the left of forward, geofence side is on left -> force opposite turn.
  float cross = fx * inward_y - fy * inward_x;
  if (cross > 0.f) {
    *increment_deg = -fabsf(heading_increment);
  } else {
    *increment_deg = fabsf(heading_increment);
  }
  return true;
}

/*
 * Compute inward unit normal and distance for the closest edge of the inner geofence polygon.
 * The inward direction is chosen using polygon centroid.
 */
uint8_t getClosestInnerEdgeInward(float *inward_x, float *inward_y, float *distance_to_edge)
{
#if defined(WP__OZ1) && defined(WP__OZ2) && defined(WP__OZ3) && defined(WP__OZ4)
  float px[4] = {
    WaypointX(WP__OZ1), WaypointX(WP__OZ2), WaypointX(WP__OZ3), WaypointX(WP__OZ4)
  };
  float py[4] = {
    WaypointY(WP__OZ1), WaypointY(WP__OZ2), WaypointY(WP__OZ3), WaypointY(WP__OZ4)
  };

  float cx_poly = 0.f;
  float cy_poly = 0.f;
  for (int i = 0; i < 4; i++) {
    cx_poly += px[i];
    cy_poly += py[i];
  }
  cx_poly *= 0.25f;
  cy_poly *= 0.25f;

  float cx = stateGetPositionEnu_f()->x;
  float cy = stateGetPositionEnu_f()->y;

  float best_dist = 1e9f;
  float best_inx = 0.f;
  float best_iny = 0.f;

  for (int i = 0; i < 4; i++) {
    int j = (i + 1) & 0x3;
    float ex = px[j] - px[i];
    float ey = py[j] - py[i];
    float el2 = ex * ex + ey * ey;
    if (el2 < 1e-6f) {
      continue;
    }

    float vx = cx - px[i];
    float vy = cy - py[i];
    float t = clampf((vx * ex + vy * ey) / el2, 0.f, 1.f);
    float qx = px[i] + t * ex;
    float qy = py[i] + t * ey;
    float dx = cx - qx;
    float dy = cy - qy;
    float dist = sqrtf(dx * dx + dy * dy);

    if (dist < best_dist) {
      best_dist = dist;

      // One of the two normals, later oriented toward polygon centroid (inward).
      float nx = -ey;
      float ny = ex;
      float nlen = sqrtf(nx * nx + ny * ny);
      if (nlen > 1e-6f) {
        nx /= nlen;
        ny /= nlen;
      } else {
        nx = 0.f;
        ny = 0.f;
      }

      float mx = 0.5f * (px[i] + px[j]);
      float my = 0.5f * (py[i] + py[j]);
      float to_cx = cx_poly - mx;
      float to_cy = cy_poly - my;
      if ((nx * to_cx + ny * to_cy) < 0.f) {
        nx = -nx;
        ny = -ny;
      }

      best_inx = nx;
      best_iny = ny;
    }
  }

  if (best_dist >= 1e8f) {
    return false;
  }

  *inward_x = best_inx;
  *inward_y = best_iny;
  *distance_to_edge = best_dist;
  return true;
#else
  (void)inward_x;
  (void)inward_y;
  (void)distance_to_edge;
  return false;
#endif
}


/*
 * when exiting SEARCH_FOR_SAFE_HEADING, project a WP_PATH_TEST to the edge which the drone is looking at, ensuring that
 * there is 'no orange' in the centre of the screen, then go to REJOIN_PATH state and if orange is immediately detected,
 * move WP_PATH to WP_PATH_TEST
 **/