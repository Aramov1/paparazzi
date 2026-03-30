
/**
 * @file "modules/cyberzoo_navigation/waypoint_navigation.c"
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
#include "modules/gate_navigator/gate_nav.h"
#include "firmwares/rotorcraft/navigation.h"
#include "generated/airframe.h"
#include "state.h"
#include "modules/core/abi.h"
#include <time.h>
#include <stdio.h>
#include <math.h>

#include "generated/flight_plan.h"

#define NAVIGATOR_VERBOSE TRUE

#define PRINT(string,...) fprintf(stderr, "[orange_avoider->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if NAVIGATOR_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

static uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters);
static uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters);
static uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor);
static uint8_t increase_nav_heading(float incrementDegrees);
static uint8_t chooseAvoidanceHeadingIncrement(void);
static uint8_t setGoalToPathWaypoint(void);
static uint8_t getHeadingToPathWaypoint(float *heading_to_path);
static uint8_t setHeadingToPathWaypointLimited(float max_delta_deg);
static uint8_t chooseEdgeAwareIncrement(float *increment_deg);
static uint8_t getClosestInnerEdgeInward(float *inward_x, float *inward_y, float *distance_to_edge);
static float angle_diff(float a, float b);
static float clampf(float v, float lo, float hi);

// define settings
#ifndef NAV_PROGRAM_MODE
#define NAV_PROGRAM_MODE 2
#endif
int16_t nav_program_mode = NAV_PROGRAM_MODE;  // 0=SimpleReactive 1=WaypointMachine 2=Perimeter
float safe_max_speed = 0.5f;         	// low cruise speed [m/s] for cautious testing
float obstacle_max_speed = 0.2f;        // commanded max speed on obstacle detection [m/s]
float heading_slew_deg = 45.f;         	// max heading change per cycle when tracking path [deg]
int16_t cycles_until_rejoin_path = 4;  	// clear samples needed before safe rejoin
float inner_edge_margin_m = 0.4f;     	// apply edge-aware turn selection when closer than this to inner geofence edge [m]

// define and initialise global variables
enum navigation_state_t navigation_state = SEARCH_FOR_SAFE_HEADING;
float heading_increment = 15.f;         // heading angle increment [deg]
int16_t rejoin_counter = 0;            	// cycles spent in REJOIN_PATH
uint8_t edge_turn_bias_active = false; 	// when true, keep turning away from geofence edge
float edge_turn_bias_sign = 1.f;       	// +1 or -1 turn sign bias when edge_turn_bias_active
uint8_t obstacle_detected = 0;          // boolean to indicate obstacle detection from vision
int16_t obstacle_free_confidence = 0;   // confidence counter for consecutive obstacle-free detections
uint8_t orange_detected = 0;            // boolean to indicate orange detection from vision
uint8_t nav_state_is_rejoin_path(void); // function to check if the navigation state is REJOIN_PATH


// --- SimpleReactive mode (nav_program_mode == 0) private state ---
// Mirrors old_logic.c exactly. Uses color_count / sensor_turn_vote from ABI.
enum sr_nav_state_t { SR_SAFE, SR_OBSTACLE_FOUND, SR_SEARCH, SR_OUT_OF_BOUNDS };
static enum sr_nav_state_t sr_navigation_state = SR_SAFE;
static float   sr_heading_increment = 5.f;
static int16_t sr_confidence        = 0;


#ifndef WAYPOINT_NAVIGATION_ORANGE_DETECTION_ID
#define WAYPOINT_NAVIGATION_ORANGE_DETECTION_ID ABI_BROADCAST
#endif

static int16_t sensor_turn_vote = 0;  // turn hint from obstacle sensor fusion (indicates direction in which it is better to turn)

static abi_event visual_detection_ev;
static void visual_detection_cb(uint8_t __attribute__((unused)) sender_id,
                                int16_t  pixel_x,
                                int16_t  __attribute__((unused)) pixel_y,
                                int16_t  __attribute__((unused)) pixel_width,
                                int16_t  __attribute__((unused)) pixel_height,
                                int32_t  quality,
                                int16_t  __attribute__((unused)) extra)
{
  obstacle_detected = (quality > 0) ? 1 : 0;
  sensor_turn_vote  = pixel_x;
  if (!obstacle_detected) {
    obstacle_free_confidence++;
  } else {
    obstacle_free_confidence = (obstacle_free_confidence >= 3) ? obstacle_free_confidence - 3 : 0;
  }
  Bound(obstacle_free_confidence, 0, cycles_until_rejoin_path + 4);
}


// Initialisation function, setting the colour filter, random seed and heading_increment
void waypoint_navigation_init(void)
{
  srand(time(NULL));
  chooseAvoidanceHeadingIncrement();
  NavSetMaxSpeed(safe_max_speed);

  // bind obstacle_avoider ABI to receive fused detection output
  AbiBindMsgVISUAL_DETECTION(WAYPOINT_NAVIGATION_ORANGE_DETECTION_ID, &visual_detection_ev, visual_detection_cb);
}


/*
 * SimpleReactive navigation (nav_program_mode == 0).
 * Direct port of old_logic.c state machine. Uses color_count and sensor_turn_vote
 * from the ABI callback (populated by obstacle_avoider.c at 20 Hz).
 */
static void simple_reactive_periodic(void)
{
  const int16_t SR_MAX_CONFIDENCE = 5;
  const float   SR_MAX_DISTANCE   = 2.25f;

  // Confidence: +1 when clear, -3 when obstacle (matches old_logic.c)
  if (orange_detected == 0) {
    sr_confidence++;
  } else {
    sr_confidence -= 3;
  }
  Bound(sr_confidence, 0, SR_MAX_CONFIDENCE);

  float moveDistance = fminf(SR_MAX_DISTANCE, 0.2f * sr_confidence);

  // Choose turn direction only while in SAFE (matches old_logic.c timing)
  if (sr_navigation_state == SR_SAFE) {
    if (sensor_turn_vote == 0) {
      sr_heading_increment = (rand() % 2 == 0) ? 5.f : -5.f;
    } else {
      sr_heading_increment = (sensor_turn_vote > 0) ? 5.f : -5.f;
    }
  }

  switch (sr_navigation_state) {
    case SR_SAFE:
      moveWaypointForward(WP_TRAJECTORY, 1.5f * moveDistance);
      if (!InsideObstacleZone(WaypointX(WP_TRAJECTORY), WaypointY(WP_TRAJECTORY))) {
        sr_navigation_state = SR_OUT_OF_BOUNDS;
      } else if (sr_confidence == 0) {
        sr_navigation_state = SR_OBSTACLE_FOUND;
      } else {
        moveWaypointForward(WP_GOAL, moveDistance);
      }
      break;

    case SR_OBSTACLE_FOUND:
      waypoint_move_here_2d(WP_GOAL);
      waypoint_move_here_2d(WP_TRAJECTORY);
      sr_navigation_state = SR_SEARCH;
      break;

    case SR_SEARCH:
      increase_nav_heading(sr_heading_increment);
      if (sr_confidence >= 2) {
        sr_navigation_state = SR_SAFE;
      }
      break;

    case SR_OUT_OF_BOUNDS:
      increase_nav_heading(sr_heading_increment);
      moveWaypointForward(WP_TRAJECTORY, 1.5f);
      waypoint_move_here_2d(WP_GOAL);
      if (InsideObstacleZone(WaypointX(WP_TRAJECTORY), WaypointY(WP_TRAJECTORY))) {
        sr_confidence = 0;
        sr_navigation_state = SR_SEARCH;
      }
      break;
  }

  // Export state for OA_STATUS telemetry (nav_state field)
  navigation_state = (sr_navigation_state == SR_SAFE)         ? SAFE :
                     (sr_navigation_state == SR_SEARCH)        ? SEARCH_FOR_SAFE_HEADING :
                     (sr_navigation_state == SR_OUT_OF_BOUNDS) ? OUT_OF_BOUNDS :
                                                                  OBSTACLE_FOUND;
  obstacle_free_confidence = sr_confidence;
}


// Function that checks it is safe to move forwards, and then moves a waypoint forward or changes the heading
void waypoint_navigation_periodic(void)
{
  // only evaluate our state machine if we are flying
  if (!autopilot_in_flight()) {
    return;
  }

  // --- Gate detection priority: checked before ALL other logic ---

  // Entry: gate detected from any state except during obstacle avoidance
  if (gate_tracking
      && navigation_state != GATE_TRACKING
      && navigation_state != OBSTACLE_FOUND
      && navigation_state != SEARCH_FOR_SAFE_HEADING) {
    navigation_state = GATE_TRACKING;
    printf("[WAY_NAV] -> GATE_TRACKING\n");
  }

  // Exit: gate lost/crossed — return to path (gate_nav already back in passive SEARCH)
  if (!gate_tracking && navigation_state == GATE_TRACKING) {
    rejoin_counter   = 0;
    navigation_state = REJOIN_PATH;
    printf("[WAY_NAV] GATE_TRACKING -> REJOIN_PATH (gate done/lost)\n");
  }

  // Mode 0: SimpleReactive — gate_nav has full control while tracking
  if (nav_program_mode == 0) {
    NavSetMaxSpeed(safe_max_speed);
    if (gate_tracking) return;  // gate_nav handles heading + waypoints
    simple_reactive_periodic();
    return;
  }

  switch (navigation_state) {
    // SAFE state: general navigation behavior while checking for obstacles and bounds violations
    case SAFE:
      // set maximum speed of bebop
      NavSetMaxSpeed(safe_max_speed);

      if (nav_program_mode == 2) {
        // Mode 2: Perimeter — track WP_PATH set by cyberzoo_perimeter_waypoints
        setGoalToPathWaypoint();
        setHeadingToPathWaypointLimited(heading_slew_deg);
      } else {
        // Mode 1: WaypointMachine — full state machine, cruise forward freely
        moveWaypointForward(WP_TRAJECTORY, 0.5f);
        if (!InsideObstacleZone(WaypointX(WP_TRAJECTORY), WaypointY(WP_TRAJECTORY))) {
          navigation_state = OUT_OF_BOUNDS;
          break;
        }
        moveWaypointForward(WP_GOAL, 0.3f);
      }

      // check whether obstacle is detected. If so, change to OBSTACLE_FOUND state
      if (obstacle_detected) {
        navigation_state = OBSTACLE_FOUND;
      }
      break;

    // OBSTACLE_FOUND state: reduce speed to obstacle_max_speed and then choose safe heading.
    // When the obstacle is no longer detected, switch to REJOIN_PATH 
    case OBSTACLE_FOUND:
      NavSetMaxSpeed(obstacle_max_speed);
      waypoint_move_here_2d(WP_TRAJECTORY);
      waypoint_move_here_2d(WP_GOAL);
      chooseAvoidanceHeadingIncrement();

      rejoin_counter = 0;

      navigation_state = SEARCH_FOR_SAFE_HEADING;
      break;

    // SEARCH_FOR_SAFE_HEADING state: turn in place until no obstacle is detected.
    // Then move forward and check if we can rejoin path or if we are out of bounds.
    case SEARCH_FOR_SAFE_HEADING:
      // Increase heading unill safe heading is found.
      // Once a candidate safe heading appears (confidence > 0), stop turning and keep testing straight ahead.
      if (obstacle_detected) {
        increase_nav_heading(heading_increment);
      }

      // if current heading is safe, move test waypoint (WP_TRAJECTORY) ahead and check if it falls out of bounds
      moveWaypointForward(WP_TRAJECTORY, 0.7f);
      if (!InsideObstacleZone(WaypointX(WP_TRAJECTORY), WaypointY(WP_TRAJECTORY))) {
        navigation_state = OUT_OF_BOUNDS;
        break;
      }

      // move in direction of test waypoint if it does not fall out of bounds
      waypoint_move_xy_i(WP_GOAL, POS_BFP_OF_REAL(WaypointX(WP_TRAJECTORY)),
                         POS_BFP_OF_REAL(WaypointY(WP_TRAJECTORY)));

      // follow candidate/safe path for certain amount of cycles, then rejoin path
      if (obstacle_free_confidence >= cycles_until_rejoin_path) {
        navigation_state = REJOIN_PATH;
        rejoin_counter = 0;
      }
      break;

    // REJOIN_PATH state: move towards WP_PATH and face it. After certain amount of cycles that let the drone
    // get closer enough to the path, switch to SAFE
    case REJOIN_PATH: {
      NavSetMaxSpeed(safe_max_speed);

      rejoin_counter++;

      // 1. check for obstacles in view
      // 2. calculate new WP_PATH using function from cyberzoo_perimeter_waypoints
      // 3. move WP_GOAL to WP_PATH and ensure to face WP_PATH (should be already the case)
      // 4. return to SAFE

      if (obstacle_detected) {
        navigation_state = OBSTACLE_FOUND;
        break;
      }

      ProjectPathToEdge();
      if (nav_program_mode == 2) {
        // Mode 2: Perimeter — steer back toward WP_PATH
        setGoalToPathWaypoint();
        setHeadingToPathWaypointLimited(heading_slew_deg);
      } else {
        // Mode 1: WaypointMachine — continue forward, no perimeter
        moveWaypointForward(WP_GOAL, 0.3f);
      }

      if (rejoin_counter >= 4) {
        rejoin_counter = 0;
        navigation_state = SAFE;
        break;
      }

      break;
    }

    // OUT_OF_BOUNDS state: turn away and move forward until being back in bounds.
    // Then switch to SEARCH_FOR_SAFE_HEADING to check for obstacles again
    case OUT_OF_BOUNDS: {
      NavSetMaxSpeed(obstacle_max_speed);
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
    }

    // GATE_TRACKING state: when a gate is detected, gate_nav takes over control of heading and waypoints.
    // The only thing waypoint_navigation does is monitor the obstacle detection to switch to OBSTACLE_FOUND if needed.
    case GATE_TRACKING: {
      // gate_nav.c exclusively controls nav.heading, WP_GOAL, WP_TRAJECTORY.
      // waypoint_navigation only monitors the obstacle safety exit condition.
      // Normal exit (gate_tracking == 0) is handled at the top of this function.
      if (obstacle_detected) {
        gate_navigator_abort();  // force gate_nav to passive SEARCH immediately
        navigation_state = OBSTACLE_FOUND;
        printf("[WAY_NAV] GATE_TRACKING -> OBSTACLE_FOUND (obstacle alarm)\n");
      }
      break;
    }

    default:
      break;
  }
}


// Function to increase the NAV heading. Assumes heading is an INT32_ANGLE. It is bound in this function.
uint8_t increase_nav_heading(float incrementDegrees)
{
  float signed_increment_deg = incrementDegrees;

  // Keep the turn away from inner-geofence edge while we remain near that edge.
  if (edge_turn_bias_active) {
    float inward_x, inward_y, edge_dist;
    if (!getClosestInnerEdgeInward(&inward_x, &inward_y, &edge_dist) || edge_dist > inner_edge_margin_m * 1.5f) {
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


// Function to set waypoint 'waypoint' to the coordinates of 'new_coor'
uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor)
{
  VERBOSE_PRINT("Moving waypoint %d to x:%f y:%f\n", waypoint, POS_FLOAT_OF_BFP(new_coor->x),
                POS_FLOAT_OF_BFP(new_coor->y));
  waypoint_move_xy_i(waypoint, new_coor->x, new_coor->y);
  return false;
}


// Function to calculate coordinates of distance forward and set waypoint 'waypoint' to those coordinates
uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters)
{
  struct EnuCoor_i new_coor;
  calculateForwards(&new_coor, distanceMeters);
  moveWaypoint(waypoint, &new_coor);
  return false;
}


// Function to calculate coordinates of a distance of 'distanceMeters' forward w.r.t. current position and heading
uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters)
{
  float heading = nav.heading;

  new_coor->x = stateGetPositionEnu_i()->x + POS_BFP_OF_REAL(sinf(heading) * distanceMeters);
  new_coor->y = stateGetPositionEnu_i()->y + POS_BFP_OF_REAL(cosf(heading) * distanceMeters);
  return false;
}


// Function to set WP_GOAL to the current nominal WP_PATH target.
uint8_t setGoalToPathWaypoint(void)
{
  waypoint_move_xy_i(WP_GOAL, POS_BFP_OF_REAL(WaypointX(WP_PATH)), POS_BFP_OF_REAL(WaypointY(WP_PATH)));
  return false;
}


// Function to compute desired heading toward WP_PATH from current position.
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


// Function to align heading toward WP_PATH with a slew-rate limit
uint8_t setHeadingToPathWaypointLimited(float max_delta_deg)
{
  float desired_heading;
  getHeadingToPathWaypoint(&desired_heading);

  float heading_error = angle_diff(desired_heading, nav.heading);
  float max_delta = RadOfDeg(max_delta_deg);
  float limited_delta = clampf(heading_error, -max_delta, max_delta);

  float new_heading = nav.heading + limited_delta;
  FLOAT_ANGLE_NORMALIZE(new_heading);
  nav.heading = new_heading;
  return false;
}


// Function to calculate difference between two headings
float angle_diff(float a, float b)
{
  float d = a - b;
  FLOAT_ANGLE_NORMALIZE(d);
  return d;
}


// Function to clamp float between two values
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


// Function to select which direction to turn depending on distance from edge
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
  float path_heading;
  getHeadingToPathWaypoint(&path_heading);
  float diff = angle_diff(path_heading, nav.heading);

  if (diff > 0.f) {
    heading_increment = fabsf(heading_increment);
  } else {
    heading_increment = -fabsf(heading_increment);
  }

  VERBOSE_PRINT("Turn closest to PATH with increment: %f\n", heading_increment);
  return false;
}


// Function to choose turn direction when close to inner geofence edge to get away from it.
uint8_t chooseEdgeAwareIncrement(float *increment_deg)
{
  float inward_x, inward_y, edge_dist;
  if (!getClosestInnerEdgeInward(&inward_x, &inward_y, &edge_dist)) {
    return false;
  }

  if (edge_dist > inner_edge_margin_m) {
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


// Function to check if the navigation state is REJOIN_PATH
uint8_t nav_state_is_rejoin_path(void) {
  return navigation_state == REJOIN_PATH;
}
