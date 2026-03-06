/*
 * Copyright (C) Kirk Scheper <kirkscheper@gmail.com>
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/orange_avoider/orange_no_gps.c"
 * @author Kirk Scheper
 * This module is an example module for the course AE4317 Autonomous Flight of Micro Air Vehicles at the TU Delft.
 * This module is used in combination with a color filter (cv_detect_color_object) and the guided mode of the autopilot.
 * The avoidance strategy is to simply count the total number of orange pixels. When above a certain percentage threshold,
 * (given by color_count_frac) we assume that there is an obstacle and we turn.
 *
 * The color filter settings are set using the cv_detect_color_object. This module can run multiple filters simultaneously
 * so you have to define which filter to use with the ORANGE_AVOIDER_VISUAL_DETECTION_ID setting.
 * This module differs from the simpler orange_avoider.xml in that this is flown in guided mode. This flight mode is
 * less dependent on a global positioning estimate as witht the navigation mode. This module can be used with a simple
 * speed estimate rather than a global position.
 *
 * Here we also need to use our onboard sensors to stay inside of the cyberzoo and not collide with the nets. For this
 * we employ a simple color detector, similar to the orange poles but for green to detect the floor. When the total amount
 * of green drops below a given threshold (given by floor_count_frac) we assume we are near the edge of the zoo and turn
 * around. The color detection is done by the cv_detect_color_object module, use the FLOOR_VISUAL_DETECTION_ID setting to
 * define which filter to use.
 */

#include "modules/mav_course_exercise/no_gps_Mattias/orange_no_gps.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"
#include "generated/airframe.h"
#include "state.h"
#include "autopilot.h"
#include "modules/core/abi.h"
#include "mcu_periph/sys_time.h"
#include <math.h>
#include <stdio.h>
#include <time.h>
#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
#include <string.h>
#endif

#define ORANGE_AVOIDER_VERBOSE TRUE

#define PRINT(string,...) fprintf(stderr, "[orange_no_gps->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if ORANGE_AVOIDER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

static uint8_t choose_random_increment_no_gps(void);
static bool ong_takeoff_gate_ready(void);
static float ong_wrap_pi(float a);
static void ong_enter_edge_escape(float floor_centroid_x_frac, float floor_ratio, float psi, uint32_t now_ts,
                                  const char *from_state, bool keep_turn_dir);

enum navigation_state_t {
  // Normal navigation: move using body-velocity commands and visual cues.
  SAFE,
  // Obstacle directly ahead: pause forward motion and pick a new heading.
  OBSTACLE_FOUND,
  // Rotate in place until the forward sector is considered safe again.
  SEARCH_FOR_SAFE_HEADING
};
static const char *ong_state_name(enum navigation_state_t s)
{
  switch (s) {
    case SAFE: return "SAFE";
    case OBSTACLE_FOUND: return "OBSTACLE_FOUND";
    case SEARCH_FOR_SAFE_HEADING: return "SEARCH_FOR_SAFE_HEADING";
    default: return "UNKNOWN";
  }
}

enum edge_state_t {
  // Inside-mat condition: run nominal orange avoid forward behavior.
  EDGE_OK = 0,
  // Edge suspected: slow down and verify edge with bottom-camera floor metrics.
  EDGE_WARN,
  // Edge confirmed: rotate inward until mat is reacquired.
  EDGE_ESCAPE,
  // After reacquire: add a fixed extra inward turn ("DVD bounce") before resuming.
  EDGE_RECOVER
};
static const char *ong_edge_state_name(enum edge_state_t s)
{
  switch (s) {
    case EDGE_OK: return "EDGE_OK";
    case EDGE_WARN: return "EDGE_WARN";
    case EDGE_ESCAPE: return "EDGE_ESCAPE";
    case EDGE_RECOVER: return "EDGE_RECOVER";
    default: return "EDGE_UNKNOWN";
  }
}

// define settings
float ong_color_count_frac = 0.18f;       // obstacle detection threshold as a fraction of total of image
float ong_floor_count_frac = 0.05f;       // floor detection threshold as a fraction of total of image
float ong_max_speed = 0.5f;               // max flight speed [m/s]
float ong_max_horiz_speed = 1.0f;         // safety limit on measured horizontal speed [m/s]
float ong_heading_rate = RadOfDeg(30.f);  // heading change setpoint for avoidance [rad/s]
float ong_speed_kp = 0.6f;                // P gain from measured forward speed to commanded forward speed
float ong_cmd_accel_limit = 0.6f;         // max command slew for body velocity [m/s^2]
float ong_max_side_speed = 0.2f;          // max lateral speed command for floor centering [m/s]
float ong_floor_center_gain = 0.8f;       // gain from floor centroid fraction to lateral speed command
float ong_edge_ok_ratio = 0.68f;          // floor-ratio threshold for normal edge-ok operation
float ong_edge_warn_ratio = 0.70f;        // floor-ratio threshold to enter edge-warn mode
float ong_edge_escape_ratio = 0.58f;      // floor-ratio threshold to trigger edge-escape turn
float ong_edge_recover_ratio = 0.62f;     // floor-ratio threshold to confirm return onto mat
float ong_edge_angle_measure_ratio = 0.90f; // latch incidence angle when >20% of bottom view is off-mat
float ong_edge_full_mat_ratio = 0.95f;    // near-full bottom view required to declare we are back on mat
float ong_edge_full_mat_conf_required = 4.f; // consecutive full-mat samples before EDGE_OK
float ong_edge_warn_speed = 0.15f;        // max forward speed while in edge-warn [m/s]
float ong_edge_escape_turn_rate = RadOfDeg(50.f); // yaw-rate used while escaping edge [rad/s]
float ong_edge_align_centroid_x = 0.18f;  // |x-centroid| threshold used to detect inward alignment
float ong_edge_escape_min_turn_time = 0.35f; // minimum forced turn time in edge-escape [s]
float ong_edge_escape_max_turn_time = 1.2f;  // maximum forced turn time in edge-escape [s]
float ong_edge_recover_angle_deg = 45.f;  // minimum outward angle from edge after bounce [deg]
float ong_edge_recover_max_angle_deg = 180.f; // maximum inward turn angle after reacquiring mat [deg]
float ong_edge_recover_forward_speed = 0.25f; // forward speed after recover turn [m/s]
float ong_edge_recover_forward_time = 1.2f;   // forward push duration after recover turn [s]
float ong_edge_turn_sign = 1.f;           // sign used to map centroid side to turn direction (+1/-1)
float ong_edge_warn_conf_required = 1.f;  // consecutive warn samples before EDGE_WARN
float ong_edge_recover_conf_required = 2.f; // consecutive recover samples before EDGE_OK
float ong_min_takeoff_alt = 0.35f;         // minimum altitude above launch before forward motion [m]
float ong_takeoff_hover_time = 1.0f;       // hover settle time after liftoff [s]
float ong_takeoff_max_horiz_speed = 0.35f; // max horizontal speed to exit takeoff guard [m/s]
float ong_takeoff_target_alt = 0.80f;      // target takeoff altitude used for climb soft-stop [m]
float ong_takeoff_brake_margin = 0.25f;    // start slowing climb this far below target altitude [m]

// define and initialise global variables
static enum navigation_state_t navigation_state = SEARCH_FOR_SAFE_HEADING;   // current state in state machine
static int32_t color_count = 0;                // orange color count from color filter for obstacle detection
static int32_t floor_count = 0;                // green color count from color filter for floor detection
static int32_t floor_centroid = 0;             // floor detector centroid in y direction
static int32_t floor_centroid_x = 0;           // floor detector centroid in x direction
static float avoidance_heading_direction = 0;  // heading change direction for avoidance [rad/s]
static int16_t obstacle_free_confidence = 0;   // a measure of how certain we are that the way ahead if safe.
static enum edge_state_t ong_edge_state = EDGE_OK; // edge handling state machine
static int16_t ong_edge_warn_confidence = 0;
static int16_t ong_edge_recover_confidence = 0;
static int16_t ong_edge_full_mat_confidence = 0;
static float ong_edge_turn_dir = 1.f;
static uint32_t ong_edge_escape_start_ts = 0;
static uint32_t ong_edge_escape_release_ts = 0;
static uint32_t ong_edge_recover_until_ts = 0;
static uint32_t ong_edge_recover_forward_until_ts = 0;
static float ong_edge_escape_entry_psi = 0.f;
static float ong_edge_angle_latch_psi = 0.f;
static bool ong_edge_angle_latched = false;
static float ong_edge_recover_hold_psi = 0.f;
static float ong_edge_in_angle_deg = 0.f;
static float ong_edge_out_angle_deg = 0.f;

static const int16_t max_trajectory_confidence = 5;  // number of consecutive negative object detections to be sure we are obstacle free

static float ong_cmd_vx = 0.f;
static float ong_cmd_vy = 0.f;
static bool ong_active = false;
static float ong_last_speed_sp = 0.f;
static float ong_last_heading_rate_cmd = 0.f;
static float ong_last_floor_centroid_frac = 0.f;
static float ong_last_floor_centroid_x_frac = 0.f;
static float ong_last_floor_ratio = 0.f;
static float ong_last_horiz_speed = 0.f;
static uint32_t ong_takeoff_gate_start_ts = 0;
static float ong_last_speed_ref = 0.f;
static float ong_last_vx_target = 0.f;
static float ong_last_vy_target = 0.f;
static bool ong_hover_hold_initialized = false;
static float ong_hover_hold_z = 0.f;

#if PERIODIC_TELEMETRY
static void ong_send_state_debug(struct transport_tx *trans, struct link_device *dev)
{
  char name[] = "orange_no_gps";
  float data[12] = {
    (float)navigation_state,
    (float)ong_edge_state,
    (float)obstacle_free_confidence,
    (float)ong_edge_warn_confidence,
    (float)ong_edge_recover_confidence,
    ong_cmd_vx,
    ong_cmd_vy,
    ong_last_speed_sp,
    ong_last_heading_rate_cmd,
    ong_last_floor_centroid_frac,
    ong_last_floor_centroid_x_frac,
    ong_last_floor_ratio
  };
  pprz_msg_send_DEBUG_VECT(trans, dev, AC_ID, strlen(name), name, 12, data);
}
#endif

// This call back will be used to receive the color count from the orange detector
#ifndef ORANGE_AVOIDER_VISUAL_DETECTION_ID
#error This module requires two color filters, as such you have to define ORANGE_AVOIDER_VISUAL_DETECTION_ID to the orange filter
#error Please define ORANGE_AVOIDER_VISUAL_DETECTION_ID to be COLOR_OBJECT_DETECTION1_ID or COLOR_OBJECT_DETECTION2_ID in your airframe
#endif
static abi_event color_detection_ev;
static void color_detection_cb(uint8_t __attribute__((unused)) sender_id,
                               int16_t __attribute__((unused)) pixel_x, int16_t __attribute__((unused)) pixel_y,
                               int16_t __attribute__((unused)) pixel_width, int16_t __attribute__((unused)) pixel_height,
                               int32_t quality, int16_t __attribute__((unused)) extra)
{
  color_count = quality;
}

#ifndef FLOOR_VISUAL_DETECTION_ID
#error This module requires two color filters, as such you have to define FLOOR_VISUAL_DETECTION_ID to the orange filter
#error Please define FLOOR_VISUAL_DETECTION_ID to be COLOR_OBJECT_DETECTION1_ID or COLOR_OBJECT_DETECTION2_ID in your airframe
#endif
static abi_event floor_detection_ev;
static void floor_detection_cb(uint8_t __attribute__((unused)) sender_id,
                               int16_t pixel_x, int16_t pixel_y,
                               int16_t __attribute__((unused)) pixel_width, int16_t __attribute__((unused)) pixel_height,
                               int32_t quality, int16_t __attribute__((unused)) extra)
{
  floor_count = quality;
  floor_centroid_x = pixel_x;
  floor_centroid = pixel_y;
}

/*
 * Initialisation function
 */
void orange_no_gps_init(void)
{
  // This module intentionally avoids GPS; it uses only local vision/IMU/AGL/state.
  // Initialise random values
  srand(time(NULL));
  choose_random_increment_no_gps();

  // bind our colorfilter callbacks to receive the color filter outputs
  AbiBindMsgVISUAL_DETECTION(ORANGE_AVOIDER_VISUAL_DETECTION_ID, &color_detection_ev, color_detection_cb);
  AbiBindMsgVISUAL_DETECTION(FLOOR_VISUAL_DETECTION_ID, &floor_detection_ev, floor_detection_cb);
#if PERIODIC_TELEMETRY
  // Persist internal navigation state and commanded controls in Paparazzi Center/logger.
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_DEBUG_VECT, ong_send_state_debug);
#endif
}

static void ong_enter_edge_escape(float floor_centroid_x_frac, float floor_ratio, float psi, uint32_t now_ts,
                                  const char *from_state, bool keep_turn_dir)
{
  ong_edge_state = EDGE_ESCAPE;
  if (!keep_turn_dir) {
    float side = (fabsf(floor_centroid_x_frac) > 0.02f) ? floor_centroid_x_frac : avoidance_heading_direction;
    ong_edge_turn_dir = (side >= 0.f) ? 1.f : -1.f;
    if (ong_edge_turn_sign < 0.f) {
      ong_edge_turn_dir = -ong_edge_turn_dir;
    }
  }
  ong_edge_escape_start_ts = now_ts;
  ong_edge_escape_release_ts = now_ts + (uint32_t)(fmaxf(0.f, ong_edge_escape_min_turn_time) * 1e6f);
  ong_edge_escape_entry_psi = ong_edge_angle_latched ? ong_edge_angle_latch_psi : psi;
  ong_edge_recover_forward_until_ts = 0;
  ong_edge_in_angle_deg = 0.f;
  ong_edge_out_angle_deg = 0.f;
  VERBOSE_PRINT("edge transition %s -> EDGE_ESCAPE turn_dir=%.1f floor_ratio=%.3f x=%.3f psi_ref=%.1f latched=%d\n",
                from_state, ong_edge_turn_dir, floor_ratio, floor_centroid_x_frac,
                DegOfRad(ong_edge_escape_entry_psi), ong_edge_angle_latched ? 1 : 0);
}

/*
 * Function that checks it is safe to move forwards, and then sets a forward velocity setpoint or changes the heading
 */
void orange_no_gps_periodic(void)
{
  static enum navigation_state_t prev_state = SEARCH_FOR_SAFE_HEADING;
  static uint32_t last_ts = 0;

  // Priority order for this loop:
  // 1) Ensure we are in GUIDED and module is active.
  // 2) Hold hover during takeoff gate (altitude + settle + low horizontal speed).
  // 3) Evaluate vision confidence (orange obstacle + green floor/edge).
  // 4) Run state machine to produce body-velocity and heading commands.

  // Only run the mudule if we are in the correct flight mode
  if (guidance_h.mode != GUIDANCE_H_MODE_GUIDED) {
    navigation_state = SEARCH_FOR_SAFE_HEADING;
    ong_edge_state = EDGE_OK;
    obstacle_free_confidence = 0;
    ong_edge_warn_confidence = 0;
    ong_edge_recover_confidence = 0;
    ong_edge_full_mat_confidence = 0;
    ong_edge_recover_forward_until_ts = 0;
    ong_edge_angle_latched = false;
    return;
  }

  // Keep takeoff/standby in GUIDED hover until START explicitly enables behavior.
  if (!ong_active) {
    guidance_h_set_body_vel(0.f, 0.f);
    guidance_h_set_heading(stateGetNedToBodyEulers_f()->psi);
    ong_cmd_vx = 0.f;
    ong_cmd_vy = 0.f;
    ong_edge_state = EDGE_OK;
    ong_edge_warn_confidence = 0;
    ong_edge_recover_confidence = 0;
    ong_edge_full_mat_confidence = 0;
    ong_edge_recover_forward_until_ts = 0;
    ong_edge_angle_latched = false;
    ong_last_heading_rate_cmd = 0.f;
    return;
  }

  if (!ong_takeoff_gate_ready()) {
    struct NedCoor_f *pos_hold = stateGetPositionNed_f();
    struct NedCoor_f *vel_hold = stateGetSpeedNed_f();
    float alt_up = fmaxf(0.f, -pos_hold->z);
    float horiz_speed = sqrtf(vel_hold->x * vel_hold->x + vel_hold->y * vel_hold->y);
    guidance_h_set_body_vel(0.f, 0.f);
    guidance_h_set_heading(stateGetNedToBodyEulers_f()->psi);
    ong_cmd_vx = 0.f;
    ong_cmd_vy = 0.f;
    ong_edge_state = EDGE_OK;
    ong_edge_warn_confidence = 0;
    ong_edge_recover_confidence = 0;
    ong_edge_full_mat_confidence = 0;
    ong_edge_recover_forward_until_ts = 0;
    ong_edge_angle_latched = false;
    ong_last_heading_rate_cmd = 0.f;
    navigation_state = SEARCH_FOR_SAFE_HEADING;
    obstacle_free_confidence = 0;
    VERBOSE_PRINT("takeoff_guard hold alt=%.2f/%.2f vxy=%.2f<=%.2f settle=%.2fs\n",
                  alt_up, ong_min_takeoff_alt, horiz_speed, ong_takeoff_max_horiz_speed, ong_takeoff_hover_time);
    return;
  }

  // compute current color thresholds
  int32_t color_count_threshold = ong_color_count_frac * front_camera.output_size.w * front_camera.output_size.h;
  uint16_t floor_w = bottom_camera.output_size.w;
  uint16_t floor_h = bottom_camera.output_size.h;
  if (floor_w == 0 || floor_h == 0) {
    floor_w = front_camera.output_size.w;
    floor_h = front_camera.output_size.h;
  }
  int32_t floor_image_area = floor_w * floor_h;
  if (floor_image_area <= 0) {
    floor_image_area = 1;
  }
  int32_t floor_count_threshold = ong_floor_count_frac * floor_image_area;
  float floor_ratio = floor_count / (float)floor_image_area;
  Bound(floor_ratio, 0.f, 1.f);
  // Color detector centroids are in pixels around image center ([-w/2..w/2], [-h/2..h/2]).
  // Normalize to [-1..1] so edge-follow gains/thresholds have a stable physical meaning.
  float floor_centroid_frac = (2.f * floor_centroid) / (float)floor_h;
  float floor_centroid_x_frac = (2.f * floor_centroid_x) / (float)floor_w;
  Bound(floor_centroid_frac, -1.f, 1.f);
  Bound(floor_centroid_x_frac, -1.f, 1.f);

  VERBOSE_PRINT("Color_count: %d  threshold: %d state: %d \n", color_count, color_count_threshold, navigation_state);
  VERBOSE_PRINT("Floor count: %d, threshold: %d\n", floor_count, floor_count_threshold);
  VERBOSE_PRINT("Floor ratio: %.3f centroid=(x:%.3f,y:%.3f)\n",
                floor_ratio, floor_centroid_x_frac, floor_centroid_frac);

  // update our safe confidence using color threshold
  if(color_count < color_count_threshold){
    obstacle_free_confidence++;
  } else {
    obstacle_free_confidence -= 2;  // be more cautious with positive obstacle detections
  }

  // bound obstacle_free_confidence
  Bound(obstacle_free_confidence, 0, max_trajectory_confidence);

  uint32_t now_ts = get_sys_time_usec();
  float dt = 0.25f;
  if (last_ts != 0 && now_ts > last_ts) {
    dt = (now_ts - last_ts) * 1e-6f;
    Bound(dt, 0.02f, 0.5f);
  }
  last_ts = now_ts;

  float speed_sp = fminf(ong_max_speed, 0.2f * obstacle_free_confidence);
  float psi = stateGetNedToBodyEulers_f()->psi;
  struct NedCoor_f *pos = stateGetPositionNed_f();
  struct NedCoor_f *vel = stateGetSpeedNed_f();
  float cpsi = cosf(psi);
  float spsi = sinf(psi);
  float vel_body_x = cpsi * vel->x + spsi * vel->y;
  float vel_body_y = -spsi * vel->x + cpsi * vel->y;
  float horiz_speed = sqrtf(vel->x * vel->x + vel->y * vel->y);
  ong_last_speed_sp = speed_sp;
  ong_last_floor_centroid_frac = floor_centroid_frac;
  ong_last_floor_centroid_x_frac = floor_centroid_x_frac;
  ong_last_floor_ratio = floor_ratio;
  ong_last_horiz_speed = horiz_speed;
  ong_last_speed_ref = 0.f;
  ong_last_vx_target = 0.f;
  ong_last_vy_target = 0.f;

  // Update edge confidences only in EDGE_OK/EDGE_WARN.
  // In EDGE_ESCAPE/EDGE_RECOVER we intentionally ignore new bottom-camera edge decisions
  // and only use full-mat confidence as exit criterion.
  if (ong_edge_state == EDGE_OK || ong_edge_state == EDGE_WARN) {
    if (floor_ratio < ong_edge_warn_ratio) {
      // Escalate faster when floor support collapses, so edge handling triggers earlier.
      int16_t warn_step = (floor_ratio < ong_edge_escape_ratio) ? 3 : 1;
      ong_edge_warn_confidence += warn_step;
    } else {
      ong_edge_warn_confidence--;
    }
    Bound(ong_edge_warn_confidence, 0, 100);

    if (floor_ratio > ong_edge_recover_ratio) {
      ong_edge_recover_confidence++;
    } else {
      ong_edge_recover_confidence--;
    }
    Bound(ong_edge_recover_confidence, 0, 100);
  }

  if (floor_ratio > ong_edge_full_mat_ratio) {
    ong_edge_full_mat_confidence++;
  } else {
    ong_edge_full_mat_confidence--;
  }
  Bound(ong_edge_full_mat_confidence, 0, 100);

  uint8_t warn_conf_required = (uint8_t)fmaxf(1.f, ong_edge_warn_conf_required);
  uint8_t recover_conf_required = (uint8_t)fmaxf(1.f, ong_edge_recover_conf_required);
  uint8_t full_mat_conf_required = (uint8_t)fmaxf(1.f, ong_edge_full_mat_conf_required);
  bool edge_inward_aligned = (floor_ratio > ong_edge_recover_ratio &&
                              fabsf(floor_centroid_x_frac) < ong_edge_align_centroid_x);

  // Latch the incidence reference heading once more than 20% of the bottom image is off-mat.
  if (floor_ratio < ong_edge_angle_measure_ratio) {
    if (!ong_edge_angle_latched) {
      ong_edge_angle_latched = true;
      ong_edge_angle_latch_psi = psi;
      VERBOSE_PRINT("edge angle latch floor_ratio=%.3f (off=%.3f) psi=%.1f\n",
                    floor_ratio, 1.f - floor_ratio, DegOfRad(psi));
    }
  } else if (ong_edge_state == EDGE_OK) {
    ong_edge_angle_latched = false;
  }

  switch (ong_edge_state) {
    case EDGE_OK:
      if (floor_ratio < ong_edge_escape_ratio) {
        ong_enter_edge_escape(floor_centroid_x_frac, floor_ratio, psi, now_ts, "EDGE_OK", false);
      } else if (floor_ratio < ong_edge_warn_ratio && ong_edge_warn_confidence >= warn_conf_required) {
        ong_edge_state = EDGE_WARN;
      } else if (floor_ratio > ong_edge_ok_ratio) {
        ong_edge_angle_latched = false;
      }
      break;

    case EDGE_WARN:
      if (floor_ratio < ong_edge_escape_ratio) {
        ong_enter_edge_escape(floor_centroid_x_frac, floor_ratio, psi, now_ts, "EDGE_WARN", false);
      } else if (floor_ratio > ong_edge_ok_ratio && ong_edge_recover_confidence >= recover_conf_required) {
        ong_edge_state = EDGE_OK;
      }
      break;

    case EDGE_ESCAPE: {
      float escape_turn_abs_deg = DegOfRad(fabsf(ong_wrap_pi(psi - ong_edge_escape_entry_psi)));
      ong_edge_in_angle_deg = escape_turn_abs_deg;
      bool min_turn_done = now_ts >= ong_edge_escape_release_ts;
      uint32_t max_turn_us = (uint32_t)(fmaxf(ong_edge_escape_min_turn_time, ong_edge_escape_max_turn_time) * 1e6f);
      bool max_turn_done = (now_ts - ong_edge_escape_start_ts) >= max_turn_us;
      if ((min_turn_done && edge_inward_aligned) || max_turn_done) {
        ong_edge_state = EDGE_RECOVER;
        float escape_turn_rate = fmaxf(RadOfDeg(8.f), fabsf(ong_edge_escape_turn_rate));
        // Beta is how much we already turned while escaping the edge.
        // Compute desired *total* bounce deflection and then only command the missing extra turn.
        float beta_deg = fminf(90.f, fmaxf(0.f, ong_edge_in_angle_deg));
        float desired_total_deflection_deg = (beta_deg < ong_edge_recover_angle_deg) ?
                                             (beta_deg + ong_edge_recover_angle_deg) :
                                             (2.f * beta_deg);
        Bound(desired_total_deflection_deg, ong_edge_recover_angle_deg, ong_edge_recover_max_angle_deg);
        float recover_extra_turn_deg = fmaxf(0.f, desired_total_deflection_deg - beta_deg);
        ong_edge_out_angle_deg = recover_extra_turn_deg;
        float extra_turn_t = fabsf((float)RadOfDeg(recover_extra_turn_deg)) / escape_turn_rate;
        ong_edge_recover_until_ts = now_ts + (uint32_t)(extra_turn_t * 1e6f);
        ong_edge_recover_forward_until_ts = 0;
        VERBOSE_PRINT("edge transition EDGE_ESCAPE -> EDGE_RECOVER in=%.1f total=%.1f extra=%.1f extra_t=%.2f floor_ratio=%.3f x=%.3f\n",
                      ong_edge_in_angle_deg, desired_total_deflection_deg, ong_edge_out_angle_deg,
                      extra_turn_t, floor_ratio, floor_centroid_x_frac);
      }
      break;
    }

    case EDGE_RECOVER:
      if (now_ts >= ong_edge_recover_until_ts) {
        if (ong_edge_recover_forward_until_ts == 0) {
          // Mark forward-recovery start once; recovery now exits only on floor confidence,
          // not elapsed time.
          ong_edge_recover_forward_until_ts = now_ts;
          ong_edge_recover_hold_psi = psi;
          VERBOSE_PRINT("edge recover forward start hold=%.1f speed=%.2f floor_ratio=%.3f\n",
                        DegOfRad(ong_edge_recover_hold_psi), ong_edge_recover_forward_speed, floor_ratio);
        }

        if (ong_edge_full_mat_confidence >= full_mat_conf_required) {
          ong_edge_state = EDGE_OK;
          ong_edge_recover_forward_until_ts = 0;
          ong_edge_angle_latched = false;
          // Clear stale edge confidences so we don't immediately re-enter EDGE_WARN
          // from old low-floor samples gathered before recovery finished.
          ong_edge_warn_confidence = 0;
          ong_edge_recover_confidence = 0;
        }
      }
      break;

    default:
      ong_edge_state = EDGE_OK;
      break;
  }

  VERBOSE_PRINT("dbg mode=%d nav_state=%s edge_state=%s conf=%d/%d edge_conf(w:%d r:%d) edge_io=(%.1f->%.1f) speed_sp=%.2f floor_ratio=%.3f floor_xy=(%.3f,%.3f) cmd=(%.2f,%.2f) vbody=(%.2f,%.2f) vxy=%.2f vmax=%.2f heading_dir=%.1f edge_turn=%.1f psi=%.2f pos=(%.2f,%.2f,%.2f) vel=(%.2f,%.2f,%.2f)\n",
                guidance_h.mode, ong_state_name(navigation_state),
                ong_edge_state_name(ong_edge_state),
                obstacle_free_confidence, max_trajectory_confidence,
                ong_edge_warn_confidence, ong_edge_recover_confidence,
                ong_edge_in_angle_deg, ong_edge_out_angle_deg,
                speed_sp,
                floor_ratio, floor_centroid_x_frac, floor_centroid_frac,
                ong_cmd_vx, ong_cmd_vy, vel_body_x, vel_body_y, horiz_speed, ong_max_horiz_speed, avoidance_heading_direction,
                ong_edge_turn_dir,
                psi, pos->x, pos->y, pos->z, vel->x, vel->y, vel->z);

  if (ong_edge_state == EDGE_ESCAPE || ong_edge_state == EDGE_RECOVER) {
    float turn_rate = fmaxf(RadOfDeg(8.f), fabsf(ong_edge_escape_turn_rate));
    navigation_state = SEARCH_FOR_SAFE_HEADING;

    if (ong_edge_state == EDGE_ESCAPE || now_ts < ong_edge_recover_until_ts) {
      // Execute edge turn using internal yaw state (IMU-based heading integration).
      guidance_h_set_body_vel(0.f, 0.f);
      ong_cmd_vx = 0.f;
      ong_cmd_vy = 0.f;
      ong_last_heading_rate_cmd = ong_edge_turn_dir * turn_rate;
      guidance_h_set_heading_rate(ong_last_heading_rate_cmd);
      VERBOSE_PRINT("cmd %s heading_rate=%.2f deg/s floor_ratio=%.3f x=%.3f\n",
                    ong_edge_state_name(ong_edge_state), DegOfRad(ong_last_heading_rate_cmd),
                    floor_ratio, floor_centroid_x_frac);
    } else {
      // After completing the turn, push forward away from the edge while holding yaw.
      guidance_h_set_heading(ong_edge_recover_hold_psi);
      guidance_h_set_body_vel(ong_edge_recover_forward_speed, 0.f);
      ong_cmd_vx = ong_edge_recover_forward_speed;
      ong_cmd_vy = 0.f;
      ong_last_heading_rate_cmd = 0.f;
      VERBOSE_PRINT("cmd EDGE_RECOVER_FORWARD body_vel=(%.2f, 0.00) hold=%.2f deg floor_ratio=%.3f x=%.3f\n",
                    ong_edge_recover_forward_speed, DegOfRad(ong_edge_recover_hold_psi),
                    floor_ratio, floor_centroid_x_frac);
    }

    if (navigation_state != prev_state) {
      VERBOSE_PRINT("transition %s -> %s\n", ong_state_name(prev_state), ong_state_name(navigation_state));
      prev_state = navigation_state;
    }
    return;
  }

  switch (navigation_state){
    case SAFE:
      bool obstacle_detected = (obstacle_free_confidence == 0);
      if (obstacle_detected){
        // Obstacle detected in front camera: run orange-avoider turn behavior.
        navigation_state = OBSTACLE_FOUND;
      } else if (horiz_speed > ong_max_horiz_speed) {
        VERBOSE_PRINT("cmd SAFE speed_guard triggered: vxy=%.2f > %.2f, braking\n", horiz_speed, ong_max_horiz_speed);
        ong_cmd_vx = 0.f;
        ong_cmd_vy = 0.f;
        ong_last_heading_rate_cmd = 0.f;
        guidance_h_set_body_vel(0.f, 0.f);
      } else {
        float speed_ref = speed_sp;
        if (ong_edge_state == EDGE_WARN) {
          speed_ref = fminf(speed_ref, ong_edge_warn_speed);
        }
        float vy_target = -ong_floor_center_gain * floor_centroid_frac;

        // Velocity command shaping:
        // - speed feedback term reduces drift between requested and measured body-x speed.
        // - accel limit prevents abrupt command jumps that can destabilize attitude.
        float vx_target = speed_ref + ong_speed_kp * (speed_ref - vel_body_x);
        Bound(vx_target, 0.f, ong_max_speed);
        BoundAbs(vy_target, ong_max_side_speed);
        float dvmax = ong_cmd_accel_limit * dt;
        float dvx = vx_target - ong_cmd_vx;
        float dvy = vy_target - ong_cmd_vy;
        BoundAbs(dvx, dvmax);
        BoundAbs(dvy, dvmax);
        ong_cmd_vx += dvx;
        ong_cmd_vy += dvy;
        ong_last_speed_ref = speed_ref;
        ong_last_vx_target = vx_target;
        ong_last_vy_target = vy_target;
        VERBOSE_PRINT("cmd SAFE body_vel=(%.2f, %.2f) target=(%.2f, %.2f) speed_ref=%.2f floor_ratio=%.3f\n",
                      ong_cmd_vx, ong_cmd_vy, vx_target, vy_target, speed_ref, floor_ratio);

        // SAFE flies heading-hold; turning behavior is done in SEARCH.
        if (fabsf(ong_last_heading_rate_cmd) > 1e-4f) {
          guidance_h_set_heading(stateGetNedToBodyEulers_f()->psi);
        }
        ong_last_heading_rate_cmd = 0.f;
        guidance_h_set_body_vel(ong_cmd_vx, ong_cmd_vy);
      }

      break;
    case OBSTACLE_FOUND:
      // stop
      VERBOSE_PRINT("cmd OBSTACLE_FOUND body_vel=(0.00, 0.00)\n");
      ong_cmd_vx = 0.f;
      ong_cmd_vy = 0.f;
      ong_last_heading_rate_cmd = 0.f;
      guidance_h_set_body_vel(0, 0);

      // randomly select new search direction
      choose_random_increment_no_gps();

      navigation_state = SEARCH_FOR_SAFE_HEADING;

      break;
    case SEARCH_FOR_SAFE_HEADING:
      guidance_h_set_body_vel(0.f, 0.f);
      ong_cmd_vx = 0.f;
      ong_cmd_vy = 0.f;
      VERBOSE_PRINT("cmd SEARCH heading_rate=%.2f deg/s\n", DegOfRad(avoidance_heading_direction * ong_heading_rate));
      ong_last_heading_rate_cmd = avoidance_heading_direction * ong_heading_rate;
      guidance_h_set_heading_rate(avoidance_heading_direction * ong_heading_rate);

      // make sure we have a couple of good readings before declaring the way safe
      if (obstacle_free_confidence >= 2){
        VERBOSE_PRINT("cmd SEARCH->SAFE heading_hold=%.2f deg\n", DegOfRad(stateGetNedToBodyEulers_f()->psi));
        guidance_h_set_heading(stateGetNedToBodyEulers_f()->psi);
        navigation_state = SAFE;
      }
      break;
    default:
      break;
  }
  if (navigation_state != prev_state) {
    VERBOSE_PRINT("transition %s -> %s\n", ong_state_name(prev_state), ong_state_name(navigation_state));
    prev_state = navigation_state;
  }
  return;
}

static bool ong_takeoff_gate_ready(void)
{
  // Gate forward/turning behavior until takeoff is physically settled.
  if (!autopilot_in_flight()) {
    ong_takeoff_gate_start_ts = 0;
    return false;
  }

  uint32_t now_ts = get_sys_time_usec();
  if (ong_takeoff_gate_start_ts == 0) {
    ong_takeoff_gate_start_ts = now_ts;
  }

  struct NedCoor_f *pos = stateGetPositionNed_f();
  struct NedCoor_f *vel = stateGetSpeedNed_f();
  float alt_up = fmaxf(0.f, -pos->z);
  float horiz_speed = sqrtf(vel->x * vel->x + vel->y * vel->y);
  uint32_t settle_us = (uint32_t)(fmaxf(ong_takeoff_hover_time, 0.f) * 1e6f);

  return alt_up >= ong_min_takeoff_alt &&
         horiz_speed <= ong_takeoff_max_horiz_speed &&
         (now_ts - ong_takeoff_gate_start_ts) >= settle_us;
}

/*
 * Command a short backward motion for the flight plan RETREAT block.
 */
void orange_no_gps_retreat(void)
{
  guidance_h_set_body_vel(-ong_max_speed, 0.f);
}

void orange_no_gps_set_active(bool active)
{
  bool was_active = ong_active;
  ong_active = active;
  // Re-latch hover altitude when mission active state toggles.
  if (was_active != ong_active) {
    ong_hover_hold_initialized = false;
  }
  if (!ong_active) {
    // Always reset state machine internals when deactivated by flight-plan blocks.
    navigation_state = SEARCH_FOR_SAFE_HEADING;
    ong_edge_state = EDGE_OK;
    obstacle_free_confidence = 0;
    ong_edge_warn_confidence = 0;
    ong_edge_recover_confidence = 0;
    ong_edge_full_mat_confidence = 0;
    ong_edge_escape_start_ts = 0;
    ong_edge_escape_release_ts = 0;
    ong_edge_recover_until_ts = 0;
    ong_edge_recover_forward_until_ts = 0;
    ong_edge_escape_entry_psi = 0.f;
    ong_edge_angle_latch_psi = 0.f;
    ong_edge_angle_latched = false;
    ong_edge_recover_hold_psi = 0.f;
    ong_edge_in_angle_deg = 0.f;
    ong_edge_out_angle_deg = 0.f;
    ong_cmd_vx = 0.f;
    ong_cmd_vy = 0.f;
    ong_last_heading_rate_cmd = 0.f;
    ong_takeoff_gate_start_ts = 0;
    guidance_h_set_body_vel(0.f, 0.f);
  }
}

bool orange_no_gps_get_active(void)
{
  return ong_active;
}

void orange_no_gps_get_debug_state(struct orange_no_gps_debug_state *out)
{
  if (out == NULL) {
    return;
  }

  out->nav_state = (uint8_t)navigation_state;
  out->edge_state = (uint8_t)ong_edge_state;
  out->obstacle_confidence = obstacle_free_confidence;
  out->edge_warn_confidence = ong_edge_warn_confidence;
  out->edge_recover_confidence = ong_edge_recover_confidence;
  out->edge_turn_dir = ong_edge_turn_dir;
  out->edge_in_angle_deg = ong_edge_in_angle_deg;
  out->edge_out_angle_deg = ong_edge_out_angle_deg;
  out->floor_ratio = ong_last_floor_ratio;
  out->floor_centroid_x_frac = ong_last_floor_centroid_x_frac;
  out->floor_centroid_frac = ong_last_floor_centroid_frac;
  out->speed_sp = ong_last_speed_sp;
  out->speed_ref = ong_last_speed_ref;
  out->cmd_vx = ong_cmd_vx;
  out->cmd_vy = ong_cmd_vy;
  out->vx_target = ong_last_vx_target;
  out->vy_target = ong_last_vy_target;
  out->heading_rate_cmd = ong_last_heading_rate_cmd;
}

bool orange_no_gps_hover_hold(void)
{
  orange_no_gps_set_active(false);
  guidance_h_set_body_vel(0.f, 0.f);
  guidance_h_set_heading(stateGetNedToBodyEulers_f()->psi);
  // Latch altitude once on entry and hold it, instead of continuously commanding only vz=0.
  // This avoids prolonged climb after takeoff when upward velocity is still high.
  if (!ong_hover_hold_initialized) {
    ong_hover_hold_z = stateGetPositionNed_f()->z;
    ong_hover_hold_initialized = true;
  }
  guidance_v_set_z(ong_hover_hold_z);
  ong_cmd_vx = 0.f;
  ong_cmd_vy = 0.f;
  ong_last_heading_rate_cmd = 0.f;
  ong_takeoff_gate_start_ts = 0;
  return true;
}

bool orange_no_gps_takeoff_vz(float vz_up)
{
  orange_no_gps_set_active(false);
  ong_hover_hold_initialized = false;
  guidance_h_set_body_vel(0.f, 0.f);
  guidance_h_set_heading(stateGetNedToBodyEulers_f()->psi);
  float alt_up = fmaxf(0.f, -stateGetPositionNed_f()->z);
  float vz_cmd = -fabsf(vz_up);
  float brake_margin = fmaxf(0.05f, ong_takeoff_brake_margin);
  float brake_start = fmaxf(0.f, ong_takeoff_target_alt - brake_margin);
  if (alt_up > brake_start && ong_takeoff_target_alt > brake_start) {
    float scale = (ong_takeoff_target_alt - alt_up) / (ong_takeoff_target_alt - brake_start);
    Bound(scale, 0.f, 1.f);
    vz_cmd *= scale;
  }
  if (alt_up >= ong_takeoff_target_alt) {
    vz_cmd = 0.f;
  }
  guidance_v_set_vz(vz_cmd);
  ong_cmd_vx = 0.f;
  ong_cmd_vy = 0.f;
  ong_last_heading_rate_cmd = 0.f;
  ong_takeoff_gate_start_ts = 0;
  return true;
}

bool orange_no_gps_land_vz(float vz_down)
{
  orange_no_gps_set_active(false);
  ong_hover_hold_initialized = false;
  guidance_h_set_body_vel(0.f, 0.f);
  guidance_h_set_heading(stateGetNedToBodyEulers_f()->psi);
  guidance_v_set_vz(fabsf(vz_down));
  ong_cmd_vx = 0.f;
  ong_cmd_vy = 0.f;
  ong_last_heading_rate_cmd = 0.f;
  ong_takeoff_gate_start_ts = 0;
  return true;
}

/*
 * Sets the variable 'incrementForAvoidance' randomly positive/negative
 */
static uint8_t choose_random_increment_no_gps(void)
{
  // Randomly choose CW or CCW avoiding direction
  if (rand() % 2 == 0) {
    avoidance_heading_direction = 1.f;
    VERBOSE_PRINT("Set avoidance increment to: %f\n", avoidance_heading_direction * ong_heading_rate);
  } else {
    avoidance_heading_direction = -1.f;
    VERBOSE_PRINT("Set avoidance increment to: %f\n", avoidance_heading_direction * ong_heading_rate);
  }
  return false;
}

static float ong_wrap_pi(float a)
{
  const float pi = 3.14159265358979323846f;
  const float two_pi = 2.f * pi;
  while (a > pi) {
    a -= two_pi;
  }
  while (a < -pi) {
    a += two_pi;
  }
  return a;
}
