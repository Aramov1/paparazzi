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

#ifndef ORANGE_NO_GPS_H
#define ORANGE_NO_GPS_H

#include "std.h"

// settings
extern float ong_color_count_frac;  // obstacle detection threshold as a fraction of total of image
extern float ong_floor_count_frac;  // floor detection threshold as a fraction of total of image
extern float ong_max_speed;         // max flight speed [m/s]
extern float ong_heading_rate;      // heading rate setpoint [rad/s]
extern float ong_max_horiz_speed;   // safety limit on measured horizontal speed [m/s]
extern float ong_speed_kp;          // forward speed feedback gain
extern float ong_cmd_accel_limit;   // body-velocity command slew limit [m/s^2]
extern float ong_max_side_speed;    // max lateral speed command [m/s]
extern float ong_floor_center_gain; // floor centroid to lateral speed gain
extern float ong_edge_ok_ratio;          // floor-ratio threshold for normal edge-ok operation
extern float ong_edge_warn_ratio;        // floor-ratio threshold to enter edge-warn mode
extern float ong_edge_escape_ratio;      // floor-ratio threshold to trigger edge-escape turn
extern float ong_edge_recover_ratio;     // floor-ratio threshold to confirm return onto mat
extern float ong_edge_angle_measure_ratio; // latch incidence angle when floor_ratio drops below this
extern float ong_edge_full_mat_ratio;    // require near-full mat ratio before exiting edge recovery
extern float ong_edge_full_mat_conf_required; // consecutive full-mat samples before EDGE_OK
extern float ong_edge_warn_speed;        // max forward speed while in edge-warn [m/s]
extern float ong_edge_escape_turn_rate;  // yaw-rate used while escaping edge [rad/s]
extern float ong_edge_align_centroid_x;  // |x-centroid| threshold used to detect inward alignment
extern float ong_edge_escape_min_turn_time; // minimum forced turn time in edge-escape [s]
extern float ong_edge_escape_max_turn_time; // maximum forced turn time in edge-escape [s]
extern float ong_edge_recover_angle_deg; // minimum inward turn angle after reacquiring mat [deg]
extern float ong_edge_recover_max_angle_deg; // maximum inward turn angle after reacquiring mat [deg]
extern float ong_edge_recover_forward_speed; // forward speed after recover turn [m/s]
extern float ong_edge_recover_forward_time;  // forward push duration after recover turn [s]
extern float ong_edge_turn_sign;         // sign used to map centroid side to turn direction (+1/-1)
extern float ong_edge_warn_conf_required;    // consecutive warn samples before EDGE_WARN
extern float ong_edge_recover_conf_required; // consecutive recover samples before EDGE_OK
extern float ong_min_takeoff_alt;         // minimum altitude above launch before forward motion [m]
extern float ong_takeoff_hover_time;      // hover settle time after liftoff [s]
extern float ong_takeoff_max_horiz_speed; // max horizontal speed to exit takeoff guard [m/s]
extern float ong_takeoff_target_alt;      // nominal takeoff target altitude used for climb soft-stop [m]
extern float ong_takeoff_brake_margin;    // altitude band before target where climb rate is reduced [m]

struct orange_no_gps_debug_state {
  uint8_t nav_state;
  uint8_t edge_state;
  int16_t obstacle_confidence;
  int16_t edge_warn_confidence;
  int16_t edge_recover_confidence;
  float edge_turn_dir;
  float edge_in_angle_deg;
  float edge_out_angle_deg;
  float floor_ratio;
  float floor_centroid_x_frac;
  float floor_centroid_frac;
  float speed_sp;
  float speed_ref;
  float cmd_vx;
  float cmd_vy;
  float vx_target;
  float vy_target;
  float heading_rate_cmd;
};

extern void orange_no_gps_init(void);
extern void orange_no_gps_periodic(void);
extern void orange_no_gps_retreat(void);
extern void orange_no_gps_set_active(bool active);
extern bool orange_no_gps_get_active(void);
extern bool orange_no_gps_hover_hold(void);
extern bool orange_no_gps_takeoff_vz(float vz_up);
extern bool orange_no_gps_land_vz(float vz_down);
extern void orange_no_gps_get_debug_state(struct orange_no_gps_debug_state *out);

#endif
