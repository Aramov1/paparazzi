/*
 * Copyright (C) 2014 Freek van Tienen <freek.v.tienen@gmail.com>
 *               2019 Tom van Dijk <tomvand@users.noreply.github.com>
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

/** @file modules/loggers/logger_file.c
 *  @brief File logger for Linux based autopilots
 */

#include "modules/mav_course_exercise/loggers/flight_logger_1.h"

#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>
#include "std.h"

#include "state.h"
#include "mcu_periph/sys_time.h"
#include "generated/airframe.h"
#include "generated/modules.h"

#include "autopilot.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"
#include "modules/nav/common_flight_plan.h"
#include "modules/datalink/datalink.h"
#include "modules/actuators/motor_mixing.h"

#ifdef MODULE_ORANGE_NO_GPS_ID
#include "modules/mav_course_exercise/no_gps_Mattias/orange_no_gps.h"
#endif

#ifdef MODULE_VIDEO_CAPTURE_ID
#include "modules/computer_vision/video_capture.h"
#endif


/** Set the default File logger path to the USB drive */
#ifndef LOGGER_FILE_PATH
#define LOGGER_FILE_PATH /home/andre/TU_Delft/AUT_MAVs/paparazzi/logs/logs
#endif

/** The file pointer */
static FILE *flight_log_file = NULL;

static bool prev_motors_on = false;
static uint32_t flight_counter = 0;


/** Write CSV header
 * Write column names at the top of the CSV file. Make sure that the columns
 * match those in logger_file_write_row! Don't forget the \n at the end of the
 * line.
 * @param file Log file pointer
 */
static void flight_logger_1_write_header(FILE *file)
{
  fprintf(file, "flight_id,sys_time_s,nav_block,");

  fprintf(file, "pos_x,pos_y,pos_z,");
  fprintf(file, "vel_x,vel_y,vel_z,");
  fprintf(file, "acc_bx,acc_by,acc_bz,");
  fprintf(file, "att_phi,att_theta,att_psi,");
  fprintf(file, "rate_p,rate_q,rate_r,");
  fprintf(file, "cmd_thrust,cmd_roll,cmd_pitch,cmd_yaw");

#ifdef MODULE_ORANGE_NO_GPS_ID
  fprintf(file, ",ong_nav_state,ong_edge_state,ong_obstacle_confidence");
  fprintf(file, ",ong_edge_warn_confidence,ong_edge_recover_confidence,ong_edge_turn_dir");
  fprintf(file, ",ong_edge_in_angle_deg,ong_edge_out_angle_deg");
  fprintf(file, ",ong_floor_ratio,ong_floor_centroid_x_frac,ong_floor_centroid_frac");
  fprintf(file, ",ong_speed_sp,ong_speed_ref,ong_cmd_vx,ong_cmd_vy,ong_vx_target,ong_vy_target,ong_heading_rate_cmd");
#endif
  fprintf(file, "\n");
}

static void flight_logger_1_write_row(FILE *file)
{
  struct NedCoor_f *pos = stateGetPositionNed_f();
  struct NedCoor_f *vel = stateGetSpeedNed_f();
  struct FloatVect3 *acc_b = stateGetAccelBody_f();
  struct FloatEulers *att = stateGetNedToBodyEulers_f();
  struct FloatRates *rates = stateGetBodyRates_f();

  fprintf(file, "%u,%.3f,%u", flight_counter, get_sys_time_float(), (unsigned)nav_block);
  fprintf(file, "%f,%f,%f,", pos->x, pos->y, pos->z);
  fprintf(file, "%f,%f,%f,", vel->x, vel->y, vel->z);
  fprintf(file, "%f,%f,%f,", acc_b->x, acc_b->y, acc_b->z);
  fprintf(file, "%f,%f,%f,", att->phi, att->theta, att->psi);
  fprintf(file, "%f,%f,%f,", rates->p, rates->q, rates->r);
  fprintf(file, "%d,%d,%d,%d",
          stabilization.cmd[COMMAND_THRUST],
          stabilization.cmd[COMMAND_ROLL],
          stabilization.cmd[COMMAND_PITCH],
          stabilization.cmd[COMMAND_YAW]);

#ifdef MODULE_ORANGE_NO_GPS_ID
  struct orange_no_gps_debug_state ong_dbg;
  orange_no_gps_get_debug_state(&ong_dbg);
  fprintf(file, ",%u,%u,%d",
          (unsigned)ong_dbg.nav_state, (unsigned)ong_dbg.edge_state, (int)ong_dbg.obstacle_confidence);
  fprintf(file, ",%d,%d,%.3f",
          (int)ong_dbg.edge_warn_confidence, (int)ong_dbg.edge_recover_confidence, ong_dbg.edge_turn_dir);
  fprintf(file, ",%.3f,%.3f", ong_dbg.edge_in_angle_deg, ong_dbg.edge_out_angle_deg);
  fprintf(file, ",%.6f,%.6f,%.6f",
          ong_dbg.floor_ratio, ong_dbg.floor_centroid_x_frac, ong_dbg.floor_centroid_frac);
  fprintf(file, ",%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
          ong_dbg.speed_sp, ong_dbg.speed_ref, ong_dbg.cmd_vx, ong_dbg.cmd_vy,
          ong_dbg.vx_target, ong_dbg.vy_target, ong_dbg.heading_rate_cmd);
#endif
  fprintf(file, "\n");
}

static void flight_logger_1_start(void)
{
  if (flight_log_file != NULL) {
    fclose(flight_log_file);
    flight_log_file = NULL;
  }

  if (access(STRINGIFY(FLIGHT_LOGGER_PATH), F_OK)) {
    char mkdir_cmd[512];
    snprintf(mkdir_cmd, sizeof(mkdir_cmd), "mkdir -p %s", STRINGIFY(LOGGER_FILE_PATH));
    if (system(mkdir_cmd) != 0) {
      printf("[flight_logger_1] Failed to create %s\n", STRINGIFY(LOGGER_FILE_PATH));
      return;
    }
  }

  char date_time[80];
  time_t now = time(NULL);
  struct tm tstruct = *localtime(&now);
  strftime(date_time, sizeof(date_time), "%Y%m%d-%H%M%S", &tstruct);

  char filename[640];
  uint32_t counter = 0;
  snprintf(filename, sizeof(filename), "%s/flight_%s.csv", STRINGIFY(LOGGER_FILE_PATH), date_time);
  while (access(filename, F_OK) == 0) {
    snprintf(filename, sizeof(filename), "%s/flight_%s_%05u.csv",
             STRINGIFY(LOGGER_FILE_PATH), date_time, counter++);
  }

  flight_log_file = fopen(filename, "w");
  if (flight_log_file == NULL) {
    printf("[flight_logger_1] Failed to open %s\n", filename);
    return;
  }

  flight_counter++;
  printf("[flight_logger_1] Logging flight %u to %s\n", flight_counter, filename);
  flight_logger_1_write_header(flight_log_file);
}

/** Stop the logger an nicely close the file */
static void flight_logger_1_stop_file(void)
{
  if (flight_log_file != NULL) {
    fflush(flight_log_file);
    fclose(flight_log_file);
    flight_log_file = NULL;
    printf("[flight_logger_1] Flight %u log closed\n", flight_counter);
  }
}


void flight_logger_1_init(void)
{

   // Create output folder if necessary
  if (access(STRINGIFY(LOGGER_FILE_PATH), F_OK)) {
    char save_dir_cmd[256];
    sprintf(save_dir_cmd, "mkdir -p %s", STRINGIFY(LOGGER_FILE_PATH));
    if (system(save_dir_cmd) != 0) {
      printf("[logger_file] Could not create log file directory %s.\n", STRINGIFY(LOGGER_FILE_PATH));
      return;
    }
  }

  prev_motors_on = autopilot_get_motors_on();
  if (prev_motors_on) {
    flight_logger_1_start();
  }
}

void flight_logger_1_periodic(void)
{
  const bool motors_on = autopilot_get_motors_on();

  if (motors_on && !prev_motors_on) {
    flight_logger_1_start();
#ifdef MODULE_VIDEO_CAPTURE_ID
    video_capture_start_capture();
#endif
  } else if (!motors_on && prev_motors_on) {
    flight_logger_1_stop_file();
#ifdef MODULE_VIDEO_CAPTURE_ID
    video_capture_stop_capture();
#endif
  }
  
  prev_motors_on = motors_on;

  if (motors_on && flight_log_file != NULL) {
    flight_logger_1_write_row(flight_log_file);
    fflush(flight_log_file);
  }
}