/*
 * @file modules/flip_bebop/guidance_flip.c
 * @author Jose Cunha
 *  Modification of firmwares/rotorcraft/guidance/guidance_flip.c, tuned to the Parrot Bebop drone. Provides open-loop
 *  guidance for making a rolling flip. Also includes modifications to be better utilised as a module which can be run
 *  at the click of a button defined in the flightplan by defining the following block:
 *
 *  <block name="FLIP">
      <call_once fun="guidance_flip_enter()"/>
      <while cond="!guidance_flip_finished()"/>
      <exception cond="guidance_flip_finished()" deroute="Standby"/>
    </block>
 *
 * Required modification to airborne/firmwares/rotorcraft/autopilot_static.c to overwrite autopilot attitude commands
 *
 * After finishing flip, it restores the heading that was set prior to the flip, as well as the prior autopilot mode.
 * Use it with caution!
 */

#include "modules/flip_bebop/guidance_flip.h"

#include "autopilot.h"
#include "firmwares/rotorcraft/autopilot_firmware.h"
#include "firmwares/rotorcraft/navigation.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_rc_setpoint.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"

#ifndef STOP_ROLL_CMD_ANGLE
#define STOP_ROLL_CMD_ANGLE 25.0
#endif
#ifndef FIRST_THRUST_DURATION
#define FIRST_THRUST_DURATION 0.25
#endif
#ifndef FIRST_THRUST_LEVEL
#define FIRST_THRUST_LEVEL 9200
#endif
#ifndef ROLL_THRUST_LEVEL
#define ROLL_THRUST_LEVEL 5200
#endif
#ifndef COAST_THRUST_LEVEL
#define COAST_THRUST_LEVEL 4800
#endif
#ifndef FINAL_THRUST_LEVEL
#define FINAL_THRUST_LEVEL 8600
#endif

uint32_t flip_counter;            /* Loop counter used to build a fixed-point timer. */
uint8_t flip_state;               /* State machine index for the flip phases. */
bool flip_running;                /* True while the flip state machine sends commands. */
bool flip_finished_flag;          /* True when a flip completes; False when entering flip. */
int32_t heading_save;             /* Heading snapshot to restore after the flip. */
uint8_t autopilot_mode_old;       /* Autopilot mode to restore on exit. */
struct Int32Vect2 flip_cmd_earth; /* Zeroed earth-frame command during recovery. */

// initialise variables
void guidance_flip_init(void)
{
  flip_counter = 0;
  flip_state = 0;
  flip_running = false;
  flip_finished_flag = false;
  heading_save = 0;
  autopilot_mode_old = AP_MODE_NAV;
  flip_cmd_earth.x = 0;
  flip_cmd_earth.y = 0;
}

// function to start flip (e.g. when FLIP button is pressed in GCS)
void guidance_flip_enter(void)
{
  flip_counter = 0;
  flip_state = 0;
  flip_running = true;
  flip_finished_flag = false;
  heading_save = stabilization_attitude_get_heading_i();
  autopilot_mode_old = autopilot_get_mode();
}

// detect when flip is finised to return to normal flight in flightplan
bool guidance_flip_finished(void)
{
  if (flip_finished_flag) {
    flip_finished_flag = false;
    return true;
  }
  return false;
}

// detect when flip is active for autopilot_static.c, to select to apply thrust commands generated in this file
bool guidance_flip_active(void)
{
  return flip_running;
}

/* Generate the required thrust commands to perform a flip.
 * May require tuning depending on the nominal height at which the Bebop flies, so that it does not impact the ground
 * when flipping.
 */
bool guidance_flip_run(void)
{
  if (!flip_running) {
    return false;
  }

#if defined(COMMAND_ROLL) && defined(COMMAND_PITCH) && defined(COMMAND_YAW) 

  uint32_t timer;
  int32_t phi;
  static uint32_t timer_save = 0;

  /* Fixed-point timer in 2^12 seconds to compare against BFP_OF_REAL(). */
  timer = (flip_counter++ << 12) / PERIODIC_FREQUENCY;
  /* Current roll angle in body frame (BFP radians). */
  phi = stateGetNedToBodyEulers_i()->phi;

  /*
   * Flipping state machine is separated into 5 phases:
   *  1. Perfom initial upward boost with high thrust level
   *  2. Apply a strong rolling comman until roll angle reaches a specified threshold (STOP_ROLL_CMD_ANGLE)
   *  3. Zero roll command at a slightly lower thrust level until Bebop is in an inverted angle window
   *  4. Command a level attitude with the pre-flip saved heading at a high thrust level
   *  5. Exit flip state and restore autopilot mode
   */

  switch (flip_state) {
    case 0:
      flip_cmd_earth.x = 0;
      flip_cmd_earth.y = 0;
      stabilization.cmd[COMMAND_THRUST] = FIRST_THRUST_LEVEL; // Boost before roll
      timer_save = 0;

      if (timer > BFP_OF_REAL(FIRST_THRUST_DURATION, 12)) {
        flip_state++;
      }
      break;

    case 1:
      stabilization.cmd[COMMAND_ROLL]   = 9600; // Rolling command
      stabilization.cmd[COMMAND_PITCH]  = 0;
      stabilization.cmd[COMMAND_YAW]    = 0;
      stabilization.cmd[COMMAND_THRUST] = ROLL_THRUST_LEVEL;

      /* Stop the roll command after crossing the stop angle. */
      if (phi > ANGLE_BFP_OF_REAL(RadOfDeg(STOP_ROLL_CMD_ANGLE))) {
        flip_state++;
      }
      break;

    case 2:
      stabilization.cmd[COMMAND_ROLL]   = 0;
      stabilization.cmd[COMMAND_PITCH]  = 0;
      stabilization.cmd[COMMAND_YAW]    = 0;
      stabilization.cmd[COMMAND_THRUST] = COAST_THRUST_LEVEL;

      /* Wait until we pass the inverted window before recovery timing. */
      if (phi > ANGLE_BFP_OF_REAL(RadOfDeg(-110.0)) && phi < ANGLE_BFP_OF_REAL(RadOfDeg(STOP_ROLL_CMD_ANGLE))) {
        timer_save = timer;
        flip_state++;
      }
      break;

    case 3:
    {
      flip_cmd_earth.x = 0;
      flip_cmd_earth.y = 0;
      struct StabilizationSetpoint flip_sp = stab_sp_from_ltp_i(&flip_cmd_earth, heading_save);
      struct ThrustSetpoint flip_thrust = th_sp_from_thrust_i(FINAL_THRUST_LEVEL, THRUST_AXIS_Z);
      stabilization_attitude_run(autopilot_in_flight(), &flip_sp, &flip_thrust, stabilization.cmd);

      /* Hold recovery for a fixed time before exiting. */
      if ((timer - timer_save) > BFP_OF_REAL(0.5, 12)) {
        flip_state++;
      }
      break;
    }

    default:
      autopilot_mode_auto2 = autopilot_mode_old;
      autopilot_set_mode(autopilot_mode_old);
      nav_set_heading_rad(ANGLE_FLOAT_OF_BFP(heading_save));
      flip_running = false;
      flip_finished_flag = true;
      flip_counter = 0;
      timer_save = 0;
      flip_state = 0;

      stabilization.cmd[COMMAND_ROLL]   = 0;
      stabilization.cmd[COMMAND_PITCH]  = 0;
      stabilization.cmd[COMMAND_YAW]    = 0;
      stabilization.cmd[COMMAND_THRUST] = FIRST_THRUST_LEVEL;
      break;
  }
#else
  autopilot_set_mode(autopilot_mode_old);
  nav_set_heading_rad(ANGLE_FLOAT_OF_BFP(heading_save));
  flip_running = false;
  flip_finished_flag = true;
#endif
  return flip_running;
}
