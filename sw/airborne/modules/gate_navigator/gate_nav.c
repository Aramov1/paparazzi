#include "modules/gate_navigator/gate_nav.h"
#include "firmwares/rotorcraft/navigation.h"
#include "generated/airframe.h"
#include "state.h"
#include "generated/flight_plan.h"
#include "math/pprz_algebra_float.h"
#include "modules/computer_vision/opencv_contour_edited.h"
#include <stdio.h>

enum nav_state_t { SEARCH, TRACK, CROSS };
static enum nav_state_t nav_state = SEARCH;
static float filtered_y = 0.f;
#define LP 0.65f
static int lost_counter = 0;
static uint32_t cross_timer = 0;
static int detection_confidence = 0;
static float locked_heading = 0.f;
#define DETECTION_CONFIDENCE_THRESHOLD 6
#define CROSS_TIMEOUT    180    // ~2.5s at 60Hz
#define CROSS_AREA_THRESHOLD 9000.f  // px^2 — trigger crossing when close enough
int gate_tracking = 0;

extern void contour_reset_tracking(void);

static uint8_t increase_nav_heading(float incrementDegrees)
{
  float new_heading = stateGetNedToBodyEulers_f()->psi + RadOfDeg(incrementDegrees);
  FLOAT_ANGLE_NORMALIZE(new_heading);
  nav.heading = new_heading;
  return false;
}

static uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters)
{
  float heading = stateGetNedToBodyEulers_f()->psi;
  struct EnuCoor_i new_coor;
  new_coor.x = stateGetPositionEnu_i()->x + POS_BFP_OF_REAL(sinf(heading) * distanceMeters);
  new_coor.y = stateGetPositionEnu_i()->y + POS_BFP_OF_REAL(cosf(heading) * distanceMeters);
  waypoint_move_xy_i(waypoint, new_coor.x, new_coor.y);
  return false;
}

void gate_navigator_init(void)
{
  nav_state            = SEARCH;
  lost_counter         = 0;
  cross_timer          = 0;
  filtered_y           = 0.f;
  detection_confidence = 0;
  locked_heading       = 0.f;
  gate_tracking        = 0;
  contour_reset_tracking();
  printf("[GATE_NAV] Initialized\n");
}

void gate_navigator_periodic(void)
{
  if (!autopilot_in_flight()) return;
  if (nav_block != 6) return;

  static int dbg = 0;
  if (dbg++ % 60 == 0)
    printf("[GATE_NAV] alive state=%d conf=%d lost=%d fy=%.2f area=%.0f\n",
           nav_state, detection_confidence, lost_counter, filtered_y, cont_est.contour_area);

  // --- Detection processing (vision always live) ---
  int fresh_detection = cont_est.gate_detected;
  cont_est.gate_detected = 0;

  if (fresh_detection) {
    detection_confidence = DETECTION_CONFIDENCE_THRESHOLD;
    filtered_y = LP * cont_est.contour_d_y + (1.f - LP) * filtered_y;
    lost_counter = 0;
  } else {
    if (detection_confidence > 0) detection_confidence--;
    else lost_counter++;
  }

  int has_detection = (detection_confidence > 0);

  // --- SEARCH -> TRACK ---
  if (has_detection && nav_state == SEARCH) {
    nav_state = TRACK;
    gate_tracking = 1; 

    locked_heading = stateGetNedToBodyEulers_f()->psi + RadOfDeg(filtered_y * 30.0f);
    FLOAT_ANGLE_NORMALIZE(locked_heading);
    nav.heading = locked_heading;
    printf("[GATE_NAV] SEARCH -> TRACK fy=%.2f locked_hdg=%.2f\n",
           filtered_y, locked_heading);
  }

  if (lost_counter % 40 == 0 && lost_counter > 0) {
    const char *s[] = {"SEARCH", "TRACK", "CROSS"};
    printf("[GATE_NAV] State=%s lost=%d fy=%.2f\n", s[nav_state], lost_counter, filtered_y);
  }

  // --- State machine ---
  switch (nav_state) {

    case SEARCH: {
      if (!InsideObstacleZone(WaypointX(WP_TRAJECTORY), WaypointY(WP_TRAJECTORY))) {
        waypoint_move_here_2d(WP_GOAL);
        waypoint_move_here_2d(WP_TRAJECTORY);
        increase_nav_heading(5.0f);
      } else {
        increase_nav_heading(1.5f);
      }
      break;
    }

    case TRACK: {
      // Improvement 1+2: absolute heading target, reduced gain K=5
      if (fresh_detection) {
        float psi = stateGetNedToBodyEulers_f()->psi;
        float target = psi + RadOfDeg(filtered_y * 30.0f);
        FLOAT_ANGLE_NORMALIZE(target);
          
        // Fix: interpolate using angle difference to handle wrap-around
        float diff = target - locked_heading;
        FLOAT_ANGLE_NORMALIZE(diff);
        locked_heading = locked_heading + 0.15f * diff;
        FLOAT_ANGLE_NORMALIZE(locked_heading);
          
        printf("[GATE_NAV] TRACK: psi=%.2f fy=%.2f target=%.2f locked=%.2f\n",
            psi, filtered_y, target, locked_heading);
      }

      nav.heading = locked_heading;
      // Improvement 3+4: slow forward when misaligned, faster when centered
      if (fabsf(filtered_y) > 0.2f) {
        // Mostly rotate — creep forward slowly
        moveWaypointForward(WP_GOAL,       0.2f);
        moveWaypointForward(WP_TRAJECTORY, 0.3);
      } else {
        // Aligned — advance toward gate
        moveWaypointForward(WP_GOAL,       2.0f);
        moveWaypointForward(WP_TRAJECTORY, 3.0f);
      }

      // Improvement 5: use area threshold for CROSS trigger
      if (fresh_detection && cont_est.contour_area > CROSS_AREA_THRESHOLD && fabsf(filtered_y) < 0.15f) {
        locked_heading = stateGetNedToBodyEulers_f()->psi;
        nav_state    = CROSS;
        cross_timer  = 0;
        lost_counter = 0;
        printf("[GATE_NAV] TRACK -> CROSS (area=%.0f)\n", cont_est.contour_area);
      }
      if (lost_counter > 120) {
        nav_state    = SEARCH;
        gate_tracking = 0;
        lost_counter = 0;
        contour_reset_tracking();
        printf("[GATE_NAV] TRACK -> SEARCH (lost)\n");
      }
      break;
    }

    case CROSS: {
      cross_timer++;
      nav.heading = locked_heading;
      moveWaypointForward(WP_TRAJECTORY, 3.0f);
      moveWaypointForward(WP_GOAL,       2.0f);

      if (cross_timer > CROSS_TIMEOUT) {
        nav_state    = SEARCH;
        gate_tracking = 0;
        lost_counter = 0;
        contour_reset_tracking();
        printf("[GATE_NAV] CROSS -> SEARCH (timeout)\n");
      }
      break;
    }
  }
}