#include "modules/gate_navigator/gate_nav.h"
#include "firmwares/rotorcraft/navigation.h"
#include "generated/airframe.h"
#include "state.h"
#include "generated/flight_plan.h"
#include "math/pprz_algebra_float.h"
#include "modules/computer_vision/c_contour_edited.h"
#include "modules/computer_vision/opticflow/size_divergence.h"
#include "modules/computer_vision/opticflow/inter_thread_data.h"
#include <stdio.h>
#include <pthread.h>
#include <stdlib.h>
#include <string.h>

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

/* GCS-settable runtime controls */
#ifndef GATE_DETECTION_ENABLED
#define GATE_DETECTION_ENABLED 1
#endif
uint8_t gate_detection_enabled  = GATE_DETECTION_ENABLED;
uint8_t gate_show_overlay       = 0;
float   gate_ttc_safe_threshold = 2.5f;  /* minimum TTC at gate center [s] */

extern struct opticflow_result_t opticflow_result[];
extern pthread_mutex_t opticflow_mutex;

extern void contour_reset_tracking(void);
extern int show_threshold_overlay;

/* -----------------------------------------------------------------------
 * Heading / waypoint helpers (body-heading based, as in original code)
 * ----------------------------------------------------------------------- */
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

/* -----------------------------------------------------------------------
 * Abort: called by waypoint_navigation when exiting GATE_TRACKING
 * ----------------------------------------------------------------------- */
void gate_navigator_abort(void)
{
  nav_state            = SEARCH;
  gate_tracking        = 0;
  lost_counter         = 0;
  detection_confidence = 0;
  contour_reset_tracking();
  printf("[GATE_NAV] Aborted — returning to passive SEARCH\n");
}

/* -----------------------------------------------------------------------
 * Y-band divergence helper (mirrors get_divergence_region but on Y axis)
 * ----------------------------------------------------------------------- */
static float divergence_region_y(struct flow_t *vectors, int count, int n_samples,
                                  int y_min, int y_max, int subpixel_factor)
{
  struct flow_t filtered[count];  /* VLA — same pattern as get_divergence_region */
  int fc = 0;
  for (int i = 0; i < count; i++) {
    int py = vectors[i].pos.y / subpixel_factor;
    if (py >= y_min && py <= y_max) {
      filtered[fc++] = vectors[i];
    }
  }
  return get_size_divergence(filtered, fc, n_samples);
}

/* -----------------------------------------------------------------------
 * TTC safety check: 1 if gate center TTC >= gate_ttc_safe_threshold.
 * Divides image into three Y bands using gate blob centroids.
 * Fail-open when opticflow data is insufficient.
 * ----------------------------------------------------------------------- */
static int ttc_safe_to_cross(void)
{
  pthread_mutex_lock(&opticflow_mutex);
  struct opticflow_result_t local_of = opticflow_result[0];
  struct flow_t *local_vectors = NULL;
  if (local_of.flow_vectors != NULL && local_of.flow_vector_count > 0) {
    local_vectors = malloc(sizeof(struct flow_t) * local_of.flow_vector_count);
    if (local_vectors)
      memcpy(local_vectors, local_of.flow_vectors,
             sizeof(struct flow_t) * local_of.flow_vector_count);
  }
  pthread_mutex_unlock(&opticflow_mutex);

  if (local_vectors == NULL || local_of.flow_vector_count < 4 || local_of.fps < 5.0f) {
    if (local_vectors) free(local_vectors);
    printf("[GATE_NAV] TTC fail-open (cnt=%d fps=%.1f)\n",
           local_of.flow_vector_count, local_of.fps);
    return 1;
  }

  int spf   = (int)local_of.subpixel_factor;
  int cnt   = local_of.flow_vector_count;
  int top_y = cont_est.top_cy;
  int bot_y = cont_est.bot_cy;
  int img_h = 520;  /* MT9F002 OUTPUT_HEIGHT from airframe XML */

  float div_above  = divergence_region_y(local_vectors, cnt, 50, 0,     top_y, spf);
  float div_center = divergence_region_y(local_vectors, cnt, 50, top_y, bot_y, spf);
  float div_below  = divergence_region_y(local_vectors, cnt, 50, bot_y, img_h, spf);
  free(local_vectors);

  float ttc_center = (fabsf(div_center) > 1e-4f) ? (1.0f / fabsf(div_center)) : 9999.f;

  printf("[GATE_NAV] TTC: above=%.4f center(ttc=%.1fs) below=%.4f threshold=%.1fs -> %s\n",
         div_above, ttc_center, div_below, gate_ttc_safe_threshold,
         ttc_center >= gate_ttc_safe_threshold ? "SAFE" : "BLOCKED");

  return (ttc_center >= gate_ttc_safe_threshold) ? 1 : 0;
}

/* -----------------------------------------------------------------------
 * Init
 * ----------------------------------------------------------------------- */
void gate_navigator_init(void)
{
  nav_state            = SEARCH;
  lost_counter         = 0;
  cross_timer          = 0;
  filtered_y           = 0.f;
  detection_confidence = 0;
  locked_heading       = 0.f;
  gate_tracking        = 0;
  gate_show_overlay    = 0;
  /* gate_detection_enabled keeps compile-time/GCS value across init */
  contour_reset_tracking();
  printf("[GATE_NAV] Initialized (gate_detection_enabled=%d)\n", gate_detection_enabled);
}

/* -----------------------------------------------------------------------
 * Periodic (60 Hz)
 * ----------------------------------------------------------------------- */
void gate_navigator_periodic(void)
{
  if (!autopilot_in_flight()) return;

  /* Sync debug overlay wrapper */
  show_threshold_overlay = (int)gate_show_overlay;

  /* GCS disable guard */
  if (!gate_detection_enabled) {
    if (gate_tracking) {
      gate_navigator_abort();
      printf("[GATE_NAV] Disabled via GCS\n");
    }
    return;
  }

  static int dbg = 0;
  if (dbg++ % 60 == 0)
    printf("[GATE_NAV] alive state=%d conf=%d lost=%d fy=%.2f area=%.0f\n",
           nav_state, detection_confidence, lost_counter, filtered_y, cont_est.contour_area);

  /* --- Detection processing (vision always live) --- */
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

  /* --- SEARCH -> TRACK --- */
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

  /* --- State machine --- */
  switch (nav_state) {

    case SEARCH:
      /* Passive: waypoint_navigation drives the drone via perimeter following.
       * gate_nav only watches cont_est for gate detection; no heading changes. */
      break;

    case TRACK: {
      if (fresh_detection) {
        float psi = stateGetNedToBodyEulers_f()->psi;
        float target = psi + RadOfDeg(filtered_y * 30.0f);
        FLOAT_ANGLE_NORMALIZE(target);

        float diff = target - locked_heading;
        FLOAT_ANGLE_NORMALIZE(diff);
        locked_heading = locked_heading + 0.15f * diff;
        FLOAT_ANGLE_NORMALIZE(locked_heading);

        printf("[GATE_NAV] TRACK: psi=%.2f fy=%.2f target=%.2f locked=%.2f\n",
               psi, filtered_y, target, locked_heading);
      }

      nav.heading = locked_heading;
      if (fabsf(filtered_y) > 0.2f) {
        moveWaypointForward(WP_GOAL,       0.2f);
        moveWaypointForward(WP_TRAJECTORY, 0.3f);
      } else {
        moveWaypointForward(WP_GOAL,       2.0f);
        moveWaypointForward(WP_TRAJECTORY, 3.0f);
      }

      /* TRACK -> CROSS: area + alignment + TTC safety check */
      if (fresh_detection
          && cont_est.contour_area > CROSS_AREA_THRESHOLD
          && fabsf(filtered_y) < 0.15f
          && ttc_safe_to_cross()) {
        locked_heading = stateGetNedToBodyEulers_f()->psi;
        nav_state    = CROSS;
        cross_timer  = 0;
        lost_counter = 0;
        printf("[GATE_NAV] TRACK -> CROSS (area=%.0f, TTC safe)\n", cont_est.contour_area);
      }
      if (lost_counter > 120) {
        nav_state     = SEARCH;
        gate_tracking = 0;
        lost_counter  = 0;
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
        nav_state     = SEARCH;
        gate_tracking = 0;
        lost_counter  = 0;
        contour_reset_tracking();
        printf("[GATE_NAV] CROSS -> SEARCH (timeout)\n");
      }
      break;
    }
  }
}
