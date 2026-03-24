/*
 * Obstacle sensor fusion module.
 *
 * Reads opticflow TTC, edge detection, floor area and tree contour signals.
 * Publishes a VISUAL_DETECTION ABI message consumed by waypoint_navigation.c:
 *
 *   quality  > 0  → obstacle detected
 *   quality == 0  → path clear
 *   pixel_x       → turn vote: positive = turn right, negative = turn left
 */

#include "obstacle_avoider.h"
#include "modules/core/abi.h"
#include "modules/computer_vision/cv_edge_detection.h"
#include "modules/cyberzoo_navigation/waypoint_navigation.h"
#include "modules/computer_vision/detect_contour.h"
#include "modules/computer_vision/opencv_contour.h"
#include "autopilot.h"
#include "math/pprz_algebra_float.h"
#include "modules/computer_vision/opticflow/size_divergence.h"

#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <pthread.h>
#include <string.h>

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
#endif

extern pthread_mutex_t opticflow_mutex;

struct contour_estimation contour_estimation;
extern struct contour_estimation cont_est;
extern pthread_mutex_t contour_mutex;

extern struct opticflow_result_t opticflow_result[];

// GCS-tunable thresholds
float OA_WARNING_TTC             = 6.0f;
float OA_SAFETY_TTC              = 1.5f;
float OA_MIN_FPS                 = 5.0f;
float OA_MIN_DIVERGENCE          = 0.003f;
int   OA_IMG_WIDTH               = 240;
float OA_REGION_MIN_DIVERGENCE   = 0.007f;
float OA_MIN_REGION_DIFF         = 0.05f;
int   OA_EDGE_OBSTACLE_THRESHOLD = 6000;
int   OA_FLOOR_MIN_AREA          = 2000;

// Per-sensor enable flags — defaults can be overridden from the airframe XML:
//   <define name="OA_USE_OPTICFLOW" value="0"/>
#ifndef OA_USE_OPTICFLOW
#define OA_USE_OPTICFLOW 1
#endif
#ifndef OA_USE_EDGE
#define OA_USE_EDGE 1
#endif
#ifndef OA_USE_FLOOR
#define OA_USE_FLOOR 1
#endif
#ifndef OA_USE_TREE
#define OA_USE_TREE 1
#endif

uint8_t oa_use_opticflow = OA_USE_OPTICFLOW;
uint8_t oa_use_edge      = OA_USE_EDGE;
uint8_t oa_use_floor     = OA_USE_FLOOR;
uint8_t oa_use_tree      = OA_USE_TREE;

// Unique sender ID — waypoint_navigation binds to ABI_BROADCAST so it
// receives from any sender without extra configuration.
#define OA_VISUAL_DETECTION_SENDER_ID 43

// Quality sent when obstacle detected — large enough to exceed navigation
// module's threshold (0.18 * camera_w * camera_h) at any camera resolution.
#define OA_OBSTACLE_QUALITY 100000

static float    oa_ttc        = 0.f;
static float    oa_div_left   = 0.f;
static float    oa_div_right  = 0.f;
static float    oa_div_size   = 0.f;
static uint16_t oa_tracked_cnt = 0;
static float    oa_contour_dy = 0.f;
static int32_t  oa_turn_vote  = 0;
static uint8_t  oa_obstacle   = 0;

#if PERIODIC_TELEMETRY
static void oa_telem_send(struct transport_tx *trans, struct link_device *dev)
{
  uint8_t nav_state_u8 = (uint8_t)navigation_state;
  pprz_msg_send_OA_STATUS(trans, dev, AC_ID,
    &oa_ttc, &oa_div_left, &oa_div_right,
    &oa_div_size, &oa_tracked_cnt,
    &edge_count_left, &edge_count_center, &edge_count_right,
    &floor_area_left, &floor_area_center, &floor_area_right,
    &oa_contour_dy, &oa_turn_vote, &oa_obstacle,
    &nav_state_u8, &obstacle_free_confidence);
}
#endif

void obstacle_avoider_init(void)
{
#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_OA_STATUS, oa_telem_send);
#endif
}

void obstacle_avoider_run(void)
{
  if (!autopilot_in_flight()) return;

  // --- Read opticflow (thread-safe copy) ---
  pthread_mutex_lock(&opticflow_mutex);
  struct opticflow_result_t local_result = opticflow_result[0];
  struct flow_t *local_vectors = NULL;
  if (local_result.flow_vectors != NULL && local_result.flow_vector_count > 0) {
    local_vectors = malloc(sizeof(struct flow_t) * local_result.flow_vector_count);
    if (local_vectors) {
      memcpy(local_vectors, local_result.flow_vectors,
             sizeof(struct flow_t) * local_result.flow_vector_count);
    }
  }
  pthread_mutex_unlock(&opticflow_mutex);

  // --- Update raw opticflow telemetry unconditionally ---
  oa_div_size    = local_result.div_size;
  oa_tracked_cnt = local_result.tracked_cnt;
  if (local_result.fps > 0.f && fabsf(local_result.div_size) > 1e-6f) {
    oa_ttc = 1.0f / (fabsf(local_result.div_size) * local_result.fps);
  } else {
    oa_ttc = 9999.f;
  }

  // --- Read tree/contour detection (thread-safe copy) ---
  pthread_mutex_lock(&contour_mutex);
  contour_estimation.contour_d_x = cont_est.contour_d_x;
  contour_estimation.contour_d_y = cont_est.contour_d_y;
  contour_estimation.contour_d_z = cont_est.contour_d_z;
  pthread_mutex_unlock(&contour_mutex);

  // --- Read edge/floor counts (thread-safe copy) ---
  int local_edge_left, local_edge_center, local_edge_right;
  int local_floor_left, local_floor_center, local_floor_right;
  pthread_mutex_lock(&edge_detection_mutex);
  local_edge_left    = edge_count_left;
  local_edge_center  = edge_count_center;
  local_edge_right   = edge_count_right;
  local_floor_left   = floor_area_left;
  local_floor_center = floor_area_center;
  local_floor_right  = floor_area_right;
  pthread_mutex_unlock(&edge_detection_mutex);

  bool obstacle_detected = false;
  int turn_vote = 0;  // positive = turn right, negative = turn left

  // --- Signal 1: Opticflow TTC + regional divergence for direction ---
  if (oa_use_opticflow) {
  if (local_result.fps >= OA_MIN_FPS &&
      local_result.tracked_cnt >= 4 &&
      local_vectors != NULL &&
      fabsf(local_result.div_size) > OA_MIN_DIVERGENCE) {

    float ttc = oa_ttc;  // already computed unconditionally above
    if (ttc < OA_WARNING_TTC) {
      obstacle_detected = true;
      VERBOSE_PRINT("TTC obstacle: %.2fs\n", ttc);
    }

    int third = OA_IMG_WIDTH / 3;
    float div_left  = get_divergence_region(local_vectors,
                                            local_result.flow_vector_count,
                                            50, 0, third,
                                            local_result.subpixel_factor);
    float div_right = get_divergence_region(local_vectors,
                                            local_result.flow_vector_count,
                                            50, 2 * third, OA_IMG_WIDTH,
                                            local_result.subpixel_factor);
    oa_div_left  = div_left;
    oa_div_right = div_right;

    if (fabsf(div_right) > OA_REGION_MIN_DIVERGENCE ||
        fabsf(div_left)  > OA_REGION_MIN_DIVERGENCE) {
      if (fabsf(div_right) > fabsf(div_left)) turn_vote--;  // obstacle right → turn left
      else                                    turn_vote++;  // obstacle left  → turn right
    }
  }
  } // oa_use_opticflow

  // --- Signal 2: Edge count ---
  if (oa_use_edge) {
  if (local_edge_center > OA_EDGE_OBSTACLE_THRESHOLD) {
    obstacle_detected = true;
    int edge_diff = abs(local_edge_right - local_edge_left);
    if (edge_diff > OA_EDGE_OBSTACLE_THRESHOLD / 4) {
      if (local_edge_right > local_edge_left) turn_vote--;
      else                                    turn_vote++;
    }
    VERBOSE_PRINT("EDGE obstacle: L=%d C=%d R=%d\n",
                  local_edge_left, local_edge_center, local_edge_right);
  }
  } // oa_use_edge

  // --- Signal 3: Floor area ---
  if (oa_use_floor) {
  if (local_floor_center < OA_FLOOR_MIN_AREA) {
    obstacle_detected = true;
    int floor_diff = abs(local_floor_right - local_floor_left);
    if (floor_diff > OA_FLOOR_MIN_AREA / 2) {
      if (local_floor_right > local_floor_left) turn_vote++;
      else                                      turn_vote--;
    }
    VERBOSE_PRINT("FLOOR obstacle: L=%d C=%d R=%d\n",
                  local_floor_left, local_floor_center, local_floor_right);
  }
  } // oa_use_floor

  // --- Signal 4: Tree/contour ---
  if (oa_use_tree) {
  if (contour_estimation.contour_d_x >= 0.0f) {
    obstacle_detected = true;
    if (contour_estimation.contour_d_y > 0.0f) turn_vote--;  // tree right → turn left
    else                                        turn_vote++;
    VERBOSE_PRINT("CONTOUR obstacle: dy=%.2f vote=%d\n",
                  contour_estimation.contour_d_y, turn_vote);
  }
  } // oa_use_tree

  // --- Update telemetry state ---
  oa_contour_dy = contour_estimation.contour_d_y;
  oa_turn_vote  = (int32_t)turn_vote;
  oa_obstacle   = obstacle_detected ? 1 : 0;

  // --- Publish to navigation module ---
  int32_t quality   = obstacle_detected ? OA_OBSTACLE_QUALITY : 0;
  int16_t direction = (int16_t)turn_vote;
  AbiSendMsgVISUAL_DETECTION(OA_VISUAL_DETECTION_SENDER_ID, direction, 0, 0, 0, quality, 0);

  if (local_vectors) {
    free(local_vectors);
  }
}
