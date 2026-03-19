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
int   OA_IMG_WIDTH               = 272;
float OA_REGION_MIN_DIVERGENCE   = 0.007f;
float OA_MIN_REGION_DIFF         = 0.05f;
int   OA_EDGE_OBSTACLE_THRESHOLD = 6000;
int   OA_FLOOR_MIN_AREA          = 2000;

// Unique sender ID — waypoint_navigation binds to ABI_BROADCAST so it
// receives from any sender without extra configuration.
#define OA_VISUAL_DETECTION_SENDER_ID 43

// Quality sent when obstacle detected — large enough to exceed navigation
// module's threshold (0.18 * camera_w * camera_h) at any camera resolution.
#define OA_OBSTACLE_QUALITY 100000

void obstacle_avoider_init(void) {}

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

  // --- Read tree/contour detection (thread-safe copy) ---
  pthread_mutex_lock(&contour_mutex);
  contour_estimation.contour_d_x = cont_est.contour_d_x;
  contour_estimation.contour_d_y = cont_est.contour_d_y;
  contour_estimation.contour_d_z = cont_est.contour_d_z;
  pthread_mutex_unlock(&contour_mutex);

  bool obstacle_detected = false;
  int turn_vote = 0;  // positive = turn right, negative = turn left

  // --- Signal 1: Opticflow TTC + regional divergence for direction ---
  if (local_result.fps >= OA_MIN_FPS &&
      local_result.tracked_cnt >= 4 &&
      local_vectors != NULL &&
      fabsf(local_result.div_size) > OA_MIN_DIVERGENCE) {

    float ttc = 1.0f / (fabsf(local_result.div_size) * local_result.fps);
    if (ttc < OA_WARNING_TTC) {
      obstacle_detected = true;
      VERBOSE_PRINT("TTC obstacle: %.2fs\n", ttc);
    }

    int third = OA_IMG_WIDTH / 3;
    float div_left  = get_divergence_region(local_result.flow_vectors,
                                            local_result.flow_vector_count,
                                            50, 0, third,
                                            local_result.subpixel_factor);
    float div_right = get_divergence_region(local_result.flow_vectors,
                                            local_result.flow_vector_count,
                                            50, 2 * third, OA_IMG_WIDTH,
                                            local_result.subpixel_factor);
    if (fabsf(div_right) > OA_REGION_MIN_DIVERGENCE ||
        fabsf(div_left)  > OA_REGION_MIN_DIVERGENCE) {
      if (fabsf(div_right) > fabsf(div_left)) turn_vote--;  // obstacle right → turn left
      else                                    turn_vote++;  // obstacle left  → turn right
    }
  }

  // --- Signal 2: Edge count ---
  if (edge_count_center > OA_EDGE_OBSTACLE_THRESHOLD) {
    obstacle_detected = true;
    int edge_diff = abs(edge_count_right - edge_count_left);
    if (edge_diff > OA_EDGE_OBSTACLE_THRESHOLD / 4) {
      if (edge_count_right > edge_count_left) turn_vote--;
      else                                    turn_vote++;
    }
    VERBOSE_PRINT("EDGE obstacle: L=%d C=%d R=%d\n",
                  edge_count_left, edge_count_center, edge_count_right);
  }

  // --- Signal 3: Floor area ---
  if (floor_area_center < OA_FLOOR_MIN_AREA) {
    obstacle_detected = true;
    int floor_diff = abs(floor_area_right - floor_area_left);
    if (floor_diff > OA_FLOOR_MIN_AREA / 2) {
      if (floor_area_right > floor_area_left) turn_vote++;
      else                                    turn_vote--;
    }
    VERBOSE_PRINT("FLOOR obstacle: L=%d C=%d R=%d\n",
                  floor_area_left, floor_area_center, floor_area_right);
  }

  // --- Signal 4: Tree/contour ---
  if (contour_estimation.contour_d_x >= 0.0f) {
    obstacle_detected = true;
    if (contour_estimation.contour_d_y > 0.0f) turn_vote--;  // tree right → turn left
    else                                        turn_vote++;
    VERBOSE_PRINT("CONTOUR obstacle: dy=%.2f vote=%d\n",
                  contour_estimation.contour_d_y, turn_vote);
  }

  // --- Publish to navigation module ---
  int32_t quality   = obstacle_detected ? OA_OBSTACLE_QUALITY : 0;
  int16_t direction = (int16_t)turn_vote;
  AbiSendMsgVISUAL_DETECTION(OA_VISUAL_DETECTION_SENDER_ID, direction, 0, 0, 0, quality, 0);

  if (local_vectors) {
    free(local_vectors);
  }
}
