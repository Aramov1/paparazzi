#include "obstacle_avoider.h"
#include "firmwares/rotorcraft/navigation.h"
#include "generated/airframe.h"
#include "generated/flight_plan.h"
#include "state.h"
#include "autopilot.h"
#include "modules/core/abi.h"
#include "math/pprz_algebra_float.h"
#include "modules/computer_vision/opticflow/size_divergence.h"
#include "lib/vision/image.h"
#include "modules/computer_vision/cv.h"
#include "modules/computer_vision/detect_contour.h"
#include "modules/computer_vision/opencv_contour.h"
#include "modules/computer_vision/cv_edge_detection.h"
#include BOARD_CONFIG

#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <pthread.h>
#include <string.h>

extern pthread_mutex_t opticflow_mutex;

struct contour_estimation contour_estimation;
extern struct contour_estimation cont_est;
extern pthread_mutex_t contour_mutex;

// GCS settings
float OA_WARNING_TTC = 6.0f;
float OA_SAFETY_TTC  = 1.5f;
float OA_MIN_FPS     = 5.0f;
float OA_MIN_DIVERGENCE = 0.003f;
int   OA_IMG_WIDTH   = 272;
int   OA_EDGE_OBSTACLE_THRESHOLD = 6000;  // Canny edge pixel count in center third triggering obstacle
int   OA_FLOOR_MIN_AREA          = 2000; // Min floor pixels in center third before triggering obstacle

extern struct opticflow_result_t opticflow_result[];

enum navigation_state_t {
  SAFE,
  OBSTACLE_FOUND,
  SEARCH_FOR_SAFE_HEADING,
  OUT_OF_BOUNDS
};

static enum navigation_state_t navigation_state = SEARCH_FOR_SAFE_HEADING;
static enum navigation_state_t prev_state = SEARCH_FOR_SAFE_HEADING;
static float heading_increment = 5.f;
static int16_t obstacle_free_confidence = 0;
static uint32_t log_counter = 0;
static bool prev_safety_crossed = false;
static bool prev_warning_crossed = false;

const int16_t max_trajectory_confidence = 5;
float maxDistance = 2.25f;



static uint8_t increase_nav_heading(float incrementDegrees)
{
  float new_heading = stateGetNedToBodyEulers_f()->psi + RadOfDeg(incrementDegrees);
  FLOAT_ANGLE_NORMALIZE(new_heading);
  nav.heading = new_heading;
  return false;
}

uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters)
{
  float heading  = stateGetNedToBodyEulers_f()->psi;

  // Now determine where to place the waypoint you want to go to
  new_coor->x = stateGetPositionEnu_i()->x + POS_BFP_OF_REAL(sinf(heading) * (distanceMeters));
  new_coor->y = stateGetPositionEnu_i()->y + POS_BFP_OF_REAL(cosf(heading) * (distanceMeters));
  return false;
}

uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor)
{
  waypoint_move_xy_i(waypoint, new_coor->x, new_coor->y);
  return false;
}

static uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters)
{
  // float heading = stateGetNedToBodyEulers_f()->psi;
  // struct EnuCoor_i new_coor;
  // new_coor.x = stateGetPositionEnu_i()->x + POS_BFP_OF_REAL(sinf(heading) * distanceMeters);
  // new_coor.y = stateGetPositionEnu_i()->y + POS_BFP_OF_REAL(cosf(heading) * distanceMeters);
  // waypoint_move_xy_i(waypoint, new_coor.x, new_coor.y);
  // return false;
  struct EnuCoor_i new_coor;
  calculateForwards(&new_coor, distanceMeters);
  moveWaypoint(waypoint, &new_coor);
  return false;
}

// Color overlay drawn on the image for RTP stream
static struct image_t *obstacle_avoider_draw(struct image_t *img, uint8_t camera_id)
{
  if (camera_id != 0) return img;

  // YUV422 color values
  // Green:  Y=150, U=44,  V=21
  // Yellow: Y=180, U=90,  V=20  (actually more like Y=210, U=90, V=150)
  // Red:    Y=76,  U=84,  V=255

  uint8_t y, u, v;

  switch (navigation_state) {
    case OBSTACLE_FOUND:
    case SEARCH_FOR_SAFE_HEADING:
      // Yellow - warning
      y = 210; u = 90; v = 150;
      break;
    case OUT_OF_BOUNDS:
      // Red - danger
      y = 76; u = 84; v = 255;
      break;
    case SAFE:
    default:
      // Green - all clear
      y = 150; u = 44; v = 21;
      break;
  }

  // Also override with TTC-based color regardless of state
  // Pull latest result directly
  struct opticflow_result_t *result = &opticflow_result[0];
  if (result->fps >= OA_MIN_FPS &&
      result->tracked_cnt >= 4 &&
      fabsf(result->div_size) > OA_MIN_DIVERGENCE) {
    float ttc = 1.0f / (fabsf(result->div_size) * result->fps);
    if (ttc < OA_SAFETY_TTC) {
      // Red
      y = 76; u = 84; v = 255;
    } else if (ttc < OA_WARNING_TTC) {
      // Yellow
      y = 210; u = 90; v = 150;
    } else {
      // Green
      y = 150; u = 44; v = 21;
    }
  }

  // Draw a border around the image, 20 pixels thick
  uint16_t border = 20;
  uint8_t *buf = img->buf;

  for (uint16_t y_px = 0; y_px < img->h; y_px++) {
    for (uint16_t x_px = 0; x_px < img->w; x_px++) {
      if (y_px < border || y_px > img->h - border ||
          x_px < border || x_px > img->w - border) {
        // YUV422: every 2 pixels share U and V
        // buffer layout: U Y1 V Y2 per 2 pixels
        uint32_t offset = (y_px * img->w + x_px);
        if (x_px % 2 == 0) {
          buf[offset * 2]     = u;   // U
          buf[offset * 2 + 1] = y;   // Y1
          buf[offset * 2 + 2] = v;   // V
          buf[offset * 2 + 3] = y;   // Y2
        }
      }
    }
  }

  return img;
}

void obstacle_avoider_init(void)
{
  // navigation_state = SAFE;
  // heading_increment = 5.f;
  // obstacle_free_confidence = 0;
  // log_counter = 0;

  // Register image callback for RTP overlay
  //cv_add_to_device(&front_camera, obstacle_avoider_draw, 0, 0);
}

void obstacle_avoider_run(void)
{
  if (!autopilot_in_flight()) return;

  log_counter++;

  // --- Read opticflow (thread-safe copy) ---
  pthread_mutex_lock(&opticflow_mutex);
  struct opticflow_result_t local_result = opticflow_result[0];
  struct flow_t *local_vectors = NULL;
  if (local_result.flow_vectors != NULL && local_result.flow_vector_count > 0) {
    local_vectors = malloc(sizeof(struct flow_t) * local_result.flow_vector_count);
    if (local_vectors != NULL) {
      memcpy(local_vectors, local_result.flow_vectors,
             sizeof(struct flow_t) * local_result.flow_vector_count);
    }
  }
  pthread_mutex_unlock(&opticflow_mutex);

  // --- Read tree detection (thread-safe copy) ---
  pthread_mutex_lock(&contour_mutex);
  contour_estimation.contour_d_x = cont_est.contour_d_x;
  contour_estimation.contour_d_y = cont_est.contour_d_y;
  contour_estimation.contour_d_z = cont_est.contour_d_z;
  pthread_mutex_unlock(&contour_mutex);

  // --- Read edge detection and floor area (int reads, single camera writer) ---
  int local_edge_left   = edge_count_left;
  int local_edge_center = edge_count_center;
  int local_edge_right  = edge_count_right;
  int local_floor_left  = floor_area_left;
  int local_floor_center = floor_area_center;
  int local_floor_right = floor_area_right;

  // ==========================================================================
  // OBSTACLE DETECTION + TURN DIRECTION VOTE
  // ==========================================================================
  bool obstacle_detected = false;
  float ttc = 0.f;
  float speed_factor = 1.0f;
  int turn_vote = 0;  // positive = turn right, negative = turn left

  struct opticflow_result_t *result = &local_result;

  // --- Signal 1: Opticflow TTC (primary) ---
  if (result->fps >= OA_MIN_FPS &&
      result->tracked_cnt >= 4 &&
      local_vectors != NULL &&
      fabsf(result->div_size) > OA_MIN_DIVERGENCE) {

    ttc = 1.0f / (fabsf(result->div_size) * result->fps);

    if (ttc < OA_SAFETY_TTC) {
      obstacle_detected = true;
      speed_factor = 0.0f;
      if (!prev_safety_crossed) {
        VERBOSE_PRINT("!!! SAFETY: TTC=%.2fs\n", ttc);
      }
      prev_safety_crossed  = true;
      prev_warning_crossed = true;
    } else if (ttc < OA_WARNING_TTC) {
      obstacle_detected = true;
      speed_factor = (ttc - OA_SAFETY_TTC) / (OA_WARNING_TTC - OA_SAFETY_TTC);
      Bound(speed_factor, 0.0f, 1.0f);
      if (!prev_warning_crossed) {
        VERBOSE_PRINT("WARNING: TTC=%.2fs speed=%.2f\n", ttc, speed_factor);
      }
      prev_warning_crossed = true;
      prev_safety_crossed  = false;
    } else {
      prev_safety_crossed  = false;
      prev_warning_crossed = false;
      if (log_counter % 30 == 0) {
        VERBOSE_PRINT("TTC nominal: %.2fs\n", ttc);
      }
    }

    // 3-region divergence → turn direction vote (left vs right, ignore center for direction)
    int third = OA_IMG_WIDTH / 3;
    float div_left  = get_divergence_region(local_vectors, local_result.flow_vector_count,
                                            30, 0,       third,        local_result.subpixel_factor);
    float div_right = get_divergence_region(local_vectors, local_result.flow_vector_count,
                                            30, 2*third, OA_IMG_WIDTH, local_result.subpixel_factor);
    float abs_left  = fabsf(div_left);
    float abs_right = fabsf(div_right);

    if (abs_right > abs_left) turn_vote--;   // more divergence right → turn left
    else                      turn_vote++;   // more divergence left  → turn right

    VERBOSE_PRINT("OF: L=%.4f R=%.4f vote=%d\n", abs_left, abs_right, turn_vote);

  } else {
    prev_safety_crossed  = false;
    prev_warning_crossed = false;
  }

  // --- Signal 2: Edge count (catches low-texture walls the opticflow misses) ---
  if (local_edge_center > OA_EDGE_OBSTACLE_THRESHOLD) {
    obstacle_detected = true;
    // Only vote on direction if one side is clearly different from the other
    int edge_diff = abs(local_edge_right - local_edge_left);
    if (edge_diff > OA_EDGE_OBSTACLE_THRESHOLD / 4) {
      if (local_edge_right > local_edge_left) turn_vote--;
      else                                    turn_vote++;
    }
    VERBOSE_PRINT("EDGE: L=%d C=%d R=%d diff=%d vote=%d\n",
                  local_edge_left, local_edge_center, local_edge_right, edge_diff, turn_vote);
  }

  // --- Signal 3: Floor area (catches obstacles/walls that occlude the floor) ---
  if (local_floor_center < OA_FLOOR_MIN_AREA) {
    obstacle_detected = true;
    // Only vote on direction if one side has meaningfully more floor than the other.
    // If both sides are equally empty (corner situation) the signal is noise — skip it.
    int floor_diff = abs(local_floor_right - local_floor_left);
    if (floor_diff > OA_FLOOR_MIN_AREA / 2) {
      if (local_floor_right > local_floor_left) turn_vote++;
      else                                      turn_vote--;
    }
    VERBOSE_PRINT("FLOOR: L=%d C=%d R=%d diff=%d vote=%d\n",
                  local_floor_left, local_floor_center, local_floor_right, floor_diff, turn_vote);
  }

  // --- Signal 4: Tree detection (direction hint only — does NOT trigger obstacle) ---
  if (contour_estimation.contour_d_x >= 0.0f) {
    if (contour_estimation.contour_d_y > 0.0f) turn_vote--;  // tree on right → turn left
    else                                        turn_vote++;  // tree on left  → turn right
    VERBOSE_PRINT("TREE hint: dy=%.2f vote=%d\n", contour_estimation.contour_d_y, turn_vote);
  }

  // Commit heading increment only while in SAFE state. Once the drone is turning
  // (SEARCH_FOR_SAFE_HEADING or OUT_OF_BOUNDS) keep the direction it already chose
  // so noisy signals cannot flip it mid-rotation.
  if (navigation_state == SAFE) {
    if (turn_vote == 0) {
      heading_increment = (rand() % 2 == 0) ? 5.f : -5.f;
    } else {
      heading_increment = (turn_vote > 0) ? 5.f : -5.f;
    }
  }

  // ==========================================================================
  // CONFIDENCE COUNTER
  // ==========================================================================
  if (!obstacle_detected) {
    obstacle_free_confidence++;
  } else {
    obstacle_free_confidence -= 3;
  }
  Bound(obstacle_free_confidence, 0, max_trajectory_confidence);

  float moveDistance = fminf(maxDistance, 0.2f * obstacle_free_confidence) * speed_factor;

  // ==========================================================================
  // STATE MACHINE
  // ==========================================================================
  switch (navigation_state) {

    case SAFE:
      moveWaypointForward(WP_TRAJECTORY, 1.5f * moveDistance);

      if (!InsideObstacleZone(WaypointX(WP_TRAJECTORY), WaypointY(WP_TRAJECTORY))) {
        VERBOSE_PRINT("Out of bounds\n");
        navigation_state = OUT_OF_BOUNDS;
      } else if (obstacle_free_confidence == 0) {
        VERBOSE_PRINT("Obstacle - turning %s (vote=%d)\n",
                      heading_increment > 0 ? "RIGHT" : "LEFT", turn_vote);
        navigation_state = OBSTACLE_FOUND;
      } else {
        moveWaypointForward(WP_GOAL, moveDistance);
      }
      break;

    case SEARCH_FOR_SAFE_HEADING:
      increase_nav_heading(heading_increment);
      if (obstacle_free_confidence >= 2) {
        navigation_state = SAFE;
        VERBOSE_PRINT("Path clear - resuming\n");
      }
      break;

    case OBSTACLE_FOUND:
      waypoint_move_here_2d(WP_GOAL);
      waypoint_move_here_2d(WP_TRAJECTORY);
      VERBOSE_PRINT("Stopped - searching for safe heading\n");
      navigation_state = SEARCH_FOR_SAFE_HEADING;
      break;

    case OUT_OF_BOUNDS:
      increase_nav_heading(heading_increment);
      moveWaypointForward(WP_TRAJECTORY, 1.5f);
      waypoint_move_here_2d(WP_GOAL);
      if (InsideObstacleZone(WaypointX(WP_TRAJECTORY), WaypointY(WP_TRAJECTORY))) {
        obstacle_free_confidence = 0;
        navigation_state = SEARCH_FOR_SAFE_HEADING;
        VERBOSE_PRINT("Back inside arena\n");
      }
      break;

    default:
      break;
  }

  if (navigation_state != prev_state) {
    VERBOSE_PRINT("STATE: %d -> %d\n", prev_state, navigation_state);
    prev_state = navigation_state;
  }

  if (local_vectors != NULL) {
    free(local_vectors);
  }
}