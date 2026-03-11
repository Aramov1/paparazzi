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
#include BOARD_CONFIG

#include <math.h>
#include <stdlib.h>
#include <stdio.h>

// GCS settings

float OA_WARNING_TTC = 6.0f;
float OA_SAFETY_TTC = 1.5f;
float OA_MIN_FPS = 5.0f;
float OA_MIN_DIVERGENCE = 0.008f;
int OA_IMG_WIDTH = 272;
float oa_region_min_divergence = 0.007f;
float OA_MIN_REGION_DIFF = 0.05f;

extern struct opticflow_result_t opticflow_result[];

enum navigation_state_t {
  SAFE,
  OBSTACLE_FOUND,
  SEARCH_FOR_SAFE_HEADING,
  OUT_OF_BOUNDS
};

static enum navigation_state_t navigation_state = SEARCH_FOR_SAFE_HEADING;
static float heading_increment = 5.f;
static int16_t obstacle_free_confidence = 0;
static uint32_t log_counter = 0;

const int16_t max_trajectory_confidence = 5;
float maxDistance = 2.25f;

static uint8_t increase_nav_heading(float incrementDegrees)
{
  float new_heading = stateGetNedToBodyEulers_f()->psi + RadOfDeg(incrementDegrees);
  FLOAT_ANGLE_NORMALIZE(new_heading);
  nav.heading = new_heading;
  VERBOSE_PRINT("Adjusting heading to %f deg\n", DegOfRad(new_heading));
  return false;
}

uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters)
{
  float heading  = stateGetNedToBodyEulers_f()->psi;

  // Now determine where to place the waypoint you want to go to
  new_coor->x = stateGetPositionEnu_i()->x + POS_BFP_OF_REAL(sinf(heading) * (distanceMeters));
  new_coor->y = stateGetPositionEnu_i()->y + POS_BFP_OF_REAL(cosf(heading) * (distanceMeters));
  VERBOSE_PRINT("Calculated %f m forward position. x: %f  y: %f based on pos(%f, %f) and heading(%f)\n", distanceMeters,	
                POS_FLOAT_OF_BFP(new_coor->x), POS_FLOAT_OF_BFP(new_coor->y),
                stateGetPositionEnu_f()->x, stateGetPositionEnu_f()->y, DegOfRad(heading));
  return false;
}

uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor)
{
  VERBOSE_PRINT("Moving waypoint %d to x:%f y:%f\n", waypoint, POS_FLOAT_OF_BFP(new_coor->x),
                POS_FLOAT_OF_BFP(new_coor->y));
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

  struct opticflow_result_t *result = &opticflow_result[0];

  // DEBUG - print every frame so we can see exactly what's happening
  VERBOSE_PRINT("=== OA DEBUG === state=%d fps=%.1f tracked=%d div=%.6f flow_vectors=%s confidence=%d\n",
                navigation_state,
                result->fps,
                result->tracked_cnt,
                result->div_size,
                result->flow_vectors != NULL ? "OK" : "NULL",
                obstacle_free_confidence);

  // ---------- OBSTACLE DETECTION via TTC ----------
  float div_full = result->div_size;
  float ttc = 0.f;
  bool obstacle_detected = false;

  if (result->fps >= OA_MIN_FPS &&
      result->tracked_cnt >= 4 &&
      result->flow_vectors != NULL &&
      fabsf(div_full) > OA_MIN_DIVERGENCE) {
    ttc = 1.0f / (fabsf(div_full) * result->fps);

    if (ttc < OA_SAFETY_TTC) {
      VERBOSE_PRINT("!!! SAFETY THRESHOLD CROSSED: TTC=%.2fs (< %.2fs) - EMERGENCY STOP\n",
                    ttc, OA_SAFETY_TTC);
      obstacle_detected = true;
    } else if (ttc < OA_WARNING_TTC) {
      VERBOSE_PRINT("WARNING THRESHOLD CROSSED: TTC=%.2fs (< %.2fs) - STEERING\n",
                    ttc, OA_WARNING_TTC);
      obstacle_detected = true;
    } else {
      if (++log_counter % 20 == 0) {
        VERBOSE_PRINT("TTC nominal: %.2fs\n", ttc);
      }
    }
  } else {
    if (++log_counter % 20 == 0) {
      VERBOSE_PRINT("TTC not computed - fps=%.1f tracked=%d div=%.4f\n",
                    result->fps, result->tracked_cnt, div_full);
    }
  }

  // ---------- CONFIDENCE ----------
  if (!obstacle_detected) {
    obstacle_free_confidence++;
  } else {
    obstacle_free_confidence -= 3;
  }
  Bound(obstacle_free_confidence, 0, max_trajectory_confidence);

  float moveDistance = fminf(maxDistance, 0.2f * obstacle_free_confidence);

  // ---------- STATE MACHINE ----------
  switch (navigation_state) {

    case SAFE:
      moveWaypointForward(WP_TRAJECTORY, 1.5f * moveDistance);

      // Check bounds against actual drone position
      if (!InsideObstacleZone(WaypointX(WP_TRAJECTORY),WaypointY(WP_TRAJECTORY))) {
        VERBOSE_PRINT("Out of bounds detected - switching to OUT_OF_BOUNDS\n");
        navigation_state = OUT_OF_BOUNDS;

      } else if (obstacle_free_confidence == 0) {
        // Choose turn direction from divergence
        if (result->flow_vectors != NULL && result->flow_vector_count >= 2) {
          float div_left  = get_divergence_region(result->flow_vectors,
                                                  result->flow_vector_count,
                                                  50, 0, OA_IMG_WIDTH / 2,
                                                  result->subpixel_factor);
          float div_right = get_divergence_region(result->flow_vectors,
                                                  result->flow_vector_count,
                                                  50, OA_IMG_WIDTH / 2, OA_IMG_WIDTH,
                                                  result->subpixel_factor);

          VERBOSE_PRINT("DIVERGENCE DEBUG: pixel_left=%.6f pixel_right=%.6f tracked=%d\n",
                        div_left, div_right, result->flow_vector_count);

          float abs_left  = fabsf(div_left);
          float abs_right = fabsf(div_right);
          float region_diff = fabsf(abs_right - abs_left);
          bool left_significant  = abs_left  > oa_region_min_divergence;
          bool right_significant = abs_right > oa_region_min_divergence;
          bool asymmetric = region_diff > OA_MIN_REGION_DIFF;

          if (left_significant || right_significant && asymmetric) {
            if (left_significant && right_significant && result->flow_vector_count >= 8) {
              heading_increment = (abs_right > abs_left) ? -5.f : 5.f;
              VERBOSE_PRINT("Both significant: left=%.4f right=%.4f turning %s\n",
                            abs_left, abs_right,
                            heading_increment > 0 ? "RIGHT" : "LEFT");
            } else if (right_significant) {
              heading_increment = -5.f;
              VERBOSE_PRINT("Only right significant: %.4f - turning LEFT\n", abs_right);
            } else {
              heading_increment = 5.f;
              VERBOSE_PRINT("Only left significant: %.4f - turning RIGHT\n", abs_left);
            }
          } else {
            heading_increment = (rand() % 2 == 0) ? 5.f : -5.f;
            VERBOSE_PRINT("Neither side significant (left=%.4f right=%.4f) - random turn %s\n",
                          abs_left, abs_right,
                          heading_increment > 0 ? "RIGHT" : "LEFT");
          }
        } else {
          heading_increment = (rand() % 2 == 0) ? 5.f : -5.f;
          VERBOSE_PRINT("Not enough flow vectors - random turn %s\n",
                        heading_increment > 0 ? "RIGHT" : "LEFT");
        }

        if (ttc < OA_SAFETY_TTC && ttc > 0.f) {
          VERBOSE_PRINT("!!! EMERGENCY HOVER - TTC=%.2fs !!!\n", ttc);
        } else {
          VERBOSE_PRINT("Obstacle at TTC=%.2fs - turning %s\n",
                        ttc, heading_increment > 0 ? "RIGHT" : "LEFT");
        }

        navigation_state = OBSTACLE_FOUND;
      } else {
        moveWaypointForward(WP_GOAL, moveDistance);
      }
      break;

    case SEARCH_FOR_SAFE_HEADING:
      increase_nav_heading(heading_increment);
      //moveWaypointForward(WP_GOAL, 1.5f);  // ADD THIS - move goal at new heading

      if (obstacle_free_confidence >= 2) {
        navigation_state = SAFE;
        VERBOSE_PRINT("Path clear - resuming forward flight\n");
      }
      break;

    case OBSTACLE_FOUND:
      waypoint_move_here_2d(WP_GOAL);
      waypoint_move_here_2d(WP_TRAJECTORY);
      VERBOSE_PRINT("Obstacle found - stopping and searching for safe heading\n");
      navigation_state = SEARCH_FOR_SAFE_HEADING;
      break;

    case OUT_OF_BOUNDS:
      increase_nav_heading(heading_increment);
      moveWaypointForward(WP_TRAJECTORY, 1.5f);  // probe at new heading
      waypoint_move_here_2d(WP_GOAL);            // keep goal here while turning

      if (InsideObstacleZone(WaypointX(WP_TRAJECTORY), WaypointY(WP_TRAJECTORY))) {
        increase_nav_heading(heading_increment);
        obstacle_free_confidence = 0;
        navigation_state = SEARCH_FOR_SAFE_HEADING;
        VERBOSE_PRINT("Back inside arena - verifying path before resuming\n");
      } else {
        if (++log_counter % 20 == 0) {
          VERBOSE_PRINT("Still out of bounds - continuing to turn\n");
        }
      }
      break;

    default:
      break;
  }
}