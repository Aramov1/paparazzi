#include "opticflow_attempt1.h"
#include <pthread.h>
#include "state.h"
#include "modules/core/abi.h"
#include "modules/pose_history/pose_history.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "cv.h"

#ifndef OPTICFLOW_FPS
#define OPTICFLOW_FPS 0
#endif

#ifdef OPTICFLOW_CAMERA2
#define ACTIVE_CAMERAS 2
#else
#define ACTIVE_CAMERAS 1
#endif

struct opticflow_t opticflow[ACTIVE_CAMERAS];
struct opticflow_result_t opticflow_result[ACTIVE_CAMERAS];
static bool opticflow_got_result[ACTIVE_CAMERAS];
static pthread_mutex_t opticflow_mutex = PTHREAD_MUTEX_INITIALIZER;

/* --- Helpers --- */
static void perform_obstacle_avoidance(struct opticflow_result_t *res)
{
  // 1. BRAKING LOGIC
  // If divergence is high, something is close and directly ahead.
  if (res->div_size > 0.15f) { 
    guidance_h_set_body_vel(0.0f, 0.0f);
  } 
  
  // 2. STEERING LOGIC (Simplified)
  // If we have significant side-flow while trying to fly straight, 
  // turn slightly to balance it out.
  float flow_deadband = 10.0f; // flow_der_x is in subpixels (int16)
  float steering_gain = 0.005f;

  if (abs(res->flow_der_x) > flow_deadband) {
    float yaw_rate_cmd = -(float)res->flow_der_x * steering_gain;
    guidance_h_set_heading_rate(yaw_rate_cmd);
  }
}

/* --- Core functions --- */
void opticflow_module_init(void) {
  for (int i = 0; i < ACTIVE_CAMERAS; i++) opticflow_got_result[i] = false;
  opticflow_calc_init(opticflow);
  cv_add_to_device(&OPTICFLOW_CAMERA, opticflow_module_calc, OPTICFLOW_FPS, 0);
}

struct image_t *opticflow_module_calc(struct image_t *img, uint8_t camera_id) {
  struct pose_t pose = get_rotation_at_timestamp(img->pprz_ts);
  img->eulers = pose.eulers;

  static struct opticflow_result_t temp;
  if (opticflow_calc_frame(&opticflow[camera_id], img, &temp)) {
    pthread_mutex_lock(&opticflow_mutex);
    opticflow_result[camera_id] = temp;
    opticflow_got_result[camera_id] = true;
    pthread_mutex_unlock(&opticflow_mutex);
  }
  return img;
}

void opticflow_module_run(void) {
  pthread_mutex_lock(&opticflow_mutex);
  for (int i = 0; i < ACTIVE_CAMERAS; i++) {
    if (opticflow_got_result[i]) {
      perform_obstacle_avoidance(&opticflow_result[i]);
      opticflow_got_result[i] = false;
    }
  }
  pthread_mutex_unlock(&opticflow_mutex);
}