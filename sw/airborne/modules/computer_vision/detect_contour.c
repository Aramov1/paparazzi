 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/computer_vision/detect_contour.c"
 * @author Roland Meertens and Peng Lu
 *
 */

#include <pthread.h>
#include "modules/computer_vision/cv.h"
#include "modules/computer_vision/detect_contour.h"
#include "modules/computer_vision/opencv_contour.h"

pthread_mutex_t contour_mutex;

#ifndef DETECT_CONTOUR_FPS
#define DETECT_CONTOUR_FPS 0       ///< Default FPS (zero means run at camera fps)
#endif
PRINT_CONFIG_VAR(DETECT_CONTOUR_FPS)

// Function
struct image_t *contour_func(struct image_t *img, uint8_t camera_id);
struct image_t *contour_func(struct image_t *img, uint8_t camera_id)
{
  if (img->type == IMAGE_YUV422) {
    // Call OpenCV (C++ from paparazzi C function)
    find_contour((char *) img->buf, img->w, img->h);
  }
  return img;
}

void detect_contour_init(void)
{
  pthread_mutex_init(&contour_mutex, NULL);
  cv_add_to_device(&DETECT_CONTOUR_CAMERA, contour_func, DETECT_CONTOUR_FPS, 0);
  // in the mavlab, bright
  cont_thres.lower_y = 16;  cont_thres.lower_u = 135; cont_thres.lower_v = 80;
  cont_thres.upper_y = 100; cont_thres.upper_u = 175; cont_thres.upper_v = 165;
  //
  // for cyberzoo: Y:12-95, U:129-161, V:80-165, turn white.
  //int y1=16;  int u1=129; int v1=80; % cyberzoo, dark
  //int y2=100; int u2=161; int v2=165;
}

