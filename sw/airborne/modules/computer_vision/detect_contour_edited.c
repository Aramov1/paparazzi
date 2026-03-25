#include "modules/computer_vision/cv.h"
#include "modules/computer_vision/detect_contour_edited.h"
#include "modules/computer_vision/c_contour_edited.h"


#ifndef DETECT_CONTOUR_EDITED_FPS
#define DETECT_CONTOUR_EDITED_FPS 0
#endif

struct image_t *contour_edited_func(struct image_t *img, uint8_t camera_id);
struct image_t *contour_edited_func(struct image_t *img, uint8_t camera_id)
{
  if (img->type == IMAGE_YUV422) {
    find_contour((char *) img->buf, img->w, img->h);
  }
  return img;
}

void detect_contour_edited_init(void)
{
  cv_add_to_device(&DETECT_CONTOUR_EDITED_CAMERA, contour_edited_func, DETECT_CONTOUR_EDITED_FPS, 0);
}