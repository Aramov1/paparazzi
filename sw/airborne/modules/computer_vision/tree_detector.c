#include "modules/computer_vision/tree_detector.h"
#include "modules/computer_vision/cv.h"

#ifndef TREE_DETECTOR_FPS
#define TREE_DETECTOR_FPS 0
#endif
PRINT_CONFIG_VAR(TREE_DETECTOR_FPS)

#ifdef __cplusplus
extern "C" {
#endif
void tree_detection_process(char *buf, int width, int height, int camera_id);
#ifdef __cplusplus
}
#endif

static struct image_t *tree_detector_cb(struct image_t *img, uint8_t camera_id)
{
  if (img && img->type == IMAGE_YUV422) {
    tree_detection_process((char *)img->buf, img->w, img->h, camera_id);
  }
  return NULL;
}

void tree_detector_init(void)
{
  cv_add_to_device(&TREE_DETECTOR_CAMERA, tree_detector_cb, TREE_DETECTOR_FPS, 0);
}