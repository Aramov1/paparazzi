/*
 * Copyright (C) C. De Wagter
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/computer_vision/cv_edge_detection.c"
 * Paparazzi module glue: registers edge detection with the camera framework.
 */

#include "modules/computer_vision/cv.h"
#include "modules/computer_vision/cv_edge_detection.h"
#include "modules/computer_vision/edge_detection.h"
#include <pthread.h>

pthread_mutex_t edge_detection_mutex;

#ifndef EDGE_DETECTION_FPS
#define EDGE_DETECTION_FPS 0       ///< Default FPS (zero means run at camera fps)
#endif
PRINT_CONFIG_VAR(EDGE_DETECTION_FPS)

struct image_t *edge_detection_func(struct image_t *img, uint8_t camera_id);
struct image_t *edge_detection_func(struct image_t *img, uint8_t camera_id)
{
  if (img->type == IMAGE_YUV422) {
    edge_detection_run((char *) img->buf, img->w, img->h);
  }
  return NULL;
}

void edge_detection_init(void)
{
  pthread_mutex_init(&edge_detection_mutex, NULL);
  cv_add_to_device(&EDGE_DETECTION_CAMERA, edge_detection_func, EDGE_DETECTION_FPS, 0);
}
