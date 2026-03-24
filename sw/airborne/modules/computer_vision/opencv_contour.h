/*
 * Copyright (C) Roland Meertens and Peng Lu
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
#ifndef OPENCV_CONTOUR_H
#define OPENCV_CONTOUR_H

#include <stdint.h>

/* ----------------------------------------------------------------
 *  Detection result — written by find_contour() each frame.
 *
 *  contour_d_x : estimated forward distance to tree (m); -1 = none
 *  contour_d_y : lateral offset, normalised [-1, +1]
 *  contour_d_z : vertical offset, normalised [-1, +1]
 *  n_trees     : number of accepted tree candidates this frame
 *  best_x/y    : pixel centre of the best (largest) tree
 *  best_w/h    : pixel size of the best tree bounding box
 *
 *  detect_contour.c uses best_x/y/w/h to draw the overlay without
 *  needing to re-run the detection.
 * ---------------------------------------------------------------- */
struct contour_estimation {
  float contour_d_x;
  float contour_d_y;
  float contour_d_z;
  int   n_trees;
  int   best_x;
  int   best_y;
  int   best_w;
  int   best_h;
};

struct contour_threshold {
  int lower_y, lower_u, lower_v;
  int upper_y, upper_u, upper_v;
};

extern struct contour_estimation cont_est_cv;
extern struct contour_threshold  cont_thres_cv;

/* ----------------------------------------------------------------
 *  Runtime-tunable parameters
 *  Defined (with defaults) in opencv_contour.c.
 *  Exposed to the GCS via opencv_contour.xml settings block.
 * ---------------------------------------------------------------- */

/* YUV colour threshold */
extern uint8_t  OPENCV_CONTOUR_Y_MIN;
extern uint8_t  OPENCV_CONTOUR_Y_MAX;
extern uint8_t  OPENCV_CONTOUR_U_MIN;
extern uint8_t  OPENCV_CONTOUR_U_MAX;
extern uint8_t  OPENCV_CONTOUR_V_MIN;
extern uint8_t  OPENCV_CONTOUR_V_MAX;

/* Morphology */
extern uint8_t  OPENCV_CONTOUR_MORPH_OPEN_RADIUS;
extern uint8_t  OPENCV_CONTOUR_MORPH_CLOSE_RADIUS;

/* Component area */
extern uint32_t OPENCV_CONTOUR_MIN_COMPONENT_AREA;
extern float    OPENCV_CONTOUR_MAX_COMPONENT_AREA_FRAC;

/* Bounding box size */
extern uint16_t OPENCV_CONTOUR_MIN_BOX_WIDTH;
extern uint16_t OPENCV_CONTOUR_MIN_BOX_HEIGHT;
extern float    OPENCV_CONTOUR_MAX_BOX_WIDTH_FRAC;
extern float    OPENCV_CONTOUR_MAX_BOX_HEIGHT_FRAC;

/* Shape */
extern float    OPENCV_CONTOUR_MIN_ASPECT_RATIO;
extern float    OPENCV_CONTOUR_MAX_ASPECT_RATIO;
extern float    OPENCV_CONTOUR_MIN_FILL_RATIO;

/* ----------------------------------------------------------------
 *  Main processing entry point
 * ---------------------------------------------------------------- */
extern void find_contour_cv(char *img, int width, int height);

#endif /* OPENCV_CONTOUR_H */
