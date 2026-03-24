/*
 * Copyright (C) 2016 Roland Meertens and Peng Lu
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file modules/computer_vision/c_contour_edited.h
 * Pure-C gate detector — no OpenCV dependency.
 * Drop-in replacement for opencv_contour_edited.h
 */

#ifndef C_CONTOUR_EDITED_H
#define C_CONTOUR_EDITED_H

#include <stdint.h>

struct contour_estimation {
  int   gate_detected;
  float contour_d_x;
  float contour_d_y;
  float contour_d_z;
  float contour_area;
  int   top_cy;   /* pixel Y of top blob centroid (smaller Y value in image) */
  int   bot_cy;   /* pixel Y of bottom blob centroid (larger Y value in image) */
};

struct contour_threshold {
  int lower_y, lower_u, lower_v;
  int upper_y, upper_u, upper_v;
};

extern struct contour_estimation cont_est;
extern struct contour_threshold  cont_thres;
extern int show_threshold_overlay;


void find_contour(char *img, int width, int height);
void contour_reset_tracking(void);

#endif /* C_CONTOUR_EDITED_H */