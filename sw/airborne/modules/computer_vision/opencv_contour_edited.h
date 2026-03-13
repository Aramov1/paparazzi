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
 *
 */

/**
 * @file modules/computer_vision/opencv_contour.h
 * Detects contours of an obstacle used in the autonomous drone racing.
 */

#ifndef CONTOUR_LOWER_Y
#define CONTOUR_LOWER_Y 49
#endif
#ifndef CONTOUR_UPPER_Y
#define CONTOUR_UPPER_Y 94
#endif
#ifndef CONTOUR_LOWER_U
#define CONTOUR_LOWER_U 148
#endif
#ifndef CONTOUR_UPPER_U
#define CONTOUR_UPPER_U 199
#endif
#ifndef CONTOUR_LOWER_V
#define CONTOUR_LOWER_V 74
#endif
#ifndef CONTOUR_UPPER_V
#define CONTOUR_UPPER_V 123
#endif

struct contour_estimation {
  int   gate_detected;
  float contour_d_x;
  float contour_d_y;
  float contour_d_z;
  float contour_area;   // <-- add this line
};

struct contour_threshold {
  int lower_y, lower_u, lower_v;
  int upper_y, upper_u, upper_v;
};

extern struct contour_estimation cont_est;
extern struct contour_threshold cont_thres;

extern int show_threshold_overlay;
extern int gate_locked;
extern int gate_tracking;

#ifdef __cplusplus
extern "C" {
#endif

void find_contour(char *img, int width, int height);
void contour_reset_tracking(void);

#ifdef __cplusplus
}
#endif