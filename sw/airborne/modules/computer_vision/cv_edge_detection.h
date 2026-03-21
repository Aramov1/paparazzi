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
 * @file "modules/computer_vision/cv_edge_detection.h"
 * Paparazzi module glue: registers edge detection with the camera framework.
 */

#ifndef CV_EDGE_DETECTION_H
#define CV_EDGE_DETECTION_H

extern void edge_detection_init(void);
extern int edge_thresh;
extern int green_thresh_value;
extern int floor_margin;
extern int floor_min_pixels;
extern int edge_draw;
extern int edge_count_left;
extern int edge_count_center;
extern int edge_count_right;
extern int edge_count_total;
extern int floor_area_left;
extern int floor_area_center;
extern int floor_area_right;

#endif
