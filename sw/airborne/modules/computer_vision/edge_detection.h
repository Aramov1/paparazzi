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
 * @file "modules/computer_vision/edge_detection.h"
 * Edge detection using manual Canny pipeline on YUV422 camera frames.
 */

#ifndef EDGE_DETECTION_H
#define EDGE_DETECTION_H

#ifdef __cplusplus
extern "C" {
#endif

int edge_detection_run(char *img, int width, int height);
extern int edge_thresh;
extern int green_thresh_value;
extern int floor_margin;

#ifdef __cplusplus
}
#endif

#endif
