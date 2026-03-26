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
 * @file "modules/computer_vision/opencv_example.cpp"
 * @author C. De Wagter
 * A simple module showing what you can do with opencv on the bebop.
 */

#include "edge_detection.h"
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <cstdio>
#include <time.h>
#include <pthread.h>
#include "viz_select.h"

extern pthread_mutex_t edge_detection_mutex;

#ifndef EDGE_THRESHOLD
#define EDGE_THRESHOLD 40
#endif
int edge_thresh = EDGE_THRESHOLD;

#ifndef GREEN_THRESH_VALUE
#define GREEN_THRESH_VALUE 165
#endif
int green_thresh_value = GREEN_THRESH_VALUE;

#ifndef FLOOR_MARGIN
#define FLOOR_MARGIN 180
#endif
int floor_margin = FLOOR_MARGIN;

#ifndef FLOOR_MIN_PIXELS
#define FLOOR_MIN_PIXELS 500
#endif
int floor_min_pixels = FLOOR_MIN_PIXELS;

#ifndef EDGE_DRAW
#define EDGE_DRAW 0
#endif
int edge_draw = EDGE_DRAW;

int edge_count_left   = 0;
int edge_count_center = 0;
int edge_count_right  = 0;
int edge_count_total  = 0;
int floor_area_left   = 0;
int floor_area_center = 0;
int floor_area_right  = 0;

int flush_counter = 0;


static FILE *log_file = NULL;


int edge_detection_run(char *img, int width, int height)
{
#if EDGE_DETECTION_GRAYSCALE
  struct timespec t0, t1;
  //clock_gettime(CLOCK_MONOTONIC, &t0);

  if (!log_file) {
    log_file = fopen("/data/ftp/internal_000/edges.log", "a");
  }

  unsigned char *buf = (unsigned char *)img;
  int margin = floor_margin;

  // ── Step 1+2 FUSED: Green floor detection + horizontal Gaussian blur ──────
  int horizonRow = height;
  int leftBound  = width;
  int rightBound = 0;
  int local_fl_left = 0, local_fl_center = 0, local_fl_right = 0;
  int floor_third = width / 3;

  static uint8_t hblur[520 * 240];

  for (int r = 0; r < height; r++) {
    hblur[r * width + 0]       = buf[r * width * 2 + 1];
    hblur[r * width + 1]       = buf[r * width * 2 + 3];
    hblur[r * width + width-2] = buf[r * width * 2 + (width-2) * 2 + 1];
    hblur[r * width + width-1] = buf[r * width * 2 + (width-1) * 2 + 1];

    for (int c = 2; c < width - 2; c++) {
      int base = r * width * 2;
      uint8_t y_val = buf[base + c * 2 + 1];
      uint8_t u_val = buf[base + (c & ~1) * 2];
      uint8_t v_val = buf[base + (c & ~1) * 2 + 2];

      if (y_val > 100 && y_val < (uint8_t)green_thresh_value &&
          u_val < 122 && v_val < 158) {
        if (r < horizonRow) horizonRow = r;
        if (c < leftBound)  leftBound  = c;
        if (c > rightBound) rightBound = c;
        if      (c < floor_third)       local_fl_left++;
        else if (c < 2 * floor_third)   local_fl_center++;
        else                            local_fl_right++;
      }

      int sum = 1 * buf[base + (c-2) * 2 + 1]
              + 4 * buf[base + (c-1) * 2 + 1]
              + 6 * y_val
              + 4 * buf[base + (c+1) * 2 + 1]
              + 1 * buf[base + (c+2) * 2 + 1];
      hblur[r * width + c] = (uint8_t)(sum >> 4);
    }
  }

  // Fallback to full image if no green detected, or too few pixels to be a real floor
  int total_floor = local_fl_left + local_fl_center + local_fl_right;
  bool green_found = (leftBound <= rightBound) && (total_floor >= floor_min_pixels);
  if (!green_found) {
    local_fl_left = local_fl_center = local_fl_right = 99999;
  }
  pthread_mutex_lock(&edge_detection_mutex);
  floor_area_left   = local_fl_left;
  floor_area_center = local_fl_center;
  floor_area_right  = local_fl_right;
  pthread_mutex_unlock(&edge_detection_mutex);

  int lb        = green_found ? ((leftBound  > margin)        ? leftBound  - margin : 0)         : 0;
  int rb        = green_found ? ((rightBound + margin < width) ? rightBound + margin : width - 1) : width - 1;
  int row_start = green_found ? horizonRow : 0;

  // ── Step 2b: Vertical Gaussian blur ───────────────────────────────────────
  static uint8_t blur_buf[520 * 240];

  for (int c = lb; c <= rb; c++) {
    blur_buf[0 * width + c]          = hblur[0 * width + c];
    blur_buf[1 * width + c]          = hblur[1 * width + c];
    blur_buf[(height-2) * width + c] = hblur[(height-2) * width + c];
    blur_buf[(height-1) * width + c] = hblur[(height-1) * width + c];
  }

  for (int r = 2; r < height - 2; r++) {
    int c_start = (r >= row_start) ? lb : 0;
    int c_end   = (r >= row_start) ? rb : width - 1;
    for (int c = c_start; c <= c_end; c++) {
      int sum = 1 * hblur[(r-2) * width + c]
              + 4 * hblur[(r-1) * width + c]
              + 6 * hblur[ r    * width + c]
              + 4 * hblur[(r+1) * width + c]
              + 1 * hblur[(r+2) * width + c];
      blur_buf[r * width + c] = (uint8_t)(sum >> 4);
    }
  }

  // ── Step 3: Sobel gradient magnitude and direction ────────────────────────
  static uint8_t mag_buf[520 * 240];
  static uint8_t dir_buf[520 * 240];

  for (int r = row_start; r < height - 1; r++) {
    for (int c = lb; c <= rb; c++) {
      #define B(rr,cc) ((int)blur_buf[(rr) * width + (cc)])
      int gx = -B(r-1,c-1) + B(r-1,c+1)
               -2*B(r,c-1) + 2*B(r,c+1)
               -B(r+1,c-1) + B(r+1,c+1);
      int gy = -B(r-1,c-1) - 2*B(r-1,c) - B(r-1,c+1)
               +B(r+1,c-1) + 2*B(r+1,c) + B(r+1,c+1);
      #undef B
      int mag = abs(gx) + abs(gy);
      mag_buf[r * width + c] = (mag > 255) ? 255 : (uint8_t)mag;

      int ax = abs(gx), ay = abs(gy);
      if (ax >= ay * 2)           dir_buf[r * width + c] = 0;
      else if (ay >= ax * 2)      dir_buf[r * width + c] = 2;
      else if (gx * gy > 0)       dir_buf[r * width + c] = 1;
      else                        dir_buf[r * width + c] = 3;
    }
    mag_buf[r * width]             = 0;
    mag_buf[r * width + width - 1] = 0;
  }
  if (row_start > 0) memset(mag_buf,                    0, width);
  memset(mag_buf + (height-1)*width, 0, width);

  // ── Step 4+5 FUSED: NMS + double threshold ────────────────────────────────
  int low_thresh  = edge_thresh;
  int high_thresh = edge_thresh * 3;

  static uint8_t edge_buf[520 * 240];
  memset(edge_buf, 0, width * height);

  for (int r = row_start; r < height - 1; r++) {
    for (int c = lb; c <= rb; c++) {
      uint8_t m = mag_buf[r * width + c];
      if (m == 0) continue;

      uint8_t d = dir_buf[r * width + c];
      uint8_t m1, m2;
      switch (d) {
        case 0: m1 = mag_buf[r*width + c-1];         m2 = mag_buf[r*width + c+1];         break;
        case 1: m1 = mag_buf[(r-1)*width + c+1];     m2 = mag_buf[(r+1)*width + c-1];     break;
        case 2: m1 = mag_buf[(r-1)*width + c];       m2 = mag_buf[(r+1)*width + c];       break;
        default:m1 = mag_buf[(r-1)*width + c-1];     m2 = mag_buf[(r+1)*width + c+1];     break;
      }
      if (m < m1 || m < m2) continue;

      if (green_found) {
        int in_mask = (c >= lb && c <= rb);
        if (!in_mask) {
          int uv = r * width * 2 + (c & ~1) * 2;
          uint8_t y = buf[r * width * 2 + c * 2 + 1], u = buf[uv], v = buf[uv + 2];
          in_mask = (y > 100 && y < (uint8_t)green_thresh_value && u < 122 && v < 158);
        }
        if (!in_mask) continue;
      }

      if      (m >= high_thresh) edge_buf[r * width + c] = 255;
      else if (m >= low_thresh)  edge_buf[r * width + c] = 128;
    }
  }

  // ── Hysteresis: single forward pass ──────────────────────────────────────
  for (int r = row_start; r < height - 1; r++) {
    for (int c = lb; c <= rb; c++) {
      if (edge_buf[r*width+c] != 128) continue;
      if (edge_buf[(r-1)*width+c-1] == 255 || edge_buf[(r-1)*width+c] == 255 || edge_buf[(r-1)*width+c+1] == 255 ||
          edge_buf[ r   *width+c-1] == 255 ||                                    edge_buf[ r   *width+c+1] == 255 ||
          edge_buf[(r+1)*width+c-1] == 255 || edge_buf[(r+1)*width+c] == 255 || edge_buf[(r+1)*width+c+1] == 255)
        edge_buf[r*width+c] = 255;
    }
  }

  // ── Step 6: Cleanup weak edges + count ───────────────────────────────────
  int third = width / 3;
  int local_left = 0, local_center = 0, local_right = 0;

  for (int r = row_start; r < height; r++) {
    for (int c = lb; c <= rb; c++) {
      int i = r * width + c;
      if (edge_buf[i] == 128) {
        edge_buf[i] = 0;
      } else if (edge_buf[i] == 255) {
        if (VIZ_ACTIVE == VIZ_EDGE) {
          buf[r * width * 2 + c * 2 + 1] = 255;
        }
        if      (c < third)       local_left++;
        else if (c < 2 * third)   local_center++;
        else                      local_right++;
      }
    }
  }

  int local_total = local_left + local_center + local_right;
  pthread_mutex_lock(&edge_detection_mutex);
  edge_count_left   = local_left;
  edge_count_center = local_center;
  edge_count_right  = local_right;
  edge_count_total  = local_total;
  pthread_mutex_unlock(&edge_detection_mutex);

  //clock_gettime(CLOCK_MONOTONIC, &t1);
  long ms = (t1.tv_sec - t0.tv_sec) * 1000 + (t1.tv_nsec - t0.tv_nsec) / 1000000;

  if (log_file) {
    fprintf(log_file, "t=%ld.%03ld | left: %d, center: %d, right: %d, total: %d | time: %ldms\n",
            t1.tv_sec, t1.tv_nsec / 1000000,
            local_left, local_center, local_right, local_total, ms);
    if (++flush_counter >= 30) { fflush(log_file); flush_counter = 0; }
  }

#endif // EDGE_DETECTION_GRAYSCALE

  return 0;
}
