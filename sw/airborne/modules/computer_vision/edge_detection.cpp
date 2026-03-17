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


#ifndef EDGE_THRESHOLD
#define EDGE_THRESHOLD 30
#endif
int edge_thresh = EDGE_THRESHOLD;

#ifndef GREEN_THRESH_VALUE
#define GREEN_THRESH_VALUE 160
#endif
int green_thresh_value = GREEN_THRESH_VALUE;

#ifndef FLOOR_MARGIN
#define FLOOR_MARGIN 180
#endif
int floor_margin = FLOOR_MARGIN;
int edge_draw = 1;
int edge_count_left   = 0;
int edge_count_center = 0;
int edge_count_right  = 0;
int edge_count_total  = 0;


int edge_detection_run(char *img, int width, int height)
{
#if EDGE_DETECTION_GRAYSCALE
  struct timespec t0, t1;
  clock_gettime(CLOCK_MONOTONIC, &t0);
  unsigned char *buf = (unsigned char *)img;
  int margin = floor_margin;

  // ── Step 1+2: Green floor detection fused with horizontal Gaussian blur ─────
  // Both passes read every pixel once, so we combine them into a single loop.
  // UYVY format: Y at c*2+1, U at (c&~1)*2, V at (c&~1)*2+2
  // Gaussian kernel [1,4,6,4,1]/16 applied horizontally to Y channel.
  int horizonRow = height;
  int leftBound  = width;
  int rightBound = 0;

  static uint8_t hblur[640 * 640];
  static uint8_t blur_buf[640 * 640];

  for (int r = 0; r < height; r++) {
    // Border pixels (mirrored, no blur)
    hblur[r * width + 0]       = buf[r * width * 2 + 1];
    hblur[r * width + 1]       = buf[r * width * 2 + 3];
    hblur[r * width + width-2] = buf[r * width * 2 + (width-2) * 2 + 1];
    hblur[r * width + width-1] = buf[r * width * 2 + (width-1) * 2 + 1];

    for (int c = 2; c < width - 2; c++) {
      int base = r * width * 2;
      uint8_t y_val = buf[base + c * 2 + 1];
      uint8_t u_val = buf[base + (c & ~1) * 2];
      uint8_t v_val = buf[base + (c & ~1) * 2 + 2];

      // Green detection
      if (y_val > 30 && y_val < (uint8_t)green_thresh_value &&
          u_val < 115 && v_val < 120) {
        if (r < horizonRow) horizonRow = r;
        if (c < leftBound)  leftBound  = c;
        if (c > rightBound) rightBound = c;
      }

      // Horizontal blur (reads Y of neighbours, already in cache from this row)
      int sum = 1 * buf[base + (c-2) * 2 + 1]
              + 4 * buf[base + (c-1) * 2 + 1]
              + 6 * y_val
              + 4 * buf[base + (c+1) * 2 + 1]
              + 1 * buf[base + (c+2) * 2 + 1];
      hblur[r * width + c] = (uint8_t)(sum >> 4);
    }
  }

  // ── Step 2b: Vertical Gaussian blur ───────────────────────────────────────
  for (int c = 0; c < width; c++) {
    blur_buf[0 * width + c]          = hblur[0 * width + c];
    blur_buf[1 * width + c]          = hblur[1 * width + c];
    blur_buf[(height-2) * width + c] = hblur[(height-2) * width + c];
    blur_buf[(height-1) * width + c] = hblur[(height-1) * width + c];
  }
  for (int r = 2; r < height - 2; r++) {
    for (int c = 0; c < width; c++) {
      int sum = 1 * hblur[(r-2) * width + c]
              + 4 * hblur[(r-1) * width + c]
              + 6 * hblur[ r    * width + c]
              + 4 * hblur[(r+1) * width + c]
              + 1 * hblur[(r+2) * width + c];
      blur_buf[r * width + c] = (uint8_t)(sum >> 4);
    }
  }

  // ── Step 3: Sobel gradient magnitude and direction ────────────────────────
  // mag_buf is uint8_t — Sobel magnitude saturated at 255.
  // Thresholds are 35 and 105, so values above 255 are always strong edges anyway.
  static uint8_t mag_buf[640 * 640];
  static uint8_t dir_buf[640 * 640];  // 0=horiz, 1=diag+, 2=vert, 3=diag-

  for (int r = 1; r < height - 1; r++) {
    for (int c = 1; c < width - 1; c++) {
      #define B(rr,cc) ((int)blur_buf[(rr) * width + (cc)])
      int gx = -B(r-1,c-1) + B(r-1,c+1)
               -2*B(r,c-1) + 2*B(r,c+1)
               -B(r+1,c-1) + B(r+1,c+1);
      int gy = -B(r-1,c-1) - 2*B(r-1,c) - B(r-1,c+1)
               +B(r+1,c-1) + 2*B(r+1,c) + B(r+1,c+1);
      #undef B
      int mag = abs(gx) + abs(gy);
      mag_buf[r * width + c] = (mag > 255) ? 255 : (uint8_t)mag;

      // Quantize gradient direction to 4 angles
      int ax = abs(gx), ay = abs(gy);
      if (ax >= ay * 2)           dir_buf[r * width + c] = 0;  // ~0°
      else if (ay >= ax * 2)      dir_buf[r * width + c] = 2;  // ~90°
      else if (gx * gy > 0)       dir_buf[r * width + c] = 1;  // ~45°
      else                        dir_buf[r * width + c] = 3;  // ~135°
    }
    mag_buf[r * width]             = 0;
    mag_buf[r * width + width - 1] = 0;
  }
  memset(mag_buf,                    0, width);
  memset(mag_buf + (height-1)*width, 0, width);

  // ── Step 4+5: NMS fused with double threshold + hysteresis ────────────────
  // NMS and thresholding iterate the same pixels in the same order, so they
  // are merged into one pass — nms_buf is eliminated entirely.
  int low_thresh  = edge_thresh;
  int high_thresh = edge_thresh * 3;

  int lb = (leftBound  > margin)         ? leftBound  - margin : 0;
  int rb = (rightBound + margin < width) ? rightBound + margin : width - 1;

  static uint8_t edge_buf[640 * 640];
  memset(edge_buf, 0, width * height);

  for (int r = 1; r < height - 1; r++) {
    for (int c = 1; c < width - 1; c++) {
      uint8_t m = mag_buf[r * width + c];
      if (m == 0) continue;

      // NMS: keep only local maxima along gradient direction
      uint8_t d = dir_buf[r * width + c];
      uint8_t m1, m2;
      switch (d) {
        case 0: m1 = mag_buf[r*width + c-1];         m2 = mag_buf[r*width + c+1];         break;
        case 1: m1 = mag_buf[(r-1)*width + c+1];     m2 = mag_buf[(r+1)*width + c-1];     break;
        case 2: m1 = mag_buf[(r-1)*width + c];       m2 = mag_buf[(r+1)*width + c];       break;
        default:m1 = mag_buf[(r-1)*width + c-1];     m2 = mag_buf[(r+1)*width + c+1];     break;
      }
      if (m < m1 || m < m2) continue;  // not a local maximum

      // Apply mask
      int in_mask = (r >= horizonRow) ? (c >= lb && c <= rb) : (c >= leftBound && c <= rightBound);
      if (!in_mask) {
        int uv = r * width * 2 + (c & ~1) * 2;
        uint8_t y = buf[r * width * 2 + c * 2 + 1], u = buf[uv], v = buf[uv + 2];
        in_mask = (y > 30 && y < (uint8_t)green_thresh_value && u < 115 && v < 120);
      }
      if (!in_mask) continue;

      if      (m >= high_thresh) edge_buf[r * width + c] = 255;  // strong
      else if (m >= low_thresh)  edge_buf[r * width + c] = 128;  // weak
    }
  }

  // Hysteresis: promote weak edges connected to strong edges (two-pass sweep)
  for (int r = 1; r < height - 1; r++) {
    for (int c = 1; c < width - 1; c++) {
      if (edge_buf[r*width+c] != 128) continue;
      if (edge_buf[(r-1)*width+c-1] == 255 || edge_buf[(r-1)*width+c] == 255 || edge_buf[(r-1)*width+c+1] == 255 ||
          edge_buf[ r   *width+c-1] == 255 ||                                    edge_buf[ r   *width+c+1] == 255 ||
          edge_buf[(r+1)*width+c-1] == 255 || edge_buf[(r+1)*width+c] == 255 || edge_buf[(r+1)*width+c+1] == 255)
        edge_buf[r*width+c] = 255;
    }
  }
  for (int r = height - 2; r >= 1; r--) {
    for (int c = width - 2; c >= 1; c--) {
      if (edge_buf[r*width+c] != 128) continue;
      if (edge_buf[(r-1)*width+c-1] == 255 || edge_buf[(r-1)*width+c] == 255 || edge_buf[(r-1)*width+c+1] == 255 ||
          edge_buf[ r   *width+c-1] == 255 ||                                    edge_buf[ r   *width+c+1] == 255 ||
          edge_buf[(r+1)*width+c-1] == 255 || edge_buf[(r+1)*width+c] == 255 || edge_buf[(r+1)*width+c+1] == 255)
        edge_buf[r*width+c] = 255;
    }
  }
  for (int i = 0; i < width * height; i++)
    if (edge_buf[i] == 128) edge_buf[i] = 0;

  // ── Step 6: Count edges and write back to YUV422 buffer ──────────────────
  int third = width / 3;
  edge_count_left = edge_count_center = edge_count_right = 0;

  for (int r = 0; r < height; r++) {
    for (int c = 0; c < width; c++) {
      uint8_t val = edge_buf[r * width + c];
      if (edge_draw) {
        buf[r * width * 2 + c * 2 + 1] = val;
        int uv_idx = r * width * 2 + (c & ~1) * 2;
        buf[uv_idx]     = 127;
        buf[uv_idx + 2] = 127;
      }
      if (val > 0) {
        if      (c < third)     edge_count_left++;
        else if (c < 2 * third) edge_count_center++;
        else                    edge_count_right++;
      }
    }
  }
  edge_count_total = edge_count_left + edge_count_center + edge_count_right;
  clock_gettime(CLOCK_MONOTONIC, &t1);
  long ms = (t1.tv_sec - t0.tv_sec) * 1000 + (t1.tv_nsec - t0.tv_nsec) / 1000000;

#endif // EDGE_DETECTION_GRAYSCALE

  return 0;
}
