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
/**
 * @file "modules/computer_vision/opencv_contour.c"
 * @author Roland Meertens and Peng Lu
 *
 * Pure C rewrite — no OpenCV, no C++, no RGB intermediate buffer.
 * Color filtering is performed directly on the UYVY (YCbCr) data.
 *
 * UYVY byte layout (4 bytes per pixel-pair):
 *   [ U | Y0 | V | Y1 ]
 *   pixel 0 uses Y0, pixel 1 uses Y1, both share U and V.
 *
 * Green/foliage threshold in YUV space — six axis-aligned bounds
 * derived by projecting inRange(HSV, Scalar(30,60,40), Scalar(100,255,255))
 * through BT.601 full-range YCbCr.  Use tune_detector.py to recalibrate.
 */

#include "opencv_contour.h"
#include "modules/core/abi.h"

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

struct contour_estimation cont_est;
struct contour_threshold  cont_thres;

/* ================================================================
 *  Threshold #defines
 *  Paste updated values from tune_detector.py / tuner widget here.
 * ================================================================ */
#define Y_MIN      40
#define Y_MAX     235
#define U_MIN      77
#define U_MAX     146
#define V_MIN      22
#define V_MAX     133

/* Maximum number of connected components the detector will track.
   Increase if your scene can contain more distinct green blobs.    */
#define MAX_COMPONENTS 128

/* ================================================================
 *  Internal types
 * ================================================================ */
typedef struct { int x, y, w, h; } Rect2;

/* ================================================================
 *  UYVY accessors
 * ================================================================ */
static inline void get_yuv(const uint8_t *buf, int width, int x, int y,
                            uint8_t *Y, uint8_t *U, uint8_t *V)
{
  const uint8_t *p = buf + y * width * 2 + (x & ~1) * 2;
  *U = p[0];
  Y = p[1 + (x & 1) * 2];   // p[1] even x, p[3] odd x */
  *V = p[2];
}

static inline void set_yuv(uint8_t *buf, int width, int x, int y,
                            uint8_t Y, uint8_t U, uint8_t V)
{
  uint8_t *p = buf + y * width * 2 + (x & ~1) * 2;
  p[0] = U;
  p[1 + (x & 1) * 2] = Y;
  p[2] = V;
}

/* ================================================================
 *  YUV threshold  →  binary mask
 * ================================================================ */
static void yuv_threshold(const uint8_t *buf, uint8_t *mask,
                           int width, int height)
{
  int x, y;
  for (y = 0; y < height; y++) {
    for (x = 0; x < width; x++) {
      uint8_t Y, U, V;
      get_yuv(buf, width, x, y, &Y, &U, &V);
      mask[y * width + x] =
        (Y >= Y_MIN && Y <= Y_MAX &&
         U >= U_MIN && U <= U_MAX &&
         V >= V_MIN && V <= V_MAX) ? 255 : 0;
    }
  }
}

/* ================================================================
 *  Median blur 5×5 on binary mask
 *  Uses a simple partial-sort (insertion sort on 25 neighbours).
 * ================================================================ */
static void uint8_sort25(uint8_t *a)
{
  /* insertion sort — fast enough for 25 elements */
  int i, j;
  for (i = 1; i < 25; i++) {
    uint8_t key = a[i];
    for (j = i - 1; j >= 0 && a[j] > key; j--)
      a[j + 1] = a[j];
    a[j + 1] = key;
  }
}

static void median_blur_5(uint8_t *mask, uint8_t *tmp, int width, int height)
{
  int x, y, dx, dy, n;
  for (y = 0; y < height; y++) {
    for (x = 0; x < width; x++) {
      uint8_t neigh[25];
      n = 0;
      for (dy = -2; dy <= 2; dy++) {
        for (dx = -2; dx <= 2; dx++) {
          int ny = y + dy, nx = x + dx;
          neigh[n++] = (ny >= 0 && ny < height && nx >= 0 && nx < width)
                       ? mask[ny * width + nx] : 0;
        }
      }
      uint8_sort25(neigh);
      tmp[y * width + x] = neigh[12];  /* median of 25 */
    }
  }
  memcpy(mask, tmp, (size_t)(width * height));
}

/* ================================================================
 *  Morphology — disk structuring element of radius r
 * ================================================================ */
static void erode(const uint8_t *src, uint8_t *dst,
                  int width, int height, int r)
{
  int x, y, dx, dy;
  for (y = 0; y < height; y++) {
    for (x = 0; x < width; x++) {
      int on = 1;
      for (dy = -r; dy <= r && on; dy++) {
        for (dx = -r; dx <= r && on; dx++) {
          if (dx * dx + dy * dy > r * r) continue;
          int ny = y + dy, nx = x + dx;
          if (ny < 0 || ny >= height || nx < 0 || nx >= width ||
              src[ny * width + nx] == 0)
            on = 0;
        }
      }
      dst[y * width + x] = on ? 255 : 0;
    }
  }
}

static void dilate(const uint8_t *src, uint8_t *dst,
                   int width, int height, int r)
{
  int x, y, dx, dy;
  for (y = 0; y < height; y++) {
    for (x = 0; x < width; x++) {
      int on = 0;
      for (dy = -r; dy <= r && !on; dy++) {
        for (dx = -r; dx <= r && !on; dx++) {
          if (dx * dx + dy * dy > r * r) continue;
          int ny = y + dy, nx = x + dx;
          if (ny >= 0 && ny < height && nx >= 0 && nx < width &&
              src[ny * width + nx] != 0)
            on = 1;
        }
      }
      dst[y * width + x] = on ? 255 : 0;
    }
  }
}

/* OPEN = erode then dilate */
static void morph_open(uint8_t *mask, uint8_t *tmp,
                       int width, int height, int r)
{
  erode (mask, tmp,  width, height, r);
  dilate(tmp,  mask, width, height, r);
}

/* CLOSE = dilate then erode */
static void morph_close(uint8_t *mask, uint8_t *tmp,
                        int width, int height, int r)
{
  dilate(mask, tmp, width, height, r);
  erode (tmp,  mask, width, height, r);
}

/* ================================================================
 *  Connected components — 4-connected BFS
 *
 *  labels[]  : output, same size as mask (0 = background)
 *  bfs_stack : caller-supplied scratch buffer, size = width*height
 *  Returns number of components found (background not counted).
 * ================================================================ */
static int connected_components(const uint8_t *mask, int *labels,
                                 int *bfs_stack,
                                 int width, int height)
{
  static const int dx4[4] = { 1, -1,  0, 0 };
  static const int dy4[4] = { 0,  0,  1,-1 };

  int next_label = 1;
  int total = width * height;
  int start;

  memset(labels, 0, (size_t)(total * (int)sizeof(int)));

  for (start = 0; start < total; start++) {
    if (mask[start] == 0 || labels[start] != 0) continue;

    /* BFS */
    int head = 0, tail = 0;
    labels[start] = next_label;
    bfs_stack[tail++] = start;

    while (head < tail) {
      int idx = bfs_stack[head++];
      int cy  = idx / width;
      int cx  = idx % width;
      int d;

      for (d = 0; d < 4; d++) {
        int nx = cx + dx4[d];
        int ny = cy + dy4[d];
        if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;
        int ni = ny * width + nx;
        if (mask[ni] != 0 && labels[ni] == 0) {
          labels[ni] = next_label;
          bfs_stack[tail++] = ni;
        }
      }
    }
    next_label++;
  }
  return next_label - 1;
}

/* ================================================================
 *  Bounding boxes + pixel areas from label map
 * ================================================================ */
static void compute_bounding_rects(const int *labels, int num_labels,
                                    int width, int height,
                                    Rect2 *rects, int *areas)
{
  int i, x, y;

  for (i = 0; i < num_labels; i++) {
    rects[i].x = width;
    rects[i].y = height;
    rects[i].w = 0;
    rects[i].h = 0;
    areas[i]   = 0;
  }

  for (y = 0; y < height; y++) {
    for (x = 0; x < width; x++) {
      int lbl = labels[y * width + x];
      if (lbl == 0) continue;
      int li = lbl - 1;
      areas[li]++;
      if (x     < rects[li].x)                rects[li].x = x;
      if (y     < rects[li].y)                rects[li].y = y;
      if (x + 1 > rects[li].x + rects[li].w) rects[li].w = x + 1 - rects[li].x;
      if (y + 1 > rects[li].y + rects[li].h) rects[li].h = y + 1 - rects[li].y;
    }
  }
}

/* ================================================================
 *  Drawing — operate directly on UYVY buffer
 *
 *  Pre-computed YUV colour constants (BT.601):
 *    White  : Y=235, U=128, V=128
 *    Green  : Y=145, U= 54, V= 34
 *    Magenta: Y= 63, U=193, V=185
 * ================================================================ */
static void draw_rect_yuv(uint8_t *buf, int width, int height,
                           int rx, int ry, int rw, int rh,
                           uint8_t Y, uint8_t U, uint8_t V,
                           int thickness)
{
  int t, x, y;
  for (t = 0; t < thickness; t++) {
    int x0 = rx - t,        y0 = ry - t;
    int x1 = rx + rw + t - 1, y1 = ry + rh + t - 1;
    for (x = x0; x <= x1; x++) {
      if (x < 0 || x >= width) continue;
      if (y0 >= 0 && y0 < height) set_yuv(buf, width, x, y0, Y, U, V);
      if (y1 >= 0 && y1 < height) set_yuv(buf, width, x, y1, Y, U, V);
    }
    for (y = y0 + 1; y < y1; y++) {
      if (y < 0 || y >= height) continue;
      if (x0 >= 0 && x0 < width) set_yuv(buf, width, x0, y, Y, U, V);
      if (x1 >= 0 && x1 < width) set_yuv(buf, width, x1, y, Y, U, V);
    }
  }
}

static void draw_circle_yuv(uint8_t *buf, int width, int height,
                             int cx, int cy, int radius,
                             uint8_t Y, uint8_t U, uint8_t V)
{
  int dx, dy;
  for (dy = -radius; dy <= radius; dy++) {
    for (dx = -radius; dx <= radius; dx++) {
      if (dx * dx + dy * dy <= radius * radius) {
        int px = cx + dx, py = cy + dy;
        if (px >= 0 && px < width && py >= 0 && py < height)
          set_yuv(buf, width, px, py, Y, U, V);
      }
    }
  }
}

/* ================================================================
 *  Minimal 5×7 bitmap font (ASCII 32–90)
 * ================================================================ */
static const uint8_t FONT5x7[][7] = {
  {0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* ' ' */
  {0x04,0x04,0x04,0x04,0x00,0x04,0x00}, /* '!' */
  {0x0A,0x0A,0x00,0x00,0x00,0x00,0x00}, /* '"' */
  {0x0A,0x1F,0x0A,0x1F,0x0A,0x00,0x00}, /* '#' */
  {0x04,0x0F,0x14,0x0E,0x05,0x1E,0x04}, /* '$' */
  {0x18,0x19,0x02,0x04,0x13,0x03,0x00}, /* '%' */
  {0x0C,0x12,0x14,0x08,0x15,0x12,0x0D}, /* '&' */
  {0x04,0x04,0x00,0x00,0x00,0x00,0x00}, /* '\'' */
  {0x02,0x04,0x08,0x08,0x08,0x04,0x02}, /* '(' */
  {0x08,0x04,0x02,0x02,0x02,0x04,0x08}, /* ')' */
  {0x00,0x04,0x15,0x0E,0x15,0x04,0x00}, /* '*' */
  {0x00,0x04,0x04,0x1F,0x04,0x04,0x00}, /* '+' */
  {0x00,0x00,0x00,0x00,0x06,0x04,0x08}, /* ',' */
  {0x00,0x00,0x00,0x1F,0x00,0x00,0x00}, /* '-' */
  {0x00,0x00,0x00,0x00,0x00,0x06,0x00}, /* '.' */
  {0x01,0x02,0x02,0x04,0x08,0x10,0x00}, /* '/' */
  {0x0E,0x11,0x13,0x15,0x19,0x11,0x0E}, /* '0' */
  {0x04,0x0C,0x04,0x04,0x04,0x04,0x0E}, /* '1' */
  {0x0E,0x11,0x01,0x02,0x04,0x08,0x1F}, /* '2' */
  {0x1F,0x02,0x04,0x02,0x01,0x11,0x0E}, /* '3' */
  {0x02,0x06,0x0A,0x12,0x1F,0x02,0x02}, /* '4' */
  {0x1F,0x10,0x1E,0x01,0x01,0x11,0x0E}, /* '5' */
  {0x06,0x08,0x10,0x1E,0x11,0x11,0x0E}, /* '6' */
  {0x1F,0x01,0x02,0x04,0x08,0x08,0x08}, /* '7' */
  {0x0E,0x11,0x11,0x0E,0x11,0x11,0x0E}, /* '8' */
  {0x0E,0x11,0x11,0x0F,0x01,0x02,0x0C}, /* '9' */
  {0x00,0x06,0x00,0x00,0x06,0x00,0x00}, /* ':' */
  {0x00,0x06,0x00,0x00,0x06,0x04,0x08}, /* ';' */
  {0x02,0x04,0x08,0x10,0x08,0x04,0x02}, /* '<' */
  {0x00,0x00,0x1F,0x00,0x1F,0x00,0x00}, /* '=' */
  {0x10,0x08,0x04,0x02,0x04,0x08,0x10}, /* '>' */
  {0x0E,0x11,0x01,0x02,0x04,0x00,0x04}, /* '?' */
  {0x0E,0x11,0x17,0x15,0x17,0x10,0x0F}, /* '@' */
  {0x0E,0x11,0x11,0x1F,0x11,0x11,0x11}, /* 'A' */
  {0x1E,0x11,0x11,0x1E,0x11,0x11,0x1E}, /* 'B' */
  {0x0E,0x11,0x10,0x10,0x10,0x11,0x0E}, /* 'C' */
  {0x1C,0x12,0x11,0x11,0x11,0x12,0x1C}, /* 'D' */
  {0x1F,0x10,0x10,0x1E,0x10,0x10,0x1F}, /* 'E' */
  {0x1F,0x10,0x10,0x1E,0x10,0x10,0x10}, /* 'F' */
  {0x0E,0x11,0x10,0x17,0x11,0x11,0x0F}, /* 'G' */
  {0x11,0x11,0x11,0x1F,0x11,0x11,0x11}, /* 'H' */
  {0x0E,0x04,0x04,0x04,0x04,0x04,0x0E}, /* 'I' */
  {0x07,0x02,0x02,0x02,0x02,0x12,0x0C}, /* 'J' */
  {0x11,0x12,0x14,0x18,0x14,0x12,0x11}, /* 'K' */
  {0x10,0x10,0x10,0x10,0x10,0x10,0x1F}, /* 'L' */
  {0x11,0x1B,0x15,0x15,0x11,0x11,0x11}, /* 'M' */
  {0x11,0x19,0x15,0x13,0x11,0x11,0x11}, /* 'N' */
  {0x0E,0x11,0x11,0x11,0x11,0x11,0x0E}, /* 'O' */
  {0x1E,0x11,0x11,0x1E,0x10,0x10,0x10}, /* 'P' */
  {0x0E,0x11,0x11,0x11,0x15,0x12,0x0D}, /* 'Q' */
  {0x1E,0x11,0x11,0x1E,0x14,0x12,0x11}, /* 'R' */
  {0x0F,0x10,0x10,0x0E,0x01,0x01,0x1E}, /* 'S' */
  {0x1F,0x04,0x04,0x04,0x04,0x04,0x04}, /* 'T' */
  {0x11,0x11,0x11,0x11,0x11,0x11,0x0E}, /* 'U' */
  {0x11,0x11,0x11,0x11,0x11,0x0A,0x04}, /* 'V' */
  {0x11,0x11,0x11,0x15,0x15,0x1B,0x11}, /* 'W' */
  {0x11,0x11,0x0A,0x04,0x0A,0x11,0x11}, /* 'X' */
  {0x11,0x11,0x0A,0x04,0x04,0x04,0x04}, /* 'Y' */
  {0x1F,0x01,0x02,0x04,0x08,0x10,0x1F}, /* 'Z' */
};
#define FONT_NCHARS ((int)(sizeof(FONT5x7) / sizeof(FONT5x7[0])))

static void draw_text_yuv(uint8_t *buf, int width, int height,
                           int ox, int oy, const char *text,
                           uint8_t Y, uint8_t U, uint8_t V,
                           int scale)
{
  int cx = ox;
  const char *p;
  for (p = text; *p; p++) {
    int idx = (unsigned char)*p - 32;
    if (idx >= 0 && idx < FONT_NCHARS) {
      const uint8_t *glyph = FONT5x7[idx];
      int row, col, sy, sx;
      for (row = 0; row < 7; row++) {
        for (col = 0; col < 5; col++) {
          if (glyph[row] & (0x10 >> col)) {
            for (sy = 0; sy < scale; sy++) {
              for (sx = 0; sx < scale; sx++) {
                int px = cx + col * scale + sx;
                int py = oy + row * scale + sy;
                if (px >= 0 && px < width && py >= 0 && py < height)
                  set_yuv(buf, width, px, py, Y, U, V);
              }
            }
          }
        }
      }
    }
    cx += (5 + 1) * scale;
  }
}

/* ================================================================
 *  Main entry point
 * ================================================================ */
void find_contour(char *img, int width, int height)
{
  int n_pixels = width * height;

  /* ------ allocate working buffers --------------------------------
     All on the heap so stack usage stays small on embedded targets. */
  uint8_t *mask      = (uint8_t *)malloc((size_t)n_pixels);
  uint8_t *tmp       = (uint8_t *)malloc((size_t)n_pixels);
  int     *labels    = (int     *)malloc((size_t)(n_pixels * (int)sizeof(int)));
  int     *bfs_stack = (int     *)malloc((size_t)(n_pixels * (int)sizeof(int)));

  /* Fixed-size arrays for accepted trees (MAX_COMPONENTS entries) */
  Rect2 all_rects [MAX_COMPONENTS];
  int   all_areas [MAX_COMPONENTS];
  Rect2 tree_boxes[MAX_COMPONENTS];
  float tree_scores[MAX_COMPONENTS];
  int   n_trees = 0;

  uint8_t *buf = (uint8_t *)img;

  if (!mask || !tmp || !labels || !bfs_stack) {
    /* out of memory — report no detection and bail */
    cont_est.contour_d_x = -1.0f;
    cont_est.contour_d_y =  0.0f;
    cont_est.contour_d_z =  0.0f;
    free(mask); free(tmp); free(labels); free(bfs_stack);
    return;
  }

  /* ------ 1. YUV threshold ---------------------------------------- */
  yuv_threshold(buf, mask, width, height);

  /* ------ 3. Median blur 5×5 -------------------------------------- */
  median_blur_5(mask, tmp, width, height);

  /* ------ 4. Morphology: OPEN (r=1) then CLOSE (r=2) -------------- */
  morph_open (mask, tmp, width, height, 1);
  morph_close(mask, tmp, width, height, 2);

  /* ------ 5. Erase bottom third ------------------------------------ */
  {
    int cut_y = (int)(2.0f * (float)height / 3.0f);
    memset(mask + cut_y * width, 0, (size_t)((height - cut_y) * width));
  }

  /* ------ 6. Connected components ---------------------------------- */
  int num_labels = connected_components(mask, labels, bfs_stack,
                                        width, height);

  /* Guard against more components than our fixed arrays can hold */
  if (num_labels > MAX_COMPONENTS) num_labels = MAX_COMPONENTS;

  /* ------ 7. Bounding boxes ---------------------------------------- */
  compute_bounding_rects(labels, num_labels, width, height,
                         all_rects, all_areas);

  /* ------ 8. Filter components ------------------------------------- */
  {
    int i;
    for (i = 0; i < num_labels && n_trees < MAX_COMPONENTS; i++) {
      int    a  = all_areas[i];
      Rect2 *r  = &all_rects[i];
      float  aspect, fill;

      if (a < 100)                              continue;
      if (a > (int)(0.4f * (float)(width * height))) continue;
      if (r->w < 10 || r->h < 10)              continue;
      if (r->w > (int)(0.8f * (float)width) ||
          r->h > (int)(0.8f * (float)height))  continue;

      aspect = (float)r->h / (float)r->w;
      if (aspect < 0.3f || aspect > 5.0f)      continue;

      fill = (float)a / (float)(r->w * r->h);
      if (fill < 0.15f)                         continue;

      tree_boxes[n_trees]  = *r;
      tree_scores[n_trees] = (float)a;
      n_trees++;
    }
  }

  /* ------ 9. Best tree → cont_est --------------------------------- */
  if (n_trees > 0) {
    int   best_idx = 0;
    float best_score = tree_scores[0];
    float best_cx, best_cy, area, dist;
    int   i;

    for (i = 1; i < n_trees; i++) {
      if (tree_scores[i] > best_score) {
        best_score = tree_scores[i];
        best_idx   = i;
      }
    }

    best_cx = (float)tree_boxes[best_idx].x + (float)tree_boxes[best_idx].w * 0.5f;
    best_cy = (float)tree_boxes[best_idx].y + (float)tree_boxes[best_idx].h * 0.5f;
    area    = (float)(tree_boxes[best_idx].w * tree_boxes[best_idx].h);

    if      (area > 28000.0f) dist = 0.1f;
    else if (area > 16000.0f) dist = 0.5f;
    else if (area > 11000.0f) dist = 1.0f;
    else if (area >  3000.0f) dist = 1.5f;
    else                      dist = 2.0f;

    cont_est.contour_d_x = dist;
    cont_est.contour_d_y = -(best_cx - (float)width  * 0.5f)
                           / (float)tree_boxes[best_idx].w;
    cont_est.contour_d_z = -(best_cy - (float)height * 0.5f)
                           / (float)tree_boxes[best_idx].h;

    /* SEND ABI MESSAGE HERE */
    /*AbiSendMsgTREE_POSITION(ABI_SENDER_TREE_DETECTOR,
                            cont_est.contour_d_x,
                            cont_est.contour_d_y,
                            cont_est.contour_d_z,
                            1);*/

  } else {
    cont_est.contour_d_x = -1.0f;
    cont_est.contour_d_y =  0.0f;
    cont_est.contour_d_z =  0.0f;

    /* SEND ABI MESSAGE HERE TOO */
    /*AbiSendMsgTREE_POSITION(ABI_SENDER_TREE_DETECTOR,
                            cont_est.contour_d_x,
                            cont_est.contour_d_y,
                            cont_est.contour_d_z,
                            0);*/

  }

  /* ------ 11. Free working buffers --------------------------------- */
  free(mask);
  free(tmp);
  free(labels);
  free(bfs_stack);
}