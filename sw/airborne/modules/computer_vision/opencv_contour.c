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
 * All tunable parameters are global variables so Paparazzi's datalink
 * layer can update them at runtime via the GCS settings panel.
 *
 * This file contains ONLY detection logic — it never draws into the
 * image buffer. All overlay drawing is handled by detect_contour.c
 * and gated by VIZ_ACTIVE.
 */

#include "opencv_contour.h"
#include "modules/core/abi.h"

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <pthread.h>

/* contour_mutex is defined in detect_contour.c */
extern pthread_mutex_t contour_mutex;

/* ================================================================
 *  Global state
 * ================================================================ */
struct contour_estimation cont_est_cv  = { .contour_d_x = -1.0f };
struct contour_threshold  cont_thres_cv;

/* ================================================================
 *  Runtime-tunable parameters
 * ================================================================ */

/* YUV colour threshold */
uint8_t OPENCV_CONTOUR_Y_MIN = 44;
uint8_t OPENCV_CONTOUR_Y_MAX = 136;
uint8_t OPENCV_CONTOUR_U_MIN = 86;
uint8_t OPENCV_CONTOUR_U_MAX = 123;
uint8_t OPENCV_CONTOUR_V_MIN = 65;
uint8_t OPENCV_CONTOUR_V_MAX = 134;

/* Morphology */
uint8_t OPENCV_CONTOUR_MORPH_OPEN_RADIUS  = 1;
uint8_t OPENCV_CONTOUR_MORPH_CLOSE_RADIUS = 2;

/* Component area */
uint32_t OPENCV_CONTOUR_MIN_COMPONENT_AREA      = 100;
float    OPENCV_CONTOUR_MAX_COMPONENT_AREA_FRAC = 0.4f;

/* Bounding box size */
uint16_t OPENCV_CONTOUR_MIN_BOX_WIDTH       = 10;
uint16_t OPENCV_CONTOUR_MIN_BOX_HEIGHT      = 10;
float    OPENCV_CONTOUR_MAX_BOX_WIDTH_FRAC  = 0.8f;
float    OPENCV_CONTOUR_MAX_BOX_HEIGHT_FRAC = 0.8f;

/* Shape */
float OPENCV_CONTOUR_MIN_ASPECT_RATIO = 0.3f;
float OPENCV_CONTOUR_MAX_ASPECT_RATIO = 5.0f;
float OPENCV_CONTOUR_MIN_FILL_RATIO   = 0.15f;

/* ================================================================
 *  Internal constants
 * ================================================================ */
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
  *Y = p[1 + (x & 1) * 2];   /* p[1] even x, p[3] odd x */
  *V = p[2];
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
        (Y >= OPENCV_CONTOUR_Y_MIN && Y <= OPENCV_CONTOUR_Y_MAX &&
         U >= OPENCV_CONTOUR_U_MIN && U <= OPENCV_CONTOUR_U_MAX &&
         V >= OPENCV_CONTOUR_V_MIN && V <= OPENCV_CONTOUR_V_MAX) ? 255 : 0;
    }
  }
}

/* ================================================================
 *  Median blur 5x5 on binary mask
 * ================================================================ */
static void uint8_sort25(uint8_t *a)
{
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
      tmp[y * width + x] = neigh[12];
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

static void morph_open(uint8_t *mask, uint8_t *tmp,
                       int width, int height, int r)
{
  erode (mask, tmp,  width, height, r);
  dilate(tmp,  mask, width, height, r);
}

static void morph_close(uint8_t *mask, uint8_t *tmp,
                        int width, int height, int r)
{
  dilate(mask, tmp, width, height, r);
  erode (tmp,  mask, width, height, r);
}

/* ================================================================
 *  Connected components — 4-connected BFS
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
 *  Main entry point
 *  Updates cont_est with the best detected tree.
 *  Never writes to the image buffer — drawing is done in
 *  detect_contour.c after this function returns.
 * ================================================================ */
void find_contour_cv(char *img, int width, int height)
{
  int n_pixels = width * height;

  uint8_t *mask      = (uint8_t *)malloc((size_t)n_pixels);
  uint8_t *tmp       = (uint8_t *)malloc((size_t)n_pixels);
  int     *labels    = (int     *)malloc((size_t)(n_pixels * (int)sizeof(int)));
  int     *bfs_stack = (int     *)malloc((size_t)(n_pixels * (int)sizeof(int)));

  Rect2 all_rects [MAX_COMPONENTS];
  int   all_areas [MAX_COMPONENTS];
  Rect2 tree_boxes[MAX_COMPONENTS];
  float tree_scores[MAX_COMPONENTS];
  int   n_trees = 0;

  const uint8_t *buf = (const uint8_t *)img;

  if (!mask || !tmp || !labels || !bfs_stack) {
    struct contour_estimation local_est = { -1.0f, 0.0f, 0.0f, 0, 0, 0, 0, 0 };
    pthread_mutex_lock(&contour_mutex);
    cont_est_cv = local_est;
    pthread_mutex_unlock(&contour_mutex);
    free(mask); free(tmp); free(labels); free(bfs_stack);
    return;
  }

  /* 1. YUV threshold */
  yuv_threshold(buf, mask, width, height);

  /* 2. Median blur 5x5 */
  median_blur_5(mask, tmp, width, height);

  /* 3. Morphology: OPEN then CLOSE */
  morph_open (mask, tmp, width, height, OPENCV_CONTOUR_MORPH_OPEN_RADIUS);
  morph_close(mask, tmp, width, height, OPENCV_CONTOUR_MORPH_CLOSE_RADIUS);

  /* 4. Erase bottom third */
  {
    int cut_y = (int)(2.0f * (float)height / 3.0f);
    memset(mask + cut_y * width, 0, (size_t)((height - cut_y) * width));
  }

  /* 5. Connected components */
  int num_labels = connected_components(mask, labels, bfs_stack, width, height);
  if (num_labels > MAX_COMPONENTS) num_labels = MAX_COMPONENTS;

  /* 6. Bounding boxes */
  compute_bounding_rects(labels, num_labels, width, height, all_rects, all_areas);

  /* 7. Filter components */
  {
    int i;
    for (i = 0; i < num_labels && n_trees < MAX_COMPONENTS; i++) {
      int    a  = all_areas[i];
      Rect2 *r  = &all_rects[i];
      float  aspect, fill;

      if (a < (int)OPENCV_CONTOUR_MIN_COMPONENT_AREA) continue;
      if (a > (int)(OPENCV_CONTOUR_MAX_COMPONENT_AREA_FRAC * (float)(width * height))) continue;
      if (r->w < (int)OPENCV_CONTOUR_MIN_BOX_WIDTH ||
          r->h < (int)OPENCV_CONTOUR_MIN_BOX_HEIGHT) continue;
      if (r->w > (int)(OPENCV_CONTOUR_MAX_BOX_WIDTH_FRAC  * (float)width)  ||
          r->h > (int)(OPENCV_CONTOUR_MAX_BOX_HEIGHT_FRAC * (float)height)) continue;

      aspect = (float)r->h / (float)r->w;
      if (aspect < OPENCV_CONTOUR_MIN_ASPECT_RATIO ||
          aspect > OPENCV_CONTOUR_MAX_ASPECT_RATIO) continue;

      fill = (float)a / (float)(r->w * r->h);
      if (fill < OPENCV_CONTOUR_MIN_FILL_RATIO) continue;

      tree_boxes[n_trees]  = *r;
      tree_scores[n_trees] = (float)a;
      n_trees++;
    }
  }

  /* 8. Best tree -> cont_est */
  if (n_trees > 0) {
    int   best_idx   = 0;
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

    struct contour_estimation local_est;
    local_est.contour_d_x = dist;
    local_est.contour_d_y = -(best_cx - (float)width  * 0.5f)
                            / (float)tree_boxes[best_idx].w;
    local_est.contour_d_z = -(best_cy - (float)height * 0.5f)
                            / (float)tree_boxes[best_idx].h;
    local_est.n_trees = n_trees;
    local_est.best_x  = (int)best_cx;
    local_est.best_y  = (int)best_cy;
    local_est.best_w  = tree_boxes[best_idx].w;
    local_est.best_h  = tree_boxes[best_idx].h;

    pthread_mutex_lock(&contour_mutex);
    cont_est_cv = local_est;
    pthread_mutex_unlock(&contour_mutex);

    /* ABI message — uncomment when message ID is defined
    AbiSendMsgTREE_POSITION(ABI_SENDER_TREE_DETECTOR,
                            cont_est.contour_d_x,
                            cont_est.contour_d_y,
                            cont_est.contour_d_z,
                            1); */

  } else {
    struct contour_estimation local_est = { -1.0f, 0.0f, 0.0f, 0, 0, 0, 0, 0 };
    pthread_mutex_lock(&contour_mutex);
    cont_est_cv = local_est;
    pthread_mutex_unlock(&contour_mutex);

    /* ABI message — uncomment when message ID is defined
    AbiSendMsgTREE_POSITION(ABI_SENDER_TREE_DETECTOR,
                            cont_est.contour_d_x,
                            cont_est.contour_d_y,
                            cont_est.contour_d_z,
                            0); */
  }

  free(mask);
  free(tmp);
  free(labels);
  free(bfs_stack);
}
