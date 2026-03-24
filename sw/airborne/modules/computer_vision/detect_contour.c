/*
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/computer_vision/detect_contour.c"
 * @author Roland Meertens and Peng Lu
 *
 * Camera callback wrapper for the tree/contour detector.
 * find_contour() always runs (updates cont_est).
 * Debug overlay is only drawn when VIZ_ACTIVE == VIZ_CONTOUR.
 */

#include <pthread.h>
#include <stdio.h>
#include "modules/computer_vision/cv.h"
#include "modules/computer_vision/detect_contour.h"
#include "modules/computer_vision/opencv_contour.h"
#include "modules/computer_vision/viz_select.h"

pthread_mutex_t contour_mutex;

#ifndef DETECT_CONTOUR_FPS
#define DETECT_CONTOUR_FPS 0
#endif
PRINT_CONFIG_VAR(DETECT_CONTOUR_FPS)

/* ----------------------------------------------------------------
 *  Drawing helpers — write directly into the UYVY buffer.
 *
 *  YUV colour constants (BT.601):
 *    White  : Y=235, U=128, V=128
 *    Green  : Y=145, U= 54, V= 34
 *    Magenta: Y= 63, U=193, V=185
 * ---------------------------------------------------------------- */
static inline void dc_set_yuv(uint8_t *buf, int width, int x, int y,
                               uint8_t Y, uint8_t U, uint8_t V)
{
  uint8_t *p = buf + y * width * 2 + (x & ~1) * 2;
  p[0] = U;
  p[1 + (x & 1) * 2] = Y;
  p[2] = V;
}

static void dc_draw_rect(uint8_t *buf, int width, int height,
                          int rx, int ry, int rw, int rh,
                          uint8_t Y, uint8_t U, uint8_t V)
{
  int x, y;
  for (x = rx; x < rx + rw; x++) {
    if (x < 0 || x >= width) continue;
    if (ry >= 0 && ry < height)           dc_set_yuv(buf, width, x, ry,        Y, U, V);
    if (ry+rh-1 >= 0 && ry+rh-1 < height) dc_set_yuv(buf, width, x, ry+rh-1,  Y, U, V);
  }
  for (y = ry; y < ry + rh; y++) {
    if (y < 0 || y >= height) continue;
    if (rx >= 0 && rx < width)           dc_set_yuv(buf, width, rx,        y, Y, U, V);
    if (rx+rw-1 >= 0 && rx+rw-1 < width) dc_set_yuv(buf, width, rx+rw-1,  y, Y, U, V);
  }
}

static void dc_draw_circle(uint8_t *buf, int width, int height,
                            int cx, int cy, int r,
                            uint8_t Y, uint8_t U, uint8_t V)
{
  int dx, dy;
  for (dy = -r; dy <= r; dy++)
    for (dx = -r; dx <= r; dx++)
      if (dx*dx + dy*dy <= r*r) {
        int px = cx+dx, py = cy+dy;
        if (px >= 0 && px < width && py >= 0 && py < height)
          dc_set_yuv(buf, width, px, py, Y, U, V);
      }
}

/* Minimal 5x7 bitmap font (ASCII 32-90) */
static const uint8_t DC_FONT[][7] = {
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
#define DC_FONT_N ((int)(sizeof(DC_FONT)/sizeof(DC_FONT[0])))

static void dc_draw_text(uint8_t *buf, int width, int height,
                          int ox, int oy, const char *text,
                          uint8_t Y, uint8_t U, uint8_t V)
{
  int cx = ox;
  const char *p;
  for (p = text; *p; p++) {
    int idx = (unsigned char)*p - 32;
    if (idx >= 0 && idx < DC_FONT_N) {
      int row, col;
      for (row = 0; row < 7; row++)
        for (col = 0; col < 5; col++)
          if (DC_FONT[idx][row] & (0x10 >> col)) {
            int px = cx + col, py = oy + row;
            if (px >= 0 && px < width && py >= 0 && py < height)
              dc_set_yuv(buf, width, px, py, Y, U, V);
          }
    }
    cx += 6;
  }
}

/* ----------------------------------------------------------------
 *  Camera callback
 * ---------------------------------------------------------------- */
struct image_t *contour_func(struct image_t *img, uint8_t camera_id);
struct image_t *contour_func(struct image_t *img, uint8_t camera_id)
{
  if (img->type != IMAGE_YUV422) { return img; }

  uint8_t *buf = (uint8_t *)img->buf;
  int w = img->w, h = img->h;

  /* Detection always runs — updates cont_est regardless of VIZ_ACTIVE */
  find_contour_cv((char *)buf, w, h);

  /* ---- Debug overlay — only when this module is selected ---- */
  if (VIZ_ACTIVE == VIZ_CONTOUR) {

    /* Read the full cont_est snapshot under mutex */
    pthread_mutex_lock(&contour_mutex);
    float  dx      = cont_est_cv.contour_d_x;
    int    n_trees = cont_est_cv.n_trees;
    int    bx      = cont_est_cv.best_x;
    int    by      = cont_est_cv.best_y;
    int    bw      = cont_est_cv.best_w;
    int    bh      = cont_est_cv.best_h;
    pthread_mutex_unlock(&contour_mutex);

    if (dx >= 0.0f && n_trees > 0) {
      /* Use actual pixel bounding box from cont_est */
      int rx = bx - bw / 2;
      int ry = by - bh / 2;

      dc_draw_rect(buf, w, h, rx, ry, bw, bh,
                   145, 54, 34);                   /* green box */
      dc_draw_circle(buf, w, h, bx, by, 5,
                     63, 193, 185);                /* magenta dot */

      char txt[64];
      snprintf(txt, sizeof(txt), "TREE %d dx=%.1f", n_trees, dx);
      dc_draw_text(buf, w, h, 8, h - 20, txt, 235, 128, 128);

    } else {
      dc_draw_text(buf, w, h, 8, h - 20,
                   "NO TREE", 235, 128, 128);
    }

    /* Module label top-left */
    dc_draw_text(buf, w, h, 8, 8, "CONTOUR", 145, 54, 34);
  }

  return img;
}

/* ----------------------------------------------------------------
 *  Init
 * ---------------------------------------------------------------- */
void detect_contour_init(void)
{
  pthread_mutex_init(&contour_mutex, NULL);
  cv_add_to_device(&DETECT_CONTOUR_CAMERA, contour_func, DETECT_CONTOUR_FPS, 0);

  /* Default colour thresholds (cyberzoo) */
  cont_thres_cv.lower_y = 16;  cont_thres_cv.lower_u = 135; cont_thres_cv.lower_v = 80;
  cont_thres_cv.upper_y = 100; cont_thres_cv.upper_u = 175; cont_thres_cv.upper_v = 165;
}
