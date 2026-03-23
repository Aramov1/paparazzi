/*
 * c_contour_edited.c
 *
 * Pure-C gate detector — no OpenCV dependency.
 * Optimized for real-time onboard use on Bebop ARM Cortex-A9.
 *
 * Pipeline:
 *   1. Single pass: classify pixels + draw overlay simultaneously
 *   2. Connected-components blob finder with path-compressed Union-Find
 *   3. Precomputed blob metrics + early rejection
 *   4. Integer-only pairing geometry (no float division in hot path)
 *   5. Temporal persistence (1-2 frame holdover on dropout)
 *
 * FIXES applied vs original port:
 *   [F1] blob_max: 14000 → 12000  (matches OpenCV)
 *   [F2] Score formula: restored area-ratio term dropped during port
 *   [F3] Spatial consistency: gated on gate_tracking flag (matches OpenCV)
 *   [F4] MIN_BLOB_AREA pre-filter: now uses pixel_count (matches OpenCV
 *        contourArea semantics) instead of bbox area
 *
 * To enable debug prints: pass -DDEBUG_CONTOUR in CFLAGS
 * To enable overlay:      set show_threshold_overlay = 1 at runtime
 */

#include "c_contour_edited.h"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* -------------------------------------------------------------------------
 * Color classifier
 * ------------------------------------------------------------------------- */
#ifdef NPS
  #include "color_classifier_sim.h"
#else
  #include "color_classifier_real.h"
#endif

/* -------------------------------------------------------------------------
 * Debug print — compiled out completely in production builds
 * ------------------------------------------------------------------------- */
#ifdef DEBUG_CONTOUR
  #include <stdio.h>
  #define DPRINT(...) printf(__VA_ARGS__)
#else
  #define DPRINT(...) ((void)0)
#endif

/* -------------------------------------------------------------------------
 * Image size limits.
 * All static buffers are sized exactly from these defines.
 * To support a larger resolution, increase these values — all buffers
 * resize automatically. The runtime guard in find_contour() enforces
 * these limits strictly.
 * ------------------------------------------------------------------------- */
#define MAX_IMG_WIDTH  320
#define MAX_IMG_HEIGHT 520
#define MAX_IMG_PIXELS (MAX_IMG_WIDTH * MAX_IMG_HEIGHT)

_Static_assert(MAX_IMG_WIDTH  > 0, "MAX_IMG_WIDTH must be positive");
_Static_assert(MAX_IMG_HEIGHT > 0, "MAX_IMG_HEIGHT must be positive");

/* -------------------------------------------------------------------------
 * Blob limits
 * ------------------------------------------------------------------------- */
#define MAX_BLOBS     64
#define MIN_BLOB_AREA 300   /* minimum pixel_count (not bbox area) — [F4] */

/* -------------------------------------------------------------------------
 * Temporal persistence
 * ------------------------------------------------------------------------- */
#define MAX_HOLDOVER 2

/* -------------------------------------------------------------------------
 * Shared state
 * ------------------------------------------------------------------------- */
struct contour_estimation cont_est;
struct contour_threshold  cont_thres;
int show_threshold_overlay = 1;

extern int gate_tracking;   /* set by navigator: 1 = actively tracking gate */

static float last_gate_cx = -1.f;
static float last_gate_cy = -1.f;
static int   holdover     = 0;
static float hold_d_y     = 0.f;
static float hold_d_z     = 0.f;
static float hold_dist    = 1.5f;
static float hold_area    = 0.f;

void contour_reset_tracking(void)
{
  last_gate_cx = -1.f;
  last_gate_cy = -1.f;
  holdover     = 0;
}

/* -------------------------------------------------------------------------
 * Overlay colors (Y, U, V)
 * ------------------------------------------------------------------------- */
#define Y_GREEN 150
#define U_GREEN  44
#define V_GREEN  21
#define Y_CYAN   178
#define U_CYAN   170
#define V_CYAN     0
#define Y_YELLOW 210
#define U_YELLOW  16
#define V_YELLOW 146

/* -------------------------------------------------------------------------
 * Write one pixel into the UYVY buffer.
 * U/V shared between col pairs — only write on even column.
 * ------------------------------------------------------------------------- */
static inline void set_uyvy_pixel(uint8_t *buf,
                                   int col, int row, int width,
                                   uint8_t y, uint8_t u, uint8_t v)
{
  int i = row * width + col;
  buf[2 * i + 1] = y;
  if ((col & 1) == 0) {
    buf[2 * i]     = u;
    buf[2 * i + 2] = v;
  }
}

/* -------------------------------------------------------------------------
 * Draw rectangle border into UYVY buffer.
 * ------------------------------------------------------------------------- */
static void draw_rect(uint8_t *buf, int width, int height,
                      int x0, int y0, int x1, int y1,
                      uint8_t y, uint8_t u, uint8_t v, int thickness)
{
  if (x0 < 0) x0 = 0;
  if (y0 < 0) y0 = 0;
  if (x1 >= width)  x1 = width  - 1;
  if (y1 >= height) y1 = height - 1;
  for (int t = 0; t < thickness; t++) {
    for (int x = x0; x <= x1; x++) {
      if (y0 + t < height) set_uyvy_pixel(buf, x, y0 + t, width, y, u, v);
      if (y1 - t >= 0)     set_uyvy_pixel(buf, x, y1 - t, width, y, u, v);
    }
    for (int yy = y0; yy <= y1; yy++) {
      if (x0 + t < width) set_uyvy_pixel(buf, x0 + t, yy, width, y, u, v);
      if (x1 - t >= 0)    set_uyvy_pixel(buf, x1 - t, yy, width, y, u, v);
    }
  }
}

/* -------------------------------------------------------------------------
 * Draw cross into UYVY buffer.
 * ------------------------------------------------------------------------- */
static void draw_cross(uint8_t *buf, int width, int height,
                       int cx, int cy, int size,
                       uint8_t y, uint8_t u, uint8_t v)
{
  for (int d = -size; d <= size; d++) {
    int x  = cx + d;
    int yy = cy + d;
    if (x  >= 0 && x  < width)  set_uyvy_pixel(buf, x,  cy, width, y, u, v);
    if (yy >= 0 && yy < height) set_uyvy_pixel(buf, cx, yy, width, y, u, v);
  }
}

/* -------------------------------------------------------------------------
 * Blob struct with precomputed metrics
 * ------------------------------------------------------------------------- */
typedef struct {
  int min_x, max_x;
  int min_y, max_y;
  int pixel_count;
  int active;
  int w, h, area, cx, cy;  /* precomputed after blob finding */
} Blob;

/* -------------------------------------------------------------------------
 * Union-Find with path compression.
 * ------------------------------------------------------------------------- */
static int16_t uf_parent[MAX_BLOBS];

static inline int uf_find(int x)
{
  int root = x;
  while (uf_parent[root] != root) root = uf_parent[root];
  while (uf_parent[x] != root) {
    int next = uf_parent[x];
    uf_parent[x] = (int16_t)root;
    x = next;
  }
  return root;
}

static inline void uf_union(int a, int b)
{
  int ra = uf_find(a);
  int rb = uf_find(b);
  if (ra != rb) uf_parent[rb] = (int16_t)ra;
}

/* -------------------------------------------------------------------------
 * Connected-components blob finder.
 * Single pass, two row buffers, path-compressed UF.
 * Precomputes metrics and rejects small blobs before returning.
 *
 * [F4] Pre-filter now uses pixel_count (actual filled pixels) to match
 *      OpenCV's contourArea(c) > 300 semantic. Previously used bbox area
 *      w*h which allowed sparse noise blobs to pass.
 * ------------------------------------------------------------------------- */
static int find_blobs(const uint8_t *mask, int width, int height, Blob *blobs)
{
  static int16_t cur_row[MAX_IMG_WIDTH];
  static int16_t prev_row[MAX_IMG_WIDTH];

  int n_blobs = 0;

  for (int b = 0; b < MAX_BLOBS; b++) {
    uf_parent[b]    = (int16_t)b;
    blobs[b].active = 0;
  }

  for (int c = 0; c < width; c++) { prev_row[c] = -1; cur_row[c] = -1; }

  for (int row = 0; row < height; row++) {
    for (int col = 0; col < width; col++) {
      cur_row[col] = -1;
      if (!mask[row * width + col]) continue;

      int label = -1;

      if (col > 0 && cur_row[col - 1] >= 0)
        label = uf_find(cur_row[col - 1]);

      if (prev_row[col] >= 0) {
        int above = uf_find(prev_row[col]);
        if (label < 0) {
          label = above;
        } else if (label != above) {
          uf_union(label, above);
          label = uf_find(label);
        }
      }

      if (label < 0) {
        if (n_blobs >= MAX_BLOBS) { cur_row[col] = 0; continue; }
        label = n_blobs++;
        blobs[label].min_x       = col;
        blobs[label].max_x       = col;
        blobs[label].min_y       = row;
        blobs[label].max_y       = row;
        blobs[label].pixel_count = 0;
        blobs[label].active      = 1;
        uf_parent[label]         = (int16_t)label;
      }

      int root = uf_find(label);
      if (col < blobs[root].min_x) blobs[root].min_x = col;
      if (col > blobs[root].max_x) blobs[root].max_x = col;
      if (row < blobs[root].min_y) blobs[root].min_y = row;
      if (row > blobs[root].max_y) blobs[root].max_y = row;
      blobs[root].pixel_count++;
      cur_row[col] = (int16_t)root;
    }
    for (int c = 0; c < width; c++) prev_row[c] = cur_row[c];
  }

  /* merge bounding boxes for union-ed blobs */
  for (int b = 0; b < n_blobs; b++) {
    int root = uf_find(b);
    if (root != b && blobs[b].active) {
      if (blobs[b].min_x < blobs[root].min_x) blobs[root].min_x = blobs[b].min_x;
      if (blobs[b].max_x > blobs[root].max_x) blobs[root].max_x = blobs[b].max_x;
      if (blobs[b].min_y < blobs[root].min_y) blobs[root].min_y = blobs[b].min_y;
      if (blobs[b].max_y > blobs[root].max_y) blobs[root].max_y = blobs[b].max_y;
      blobs[root].pixel_count += blobs[b].pixel_count;
      blobs[b].active = 0;
    }
  }

  /* precompute metrics + early size rejection.
   * [F4] Use pixel_count (not bbox area) for the minimum size guard,
   *      matching OpenCV's contourArea(c) > 300 pre-filter. */
  for (int b = 0; b < n_blobs; b++) {
    if (!blobs[b].active) continue;
    blobs[b].w    = blobs[b].max_x - blobs[b].min_x + 1;
    blobs[b].h    = blobs[b].max_y - blobs[b].min_y + 1;
    blobs[b].area = blobs[b].w * blobs[b].h;
    // blobs[b].area = blobs[b].pixel_count;  /* [F4] area = actual filled pixels */
    blobs[b].cx   = blobs[b].min_x + blobs[b].w / 2;
    blobs[b].cy   = blobs[b].min_y + blobs[b].h / 2;
    if (blobs[b].pixel_count < MIN_BLOB_AREA) blobs[b].active = 0;  /* [F4] */
  }

  return n_blobs;
}

/* -------------------------------------------------------------------------
 * Holdover helper — report last known detection on dropout
 * ------------------------------------------------------------------------- */
static void report_holdover(void)
{
  if (holdover > 0) {
    holdover--;
    cont_est.gate_detected = 1;
    cont_est.contour_d_y   = hold_d_y;
    cont_est.contour_d_z   = hold_d_z;
    cont_est.contour_d_x   = hold_dist;
    cont_est.contour_area  = hold_area;
  } else {
    cont_est.gate_detected = 0;
  }
}

/* -------------------------------------------------------------------------
 * Main detection function
 * ------------------------------------------------------------------------- */
void find_contour(char *img, int width, int height)
{
  if (width > MAX_IMG_WIDTH || height > MAX_IMG_HEIGHT) {
    cont_est.gate_detected = 0;
    return;
  }

  uint8_t *buf = (uint8_t *)img;

  /* -----------------------------------------------------------------
   * Step 1: single pass — classify + overlay.
   *
   * Process two columns at a time — UYVY shares U/V between col pairs,
   * so read chroma once and classify both Y values separately.
   * ----------------------------------------------------------------- */
  static uint8_t mask[MAX_IMG_PIXELS];

  int even_width = width & ~1;

  for (int row = 0; row < height; row++) {
    for (int col = 0; col < even_width; col += 2) {
      int i = row * width + col;
      uint8_t u  = buf[2 * i];
      uint8_t y0 = buf[2 * i + 1];
      uint8_t v  = buf[2 * i + 2];
      uint8_t y1 = buf[2 * i + 3];

      uint8_t g0 = classify_yuv_pixel(y0, u, v) ? 1u : 0u;
      uint8_t g1 = classify_yuv_pixel(y1, u, v) ? 1u : 0u;
      mask[i]     = g0;
      mask[i + 1] = g1;

      if (show_threshold_overlay) {
        if (g0) set_uyvy_pixel(buf, col,     row, width, Y_GREEN, U_GREEN, V_GREEN);
        if (g1) set_uyvy_pixel(buf, col + 1, row, width, Y_GREEN, U_GREEN, V_GREEN);
      }
    }

    /* handle last column if width is odd */
    if (width & 1) {
      int col = width - 1;
      int i   = row * width + col;
      uint8_t u  = buf[2 * i];
      uint8_t y0 = buf[2 * i + 1];
      uint8_t v  = buf[2 * i + 2];
      uint8_t g0 = classify_yuv_pixel(y0, u, v) ? 1u : 0u;
      mask[i] = g0;
      if (show_threshold_overlay && g0)
        set_uyvy_pixel(buf, col, row, width, Y_GREEN, U_GREEN, V_GREEN);
    }
  }

  /* -----------------------------------------------------------------
   * Step 2: blob finding with precomputed metrics + early rejection
   * ----------------------------------------------------------------- */
  static Blob blobs[MAX_BLOBS];
  int n_blobs = find_blobs(mask, width, height, blobs);

  static int active[MAX_BLOBS];
  int na = 0;
  for (int b = 0; b < n_blobs; b++) {
    if (blobs[b].active) active[na++] = b;
  }

  /* draw yellow rectangle around every active blob so we can see what is detected */
  if (show_threshold_overlay) {
    for (int b = 0; b < na; b++) {
      int idx = active[b];
      draw_rect(buf, width, height,
                blobs[idx].min_x, blobs[idx].min_y,
                blobs[idx].max_x, blobs[idx].max_y,
                Y_YELLOW, U_YELLOW, V_YELLOW, 1);
    }
  }

  if (na < 2) {
    DPRINT("[CONTOUR] Less than 2 blobs: %d\n", na);
    report_holdover();
    return;
  }

  /* -----------------------------------------------------------------
   * Step 3: pair blobs — integer geometry, no float division.
   *
   * Threshold cross-multiplied integer equivalents:
   *   ratio_max=1.7   → a_big*10 > a_small*17
   *   blob_max=12000  → ai > 12000 || aj > 12000          [F1] was 14000
   *   sep_min=h*0.15  → sep*20 < h*3
   *   sep_max=h*0.70  → sep*10 > h*7
   *   xdiff_max=w*0.15→ xdiff*20 > w*3
   *   asp_min=1.5     → w*2 < h*3
   *   asp_max=3.5     → w*2 > h*7
   *   asp_r_max=1.2   → (w1*h2)*5 > (w2*h1)*6
   *
   * Score formula [F2]:
   *   OpenCV: ratio + (x_diff/width)*2
   *   → integer: (a_big*100/a_small - 100) + x_diff*200/width
   *   Both terms are in [0, ~170] and [0, 200] range respectively,
   *   giving balanced weighting without float division.
   * ----------------------------------------------------------------- */
  int best_i     = -1;
  int best_j     = -1;
  int best_score = 0x7fffffff;

  for (int ii = 0; ii < na; ii++) {
    for (int jj = ii + 1; jj < na; jj++) {
      int bi = active[ii];
      int bj = active[jj];

      /* ensure bi=top bj=bottom */
      if (blobs[bi].cy > blobs[bj].cy) { int tmp = bi; bi = bj; bj = tmp; }

      int ai     = blobs[bi].area;
      int aj     = blobs[bj].area;
      int wi     = blobs[bi].w;
      int hi     = blobs[bi].h;
      int wj     = blobs[bj].w;
      int hj     = blobs[bj].h;
      int sep    = blobs[bj].cy - blobs[bi].cy;
      int x_diff = blobs[bi].cx - blobs[bj].cx;
      if (x_diff < 0) x_diff = -x_diff;
      int total  = ai + aj;

      int64_t ai64 = ai, aj64 = aj;
      int64_t wi64 = wi, hi64 = hi, wj64 = wj, hj64 = hj;
      int64_t sep64 = sep, xd64 = x_diff;

      /* [F1] blob_max corrected from 14000 → 12000 to match OpenCV */
      // if (ai > aj) { if (ai64 * 10 > aj64 * 17) continue; }
      // else         { if (aj64 * 10 > ai64 * 17) continue; }
      // if (total > 25000)            continue;
      // if (ai > 12000 || aj > 12000) continue;  /* [F1] was 14000 */
      // if (sep64 * 20 < (int64_t)height * 3) continue;
      // if (sep64 * 10 > (int64_t)height * 7) continue;
      // if (xd64  * 20 > (int64_t)width  * 3) continue;
      // if (wi64  * 2  < hi64 * 3)   continue;
      // if (wj64  * 2  < hj64 * 3)   continue;
      // if (wi64  * 2  > hi64 * 7)   continue;
      // if (wj64  * 2  > hj64 * 7)   continue;

      // int64_t asp_num = wi64 * hj64;
      // int64_t asp_den = wj64 * hi64;
      // if (asp_num > asp_den) { if (asp_num * 5 > asp_den * 8) continue; }
      // else                   { if (asp_den * 5 > asp_num * 8) continue; }

      /* --- Ratio check --- */
      if (ai > aj) {
        if (ai64 * 10 > aj64 * 17) {
          printf("[REJECT] ratio too big (top bigger): ai=%d aj=%d ratio=%.2f\n",
                ai, aj, (float)ai/aj);
          continue;
        }
      } else {
        if (aj64 * 10 > ai64 * 17) {
          printf("[REJECT] ratio too big (bottom bigger): ai=%d aj=%d ratio=%.2f\n",
                ai, aj, (float)aj/ai);
          continue;
        }
      }

      /* --- Total area --- */
      if (total > 25000) {
        printf("[REJECT] total too big: total=%d\n", total);
        continue;
      }

      /* --- Individual blob size --- */
      if (ai > 12000 || aj > 12000) {
        printf("[REJECT] blob too big: ai=%d aj=%d\n", ai, aj);
        continue;
      }

      /* --- Vertical separation --- */
      if (sep64 * 20 < (int64_t)height * 3) {
        printf("[REJECT] sep too small: sep=%d (%.2f%% height)\n",
              sep, 100.f * sep / height);
        continue;
      }

      if (sep64 * 10 > (int64_t)height * 7) {
        printf("[REJECT] sep too large: sep=%d (%.2f%% height)\n",
              sep, 100.f * sep / height);
        continue;
      }

      /* --- Horizontal alignment --- */
      if (xd64 * 20 > (int64_t)width * 3) {
        printf("[REJECT] x_diff too large: xd=%d (%.2f%% width)\n",
              x_diff, 100.f * x_diff / width);
        continue;
      }

      /* --- Aspect ratio --- */
      if (wi64 * 2 < hi64 * 3) {
        printf("[REJECT] blob i too tall: w=%d h=%d ratio=%.2f\n",
              wi, hi, (float)wi/hi);
        continue;
      }

      if (wj64 * 2 < hj64 * 3) {
        printf("[REJECT] blob j too tall: w=%d h=%d ratio=%.2f\n",
              wj, hj, (float)wj/hj);
        continue;
      }

      if (wi64 * 2 > hi64 * 7) {
        printf("[REJECT] blob i too wide: w=%d h=%d ratio=%.2f\n",
              wi, hi, (float)wi/hi);
        continue;
      }

      if (wj64 * 2 > hj64 * 7) {
        printf("[REJECT] blob j too wide: w=%d h=%d ratio=%.2f\n",
              wj, hj, (float)wj/hj);
        continue;
      }

      /* --- Cross aspect consistency --- */
      int64_t asp_num = wi64 * hj64;
      int64_t asp_den = wj64 * hi64;

      if (asp_num > asp_den) {
        if (asp_num * 5 > asp_den * 8) {
          printf("[REJECT] aspect mismatch (case 1)\n");
          continue;
        }
      } else {
        if (asp_den * 5 > asp_num * 8) {
          printf("[REJECT] aspect mismatch (case 2)\n");
          continue;
        }
      }

      /* [F2] Restored area-ratio term in score — was only x_diff*1000/width.
       *      OpenCV score: ratio + (x_diff/width)*2
       *      Integer form: (a_big*100/a_small - 100) + x_diff*200/width
       *      a_big*100/a_small gives ratio*100 (e.g. 1.5 → 150),
       *      subtract 100 to zero-base it (perfect pair = 0).            */
      int a_big   = ai > aj ? ai : aj;
      int a_small = ai > aj ? aj : ai;
      int ratio_score = (int)((int64_t)a_big * 100 / a_small) - 100;
      int xdiff_score = (int)(xd64 * 200 / width);
      int score = ratio_score + xdiff_score;

      if (score < best_score) {
        best_score = score;
        best_i = bi;
        best_j = bj;
      }
    }
  }

  if (best_i < 0) {
    DPRINT("[CONTOUR] No valid pair found\n");
    report_holdover();
    return;
  }

  /* -----------------------------------------------------------------
   * Step 4: compute gate position and fill cont_est
   * ----------------------------------------------------------------- */
  float top_cx    = (float)blobs[best_i].cx;
  float top_cy    = (float)blobs[best_i].cy;
  float bottom_cx = (float)blobs[best_j].cx;
  float bottom_cy = (float)blobs[best_j].cy;

  /* floor rejection — integer: cy*100 > height*99 */
  if (blobs[best_j].cy * 100 > height * 99) {
    DPRINT("[CONTOUR] Bottom blob too low\n");
    cont_est.gate_detected = 0;
    return;
  }

  float gate_cx = (top_cx + bottom_cx) * 0.5f;
  float gate_cy = (top_cy + bottom_cy) * 0.5f;

  /* [F3] Spatial consistency — only active in tracking mode.
   *      OpenCV gated this on gate_tracking==1; the original port
   *      ran it unconditionally, rejecting valid first-approach frames. */
  if (gate_tracking == 1 && last_gate_cx >= 0.f) {
    float dx = gate_cx - last_gate_cx; if (dx < 0.f) dx = -dx;
    float dy = gate_cy - last_gate_cy; if (dy < 0.f) dy = -dy;
    if (dx * 4.f > (float)width || dy * 4.f > (float)height) {
      DPRINT("[CONTOUR] Tracking: gate jumped, rejecting (dx=%.0f dy=%.0f)\n",
             dx, dy);
      cont_est.gate_detected = 0;
      return;
    }
  }

  last_gate_cx = gate_cx;
  last_gate_cy = gate_cy;

  float area = (float)(blobs[best_i].area + blobs[best_j].area);

  float dist;
  if      (area > 28000.f) dist = 0.1f;
  else if (area > 20000.f) dist = 0.5f;
  else if (area > 14000.f) dist = 1.0f;
  else if (area >  3000.f) dist = 1.5f;
  else                     dist = 2.5f;

  cont_est.gate_detected = 1;
  cont_est.contour_d_x   = dist;
  cont_est.contour_d_y   = (gate_cy - height * 0.5f) / (float)height;
  cont_est.contour_d_z   = -(gate_cx - width  * 0.5f) / (float)width;
  cont_est.contour_area  = area;

  /* update holdover with current detection */
  holdover  = MAX_HOLDOVER;
  hold_d_y  = cont_est.contour_d_y;
  hold_d_z  = cont_est.contour_d_z;
  hold_dist = cont_est.contour_d_x;
  hold_area = cont_est.contour_area;

  DPRINT("[CONTOUR] top=(%.0f,%.0f) bot=(%.0f,%.0f) gate=(%.0f,%.0f) "
         "dist=%.2f lat=%.2f area=%.0f asp1=%.1f asp2=%.1f score=%d [%s]\n",
         top_cx, top_cy, bottom_cx, bottom_cy, gate_cx, gate_cy,
         dist, cont_est.contour_d_y, area,
         (float)blobs[best_i].w / blobs[best_i].h,
         (float)blobs[best_j].w / blobs[best_j].h,
         best_score,
         gate_tracking ? "TRACKING" : "SEARCH");

  /* -----------------------------------------------------------------
   * Step 5: draw detection overlay — cyan rects + green cross
   * ----------------------------------------------------------------- */
  if (show_threshold_overlay) {
    draw_rect(buf, width, height,
              blobs[best_i].min_x, blobs[best_i].min_y,
              blobs[best_i].max_x, blobs[best_i].max_y,
              Y_CYAN, U_CYAN, V_CYAN, 2);
    draw_rect(buf, width, height,
              blobs[best_j].min_x, blobs[best_j].min_y,
              blobs[best_j].max_x, blobs[best_j].max_y,
              Y_CYAN, U_CYAN, V_CYAN, 2);
    draw_cross(buf, width, height,
               (int)gate_cx, (int)gate_cy, 15,
               Y_GREEN, U_GREEN, V_GREEN);
  }
}