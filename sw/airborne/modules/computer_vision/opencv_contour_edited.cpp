#include "opencv_contour_edited.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv_image_functions.h"

using namespace cv;
using namespace std;

struct contour_estimation cont_est;
struct contour_threshold cont_thres;
int show_threshold_overlay = 0;
extern int gate_locked;
extern int gate_tracking;

static float last_gate_cx = -1.f;
static float last_gate_cy = -1.f;
RNG rng(12345);


void yuv_opencv_to_yuv422(Mat image, char *img, int width, int height)
{
  for (int row = 0; row < height; row++) {
    for (int col = 0; col < width; col++) {
      cv::Vec3b &c = image.at<cv::Vec3b>(row, col);
      int i = row * width + col;
      img[2 * i]     = c[0];                    // Y in position 0
      img[2 * i + 1] = col % 2 ? c[2] : c[1];  // U on even, V on odd
    }
  }
}

void uyvy_opencv_to_yuv_opencv(Mat image, Mat image_in, int width, int height)
{
  for (int row = 0; row < height; row++) {
    for (int col = 0; col < width; col++) {
      cv::Vec3b c    = image_in.at<cv::Vec3b>(row, col);
      cv::Vec3b c_m1 = image_in.at<cv::Vec3b>(row, col);
      cv::Vec3b c_p1 = image_in.at<cv::Vec3b>(row, col);
      if (col > 0)         c_m1 = image_in.at<cv::Vec3b>(row, col - 1);
      if (col < width - 1) c_p1 = image_in.at<cv::Vec3b>(row, col + 1);
      image.at<cv::Vec3b>(row, col)[0] = c[1];
      image.at<cv::Vec3b>(row, col)[1] = col % 2 ? c[0] : c_m1[0];
      image.at<cv::Vec3b>(row, col)[2] = col % 2 ? c_p1[0] : c[0];
    }
  }
}

extern "C" {
  void contour_reset_tracking(void)
{
  last_gate_cx = -1.f;
  last_gate_cy = -1.f;
}
void find_contour(char *img, int width, int height)
{

  Mat M(height, width, CV_8UC2, img);
  Mat image;
  cvtColor(M, image, CV_YUV2BGR_Y422);  // Paparazzi standard UYVY→BGR
  Mat yuv;
  cvtColor(image, yuv, cv::COLOR_BGR2YUV);

  Mat thresh_image;
  inRange(yuv,
          Scalar(CONTOUR_LOWER_Y, CONTOUR_LOWER_U, CONTOUR_LOWER_V),
          Scalar(CONTOUR_UPPER_Y, CONTOUR_UPPER_U, CONTOUR_UPPER_V),
          thresh_image);

  Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
  morphologyEx(thresh_image, thresh_image, MORPH_CLOSE, kernel);

  vector<vector<Point>> contours;
  vector<Vec4i> hierarchy;
  findContours(thresh_image, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

  vector<vector<Point>> valid_contours;
  for (auto &c : contours) {
    if (contourArea(c) > 300) valid_contours.push_back(c);
  }

  // Build overlay base (green threshold)
  Mat overlay(height, width, CV_8UC3);
  if (show_threshold_overlay) {
    overlay = image.clone();
    for (int row = 0; row < height; row++) {
      for (int col = 0; col < width; col++) {
        if (thresh_image.at<uint8_t>(row, col) > 0) {
          overlay.at<cv::Vec3b>(row, col)[0] = 0;
          overlay.at<cv::Vec3b>(row, col)[1] = 255;
          overlay.at<cv::Vec3b>(row, col)[2] = 0;
        }
      }
    }
  }

  auto write_overlay = [&]() {
    if (show_threshold_overlay) {
      colorbgr_opencv_to_yuv422(overlay, img, width, height);
    }
  };

  if (valid_contours.size() < 2) {
    printf("[CONTOUR] Less than 2 blobs: %d\n", (int)valid_contours.size());
    write_overlay();
    return;
  }
  
  bool tracking = (gate_tracking == 1);

  // float ratio_max  = tracking ? 2.0f          : 1.7f;
  // float total_max  = tracking ? 25000.f        : 25000.f;
  // float blob_max   = tracking ? 12000.f        : 12000.f;
  // float sep_min    = tracking ? height * 0.08f : height * 0.15f;
  // float sep_max    = tracking ? height * 0.85f : height * 0.70f;
  // float xdiff_max  = tracking ? width  * 0.25f : width  * 0.15f;
  // float asp_min    = tracking ? 1.0f           : 1.5f;
  // float asp_max    = tracking ? 8.0f          : 3.5f;
  // float asp_r_max  = tracking ? 1.8f           : 1.2f;
  // float floor_thr  = tracking ? 0.99f          : 0.97f;

  float ratio_max  = 1.7f;
  float total_max  = 25000.f;
  float blob_max   = 12000.f;
  float sep_min    = height * 0.15f;
  float sep_max    = height * 0.70f;
  float xdiff_max  = width  * 0.15f;
  float asp_min    = 1.5f;
  float asp_max    = 3.5f;
  float asp_r_max  = 1.2f;
  float floor_thr  = 0.97f;

  Rect best_top, best_bottom;
  float best_score = 1e9f;
  bool found_pair = false;

  for (int i = 0; i < (int)valid_contours.size(); i++) {
    for (int j = i + 1; j < (int)valid_contours.size(); j++) {
      Rect ra = boundingRect(valid_contours[i]);
      Rect rb = boundingRect(valid_contours[j]);

      float ca_y = ra.y + ra.height / 2.0f;
      float cb_y = rb.y + rb.height / 2.0f;
      float ca_x = ra.x + ra.width  / 2.0f;
      float cb_x = rb.x + rb.width  / 2.0f;

      // Ensure ra=top, rb=bottom
      if (ca_y > cb_y) {
        swap(ra, rb);
        swap(ca_y, cb_y);
        swap(ca_x, cb_x);
      }

      float a1     = ra.width * ra.height;
      float a2     = rb.width * rb.height;
      float ratio  = a1 > a2 ? a1 / a2 : a2 / a1;
      float x_diff = fabsf(ca_x - cb_x);
      float sep    = cb_y - ca_y;
      float total  = a1 + a2;
      float asp1   = (float)ra.width / ra.height;
      float asp2   = (float)rb.width / rb.height;
      float asp_ratio = asp1 > asp2 ? asp1 / asp2 : asp2 / asp1;

      if (ratio     > ratio_max)  continue;
      if (total     > total_max)  continue;
      if (a1        > blob_max || a2 > blob_max) continue;
      if (sep       < sep_min)    continue;
      if (sep       > sep_max)    continue;
      if (x_diff    > xdiff_max)  continue;
      if (asp1      < asp_min || asp2 < asp_min) continue;
      if (asp1      > asp_max || asp2 > asp_max) continue;
      if (asp_ratio > asp_r_max)  continue;

      float score = ratio + (x_diff / width) * 2.0f;
      if (score < best_score) {
        best_score  = score;
        best_top    = ra;
        best_bottom = rb;
        found_pair  = true;
      }
    }
  }

  if (!found_pair) {
    printf("[CONTOUR] No valid pair found\n");
    if (show_threshold_overlay) {
      for (auto &c : valid_contours)
        rectangle(overlay, boundingRect(c), Scalar(255, 255, 0), 2);
    }
    write_overlay();
    return;
  }

  float top_cx    = best_top.x    + best_top.width    / 2.0f;
  float bottom_cx = best_bottom.x + best_bottom.width / 2.0f;
  float top_cy    = best_top.y    + best_top.height   / 2.0f;
  float bottom_cy = best_bottom.y + best_bottom.height / 2.0f;

  // FIX: relaxed from 0.9 to 0.97 — was rejecting valid detections at close range
  if (bottom_cy > height * 0.99f) {
    printf("[CONTOUR] Bottom blob too low (floor?)\n");
    if (show_threshold_overlay) {
      rectangle(overlay, best_top,    Scalar(255, 128, 0), 2);
      rectangle(overlay, best_bottom, Scalar(255, 128, 0), 2);
    }
    write_overlay();
    return;
  }

  float gate_cx = (top_cx + bottom_cx) / 2.0f;
  float gate_cy = (top_cy + bottom_cy) / 2.0f;

  printf("[CONTOUR] top_cx=%.0f top_cy=%.0f bot_cx=%.0f bot_cy=%.0f gate_cx=%.0f gate_cy=%.0f img=%dx%d\n",
       top_cx, top_cy, bottom_cx, bottom_cy, gate_cx, gate_cy, width, height);
  printf("[CONTOUR] d_y=%.3f d_z=%.3f (d_y=lateral d_z=vertical)\n",
       cont_est.contour_d_y, cont_est.contour_d_z);

  // --- Spatial consistency check (tracking mode only) ---
  // Reject if gate center jumps more than 25% of image in one frame — likely false positive
  if (tracking && last_gate_cx >= 0.f) {
    float dx = fabsf(gate_cx - last_gate_cx);
    float dy = fabsf(gate_cy - last_gate_cy);
    if (dx > width * 0.25f || dy > height * 0.25f) {
      printf("[CONTOUR] Tracking: gate jumped, rejecting (dx=%.0f dy=%.0f)\n", dx, dy);
      if (show_threshold_overlay) {
        rectangle(overlay, best_top,    Scalar(255, 0, 0), 2);
        rectangle(overlay, best_bottom, Scalar(255, 0, 0), 2);
      }
      write_overlay();
      return;
    }
  }

  // Update spatial memory
  last_gate_cx = gate_cx;
  last_gate_cy = gate_cy;

  float area = best_top.width  * best_top.height +
               best_bottom.width * best_bottom.height;

  float dist;
  if      (area > 28000.f) dist = 0.1f;
  else if (area > 20000.f) dist = 0.5f;
  else if (area > 14000.f) dist = 1.0f;
  else if (area >  3000.f) dist = 1.5f;
  else                     dist = 2.5f;

  cont_est.gate_detected = 1;
  cont_est.contour_d_x   = dist;
  cont_est.contour_d_y   = (gate_cy - height / 2.0f) / (float)height;
  cont_est.contour_d_z   = -(gate_cx - width  / 2.0f) / (float)width;
  cont_est.contour_area  = area;  // FIX: expose raw area for CROSS trigger in gate_nav

  printf("[CONTOUR] Gate confirmed! dist=%.2f lateral=%.2f area=%.0f asp1=%.1f asp2=%.1f [%s]\n",
         dist, cont_est.contour_d_y, area,
         (float)best_top.width / best_top.height,
         (float)best_bottom.width / best_bottom.height,
         tracking ? "RELAXED" : "STRICT");

  if (show_threshold_overlay) {
    rectangle(overlay, best_top,    Scalar(0, 255, 255), 3);
    rectangle(overlay, best_bottom, Scalar(0, 255, 255), 3);
    drawMarker(overlay, Point((int)gate_cx, (int)gate_cy),
               Scalar(0, 255, 0), MARKER_CROSS, 30, 2);
  }
  write_overlay();
}
} // end extern "C"