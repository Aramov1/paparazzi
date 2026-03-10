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
 * @file "modules/computer_vision/opencv_contour.cpp"
 * @author Roland Meertens and Peng Lu
 *
 */

#include "opencv_contour.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv_image_functions.h"
#include "modules/core/abi.h"
using namespace cv;
using namespace std;

struct contour_estimation cont_est;
struct contour_threshold cont_thres;

RNG rng(12345);

// Convert OpenCV RGB image back to Paparazzi YUV422 buffer
static void yuv_opencv_to_yuv422(Mat image, char *img, int width, int height)
{
  for (int row = 0; row < height; row++) {
    for (int col = 0; col < width; col++) {
      Vec3b &c = image.at<Vec3b>(row, col);

      int i = row * width + col;
      img[2 * i + 1] = c[0];                // Y
      img[2 * i] = (col % 2) ? c[1] : c[2]; // U or V
    }
  }
}

// Convert UYVY-like OpenCV image to YUV OpenCV image
static void uyvy_opencv_to_yuv_opencv(Mat image, Mat image_in, int width, int height)
{
  for (int row = 0; row < height; row++) {
    for (int col = 0; col < width; col++) {
      Vec3b c = image_in.at<Vec3b>(row, col);
      Vec3b c_m1 = image_in.at<Vec3b>(row, col);
      Vec3b c_p1 = image_in.at<Vec3b>(row, col);

      if (col > 0) {
        c_m1 = image_in.at<Vec3b>(row, col - 1);
      }
      if (col < width - 1) {
        c_p1 = image_in.at<Vec3b>(row, col + 1);
      }

      image.at<Vec3b>(row, col)[0] = c[1];
      image.at<Vec3b>(row, col)[1] = (col % 2) ? c[0] : c_m1[0];
      image.at<Vec3b>(row, col)[2] = (col % 2) ? c_p1[0] : c[0];
    }
  }
}
void find_contour(char *img, int width, int height)
{
  // Wrap incoming Paparazzi image buffer
  Mat raw(height, width, CV_8UC2, img);

  // Convert packed UYVY to RGB for processing and drawing
  Mat rgb;
  cvtColor(raw, rgb, cv::COLOR_YUV2RGB_UYVY);

  putText(rgb, "TREE DETECTOR ACTIVE",
          Point(20, 40),
          FONT_HERSHEY_SIMPLEX,
          0.8,
          Scalar(255, 255, 255),
          2);

  // ------------------------------------------------------------
  // HSV threshold
  // ------------------------------------------------------------
  Mat hsv;
  cvtColor(rgb, hsv, cv::COLOR_RGB2HSV);

  Mat thresh_image;
  inRange(hsv,
          Scalar(30, 60, 40),
          Scalar(100, 255, 255),
          thresh_image);

  // Optional: reduce isolated noise
  medianBlur(thresh_image, thresh_image, 5);

  

  // ------------------------------------------------------------
  // Morphology
  // ------------------------------------------------------------
  Mat kernel_small = getStructuringElement(MORPH_ELLIPSE, Size(3, 3));
  Mat kernel_big   = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));

  morphologyEx(thresh_image, thresh_image, MORPH_OPEN, kernel_small);
  morphologyEx(thresh_image, thresh_image, MORPH_CLOSE, kernel_big);
// ------------------------------------------------------------
  // Remove bottom third of the image
  // ------------------------------------------------------------
  int cut_y = (int)(2.0 * height / 3.0);
  rectangle(thresh_image,
            Rect(0, cut_y, width, height - cut_y),
            Scalar(0),
            FILLED);
  // ------------------------------------------------------------
  // Find contours
  // ------------------------------------------------------------
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;
  findContours(thresh_image, contours, hierarchy,
               cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, Point(0, 0));

  vector<Rect> tree_boxes;
  vector<float> tree_scores;

  for (unsigned int i = 0; i < contours.size(); i++) {
    double a = contourArea(contours[i], false);

    if (a < 100.0) {
      continue;
    }

    if (a > 0.4 * width * height) {
      continue;
    }

    Rect r = boundingRect(contours[i]);

    if (r.width < 10 || r.height < 10) {
      continue;
    }

    if (r.width > 0.8 * width || r.height > 0.8 * height) {
      continue;
    }

    float aspect_ratio = (float)r.height / (float)r.width;
    if (aspect_ratio < 0.3f || aspect_ratio > 5.0f) {
      continue;
    }

    float fill_ratio = (float)a / (float)(r.width * r.height);
    if (fill_ratio < 0.15f) {
      continue;
    }

    tree_boxes.push_back(r);
    tree_scores.push_back((float)a);   // contour area as score
  }

  // ------------------------------------------------------------
  // Draw all detected trees
  // ------------------------------------------------------------
  for (unsigned int i = 0; i < tree_boxes.size(); i++) {
    Rect r = tree_boxes[i];
    Point2f c(r.x + r.width / 2.0f, r.y + r.height / 2.0f);

    rectangle(rgb, r, Scalar(0, 255, 0), 2, 8, 0);
    circle(rgb, c, 4, Scalar(255, 0, 255), -1, 8, 0);

    char label[32];
    snprintf(label, sizeof(label), "TREE %d", (int)i + 1);
    putText(rgb, label,
            Point(r.x, max(20, r.y - 8)),
            FONT_HERSHEY_SIMPLEX,
            0.5,
            Scalar(0, 255, 0),
            1);
  }

  // ------------------------------------------------------------
  // Choose main tree for output
  // ------------------------------------------------------------
  if (!tree_boxes.empty()) {
    int best_idx = 0;
    float best_score = tree_scores[0];

    for (unsigned int i = 1; i < tree_boxes.size(); i++) {
      if (tree_scores[i] > best_score) {
        best_score = tree_scores[i];
        best_idx = (int)i;
      }
    }

    Rect best_box = tree_boxes[best_idx];
    Point2f best_center(best_box.x + best_box.width / 2.0f,
                        best_box.y + best_box.height / 2.0f);

    float contour_distance_est = 2.0f;
    float area = (float)(best_box.width * best_box.height);

    if (area > 28000.0f) {
      contour_distance_est = 0.1f;
    } else if (area > 16000.0f) {
      contour_distance_est = 0.5f;
    } else if (area > 11000.0f) {
      contour_distance_est = 1.0f;
    } else if (area > 3000.0f) {
      contour_distance_est = 1.5f;
    } else {
      contour_distance_est = 2.0f;
    }

    cont_est.contour_d_x = contour_distance_est;

    float Im_center_w = width / 2.0f;
    float Im_center_h = height / 2.0f;
    float real_size = 1.0f;

    cont_est.contour_d_y = -(best_center.x - Im_center_w) * real_size / float(best_box.width);
    cont_est.contour_d_z = -(best_center.y - Im_center_h) * real_size / float(best_box.height);

    // SEND ABI MESSAGE HERE
  /*AbiSendMsgTREE_POSITION(ABI_SENDER_TREE_DETECTOR,
                          cont_est.contour_d_x,
                          cont_est.contour_d_y,
                          cont_est.contour_d_z,
                          1);
*/
    char txt[160];
    snprintf(txt, sizeof(txt), "TREES=%d MAIN cx=%.1f cy=%.1f area=%.0f",
             (int)tree_boxes.size(), best_center.x, best_center.y, area);
    putText(rgb, txt,
            Point(20, height - 20),
            FONT_HERSHEY_SIMPLEX,
            0.5,
            Scalar(255, 255, 255),
            1);
  } else {
    cont_est.contour_d_x = -1.0f;
    cont_est.contour_d_y = 0.0f;
    cont_est.contour_d_z = 0.0f;
  // SEND ABI MESSAGE HERE TOO
  
  //AbiSendMsgTREE_POSITION(ABI_SENDER_TREE_DETECTOR,
                          //cont_est.contour_d_x,
                          //cont_est.contour_d_y,
                          //cont_est.contour_d_z,
                          //0);
    putText(rgb, "NO TREE DETECTED",
            Point(20, height - 20),
            FONT_HERSHEY_SIMPLEX,
            0.6,
            Scalar(255, 255, 255),
            2);
  }

  // ------------------------------------------------------------
  // Write processed image back to Paparazzi buffer
  // ------------------------------------------------------------
  yuv_opencv_to_yuv422(rgb, img, width, height);
}