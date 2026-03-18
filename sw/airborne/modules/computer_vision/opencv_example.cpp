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


#include "opencv_example.h"



using namespace std;
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;
#include "opencv_image_functions.h"

#ifndef EDGE_THRESHOLD
#define EDGE_THRESHOLD 35
#endif
int edge_thresh = EDGE_THRESHOLD;

#ifndef GREEN_THRESH_VALUE
#define GREEN_THRESH_VALUE 160
#endif
int green_thresh_value = GREEN_THRESH_VALUE;

#ifndef FLOOR_MARGIN
#define FLOOR_MARGIN 100
#endif
int floor_margin = FLOOR_MARGIN;
int edge_count_left   = 0;
int edge_count_center = 0;
int edge_count_right  = 0;
int edge_count_total  = 0;

int floor_area_left   = 99999;
int floor_area_center = 99999;
int floor_area_right  = 99999;


int opencv_example(char *img, int width, int height)
{
  // Create a new image, using the original bebop image.
  Mat M(height, width, CV_8UC2, img);
  Mat image;

#if OPENCVDEMO_GRAYSCALE
  //  Grayscale image example
  cvtColor(M, image, cv::COLOR_YUV2BGR_YUY2);
  GaussianBlur(image, image, Size(5, 5), 0);
  Mat hsv;
  cvtColor(image, hsv, cv::COLOR_BGR2HSV);
  Mat greenMask;
  inRange(hsv,
          Scalar(40, 150, 50),   // hue 40-75, saturation >150, value 50-180
          Scalar(75, 255, green_thresh_value),  // upper value cuts out bright wall
          greenMask);

  // Find largest green contour and fill it as the floor mask
  vector<vector<Point>> contours;
  findContours(greenMask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

  Mat floorMask = Mat::zeros(greenMask.size(), CV_8UC1);
  if (!contours.empty()) {
      int largestIdx = 0;
      double largestArea = 0;
      for (int i = 0; i < (int)contours.size(); i++) {
          double area = contourArea(contours[i]);
          if (area > largestArea) {
              largestArea = area;
              largestIdx = i;
          }
      }
      drawContours(floorMask, contours, largestIdx, Scalar(255), -1);
  }

  // Export floor area per real-world horizontal third for obstacle_avoider.
  // Camera is rotated 90°: rows map to the real-world horizontal axis.
  {
    int third_h = floorMask.rows / 3;
    floor_area_left   = countNonZero(floorMask(Rect(0, 0,         floorMask.cols, third_h)));
    floor_area_center = countNonZero(floorMask(Rect(0, third_h,   floorMask.cols, third_h)));
    floor_area_right  = countNonZero(floorMask(Rect(0, 2*third_h, floorMask.cols, third_h)));
  }

  // Find horizon row (topmost floor pixel)
  int horizonRow = floorMask.rows;
  for (int row = 0; row < floorMask.rows; row++) {
      for (int col = 0; col < floorMask.cols; col++) {
          if (floorMask.at<uchar>(row, col) > 0) {
              horizonRow = row;
              goto foundHorizon;
          }
      }
  }
  foundHorizon:

  // Find left and right boundaries
  int leftBound = floorMask.cols;
  int rightBound = 0;
  for (int col = 0; col < floorMask.cols; col++) {
      for (int row = 0; row < floorMask.rows; row++) {
          if (floorMask.at<uchar>(row, col) > 0) {
              if (col < leftBound) leftBound = col;
              if (col > rightBound) rightBound = col;
              break;
          }
      }
  }

  // Build dilated mask: below horizon with margin, above without
  int margin = floor_margin;
  Mat dilatedMask = Mat::zeros(floorMask.size(), CV_8UC1);
  rectangle(dilatedMask,
            Point(max(0, leftBound - margin), horizonRow),
            Point(min(floorMask.cols, rightBound + margin), floorMask.rows),
            Scalar(255), -1);
  rectangle(dilatedMask,
            Point(leftBound, 0),
            Point(rightBound, horizonRow),
            Scalar(255), -1);
  dilatedMask |= floorMask;

  // Run Canny on the whole image, then mask to floor region
  Mat edges;
  Canny(image, edges, edge_thresh, edge_thresh * 3);
  Mat maskedEdges;
  if (contours.empty()) {
    maskedEdges = edges;
  } else {
    edges.copyTo(maskedEdges, dilatedMask);
  }

  // Count edges in left, center, right thirds.
  // Camera is rotated 90°: rows map to the real-world horizontal axis.
  int third = maskedEdges.rows / 3;
  Mat leftRegion   = maskedEdges(Rect(0, 0,       maskedEdges.cols, third));
  Mat centerRegion = maskedEdges(Rect(0, third,   maskedEdges.cols, third));
  Mat rightRegion  = maskedEdges(Rect(0, 2*third, maskedEdges.cols, third));

  edge_count_left   = countNonZero(leftRegion);
  edge_count_center = countNonZero(centerRegion);
  edge_count_right  = countNonZero(rightRegion);
  edge_count_total  = edge_count_left + edge_count_center + edge_count_right;

  grayscale_opencv_to_yuv422(maskedEdges, img, width, height);
#else // OPENCVDEMO_GRAYSCALE
  // Color image example
  cvtColor(M, image, cv::COLOR_YUV2BGR_YUY2);
  blur(image, image, Size(5, 5));
  colorbgr_opencv_to_yuv422(image, img, width, height);
#endif // OPENCVDEMO_GRAYSCALE

  return 0;
}
