#include <cstdio>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

extern "C" void tree_detection_process(char *buf, int width, int height, int camera_id)
{
  if (buf == nullptr || width <= 0 || height <= 0) {
    return;
  }

  // Paparazzi image buffer is YUV422 / YUYV
  cv::Mat yuv(height, width, CV_8UC2, buf);
  cv::Mat bgr;
  cv::cvtColor(yuv, bgr, cv::COLOR_YUV2BGR_YUYV);

  // Restrict detection to the upper/middle part of the image
  // to avoid detecting the green ground.
  const int roi_y = static_cast<int>(0.05 * height);
  const int roi_h = static_cast<int>(0.60 * height);
  cv::Rect roi_rect(0, roi_y, width, roi_h);
  cv::Mat roi_bgr = bgr(roi_rect);

  // Convert to HSV
  cv::Mat hsv;
  cv::cvtColor(roi_bgr, hsv, cv::COLOR_BGR2HSV);

  // Vegetation-like color masks:
  // 1) green foliage
  cv::Mat mask_green;
  cv::inRange(hsv,
              cv::Scalar(30, 35, 25),
              cv::Scalar(95, 255, 255),
              mask_green);

  // 2) optional brown / trunk-ish tones
  cv::Mat mask_brown;
  cv::inRange(hsv,
              cv::Scalar(5, 40, 20),
              cv::Scalar(25, 255, 200),
              mask_brown);

  cv::Mat vegetation_mask;
  cv::bitwise_or(mask_green, mask_brown, vegetation_mask);

  // Morphological cleanup
  cv::Mat kernel3 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  cv::Mat kernel5 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
  cv::morphologyEx(vegetation_mask, vegetation_mask, cv::MORPH_OPEN, kernel3);
  cv::morphologyEx(vegetation_mask, vegetation_mask, cv::MORPH_CLOSE, kernel5);

  // Edge density helps reject flat green patches
  cv::Mat gray, edges;
  cv::cvtColor(roi_bgr, gray, cv::COLOR_BGR2GRAY);
  cv::Canny(gray, edges, 60, 140);

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(vegetation_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  bool obstacle_left = false;
  bool obstacle_center = false;
  bool obstacle_right = false;

  double left_score = 0.0;
  double center_score = 0.0;
  double right_score = 0.0;

  cv::Rect best_box;
  double best_score = 0.0;

  for (const auto &contour : contours) {
    const double area = cv::contourArea(contour);
    if (area < 250.0) {
      continue;
    }

    cv::Rect box = cv::boundingRect(contour);

    // Reject wide, shallow regions that are likely ground remnants
    if (box.height < 20) {
      continue;
    }
    if (box.width > 2.5 * box.height) {
      continue;
    }

    // Require some vertical extent
    if (box.height < 0.12 * roi_h) {
      continue;
    }

    // Compute edge density inside the blob box
    cv::Rect bounded_box = box & cv::Rect(0, 0, edges.cols, edges.rows);
    if (bounded_box.area() <= 0) {
      continue;
    }

    cv::Mat edge_patch = edges(bounded_box);
    const int edge_pixels = cv::countNonZero(edge_patch);
    const double edge_density = static_cast<double>(edge_pixels) /
                                static_cast<double>(bounded_box.area());

    // Reject smooth blobs
    if (edge_density < 0.03) {
      continue;
    }

    // Obstacle score: mostly area, slightly boosted by verticality and texture
    const double verticality = static_cast<double>(box.height) /
                               static_cast<double>(std::max(box.width, 1));
    const double score = area * (1.0 + 0.15 * std::min(verticality, 4.0)) * (1.0 + edge_density);

    const int center_x = box.x + box.width / 2;

    if (center_x < width / 3) {
      left_score += score;
      obstacle_left = true;
    } else if (center_x < 2 * width / 3) {
      center_score += score;
      obstacle_center = true;
    } else {
      right_score += score;
      obstacle_right = true;
    }

    if (score > best_score) {
      best_score = score;
      best_box = box;
    }
  }

  // Simple urgency metric based on center clutter
  const double roi_area = static_cast<double>(roi_rect.area());
  const double center_fraction = center_score / std::max(roi_area, 1.0);
  const bool obstacle_ahead = center_fraction > 0.015 || center_score > left_score * 0.9 + right_score * 0.9;

  const char *suggested_turn = "NONE";
  if (obstacle_ahead) {
    suggested_turn = (left_score > right_score) ? "RIGHT" : "LEFT";
  }

  std::printf(
      "[tree_detector cam=%d] left=%.1f center=%.1f right=%.1f ahead=%d turn=%s best_box=(%d,%d,%d,%d)\n",
      camera_id,
      left_score,
      center_score,
      right_score,
      obstacle_ahead ? 1 : 0,
      suggested_turn,
      best_box.x,
      best_box.y + roi_y,
      best_box.width,
      best_box.height
  );
}