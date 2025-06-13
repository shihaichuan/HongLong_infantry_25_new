// Copyright (c) 2022 ChenJun
// Licensed under the MIT License.

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/core/base.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

// STD
#include <algorithm>
#include <cmath>
#include <execution>
#include <vector>

#include "armor_detector/detector.hpp"
#include "base_interfaces/msg/debug_armor.hpp"
#include "base_interfaces/msg/debug_light.hpp"
#include "rm_utils/common.hpp"
#include "armor_detector/types.hpp"



namespace rm_auto_aim {
Detector::Detector(const int& bin_thres, 
                   const int& color, 
                   const LightParams& l,
                   const ArmorParams& a)
    : binary_thres(bin_thres), detect_color(color), light_params(l), armor_params(a)  {}

std::vector<Armor> Detector::detect(const cv::Mat &input) {
      // 1. Preprocess the image
    binary_img = preprocessImage(input);
      // 2. Find lights
    lights_ = findLights(input,binary_img);
      // 3. Match lights to armors
    armors_ = matchLights(lights_);
    if (!armors_.empty() && classifier != nullptr) {

        // Parallel processing
      classifier->extractNumber(input, armors_);
      classifier->classify(armors_);
      if (corner_corrector != nullptr) {
        std::for_each(std::execution::par, armors_.begin(), armors_.end(),[this, &input](Armor &armor){
    corner_corrector->correctCorners(armor, gray_img_);
      });
      }
    }
      return armors_;
}

cv::Mat Detector::preprocessImage(const cv::Mat& rgb_img) {
  cv::Mat gray_img;
  cv::cvtColor(rgb_img, gray_img, cv::COLOR_RGB2GRAY);
  cv::Mat binary_img;
  cv::threshold(gray_img, binary_img, binary_thres, 255, cv::THRESH_BINARY);
  return binary_img;
}

std::vector<Light> Detector::findLights(const cv::Mat &rgb_img,
  const cv::Mat &binary_img) noexcept {
  using std::vector;
  vector<vector<cv::Point>> contours;
  vector<cv::Vec4i> hierarchy;
  cv::findContours(binary_img, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
  vector<Light> lights;
  debug_lights.data.clear();
  for (const auto &contour : contours) {
    if (contour.size() < 5) continue;
    Light light(contour);
    auto b_rect = cv::boundingRect(contour);
    auto r_rect = cv::minAreaRect(contour);
    cv::Mat mask = cv::Mat::zeros(b_rect.size(), CV_8UC1);
    std::vector<cv::Point> mask_contour;
    for (const auto & p : contour) {
      mask_contour.emplace_back(p - cv::Point(b_rect.x, b_rect.y));
    }
    cv::fillPoly(mask, {mask_contour}, 255);
    std::vector<cv::Point> points;
    cv::findNonZero(mask, points);
    // points / rotated rect area
    bool is_fill_rotated_rect =
     points.size() / (r_rect.size.width * r_rect.size.height) > light_params.min_fill_ratio;
  if (isLight(light) && is_fill_rotated_rect) {
 
        int sum_r = 0, sum_b = 0;
        auto roi = rgb_img(b_rect);
        // Iterate through the ROI
        for (int i = 0; i < roi.rows; i++) {
          for (int j = 0; j < roi.cols; j++) {
              // if point is inside contour
              sum_r += roi.at<cv::Vec3b>(i, j)[0];  // 蓝色通道
              sum_b += roi.at<cv::Vec3b>(i, j)[2];  // 红色通道 
          }
        }
        // Sum of red pixels > sum of blue pixels ?
        light.color = sum_r > sum_b ? RED : BLUE;
        lights.emplace_back(light);
    }
}
  return lights;
}

bool Detector::isLight(const Light& light) {
  // The ratio of light (short side / long side)
  float ratio = light.width / light.length;
  bool ratio_ok = light_params.min_ratio < ratio && ratio < light_params.max_ratio;
  bool angle_ok = light.tilt_angle < light_params.max_angle;
  bool is_light = ratio_ok && angle_ok;

  // Fill in debug information
  base_interfaces::msg::DebugLight light_data;
  light_data.center_x = light.center.x;
  light_data.ratio = ratio;
  light_data.angle = light.tilt_angle;
  light_data.is_light = is_light;
  this->debug_lights.data.emplace_back(light_data);

  return is_light;
}

std::vector<Armor> Detector::matchLights(const std::vector<Light>& lights) {
  std::vector<Armor> armors;
  this->debug_armors.data.clear();
  // Loop all the pairing of lights
  for (auto light_1 = lights.begin(); light_1 != lights.end(); light_1++) {
    for (auto light_2 = light_1 + 1; light_2 != lights.end(); light_2++) {
      if (light_1->color != detect_color || light_2->color != detect_color)
        continue;
      if (containLight(*light_1, *light_2, lights)) {
        continue;
      }
      auto type = isArmor(*light_1, *light_2);
      if (type != ArmorType::INVALID) {
        auto armor = Armor(*light_1, *light_2);
        armor.type = type;
        armors.emplace_back(armor);
      }
    }
  }

  return armors;
}

bool Detector::containLight(
  const Light & light_1, const Light & light_2, const std::vector<Light> & lights)
{
  auto points = std::vector<cv::Point2f>{light_1.top, light_1.bottom, light_2.top, light_2.bottom};
  auto bounding_rect = cv::boundingRect(points);

  for (const auto & test_light : lights) {
    if (test_light.center == light_1.center || test_light.center == light_2.center) continue;

    if (
      bounding_rect.contains(test_light.top) || bounding_rect.contains(test_light.bottom) ||
      bounding_rect.contains(test_light.center)) {
      return true;
    }
  }
  return false;
}

ArmorType Detector::isArmor(const Light & light_1, const Light & light_2)
{
  // Ratio of the length of 2 lights (short side / long side)
  float light_length_ratio = light_1.length < light_2.length ? light_1.length / light_2.length
                                                             : light_2.length / light_1.length;
  bool light_ratio_ok = light_length_ratio > armor_params.min_light_ratio;

  // Distance between the center of 2 lights (unit : light length)
  float avg_light_length = (light_1.length + light_2.length) / 2;
  float center_distance = cv::norm(light_1.center - light_2.center) / avg_light_length;
  bool center_distance_ok = (armor_params.min_small_center_distance <= center_distance &&
                             center_distance < armor_params.max_small_center_distance) ||
                            (armor_params.min_large_center_distance <= center_distance &&
                             center_distance < armor_params.max_large_center_distance);

  // Angle of light center connection
  cv::Point2f diff = light_1.center - light_2.center;
  float angle = std::abs(std::atan(diff.y / diff.x)) / CV_PI * 180;
  bool angle_ok = angle < armor_params.max_angle;

  bool is_armor = light_ratio_ok && center_distance_ok && angle_ok;

  // Judge armor type
  ArmorType type;
  if (is_armor) {
    type = center_distance > armor_params.min_large_center_distance ? ArmorType::LARGE : ArmorType::SMALL;
  } else {
    type = ArmorType::INVALID;
  }

  // Fill in debug information
  base_interfaces::msg::DebugArmor armor_data;
  armor_data.type = armorTypeToString(type);
  armor_data.center_x = (light_1.center.x + light_2.center.x) / 2;
  armor_data.light_ratio = light_length_ratio;
  armor_data.center_distance = center_distance;
  armor_data.angle = angle;
  this->debug_armors.data.emplace_back(armor_data);

  return type;
}

cv::Mat Detector::getAllNumbersImage() {
  if (armors_.empty()) {
    return cv::Mat(cv::Size(20, 28), CV_8UC1);
  } else {
    std::vector<cv::Mat> number_imgs;
    number_imgs.reserve(armors_.size());
    for (auto& armor : armors_) {
      number_imgs.emplace_back(armor.number_img);
    }
    cv::Mat all_num_img;
    cv::vconcat(number_imgs, all_num_img);
    return all_num_img;
  }
}


void Detector::drawResults(cv::Mat &img) const noexcept {
  for (const auto &armor : armors_) {
      // 绘制装甲板的左右两侧的垂直线
      cv::line(
          img, armor.left_light.top, armor.left_light.bottom, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
      cv::line(
          img, armor.right_light.bottom, armor.right_light.top, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);

      // 绘制对角线：左上角到右下角
      cv::line(
          img, armor.left_light.top, armor.right_light.bottom, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);

      // 绘制对角线：左下角到右上角
      cv::line(
          img, armor.left_light.bottom, armor.right_light.top, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
      cv::Point2f center = (armor.left_light.top + armor.right_light.bottom) / 2;
      cv::circle(img, 
                 center,      // 圆心坐标
                 20,   
                 cv::Scalar(255, 0, 0),
                 2,    
                 cv::LINE_AA); // 抗锯齿
         }
  // 显示数字和置信度
  for (const auto &armor : armors_) {
      cv::putText(img, armor.classfication_result, armor.left_light.top,
                  cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2);
  }
}

}  // namespace rm_auto_aim
