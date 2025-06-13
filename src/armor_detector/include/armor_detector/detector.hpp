// Copyright 2022 Chen Jun
// Licensed under the MIT License.

#ifndef ARMOR_DETECTOR__DETECTOR_HPP_
#define ARMOR_DETECTOR__DETECTOR_HPP_

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>

// STD
#include <cmath>
#include <string>
#include <vector>
#include <rm_utils/common.hpp>

#include "armor_detector/types.hpp"
#include "armor_detector/number_classifier.hpp"
#include "armor_detector/light_corner_corrector.hpp"
#include "base_interfaces/msg/debug_armors.hpp"
#include "base_interfaces/msg/debug_lights.hpp"
// 
namespace rm_auto_aim
{
class Detector
{
public:
  struct LightParams
  {
    // width / height
    double min_ratio;
    double max_ratio;
    // vertical angle
    double max_angle;
    double min_fill_ratio;
    int color_diff_thresh;
  };

  struct ArmorParams
  {
    double min_light_ratio;
    // light pairs distance
    double min_small_center_distance;
    double max_small_center_distance;
    double min_large_center_distance;
    double max_large_center_distance;
    double max_angle_diff ;      // 两灯条最大角度差
    double min_parallel_diff; 
    // horizontal angle
    double max_angle;
  // 平行系数阈值
  };

  
  Detector(const int & bin_thres, const int & color, const LightParams & l, const ArmorParams & a);

  std::vector<Armor> detect(const cv::Mat & input);

  cv::Mat preprocessImage(const cv::Mat & input);

  // cv::Mat applyHSVFilter(const cv::Mat & input);
  std::vector<Light> findLights(const cv::Mat &rbg_img,
    const cv::Mat &binary_img) noexcept;
  std::vector<Armor> matchLights(const std::vector<Light> & lights);

  // For debug usage
  cv::Mat getAllNumbersImage();
  void drawResults(cv::Mat &img) const noexcept;

  int binary_thres;
  int detect_color;
  LightParams light_params;
  ArmorParams armor_params;
  // HSVParams hsv_params;

  std::unique_ptr<NumberClassifier> classifier;
  std::unique_ptr<LightCornerCorrector> corner_corrector;

  // Debug msgs
  cv::Mat binary_img;
  // cv::Mat color_mask;
  base_interfaces::msg::DebugLights debug_lights;
  base_interfaces::msg::DebugArmors debug_armors;

private:
  bool isLight(const Light & possible_light);
  bool containLight(
    const Light & light_1, const Light & light_2, const std::vector<Light> & lights);
  ArmorType isArmor(const Light & light_1, const Light & light_2);

  cv::Mat gray_img_;
  std::vector<Light> lights_;
  std::vector<Armor> armors_;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__DETECTOR_HPP_
