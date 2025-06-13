/*
 * @Author: error: error: git config user.name & please set dead value or
 * install git && error: git config user.email & please set dead value or
 * install git & please set dead value or install git
 * @Date: 2023-10-15 15:22:36
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * install git && error: git config user.email & please set dead value or
 * install git & please set dead value or install git
 * @LastEditTime: 2023-11-20 13:20:14
 * @FilePath:
 * \auto_aim\rm_auto_aim\armor_detector\include\armor_detector\armor.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置
 * 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
// Copyright 2022 Chen Jun
// Licensed under the MIT License.

#ifndef ARMOR_DETECTOR_ARMOR_HPP_
#define ARMOR_DETECTOR_ARMOR_HPP_

#include <algorithm>
#include <numeric>
#include <string>
// 3rd party
#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include <sophus/so3.hpp>
// project
#include "rm_utils/assert.hpp"
#include "rm_utils/common.hpp"

namespace rm_auto_aim {
//////////
constexpr double SMALL_ARMOR_WIDTH = 133.0 / 1000.0; // 135
constexpr double SMALL_ARMOR_HEIGHT = 50.0 / 1000.0; // 55
constexpr double LARGE_ARMOR_WIDTH = 225.0 / 1000.0;
constexpr double LARGE_ARMOR_HEIGHT = 50.0 / 1000.0; // 55

// 15 degree in rad
constexpr double FIFTTEN_DEGREE_RAD = 15 * CV_PI / 180;
 
 

enum class ArmorType { SMALL, LARGE, INVALID };
inline std::string armorTypeToString(const ArmorType &type) {
  switch (type) {
    case ArmorType::SMALL:
      return "small";
    case ArmorType::LARGE:
      return "large";
    default:
      return "invalid";
  }
}
const int RED=1;
const int BLUE=0;
const int White=2;

struct Light : public cv::RotatedRect {
  Light() = default;
  explicit Light(const std::vector<cv::Point> &contour)
  : cv::RotatedRect(cv::minAreaRect(contour)), color(2) {
    // FYT_ASSERT(contour.size() > 0);
    if (contour.size() < 0) {
      return;
    }
    center = std::accumulate(
      contour.begin(),
      contour.end(),
      cv::Point2f(0, 0),
      [n = static_cast<float>(contour.size())](const cv::Point2f &a, const cv::Point &b) {
        return a + cv::Point2f(b.x, b.y) / n;
      });

    cv::Point2f p[4];
    this->points(p);
    std::sort(p, p + 4, [](const cv::Point2f &a, const cv::Point2f &b) { return a.y < b.y; });
    top = (p[0] + p[1]) / 2;
    bottom = (p[2] + p[3]) / 2;
    // center=top;
    

    length = cv::norm(top - bottom);
    width = cv::norm(p[0] - p[1]);

    axis = top - bottom;
    axis = axis / cv::norm(axis);

    // Calculate the tilt angle
    // The angle is the angle between the light bar and the horizontal line
    tilt_angle = std::atan2(std::abs(top.x - bottom.x), std::abs(top.y - bottom.y));
    tilt_angle = tilt_angle / CV_PI * 180;
  }

  int color;
  cv::Point2f top, bottom,center;
  cv::Point2f axis;
  double length;
  double width;
  float tilt_angle;

};

struct Armor {
  Armor() = default;
   // 定义常量N_LANDMARKS，表示特征点的数量
  static constexpr const int N_LANDMARKS = 6;
  // 定义常量N_LANDMARKS_2，表示特征点的数量乘以2
  static constexpr const int N_LANDMARKS_2 = N_LANDMARKS * 2;
  std::array<cv::Point2f, 4> vertices;
  Armor(const Light& l1, const Light& l2) {
    
    if (l1.center.x < l2.center.x) {
      left_light = l1, right_light = l2;
    } else {
      left_light = l2, right_light = l1;
    }
    center = (left_light.center + right_light.center) / 2;
    // std::cout<<"left_light.center.y: "<<left_light.center.y<<std::endl;
    // std::cout<<"left_light.bottom.: "<<left_light.bottom.y<<std::endl;
  }
/////////////////////////////////////////
template <typename PointType>
  static inline std::vector<PointType> buildObjectPoints(const double &w,
                                                         const double &h) noexcept {
    if constexpr (N_LANDMARKS == 4) {
      return {PointType(0, w / 2, -h / 2),
              PointType(0, w / 2, h / 2),
              PointType(0, -w / 2, h / 2),
              PointType(0, -w / 2, -h / 2)};
    } else {
      return {PointType(0, w / 2, -h / 2),
              PointType(0, w / 2, 0),
              PointType(0, w / 2, h / 2),
              PointType(0, -w / 2, h / 2),
              PointType(0, -w / 2, 0),
              PointType(0, -w / 2, -h / 2)};
    }
  }
//////////////////////////////////////////////////////
// std::vector<cv::Point2f> landmarks() const {
//     if constexpr (N_LANDMARKS == 4) {
//       return {left_light.bottom, left_light.top, right_light.top, right_light.bottom};
//     } else {
//       return {left_light.bottom,
//               left_light.center,
//               left_light.top,
//               right_light.top,
//               right_light.center,
//               right_light.bottom};
//     }
//   }
  std::vector<cv::Point2f> landmarks() const {
    if constexpr (N_LANDMARKS == 4) {
      return {cv::Point2f(left_light.bottom.x, left_light.bottom.y),
              cv::Point2f(left_light.top.x, left_light.top.y),
              cv::Point2f(right_light.top.x, right_light.top.y),
              cv::Point2f(right_light.bottom.x, right_light.bottom.y)};
    } else {
      
      return {cv::Point2f(left_light.bottom.x, left_light.bottom.y),
              cv::Point2f(left_light.center.x, left_light.center.y),
              cv::Point2f(left_light.top.x, left_light.top.y),
              cv::Point2f(right_light.top.x, right_light.top.y),
              cv::Point2f(right_light.center.x, right_light.center.y),
              cv::Point2f(right_light.bottom.x, right_light.bottom.y)};
    }
  }
  //////////////////////////////////
  // Light pairs part
  Light left_light, right_light;
  cv::Point2f center;
  ArmorType type;

  // Number part
  cv::Mat number_img;
  std::string number;
  float confidence;
  std::string classfication_result;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__ARMOR_HPP_
