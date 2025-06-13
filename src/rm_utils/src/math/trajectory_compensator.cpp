// Created by Chengfu Zou
// Copyright (C) FYT Vision Group. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rm_utils/math/trajectory_compensator.hpp"

namespace rm_auto_aim {
bool TrajectoryCompensator::compensate(const Eigen::Vector3d &target_position,
                                       double &pitch) const noexcept {
  double target_height = target_position(2);//目标高度
  // The iterative_height is used to calculate angle in each iteration
  double iterative_height = target_height;//迭代高度 初始为目标高度
  double impact_height = 0;//轨迹高度
  double distance =
    std::sqrt(target_position(0) * target_position(0) + target_position(1) * target_position(1));//距离
  double angle = std::atan2(target_height, distance);//初始角度
  double dh = 0;//高度差
  // Iterate to find the right angle, which makes the impact height equal to the
  // target height
  for (int i = 0; i < iteration_times; ++i) {
    angle = std::atan2(iterative_height, distance);
    if (std::abs(angle) > M_PI / 2.5) {
      break;
    }
    impact_height = calculateTrajectory(distance, angle);
    dh = target_height - impact_height;
    if (std::abs(dh) < 0.01) {
      break;
    }
    iterative_height += dh;//////////////////迭代计算高度
  }
  if (std::abs(dh) > 0.01 || std::abs(angle) > M_PI / 2.5) {
    return false;
  }
  pitch = angle;///////////////将计算出的角度赋值给pitch
  return true;
}

std::vector<std::pair<double, double>> TrajectoryCompensator::getTrajectory(
  double distance, double angle) const noexcept {
  std::vector<std::pair<double, double>> trajectory;

  if (distance < 0) {
    return trajectory;
  }

  for (double x = 0; x < distance; x += 0.03) {
    trajectory.emplace_back(x, calculateTrajectory(x, angle));
  }
  return trajectory;
}//计算轨迹

double IdealCompensator::calculateTrajectory(const double x, const double angle) const noexcept {
  double t = x / (velocity * cos(angle));
  double y = velocity * sin(angle) * t - 0.5 * gravity * t * t;
  return y;
}//计算给定初始位置和发射角度下的理想补偿器的轨迹高度

double IdealCompensator::getFlyingTime(const Eigen::Vector3d &target_position) const noexcept {
  double distance =
    sqrt(target_position(0) * target_position(0) + target_position(1) * target_position(1));
  double angle = atan2(target_position(2), distance);
  double t = distance / (velocity * cos(angle));
  return t;
}

double ResistanceCompensator::calculateTrajectory(const double x,
                                                  const double angle) const noexcept {
  double r = resistance < 1e-4 ? 1e-4 : resistance;
  double t = (exp(r * x) - 1) / (r * velocity * cos(angle));
  double y = velocity * sin(angle) * t - 0.5 * gravity * t * t;
  return y;
}

double ResistanceCompensator::getFlyingTime(const Eigen::Vector3d &target_position) const noexcept {
  double r = resistance < 1e-4 ? 1e-4 : resistance;
  double distance =
    sqrt(target_position(0) * target_position(0) + target_position(1) * target_position(1));
  double angle = atan2(target_position(2), distance);
  double t = (exp(r * distance) - 1) / (r * velocity * cos(angle));
  return t;
}
}  // namespace fyt
