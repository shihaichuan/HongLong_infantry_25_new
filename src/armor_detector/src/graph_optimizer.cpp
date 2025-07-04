#include "armor_detector/graph_optimizer.hpp"
// std
#include <algorithm>
// third party
#include <Eigen/Core>
#include <g2o/types/slam3d/vertex_pointxyz.h>
#include <sophus/so3.hpp>
// project
#include "armor_detector/types.hpp"
#include "rm_utils/math/utils.hpp"

namespace rm_auto_aim {

void VertexYaw::oplusImpl(const double *update) {
  Sophus::SO3d R_yaw = Sophus::SO3d::exp(Eigen::Vector3d(0, 0, update[0])) *
                       Sophus::SO3d::exp(Eigen::Vector3d(0, 0, _estimate));
  _estimate = R_yaw.log()(2);
}

EdgeProjection::EdgeProjection(const Sophus::SO3d &R_camera_imu,
                               const Sophus::SO3d &R_pitch,
                               const Eigen::Vector3d &t,
                               const Eigen::Matrix3d &K)
    : R_camera_imu_(R_camera_imu), R_pitch_(R_pitch), t_(t), K_(K) {}

void EdgeProjection::computeError() {
  // Get the rotation
  double yaw = static_cast<VertexYaw *>(_vertices[0])->estimate();
  Sophus::SO3d R_yaw = Sophus::SO3d::exp(Eigen::Vector3d(0, 0, yaw));
  Sophus::SO3d R = R_camera_imu_ * R_yaw * R_pitch_;

  // Get the 3D point
  Eigen::Vector3d p_3d =
      static_cast<g2o::VertexPointXYZ *>(_vertices[1])->estimate();

  // Get the observed 2D point
  Eigen::Vector2d obs(_measurement);

  // Project the 3D point to the 2D point
  Eigen::Vector3d p_2d = R * p_3d + t_;
  p_2d = K_ * (p_2d / p_2d.z());
  // Calculate the error
  _error = obs - p_2d.head<2>();
}
} // namespace rm_auto_aim
