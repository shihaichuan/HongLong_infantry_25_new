#ifndef ARMOR_DETECTOR_BA_SOLVER_HPP_
#define ARMOR_DETECTOR_BA_SOLVER_HPP_

// std
#include <array>
#include <cstddef>
#include <tuple>
#include <vector>
// 3rd party
#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <sophus/so3.hpp>
#include <std_msgs/msg/float32.hpp>
// g2o
#include <g2o/core/base_multi_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/optimization_algorithm.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/sparse_optimizer.h>
// project
#include "armor_detector/graph_optimizer.hpp"
#include "armor_detector/types.hpp"

namespace rm_auto_aim {

// BA algorithm based Optimizer for the armor pose estimation (Particularly for
// the Yaw angle)
class BaSolver {
public:
  BaSolver(std::array<double, 9> &camera_matrix,
           std::vector<double> &dist_coeffs);

  // Solve the armor pose using the BA algorithm, return the optimized rotation
  Eigen::Matrix3d solveBa(const Armor &armor,
                          const Eigen::Vector3d &t_camera_armor,
                          const Eigen::Matrix3d &R_camera_armor,
                          const Eigen::Matrix3d &R_imu_camera) noexcept;

private:
  Eigen::Matrix3d K_;
  g2o::SparseOptimizer optimizer_;
  g2o::OptimizationAlgorithmProperty solver_property_;
  g2o::OptimizationAlgorithmLevenberg *lm_algorithm_;
};

} // namespace fyt::auto_aim
#endif // ARMOR_DETECTOR_BAS_SOLVER_HPP_












// #ifndef ARMOR_DETECTOR_BA_SOLVER_HPP_
// #define ARMOR_DETECTOR_BA_SOLVER_HPP_

// // std
// #include <array>
// #include <cstddef>
// #include <tuple>
// #include <vector>
// // 3rd party
// #include <Eigen/Core>
// #include <Eigen/Dense>
// #include <opencv2/core.hpp>
// #include <sophus/so3.hpp>
// #include <std_msgs/msg/float32.hpp>
// // g2o
// #include <g2o/core/base_multi_edge.h>
// #include <g2o/core/base_vertex.h>
// #include <g2o/core/optimization_algorithm.h>
// #include <g2o/core/optimization_algorithm_factory.h>
// #include <g2o/core/optimization_algorithm_levenberg.h>
// #include <g2o/core/robust_kernel.h>
// #include <g2o/core/sparse_optimizer.h>
// #include "armor_detector/types.hpp"
// #include "armor_detector/graph_optimizer.hpp"

// namespace rm_auto_aim {

// class BaSolver {
// public:
//   BaSolver(std::array<double, 9> &camera_matrix, std::vector<double> &dist_coeffs);
//   ~BaSolver();
  
//   Eigen::Matrix3d solveBa(const Armor &armor, const Eigen::Vector3d &t_camera_armor,
//                          const Eigen::Matrix3d &R_camera_armor,
//                          const Eigen::Matrix3d &R_imu_camera) noexcept;

// private:
//   // Helper method to build object points efficiently
//   std::vector<Eigen::Vector3d> buildObjectPoints(double half_width, double half_height);

//   // Pre-computed camera matrix
//   Eigen::Matrix3d K_;
  
//   // Pre-compute object points for both armor types to avoid repeated calculations
//   std::vector<Eigen::Vector3d> small_object_points_;
//   std::vector<Eigen::Vector3d> large_object_points_;

//   // g2o optimizer and algorithm
//   g2o::SparseOptimizer optimizer;
//   g2o::OptimizationAlgorithmLevenberg *lm_algorithm_;
//   g2o::OptimizationAlgorithmProperty solver_property_;
  
//   // Pre-allocated vertices and edges to avoid memory allocation during optimization
//   VertexYaw* v_yaw_;
//   std::vector<g2o::VertexPointXYZ*> v_points_;
//   std::vector<EdgeProjection*> edges_;
// };

// } // namespace rm_auto_aim
