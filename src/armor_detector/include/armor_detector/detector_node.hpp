// Copyright 2022 Chen Jun
// Licensed under the MIT License.

#ifndef ARMOR_DETECTOR__DETECTOR_NODE_HPP_
#define ARMOR_DETECTOR__DETECTOR_NODE_HPP_

// ROS
#include <tf2_ros/buffer.h>
#include <tf2_ros/buffer_interface.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker_array.hpp>


// STD
#include <memory>
#include <string>
#include <vector>

#include "armor_detector/detector.hpp"
#include "armor_detector/number_classifier.hpp"
#include "armor_detector/armor_pose_estimator.hpp"
#include "base_interfaces/msg/armors.hpp"
#include "rm_utils/heartbeat.hpp"
#include "base_interfaces/msg/target.hpp"
#include "armor_detector/types.hpp"

// ROS
#include <image_transport/image_transport.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

// MindVision Camera SDK
#include <CameraApi.h>
#include <CameraDefine.h>  // 添加必要的类型定义头文件
#include <camera_info_manager/camera_info_manager.hpp>  // 添加相机信息管理器头文件

 
namespace rm_auto_aim
{
class ArmorDetectorNode : public rclcpp::Node
{
public:
  explicit ArmorDetectorNode(const rclcpp::NodeOptions & options);
  ~ArmorDetectorNode();

private:
  // Camera
  int h_camera_;
  tSdkCameraCapbility t_capability_;  // 添加 CameraDefine.h 解决了类型定义问题
  tSdkFrameHead s_frame_info_;
  unsigned char * pby_buffer_;  // 使用 unsigned char 替代 BYTE 类型
  int fail_count_;
  bool stop_processing_;
  std::thread processing_thread_;

  // Camera parameters
  int r_gain_, g_gain_, b_gain_;
  bool flip_image_;
  std::string camera_name_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  cv::Point2f cam_center_;

  // Detector
  std::unique_ptr<Detector> detector_;
  bool use_ba_;
  std::shared_ptr<sensor_msgs::msg::CameraInfo> cam_info_;
  std::unique_ptr<ArmorPoseEstimator> armor_pose_estimator_;

  // ROS
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
  rclcpp::Publisher<base_interfaces::msg::Armors>::SharedPtr armors_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  base_interfaces::msg::Armors armors_msg_;
  visualization_msgs::msg::Marker armor_marker_;
  visualization_msgs::msg::Marker text_marker_;
  visualization_msgs::msg::MarkerArray marker_array_;

  // Debug
  bool debug_;
  std::shared_ptr<rclcpp::ParameterEventHandler> debug_param_sub_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> debug_cb_handle_;
  image_transport::Publisher result_img_pub_;
  image_transport::Publisher binary_img_pub_;
  image_transport::Publisher number_img_pub_;
  rclcpp::Publisher<base_interfaces::msg::DebugLights>::SharedPtr lights_data_pub_;
  rclcpp::Publisher<base_interfaces::msg::DebugArmors>::SharedPtr armors_data_pub_;

  // ROI
  bool roi_enabled_;
  double roi_scale_x_;
  double roi_scale_y_;
  int max_lost_frames_;
  double min_roi_area_;
  double roi_min_size_;
  int lost_frames_;
  cv::Rect last_roi_;
  cv::Point2f roi_center_point_;

  // For FPS
  rclcpp::Time last_stamp_;
  double current_fps_;

  // TF
  std::string odom_frame;
  Eigen::Matrix3d imu_to_camera_;

  void declareCameraParameters();
  std::unique_ptr<Detector> initDetector();
  void imageProcessingLoop();
  std::vector<Armor> detectArmors(
    const sensor_msgs::msg::Image::ConstSharedPtr & img_msg, const cv::Mat & roi_img,
    const cv::Rect & roi_offset);
  void updateROI(const std::vector<Armor> & armors, const cv::Size & image_size);
  rcl_interfaces::msg::SetParametersResult onSetParameters(
    const std::vector<rclcpp::Parameter> & parameters);  // 添加 const 引用
  void createDebugPublishers();
  void destroyDebugPublishers();
  void publishVisualizationMarkers();

  // Parameter callback
  OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__DETECTOR_NODE_HPP_