// Copyright 2022 Chen Jun
// Licensed under the MIT License.

#include <cv_bridge/cv_bridge.h>
#include <rmw/qos_profiles.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/qos.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// STD
#include <algorithm>
#include <cstddef>
#include <filesystem>
#include <functional>
#include <map>
#include <memory>
#include <numeric>
#include <string>
#include <vector>

#include "armor_detector/types.hpp"
#include "armor_detector/detector_node.hpp"
#include "armor_detector/detector.hpp"
#include "armor_detector/ba_solver.hpp"
#include "rm_utils/assert.hpp"
#include "rm_utils/common.hpp"
#include "rm_utils/math/pnp_solver.hpp"
#include "rm_utils/math/utils.hpp"
#include "rm_utils/url_resolver.hpp"
#include <CameraApi.h>
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

// C++ system
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace rm_auto_aim {
ArmorDetectorNode::ArmorDetectorNode(const rclcpp::NodeOptions& options)
    : Node("armor_detector", options), fail_count_(0) {
  RCLCPP_INFO(this->get_logger(), "Starting DetectorNode with Camera Integration!");
std::cout<<"111111"<<std::endl;

  // Initialize camera
  CameraSdkInit(1);
  std::cout<<"111111"<<std::endl;
  // Enumerate devices
  int i_camera_counts = 1;
  tSdkCameraDevInfo t_camera_enum_list;
  int i_status = CameraEnumerateDevice(&t_camera_enum_list, &i_camera_counts);
  RCLCPP_INFO(this->get_logger(), "Enumerate state = %d", i_status);
  RCLCPP_INFO(this->get_logger(), "Found camera count = %d", i_camera_counts);

  if (i_camera_counts == 0) {
    RCLCPP_ERROR(this->get_logger(), "No camera found!");
    throw std::runtime_error("No camera found");
  }

  // Initialize camera
  i_status = CameraInit(&t_camera_enum_list, -1, -1, &h_camera_);

  if (i_status != CAMERA_STATUS_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Camera init failed: %d", i_status);
    throw std::runtime_error("Camera init failed");
  }

  // Get camera capability
  CameraGetCapability(h_camera_, &t_capability_);

  // Set camera parameters
  CameraSetAeState(h_camera_, false); // Manual exposure
  CameraSetIspOutFormat(h_camera_, CAMERA_MEDIA_TYPE_RGB8);

  // Declare camera parameters
  declareCameraParameters();
std::cout<<"1111"<<std::endl;

  // Start camera
  CameraPlay(h_camera_);
  
  // Detector initialization
  detector_ = initDetector();
  use_ba_ = this->declare_parameter("use_ba", true);
  roi_enabled_ = this->declare_parameter("roi.enabled", true);
  roi_scale_x_ = this->declare_parameter("roi.scale_x", 2.0);
  roi_scale_y_ = this->declare_parameter("roi.scale_y", 1.5);
  max_lost_frames_ = this->declare_parameter("roi.max_lost_frames", 5);
  min_roi_area_ = this->declare_parameter("roi.min_area", 10000.0);
  roi_min_size_ = this->declare_parameter("roi.min_size", 100.0);

  // Armors Publisher
  armors_pub_ = this->create_publisher<base_interfaces::msg::Armors>(
      "/detector/armors", rclcpp::SensorDataQoS());
  odom_frame = this->declare_parameter("target_frame", "odom");
  imu_to_camera_ = Eigen::Matrix3d::Identity();
std::cout<<"1111"<<std::endl;

  // Visualization Marker Publisher
  armor_marker_.ns = "armors";
  armor_marker_.action = visualization_msgs::msg::Marker::ADD;
  armor_marker_.type = visualization_msgs::msg::Marker::CUBE;
  armor_marker_.scale.x = 0.05;
  armor_marker_.scale.z = 0.125;
  armor_marker_.color.a = 1.0;
  armor_marker_.color.g = 0.5;
  armor_marker_.color.b = 1.0;
  armor_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);

  text_marker_.ns = "classification";
  text_marker_.action = visualization_msgs::msg::Marker::ADD;
  text_marker_.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  text_marker_.scale.z = 0.1;
  text_marker_.color.a = 1.0;
  text_marker_.color.r = 1.0;
  text_marker_.color.g = 1.0;
  text_marker_.color.b = 1.0;
  text_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);

  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/detector/marker", 10);

  // Debug Publishers

std::cout<<"1111"<<std::endl;
  // Debug param change monitor
  debug_ = this->declare_parameter("debug", true);
  //  debug_param_sub_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
  // debug_cb_handle_ = debug_param_sub_->add_parameter_callback(
  //     "debug", [this](const rclcpp::Parameter& p) {
  //       debug_ = p.as_bool();
  //       debug_ ? createDebugPublishers() : destroyDebugPublishers();
  //     });

  // Camera info
  camera_name_ = this->declare_parameter("camera_name", "mv_camera");
  camera_info_manager_ =
      std::make_unique<camera_info_manager::CameraInfoManager>(this, camera_name_);
  auto camera_info_url = this->declare_parameter(
      "camera_info_url",
      "package://mindvision_camera/config/camera_info.yaml");
  if (camera_info_manager_->validateURL(camera_info_url)) {
    camera_info_manager_->loadCameraInfo(camera_info_url);
    cam_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(camera_info_manager_->getCameraInfo());
    cam_center_ = cv::Point2f(cam_info_->k[2], cam_info_->k[5]);
    armor_pose_estimator_ = std::make_unique<ArmorPoseEstimator>(cam_info_); 
    armor_pose_estimator_->enableBA(use_ba_);
  } else {
    RCLCPP_WARN(this->get_logger(), "Invalid camera info URL: %s", camera_info_url.c_str());
  }

  // TF setup
  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(), this->get_node_timers_interface());
  tf2_buffer_->setCreateTimerInterface(timer_interface);
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

  // Start image capture and processing thread
  processing_thread_ = std::thread(&ArmorDetectorNode::imageProcessingLoop, this);
    // debug_ = this->declare_parameter("debug", true);
  if (debug_) {
    createDebugPublishers();
  }
}

ArmorDetectorNode::~ArmorDetectorNode() {
  // Signal thread to stop
  stop_processing_ = true;
  
  // Join thread
  if (processing_thread_.joinable()) {
    processing_thread_.join();
  }
  
  // Release camera resources
  CameraUnInit(h_camera_);
  
  RCLCPP_INFO(this->get_logger(), "Detector node shutdown");
}

void ArmorDetectorNode::imageProcessingLoop() {
  cv::Mat img;
  
  while (rclcpp::ok() && !stop_processing_) {
    // Get image from camera
    int status = CameraGetImageBuffer(h_camera_, &s_frame_info_, &pby_buffer_, 100);
    
    if (status != CAMERA_STATUS_SUCCESS) {
      if (status != CAMERA_STATUS_TIME_OUT) {
        RCLCPP_WARN(this->get_logger(), "Failed to get image: %d", status);
        if (++fail_count_ > 15) {
          RCLCPP_FATAL(this->get_logger(), "Continuous camera failures, shutting down");
          rclcpp::shutdown();
          return;
        }
      }
      continue;
    }
    
    // Process successful frame
    fail_count_ = 0;
    
    // Convert to OpenCV image
    img.create(s_frame_info_.iHeight, s_frame_info_.iWidth, CV_8UC3);
    CameraImageProcess(h_camera_, pby_buffer_, img.data, &s_frame_info_);
    
    // Flip if needed
    if (flip_image_) {
      cv::flip(img, img, -1); // Flip both horizontally and vertically
    }
    
    // Release camera buffer
    CameraReleaseImageBuffer(h_camera_, pby_buffer_);
    // Create ROS image message
    auto img_msg = std::make_shared<sensor_msgs::msg::Image>();
    img_msg->header.stamp = this->now();
    img_msg->header.frame_id = "camera_optical_frame";
    img_msg->height = img.rows;
    img_msg->width = img.cols;
    img_msg->encoding = "rgb8";
    img_msg->is_bigendian = false;
    img_msg->step = img.cols * 3;
    img_msg->data.assign(img.datastart, img.dataend);
    
    // Apply ROI
    cv::Rect roi;
    cv::Mat roi_img;
    if (roi_enabled_ && !last_roi_.empty()) {
      roi = last_roi_ & cv::Rect(0, 0, img.cols, img.rows);
      if (roi.area() > min_roi_area_) {
        roi_img = img(roi);
      } else {
        roi_enabled_ = false; // ROI too small, use full image
      }
    }
    
    // If no valid ROI, use full image
    if (roi_img.empty()) {
      roi_img = img;
      roi = cv::Rect(0, 0, img.cols, img.rows);
    }

    // Detect armors
    auto armors = detectArmors(img_msg, roi_img, roi);
    
    // Publish armors message
    armors_msg_.header = img_msg->header;
    armors_msg_.armors.clear();
    if (armor_pose_estimator_) {
      armors_msg_.armors = armor_pose_estimator_->extractArmorPoses(armors, imu_to_camera_);
    }
    
    // Update ROI
    if (!armors.empty()) {
      updateROI(armors, img.size());
      lost_frames_ = 0; // Reset lost frames count
    } else if (roi_enabled_) {
      lost_frames_++;
      if (lost_frames_ > max_lost_frames_) {
        last_roi_ = cv::Rect(); // Reset ROI
        lost_frames_ = 0;
      }
    }
    
    // Publish visualization markers
    if (debug_) {
      publishVisualizationMarkers();
    }
    
    // Publish detected armors
    armors_pub_->publish(armors_msg_);
    
    // Update FPS calculation
    auto current_rclcpp_time = this->now();
    
    // 计算FPS
    if (last_stamp_.nanoseconds() > 0) {
      auto dt = (current_rclcpp_time - last_stamp_).seconds();
      current_fps_ = 1.0 / dt;
    }
    last_stamp_ = current_rclcpp_time;  // 更新上次时间
    
    // 设置图像消息的时间戳
    img_msg->header.stamp = current_rclcpp_time;
  }
}

void ArmorDetectorNode::declareCameraParameters() {
  rcl_interfaces::msg::ParameterDescriptor param_desc;
  param_desc.integer_range.resize(1);
  param_desc.integer_range[0].step = 1;

  // Exposure time
  param_desc.description = "Exposure time in microseconds";
  double exposure_line_time;
  CameraGetExposureLineTime(h_camera_, &exposure_line_time);
  param_desc.integer_range[0].from_value =
      t_capability_.sExposeDesc.uiExposeTimeMin * exposure_line_time;
  param_desc.integer_range[0].to_value =
      t_capability_.sExposeDesc.uiExposeTimeMax * exposure_line_time;
  double exposure_time =
      this->declare_parameter("exposure_time", 1000, param_desc);
  CameraSetExposureTime(h_camera_, exposure_time);
  RCLCPP_INFO(this->get_logger(), "Exposure time = %f", exposure_time);

  // Analog gain
  param_desc.description = "Analog gain";
  param_desc.integer_range[0].from_value =
      t_capability_.sExposeDesc.uiAnalogGainMin;
  param_desc.integer_range[0].to_value =
      t_capability_.sExposeDesc.uiAnalogGainMax;
  int analog_gain;
  CameraGetAnalogGain(h_camera_, &analog_gain);
  analog_gain =
      this->declare_parameter("analog_gain", analog_gain, param_desc);
  CameraSetAnalogGain(h_camera_, analog_gain);
  RCLCPP_INFO(this->get_logger(), "Analog gain = %d", analog_gain);

  // RGB Gain
  CameraGetGain(h_camera_, &r_gain_, &g_gain_, &b_gain_);
  
  // R Gain
  param_desc.integer_range[0].from_value =
      t_capability_.sRgbGainRange.iRGainMin;
  param_desc.integer_range[0].to_value =
      t_capability_.sRgbGainRange.iRGainMax;
  r_gain_ = this->declare_parameter("rgb_gain.r", r_gain_, param_desc);
  
  // G Gain
  param_desc.integer_range[0].from_value =
      t_capability_.sRgbGainRange.iGGainMin;
  param_desc.integer_range[0].to_value =
      t_capability_.sRgbGainRange.iGGainMax;
  g_gain_ = this->declare_parameter("rgb_gain.g", g_gain_, param_desc);
  
  // B Gain
  param_desc.integer_range[0].from_value =
      t_capability_.sRgbGainRange.iBGainMin;
  param_desc.integer_range[0].to_value =
      t_capability_.sRgbGainRange.iBGainMax;
  b_gain_ = this->declare_parameter("rgb_gain.b", b_gain_, param_desc);
  
  // Set gain
  CameraSetGain(h_camera_, r_gain_, g_gain_, b_gain_);
  RCLCPP_INFO(this->get_logger(), "RGB Gain: R=%d, G=%d, B=%d", r_gain_, g_gain_, b_gain_);

  // Saturation
  param_desc.description = "Saturation";
  param_desc.integer_range[0].from_value =
      t_capability_.sSaturationRange.iMin;
  param_desc.integer_range[0].to_value = t_capability_.sSaturationRange.iMax;
  int saturation;
  CameraGetSaturation(h_camera_, &saturation);
  saturation = this->declare_parameter("saturation", saturation, param_desc);
  CameraSetSaturation(h_camera_, saturation);
  RCLCPP_INFO(this->get_logger(), "Saturation = %d", saturation);

  // Gamma
  param_desc.integer_range[0].from_value = t_capability_.sGammaRange.iMin;
  param_desc.integer_range[0].to_value = t_capability_.sGammaRange.iMax;
  int gamma;
  CameraGetGamma(h_camera_, &gamma);
  gamma = this->declare_parameter("gamma", gamma, param_desc);
  CameraSetGamma(h_camera_, gamma);
  RCLCPP_INFO(this->get_logger(), "Gamma = %d", gamma);

  // Flip
  flip_image_ = this->declare_parameter("flip_image", false);
}

std::unique_ptr<Detector> ArmorDetectorNode::initDetector() {
  rcl_interfaces::msg::ParameterDescriptor param_desc;
  param_desc.integer_range.resize(1);
  param_desc.integer_range[0].step = 1;
  param_desc.integer_range[0].from_value = 0;
  param_desc.integer_range[0].to_value = 255;
  int binary_thres = declare_parameter("binary_thres", 160, param_desc);

  param_desc.description = "1-RED, 0-BLUE";
  param_desc.integer_range[0].from_value = 0;
  param_desc.integer_range[0].to_value = 1;
  auto detect_color = declare_parameter("detect_color", RED, param_desc);
  
  Detector::LightParams l_params = {
      .min_ratio = declare_parameter("light.min_ratio", 0.1),
      .max_ratio = declare_parameter("light.max_ratio", 0.4),
      .max_angle = declare_parameter("light.max_angle", 40.0)};

  Detector::ArmorParams a_params = {
      .min_light_ratio = declare_parameter("armor.min_light_ratio", 0.7),
      .min_small_center_distance =
          declare_parameter("armor.min_small_center_distance", 0.8),
      .max_small_center_distance =
          declare_parameter("armor.max_small_center_distance", 3.2),
      .min_large_center_distance =
          declare_parameter("armor.min_large_center_distance", 3.2),
      .max_large_center_distance =
          declare_parameter("armor.mx_large_center_distance", 5.5),
      .max_angle_diff = 
          declare_parameter("armor.max_angle_diff", 1.5),
      .min_parallel_diff =
          declare_parameter("armor.min_parallel_diff", 0.25),
      .max_angle = declare_parameter("armor.max_angle", 35.0)};

  auto detector = std::make_unique<Detector>(binary_thres, detect_color,
                                             l_params, a_params);
std::cout<<"11111111111"<<std::endl;
  // Init classifier
  auto pkg_path =
      ament_index_cpp::get_package_share_directory("armor_detector");
  auto model_path = pkg_path + "/model/mlp.onnx";
  auto label_path = pkg_path + "/model/label.txt";
  double threshold = this->declare_parameter("classifier_threshold", 0.7);
  std::vector<std::string> ignore_classes = this->declare_parameter(
      "ignore_classes", std::vector<std::string>{"negative"});
  detector->classifier = std::make_unique<NumberClassifier>(
      model_path, label_path, threshold, ignore_classes);

  // Corner detection
  bool use_pca = this->declare_parameter("use_pca", true);
  if (use_pca) {
    detector->corner_corrector = std::make_unique<LightCornerCorrector>();
  }
  
  on_set_parameters_callback_handle_ =
      this->add_on_set_parameters_callback(std::bind(
          &ArmorDetectorNode::onSetParameters, this, std::placeholders::_1));

  return detector;
}

std::vector<Armor> ArmorDetectorNode::detectArmors(
    const sensor_msgs::msg::Image::ConstSharedPtr& img_msg, 
    const cv::Mat& roi_img,
    const cv::Rect& roi_offset) 
{
  auto armors = detector_->detect(roi_img);
  
  // Adjust coordinates for ROI
  for (auto& armor : armors) {
    for (auto& pt : armor.vertices) {
      pt.x += roi_offset.x;
      pt.y += roi_offset.y;
    }
    armor.center.x += roi_offset.x;
    armor.center.y += roi_offset.y;
  }

  // Publish debug info
  if (debug_) {
    binary_img_pub_.publish(
        cv_bridge::CvImage(img_msg->header, "mono8", detector_->binary_img)
            .toImageMsg());

    // Sort lights and armors data
    std::sort(detector_->debug_lights.data.begin(),
              detector_->debug_lights.data.end(),
              [](const auto& l1, const auto& l2) {
                return l1.center_x < l2.center_x;
              });
    std::sort(detector_->debug_armors.data.begin(),
              detector_->debug_armors.data.end(),
              [](const auto& a1, const auto& a2) {
                return a1.center_x < a2.center_x;
              });

    lights_data_pub_->publish(detector_->debug_lights);
    armors_data_pub_->publish(detector_->debug_armors);

    if (!armors.empty()) {
      auto all_num_img = detector_->getAllNumbersImage();
      number_img_pub_.publish(
          *cv_bridge::CvImage(img_msg->header, "mono8", all_num_img)
               .toImageMsg());
    }

    // Create debug visualization image
    cv::Mat debug_img;
    cv::cvtColor(roi_img, debug_img, cv::COLOR_RGB2BGR);
    
    // Draw ROI boundary
    if (!last_roi_.empty()) {
      cv::rectangle(debug_img, last_roi_, cv::Scalar(0, 255, 255), 2);
    }
    
    // Draw detection results
    detector_->drawResults(debug_img);
    std::cout<<"666666666666"<<std::endl;

    // Draw camera center
    cv::circle(debug_img, cam_center_, 5, cv::Scalar(0, 0, 255), 2);
    
    // Draw latency and FPS
    auto latency = (this->now() - img_msg->header.stamp).seconds() * 1000;
    std::stringstream info_ss;
    info_ss << "Latency: " << std::fixed << std::setprecision(2) << latency << "ms";
    std::cout<<"Latency" <<latency<<std::endl;
    cv::putText(debug_img, info_ss.str(), cv::Point(10, 30), 
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 1);
    
    info_ss.str("");
    info_ss << "FPS: " << std::fixed << std::setprecision(1) << current_fps_;
    cv::putText(debug_img, info_ss.str(), cv::Point(10, 60), 
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 1);

    // Publish debug image
    result_img_pub_.publish(
        cv_bridge::CvImage(img_msg->header, "bgr8", debug_img).toImageMsg());
  }

  return armors;
}

void ArmorDetectorNode::updateROI(const std::vector<Armor>& armors, const cv::Size& image_size) {
  if (armors.empty()) return;

  // 收集所有装甲板的中心点
  std::vector<cv::Point2f> centers;
  for (const auto& armor : armors) {
    centers.push_back(armor.center);
  }

  // 计算所有装甲板的平均中心点
  cv::Point2f roi_center(0, 0);
  for (const auto& center : centers) {
    roi_center += center;
  }
  if (!centers.empty()) {
    roi_center.x /= centers.size();
    roi_center.y /= centers.size();
  }

  // 设置固定的ROI尺寸
  int roi_width = 4000; // ROI宽度像素
  int roi_height = 3000; // ROI高度像素
  if (!armors.empty()) {
    double max_width = 0;
    for (const auto& armor : armors) {
        // 计算装甲板宽度
        double armor_width = cv::norm(armor.vertices[1] - armor.vertices[0]);
        if (armor_width > max_width) max_width = armor_width;
    }
    
    // 根据装甲板大小调整ROI（例如5倍装甲板大小）
    int scale_factor = 5;
    roi_width = static_cast<int>(max_width * scale_factor);
    roi_height = static_cast<int>(roi_width * 1.33);  // 保持4:3比例
    
    // 设置最小和最大尺寸
    roi_width = std::clamp(roi_width, 100, image_size.width);
    roi_height = std::clamp(roi_height, 100, image_size.height);
}
  // 创建以中心点为中心的ROI
  cv::Rect new_roi(
      static_cast<int>(roi_center.x - roi_width / 2),
      static_cast<int>(roi_center.y - roi_height / 2),
      roi_width,
      roi_height
  );
  
  // 确保在图像范围内
  new_roi &= cv::Rect(0, 0, image_size.width, image_size.height);
  
  // 更新ROI
  last_roi_ = new_roi;
  
  // 存储中心点用于可视化
  roi_center_point_ = roi_center;
}

rcl_interfaces::msg::SetParametersResult
ArmorDetectorNode::onSetParameters(const std::vector<rclcpp::Parameter> &parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  
  for (const auto &param : parameters) {
    // Camera parameters
    if (param.get_name() == "exposure_time") {
      int status = CameraSetExposureTime(h_camera_, param.as_int());
      if (status != CAMERA_STATUS_SUCCESS) {
        result.successful = false;
        result.reason = "Failed to set exposure time: " + std::to_string(status);
      }
    } 
    else if (param.get_name() == "analog_gain") {
      int status = CameraSetAnalogGain(h_camera_, param.as_int());
      if (status != CAMERA_STATUS_SUCCESS) {
        result.successful = false;
        result.reason = "Failed to set analog gain: " + std::to_string(status);
      }
    } 
    else if (param.get_name() == "rgb_gain.r") {
      r_gain_ = param.as_int();
      int status = CameraSetGain(h_camera_, r_gain_, g_gain_, b_gain_);
      if (status != CAMERA_STATUS_SUCCESS) {
        result.successful = false;
        result.reason = "Failed to set RGB gain: " + std::to_string(status);
      }
    } 
    else if (param.get_name() == "rgb_gain.g") {
      g_gain_ = param.as_int();
      int status = CameraSetGain(h_camera_, r_gain_, g_gain_, b_gain_);
      if (status != CAMERA_STATUS_SUCCESS) {
        result.successful = false;
        result.reason = "Failed to set RGB gain: " + std::to_string(status);
      }
    } 
    else if (param.get_name() == "rgb_gain.b") {
      b_gain_ = param.as_int();
      int status = CameraSetGain(h_camera_, r_gain_, g_gain_, b_gain_);
      if (status != CAMERA_STATUS_SUCCESS) {
        result.successful = false;
        result.reason = "Failed to set RGB gain: " + std::to_string(status);
      }
    } 
    else if (param.get_name() == "saturation") {
      int status = CameraSetSaturation(h_camera_, param.as_int());
      if (status != CAMERA_STATUS_SUCCESS) {
        result.successful = false;
        result.reason = "Failed to set saturation: " + std::to_string(status);
      }
    } 
    else if (param.get_name() == "gamma") {
      int status = CameraSetGamma(h_camera_, param.as_int());
      if (status != CAMERA_STATUS_SUCCESS) {
        result.successful = false;
        result.reason = "Failed to set gamma: " + std::to_string(status);
      }
    } 
    else if (param.get_name() == "flip_image") {
      flip_image_ = param.as_bool();
    }
    
    // Detector parameters
    else if (param.get_name() == "binary_thres") {
      detector_->binary_thres = param.as_int();
    } 
    else if (param.get_name() == "roi.enabled") {
      roi_enabled_ = param.as_bool();
      if (!roi_enabled_) last_roi_ = cv::Rect(); 
    }
    else if (param.get_name() == "roi.scale_x") {
      roi_scale_x_ = param.as_double();
    }
    else if (param.get_name() == "roi.scale_y") {
      roi_scale_y_ = param.as_double();
    }
    else if (param.get_name() == "roi.max_lost_frames") {
      max_lost_frames_ = param.as_int();
    }
    else if (param.get_name() == "roi.min_area") {
      min_roi_area_ = param.as_double();
    } 
    else if (param.get_name() == "classifier_threshold") {
      detector_->classifier->threshold = param.as_double();
    } 
    else if (param.get_name() == "light.min_ratio") {
      detector_->light_params.min_ratio = param.as_double();
    } 
    else if (param.get_name() == "light.max_ratio") {
      detector_->light_params.max_ratio = param.as_double();
    } 
    else if (param.get_name() == "light.max_angle") {
      detector_->light_params.max_angle = param.as_double();
    } 
    else if (param.get_name() == "armor.min_light_ratio") {
      detector_->armor_params.min_light_ratio = param.as_double();
    } 
    else if (param.get_name() == "armor.max_angle_diff") {
        detector_->armor_params.max_angle_diff = param.as_double();
    } 
    else if (param.get_name() == "armor.min_parallel_diff") {
        detector_->armor_params.min_parallel_diff = param.as_double();
    } 
    else if (param.get_name() == "armor.min_small_center_distance") {
      detector_->armor_params.min_small_center_distance = param.as_double();
    } 
    else if (param.get_name() == "armor.max_small_center_distance") {
      detector_->armor_params.max_small_center_distance = param.as_double();
    } 
    else if (param.get_name() == "armor.min_large_center_distance") {
      detector_->armor_params.min_large_center_distance = param.as_double();
    } 
    else if (param.get_name() == "armor.max_large_center_distance") {
      detector_->armor_params.max_large_center_distance = param.as_double();
    } 
    else if (param.get_name() == "armor.max_angle") {
      detector_->armor_params.max_angle = param.as_double();
    }
  }
  
  return result;
}

void ArmorDetectorNode::publishVisualizationMarkers() {
  marker_array_.markers.clear();
  armor_marker_.id = 0;
  text_marker_.id = 0;
  armor_marker_.header = text_marker_.header = armors_msg_.header;
  
  // Fill the markers
  for (const auto &armor : armors_msg_.armors) {
    armor_marker_.pose = armor.pose;
    armor_marker_.id++;
    text_marker_.pose.position = armor.pose.position;
    text_marker_.id++;
    text_marker_.pose.position.y -= 0.1;
    text_marker_.text = armor.number;
    marker_array_.markers.emplace_back(armor_marker_);
    marker_array_.markers.emplace_back(text_marker_);
  }
  
  marker_pub_->publish(marker_array_);
}

void ArmorDetectorNode::createDebugPublishers() {
  lights_data_pub_ =
      this->create_publisher<base_interfaces::msg::DebugLights>(
          "/detector/debug_lights", 10);
  armors_data_pub_ =
      this->create_publisher<base_interfaces::msg::DebugArmors>(
          "/detector/debug_armors", 10);

  // this->declare_parameter("detector.result_img.jpeg_quality", 50);
  // this->declare_parameter("detector.binary_img.jpeg_quality", 50);

  binary_img_pub_ =
      image_transport::create_publisher(this, "/detector/binary_img");
  number_img_pub_ =
      image_transport::create_publisher(this, "/detector/number_img");
  result_img_pub_ =
      image_transport::create_publisher(this, "/detector/result_img");
}

void ArmorDetectorNode::destroyDebugPublishers() {
  lights_data_pub_.reset();
  armors_data_pub_.reset();

  binary_img_pub_.shutdown();
  number_img_pub_.shutdown();
  result_img_pub_.shutdown();
}

}  // namespace rm_auto_aim

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::ArmorDetectorNode)