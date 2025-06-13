// #include "hik_camera_node.h"
// #include <cstdio>
// #include <fmt/color.h>
// #include <iostream>
// #include <opencv2/core.hpp>
// #include <opencv2/core/operations.hpp>
// #include <string>
// #include <unistd.h>
// using namespace cv;
// using namespace std;
// namespace HDURM {
// HKcam::HKcam() {
//   nRet = MV_OK;
//   handle = nullptr;
//   pData = nullptr;
//   connected_flag = false;
// }

// void HKcam::SetParam() {
//   // 读取相机参数
//   char cwd[PATH_MAX];
//   if (getcwd(cwd, sizeof(cwd)) == nullptr) {
//     fmt::print(fmt::fg(fmt::color::red), "fail to getcwd(CameraParam.xml)\n");
//     return;
//   }
//   string path = cwd + string("/config/CameraParam.xml");
//   fmt::print(fmt::fg(fmt::color::green), "CameraParam.xml path: {}\n", path);
//   FileStorage CameraParam(path, FileStorage::READ);
//   if (!CameraParam.isOpened()) {
//     std::cerr << "failed to open CameraParam.xml" << std::endl;
//     return;
//   }

//   CameraParam["nImageOrientation"] >> nImageOrientation;
//   printf("imgOrientation: %d", nImageOrientation);

//   // set width
//   // 设置宽度
//   // 宽设置时需考虑步进(16)，即设置宽需16的倍数
//   // Step (16) should be considered when setting width, that is the width should
//   // be a multiple of 16
//   int nWidthValue;
//   CameraParam["nWidthValue"] >> nWidthValue;
//   nRet = MV_CC_SetIntValue(handle, "Width", nWidthValue);
//   if (MV_OK == nRet) {
//     printf("set Width OK!\n");
//   } else {
//     printf("set Width failed! nRet [%x]\n", nRet);
//   }

//   // set height
//   // 设置高度
//   // 高设置时需考虑步进(2)，即设置高需16的倍数
//   // Step (2) should be considered when setting height, that is the height
//   // should be a multiple of 2
//   int nHeightValue;
//   CameraParam["nHeightValue"] >> nHeightValue;
//   nRet = MV_CC_SetIntValue(handle, "Height", nHeightValue);
//   if (MV_OK == nRet) {
//     printf("set height OK!\n");
//   } else {
//     printf("set height failed! nRet [%x]\n", nRet);
//   }

//   // set OffsetX
//   // 设置水平偏移
//   int nOffsetXValue;
//   CameraParam["nOffsetXValue"] >> nOffsetXValue;
//   nRet = MV_CC_SetIntValue(handle, "OffsetX", nOffsetXValue);
//   if (MV_OK == nRet) {
//     printf("set OffsetX OK!\n");
//   } else {
//     printf("set OffsetX failed! nRet [%x]\n", nRet);
//   }

//   // set OffsetY
//   // 设置垂直偏移
//   int nOffsetYValue;
//   CameraParam["nOffsetYValue"] >> nOffsetYValue;
//   nRet = MV_CC_SetIntValue(handle, "OffsetY", nOffsetYValue);
//   if (MV_OK == nRet) {
//     printf("set OffsetY OK!\n");
//   } else {
//     printf("set OffsetY failed! nRet [%x]\n", nRet);
//   }
//   // set ReverseX
//   // 设置水平镜像
//   bool bSetBoolValue5;
//   CameraParam["bSetBoolValue5"] >> bSetBoolValue5;
//   nRet = MV_CC_SetBoolValue(handle, "ReverseX", bSetBoolValue5);
//   if (MV_OK == nRet) {
//     printf("Set ReverseX OK!\n");
//   } else {
//     printf("Set ReverseX Failed! nRet = [%x]\n", nRet);
//   }
//   // set ReverseY
//   // 设置垂直镜像
//   bool bSetBoolValue1;
//   CameraParam["bSetBoolValue1"] >> bSetBoolValue1;
//   nRet = MV_CC_SetBoolValue(handle, "ReverseY", bSetBoolValue1);
//   if (MV_OK == nRet) {
//     printf("Set ReverseY OK!\n");
//   } else {
//     printf("Set ReverseY Failed! nRet = [%x]\n", nRet);
//   }

//   // set PixelFormat
//   // 设置像素格式
//   int nPixelFormat;
//   CameraParam["PixelFormat"] >> nPixelFormat;
//   nRet = MV_CC_SetEnumValue(handle, "PixelFormat", nPixelFormat);
//   if (MV_OK == nRet) {
//     printf("set PixelFormat OK!\n");
//   } else {
//     printf("set PixelFormat failed! nRet [%x]\n", nRet);
//   }

//   // set BinningHorizontal
//   // 设置水平合并
//   int nBinningHorizontal;
//   CameraParam["nBinningHorizontal"] >> nBinningHorizontal;
//   nRet = MV_CC_SetEnumValue(handle, "BinningHorizontal", nBinningHorizontal);
//   if (MV_OK == nRet) {
//     printf("set BinningHorizontal OK!\n");
//   } else {
//     printf("set BinningHorizontal failed! nRet [%x]\n", nRet);
//   }

//   // set BinningVertical
//   // 设置垂直合并
//   int nBinningVertical;
//   CameraParam["nBinningVertical"] >> nBinningVertical;
//   nRet = MV_CC_SetEnumValue(handle, "BinningVertical", nBinningVertical);
//   if (MV_OK == nRet) {
//     printf("set BinningVertical OK!\n");
//   } else {
//     printf("set BinningVertical failed! nRet [%x]\n", nRet);
//   }

//   // set DecimationHorizontal
//   // 设置水平下采样
//   int nDecimationHorizontal;
//   CameraParam["nDecimationHorizontal"] >> nDecimationHorizontal;
//   nRet =
//       MV_CC_SetEnumValue(handle, "DecimationHorizontal", nDecimationHorizontal);
//   if (MV_OK == nRet) {
//     printf("set DecimationHorizontal OK!\n");
//   } else {
//     printf("set DecimationHorizontal failed! nRet [%x]\n", nRet);
//   }

//   // set DecimationVertical
//   // 设置垂直下采样
//   int nDecimationVertical;
//   CameraParam["nDecimationVertical"] >> nDecimationVertical;
//   nRet = MV_CC_SetEnumValue(handle, "DecimationVertical", nDecimationVertical);
//   if (MV_OK == nRet) {
//     printf("set DecimationVertical OK!\n");
//   } else {
//     printf("set DecimationVertical failed! nRet [%x]\n", nRet);
//   }

//   // set AcquisitionBurstFrameCount
//   // 设置采集触发帧率
//   int nAcquisitionBurstFrameCountValue;
//   CameraParam["nAcquisitionBurstFrameCountValue"] >>
//       nAcquisitionBurstFrameCountValue;
//   nRet = MV_CC_SetIntValue(handle, "AcquisitionBurstFrameCount",
//                            nAcquisitionBurstFrameCountValue);
//   if (MV_OK == nRet) {
//     printf("set AcquisitionBurstFrameCount OK!\n");
//   } else {
//     printf("set AcquisitionBurstFrameCount failed! nRet [%x]\n", nRet);
//   }

//   // set AcquisitionFrameRate
//   // 设置采集帧率
//   float fFPSValue;
//   CameraParam["fFPSValue"] >> fFPSValue;
//   nRet = MV_CC_SetFloatValue(handle, "AcquisitionFrameRate", fFPSValue);

//   if (MV_OK == nRet) {
//     printf("set AcquisitionFrameRate OK!\n");
//   } else {
//     printf("set AcquisitionFrameRate failed! nRet [%x]\n", nRet);
//   }

//   // set AcquisitionFrameRateEnable
//   // 设置使能采集帧率控制
//   bool bSetBoolValue3;
//   CameraParam["bSetBoolValue3"] >> bSetBoolValue3;
//   nRet =
//       MV_CC_SetBoolValue(handle, "AcquisitionFrameRateEnable", bSetBoolValue3);
//   if (MV_OK == nRet) {
//     printf("Set AcquisitionFrameRateEnable OK!\n");
//   } else {
//     printf("Set AcquisitionFrameRateEnable Failed! nRet = [%x]\n", nRet);
//   }

//   // set ExposureTime
//   // 设置曝光时间
//   float fExposureTime;
//   CameraParam["fExposureTime"] >> fExposureTime;
//   nRet = MV_CC_SetFloatValue(handle, "ExposureTime", fExposureTime);
//   if (MV_OK == nRet) {
//     printf("set ExposureTime OK!\n");
//   } else {
//     printf("set ExposureTime failed! nRet [%x]\n", nRet);
//   }

//   // set Gain
//   // 设置增益
//   float fGainvalue;
//   CameraParam["fGainvalue"] >> fGainvalue;
//   nRet = MV_CC_SetFloatValue(handle, "Gain", fGainvalue);
//   if (MV_OK == nRet) {
//     printf("set Gain OK!\n");
//   } else {
//     printf("set Gain failed! nRet [%x]\n", nRet);
//   }

//   // set GainAuto
//   // 设置自动增益
//   int nGainAuto;
//   CameraParam["nGainAuto"] >> nGainAuto;
//   nRet = MV_CC_SetEnumValue(handle, "GainAuto", nGainAuto);
//   if (MV_OK == nRet) {
//     printf("set GainAuto OK!\n");
//   } else {
//     printf("set GainAuto failed! nRet [%x]\n", nRet);
//   }

//   // set Blacklevel
//   // 设置黑电平
//   int nBlacklevelvalue;
//   CameraParam["nBlacklevelvalue"] >> nBlacklevelvalue;
//   nRet = MV_CC_SetIntValue(handle, "BlackLevel", nBlacklevelvalue);
//   if (MV_OK == nRet) {
//     printf("set Blacklevel OK!\n");
//   } else {
//     printf("set Blacklevel failed! nRet [%x]\n", nRet);
//   }

//   // set BlackLevelEnable
//   // 设置黑电平使能
//   bool bSetBoolValue2;
//   CameraParam["bSetBoolValue2"] >> bSetBoolValue2;
//   nRet = MV_CC_SetBoolValue(handle, "BlackLevelEnable", bSetBoolValue2);
//   if (MV_OK == nRet) {
//     printf("Set BlackLevelEnable OK!\n");
//   } else {
//     printf("Set BlackLevelEnable Failed! nRet = [%x]\n", nRet);
//   }
//   // set Gamma
//   // 设置伽马校正
//   float fGammaValue;
//   CameraParam["fGammaValue"] >> fGammaValue;
//   nRet = MV_CC_SetFloatValue(handle, "Gamma", fGammaValue);
//   if (MV_OK == nRet) {
//     printf("set Gamma OK!\n");
//   } else {
//     printf("set Gamma failed! nRet [%x]\n", nRet);
//   }

//   // set GammaEnable
//   // 设置伽马校正使能
//   bool bSetBoolValue4;
//   CameraParam["bSetBoolValue4"] >> bSetBoolValue4;
//   nRet = MV_CC_SetBoolValue(handle, "GammaEnable", bSetBoolValue4);
//   if (MV_OK == nRet) {
//     printf("Set GammaEnable OK!\n");
//   } else {
//     printf("Set GammaEnable Failed! nRet = [%x]\n", nRet);
//   }
// }

// bool HKcam::OpenCam(const string &cameraID) {
//   nRet = MV_OK;

//   MV_CC_DEVICE_INFO_LIST stDeviceList;
//   memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
//   // 枚举设备
//   // enum device
//   nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
//   if (!stDeviceList.nDeviceNum) {
//     printf("Find No Devices!\n");
//     return false;
//   }

//   // select the first camera connected
//   unsigned int nIndex = 0;

//   while (true) {
//     // 选择设备并创建句柄
//     // select device and create handle
//     nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[nIndex]);
//     if (MV_OK != nRet) {
//       printf("MV_CC_CreateHandle fail! nRet [%x]\n", nRet);
//       return false;
//     }

//     // 获取设备id
//     // get device id
//     stringstream ss;
//     ss << stDeviceList.pDeviceInfo[nIndex]
//               ->SpecialInfo.stUsb3VInfo.chDeviceGUID;
//     ss >> id;
//     cout << "camera id " << id << endl;

//     // 若指定了相机id，则判断是否为指定相机
//     // if camera id is specified, check if it is the specified camera
//     if (!cameraID.empty()) {
//       if (cameraID != id) // 若不是指定相机，则关闭句柄并继续枚举
//       {
//         printf("camera id %s not matched to desired %s\n", id.c_str(),
//                cameraID.c_str());
//         MV_CC_CloseDevice(handle);
//         MV_CC_DestroyHandle(handle);
//         nIndex++;
//         if (nIndex >= stDeviceList.nDeviceNum) // 若已枚举完所有相机，则返回
//         {
//           printf("Find No Devices!\n");
//           return false;
//         }
//         continue;
//       } else {
//         printf("ready to open camera %s\n", id.c_str());
//       }
//     } else {
//       printf("camera id not set, ready to open camera %s\n", id.c_str());
//     }

//     // 打开设备
//     // open device
//     nRet = MV_CC_OpenDevice(handle);
//     if (MV_OK != nRet) {
//       printf("MV_CC_OpenDevice fail! nRet [%x]\n", nRet);
//       return false;
//     }

//     // 设置触发模式为off
//     // set trigger mode as off
//     nRet = MV_CC_SetEnumValue(handle, "TriggerMode", 0);
//     if (MV_OK != nRet) {
//       printf("MV_CC_SetTriggerMode fail! nRet [%x]\n", nRet);
//       return false;
//     }

//     // set param
//     // 设置参数
//     SetParam();

//     // Get payload size
//     memset(&stParam, 0, sizeof(MVCC_INTVALUE));
//     nRet = MV_CC_GetIntValue(handle, "PayloadSize", &stParam);
//     if (MV_OK != nRet) {
//       printf("Get PayloadSize fail! nRet [0x%x]\n", nRet);
//       return false;
//     }

//     // s tart grab stream
//     nRet = MV_CC_StartGrabbing(handle);
//     if (MV_OK != nRet) {
//       printf("Start Grabbing fail! nRet [0x%x]\n", nRet);
//       return false;
//     }

//     // check
//     stImageInfo = {0};
//     memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));
//     pData = (unsigned char *)malloc(sizeof(unsigned char) * stParam.nCurValue);
//     if (NULL == pData) {
//       std::cout << "can't get size of a frame!" << std::endl;
//       return false;
//     }

//     connected_flag = true;

//     return true;
//   }
// }

// bool HKcam::CloseCam() {
//   // 停止取流
//   // end grab image
//   nRet = MV_CC_StopGrabbing(handle);
//   if (MV_OK != nRet) {
//     printf("MV_CC_StopGrabbing fail! nRet [%x]\n", nRet);
//     return false;
//   }
//   // 关闭设备
//   // close device
//   nRet = MV_CC_CloseDevice(handle);
//   if (MV_OK != nRet) {
//     printf("MV_CC_CloseDevice fail! nRet [%x]\n", nRet);
//     return false;
//   }
//   // 销毁句柄
//   // destroy handle
//   nRet = MV_CC_DestroyHandle(handle);
//   if (MV_OK != nRet) {
//     printf("MV_CC_DestroyHandle fail! nRet [%x]\n", nRet);
//     return false;
//   }
//   return true;
// }

// bool HKcam::GetFlame(cv::Mat &img) {
//   nRet = MV_OK;
//   nRet = MV_CC_GetOneFrameTimeout(handle, pData, stParam.nCurValue,
//                                   &stImageInfo, 1000);

//   if (nRet != MV_OK) {
//     printf("No data[0x%x]\n", nRet);
//     return false;
//   }

//   // if you want change the format ,edit here

//   img = cv::Mat(stImageInfo.nHeight, stImageInfo.nWidth, CV_8UC3, pData);

//   // 根据参数对img旋转
//   switch (nImageOrientation) {
//   case 0:
//     break;
//   case 1:
//     cv::rotate(img, img, cv::ROTATE_90_CLOCKWISE);
//     break;
//   case 2:
//     cv::rotate(img, img, cv::ROTATE_180);
//     break;
//   case 3:
//     cv::rotate(img, img, cv::ROTATE_90_COUNTERCLOCKWISE);
//     break;
//   }

//   return true;
// }

// string HKcam::GetCamName() { return id; }

// } // namespace HDURM











 






// // #include "MvCameraControl.h"
// // // ROS
// // #include <camera_info_manager/camera_info_manager.hpp>
// // #include <image_transport/image_transport.hpp>
// // #include <rclcpp/logging.hpp>
// // #include <rclcpp/rclcpp.hpp>
// // #include <rclcpp/utilities.hpp>
// // #include <sensor_msgs/msg/camera_info.hpp>
// // #include <sensor_msgs/msg/image.hpp>

// // namespace hik_camera
// // {
// // class HikCameraNode : public rclcpp::Node
// // {
// // public:
// //   explicit HikCameraNode(const rclcpp::NodeOptions & options) : Node("hik_camera", options)
// //   {
// //     RCLCPP_INFO(this->get_logger(), "Starting HikCameraNode!");

// //     MV_CC_DEVICE_INFO_LIST device_list;
// //     // enum device
// //     nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
// //     RCLCPP_INFO(this->get_logger(), "Found camera count = %d", device_list.nDeviceNum);

// //     while (device_list.nDeviceNum == 0 && rclcpp::ok()) {
// //       RCLCPP_ERROR(this->get_logger(), "No camera found!");
// //       RCLCPP_INFO(this->get_logger(), "Enum state: [%x]", nRet);
// //       std::this_thread::sleep_for(std::chrono::seconds(1));
// //       nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
// //     }

// //     MV_CC_CreateHandle(&camera_handle_, device_list.pDeviceInfo[0]);

// //     MV_CC_OpenDevice(camera_handle_);

// //     // Get camera infomation
// //     MV_CC_GetImageInfo(camera_handle_, &img_info_);
// //     image_msg_.data.reserve(img_info_.nHeightMax * img_info_.nWidthMax * 3);

// //     // Init convert param
// //     convert_param_.nWidth = img_info_.nWidthValue;
// //     convert_param_.nHeight = img_info_.nHeightValue;
// //     convert_param_.enDstPixelType = PixelType_Gvsp_RGB8_Packed;

// //     bool use_sensor_data_qos = this->declare_parameter("use_sensor_data_qos", true);
// //     auto qos = use_sensor_data_qos ? rmw_qos_profile_sensor_data : rmw_qos_profile_default;
// //     camera_pub_ = image_transport::create_camera_publisher(this, "/image_raw", qos);

// //     declareParameters();

// //     MV_CC_StartGrabbing(camera_handle_);

// //     // Load camera info
// //     camera_name_ = this->declare_parameter("camera_name", "narrow_stereo");
// //     camera_info_manager_ =
// //       std::make_unique<camera_info_manager::CameraInfoManager>(this, camera_name_);
// //     auto camera_info_url =
// //       this->declare_parameter("camera_info_url", "package://hik_camera/config/camera_info.yaml");
// //     if (camera_info_manager_->validateURL(camera_info_url)) {
// //       camera_info_manager_->loadCameraInfo(camera_info_url);
// //       camera_info_msg_ = camera_info_manager_->getCameraInfo();
// //     } else {
// //       RCLCPP_WARN(this->get_logger(), "Invalid camera info URL: %s", camera_info_url.c_str());
// //     }

// //     params_callback_handle_ = this->add_on_set_parameters_callback(
// //       std::bind(&HikCameraNode::parametersCallback, this, std::placeholders::_1));

// //     capture_thread_ = std::thread{[this]() -> void {
// //       MV_FRAME_OUT out_frame;

// //       RCLCPP_INFO(this->get_logger(), "Publishing image!");

// //       image_msg_.header.frame_id = "camera_optical_frame";
// //       image_msg_.encoding = "rgb8";

// //       while (rclcpp::ok()) {
// //         nRet = MV_CC_GetImageBuffer(camera_handle_, &out_frame, 1000);
// //         if (MV_OK == nRet) {
// //           convert_param_.pDstBuffer = image_msg_.data.data();
// //           convert_param_.nDstBufferSize = image_msg_.data.size();
// //           convert_param_.pSrcData = out_frame.pBufAddr;
// //           convert_param_.nSrcDataLen = out_frame.stFrameInfo.nFrameLen;
// //           convert_param_.enSrcPixelType = out_frame.stFrameInfo.enPixelType;

// //           MV_CC_ConvertPixelType(camera_handle_, &convert_param_);

// //           image_msg_.header.stamp = this->now();
// //           image_msg_.height = out_frame.stFrameInfo.nHeight;
// //           image_msg_.width = out_frame.stFrameInfo.nWidth;
// //           image_msg_.step = out_frame.stFrameInfo.nWidth * 3;
// //           image_msg_.data.resize(image_msg_.width * image_msg_.height * 3);

// //           camera_info_msg_.header = image_msg_.header;
// //           camera_pub_.publish(image_msg_, camera_info_msg_);

// //           MV_CC_FreeImageBuffer(camera_handle_, &out_frame);
// //           fail_conut_ = 0;
// //         } else {
// //           RCLCPP_WARN(this->get_logger(), "Get buffer failed! nRet: [%x]", nRet);
// //           MV_CC_StopGrabbing(camera_handle_);
// //           MV_CC_StartGrabbing(camera_handle_);
// //           fail_conut_++;
// //         }

// //         if (fail_conut_ > 5) {
// //           RCLCPP_FATAL(this->get_logger(), "Camera failed!");
// //           rclcpp::shutdown();
// //         }
// //       }
// //     }};
// //   }

// //   ~HikCameraNode() override
// //   {
// //     if (capture_thread_.joinable()) {
// //       capture_thread_.join();
// //     }
// //     if (camera_handle_) {
// //       MV_CC_StopGrabbing(camera_handle_);
// //       MV_CC_CloseDevice(camera_handle_);
// //       MV_CC_DestroyHandle(&camera_handle_);
// //     }
// //     RCLCPP_INFO(this->get_logger(), "HikCameraNode destroyed!");
// //   }

// // private:
// //   void declareParameters()
// //   {
// //     rcl_interfaces::msg::ParameterDescriptor param_desc;
// //     MVCC_FLOATVALUE f_value;
// //     param_desc.integer_range.resize(1);
// //     param_desc.integer_range[0].step = 1;
// //     // Exposure time
// //     param_desc.description = "Exposure time in microseconds";
// //     MV_CC_GetFloatValue(camera_handle_, "ExposureTime", &f_value);
// //     param_desc.integer_range[0].from_value = f_value.fMin;
// //     param_desc.integer_range[0].to_value = f_value.fMax;
// //     double exposure_time = this->declare_parameter("exposure_time", 5000, param_desc);
// //     MV_CC_SetFloatValue(camera_handle_, "ExposureTime", exposure_time);
// //     RCLCPP_INFO(this->get_logger(), "Exposure time: %f", exposure_time);

// //     // Gain
// //     param_desc.description = "Gain";
// //     MV_CC_GetFloatValue(camera_handle_, "Gain", &f_value);
// //     param_desc.integer_range[0].from_value = f_value.fMin;
// //     param_desc.integer_range[0].to_value = f_value.fMax;
// //     double gain = this->declare_parameter("gain", f_value.fCurValue, param_desc);
// //     MV_CC_SetFloatValue(camera_handle_, "Gain", gain);
// //     RCLCPP_INFO(this->get_logger(), "Gain: %f", gain);
// //   }

// //   rcl_interfaces::msg::SetParametersResult parametersCallback(
// //     const std::vector<rclcpp::Parameter> & parameters)
// //   {
// //     rcl_interfaces::msg::SetParametersResult result;
// //     result.successful = true;
// //     for (const auto & param : parameters) {
// //       if (param.get_name() == "exposure_time") {
// //         int status = MV_CC_SetFloatValue(camera_handle_, "ExposureTime", param.as_int());
// //         if (MV_OK != status) {
// //           result.successful = false;
// //           result.reason = "Failed to set exposure time, status = " + std::to_string(status);
// //         }
// //       } else if (param.get_name() == "gain") {
// //         int status = MV_CC_SetFloatValue(camera_handle_, "Gain", param.as_double());
// //         if (MV_OK != status) {
// //           result.successful = false;
// //           result.reason = "Failed to set gain, status = " + std::to_string(status);
// //         }
// //       } else {
// //         result.successful = false;
// //         result.reason = "Unknown parameter: " + param.get_name();
// //       }
// //     }
// //     return result;
// //   }

// //   sensor_msgs::msg::Image image_msg_;

// //   image_transport::CameraPublisher camera_pub_;

// //   int nRet = MV_OK;
// //   void * camera_handle_;
// //   MV_IMAGE_BASIC_INFO img_info_;

// //   MV_CC_PIXEL_CONVERT_PARAM convert_param_;

// //   std::string camera_name_;
// //   std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
// //   sensor_msgs::msg::CameraInfo camera_info_msg_;

// //   int fail_conut_ = 0;
// //   std::thread capture_thread_;

// //   OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;
// // };
// // }  // namespace hik_camera

// // #include "rclcpp_components/register_node_macro.hpp"

// // RCLCPP_COMPONENTS_REGISTER_NODE(hik_camera::HikCameraNode)
