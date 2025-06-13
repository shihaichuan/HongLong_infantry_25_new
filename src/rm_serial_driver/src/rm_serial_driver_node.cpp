#include "rm_serial_driver/rm_serial_driver_node.hpp"
#include <Eigen/Geometry>
#include <rm_utils/math/utils.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
// std
#include <chrono>
#include <cstdint>
#include <memory>
#include <thread>
// ros2
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
// project
#include "rm_utils/math/utils.hpp"
#define DEBUG_SERIAL_DRIVER 0
namespace rm_serial_driver
{
  SerialDriverNode::SerialDriverNode(const rclcpp::NodeOptions &options)
      : rclcpp::Node("rm_serial_driver_node", options)
  {
    getParam();
    target_frame_ = this->declare_parameter("target_frame","base_link");
    port_ = std::make_shared<Port>(config_);
    RCLCPP_INFO(get_logger(), "Begin the driver Node !");
    timestamp_offset_ = this->declare_parameter("timestamp_offset", 0.0);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    detector_param_client_ =
        std::make_shared<rclcpp::AsyncParametersClient>(this, "armor_detector");
    reset_tracker_client_ =
        this->create_client<std_srvs::srv::Trigger>("/tracker/reset");
    yaw_pub_ = this->create_publisher<std_msgs::msg::Float64>("current_yaw", 10);

    latency_pub_ =
        this->create_publisher<std_msgs::msg::Float64>("/latency", 10);

    for (int i = 0; i < 4; i++)
    {
      if (port_->isPortOpen())
      {
        break;
      }
      else
      {
        port_->openPort();
      }
    }
    if (!port_->isPortOpen())
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to open port");
      return;
    }

    gimbal_cmd_sub_ = this->create_subscription<base_interfaces::msg::GimbalPose>(
        "setGimbalAngle", rclcpp::SensorDataQoS(),
        std::bind(&SerialDriverNode::GimbalCmdCallback, this,
                  std::placeholders::_1));
    timer_transmit =
        this->create_wall_timer(std::chrono::milliseconds(1),
                                std::bind(&SerialDriverNode::transmit, this));
    rx_thread = std::thread(&SerialDriverNode::rx, this);
    heartbeat_ = rm_auto_aim::HeartBeatPublisher::create(this);
  }

  void SerialDriverNode::rx()
  {
    while (rclcpp::ok())
    {
      receive();
      pkgState = PkgState::COMPLETE;
      while (receive_buffer.size() > 0 &&
             pkgState != PkgState::HEADER_INCOMPLETE &&
             pkgState != PkgState::PAYLOAD_INCOMPLETE)
      {
        pkgState = decode();
      }
    }
  }
  int SerialDriverNode::receive()
  {
    int read_num = 0;

    // read_num =read(port_->fd,receiveBuffer, 64);
    read_num = port_->receive(receiveBuffer);
    read_sum += read_num;
    if (read_num > 0)
    {
      receive_buffer.insert(receive_buffer.end(), receiveBuffer,
                            receiveBuffer + read_num);
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "Can not read from the uart !");
      port_->closePort();
      rclcpp::shutdown();
      // port_->openPort();
    }
    return read_num;
  }
  int SerialDriverNode::transmit()
  {
    uint8_t buffer[TRANSMIT_BUFFER];
    long size = transmit_buffer.size();
    if (size)
    {
      while (size > 2 * TRANSMIT_BUFFER && transmit_buffer.size() > 0)
      {
        size -= TRANSMIT_BUFFER;
        std::lock_guard<std::mutex> lockf(transmit_mutex);
        std::copy(transmit_buffer.begin(),
                  transmit_buffer.begin() + TRANSMIT_BUFFER, buffer);
        transmit_buffer.erase(transmit_buffer.begin(),
                              transmit_buffer.begin() + TRANSMIT_BUFFER);

        write_num = port_->transmit(buffer, TRANSMIT_BUFFER);

        // write_num = write(port->fd, buffer, TRANSMIT_BUFFER);

        if (write_num < 0)
        {
          RCLCPP_ERROR(get_logger(), "Can not transmit");
          port_->closePort();
          rclcpp::sleep_for(std::chrono::seconds(1));
          port_->openPort();
        }
        // GimbalCommand pack = bufferToStruct<GimbalCommand>(buffer);
        // for(int i = 0; i < 33; i++){
        //   printf("%02X ", buffer[i]);
        // }
        // printf("\n");
        // RCLCPP_INFO(get_logger(), "transmit: pitch:%f yaw:%f is_shoot:%d", pack.pitch, pack.yaw, pack.is_shoot);
      }
    }
    return write_num;
  }
  void SerialDriverNode::classify(uint8_t *data)
  {
    // Header header = arrayToStruct<Header>(data);
    // target_frame_ = this->declare_parameter("target_frame","base_link");
    InfantryGimbalMsg packet = bufferToStruct<InfantryGimbalMsg>(data);
    if (packet.header.protocolID != INFANTRY_GIMBAL_MSG)
      return;
    try
    {
      classify_pkg_sum++;
      geometry_msgs::msg::TransformStamped t;
      timestamp_offset_ = this->get_parameter("timestamp_offset").as_double();
      t.header.stamp =
          this->now() + rclcpp::Duration::from_seconds(timestamp_offset_);
      t.header.frame_id = "base_link";
      t.child_frame_id = "gimbal_link";
      tf2::Quaternion q;
      double roll, pitch, yaw;
      roll = packet.roll * M_PI / 180.0;
      pitch = packet.pitch * M_PI / 180.0;
      yaw = packet.yaw * M_PI / 180.0;
      auto yaw_msg=std_msgs::msg::Float64();
      yaw_msg.data=packet.yaw*M_PI/180.0;
      yaw_pub_->publish(yaw_msg);
      q.setRPY(roll, pitch, yaw);
      t.transform.rotation = tf2::toMsg(q);
      // RCLCPP_INFO(get_logger(), "roll:%f pitch:%f yaw:%f", roll, pitch, yaw);
      tf_broadcaster_->sendTransform(t);

    }
    catch (const std::exception &ex)
    {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 20,
                            "Error while receiving data: %s", ex.what());
      port_->closePort();
      rclcpp::sleep_for(std::chrono::seconds(1));
      port_->openPort();
    }
  }

  PkgState SerialDriverNode::decode()
  {
    int size = receive_buffer.size();

    for (int i = 0; i < size; i++)
    {
      if (receive_buffer[i] == 0xFF)
      {
        if (i + int(sizeof(Header)) > size)
        {
          return PkgState::HEADER_INCOMPLETE;
        }

        std::copy(receive_buffer.begin() + i,
                  receive_buffer.begin() + i + sizeof(Header), decodeBuffer);
        crc_ok_header =
            crc16::Verify_CRC16_Check_Sum(decodeBuffer, sizeof(Header));

        if (!crc_ok_header)
        {
          RCLCPP_ERROR(get_logger(), "CRC error in header !");
          error_sum_header++;
          try
          {
            receive_buffer.erase(receive_buffer.begin() + i,
                                 receive_buffer.begin() + i + sizeof(Header));
          }
          catch (const std::exception &ex)
          {
            RCLCPP_ERROR(get_logger(), "Error creating serial port:  %s",
                         ex.what());
          }

          return PkgState::CRC_HEADER_ERRROR;
        }

        this->header = bufferToStruct<Header>(decodeBuffer);

        // pkg length = payload(dataLen) + header len (include header crc) +
        // 2crc
        if (i + int(header.dataLen) + int(sizeof(Header)) + 2 > size)
        {
          return PkgState::PAYLOAD_INCOMPLETE;
        }

        std::copy(
            receive_buffer.begin() + i,
            receive_buffer.begin() + i + header.dataLen + sizeof(Header) + 2,
            decodeBuffer);
        crc_ok = crc16::Verify_CRC16_Check_Sum(
            decodeBuffer, header.dataLen + sizeof(Header) + 2);

        if (!crc_ok)
        {
          error_sum_payload++;
          // payload error
          try
          {
            // check if there is a coming pkg
            for (int j = i + 1;
                 j < int(header.dataLen) + int(sizeof(Header)) + 2 + i; j++)
            {
              if (receive_buffer[j] == 0xFF)
              {
                if (j + sizeof(Header) >
                    header.dataLen + sizeof(Header) + 2 + i)
                {
                  receive_buffer.erase(receive_buffer.begin(),
                                       receive_buffer.begin() + j);
                  return PkgState::HEADER_INCOMPLETE;
                }
                std::copy(receive_buffer.begin() + i,
                          receive_buffer.begin() + i + sizeof(Header),
                          decodeBuffer);
                crc_ok_header =
                    crc16::Verify_CRC16_Check_Sum(decodeBuffer, sizeof(Header));

                if (!crc_ok_header)
                {
                  receive_buffer.erase(
                      receive_buffer.begin(),
                      receive_buffer.begin() + j + sizeof(Header));
                  j += sizeof(Header) - 1;
                  continue;
                }

                this->header = bufferToStruct<Header>(decodeBuffer);

                if (j + sizeof(Header) + header.dataLen + 2)
                {
                  receive_buffer.erase(receive_buffer.begin(),
                                       receive_buffer.begin() + j);
                  return PkgState::PAYLOAD_INCOMPLETE;
                }

                std::copy(receive_buffer.begin() + i,
                          receive_buffer.begin() + i + header.dataLen +
                              sizeof(Header) + 2,
                          decodeBuffer);
                crc_ok = crc16::Verify_CRC16_Check_Sum(
                    decodeBuffer, header.dataLen + sizeof(Header) + 2);

                if (!crc_ok)
                {
                  RCLCPP_ERROR(get_logger(), "CRC error in payload !");
                  receive_buffer.erase(receive_buffer.begin(),
                                       receive_buffer.begin() + j +
                                           sizeof(Header) + header.dataLen + 2);
                  j += sizeof(Header) + header.dataLen + 2 - 1;
                  continue;
                }
              }
            }
            receive_buffer.erase(receive_buffer.begin(),
                                 receive_buffer.begin() + i + header.dataLen +
                                     sizeof(Header) + 2);
          }
          catch (const std::exception &ex)
          {
            RCLCPP_ERROR(get_logger(), "Error creating serial port:  %s",
                         ex.what());
          }

          return PkgState::CRC_PKG_ERROR;
        }

        // complete
        try
        {
          receive_buffer.erase(
              receive_buffer.begin(),
              receive_buffer.begin() + i + header.dataLen + sizeof(Header) + 2);
        }
        catch (const std::exception &ex)
        {
          RCLCPP_ERROR(get_logger(), "Error creating serial port:  %s",
                       ex.what());
        }

        pkg_sum++;

        std::chrono::high_resolution_clock::time_point end =
            std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> time = end - start;

#if DEBUG_SERIAL_DRIVER
        RCLCPP_INFO(
            get_logger(),
            "crc error rate : %f pkg sum rate: %f hz classify_pkg_sum "
            "rate:%f hz,read_sum rate: %f "
            "transmit hz :%f  time : %f \n",
            float(error_sum_header + error_sum_payload) / float(pkg_sum),
            double(pkg_sum) / time.count(),
            double(classify_pkg_sum) / time.count(),
            float(read_sum) * 11 / time.count(), trans_pkg_sum / time.count(),
            time.count());
#endif

        classify(decodeBuffer);
        return PkgState::COMPLETE;
      }
    }
    receive_buffer.erase(receive_buffer.begin(), receive_buffer.end());
    return PkgState::OTHER;
  }

  // communicate with RV
  void SerialDriverNode::GimbalCmdCallback(base_interfaces::msg::GimbalPose::SharedPtr msg)
  {
    try
    {

      uint8_t buffer[sizeof(GimbalCommand)];
      GimbalCommand packet;
      packet.header.dataLen = sizeof(GimbalCommand) - sizeof(Header) - 2;
      packet.header.protocolID = GIMBAL_CMD;
      packet.pitch = -(msg->pitch);
      packet.yaw = msg->yaw;
      // std::cout<<"yaw        "<<packet.yaw<<std::endl;
      // std::cout<<"pitch      "<<packet.pitch<<std::endl;
      packet.bulletnum = (uint8_t)msg->bulletnum;
      crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet),
                                    sizeof(Header));
      crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet),
                                    sizeof(GimbalCommand));

      structToBuffer(packet, buffer);
      std::lock_guard<std::mutex> lockf(transmit_mutex);
      transmit_buffer.insert(transmit_buffer.end(), buffer,
                             buffer + sizeof(GimbalCommand));
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(get_logger(), "Error while sending data: %s", e.what());
      port_->closePort();
      rclcpp::sleep_for(std::chrono::seconds(1));
      port_->openPort();
    }
  }




  void SerialDriverNode::setParam(const rclcpp::Parameter &param)
  {
    if (!detector_param_client_->service_is_ready())
    {
      RCLCPP_WARN(get_logger(), "Service not ready, skipping parameter set");
      return;
    }

    if (!set_param_future_.valid() ||
        set_param_future_.wait_for(std::chrono::seconds(0)) ==
            std::future_status::ready)
    {
      RCLCPP_INFO(get_logger(), "Setting detect_color to %ld...",
                  param.as_int());
      set_param_future_ = detector_param_client_->set_parameters(
          {param}, [this, param](const ResultFuturePtr &results)
          {
            for (const auto& result : results.get()) {
              if (!result.successful) {
                RCLCPP_ERROR(get_logger(), "Failed to set parameter: %s",
                             result.reason.c_str());
                return;
              }
            }
            RCLCPP_INFO(get_logger(), "Successfully set detect_color to %ld!",
                        param.as_int());
            initial_set_param_ = true; });
    }
  }

  void SerialDriverNode::getParam()
  {

    int baud_rate{};
    bool flowcontrol = false;
    auto pt = Parity::NONE;
    auto sb = rm_serial_driver::StopBit::ONE;

    try
    {
      device_name_ =
          declare_parameter<std::string>("device_name", "/dev/ttyACM0");
    }
    catch (rclcpp::ParameterTypeException &ex)
    {
      RCLCPP_ERROR(get_logger(), "The device name provided was invalid");
      throw ex;
    }

    try
    {
      baud_rate = declare_parameter<int>("baud_rate", 961200);
    }
    catch (rclcpp::ParameterTypeException &ex)
    {
      RCLCPP_ERROR(get_logger(), "The baud_rate provided was invalid");
      throw ex;
    }

    try
    {
      flowcontrol = declare_parameter<bool>("flow_control", false);
    }
    catch (rclcpp::ParameterTypeException &ex)
    {
      RCLCPP_ERROR(get_logger(), "The flow_control provided was invalid");
      throw ex;
    }

    try
    {
      const auto pt_string = declare_parameter<std::string>("parity", "none");

      if (pt_string == "none")
      {
        pt = Parity::NONE;
      }
      else if (pt_string == "odd")
      {
        pt = Parity::ODD;
      }
      else if (pt_string == "even")
      {
        pt = Parity::EVEN;
      }
      else
      {
        throw std::invalid_argument{
            "The parity parameter must be one of: none, odd, or even."};
      }
    }
    catch (rclcpp::ParameterTypeException &ex)
    {
      RCLCPP_ERROR(get_logger(), "The parity provided was invalid");
      throw ex;
    }

    try
    {
      const auto sb_string = declare_parameter<std::string>("stop_bits", "1.0");

      if (sb_string == "1" || sb_string == "1.0")
      {
        sb = StopBit::ONE;
      }
      else if (sb_string == "1.5")
      {
        sb = StopBit::ONE_POINT_FIVE;
      }
      else if (sb_string == "2" || sb_string == "2.0")
      {
        sb = StopBit::TWO;
      }
      else
      {
        throw std::invalid_argument{
            "The stop_bits parameter must be one of: 1, 1.5, or 2."};
      }
    }
    catch (rclcpp::ParameterTypeException &ex)
    {
      RCLCPP_ERROR(get_logger(), "The stop_bits provided was invalid");
      throw ex;
    }

    config_ = std::make_unique<SerialConfig>(baud_rate, 8, flowcontrol, sb, pt,
                                             device_name_);
  }
}
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rm_serial_driver::SerialDriverNode)