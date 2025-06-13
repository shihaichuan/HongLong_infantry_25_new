#include <vector>
#include <string>
#include <cmath>         // 用于round函数和PI常量（需确认是否有PI定义）
#include <opencv2/core.hpp>  // 用于cv::Point2d
#include <Eigen/Dense>       // 用于Eigen::Matrix
#include "base_interfaces/msg/armor.hpp"
#include "base_interfaces/msg/armors.hpp"
#include <geometry_msgs/msg/vector3.hpp>

#include <map>
#include <iostream>
#include "base_interfaces/msg/armor.hpp" // 定义Armors
#include "rclcpp/rclcpp.hpp" // ROS2节点支持


#pragma once


// 将弧度约束在[-M_PI, M_PI]范围内
#ifndef _std_radian
#define _std_radian(angle) ((angle) + round((0 - (angle)) / (2 * M_PI)) * (2 * M_PI))
#endif


namespace rm_auto_aim
{
    /**
     * @brief 运动学状态
     */
    class KinematicModel
    {
    public:
        KinematicModel();
        ~KinematicModel();



    public:
        base_interfaces::msg::Armors armors; 
        std_msgs::msg::Header header;
        geometry_msgs::msg::Point center;
        geometry_msgs::msg::Vector3 velocity;
        
        std::vector<float> height;     // 装甲高度。0对应索引0、2，1对应索引1、3
        std::vector<float> radius;     // 旋转半径。0对应索引0、2，1对应索引1、3
        double phase;                   // 角度（并非定值）
        double palstance; // 角速度
        int32_t armor_num;              

        int number;                     // 装甲板数量
        int index;                      // 当前目标索引

    public:
        enum Type
        {
            UNKNOWN = -1,
            STANDARD = 0,
            BALANCE = 1,
            OUTPOST = 2
        };
    };

    class StandardModel : public KinematicModel
    {
    public:
        StandardModel();
        StandardModel(const Eigen::Matrix<double, 10, 1> &X);
        ~StandardModel();

        /**
         * @brief 为了保留index，赋值时不继承index
         */
        StandardModel operator=(const StandardModel &status);

        /**
         * @brief 获取该运动状态下所有装甲板
         * @param predict_time 预测时间
         * @brief 打印信息
         */

           // 旋转中心速度
    };

    class BalanceModel : public KinematicModel
    {
    public:
        BalanceModel();
        BalanceModel(const Eigen::Matrix<double, 7, 1> &X);
        ~BalanceModel();

        /**
         * @brief 为了保留index，赋值时不继承index
         */
        BalanceModel operator=(const BalanceModel &status);


               // 旋转中心速度
    };



}   // HL
