#pragma once
#include <map>
#include <iostream>
#include "base_interfaces/msg/armor.hpp" // 定义Armors
#include "rclcpp/rclcpp.hpp" // ROS2节点支持
#include "aimer/KinematicModel.hpp"
#include "aimer/ModelObserver.hpp"
#include "base_interfaces/msg/armor.hpp"
#include "base_interfaces/msg/armors.hpp"
#include "base_interfaces/msg/gimbal_pose.hpp"
#include "pose_solver/pose_solver.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <deque>
#include <geometry_msgs/msg/vector3.hpp>
#include "base_interfaces/msg/kinematic_status.hpp" 
#include "sensor_msgs/msg/camera_info.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <message_filters/subscriber.h>
#include <tf2_ros/message_filter.h>
#include "sensor_msgs/msg/image.hpp"
#include <image_transport/image_transport.hpp> 
#include "std_msgs/msg/float64.hpp"

// #include "std_msgs/msg/float64.hpp"
// #define STANDARD
// #define BALANCE



namespace rm_auto_aim
{
using tf2_filter = tf2_ros::MessageFilter< const base_interfaces::msg::Armors>;
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
    class Aimer : public rclcpp::Node
    {
    public:
        Aimer(const rclcpp::NodeOptions & options);
        // ~Aimer();
        /**
         * @brief 模块主接口。
         * @param armors 相机坐标系装甲板序列
         * @param cur_pose 当前位姿 
         * @param bullet_speed 当前射速
         * @param control_mode 0 返回目标位姿，1 返回目标速度
         * @return 云台的目标位姿或者目标速度
         */
        void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg);
        void buildModel(const base_interfaces::msg::Armors::SharedPtr armors_msg);
        void ModelCallBack();
        /**
         * @brief 数据解算核心过程，同时也是提供给数据模拟器的主接口
         * @param armors 装甲板序列
         * @return 运动学状态
         */
        std::shared_ptr<KinematicModel> resolve(base_interfaces::msg::Armors &armors);

        /**
         * @brief 完全重置
         */
        void reset();

        /**
         * @return 是否允许控制
         */
        bool isReady();

    base_interfaces::msg::GimbalPose getAngle(geometry_msgs::msg::Pose target_armor_pose, float bullet_speed, base_interfaces::msg::GimbalPose &gimbal_pose);

        /**
         * @brief 抛弃装甲板序列中数据异常者
         * @param armors 用于处理的装甲板序列
         */
        void errorHandling(base_interfaces::msg::Armors &armors);

        /** 
         * @brief 强制指定运动学模型并实例化，慎用
         */
        // 新增时间同步函数声明
    double estimateTimeDelay(const base_interfaces::msg::Armors& armors);
    void setModelType(KinematicModel::Type type);
    // double getDistance(const cv::Point3d& point);
    std::vector<base_interfaces::msg::Armor> getArmors(double predict_time, int number);
    bool getTransform(std::string src_frame,std::string tar_frame,  builtin_interfaces::msg::Time stamp, geometry_msgs::msg::TransformStamped &transform);

    private:
        // void timer_callback();
        /**
         * @brief 读取参数配置文    rclcpp::TimerBase::SharedPtr timer_ptr_; 件
         * @param file_path 配置文件路径
         */
        // void setParam(const std::string &file_path);

        /**
         * @brief 根据熵权法获取装甲板相关性矩阵，相关性指标为负向指标
         * @param armors 识别装甲板序列
         * @param status 当前运动状态
         * @return 相关性矩阵
         */
        Eigen::MatrixXd getScoreMat(base_interfaces::msg::Armors &detect_armors, base_interfaces::msg::Armors &standard_armors);

        /**
         * @brief 装甲板匹配
         * @param score 相关性矩阵。相关性指标为负向指标
         * @return 装甲板关联。键为识别装甲板索引，值为标准装甲板索引
         */
        //std::map<int, int> match(const Eigen::MatrixXd &score);

        
        bool m_debug;
        bool m_ekf_on;                          // 是否使用EKF
        bool m_predict_on;                      // 是否使用预测
        bool m_track_center;                    // 是否跟随中心
        bool m_tracking;                        // 是否跟随
        bool m_enable_shoot;                    // 是否允许自动击发

        std::string m_tracked_ID;                       // 当前锁定ID
        int m_type_init_cnt;                    // 模型判断识别计数
        int m_track_lost_cnt;                   // 完全丢识别计数
        int m_center_tracked;                   // 锁中心状态标志位
        int m_all_white_cnt;                    // 连续识别到全部为白色装甲板的次数
        double m_next_shoot_time;               // 下次允许射击时间

        // 参数
        double m_time_off;                      // 预测时间补偿
        double m_switch_threshold;              // 更新装甲板切换的角度阈值，角度制
        double m_init_pose_tolerance;           // 初始化位姿变化最大值，角度制
        double m_rubbish_data_tolerance;        // 可接受作为输入数据的装甲板在相机系的最大角度，角度制
        double m_force_aim_palstance_threshold; // 强制允许发射的目标旋转速度最大值，弧度制
        double m_aim_center_palstance_threshold;// 跟随圆心转跟随装甲板的目标旋转速度最大值，弧度制
        double m_switch_trackmode_threshold;    // 更换锁中心模式角速度阈值，弧度制
        double m_aim_angle_tolerance;           // 自动击发时目标装甲板相对偏角最大值，角度制
        double m_aim_pose_tolerance;            // 自动击发位姿偏差最大值，弧度制
        double m_aim_center_angle_tolerance;    // 跟随圆心自动击发目标偏角判断，角度制
        double m_score_tolerance;               // 装甲板匹配得分最大值
        int m_all_white_tolerance_stop_shoot;   // 连续识别到全部为白色装甲板的次数容忍度，将会停止发射
        int m_all_white_tolerance_reset;        // 连续识别到全部为白色装甲板的次数容忍度，将会重置整个模块
        double m_shoot_interval;                // 发射间隔，单位：秒
        double pitch_off_ = 0.0;                         ///< 俯仰偏移
        double yaw_off_ = 0.0;                           ///< 偏航偏移
        bool fix_on_ = false;

        bool m_observer_debug;
        double m_observer_dt;
        double m_standard_init_radius;
        double m_standard_gain;
        std::array<double, 4> m_standard_process_noise;
        std::array<double, 3> m_standard_measure_noise;
        std::array<double, 4> m_balance_process_noise;
        std::array<double, 3> m_balance_measure_noise;

        double m_balance_init_radius;
        double m_balance_gain;

        std::string id;


        base_interfaces::msg::GimbalPose m_cur_pose;                  // 当前位姿
        base_interfaces::msg::GimbalPose m_target_pose;               // 目标位姿

        KinematicModel::Type m_model_type;                  // 模型类型
        std::shared_ptr<KinematicModel> m_status;           // 当前状态。为了保留最佳目标装甲板，必须声明为类成员变量以保留其索引值
        std::shared_ptr<ModelObserver> m_model_observer;    // 状态观测器
        rclcpp::Publisher<base_interfaces::msg::KinematicStatus>::SharedPtr top_pub_;
        // rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr test_pub_;
        rclcpp::TimerBase::SharedPtr timer_ptr_;
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
        message_filters::Subscriber<base_interfaces::msg::Armors> armors_sub_;
        std::shared_ptr<tf2_ros::MessageFilter<base_interfaces::msg::Armors>> tf2_filter_;
        std::shared_ptr<sensor_msgs::msg::CameraInfo> cam_info_;
        std::string camera_optical_link = "camera_optical_frame";
        std::string base_link_ = "base_link"; 
        std::string gimbal_link = "gimbal_link";
        std::string camera_link = "camera_link";
        double bullet_speed = 24;  
        base_interfaces::msg::Armors init_armors;
        std::array<double, 3> rpy_;
        std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
        rclcpp::TimerBase::SharedPtr timer_; 
        base_interfaces::msg::Armors processed_armor_seq_;
        image_transport::Publisher aimer_img_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_pu_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr x_pu_;


        
    private:
    class HungarianSolver {
    public:
         std::map<int, int> solve(const Eigen::MatrixXd& cost_matrix, double score_max, int m);
    private:
        Eigen::MatrixXd cost_matrix_;
        int rows_, cols_;
        std::vector<double> label_x_, label_y_;
        std::vector<int> match_x_, match_y_;
        std::vector<bool> visited_x_, visited_y_;
        int dim_;
         int original_cols_;  // 新增成员变量
        int original_rows_;  // 新增成员变量


    bool findAugmentingPath(int u);
    void updateLabels();
    double calculateEpsilon(int u, int v) const;
    
    };
    HungarianSolver m_hungarian_solver;

    /************ ROI ************
     * @brief 控制重投影ROI回传
     */
    public:
    private:
        // /**

        // cv::Rect2d m_deep_default_roi;
        cv::Mat m_camera_mat;

    /************ ROI ************/
    };

}   // HL
