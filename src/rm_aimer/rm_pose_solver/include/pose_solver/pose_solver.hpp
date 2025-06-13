#include <opencv2/core.hpp>
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <base_interfaces/msg/armor.hpp>
#include <base_interfaces/msg/armors.hpp>
#include <base_interfaces/msg/gimbal_pose.hpp>
#include <base_interfaces/msg/kinematic_status.hpp>
#include <base_interfaces/msg/debug_of_kinematic.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <chrono>
#include <mutex>
#include <cmath>   
#include "std_msgs/msg/float64.hpp"


// 将角度约束在[-Pi, Pi]
#ifndef _std_radian
// #define _std_radian(angle) ((angle) + round((0 - (angle)) / (2 * PI)) * (2 * PI))
#define _std_radian(angle) ((angle) + round((0 - (angle)) / (2 * M_PI)) * (2 * M_PI))
#define D2R(x) ((x) * M_PI / 180.0)
#define R2D(x) ((x) * 180.0 / M_PI)
#endif

using std::placeholders::_1;
using std::placeholders::_2;

namespace rm_auto_aim{

/**
 * @brief PoseSolver类，用于位姿解算，并云台发布位姿
 */
class PoseSolver : public rclcpp::Node
{
public:
    /**
     * @brief 构造函数
     * @param name 节点名字
     */
    explicit PoseSolver(const rclcpp::NodeOptions & options);
    bool getAngle(geometry_msgs::msg::Pose target_position, float bullet_speed, base_interfaces::msg::GimbalPose &gimbal_pose);
    std::vector<base_interfaces::msg::Armor> getArmors(double predict_time, int number);
    bool getTransform(std::string src_frame, std::string tar_frame, geometry_msgs::msg::TransformStamped &transform);
private:
    std::map<int, int> model_armors;                                                    ///< 存储模型到装甲板数量的映射
    std::mutex mtx;                                                                     ///< 用于保护model指针的互斥锁
    rclcpp::TimerBase::SharedPtr timer_ptr_;                                            ///< 小云台控制timer                                  ///< 大Yaw控制timer
    
    std::shared_ptr<base_interfaces::msg::KinematicStatus> status_ptr_;
std::mutex status_mutex_;                     ///< model指针
    rclcpp::Subscription<base_interfaces::msg::KinematicStatus>::SharedPtr top_sub_;    ///< ekf模型订阅者
    rclcpp::Publisher<base_interfaces::msg::GimbalPose>::SharedPtr Gimbal_pose_pub_;    ///< 云台位姿发布者    ///< 大Yaw发布者
    rclcpp::Publisher<base_interfaces::msg::DebugOfKinematic>::SharedPtr debug_pub;
            rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_p_;


    ///< debug使用，armors发布者，用于发布预测后的装甲板
    rclcpp::Publisher<base_interfaces::msg::Armors>::SharedPtr armors_pub;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};                  ///< tf监听器
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;                                        ///< tf缓冲区

    std::string base_link_ = "base_link";           ///< 绝对系
    double m_bullet_speed = 24;                     ///< 子弹速度

    std::string shooter_link_ = "shooter_link";        ///< 电机系
    std::string gimbal_link = "gimbal_link";   ///< 云台系
    bool fix_on_ = false;                           ///< 是否补偿
    bool m_track_center = false;                    ///< 是否（想要）锁中心
    bool m_enable_center_track = false;             ///< 是否能够锁中心
    float pitch_off_ = 0.0;                         ///< 俯仰偏移
    float yaw_off_ = 0.0;                           ///< 偏航偏移
    float m_timeout = 0.1;

    float m_filter_alpha = 0.1;
    float m_filter_last_output;
    bool  m_filter_init = false;

    float m_switch_threshold = 0.0;                 ///< 装甲板切换的角度差
    float m_time_off = 0.0;                         ///< 预测时间
    float m_force_aim_palstance_threshold = 0.0;    ///< 强制允许发射的目标旋转速度最大值
    float m_aim_angle_tolerance = 0.0;              ///< 自动击发时目标装甲板绝对偏角最大值
    float m_aim_pose_tolerance = 0.0;               ///< 发射角度容忍度
    float m_aim_center_palstance_threshold = 0.0;   ///< 锁中心的最低角速度阈值，角速度小于这个值不锁中心，弧度制
    float m_switch_trackmode_threshold = 0.0;       ///< 更换锁中心模式角速度阈值，弧度制
    float m_aim_center_angle_tolerance = 0.0;       ///< 锁中心，目标装甲板偏角判断

    bool debug = false;                             ///< 调试模式
    double last_time = 0;
    double shoot_interval = 0.0;//发射间隔
    double last_shooter_time = 0;
    
    int all_white_tolerance_stop_shoot_;
    int white_frame_ = 0;

    visualization_msgs::msg::Marker position_marker_;
    visualization_msgs::msg::Marker position_;

    visualization_msgs::msg::Marker linear_v_marker_;
    visualization_msgs::msg::Marker angular_v_marker_;
    visualization_msgs::msg::Marker armors_marker_;
    visualization_msgs::msg::Marker selection_marker_;
    visualization_msgs::msg::Marker trajectory_marker_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    std::atomic<int> enable_shoot;    
                                          ///< 是否能发射，由行为树 
    builtin_interfaces::msg::Time header_stamp_;
    float palstance_;
    geometry_msgs::msg::Point center_;
    geometry_msgs::msg::Vector3 velocity_;
    float phase_;
    int armor_num_;
    std::vector<float> radius_;
    std::vector<float> height_;
    bool is_track_;
    std::string id_;
    int model_type_;
    std::vector<base_interfaces::msg::Armor> armors_;
    float time_off_;
    std_msgs::msg::Header header_;

   std::mutex status_mutex;
   bool valid_status_ = false;

private:
    /**
     * @brief 云台控制定时器回调函数
     */
    void control_callback();
    /**
     * @brief 大Yaw控制定时器回调函数
     */
    /**
     * @brief 模型接收回调函数
     * @param status_msg ekf模型
     */
void top_sb_(const base_interfaces::msg::KinematicStatus& status_msg);
    double getDistance1(const cv::Point3d& point);
    double getDistance2(const cv::Point2d& point);

    /**
     * @brief 获取src_frame至tar_frame的坐标变换
     * @param src_frame 源坐标系  
     * @param tar_frame 目标坐标系 
     * @param stamp     时间戳
     * @param transform 返回的坐标变化 
     * @return 是否成功进行坐标变化
     */
    

    /**
     * @brief 获取云台角度
     * @param target_position   目标装甲板位置
     * @param bullet_speed      子弹速度
     * @param gimbal_pose       返回的云台角度 
     * @return 是否成功获取云台角度
     */
    
    
    /**
     * @brief 获取装甲板
     * @param predict_time  预测时间 
     * @param number        返回的装甲板数量
     * @return 装甲板
     */
    

    /**
     * @brief 获取最近的装甲板位姿 
     * @param predict_time      预测时间
     * @param armor_pose_out    返回的装甲板位姿 
     * @return 是否成功获取装甲板位姿
     */
    bool getClosestArmorPose(double predict_time, geometry_msgs::msg::Pose &armor_pose_out);

    bool getFacingArmorPose(double predict_time, geometry_msgs::msg::Pose &armor_pose_out);
    
    /**
     * @brief 是否可以射击
     * @param target_armor_pose     目标装甲板位姿 
     * @param target_gimbal_pose    目标云台位姿 
     * @return 是否可以射击
     */
    bool shootable(geometry_msgs::msg::Pose target_armor_pose, base_interfaces::msg::GimbalPose target_gimbal_pose);

    void initMarkers();
    void publishMarkers(const geometry_msgs::msg::Pose& target_pose,
                               const base_interfaces::msg::GimbalPose& gimbal_cmd);
};
} //namespace HL