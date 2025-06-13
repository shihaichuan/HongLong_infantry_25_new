#include "aimer/Aimer.hpp"
#include <cmath>
#include <algorithm> 
#include <opencv2/imgproc.hpp>  
#include <chrono>
#include <tf2_ros/create_timer_ros.h>
#include <cv_bridge/cv_bridge.h>
using namespace std::chrono_literals;

#define _red(x)     "\033[31m" x "\033[0m"
#define _green(x)   "\033[32m" x "\033[0m"
#define _blue(x)    "\033[34m" x "\033[0m"
#define _purple(x)  "\033[35m" x "\033[0m"
#define _white(x)   "\033[37m" x "\033[0m"
#define D2R(deg) ((deg) * M_PI / 180.0)
#define R2D(rad) ((rad) * 180.0 / M_PI)
#define _lightred(str) 

namespace rm_auto_aim{
Aimer::Aimer(const rclcpp::NodeOptions & options)
: rclcpp::Node("rm_aimer", options)
{
    
tf2_buffer_ = std::make_unique<tf2_ros::Buffer>(
    this->get_clock(),
    tf2::Duration(30s)// 最大缓存时间
);
    tf2_listener_ = (std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_));
    
    m_debug = this->declare_parameter<bool>("debug", true);
    m_ekf_on = this->declare_parameter<bool>("ekf_on", true);
    m_track_center = this->declare_parameter<bool>("track_center", true);
    m_predict_on = this->declare_parameter<bool>("predict_on", true);
    pitch_off_ = this->declare_parameter<double>("Gimbal.pitch_off", 0.12);
    yaw_off_ = this->declare_parameter<double>("Gimbal.yaw_off", 0.2);
    m_time_off = this->declare_parameter<double>("time_off", 0.1);
    m_switch_threshold = this->declare_parameter<double>("switch_threshold", 15.0);
    m_init_pose_tolerance = this->declare_parameter<double>("init_pose_tolerance", 15.0);
    m_rubbish_data_tolerance = this->declare_parameter<double>("rubbish_data_tolerance", 50.0);
    m_force_aim_palstance_threshold = this->declare_parameter<double>("force_aim_palstance_threshold", 0.8);
    m_aim_angle_tolerance = this->declare_parameter<double>("aim_angle_tolerance", 4.0);
    m_aim_pose_tolerance = this->declare_parameter<double>("aim_pose_tolerance", 0.05);
    m_score_tolerance = this->declare_parameter<double>("score_tolerance", 1.0);
    m_all_white_tolerance_stop_shoot = this->declare_parameter<int>("all_white_tolerance_stop_shoot", 10);
    m_all_white_tolerance_reset = this->declare_parameter<int>("all_white_tolerance_reset", 100);
    m_aim_center_palstance_threshold = this->declare_parameter<double>("CenterTrack.aim_center_palstance_threshold", 6.5);
    m_switch_trackmode_threshold = this->declare_parameter<double>("CenterTrack.switch_trackmode_threshold", 2.0);
    m_aim_center_angle_tolerance = this->declare_parameter<double>("CenterTrack.aim_center_angle_tolerance", 1.0);

    m_observer_debug = this->declare_parameter<bool>("ModelObserver.debug", true);
    m_observer_dt = this->declare_parameter<double>("ModelObserver.dt", 0.01);
    m_standard_init_radius = this->declare_parameter<double>("ModelObserver.Standard.init_radius", 0.25);
    m_standard_gain = this->declare_parameter<double>("ModelObserver.Standard.gain", 15.0);
    m_standard_process_noise[0] = this->declare_parameter<double>("ModelObserver.Standard.process_noise.displace_high_diff", 3.0);
    m_standard_process_noise[1] = this->declare_parameter<double>("ModelObserver.Standard.process_noise.anglar_high_diff", 15.0);
    m_standard_process_noise[2] = this->declare_parameter<double>("ModelObserver.Standard.process_noise.height", 0.0001);
    m_standard_process_noise[3] = this->declare_parameter<double>("ModelObserver.Standard.process_noise.radius", 0.0001);
    m_standard_measure_noise[0] = this->declare_parameter<double>("ModelObserver.Standard.measure_noise.pose", 3.0);
    m_standard_measure_noise[1] = this->declare_parameter<double>("ModelObserver.Standard.measure_noise.distance", 0.1);
    m_standard_measure_noise[2] = this->declare_parameter<double>("ModelObserver.Standard.measure_noise.angle", 0.5);

    m_balance_init_radius = this->declare_parameter<double>("ModelObserver.Balance.init_radius", 0.25);
    m_balance_gain = this->declare_parameter<double>("ModelObserver.Balance.gain", 15.0);
    m_balance_process_noise[0] = this->declare_parameter<double>("ModelObserver.Balance.process_noise.displace_high_diff", 0.2);
    m_balance_process_noise[1] = this->declare_parameter<double>("ModelObserver.Balance.process_noise.anglar_high_diff", 10.0);
    m_balance_process_noise[2] = this->declare_parameter<double>("ModelObserver.Balance.process_noise.height", 0.0001);
    m_balance_process_noise[3] = this->declare_parameter<double>("ModelObserver.Balance.process_noise.radius", 0.0001);
    m_balance_measure_noise[0] = this->declare_parameter<double>("ModelObserver.Balance.measure_noise.pose", 3.0);
    m_balance_measure_noise[1] = this->declare_parameter<double>("ModelObserver.Balance.measure_noise.distance", 0.1);
    m_balance_measure_noise[2] = this->declare_parameter<double>("ModelObserver.Balance.measure_noise.angle", 0.1);

    top_pub_ = this->create_publisher<base_interfaces::msg::KinematicStatus>(
    "/Top_status", 
    rclcpp::SensorDataQoS());
    
    cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/camera_info", rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info) {
    cam_info_ =
        std::make_shared<sensor_msgs::msg::CameraInfo>(*camera_info);
    cam_info_sub_.reset();
    });
    img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/image_raw", rclcpp::SensorDataQoS(),
        std::bind(&Aimer::imageCallback, this,
                    std::placeholders::_1));

    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(),
    this->get_node_timers_interface());
    tf2_buffer_->setCreateTimerInterface(timer_interface);
    armors_sub_.subscribe(this, "/detector/armors", rmw_qos_profile_sensor_data);
    tf2_filter_ = std::make_shared<tf2_ros::MessageFilter<base_interfaces::msg::Armors>>(armors_sub_,
                                             *tf2_buffer_,
                                             base_link_,
                                             10,
                                             this->get_node_logging_interface(),
                                             this->get_node_clock_interface(),
                                             std::chrono::duration<int>(1));
    tf2_filter_->registerCallback(&Aimer::buildModel, this);
    timer_ptr_ = this->create_wall_timer(10ms, std::bind(&Aimer::ModelCallBack, this));
    yaw_pu_ = this->create_publisher<std_msgs::msg::Float64>("yaw", 10);//调试用
    x_pu_ = this->create_publisher<std_msgs::msg::Float64>("x", 10);//调试用


    reset();
}

 void Aimer::reset()
{
    m_tracking = false;
    m_enable_shoot = false;
    m_tracked_ID = "-1";
    m_type_init_cnt = 0;
    m_track_lost_cnt = 0;
    m_all_white_cnt = 0;
    m_model_type = KinematicModel::UNKNOWN;
    m_status = std::make_shared<StandardModel>();   // 只为初始化index，具体设为何种模型并无实际影响
    m_status->header.frame_id = "base_link";  // 默认坐标系
    if (m_model_observer)
    {
        m_model_observer->reset();
    }
    if (m_debug)
        std::cout << "[Aimer] Reset"<< std::endl;
}

void Aimer::buildModel(const base_interfaces::msg::Armors::SharedPtr armors_msg)//根据输入的装甲板数据，使用KinematicModel模型进行预测和匹配，并返回匹配后的结果。
{   
    base_interfaces::msg::KinematicStatus status_msg;
    base_interfaces::msg::GimbalPose cur_pose;
    auto gimbal_tf =
    tf2_buffer_->lookupTransform(base_link_, gimbal_link, tf2::TimePointZero);//获取当前云台姿态 rpy
    auto msg_q = gimbal_tf.transform.rotation;//云台四元数
    tf2::Quaternion tf_q;//定义四元数
    tf2::fromMsg(msg_q, tf_q);//ros消息四元数转tf2四元数
    tf2::Matrix3x3(tf_q).getRPY(rpy_[0], rpy_[1], rpy_[2]);//tf2四元数转rpy 提取欧辣角
    rpy_[1] = -rpy_[1];//俯仰角取反
    cur_pose.pitch = rpy_[1];
    cur_pose.yaw = rpy_[2];
    m_cur_pose = cur_pose;//当前云台姿态
    // std::cout<<"当前  m_cur_pose pitch  "<<m_cur_pose.pitch<<std::endl;
    //  std::cout<<" 当前  m_cur_pose  yaw  "<<m_cur_pose.yaw<<std::endl;
    // 初始化锁ID
    if (m_tracked_ID == "-1")
    {
        // 建立一个初始化序列，根据瞄准所需位姿和当前位姿的差值对所有识别到的装甲板正序排序
        auto& init_armors = armors_msg->armors; // 注意使用引用
        if (!init_armors.empty()) {
        }
        std::sort(init_armors.begin(), init_armors.end(),
    [this](base_interfaces::msg::Armor &a, base_interfaces::msg::Armor &b) {
        const auto a_pose = getAngle(a.pose, bullet_speed, m_cur_pose);
        const auto b_pose = getAngle(b.pose, bullet_speed, m_cur_pose);
        const auto a_diff = std::hypot(a_pose.pitch - m_cur_pose.pitch,
                                     a_pose.yaw - m_cur_pose.yaw);
        const auto b_diff = std::hypot(b_pose.pitch - m_cur_pose.pitch,
                                     b_pose.yaw - m_cur_pose.yaw);
        return a_diff < b_diff;
    }
    );
        // 在最接近当前瞄准的装甲板排序中，选择最优的非白色装甲板ID作为目标，且该目标与当前瞄准位置偏差在容忍范围内
        // 即防止直接瞄准死车，或因为近处目标丢识别而甩飞；但是当与死车同ID的活车出现时，还是可以锁上该ID
    for (auto &armor : init_armors) {
    const auto& target_pose = getAngle(armor.pose, bullet_speed, m_cur_pose);
    const float delta_pitch = target_pose.pitch - m_cur_pose.pitch;
    const float delta_yaw = target_pose.yaw - m_cur_pose.yaw;
    const float angle_diff = std::hypot(delta_pitch, delta_yaw);
    if (angle_diff < m_init_pose_tolerance)
                    {
                        m_tracked_ID = armor.number;
                        break;
                    }
        }
    }
    // 如果ID未初始化则直接返回当前位姿

    status_msg.is_track = true;
    //top_pub_->publish(status_msg);

    if (m_debug)
    std::cout << "[Aimer] id: " << m_tracked_ID << std::endl;
    base_interfaces::msg::Armors target_armor_seq;        
    for (auto &armor : armors_msg->armors)
    {
        // 将与目标ID相同、目标大小相同且角度在可接受范围内的装甲板填入序列中但仍是相机系
        if (armor.number == m_tracked_ID && abs(R2D(armor.yaw_angle)) < m_rubbish_data_tolerance
            && ((armor.type == "large" &&  m_model_type == KinematicModel::BALANCE)
            || (armor.type == "small"  &&  m_model_type != KinematicModel::BALANCE)
            || (armor.type == "large"  && m_tracked_ID == "1" && KinematicModel::STANDARD)
            || (m_model_type == KinematicModel::UNKNOWN)))
        {
            target_armor_seq.armors.push_back(armor); 
            // auto color = static_cast<rm_auto_aim::ArmorColor>(armor.m_color);
            // if (color != rm_auto_aim::ArmorColor::WHITE)
            // {
            //     all_white = false;
            // }
        }
    }
    // 判断当前序列是否全部为白色装甲板
    if (target_armor_seq.armors.size()==0)
    {
        m_all_white_cnt++;
        std::cout << _red("[Aimer] all white !") << std::endl;
    }
    else
    {
        m_all_white_cnt = 0;
    }
    
    // 若连续多帧没有识别到目标ID下亮着的装甲板，则认为目标丢失，自动重置
    if (m_all_white_cnt > m_all_white_tolerance_reset)
    {
        reset();
    }
    // if (m_debug)

    // 判断模型类型
    
    // std::cout<<"m_type_init_cnt "<<m_type_init_cnt<<std::endl;
    if (m_type_init_cnt < 3)
    {
        if (target_armor_seq.armors.size() > 0)
        {

            base_interfaces::msg::Armor armor = target_armor_seq.armors[0];
            // 英雄、非平衡步兵与哨兵
            if ((armor.number == "1") ||
                (armor.number == "2" ))
            {
                if ( m_model_type != KinematicModel::STANDARD)
                    m_type_init_cnt = 0;
                 m_model_type = KinematicModel::STANDARD;
                m_type_init_cnt++;
                status_msg.model_type =  m_model_type;
                id = armor.number; 
            }
            // 平衡步兵
            else if (armor.number  == "3" || armor.number == "5" || armor.number == "4")
            {
                if ( m_model_type != KinematicModel::BALANCE)
                    m_type_init_cnt = 0;
                 m_model_type = KinematicModel::BALANCE;
                m_type_init_cnt++;
                status_msg. model_type =  m_model_type;
                id= armor.number;
            }
        }
        if (m_type_init_cnt >= 1)
        {
            setModelType( m_model_type);  
        }
        else
        {
            // 未完成类别判断，返回当前位姿
        }
        
    }
    status_msg.is_track = true;
    // 异常数据检查
    errorHandling(target_armor_seq);
    // 根据瞄准所需位姿和当前位姿的差值对目标ID的装甲板正序排序
    auto& target_armor_seq_armors = target_armor_seq.armors;
    std::sort(target_armor_seq_armors.begin(), target_armor_seq_armors.end(),
    [this](base_interfaces::msg::Armor &a, base_interfaces::msg::Armor &b) {
        const auto a_pose = getAngle(a.pose, bullet_speed, m_cur_pose);
        const auto b_pose = getAngle(b.pose, bullet_speed, m_cur_pose);
        
        const auto a_diff = std::hypot(a_pose.pitch - m_cur_pose.pitch,
                                     a_pose.yaw - m_cur_pose.yaw);
        const auto b_diff = std::hypot(b_pose.pitch - m_cur_pose.pitch,
                                     b_pose.yaw - m_cur_pose.yaw);
        return a_diff < b_diff;
    }
);
    // 将目标装甲板序列的坐标和角度转为绝对系
    for (auto &armor : target_armor_seq.armors)
    {
        geometry_msgs::msg::PoseStamped ps;
        ps.header.frame_id = camera_optical_link;
        ps.pose = armor.pose;
        try {
        armor.pose = tf2_buffer_->transform(ps, base_link_).pose;
        std::cout << "转换到base_link后的装甲板坐标 - X: " << armor.pose.position.x 
                    << " Y: " << armor.pose.position.y 
                    << " Z: " << armor.pose.position.z << std::endl;
        tf2::Quaternion q(
                armor.pose.orientation.x,
                armor.pose.orientation.y,
                armor.pose.orientation.z,
                armor.pose.orientation.w);            
            // 计算相对于绝对系的偏航角
            double roll, pitch, yaw;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
            
            // 结合云台当前姿态补偿
            //yaw = yaw;
            double yaw_ = _std_radian(m_cur_pose.yaw +  yaw);
            // 更新装甲板角度信息
            armor.yaw_angle = yaw_;
            // auto aw_msg = std::make_shared<std_msgs::msg::Float64>();
            //         aw_msg->data = armor.yaw_angle; 
            //         x_pu_->publish(*aw_msg);//可视化调试
            // std::cout<<" yaw "<<armor.yaw_angle<<std::endl;

            
        } catch (const tf2::TransformException &ex) {
         std::cout<<"armor_solverTransform error:"<<std::endl;;
        return;
        }
            // 角度转换：从四元数提取绝对系偏航角
            
        }
        processed_armor_seq_ = target_armor_seq; 
        status_msg.armors = target_armor_seq.armors;
}

void Aimer::imageCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr &img_msg) {
  try {
  rclcpp::Time target_time = img_msg->header.stamp;
  auto base_link_to_gimbal = tf2_buffer_->lookupTransform(
      base_link_, img_msg->header.frame_id, target_time,
      rclcpp::Duration::from_seconds(0.01));
  auto msg_q = base_link_to_gimbal.transform.rotation;
  tf2::Quaternion tf_q;
  tf2::fromMsg(msg_q, tf_q);
  } catch (...) {
     RCLCPP_ERROR(this->get_logger(), "Something Wrong when lookUpTransform");
    return;
  }
  auto img = cv_bridge::toCvShare(img_msg, "rgb8")->image;
      aimer_img_pub_ =
    image_transport::create_publisher(this, "/detector/aimer_img");
        aimer_img_pub_.publish(
    cv_bridge::CvImage(img_msg->header, "rgb8", img).toImageMsg());
  }

void Aimer::ModelCallBack()
{
    base_interfaces::msg::KinematicStatus status_msg;
    status_msg.header.frame_id = m_status->header.frame_id = base_link_;  // 传递坐标系
    status_msg.header.stamp = m_status->header.stamp;        // 传递时间戳
    status_msg.velocity.x = 0.0;  
    status_msg.model_type = 0; // STANDARD模型
    status_msg.radius = std::vector<float>{0.3, 0.3}; // 两个半径
    status_msg.height = std::vector<float>{0.15, 0.15}; // 两个高度
    status_msg.palstance = 0.0;
    if(processed_armor_seq_.armors.size()>0)
    {
        if (m_ekf_on == 1)
        {
            // 使用EKF进行建模
            m_status = resolve(processed_armor_seq_);
            bool valid_data = false;
            if(m_model_type == 0) {
                // 检查STANDARD模型的radius和height
                if(m_status->radius[0] == 0 || m_status->radius[1] == 0 ||
                    m_status->height[0] == 0 || m_status->height[1] == 0) {
                    valid_data = false;
                    RCLCPP_WARN(this->get_logger(), "Invalid data: radius or height is 0");
                }
            } else if(m_model_type == KinematicModel::BALANCE) {
                // 检查BALANCE模型的radius和height
                if(m_status->radius[0] == 0 || m_status->height[0] == 0) {
                    valid_data = false;
                    RCLCPP_WARN(this->get_logger(), "Invalid data: radius or height is 0");
                }
            }
            if(valid_data == false) {
                if(m_model_type == 0){
                    status_msg.header = m_status->header;
                    m_status->header.stamp = this->now();
                    status_msg.header.stamp = m_status->header.stamp;
                    status_msg.palstance = m_status->palstance;
                    status_msg.center = m_status->center;
                    status_msg.velocity.x = m_status->velocity.x;  // 赋值 x 分量
                    status_msg.velocity.y = m_status->velocity.y;  // 赋值 y 分量
                    status_msg.phase = _std_radian(m_status->phase);
                    auto yaw_msg = std::make_shared<std_msgs::msg::Float64>();
                    yaw_msg->data = status_msg.phase; 
                    yaw_pu_->publish(*yaw_msg);//可视化调试
                    status_msg.armor_num = m_status->number;
                    status_msg.radius = std::vector<float>(2);
                    status_msg.radius[0] = m_status->radius[0];
                    status_msg.radius[1] = m_status->radius[1];
                    status_msg.height = std::vector<float>(2);
                    status_msg.height[0] = m_status->height[0];
                    status_msg.height[1] = m_status->height[1];
                    status_msg.is_track = true;
                    status_msg.model_type =  m_model_type;
                    status_msg.id= id;
                    //  std::cout<<"centerx"<< m_status->center.x<<std::endl;
                    // // std::cout<<"velocity.x  "<<m_status->velocity.x<<std::endl;
                    //  std::cout<<"centery"<< m_status->center.y<<std::endl;
                    // std::cout<<"velocity.y  "<<m_status->velocity.y<<std::endl;
                    //std::cout<<".height[0]"<<m_status->height[0]<<std::endl;
                    //std::cout<<"height[1]"<<m_status->height[1]<<std::endl;
                    // std::cout<<"radius[0]"<<m_status->radius[0]<<std::endl;
                    // std::cout<<"radius[1]"<<m_status->radius[1]<<std::endl;
                    // std::cout<<"phase"<<m_status->phase<<std::endl;
                    // std::cout<<"palstance"<<m_status->palstance<<std::endl;
                    top_pub_->publish(status_msg);
                    

                }
                else if( m_model_type == KinematicModel::BALANCE){
                    status_msg.header.stamp = m_status->header.stamp;
                    status_msg.palstance = m_status->palstance;
                    status_msg.center = m_status->center;
                    status_msg.velocity.x = m_status->velocity.x;  // 赋值 x 分量
                    status_msg.velocity.y = m_status->velocity.y;  // 赋值 y 分量
                    status_msg.phase = m_status->phase;
                    status_msg.armor_num = m_status->number;
                    status_msg.radius[0] = m_status->radius[0];
                    status_msg.height[0] = m_status->height[0];//打平衡步兵
                    top_pub_->publish(status_msg);
                }
                else
                { 
                m_status->armors = processed_armor_seq_;//直接将目标装甲板序列赋值给m_status
                top_pub_->publish(status_msg);
                }
            }
        }
    }
}

base_interfaces::msg::GimbalPose Aimer::getAngle(geometry_msgs::msg::Pose target_armor_pose, float bullet_speed, base_interfaces::msg::GimbalPose &gimbal_pose)
{
    base_interfaces::msg::GimbalPose solve_angle;
    solve_angle.pitch = -std::atan(target_armor_pose.position.z / std::sqrt(target_armor_pose.position.x * target_armor_pose.position.x + target_armor_pose.position.y * target_armor_pose.position.y));
    solve_angle.yaw = std::atan2(target_armor_pose.position.y, target_armor_pose.position.x);
    if (m_debug)
    {
        // this->get_parameter<float>("Gimbal.pitch_off", pitch_off_);
        // this->get_parameter<float>("Gimbal.yaw_off", yaw_off_);
    }
    solve_angle.pitch += pitch_off_;
    solve_angle.yaw += yaw_off_;
    
    if (m_debug)
    {
        // this->get_parameter<bool>("Gimbal.fix_on", fix_on_);
    }
    if(fix_on_)
    {
        double dis = sqrt(target_armor_pose.position.x * target_armor_pose.position.x + target_armor_pose.position.z * target_armor_pose.position.z);
        double angle_fix = 0;
        if (abs(9.8 * dis * pow(cos(solve_angle.pitch), 2)) / pow(bullet_speed, 2) - sin(solve_angle.pitch) <= 1)
        {
            angle_fix = 0.5 * (asin((9.8 * dis * pow(cos(solve_angle.pitch), 2)) / pow(bullet_speed, 2) - sin(solve_angle.pitch)) + solve_angle.pitch);
        }
        solve_angle.pitch -= angle_fix;
    }
    return solve_angle;
}//计算角度

bool Aimer::getTransform(std::string src_frame, std::string tar_frame,  builtin_interfaces::msg::Time stamp, geometry_msgs::msg::TransformStamped &transform)
{
    try {
        transform = tf2_buffer_->lookupTransform(
                 src_frame,tar_frame,
                stamp,
                rclcpp::Duration::from_nanoseconds(50000));
    }
    catch (const tf2::TransformException & ex) {
        RCLCPP_INFO(
            this->get_logger(), "Could not transform %s to %s: %s",
            tar_frame.c_str(), src_frame.c_str(), ex.what());
        return false;
    }
    return true;
}//获取坐标变换

std::vector<base_interfaces::msg::Armor> Aimer::getArmors(double predict_time, int number)
{
        std::cout<<"number "<<m_status->number<<std::endl;

    std::vector<base_interfaces::msg::Armor> armors(number);
    if (number == 4)    // STANDARD模型 四块装甲板
    {
        for (int i = 0; i < number; i++)
        {
            geometry_msgs::msg::Point center;
            // 装甲板yaw角，绝对系
            double angle = _std_radian(m_status->phase + m_status->palstance * predict_time + i * M_PI / 2);
            cv::Point2d point = cv::Point2d(m_status->center.x, m_status->center.y) + 
                                cv::Point2d(m_status->velocity.x, m_status->velocity.y) * predict_time + 
                                cv::Point2d(m_status->radius[i % 2] * cos(angle), m_status->radius[i % 2] * sin(angle));
            armors[i].pose.position.x = point.x;
            armors[i].pose.position.y = point.y;
            armors[i].pose.position.z = m_status->height[i % 2];
            // 将yaw角转四元数
            tf2::Quaternion q;
            q.setRPY(0, 0, angle);
            armors[i].pose.orientation.x = q.x();
            armors[i].pose.orientation.y = q.y();
            armors[i].pose.orientation.z = q.z();
            armors[i].pose.orientation.w = q.w();
            armors[i].yaw_angle = angle;
        }
    }
    else if (number == 2)   //  BALANCE模型 两块装甲板
    {
        for (int i = 0; i < number; i++)
        {
            // 装甲板yaw角，绝对系
            double angle = _std_radian(m_status->phase + m_status->palstance * predict_time + i * M_PI);
            cv::Point2d point = cv::Point2d(m_status->center.x, m_status->center.y) +
                                cv::Point2d(m_status->velocity.x * cos(m_status->phase) * predict_time,
                                            m_status->velocity.x * sin(m_status->phase) * predict_time) +
                                cv::Point2d(m_status->radius[0] * cos(angle), m_status->radius[0] * sin(angle));
            armors[i].pose.position.x = point.x;
            armors[i].pose.position.y = point.y;
            armors[i].pose.position.z = m_status->height[0];
            // 将yaw角转四元数
            tf2::Quaternion q;
            q.setRPY(0, 0, angle);
            armors[i].pose.orientation.x = q.x();
            armors[i].pose.orientation.y = q.y();
            armors[i].pose.orientation.z = q.z();
            armors[i].pose.orientation.w = q.w();
            armors[i].yaw_angle = angle;
        }
    }
    return armors;
}//预测时间后装甲板位姿

std::shared_ptr<KinematicModel> Aimer::resolve(base_interfaces::msg::Armors &armors)
{    
    std::pair<Eigen::MatrixXd, Eigen::MatrixXd> prior = m_model_observer->predict(armors);//对输入的装甲板进行预测
    std::shared_ptr<KinematicModel> prior_status;
    if ( m_model_type == KinematicModel::STANDARD)
    {
        prior_status = std::make_shared<StandardModel>(prior.first);
    }
    else if ( m_model_type == KinematicModel::BALANCE)
    {
        prior_status = std::make_shared<BalanceModel>(prior.first);
    }//根据成员变量model_type的值，创建不同类型的KinematicModel对象，并将其指针赋给prior_status
    prior_status->header.frame_id = "base_link";  // 明确设置坐标系
    prior_status->header.stamp = this->now();    // 设置时间戳
    base_interfaces::msg::Armors standard_armors_msg;
    standard_armors_msg.armors = getArmors(0, prior_status->number);

    Eigen::MatrixXd score = getScoreMat(armors, standard_armors_msg);
    //if (m_debug)
        // std::cout << "score: " << std::endl << score << std::endl;//获取评分矩阵，用于后续的匹配操作
 std::map<int, int> armor_match = m_hungarian_solver.solve(score, m_score_tolerance, prior_status->number);                   
// std::cout<<"std::map<int, int> armor_match "<< armor_match.size() <<std::endl;
    std::shared_ptr<KinematicModel> posterior_status = m_model_observer->update(armors.armors, prior.first, prior.second, armor_match);


    return posterior_status;//使用m_match对象的getMatch方法，根据得分矩阵score、得分容忍度m_score_tolerance和先验状态的装甲板数量prior_status->number，得到装甲板的匹配结果armor_match。
}
bool Aimer::isReady()
{
    return m_tracking;
}
void Aimer::errorHandling(base_interfaces::msg::Armors& armors)
{
    for (auto it = armors.armors.begin(); it != armors.armors.end();)
    {
        if (isinf(it->pose.position.x) || isnan(it->pose.position.x) ||
            isinf(it->pose.position.y) || isnan(it->pose.position.y) ||
            isinf(it->pose.position.z) || isnan(it->pose.position.z) ||
            isinf(it->yaw_angle)  || isnan(it->yaw_angle)
            )
        {
            // std::cout << _lightred("[Aimer] Error data: inf or nan") << std::endl;
            it = armors.armors.erase(it);
        }
        else
        {
            ++it;
        }

    }
}//对输入的装甲板数据进行错误处理，删除包含inf或nan的装甲板

// 新增时间偏差估计成员
double Aimer::estimateTimeDelay(const base_interfaces::msg::Armors& armors)
{
    static std::deque<double> time_diffs;
    if(armors.armors.size() > 0){
        const auto& stamp = armors.armors[0].header.stamp;
        double armor_time = stamp.sec + 1e-9 * stamp.nanosec; // 转换为秒
        double current_time = now().seconds(); 

        // 计算时间差
        double diff = current_time - armor_time;
        time_diffs.push_back(diff);
        if(time_diffs.size() > 3){
            time_diffs.pop_front();
        }
    }
    return time_diffs.empty() ? 0.0 : std::accumulate(time_diffs.begin(), time_diffs.end(), 0.0) / time_diffs.size();  
}


void Aimer::setModelType(KinematicModel::Type type)
{
     m_model_type = type;
    // 大装甲板击发位姿增大
    if (m_tracked_ID == "1" || type == KinematicModel::BALANCE)
        m_aim_pose_tolerance *= 1.5;

    if (type == KinematicModel::STANDARD) {
        std::array<double,4> process_noise = {
            m_standard_process_noise[0],
            m_standard_process_noise[1],
            m_standard_process_noise[2],
            m_standard_process_noise[3]
        };
        
        std::array<double,3> measure_noise = {
            m_standard_measure_noise[0],
            m_standard_measure_noise[1],
            m_standard_measure_noise[2]
        };

        m_model_observer = std::make_shared<StandardObserver>(
            m_observer_debug,
            m_observer_dt,
            m_standard_init_radius,
            m_standard_gain,
            process_noise,
            measure_noise
        );
    }
    else if (type == KinematicModel::BALANCE)
    {
        std::array<double,4> process_noise = {
            m_balance_process_noise[0],
            m_balance_process_noise[1],
            m_balance_process_noise[2],
            m_balance_process_noise[3]
        };
        
        std::array<double,3> measure_noise = {
            m_balance_measure_noise[0],
            m_balance_measure_noise[1],
            m_balance_measure_noise[2]
        };

        m_model_observer = std::make_shared<StandardObserver>(
            m_observer_debug,
            m_observer_dt,
            m_balance_init_radius,
            m_balance_gain,
            process_noise,
            measure_noise
        );
    }
}

Eigen::MatrixXd Aimer::getScoreMat(base_interfaces::msg::Armors &detect_armors, 
                                  base_interfaces::msg::Armors &standard_armors)
{
    // 使用size_t避免符号不匹配警告
    const size_t m = detect_armors.armors.size();
    const size_t n = standard_armors.armors.size();
    // 计算两组装甲板之间的坐标差和角度差
    Eigen::Matrix<double, Eigen::Dynamic, 2> negative_score(m * n, 2);
    
    for (size_t i = 0; i < m; ++i) {
        for (size_t j = 0; j < n; ++j) {
            // 计算位置距离（使用Point的x,y,z分量分别计算）
            const auto& pos1 = detect_armors.armors[i].pose.position;
            const auto& pos2 = standard_armors.armors[j].pose.position;
            const double dx = pos1.x - pos2.x;
            const double dy = pos1.y - pos2.y;
            const double dz = pos1.z - pos2.z;
            negative_score(i * n + j, 0) = std::sqrt(dx*dx + dy*dy + dz*dz);
            
            // 计算角度差（直接访问yaw_angle成员）
            negative_score(i * n + j, 1) = std::abs(
                _std_radian(detect_armors.armors[i].yaw_angle - 
                           standard_armors.armors[j].yaw_angle)
            );
        }
    }
    // 数据标准化
    Eigen::Matrix<double, Eigen::Dynamic, 2> regular_score(m * n, 2);
    const auto max0 = negative_score.col(0).maxCoeff();
    const auto min0 = negative_score.col(0).minCoeff();
    const auto max1 = negative_score.col(1).maxCoeff();
    const auto min1 = negative_score.col(1).minCoeff();
    
    for (size_t i = 0; i < m * n; ++i) {
        regular_score(i, 0) = (max0 - negative_score(i, 0)) / (max0 - min0 + 1e-6);
        regular_score(i, 1) = (max1 - negative_score(i, 1)) / (max1 - min1 + 1e-6);
    }

    // 计算样本值占指标的比重
    Eigen::Matrix<double, Eigen::Dynamic, 2> score_weight(m * n, 2);
    const Eigen::Vector2d col_sum = regular_score.colwise().sum();
    
    for (size_t i = 0; i < m * n; ++i) {
        score_weight(i, 0) = regular_score(i, 0) / (col_sum(0) + 1e-6);
        score_weight(i, 1) = regular_score(i, 1) / (col_sum(1) + 1e-6);
    }

    // 计算每项指标的熵值
    Eigen::Vector2d entropy = Eigen::Vector2d::Zero();
    for (size_t i = 0; i < m * n; ++i) {
        if (score_weight(i, 0) > 1e-6)
            entropy(0) -= score_weight(i, 0) * std::log(score_weight(i, 0));
        if (score_weight(i, 1) > 1e-6)
            entropy(1) -= score_weight(i, 1) * std::log(score_weight(i, 1));
    }
    entropy /= std::log(static_cast<double>(m * n));

    // 计算权重
    const Eigen::Vector2d weight = (Eigen::Vector2d::Ones() - entropy).array() / 
                                  (2.0 - entropy.sum());
    // 计算最终匹配得分矩阵
    Eigen::MatrixXd score(m, n);
    for (size_t i = 0; i < m; ++i) {
        for (size_t j = 0; j < n; ++j) {
            score(i, j) = negative_score.row(i * n + j) * weight;
        }
    }
    return score; // 确保有返回值
}

std::map<int, int> Aimer::HungarianSolver::solve(
    const Eigen::MatrixXd& cost_matrix,
    double score_max,
    int m) 
{
    // 初始化原始维度
    original_rows_ = cost_matrix.rows();
    original_cols_ = cost_matrix.cols();
    // std::cout<<"cost_matrix.rows() "<<cost_matrix.rows()<<std::endl;
    // std::cout<<"cost_matrix.cols() "<<cost_matrix.cols()<<std::endl;

    dim_ = std::max(original_rows_, original_cols_);
    // 矩阵扩展
    cost_matrix_ = cost_matrix;
    cost_matrix_.conservativeResize(dim_, dim_);
    
    const double INVALID_VALUE = score_max * 2 + 1000;
    if (original_cols_ < dim_) {
        cost_matrix_.block(0, original_cols_, dim_, dim_ - original_cols_)
            .setConstant(INVALID_VALUE);
    }
    if (original_rows_ < dim_) {
        cost_matrix_.block(original_rows_, 0, dim_ - original_rows_, dim_)
            .setConstant(INVALID_VALUE);
    }

    // 初始化算法数据结构
    label_x_ = std::vector<double>(dim_, 0);
    label_y_ = std::vector<double>(dim_, 0);
    match_x_ = std::vector<int>(dim_, -1);
    match_y_ = std::vector<int>(dim_, -1);
    // 初始化X部标号
    for (int u = 0; u < dim_; ++u) {
        label_x_[u] = *std::max_element(
            cost_matrix_.row(u).data(), 
            cost_matrix_.row(u).data() + dim_
        );
    }
    // 主算法流程
    for (int u = 0; u < dim_; ++u) {
        while (true) {
            visited_x_.assign(dim_, false);
            visited_y_.assign(dim_, false);
            if (findAugmentingPath(u)) break;
            updateLabels();
        }
    }
    // 收集有效结果
    std::map<int, int> result;
    for (int u = 0; u < original_rows_; ++u) {
        int v = match_x_[u];

        if (v < original_cols_ && cost_matrix_(u, v) <= score_max) {
            result[u] = v;
        }
    }
    // 数量约束处理
    if (m > 0 && result.size() > static_cast<size_t>(m)) {
        std::vector<std::pair<double, int>> sorted_pairs;
        for (const auto& pair : result) {
            sorted_pairs.emplace_back(cost_matrix_(pair.first, pair.second), pair.first);
        }
        std::sort(sorted_pairs.begin(), sorted_pairs.end());
        
        result.clear();
        for (size_t i = 0; i < std::min(static_cast<size_t>(m), sorted_pairs.size()); ++i) {
            result[sorted_pairs[i].second] = match_x_[sorted_pairs[i].second];
        }
    }

    return result;
}//这个函数可以用于在二维矩阵中寻找最优的行列匹配，使得匹配的值最小。具体来说，它首先计算了所有可能的行和列的组合，然后计算了每种组合的匹配值，并选择匹配值最小的组合作为最优匹配。

bool Aimer::HungarianSolver::findAugmentingPath(int u) {
    visited_x_[u] = true;
    for (int v = 0; v < original_cols_; ++v) {
        const double eps = calculateEpsilon(u, v);
        if (!visited_y_[v] && 
            std::abs(label_x_[u] + label_y_[v] - cost_matrix_(u, v)) < eps) 
        {
            visited_y_[v] = true;
            if (match_y_[v] == -1 || findAugmentingPath(match_y_[v])) {
                match_x_[u] = v;
                match_y_[v] = u;
                return true;
            }
        }
    }
    return false;
}

void Aimer::HungarianSolver::updateLabels() {
    double delta = std::numeric_limits<double>::max();
    
    // 计算最小delta
    for (int u = 0; u < original_rows_; ++u) {
        if (visited_x_[u]) {
            for (int v = 0; v < original_cols_; ++v) {
                if (!visited_y_[v]) {
                    const double diff = label_x_[u] + label_y_[v] - cost_matrix_(u, v);
                    if (diff > 1e-9) {
                        delta = std::min(delta, diff);
                    }
                }
            }
        }
    }
    // 更新标号
    for (int u = 0; u < dim_; ++u) if (visited_x_[u]) label_x_[u] -= delta;
    for (int v = 0; v < dim_; ++v) if (visited_y_[v]) label_y_[v] += delta;
}

double Aimer::HungarianSolver::calculateEpsilon(int u, int v) const {
    return 1e-6 * std::max(1.0, std::abs(cost_matrix_(u, v)));
}

// void Aimer::setROI(std::shared_ptr<KinematicModel> status, const Armors &armors)
// {
//     cv::Rect2d roi;
//     double m_roi_value = m_roi_params->m_roi_lost_mul[m_track_lost_cnt + 1];
//         if (armors.empty())
//         {
//             if (m_track_lost_cnt > m_roi_params->m_max_track_lost)
//             {
//                 // roi返回为中央roi
//                 m_return_roi_left = m_roi_params->m_camera_resolution;
//                 m_return_roi_right =  m_roi_params->m_camera_resolution;
//                 return;
//             }
//             else
//             {
//                 m_track_lost_cnt++;
//             }
//         }
//         else
//         {
//             m_track_lost_cnt = 0;
//         }
//         // 反陀螺和常规模式，则返回一个全车装甲板的矩形roi区域
//         Armors pre_armors = status->getArmors();
//         std::vector <cv::Point2f> reprojection_points;
//         if(getTransform(camera_link_, base_link_, m_status->header.stamp, t))   // 获取绝对系到相机系坐标变化
//         {
//             tf2::doTransform(armor.pose.position.position, armor.pose.position.position, t);    // 将装甲板位姿从绝对系转到枪管
//         }
//         for (auto &armor : armors)
//         {
//             reprojection_points.emplace_back(getReProjectPoint(armor.pose.position.position));
//         }
//         for (auto &armor : pre_armors)
//         {
//             reprojection_points.emplace_back(getReProjectPoint(armor.pose.position.position));
//         }
//         if(getTransform(camera_link_, base_link_, m_status->header.stamp, t))   // 获取绝对系到相机系坐标变化
//         {
//             tf2::doTransform(cv::Point3f(status->center.x, status->center.y, status->height[0]), cv::Point3f(status->center.x, status->center.y, status->height[0]), t);    // 将装甲板位姿从绝对系转到电机系
//         }
//         reprojection_points.emplace_back(getReProjectPoint(cv::Point3f(status->center.x, status->center.y, status->height[0])));

//         float max_x, min_x, max_y, min_y;
//         max_x = min_x = reprojection_points[0].x;
//         max_y = min_y = reprojection_points[0].y;
//         for(auto &point : reprojection_points)
//         {
//             max_x = max_x > point.x ? max_x : point.x;
//             min_x = min_x < point.x ? min_x : point.x;
//             max_y = max_y > point.y ? max_y : point.y;
//             min_y = min_y < point.y ? min_y : point.y;
//         }

//         float height,width;
//         height = max_y - min_y;
//         height = height > m_min_roi_height ? height : m_min_roi_height;
//         width = max_x - min_x;
//         roi = cv::Rect2d(
//             min_x - (m_roi_width_zoom_rate - 1) * 0.5 * width,
//             min_y - (m_roi_height_zoom_rate - 1) * 0.5 * height,
//             width * m_roi_width_zoom_rate,
//             height * m_roi_height_zoom_rate
//         );
//         roi &= m_roi_params->m_camera_resolution;

//         if (roi.area() == 0)
//         {
//             roi = m_roi_params->m_camera_resolution;
//         }

//     return;
// }

// void Aimer::setDeepROISize(cv::Size2i deep_roi_size)
// {
//     m_roi_params->m_deep_roi_size = deep_roi_size;
//     double rate = (double)deep_roi_size.height / (double)deep_roi_size.width;
//     m_roi_params->m_deep_default_roi = cv::Rect2d(
//         0,
//         (m_roi_params->m_camera_resolution.height - m_roi_params->m_camera_resolution.width * rate) / 2,
//         m_roi_params->m_camera_resolution.width,
//         (m_roi_params->m_camera_resolution.width * rate)
//     );
//     m_return_roi_left = m_roi_params->m_deep_default_roi;
//     m_return_roi_right = m_roi_params->m_deep_default_roi;
// }

//在输出的相机图像上将预测到的装甲板的位置画出来

}
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::Aimer)