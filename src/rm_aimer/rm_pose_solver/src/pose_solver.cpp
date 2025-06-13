#include <pose_solver/pose_solver.hpp>
using namespace std::chrono_literals;

namespace rm_auto_aim{
PoseSolver::PoseSolver(const rclcpp::NodeOptions & options) 
    : Node("pose_solver", options)
{    
    //不同模型装甲板数量，STANDARD为4个装甲板，BALANCE为2个装甲板
    model_armors[0] = 4;
    model_armors[1] = 2;

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock(), tf2::Duration(3s));
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    enable_shoot = true;

    radius_ = std::vector<float>(3);
    height_ = std::vector<float>(3);

    this->declare_parameter<bool>("debug", debug);
    this->declare_parameter<float>("timeout", m_timeout);
    this->declare_parameter<double>("shoot_interval", shoot_interval);
    this->declare_parameter<std::string>("Gimbal.shooter_link", shooter_link_);
    this->declare_parameter<std::string>("Gimbal.gimbal_link", gimbal_link);
    this->declare_parameter<std::string>("Gimbal.base_link", base_link_);
    this->declare_parameter<bool>("Gimbal.fix_on", fix_on_);
    this->declare_parameter<bool>("Gimbal.track_center", m_track_center);
    this->declare_parameter<float>("Gimbal.pitch_off", pitch_off_);
    this->declare_parameter<float>("Gimbal.yaw_off", yaw_off_);
    this->declare_parameter<float>("Aimer.switch_threshold", m_switch_threshold);
    this->declare_parameter<float>("Aimer.time_off", m_time_off);
    this->declare_parameter<float>("Aimer.force_aim_palstance_threshold", m_force_aim_palstance_threshold);
    this->declare_parameter<float>("Aimer.aim_angle_tolerance", m_aim_angle_tolerance);
    this->declare_parameter<float>("Aimer.aim_pose_tolerance", m_aim_pose_tolerance);
    this->declare_parameter<float>("Aimer.CenterTrack.aim_center_palstance_threshold", m_aim_center_palstance_threshold);
    this->declare_parameter<float>("Aimer.CenterTrack.switch_trackmode_threshold", m_switch_trackmode_threshold);
    this->declare_parameter<float>("Aimer.CenterTrack.aim_center_angle_tolerance", m_aim_center_angle_tolerance);
    //this->declare_parameter<int>("Aimer.all_white_tolerance_stop_shoot", all_white_tolerance_stop_shoot_);

    this->get_parameter<bool>("debug", debug);
    this->get_parameter<float>("timeout", m_timeout);
    this->get_parameter<double>("shoot_interval", shoot_interval);
    this->get_parameter<std::string>("Gimbal.shooter_link", shooter_link_);
    this->get_parameter<std::string>("Gimbal.gimbal_link", gimbal_link);
    this->get_parameter<std::string>("Gimbal.base_link", base_link_);
    this->get_parameter<bool>("Gimbal.fix_on", fix_on_);
    this->get_parameter<bool>("Gimbal.track_center", m_track_center);
    this->get_parameter<float>("Gimbal.pitch_off", pitch_off_);
    this->get_parameter<float>("Gimbal.yaw_off", yaw_off_);
    this->get_parameter<float>("Aimer.switch_threshold", m_switch_threshold);
    this->get_parameter<float>("Aimer.time_off", m_time_off);
    this->get_parameter<float>("Aimer.force_aim_palstance_threshold", m_force_aim_palstance_threshold);
    this->get_parameter<float>("Aimer.aim_angle_tolerance", m_aim_angle_tolerance);
    this->get_parameter<float>("Aimer.aim_pose_tolerance", m_aim_pose_tolerance);
    this->get_parameter<float>("Aimer.CenterTrack.aim_center_palstance_threshold", m_aim_center_palstance_threshold);
    this->get_parameter<float>("Aimer.CenterTrack.m_switch_trackmode_threshold", m_switch_trackmode_threshold);
    this->get_parameter<float>("Aimer.CenterTrack.aim_center_angle_tolerance", m_aim_center_angle_tolerance);
    //this->get_parameter<int>("Aimer.all_white_tolerance_stop_shoot", all_white_tolerance_stop_shoot_);

    top_sub_ = this->create_subscription<base_interfaces::msg::KinematicStatus>("/Top_status",rclcpp::SensorDataQoS(),std::bind(&PoseSolver::top_sb_, this, std::placeholders::_1));

    Gimbal_pose_pub_ = this->create_publisher<base_interfaces::msg::GimbalPose>(
                "setGimbalAngle", rclcpp::QoS(rclcpp::KeepLast(1)));
    yaw_p_ = this->create_publisher<std_msgs::msg::Float64>("yaw_", 10);//调试用

    
            
    debug_pub = this -> create_publisher<base_interfaces::msg::DebugOfKinematic>("debug",2);
    timer_ptr_ = this->create_wall_timer(5ms, std::bind(&PoseSolver::control_callback, this));    //200帧控制帧率
    if (debug)  // 调试模式才发布预测装甲板
    {
        armors_pub = this->create_publisher<base_interfaces::msg::Armors>("test", 1);
    }
    if (debug) {
    initMarkers();
    }
    header_.frame_id = shooter_link_;

}

void PoseSolver::control_callback()
{        

    mtx.lock();
    if (is_track_ == 1) // 是否是自瞄状态
    {
        builtin_interfaces::msg::Time time = this->now();//获取当前时间
        builtin_interfaces::msg::Time status_time = header_.stamp;//从status_ptr_指向的对象中提取时间戳
        double time_now = time.sec + time.nanosec * 1e-9;
        double time_status = status_time.sec + status_time.nanosec * 1e-9;
        if(time_now - time_status > m_timeout)
        {
            std::cout << "TIMEOUT!" << std::endl;
            mtx.unlock();   // 每个return都要解锁，不然model永远不更新
            return;
        }
        geometry_msgs::msg::Pose target_armor_pose; //获取目标装甲板位姿
        if (model_type_ == 0 || model_type_ == 1)    // model模型是否存在
        {
            // 判断是否锁中心
            if (m_track_center && fabs(palstance_) > m_switch_trackmode_threshold + m_aim_center_palstance_threshold)//用status_ptr_指针获取palstance（角速度）
            {
                m_enable_center_track = true;
            }
            else if (m_track_center && fabs(palstance_) < m_aim_center_palstance_threshold)
            {
                m_enable_center_track = false;
            }
            else if (!m_track_center)
                m_enable_center_track = false;

            // 获取模型当前时刻要击打的装甲板位姿
            if (!m_enable_center_track) // 不锁中心
            {
                if (!getClosestArmorPose(0, target_armor_pose))
                {

                    mtx.unlock();
                    return;
                }
            }//不锁中心，获取最靠近的装甲板位姿
            else    // 锁中心
            {               
                if (!getFacingArmorPose(0, target_armor_pose))
                {
                    mtx.unlock();
                    return;
                }
            }//锁中心，获取面对的装甲板位姿
            // 计算预测时间，即子弹飞行时间 + 建模时间 + time_off
            double start_time, end_time;
            start_time = header_stamp_.sec + header_stamp_.nanosec * 1e-9;//获取开始时间戳
            builtin_interfaces::msg::Time t = this->now();
            end_time = t.sec + t.nanosec * 1e-9;//获取结束时间戳
            m_time_off = time_off_;
            double predict_time = getDistance1(cv::Point3d(target_armor_pose.position.x, target_armor_pose.position.y, target_armor_pose.position.z)) / m_bullet_speed + (end_time - start_time) + m_time_off;//计算预测时间
            // 获取预测时间后的装甲板位姿
            if (!m_enable_center_track)
            {
                if (!getClosestArmorPose(predict_time, target_armor_pose))
                {
                    mtx.unlock();
                    return;
                }
            }
            else
            {
                if (!getFacingArmorPose(predict_time, target_armor_pose))
                {
                    mtx.unlock();
                    return;
                }
            }
        } 
        else    // 没有模型，直接击打检测到的装甲板
        {
            if (armors_.size() > 0)
            {
                target_armor_pose = armors_[0].pose;
                geometry_msgs::msg::PoseStamped p;
                p.header = armors_[0].header;
                p.header.frame_id = base_link_;
                p.pose = target_armor_pose;
                target_armor_pose = tf_buffer_->transform(p, shooter_link_).pose;
                std::cout << "转换后到枪管系的装甲板坐标 - X: " << target_armor_pose.position.x 
                            << " Y: " << target_armor_pose.position.y 
                            << " Z: " << target_armor_pose.position.z << std::endl;
            }
            else
            {
              mtx.unlock();
                // std::cout << "not have armor to aim" << std::endl;
              return;
            }
        }
        base_interfaces::msg::GimbalPose target_gimbal_pose;//目标云台位姿
        if (!getAngle(target_armor_pose, m_bullet_speed, target_gimbal_pose))   // 获取目标云台位姿
        {
            mtx.unlock();
            return;
        }
        Gimbal_pose_pub_->publish(target_gimbal_pose);//发布目标云台位姿
        auto yaw_msg = std::make_shared<std_msgs::msg::Float64>();
                    yaw_msg->data = target_gimbal_pose.yaw; 
                    yaw_p_->publish(*yaw_msg);//可视化调试
        if (debug)
        {
             //std::cout << "target_gimbal_pose.pitch:12 " << target_gimbal_pose.pitch << std::endl;
             //std::cout << "target_gimbal_pose.yaw:1 " << target_gimbal_pose.yaw << std::endl;
        }
        geometry_msgs::msg::Pose armor_pose_abs = target_armor_pose;
        geometry_msgs::msg::TransformStamped t_debug;
        base_interfaces::msg::DebugOfKinematic solverDebug;
        solverDebug.pose = armor_pose_abs;
        solverDebug.armors = armors_;
        solverDebug.id = id_;
        debug_pub -> publish(solverDebug);

        base_interfaces::msg::GimbalPose shoot_msg;//击打消息
        shoot_msg.bulletnum = 0;
        if(this->enable_shoot && shootable(target_armor_pose, target_gimbal_pose))    // 判断是否被击打
        {
            builtin_interfaces::msg::Time t = this->now();
            double time = t.sec + t.nanosec * 1e-9;
            if (time - last_shooter_time > shoot_interval)
            {
                last_shooter_time = time;
                 std::cout << "enable shoot!" << std::endl;
                if(shoot_interval > 0) shoot_msg.bulletnum = 1;
                else shoot_msg.bulletnum = -1;//子弹数量
            }
            if (armors_.size() > 0)
            {
              white_frame_ = 0;  
            }
            else
            {
                white_frame_++;
            }
            if (white_frame_ > all_white_tolerance_stop_shoot_)
                shoot_msg.bulletnum = 0;
        }
        else
        {
             std::cout << "not shoot!" << std::endl;
        }
        //Gimbal_pose_pub_->publish(shoot_msg);

        if (debug) {
    try {
        // 正确获取 TF 变换
        auto transform = tf_buffer_->lookupTransform(
            base_link_, shooter_link_, tf2::TimePointZero);
        
        // 创建要转换的位姿
        geometry_msgs::msg::PoseStamped p;
        p.header.frame_id = shooter_link_;
        p.header.stamp = this->now();
        p.pose = target_armor_pose;
        
        // 执行坐标变换
        geometry_msgs::msg::PoseStamped transformed_pose;
        tf2::doTransform(p, transformed_pose, transform);
        
        // 发布标记
        publishMarkers(transformed_pose.pose, target_gimbal_pose);
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(this->get_logger(), "TF transform error: %s", ex.what());
    }
}
    }
    else
    {
        //std::cout << "no tracked armor" << std::endl; 
    }
    mtx.unlock();
    return;
}
//3D点坐标
double PoseSolver::getDistance1(const cv::Point3d& point) {
    return std::sqrt(
        point.x * point.x + 
        point.y * point.y + 
        point.z * point.z
    );
}
double PoseSolver::getDistance2(const cv::Point2d& point) {
    return std::sqrt(
        point.x * point.x + 
        point.y * point.y 
    );
}

std::vector<base_interfaces::msg::Armor> PoseSolver::getArmors(double predict_time, int number)
{
    std::vector<base_interfaces::msg::Armor> armors(number);
    if (number == 4)    // STANDARD模型 四块装甲板
    {
        for (int i = 0; i < number; i++)
        {
            //std::cout << "i: " << i << std::endl;
            size_t radius_index = (i == 0 || i == 2) ? 0 : 1; // 装甲板0/2用radius_[0], 1/3用radius_[1]
            size_t height_index = (i == 0 || i == 2) ? 0 : 1;  // 装甲板0/2用height_[0], 1/3用height_[1]
            
            if((height_index >= radius_.size()) || (radius_index >= height_.size())) {
                RCLCPP_ERROR(this->get_logger(), "Array index out of bounds!");
                continue;
            }
            // 装甲板yaw角，绝对系
            double angle = _std_radian(phase_ + palstance_ * predict_time + i * M_PI / 2);
            cv::Point2d point = cv::Point2d(center_.x, center_.y) + 
                                cv::Point2d(velocity_.x, velocity_.y) * predict_time + 
                                cv::Point2d(radius_[radius_index] * cos(angle), 
                                          radius_[radius_index] * sin(angle));

            armors[i].pose.position.x = point.x;
            armors[i].pose.position.y = point.y;
            armors[i].pose.position.z = height_[height_index];
            // 将yaw角转四元数
            tf2::Quaternion q;
            q.setRPY(0, 0, angle);
            armors[i].pose.orientation.x = q.x();
            armors[i].pose.orientation.y = q.y();
            armors[i].pose.orientation.z = q.z();
            armors[i].pose.orientation.w = q.w();

        }
    }
    else if (number == 2)   //  BALANCE模型 两块装甲板
    {
        for (int i = 0; i < number; i++)
        {
            // 装甲板yaw角，绝对系
            double angle = _std_radian(phase_ + palstance_ * predict_time + i * M_PI);
            cv::Point2d point = cv::Point2d(center_.x, center_.y) +
                                cv::Point2d(velocity_.x * cos(phase_) * predict_time,
                                            velocity_.x * sin(phase_) * predict_time) 
                                  + cv::Point2d(radius_[0] * cos(angle), radius_[0] * sin(angle));
            armors[i].pose.position.x = point.x;
            armors[i].pose.position.y = point.y;
            armors[i].pose.position.z = height_[0];
            // 将yaw角转四元数
            tf2::Quaternion q;
            q.setRPY(0, 0, angle);
            armors[i].pose.orientation.x = q.x();
            armors[i].pose.orientation.y = q.y();
            armors[i].pose.orientation.z = q.z();
            armors[i].pose.orientation.w = q.w();
        }
    }
    return armors;
}//预测时间后装甲板位姿

bool PoseSolver::getClosestArmorPose(double predict_time, geometry_msgs::msg::Pose &armor_pose_out)
{
    // 获取装甲板数量

    int number = 0;
    if (model_armors.count(model_type_))
    {                            
        number = model_armors[model_type_];
    }
    else
    {
         std::cout << "error model type: " << model_type_ << std::endl;
        return false;
    }
    // 计算预测后装甲板的位姿
    if (debug)
    {
        this->get_parameter<float>("Aimer.switch_threshold", m_switch_threshold);
    }
    double switch_advanced_time = 0.2;           
    if (palstance_ != 0)
    {
        switch_advanced_time = std::min(0.2, D2R(m_switch_threshold) / abs(palstance_));
    }
    std::vector<base_interfaces::msg::Armor> armors = getArmors(predict_time + switch_advanced_time, number);
    for (auto &armor : armors)
    {
        geometry_msgs::msg::PoseStamped p;
        p.header = armor.header;
        p.header.frame_id = base_link_;
        p.pose = armor.pose;
        armor.pose = tf_buffer_->transform(p, shooter_link_).pose;
        // std::cout << "getClosestArmorPose  转换后的装甲板坐标 - X: " << armor.pose.position.x 
        //             << " Y: " << armor.pose.position.y 
        //             << " Z: " << armor.pose.position.z << std::endl;
        tf2::Quaternion tf_q;
        tf2::fromMsg(armor.pose.orientation, tf_q);   // 将geometry_msgs::msg::Quaternion转成tf2::Quaternion消息
        double roll, pitch, yaw;
        tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);  // 获取电机系下装甲板yaw角
        armor.yaw_angle = yaw;
    }
    // 计算预测后的机器人位置
    geometry_msgs::msg::Pose center;    // OUTPOST模型
    center.position.x = center_.x;
    center.position.y = center_.y;
    if (model_type_ == 0)   // STANDAR模型
    {
        center.position.x += velocity_.x * predict_time;
        center.position.y += velocity_.y * predict_time;
    }
    else if (model_type_ == 1)  // BALANCE模型
    {
        center.position.x += velocity_.x * cos(phase_) * predict_time;
        center.position.y += velocity_.x * sin(phase_) * predict_time;
    }
    geometry_msgs::msg::PoseStamped ps;
    ps.header = header_;
    ps.header.frame_id = base_link_;
    ps.pose = center;
    center = tf_buffer_->transform(ps, shooter_link_).pose;
    // std::cout << "转换后的中心装甲板坐标 - X: " << center.position.x 
    //             << " Y: " << center.position.y << std::endl;
    // 选择要击打的装甲板
    int index = 0;
    for (int i = 1; i < number; i++)
    {
        if (abs(_std_radian(M_PI + armors[i].yaw_angle - atan2(center.position.y, center.position.x))) + D2R(m_switch_threshold) <
            abs(_std_radian(M_PI + armors[index].yaw_angle - atan2(center.position.y, center.position.x))))
        {
            index = i;
        }
    }
    armor_pose_out = armors[index].pose; 
    if (debug)
    {
    if (predict_time != 0)
    {
        base_interfaces::msg::Armors armors_msg;
        for (auto i = armors.begin(); i != armors.end(); ++i)
        {
            i->header.frame_id = shooter_link_;       // 使用迭代器成员访问
            i->header.stamp = header_stamp_;
            armors_msg.armors.push_back(*i);          // 解引用迭代器
            armors_msg.num++;
        }
        base_interfaces::msg::Armor armor;
        armor.header.frame_id = shooter_link_;
        armor.header.stamp = header_stamp_;
        armor.pose = center;
        armor.number = index;
        armors_msg.armors.push_back(armor);
        armors_pub->publish(armors_msg);
    }
    }
    return true;
}//预测时间后最近的装甲板位姿

bool PoseSolver::getFacingArmorPose(double predict_time, geometry_msgs::msg::Pose &armor_pose_out)
{
    // 获取装甲板数量
    int number = 0;

    if (model_armors.count(model_type_))
    {
        number = model_armors[model_type_];
    }
    else
    {
        // std::cout << "error model type: " << model_type_ << std::endl;
        return false;
    }
    // 计算预测后装甲板的位姿
    std::vector<base_interfaces::msg::Armor> armors = getArmors(predict_time, number);
    for (auto &armor : armors)
    {
        geometry_msgs::msg::PoseStamped ps;
        ps.header = armor.header;
        ps.header.frame_id = base_link_;
        ps.pose = armor.pose;
        armor.pose = tf_buffer_->transform(ps, shooter_link_).pose;
        std::cout << " getFacingArmorPose 转换后的装甲板坐标 - X: " << armor.pose.position.x 
                    << " Y: " << armor.pose.position.y 
                    << " Z: " << armor.pose.position.z << std::endl;
        tf2::Quaternion tf_q;
        tf2::fromMsg(armor.pose.orientation, tf_q);   // 将geometry_msgs::msg::Quaternion转成tf2::Quaternion消息
        double roll, pitch, yaw;
        tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);  // 获取电机系下装甲板yaw角
        armor.yaw_angle = _std_radian(yaw);
    }
    // 计算预测后的机器人位置
    geometry_msgs::msg::Pose center;
    if (model_type_ == 0)   // STANDAR模型
    {
        center.position.x += velocity_.x * predict_time;
        center.position.y += velocity_.y * predict_time;
    }
    else if (model_type_ == 1)  // BALANCE模型
    {
        center.position.x += velocity_.x * cos(phase_) * predict_time;
        center.position.y += velocity_.x * sin(phase_) * predict_time;
    }
    geometry_msgs::msg::TransformStamped t;
    geometry_msgs::msg::PoseStamped ps;
    ps.header = header_;
    ps.pose = center;
    center = tf_buffer_->transform(ps, shooter_link_).pose;
    std::cout << " getFacingArmorPose 转换后的装甲板坐标 - X: " << center.position.x 
                << " Y: " << center.position.y 
                << " Z: " << center.position.z << std::endl;
    // 选择要击打的装甲板
    int index = 0;
    for (int i = 1; i < number; i++)
    { 
        if (abs(_std_radian(M_PI + armors[i].yaw_angle - atan2(center.position.y, center.position.x))) <
            abs(_std_radian(M_PI + armors[index].yaw_angle - atan2(center.position.y, center.position.x))))
        {
            index = i;
        }
    }

    double angle = _std_radian(atan2(center.position.y, center.position.x) - M_PI);
    // 将yaw角转四元数
    tf2::Quaternion q;
    q.setRPY(0, 0, angle);
    armor_pose_out.orientation.x = q.x();
    armor_pose_out.orientation.y = q.y();
    armor_pose_out.orientation.z = q.z();
    armor_pose_out.orientation.w = q.w();
    if (model_type_ == 0)   // STANDARD模型
    {
        // 如果最近的装甲板处于远离状态，则击打下一装甲板
        if (abs(_std_radian(M_PI + armors[index].yaw_angle + (palstance_ / abs(palstance_)) * D2R(-50) - atan2(center.position.y, center.position.x))) <
            abs(_std_radian(M_PI + armors[index].yaw_angle - atan2(center.position.y, center.position.x))))
        {
            index = (index + 1) % number;
        }
        armor_pose_out.position.x = center.position.x + radius_[index % 2] * cos(angle);
        armor_pose_out.position.y = center.position.y + radius_[index % 2] * sin(angle);
        armor_pose_out.position.z = height_[index % 2];
    }
    else    // BALANCE模型 
    {
        armor_pose_out.position.x = center.position.x + radius_[0] * cos(angle);
        armor_pose_out.position.z = center.position.y + radius_[0] * sin(angle);
        armor_pose_out.position.z = height_[0];
    }
    return true;
}//获取正面对的装甲板位姿

bool PoseSolver::getAngle(geometry_msgs::msg::Pose target_armor_pose, float bullet_speed, base_interfaces::msg::GimbalPose &gimbal_pose)
{
    gimbal_pose.pitch = -std::atan(target_armor_pose.position.z / std::sqrt(target_armor_pose.position.x * target_armor_pose.position.x + target_armor_pose.position.y * target_armor_pose.position.y));
    gimbal_pose.yaw = std::atan2(target_armor_pose.position.y, target_armor_pose.position.x);
    std::cout<<" 目标  gimbal_pose.pitch  "<<gimbal_pose.pitch<<std::endl;
    std::cout<<"  目标  gimbal_pose.yaw  "<<gimbal_pose.yaw<<std::endl;

    if (debug)
    {
        this->get_parameter<float>("Gimbal.pitch_off", pitch_off_);
        this->get_parameter<float>("Gimbal.yaw_off", yaw_off_);
    }
    gimbal_pose.pitch += pitch_off_;
    gimbal_pose.yaw += yaw_off_;
    
    if (debug)
    {
        this->get_parameter<bool>("Gimbal.fix_on", fix_on_);
    }
    if(fix_on_)
    {
        double dis = sqrt(target_armor_pose.position.x * target_armor_pose.position.x + target_armor_pose.position.z * target_armor_pose.position.z);
        double angle_fix = 0;
        if (abs(9.8 * dis * pow(cos(gimbal_pose.pitch), 2)) / pow(bullet_speed, 2) - sin(gimbal_pose.pitch) <= 1)
        {
            angle_fix = 0.5 * (asin((9.8 * dis * pow(cos(gimbal_pose.pitch), 2)) / pow(bullet_speed, 2) - sin(gimbal_pose.pitch)) + gimbal_pose.pitch);
        }
        else
        {
            return false;
        }
        gimbal_pose.pitch -= angle_fix;
    }
    return true;
}//计算射击角度

bool PoseSolver::shootable(geometry_msgs::msg::Pose target_armor_pose, base_interfaces::msg::GimbalPose target_gimbal_pose)
{
    /**
    * 判断自动击发，条件如下：
    * 1. 目标旋转速度小于一定阈值，或目标装甲板相对偏角不超过一定范围
    * 2. EKF先验稳定
    * 3. 没有长时间丢识别
    * 4. 当前云台跟随稳定，即当前位姿和目标位姿相近
    */
    geometry_msgs::msg::Pose vector_c_pose;
    vector_c_pose.position.x = center_.x;
    vector_c_pose.position.y = center_.y;
    geometry_msgs::msg::TransformStamped _t;
    cv::Point2d vector_c = cv::Point2d(vector_c_pose.position.x, vector_c_pose.position.y);//中心坐标
    cv::Point2d vector_a = cv::Point2d(vector_c_pose.position.x - target_armor_pose.position.x, vector_c_pose.position.y - target_armor_pose.position.y);
    double abs_angle = R2D(acos((vector_c.x * vector_a.x + vector_c.y * vector_a.y) / (getDistance2(vector_c) * getDistance2(vector_a))));//
    if (debug)
    {
        this->get_parameter<float>("Aimer.force_aim_palstance_threshold", m_force_aim_palstance_threshold);
        this->get_parameter<float>("Aimer.aim_angle_tolerance", m_aim_angle_tolerance);        
    }
    if (abs(palstance_) < m_force_aim_palstance_threshold || abs_angle < m_aim_angle_tolerance)
    {
        if (debug)
        {  
            this->get_parameter<float>("Aimer.CenterTrack.aim_center_angle_tolerance", m_aim_center_angle_tolerance);
        }
        if (m_enable_center_track && !(abs_angle < m_aim_center_angle_tolerance))
        {
             std::cout << "[shootable]: false 1! abs_angle = " << abs_angle << " < m_aim_center_angle_tolerance" << std::endl;
            return false;
        }
    }
    else
    {
        std::cout << "[shootable]: false 2! abs(palstance_): " << abs(palstance_) << " , abs_angle: " << abs_angle << std::endl;
        return false;
    }
    double roll, pitch, yaw, pitch_, yaw_;
    geometry_msgs::msg::TransformStamped t;
    auto shooter_tf = tf_buffer_->lookupTransform(base_link_,shooter_link_,  header_stamp_);//获取当前云台姿态 rpy
    auto msg_q = shooter_tf.transform.rotation;//云台四元数
    tf2::Quaternion tf_q;//定义四元数
    tf2::fromMsg(msg_q, tf_q);//ros消息四元数转tf2四元数
    tf2::Matrix3x3(tf_q).getRPY(roll, pitch_, yaw_);//tf2四元数转rpy 提取欧辣角
    yaw = yaw_;
    // yaw = _std_radian(yaw);
    pitch = -pitch_;
    // pitch = pitch * (-1);
    if (debug)
    {
        std::cout << "[shootable] pitch: " <<  pitch << std::endl;
        std::cout << "[shootable] yaw: " <<  yaw << std::endl;
    }
    this->get_parameter<float>("Aimer.aim_pose_tolerance", m_aim_pose_tolerance);
    if (id_ == "1" || model_type_ == 1)
    {
        m_aim_pose_tolerance *= 1.5;
    }
    std::cout<<"hypot(target_gimbal_pose.pitch - pitch, target_gimbal_pose.yaw - yaw) "<<hypot(target_gimbal_pose.pitch - pitch, target_gimbal_pose.yaw - yaw)<<std::endl;
    if (std::hypot(target_gimbal_pose.pitch - pitch, target_gimbal_pose.yaw - yaw) < m_aim_pose_tolerance)
    {
        return true;
    }
    return false;
}//判断是否可射击

void PoseSolver::publishMarkers(const geometry_msgs::msg::Pose& target_pose,
                               const base_interfaces::msg::GimbalPose& gimbal_cmd) {
  if (header_.frame_id.empty()) {
    header_.frame_id = base_link_;
    RCLCPP_WARN_ONCE(get_logger(), "Marker frame_id not set, defaulting to base_link");
  }
  header_.frame_id = "base_link"; 
  header_.stamp = this->now();
  visualization_msgs::msg::MarkerArray marker_array;
  
  // 机器人中心
  position_marker_.header = header_;
  position_marker_.pose.position.x = center_.x;
  position_marker_.pose.position.y = center_.y;
  position_marker_.pose.position.z = height_[0];
  position_marker_.action = visualization_msgs::msg::Marker::ADD;
  marker_array.markers.push_back(position_marker_);

  position_.header = header_;
  position_.pose.position.x = center_.x;
  position_.pose.position.y = center_.y;
  position_.pose.position.z = height_[0];
  position_.action = visualization_msgs::msg::Marker::ADD;
  marker_array.markers.push_back(position_);

  // 预测装甲板
   armors_marker_.header = header_;
   std::vector<base_interfaces::msg::Armor> predicted_armors = getArmors(0, 4);
   

    for (int i = 0; i < 4; ++i) {
        visualization_msgs::msg::Marker armor_marker;
        armor_marker.header = header_;
        armor_marker.ns = "predicted_armors";
        armor_marker.id = i;
        armor_marker.type = visualization_msgs::msg::Marker::CUBE; 
        armor_marker.action = visualization_msgs::msg::Marker::ADD;
        armor_marker.pose = predicted_armors[i].pose;
        armor_marker.scale.x = 0.06; 
        armor_marker.scale.y = 0.43;
        armor_marker.scale.z = 0.45; 
        armor_marker.color.r = 0.0; 
        armor_marker.color.g = 1.0;  
        armor_marker.color.b = 0.0;  
        armor_marker.color.a = 1.0; 
        marker_array.markers.push_back(armor_marker);
    }

    if (velocity_.x != 0 || velocity_.y != 0) {
    visualization_msgs::msg::Marker linear_marker = linear_v_marker_;
    linear_marker.header = header_;
    linear_marker.id = 0;
    linear_marker.pose.position.x = center_.x;
    linear_marker.pose.position.y = center_.y;
    linear_marker.pose.position.z = height_[0] + 0.1; // 略高于中心
    
    // 计算线速度方向
    const double speed = sqrt(velocity_.x * velocity_.x + velocity_.y * velocity_.y);
    const double angle = atan2(velocity_.y, velocity_.x);
    tf2::Quaternion q;
    q.setRPY(0, 0, angle); 
    linear_marker.pose.orientation = tf2::toMsg(q);
    linear_marker.scale.x = speed * 0.1; // 缩放速度长度
    
    marker_array.markers.push_back(linear_marker);
  }

  // 选中装甲板
  selection_marker_.header = header_;
  selection_marker_.pose = target_pose;
  //std::cout<<"target_pose  "<<target_pose.position.x<<std::endl;
  //std::cout<<"target_pose  "<<target_pose.position.y<<std::endl;
  //std::cout<<"target_pose  "<<target_pose.position.z<<std::endl;

//   selection_marker_.pose.position.z = 0.15;
  marker_array.markers.push_back(selection_marker_);
  marker_pub_->publish(marker_array);
}

void PoseSolver::initMarkers() {
  position_marker_.ns = "robot_center";
  position_marker_.type = visualization_msgs::msg::Marker::SPHERE;
  position_marker_.scale.x = position_marker_.scale.y = position_marker_.scale.z = 0.15;
  position_marker_.color.a = 1.0;
  position_marker_.color.g = 1.0;

  position_.ns = "robot_";
  position_.type = visualization_msgs::msg::Marker::SPHERE;
  position_.scale.x = position_marker_.scale.y = position_marker_.scale.z = 0.15;
  position_.color.a = 1.0;
  position_.color.g = 1.0;

  linear_v_marker_.type = visualization_msgs::msg::Marker::ARROW;
  linear_v_marker_.ns = "linear_velocity";
  linear_v_marker_.scale.x = 0.05;  // 杆部直径增大到0.05m
  linear_v_marker_.scale.y = 0.08;  // 头部直径增大到0.08m
  linear_v_marker_.color.r = 1.0;   // 纯红色
  linear_v_marker_.color.a = 0.8;   // 略微透明避免遮挡

  angular_v_marker_.type = visualization_msgs::msg::Marker::ARROW;
  angular_v_marker_.ns = "angular_velocity";
  angular_v_marker_.scale.x = 0.03;
  angular_v_marker_.scale.y = 0.05; 
  angular_v_marker_.color.a = 1.0;
  angular_v_marker_.color.b = 1.0;

  armors_marker_.ns = "predicted_armors";
  armors_marker_.type = visualization_msgs::msg::Marker::CUBE;
  armors_marker_.scale.x = 0.12;
  armors_marker_.scale.z = 0.23;
  armors_marker_.color.a = 1.0;
  armors_marker_.color.b = 1.0;

  selection_marker_.ns = "selected_armor";
  selection_marker_.type = visualization_msgs::msg::Marker::SPHERE;
  selection_marker_.scale.x = selection_marker_.scale.y = selection_marker_.scale.z = 0.1;
  selection_marker_.color.a = 1.0;
  selection_marker_.color.r = 1.0;
  selection_marker_.color.g = 1.0;

  trajectory_marker_.ns = "bullet_trajectory";
  trajectory_marker_.type = visualization_msgs::msg::Marker::LINE_STRIP;
  trajectory_marker_.scale.x = 0.02;
  trajectory_marker_.color.a = 1.0;
  trajectory_marker_.color.r = 1.0;
  trajectory_marker_.color.g = 0.5;

  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/pose_solver/markers", 10);
}

void PoseSolver::top_sb_(const base_interfaces::msg::KinematicStatus& status_msg)
{
    std::lock_guard<std::mutex> lock(mtx);
    
    armors_ = status_msg.armors;
    header_ = status_msg.header;
    header_stamp_ = status_msg.header.stamp;
    palstance_ = status_msg.palstance;
    center_ = status_msg.center;
    velocity_ = status_msg.velocity;
    phase_ = status_msg.phase;
    armor_num_ = status_msg.armor_num;
    
    radius_ = status_msg.radius;
    height_ = status_msg.height;

    is_track_ = status_msg.is_track;
    id_ = status_msg.id;
    model_type_ = status_msg.model_type;

    armors_ = status_msg.armors;
    time_off_ = status_msg.time_off;
}
} //namespace HL
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::PoseSolver)