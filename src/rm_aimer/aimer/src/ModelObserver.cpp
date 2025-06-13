#include "aimer/ModelObserver.hpp"


namespace rm_auto_aim{
ModelObserver::ModelObserver()
    {}
    ModelObserver::~ModelObserver()
    {}


    StandardObserver::~StandardObserver()
    {}

    void StandardObserver::reset()
    {
        m_init = false;

        m_P = Eigen::Matrix<double, 10, 10>::Identity() * 1e-5;
    }

    std::pair<Eigen::MatrixXd, Eigen::MatrixXd> StandardObserver::predict(base_interfaces::msg::Armors &armors)
    {                        
        m_init = false;
        

        if (!m_init)
        {   
            for (auto &armor : armors.armors)
            {
                m_X << armor.pose.position.x + m_init_radius * cos(armor.yaw_angle),//中心x
                        0,//x速度
                        armor.pose.position.y + m_init_radius * sin(armor.yaw_angle),//中心y
                        0,//y速度
                        armor.pose.position.z,//高度
                        armor.pose.position.z,
                        m_init_radius,//半径
                        m_init_radius,
                        armor.yaw_angle,
                        0;//角速度
            //std::cout<<"m_X        "<<m_X<<std::endl;//状态向量
            //std::cout << "yaw_angle x: " << armor.yaw_angle << std::endl;
                        m_init = true;
                        break;
            }
         }

        // 预测
        Eigen::MatrixXd X_ = m_F * m_X;//状态预测 
        ///std::cout<<"X_    "<<X_<<std::endl;
        //std::cout<<"m_F    "<<m_F<<std::endl;
        Eigen::MatrixXd P_ = m_F * m_P * m_F.transpose() + m_Q;//P_​​先验状态协方差矩阵（预测后的不确定性） ​​ m_P后验状态协方差矩阵​
        //std::cout<<"P_    "<<P_<<std::endl;

        if (m_debug)
        {
            //std::cout << "[StandardObserver] X_: " << std::endl << X_ << std::endl;
            //std::cout << "[StandardObserver] P_: " << std::endl << P_ << std::endl;
        }
        return std::make_pair(X_, P_);
    }
    std::shared_ptr<KinematicModel> StandardObserver::update(std::vector<base_interfaces::msg::Armor> &armors, const Eigen::MatrixXd &X_, const Eigen::MatrixXd &P_, std::map<int, int> &match)
    {
        // 观测
        Eigen::MatrixXd Z;      // 实际观测量
        Eigen::MatrixXd h;      // 先验观测量
        Eigen::MatrixXd H;      // 观测方程偏导矩阵
        Eigen::MatrixXd V;      // 观测噪声偏导矩阵
        Eigen::MatrixXd K;      // 置信度权重矩阵
        Eigen::MatrixXd R;      // 时变观测噪声矩阵
        if (match.size() == 1)
        {
            int num = match.begin()->first;
            Z.resize(4, 1);
            Z << armors[num].pose.position.x, armors[num].pose.position.y, armors[num].pose.position.z, armors[num].yaw_angle;//实际观测矩阵
            //std::cout<<"实际观测矩阵 "<<Z<<std::endl;
            h.resize(4, 1);
            h << getPredictiveMeasurement(X_, match[num]);
           // std::cout<<"先验观测 "<<h<<std::endl;
            H.resize(4, 10);
            H << getMeasurementPD(X_, match[num]);//   非线性观测模型 h(X) 需要通过雅可比矩阵 H 线性化
            //std::cout<<"观测方程求偏导 "<<H<<std::endl;
            V.resize(4, 4);
            V << getMeasureNoisePD(X_, match[num]);//观测噪声的偏导矩阵
            //std::cout<<"观测噪声求偏导 "<<V<<std::endl;
            R = m_R;//测量噪声的对角矩阵
            //std::cout<<"测量噪声的对角矩阵 "<<R<<std::endl;
            R(2, 2) *= m_gain * abs(Z(3, 0));
            K = P_ * H.transpose() * ((H * P_ * H.transpose() + V * R * V.transpose())).inverse();
            //std::cout<<"置信度权重矩阵 "<<K<<std::endl;
        }
        
        else if (match.size() == 2)
        {

            int num1 = match.begin()->first;
            int num2 = (++match.begin())->first;
            Z.resize(8, 1);
            Z << armors[num1].pose.position.x, armors[num1].pose.position.y, armors[num1].pose.position.z, armors[num1].yaw_angle,
                 armors[num2].pose.position.x, armors[num2].pose.position.y, armors[num2].pose.position.z, armors[num2].yaw_angle;
            h.resize(8, 1);
            h << getPredictiveMeasurement(X_, match[num1]),
                 getPredictiveMeasurement(X_, match[num2]);
            H.resize(8, 10);
            H << getMeasurementPD(X_, match[num1]),
                 getMeasurementPD(X_, match[num2]);
            V.resize(8, 8);
            V << getMeasureNoisePD(X_, match[num1]), Eigen::MatrixXd::Zero(4, 4),
                 Eigen::MatrixXd::Zero(4, 4), getMeasureNoisePD(X_, match[num2]);
            R = m_RR;
            R(2, 2) *= m_gain * abs(Z(3, 0));
            R(6, 6) *= m_gain * abs(Z(7, 0));

            K = P_ * H.transpose() * ((H * P_ * H.transpose() + V * R * V.transpose())).inverse();
        }
        else
        {
            // 若匹配数目不为1或2，则认为丢识别或者误判，直接返回先验状态
            m_X = X_;
            m_P = P_;

            return std::make_shared<StandardModel>(m_X);
        }

        if (m_debug)
        {
            // std::cout << "[StandardObserver] Z: " << std::endl << Z << std::endl;
            // std::cout << "[StandardObserver] h: " << std::endl << h << std::endl;
            // std::cout << "[StandardObserver] H: " << std::endl << H << std::endl;
        }

        // 更新
        Eigen::MatrixXd tmp = Z - h;
        for (size_t i = 0; i < match.size(); i++)
        {
            tmp(3 + i * 4, 0) = _std_radian(tmp(3 + i * 4, 0));
        }
        m_X_update = K * tmp;
        if (m_debug)
        {
            // std::cout << "[StandardObserver] X_update: " << std::endl << m_X_update << std::endl;
        }

        m_X = X_ + m_X_update;
        m_P = (Eigen::Matrix<double, 10, 10>::Identity() - K * H) * P_;
        //std::cout<<" 最终预测  m_X  11"<<m_X<<std::endl;

        if (m_debug)
        {
        //    std::cout << "[StandardObserver] X:" << std::endl << m_X << std::endl;
        //     std::cout << "[StandardObserver] P:" << std::endl << m_P << std::endl;
        }
        return std::make_shared<StandardModel>(m_X);
    }
    bool StandardObserver::stable()
    {
        return true;
    }

    // Eigen::MatrixXd StandardObserver::getPredictiveMeasurement(const Eigen::MatrixXd &X, int i)
    // {

    //     Eigen::Matrix<double, 4, 1> h;
    //     double x = X(0, 0) + X(6 + i % 2, 0) * cos(X(8, 0) + i * M_PI / 2);
    //     double y = X(2, 0) + X(6 + i % 2, 0) * sin(X(8, 0) + i * M_PI / 2);
    //     double z = X(4 + i % 2, 0);
    //     h << x, //x坐标
    //          y, //y坐标
    //          sqrt(x*x + y*y + z*z), //距离 
    //          X(8)+i * M_PI/2;//yaw
    //     return h;
    // }//构造观测矩阵 xy distance yaw 

    // Eigen::MatrixXd StandardObserver::getMeasurementPD(const Eigen::MatrixXd &X, int i)
    // {        

    //     Eigen::Matrix<double, 4, 10> H;
    //     const double theta = X(8, 0) + i * M_PI / 2.0;
    //     const double radius = X(6 + i % 2, 0);
    //     const double x = X(0, 0) + radius * cos(theta);
    //     const double y = X(2, 0) + radius * sin(theta);
    //     const double z = X(4 + i % 2, 0);
    //     const double distance = sqrt(x*x + y*y + z*z);
    //     const double inv_dist = 1.0 / distance;//首先保证dis不为0  防止除0错误 下列求偏导应该在dis不为0的情况下进行
    //     H << 1, 0, 0, 0, 0,           0,     ((i + 1) % 2) * cos(X(8, 0) + i * M_PI / 2), (i % 2) * cos(X(8, 0) + i * M_PI / 2),-X(6 + i % 2, 0) * sin(X(8, 0) + i * M_PI / 2), 0,
    //          0, 0, 1, 0, 0,           0,     ((i + 1) % 2) * sin(X(8, 0) + i * M_PI / 2), (i % 2) * sin(X(8, 0) + i * M_PI / 2), X(6 + i % 2, 0) * cos(X(8, 0) + i * M_PI / 2), 0,
    //          x * inv_dist, 0, y * inv_dist, 0, ((i + 1) % 2) * z * inv_dist, ((i + 1) % 2) * z * inv_dist, (x*cos(theta) + y*sin(theta)) * inv_dist,  (x*cos(theta) + y*sin(theta)) * inv_dist, (-x*radius*sin(theta) + y*radius*cos(theta)) * inv_dist, 0,
    //          0, 0, 0, 0, 0,           0,     0,                                           0,                                   1,                                           0;
    //             return H;
    // }//预测观测值 h（4×1）对状态 X（10×1）的偏导数

    // Eigen::MatrixXd StandardObserver::getMeasureNoisePD(const Eigen::MatrixXd &X, int i)
    // {
    //     Eigen::Matrix<double, 4, 1> h = getPredictiveMeasurement(X, i);
    //     double psi = atan2(h(1, 0), h(0, 0));//装甲板在水平面上的方向 极坐标
    //     double phi = atan2(X(4 + i % 2, 0), sqrt(pow(h(0, 0), 2) + pow(h(1, 0), 2)));//装甲板相对于水平面的仰角
    //     double d = h(2, 0);//装甲板到观测点的欧氏距离
    //     Eigen::Matrix<double, 4, 4> V;
    //    V << cos(phi)*cos(psi), -d*cos(phi)*sin(psi), -d*sin(phi)*cos(psi), 0,
    //         cos(phi)*sin(psi),  d*cos(phi)*cos(psi), -d*sin(phi)*sin(psi), 0,
    //         0,                  0,                    1,                    0,
    //         0,                  0,                    0,                    1;
    //     return V;//将极坐标系下的测量噪声转换到笛卡尔坐标系，以便在扩展卡尔曼滤波中修正状态协方差
    // }

    Eigen::MatrixXd StandardObserver::getPredictiveMeasurement(const Eigen::MatrixXd &X, int i)
    {

        Eigen::Matrix<double, 4, 1> h;
        h << X(0, 0) + X(6 + i % 2, 0) * cos(X(8, 0) + i * M_PI / 2),//
             X(2, 0) + X(6 + i % 2, 0) * sin(X(8, 0) + i * M_PI / 2),
             X(4 + i % 2, 0),
             X(8, 0) + i * M_PI / 2;
        return h;
        
    }

    Eigen::MatrixXd StandardObserver::getMeasurementPD(const Eigen::MatrixXd &X, int i)
    {        

        Eigen::Matrix<double, 4, 10> H;
        H << 1, 0, 0, 0, 0,           0,     ((i + 1) % 2) * cos(X(8, 0) + i * M_PI / 2), (i % 2) * cos(X(8, 0) + i * M_PI / 2),-X(6 + i % 2, 0) * sin(X(8, 0) + i * M_PI / 2), 0,
             0, 0, 1, 0, 0,           0,     ((i + 1) % 2) * sin(X(8, 0) + i * M_PI / 2), (i % 2) * sin(X(8, 0) + i * M_PI / 2), X(6 + i % 2, 0) * cos(X(8, 0) + i * M_PI / 2), 0,
             0, 0, 0, 0, (i + 1) % 2, i % 2, 0,                                         0,                                   0,                                           0,
             0, 0, 0, 0, 0,           0,     0,                                         0,                                   1,                                           0;
                return H;
        
    }//预测观测值 h（4×1）对状态 X（10×1）的偏导数

    Eigen::MatrixXd StandardObserver::getMeasureNoisePD(const Eigen::MatrixXd &X, int i)
    {
        Eigen::Matrix<double, 4, 1> h = getPredictiveMeasurement(X, i);
        double psi = atan2(h(1, 0), h(0, 0));//装甲板在水平面上的方向
        double phi = atan2(h(2, 0), sqrt(pow(h(0, 0), 2) + pow(h(1, 0), 2)));//装甲板相对于水平面的仰角
        double d = sqrt(pow(h(0, 0), 2) + pow(h(1, 0), 2) + pow(h(2, 0), 2));//装甲板到观测点的欧氏距离
        Eigen::Matrix<double, 4, 4> V;
        V << -d * cos(phi) * sin(psi), -d * sin(phi) * cos(psi), cos(phi) * cos(psi), 0,
              d * cos(phi) * cos(psi), -d * sin(phi) * sin(psi), cos(phi) * sin(psi), 0,
              0,                        d * cos(phi),            sin(phi),            0,
              0,                        0,                       0,                   1;
        return V;
    }
//Balance//Balance//Balance//Balance//Balance//Balance//Balance//Balance//Balance//Balance//Balance//Balance//
    void BalanceObserver::reset()
    {
        m_init = false;

        m_P = Eigen::Matrix<double, 7, 7>::Identity() * 1e-5;
    }

    std::pair<Eigen::MatrixXd, Eigen::MatrixXd> BalanceObserver::predict(base_interfaces::msg::Armors &armors)
    {
        if (!m_init)
        {                                       
            for (auto &armor : armors.armors)
            {
                    m_X << armor.pose.position.x - m_init_radius * cos(armor.yaw_angle),
                           armor.pose.position.y - m_init_radius * sin(armor.yaw_angle),
                           0,
                           armor.pose.position.z,
                           m_init_radius,
                           armor.yaw_angle,
                           0;
                    m_init = true;
                    break;
            }
        }
                                
        // 预测
        Eigen::MatrixXd X_ = getTransformation(m_X);
        m_F = getTransformDP(m_X);
        m_Q = getTransformNoise(m_X);
        Eigen::MatrixXd P_ = m_F * m_P * m_F.transpose() + m_Q;
        if (m_debug)
        {
            // std::cout << "[BalanceObserver] X_: " << std::endl << X_ << std::endl;
            // std::cout << "[BalanceObserver] P_: " << std::endl << P_ << std::endl;
        }
        return std::make_pair(X_, P_);
    }

    std::shared_ptr<KinematicModel> BalanceObserver::update(std::vector<base_interfaces::msg::Armor> &armors, const Eigen::MatrixXd &X_, const Eigen::MatrixXd &P_, std::map<int, int> &match)
    {
        // 观测
        Eigen::MatrixXd Z;      // 实际观测量
        Eigen::MatrixXd h;      // 先验观测量
        Eigen::MatrixXd H;      // 观测方程偏导矩阵
        Eigen::MatrixXd V;      // 观测噪声偏导矩阵
        Eigen::MatrixXd K;      // 置信度权重矩阵
        Eigen::MatrixXd R;      // 时变观测噪声矩阵

        if (match.size() == 1)
        {
            int num = match.begin()->first;
            Z.resize(4, 1);
            Z << armors[num].pose.position.x, armors[num].pose.position.y, armors[num].pose.position.z, armors[num].yaw_angle;
            h.resize(4, 1);
            h << getPredictiveMeasurement(X_, match[num]);
            H.resize(4, 7);
            H << getMeasurementPD(X_, match[num]);
            V.resize(4, 4);
            V << getMeasureNoisePD(X_, match[num]);
            R = m_R;
            R(2, 2) *= m_gain * abs(Z(3, 0));

            K = P_ * H.transpose() * ((H * P_ * H.transpose() + V * R * V.transpose())).inverse();
        }
        else
        {
            // 若匹配数目不为1，则认为丢识别或者误判，直接返回先验状态
            m_X = X_;
            m_P = P_;

            return std::make_shared<BalanceModel>(m_X);
        }

        // if (m_debug)
        // {
        //     std::cout << "[BalanceObserver] Z: " << std::endl << Z << std::endl;
        //     std::cout << "[BalanceObserver] h: " << std::endl << h << std::endl;
        //     std::cout << "[BalanceObserver] H: " << std::endl << H << std::endl;
        // }

        // 更新
        Eigen::MatrixXd tmp = Z - h;
        tmp(3, 0) = _std_radian(tmp(3, 0));
        m_X_update = K * tmp;
        if (m_debug)
        {
            // std::cout << "[BalanceObserver] X_update: " << std::endl << m_X_update << std::endl;
        }

        m_X = X_ + m_X_update;
        m_P = (Eigen::Matrix<double, 7, 7>::Identity() - K * H) * P_;

        if (m_debug)
        {
            // std::cout << "[BalanceObserver] X:" << std::endl << m_X << std::endl;
            // std::cout << "[BalanceObserver] P:" << std::endl << m_P << std::endl;
        }

        return std::make_shared<BalanceModel>(m_X);
    }

    bool BalanceObserver::stable()
    {
        return true;
    }

    Eigen::MatrixXd BalanceObserver::getTransformation(const Eigen::MatrixXd &X)
    {
        Eigen::Matrix<double, 7, 1> X_;
        // if (X(6, 0) == 0)
        {
            X_ << X(0, 0) + X(2, 0) * cos(X(5, 0)) * m_dt,
                  X(1, 0) + X(2, 0) * sin(X(5, 0)) * m_dt,
                  X(2, 0),
                  X(3, 0),
                  X(4, 0),
                  X(5, 0) + X(6, 0) * m_dt,
                  X(6, 0);
        }
        // else
        // {
        //     X_ << X(0, 0) + X(2, 0) / X(6, 0) * (sin(X(6, 0) * m_dt) + X(5, 0) - sin(X(5, 0))),
        //           X(1, 0) - X(2, 0) / X(6, 0) * (cos(X(6, 0) * m_dt) - X(5, 0) + cos(X(5, 0))),
        //           X(2, 0),
        //           X(3, 0),
        //           X(4, 0),
        //           X(5, 0) + X(6, 0) * m_dt,
        //           X(6, 0);
        // }
        return X_;
    }

    Eigen::MatrixXd BalanceObserver::getTransformDP(const Eigen::MatrixXd &X)
    {
        Eigen::Matrix<double, 7, 7> F;
        // if (X(6, 0) == 0)
        {
            F << 1, 0, cos(X(5, 0)) * m_dt, 0, 0,-X(2, 0) * sin(X(5, 0)) * m_dt, 0,
                 0, 1, sin(X(5, 0)) * m_dt, 0, 0, X(2, 0) * cos(X(5, 0)) * m_dt, 0,
                 0, 0, 1,                   0, 0, 0,                             0,
                 0, 0, 0,                   1, 0, 0,                             0,
                 0, 0, 0,                   0, 1, 0,                             0,
                 0, 0, 0,                   0, 0, 1,                             m_dt,
                 0, 0, 0,                   0, 0, 0,                             1;
        }
        // else
        // {
        //     double cos_theta = cos(X(5, 0));
        //     double cos_theta_t = cos(X(6, 0) * m_dt + X(5, 0));
        //     double sin_theta = sin(X(5, 0));
        //     double sin_theta_t = sin(X(6, 0) * m_dt + X(5, 0));
        //     F << 1, 0, 1 / X(6, 0) * (sin_theta_t - sin_theta), 0, 0, X(2, 0) / X(6, 0) * (cos_theta_t - cos_theta), X(2, 0) / X(6, 0) * cos_theta_t * m_dt - X(2, 0) / pow(X(6, 0), 2) * (sin_theta_t - sin_theta),
        //          0, 1,-1 / X(6, 0) * (cos_theta_t - cos_theta), 0, 0, X(2, 0) / X(6, 0) * (sin_theta_t - sin_theta), X(2, 0) / X(6, 0) * sin_theta_t * m_dt + X(2, 0) / pow(X(6, 0), 2) * (cos_theta_t - cos_theta),
        //          0, 0, 1,                                       0, 0, 0,                                             0,
        //          0, 0, 0,                                       1, 0, 0,                                             0,
        //          0, 0, 0,                                       0, 1, 0,                                             0,
        //          0, 0, 0,                                       0, 0, 1,                                             m_dt,
        //          0, 0, 0,                                       0, 0, 0,                                             1;
        // }
        return F; 
    }

    Eigen::MatrixXd BalanceObserver::getTransformNoise(const Eigen::MatrixXd &X)
    {
        Eigen::Matrix<double, 7, 7> Q;
        double theta = X(5, 0);
        double v = X(2, 0);
        double dd = m_process_noise[0];
        double da = m_process_noise[1];
        double dz = m_process_noise[2];
        double dr = m_process_noise[3];
        double t5 = pow(m_dt, 5) / 5;
        double t4 = pow(m_dt, 4) / 4;
        double t3 = pow(m_dt, 3) / 3;
        double t2 = pow(m_dt, 2) / 2;
        double cos2 = pow(cos(theta), 2);
        double sin2 = pow(sin(theta), 2);
        double sincos = sin(theta) * cos(theta);
        Q << t5 * da * pow(v, 2) * sin2 / 4 + t3 * dd * cos2,         sin(theta) * t3 * dd - pow(v, 2) * sincos * t5 * da / 4, t2 * dd * cos(theta), 0,        0,        -t4 * da * v * sin(theta) / 2,-t3 * da * v * sin(theta) / 2,
             sin(theta) * t3 * dd - pow(v, 2) * sincos * t5 * da / 4, t3 * dd * sin2 + t5 * da * pow(v, 2) * cos2 / 4,         t2 * dd * sin(theta), 0,        0,         t4 * da * v * cos(theta) / 2, t3 * da * v * cos(theta) / 2,
             t2 * dd * cos(theta),                                    t2 * dd * sin(theta),                                    m_dt * dd,            0,        0,         0,                            0,
             0,                                                       0,                                                       0,                    m_dt *dz, 0,         0,                            0,
             0,                                                       0,                                                       0,                    0,        m_dt * dr, 0,                            0,
            -t4 * da * v * sin(theta) / 2,                            t4 * da * v * cos(theta) / 2,                            0,                    0,        0,         t3 * da,                      t2 * da,
            -t3 * da * v * sin(theta) / 2,                            t3 * da * v * cos(theta) / 2,                            0,                    0,        0,         t2 * da,                      m_dt * da;
        return Q;
    }

    Eigen::MatrixXd BalanceObserver::getPredictiveMeasurement(const Eigen::MatrixXd &X, int i)
    {
        Eigen::Matrix<double, 4, 1> h;
        h << X(0, 0) + X(4, 0) * cos(X(5, 0) + i * M_PI),
             X(1, 0) + X(4, 0) * sin(X(5, 0) + i * M_PI),
             X(3, 0),
             X(5, 0) + i * M_PI;
        return h;
    }

    Eigen::MatrixXd BalanceObserver::getMeasurementPD(const Eigen::MatrixXd &X, int i)
    {
        Eigen::Matrix<double, 4, 7> H;
        H << 1, 0, 0, 0, cos(X(5, 0) + i * M_PI),-X(4, 0) * sin(X(5, 0) + i * M_PI), 0,
             0, 1, 0, 0, sin(X(5, 0) + i * M_PI), X(4, 0) * cos(X(5, 0) + i * M_PI), 0,
             0, 0, 0, 1, 0,                     0,                               0,
             0, 0, 0, 0, 0,                     1,                               0;
        return H;
    }

    Eigen::MatrixXd BalanceObserver::getMeasureNoisePD(const Eigen::MatrixXd &X, int i)
    {
        Eigen::Matrix<double, 4, 1> h = getPredictiveMeasurement(X, i);
        double psi = atan2(h(1, 0), h(0, 0));
        double phi = atan2(h(2, 0), sqrt(pow(h(0, 0), 2) + pow(h(1, 0), 2)));
        double d = sqrt(pow(h(0, 0), 2) + pow(h(1, 0), 2) + pow(h(2, 0), 2));
        Eigen::Matrix<double, 4, 4> V;
        V << -d * cos(phi) * sin(psi), -d * sin(phi) * cos(psi), cos(phi) * cos(psi), 0,
              d * cos(phi) * cos(psi), -d * sin(phi) * sin(psi), cos(phi) * sin(psi), 0,
              0,                        d * cos(phi),            sin(phi),            0,
              0,                        0,                       0,                   1;
        return V;
    }
}

