#pragma once

#include "aimer/KinematicModel.hpp"
#include <vector>
#include <string>
#include <cmath>         // 用于round函数和PI常量
#include <opencv2/core.hpp>  // 用于cv::Point2d
#include <Eigen/Dense>       // 用于Eigen::Matrix
#include "base_interfaces/msg/armor.hpp"
#include "base_interfaces/msg/armors.hpp"
#include <map>
#include <iostream>
#include "rclcpp/rclcpp.hpp" // ROS2节点支持
namespace rm_auto_aim
{
    enum class ArmorColor { WHITE = 0, BLUE = 1, RED = 2 };
    // class ModelObserver : public rclcpp::Node
    // {
    // public:
    //     explicit ModelObserver(const rclcpp::NodeOptions& options) : rclcpp::Node("model_observer", options) {}

    // using rclcpp::Node::Node;// 正确继承基类构造函数

    // // 显式声明构造函数为 protected，允许派生类继承
    //     //using ModelObserver::ModelObserver;  // 继承所有构造函数

    //     virtual std::pair<Eigen::MatrixXd, Eigen::MatrixXd> predict(base_interfaces::msg::Armors &armors) = 0;
    //     virtual std::shared_ptr<KinematicModel> update(std::vector<base_interfaces::msg::Armor> &armors, const Eigen::MatrixXd &X_, const Eigen::MatrixXd &P_, std::map<int, int> &match) = 0;

    //     virtual void reset() = 0;
    //     virtual bool stable() = 0;

    // protected:

    //     virtual Eigen::MatrixXd getPredictiveMeasurement(const Eigen::MatrixXd &X, int i) = 0;
    //     virtual Eigen::MatrixXd getMeasurementPD(const Eigen::MatrixXd &X, int i) = 0;
    //     virtual Eigen::MatrixXd getMeasureNoisePD(const Eigen::MatrixXd &X, int i) = 0;

    // protected:
    //     bool m_debug;
    //     bool m_init;

    //     // 参数
    //     double m_dt;                                // 单位时间
    //     double m_init_radius;                       // 初始半径
    //     double m_gain;                              // 测距噪声关于角度的增益倍数

    //     double m_process_noise[4];                  // 状态转移噪声系数
    //     double m_measure_noise[3];                  // 观测噪声系数

    //     Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> m_X;          // 状态
    //     Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> m_X_update;   // 状态修正
    //     Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> m_P;          // 状态协方差
    //     Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> m_F;          // 状态转移矩阵
    //     Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> m_Q;          // 状态转移噪声
    //     Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> m_R;          // 观测噪声
    // };

    // class StandardObserver : public ModelObserver {
    // public:
    // // using ModelObserver::ModelObserver;  // 继承所有构造函数
    // explicit StandardObserver(const rclcpp::NodeOptions& options);
    // // ~StandardObserver();
    //     /**
    //      * @brief 先验预测
    //      * @param armors 绝对坐标系装甲板序列
    //      * @return std::pair<Eigen::Matrix<double, 10, 1>, Eigen::Matrix<double, 10, 10>>(X_, P_)
    //      */
    //     std::pair<Eigen::MatrixXd, Eigen::MatrixXd> predict(base_interfaces::msg::Armors &armors);

    //     /**
    //      * @brief 匹配装甲板序列和标准装甲板
    //      * @param armors 绝对坐标系装甲板序列
    //      * @param X_ Eigen::Matrix<double, 10, 1> 先验状态
    //      * @param P_ Eigen::Matrix<double, 10, 10> 先验状态协方差
    //      * @param match 装甲板关联
    //      * @return 运动状态
    //      */
    //     std::shared_ptr<KinematicModel> update(std::vector<base_interfaces::msg::Armor> &armors, const Eigen::MatrixXd &X_, const Eigen::MatrixXd &P_, std::map<int, int> &match);

    //     /**
    //      * @brief 重置
    //      */
    //     void reset();

    //     /**
    //      * @return 模型是否稳定
    //      */
    //     bool stable();

    // private:
    //     /**
    //      * @brief 获取先验观测量
    //      * @param X Eigen::Matrix<double, 10, 1> 先验状态
    //      * @param i 标准装甲板索引
    //      * @return Eigen::Matrix<double, 4, 1> 先验观测矩阵
    //      */
    //     Eigen::MatrixXd getPredictiveMeasurement(const Eigen::MatrixXd &X, int i);

    //     /**
    //      * @brief 获取观测方程偏导矩阵
    //      * @param X Eigen::Matrix<double, 10, 1> 先验状态
    //      * @param i 标准装甲板索引
    //      * @return Eigen::Matrix<double, 4, 10> 观测偏导矩阵
    //      */
    //     Eigen::MatrixXd getMeasurementPD(const Eigen::MatrixXd &X, int i);

    //     /**
    //      * @brief 获取观测噪声偏导矩阵
    //      * @param X Eigen::Matrix<double, 10, 1> 先验状态
    //      * @param i 标准装甲板索引
    //      * @return Eigen::Matrix<double, 4, 4> 观测偏导矩阵
    //      */
    //     Eigen::MatrixXd getMeasureNoisePD(const Eigen::MatrixXd &X, int i);

    // private:
    //     Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> m_RR;         // 增广观测噪声
    // };

    // class BalanceObserver : public ModelObserver
    // {
    // public:
    //     //using ModelObserver::ModelObserver;  // 继承基类所有构造函数
    //     explicit BalanceObserver(const rclcpp::NodeOptions& options);
    //     // ~BalanceObserver();

    //     /**
    //      * @brief 先验预测
    //      * @param armors 绝对坐标系装甲板序列
    //      * @return std::pair<Eigen::Matrix<double, 7, 1>, Eigen::Matrix<double, 7, 7>>(X_, P_)
    //      */
    //     std::pair<Eigen::MatrixXd, Eigen::MatrixXd> predict(base_interfaces::msg::Armors &armors);

    //     /**
    //      * @brief 匹配装甲板序列和标准装甲板
    //      * @param armors 绝对坐标系装甲板序列
    //      * @param X_ Eigen::Matrix<double, 7, 1> 先验状态
    //      * @param P_ Eigen::Matrix<double, 7, 7> 先验状态协方差
    //      * @param match 装甲板关联
    //      * @return 运动状态
    //      */
    //     std::shared_ptr<KinematicModel> update(std::vector<base_interfaces::msg::Armor> &armors, const Eigen::MatrixXd &X_, const Eigen::MatrixXd &P_, std::map<int, int> &match);

    //     /**
    //      * @brief 重置
    //      */
    //     void reset();

    //     /**
    //      * @return 模型是否稳定
    //      */
    //     bool stable();

    // private:
    //     /**
    //      * @brief 读取参数配置文件
    //      * @param file_path 配置文件路径
    //      */

    //     /**
    //      * @brief 获取先验状态
    //      * @param X Eigen::Matrix<double, 7, 1> 上一时刻状态
    //      * @return Eigen::Matrix<double, 7, 1> 先验状态
    //      */
    //     Eigen::MatrixXd getTransformation(const Eigen::MatrixXd &X);

    //     /**
    //      * @brief 获取状态转移偏导矩阵
    //      * @param X Eigen::Matrix<double, 7, 1> 先验状态
    //      * @return Eigen::Matrix<double, 7, 7> 状态转移偏导矩阵
    //      */
    //     Eigen::MatrixXd getTransformDP(const Eigen::MatrixXd &X);

    //     /**
    //      * @brief 获取状态转移噪声矩阵
    //      * @param X Eigen::Matrix<double, 7, 1> 先验状态
    //      * @return Eigen::Matrix<double, 7, 7> 状态转移噪声矩阵
    //      */
    //     Eigen::MatrixXd getTransformNoise(const Eigen::MatrixXd &X);

    //     /**
    //      * @brief 获取先验观测量
    //      * @param X Eigen::Matrix<double, 7, 1> 先验状态
    //      * @param i 标准装甲板索引
    //      * @return Eigen::Matrix<double, 4, 1> 先验观测矩阵
    //      */
    //     Eigen::MatrixXd getPredictiveMeasurement(const Eigen::MatrixXd &X, int i);

    //     /**
    //      * @brief 获取观测方程偏导矩阵
    //      * @param X Eigen::Matrix<double, 7, 1> 先验状态
    //      * @param i 标准装甲板索引
    //      * @return Eigen::Matrix<double, 4, 7> 观测偏导矩阵
    //      */
    //     Eigen::MatrixXd getMeasurementPD(const Eigen::MatrixXd &X, int i);

    //     /**
    //      * @brief 获取观测噪声偏导矩阵
    //      * @param X Eigen::Matrix<double, 7, 1> 先验状态
    //      * @param i 标准装甲板索引
    //      * @return Eigen::Matrix<double, 4, 4> 观测偏导矩阵
    //      */
    //     Eigen::MatrixXd getMeasureNoisePD(const Eigen::MatrixXd &X, int i);

    // private:
    //     Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> m_F;          // 状态转移偏导矩阵
    // };
    class ModelObserver
    {
    public:
        ModelObserver();
        ~ModelObserver();

        virtual std::pair<Eigen::MatrixXd, Eigen::MatrixXd> predict(base_interfaces::msg::Armors &armors) = 0;
         virtual std::shared_ptr<KinematicModel> update(std::vector<base_interfaces::msg::Armor> &armors, const Eigen::MatrixXd &X_, const Eigen::MatrixXd &P_, std::map<int, int> &match) = 0;

        virtual void reset() = 0;
        virtual bool stable() = 0;

    protected:
       

        virtual Eigen::MatrixXd getPredictiveMeasurement(const Eigen::MatrixXd &X, int i) = 0;
        virtual Eigen::MatrixXd getMeasurementPD(const Eigen::MatrixXd &X, int i) = 0;
        virtual Eigen::MatrixXd getMeasureNoisePD(const Eigen::MatrixXd &X, int i) = 0;

    protected:
        bool m_debug;
        bool m_init;

        // 参数
        double m_dt;                                // 单位时间
        double m_init_radius;                       // 初始半径
        double m_gain;                              // 测距噪声关于角度的增益倍数

       std::array<double, 4> m_process_noise;  
       std::array<double, 3> m_measure_noise;  

        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> m_X;          // 状态
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> m_X_update;   // 状态修正
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> m_P;          // 状态协方差
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> m_F;          // 状态转移矩阵
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> m_Q;          // 状态转移噪声
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> m_R;          // 观测噪声
    };

    class StandardObserver : public ModelObserver
    {
    public:
        StandardObserver(
        bool debug, 
        double dt,
        double init_radius,
        double gain,
        const std::array<double,4>& process_noise,
        const std::array<double,3>& measure_noise)
        {
            m_debug = debug;
            m_dt = dt;
            m_init_radius = init_radius;
            m_gain = gain;
            m_process_noise = process_noise;
            m_measure_noise = measure_noise;
            
            
            m_X.resize(10, 1);
            m_X_update.resize(10, 1);
            m_P.resize(10, 10);
            m_F.resize(10, 10);
            m_Q.resize(10, 10);
            m_R.resize(4, 4);
            m_RR.resize(8, 8);
            m_init = false;
            m_P = Eigen::Matrix<double, 10, 10>::Identity() * 1e-5;
            
        m_F << 1, m_dt, 0, 0,    0, 0, 0, 0, 0, 0,
               0, 1,    0, 0,    0, 0, 0, 0, 0, 0,
               0, 0,    1, m_dt, 0, 0, 0, 0, 0, 0,
               0, 0,    0, 1,    0, 0, 0, 0, 0, 0,
               0, 0,    0, 0,    1, 0, 0, 0, 0, 0,
               0, 0,    0, 0,    0, 1, 0, 0, 0, 0,
               0, 0,    0, 0,    0, 0, 1, 0, 0, 0,
               0, 0,    0, 0,    0, 0, 0, 1, 0, 0,
               0, 0,    0, 0,    0, 0, 0, 0, 1, m_dt,
               0, 0,    0, 0,    0, 0, 0, 0, 0, 1;

            double dd = m_process_noise[0];//​​位移高阶变化噪声​​ 值越大，滤波器对突变运动响应越快
            double da = m_process_noise[1];//角度高阶变化噪声 值越大，对旋转速度变化更敏感
            double dz = m_process_noise[2];//​​高度变化噪声​ 控制高度方向的滤波平滑度
            double dr = m_process_noise[3];//半径变化噪声​ 影响装甲板距离中心的半径估计稳定性
            double t4 = pow(m_dt, 3) / 3;
            double t3 = pow(m_dt, 2) / 2;
            double t2 = pow(m_dt, 1);
            m_Q << t4 * dd, t3 * dd, 0,       0,       0,  0, 0,  0,  0,       0,
                    t3 * dd, t2 * dd, 0,       0,       0,  0, 0,  0,  0,       0,
                    0,       0,       t4 * dd, t3 * dd, 0,  0, 0,  0,  0,       0,
                    0,       0,       t3 * dd, t2 * dd, 0,  0, 0,  0,  0,       0,
                    0,       0,       0,       0,       dz, 0, 0,  0,  0,       0,
                    0,       0,       0,       0,       0, dz, 0,  0,  0,       0,
                    0,       0,       0,       0,       0,  0, dr, 0,  0,       0,
                    0,       0,       0,       0,       0,  0, 0,  dr, 0,       0,
                    0,       0,       0,       0,       0,  0, 0,  0,  t4 * da, t3 * da,
                    0,       0,       0,       0,       0,  0, 0,  0,  t3 * da, t2 * da;

            Eigen::VectorXd measurement_noise4(4);
            measurement_noise4 << m_measure_noise[0],
                                    m_measure_noise[0],
                                    m_measure_noise[1],
                                    m_measure_noise[2];
            m_R = measurement_noise4.asDiagonal();
            Eigen::VectorXd measurement_noise8(8);
            measurement_noise8 << measurement_noise4,
                                    measurement_noise4;
            m_RR = measurement_noise8.asDiagonal();




        }
        ~StandardObserver();

        /**
         * @brief 先验预测
         * @param armors 绝对坐标系装甲板序列
         * @return std::pair<Eigen::Matrix<double, 10, 1>, Eigen::Matrix<double, 10, 10>>(X_, P_)
         */
        std::pair<Eigen::MatrixXd, Eigen::MatrixXd> predict(base_interfaces::msg::Armors &armors);

        /**
         * @brief 匹配装甲板序列和标准装甲板
         * @param armors 绝对坐标系装甲板序列
         * @param X_ Eigen::Matrix<double, 10, 1> 先验状态
         * @param P_ Eigen::Matrix<double, 10, 10> 先验状态协方差
         * @param match 装甲板关联
         * @return 运动状态
         */
         std::shared_ptr<KinematicModel> update(std::vector<base_interfaces::msg::Armor> &armors, const Eigen::MatrixXd &X_, const Eigen::MatrixXd &P_, std::map<int, int> &match);

        /**
         * @brief 重置
         */
        void reset();

        /**
         * @return 模型是否稳定
         */
        bool stable();

    private:
        /**
         * @brief 读取参数配置文件
         * @param file_path 配置文件路径
         */
        

        /**
         * @brief 获取先验观测量
         * @param X Eigen::Matrix<double, 10, 1> 先验状态
         * @param i 标准装甲板索引
         * @return Eigen::Matrix<double, 4, 1> 先验观测矩阵
         */
        Eigen::MatrixXd getPredictiveMeasurement(const Eigen::MatrixXd &X, int i);

        /**
         * @brief 获取观测方程偏导矩阵
         * @param X Eigen::Matrix<double, 10, 1> 先验状态
         * @param i 标准装甲板索引
         * @return Eigen::Matrix<double, 4, 10> 观测偏导矩阵
         */
        Eigen::MatrixXd getMeasurementPD(const Eigen::MatrixXd &X, int i);

        /**
         * @brief 获取观测噪声偏导矩阵
         * @param X Eigen::Matrix<double, 10, 1> 先验状态
         * @param i 标准装甲板索引
         * @return Eigen::Matrix<double, 4, 4> 观测偏导矩阵
         */
        Eigen::MatrixXd getMeasureNoisePD(const Eigen::MatrixXd &X, int i);

    private:
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> m_RR;         // 增广观测噪声
    };

    class BalanceObserver : public ModelObserver
    {
    public:
        BalanceObserver(
        bool debug, 
        double dt,
        double init_radius,
        double gain,
        const std::array<double,4>& process_noise,
        const std::array<double,3>& measure_noise
        ){
            m_debug = debug;
            m_dt = dt;
            m_init_radius = init_radius;
            m_gain = gain;
            m_process_noise = process_noise;
            m_measure_noise = measure_noise;

            m_X.resize(10, 1);
            m_X_update.resize(10, 1);
            m_P.resize(10, 10);
            m_F.resize(10, 10);
            m_Q.resize(10, 10);
            m_R.resize(4, 4);
            m_R.resize(8, 8);
        }
        ~BalanceObserver();

        /**
         * @brief 先验预测
         * @param armors 绝对坐标系装甲板序列
         * @return std::pair<Eigen::Matrix<double, 7, 1>, Eigen::Matrix<double, 7, 7>>(X_, P_)
         */
         std::pair<Eigen::MatrixXd, Eigen::MatrixXd> predict(base_interfaces::msg::Armors &armors);

        /**
         * @brief 匹配装甲板序列和标准装甲板
         * @param armors 绝对坐标系装甲板序列
         * @param X_ Eigen::Matrix<double, 7, 1> 先验状态
         * @param P_ Eigen::Matrix<double, 7, 7> 先验状态协方差
         * @param match 装甲板关联
         * @return 运动状态
         */
         std::shared_ptr<KinematicModel> update(std::vector<base_interfaces::msg::Armor> &armors, const Eigen::MatrixXd &X_, const Eigen::MatrixXd &P_, std::map<int, int> &match);

        /**
         * @brief 重置
         */
        void reset();

        /**
         * @return 模型是否稳定
         */
        bool stable();

    private:
        /**
         * @brief 读取参数配置文件
         * @param file_path 配置文件路径
         */
        void setParam(const std::string &file_path);

        /**
         * @brief 获取先验状态
         * @param X Eigen::Matrix<double, 7, 1> 上一时刻状态
         * @return Eigen::Matrix<double, 7, 1> 先验状态
         */
        Eigen::MatrixXd getTransformation(const Eigen::MatrixXd &X);

        /**
         * @brief 获取状态转移偏导矩阵
         * @param X Eigen::Matrix<double, 7, 1> 先验状态
         * @return Eigen::Matrix<double, 7, 7> 状态转移偏导矩阵
         */
        Eigen::MatrixXd getTransformDP(const Eigen::MatrixXd &X);

        /**
         * @brief 获取状态转移噪声矩阵
         * @param X Eigen::Matrix<double, 7, 1> 先验状态
         * @return Eigen::Matrix<double, 7, 7> 状态转移噪声矩阵
         */
        Eigen::MatrixXd getTransformNoise(const Eigen::MatrixXd &X);

        /**
         * @brief 获取先验观测量
         * @param X Eigen::Matrix<double, 7, 1> 先验状态
         * @param i 标准装甲板索引
         * @return Eigen::Matrix<double, 4, 1> 先验观测矩阵
         */
        Eigen::MatrixXd getPredictiveMeasurement(const Eigen::MatrixXd &X, int i);

        /**
         * @brief 获取观测方程偏导矩阵
         * @param X Eigen::Matrix<double, 7, 1> 先验状态
         * @param i 标准装甲板索引
         * @return Eigen::Matrix<double, 4, 7> 观测偏导矩阵
         */
        Eigen::MatrixXd getMeasurementPD(const Eigen::MatrixXd &X, int i);

        /**
         * @brief 获取观测噪声偏导矩阵
         * @param X Eigen::Matrix<double, 7, 1> 先验状态
         * @param i 标准装甲板索引
         * @return Eigen::Matrix<double, 4, 4> 观测偏导矩阵
         */
        Eigen::MatrixXd getMeasureNoisePD(const Eigen::MatrixXd &X, int i);

    private:
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> m_F;          // 状态转移偏导矩阵
    };



}   // HL
