#include "aimer/KinematicModel.hpp"

namespace rm_auto_aim
{
    KinematicModel::KinematicModel()
    :   index(0)
    {
        height.clear();
        radius.clear();
    }
    KinematicModel::~KinematicModel()
    {}


    StandardModel::StandardModel()
    :   StandardModel(Eigen::Matrix<double, 10, 1>::Zero())
    {}
    StandardModel::StandardModel(const Eigen::Matrix<double, 10, 1> &X)
    :   KinematicModel()
    {
        height.resize(2);//
        radius.resize(2);
        number = 4;

        center.x = X(0, 0);  // 手动赋值 x
        center.y = X(2, 0);  // 手动赋值 y
        velocity.x = X(1, 0);  // 根据实际数据索引调整
        velocity.y = X(3, 0);
        height[0] = X(4, 0);//高度
        height[1] = X(5, 0);
        radius[0] = X(6, 0);
        radius[1] = X(7, 0);
        phase = X(8, 0);//相位
        palstance = X(9, 0);//角速度
    }
    StandardModel::~StandardModel()
    {}

    StandardModel StandardModel::operator=(const StandardModel &status)
    {
        int tmp_index = this->index;
        *this = status;
        this->index = tmp_index;
        return *this;
    }

    BalanceModel::BalanceModel()
    :   BalanceModel(Eigen::Matrix<double, 7, 1>::Zero())
    {}
    BalanceModel::BalanceModel(const Eigen::Matrix<double, 7, 1> &X)
    :   KinematicModel()
    {
        height.resize(1);
        radius.resize(1);
        number = 2;

        center.x = X(0, 0);  // 手动赋值 x
        center.y = X(1, 0);  // 手动赋值 y
        velocity.x = X(0, 0);  // 假设 X(0,0) 是速度的 x 分量
        velocity.y = X(1, 0);  // 假设 X(1,0) 是速度的 y 分量
        height[0] = X(3, 0);
        radius[0] = X(4, 0);
        phase = X(5, 0);
        palstance = X(6, 0);
    }
    BalanceModel::~BalanceModel()
    {}

    BalanceModel BalanceModel::operator=(const BalanceModel &status)
    {
        int tmp_index = this->index;
        *this = status;
        this->index = tmp_index;
        return *this;
    }
}   // HL
