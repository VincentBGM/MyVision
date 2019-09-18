#include <iostream>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <ctime>

#include <sophus/so3.h>

using namespace std;

int main(int argc, char** argv)
{
    // 旋转矩阵 扰动更新
    Eigen::Matrix3d Rotation = Eigen::Matrix3d::Identity();
    Sophus::SO3 SO3_Rotation (Rotation);    // 初始化SO3
    Eigen::Vector3d w(0.01, 0.02, 0.03);
    // 更新 SO3 更新是指数映射
    Sophus::SO3 Rotation_update = SO3_Rotation * Sophus::SO3::exp(w);

    // 四元数 绕动更新
    Eigen::Quaterniond quaternin(Rotation); // 初始化四元数
    Eigen::Quaterniond quaternion_update(1, 0.01/2, 0.02/2, 0.03/3);    // 四元数 [1, w/2]
    quaternion_update.normalize();  // 四元数一定要归一化
    // 更新 q * [1, w/2]
    quaternin = quaternin * quaternion_update;


    cout << "SO3的更新结果： " << endl << Rotation_update.unit_quaternion().coeffs() << endl;
    cout << "四元数更新结果： " << endl << quaternin.coeffs() << endl;

    return 0;
}
