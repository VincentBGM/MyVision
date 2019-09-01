#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <cmath>       //

using namespace std;

// 扩展：
// 1、弧度 = 角度 × M_PI / 180
// 2、角度 = 弧度 × 180 / M_PI
// 3、三维空间坐标变化有3种实现： 旋转向量 和 旋转矩阵   欧拉角   四元数
// 4、
//    R_roll  = [1,         0,          0
//               0, cos(roll), -sin(roll)
//               0, sin(roll),  cos(roll)]

//    R_pithc = [cos(pithc),    0,  sin(pitch)
//                        0,    1,  0
//              -sin(pithc),    0,  cos(pithc)]

//    R_yaw   = [cos(yaw),    -sin(yaw),  0
//               sin(yaw),     cos(yaw),  0
//                      0,            0,  1]
// 5、四元数(x, y, z, w); ax, ay, zy 是矢量
//    w = cos( theta / 2 )
//    x = ax * sin( theta / 2 )
//    y = ay * sin( theta / 2 )
//    z = az * sin( theta / 2 )


void eigenDemo1()
{
    // ======================初始化=======================

    // 欧拉角 3*1
    Eigen::Vector3d ruler_vector = Eigen::Vector3d(M_PI / 4, 0, 0);   // ZYX 顺序
    cout << "欧拉角： " << endl << ruler_vector << endl;
    cout << "欧拉角转置后： " << endl << ruler_vector.transpose() << endl;

    // 旋转向量 3×1
    Eigen::AngleAxisd rotation_vector(M_PI / 4, Eigen::Vector3d(0, 0, 1));  //`XYZ顺序
//    cout << "旋转向量为： " << rotation_vector << endl;
    cout << "旋转向量转换成旋转矩阵1： " << endl << rotation_vector.matrix() << endl;                          // 转换成旋转矩阵
    cout << "旋转向量转换成旋转矩阵2： " << endl << rotation_vector.toRotationMatrix() << endl;    // 转换成旋转矩阵
    cout << "旋转向量的旋转轴为： " << rotation_vector.axis() << endl;
    cout << "旋转向量的旋转角为： " << rotation_vector.angle() << endl;

    // 旋转矩阵 3×3
    Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();
    cout << "旋转矩阵单位矩阵： " << endl << rotation_matrix << endl;

    // 四元数 4×1
    Eigen::Quaterniond quat = Eigen::Quaterniond(0, 0, 0.383, 0.924);
    cout << "四元数输出： " << endl << quat.coeffs() << endl;   // 输出时（x, y, z, w） 最后一个是实部
    cout << "四元数输出单个元素： " << endl;
    cout << "四元数x = " << quat.x() << endl;
    cout << "四元数y = " << quat.y() << endl;
    cout << "四元数z = " << quat.z() << endl;
    cout << "四元数w = " << quat.w() << endl;

}

void eigenDemo02()
{
    // 旋转向量===> 旋转矩阵
    Eigen::AngleAxisd rotation_vector(M_PI / 4, Eigen::Vector3d(0, 0, 1));
    Eigen::Matrix3d rotation_matrix;
    rotation_vector = rotation_vector.toRotationMatrix();   // 方法 一
//    rotation_vector = rotation_vector.matrix();           // 方法 二

    // 旋转向量====> 四元数
    Eigen::Quaterniond quat;
    quat = Eigen::Quaterniond(rotation_vector);
    cout << "旋转向量转换成四元数： " << endl << quat.coeffs() << endl;

}

void eigenDemo03()
{
    Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();
    rotation_matrix <<  0.707, -0.707,      0,
                        0.707,  0.707,      0,
                        0,      0,          1;
    cout << "旋转矩阵 = " << endl << rotation_matrix << endl;

    // 旋转矩阵====> 旋转向量
    Eigen::Vector3d rotation_vector;
    rotation_vector.fromRotationMatrix(rotation_matrix);
    cout << "旋转向量为： " << rotation_vector.axis() << endl;

    // 旋转矩阵====> 四元数
    Eigen::Quaterniond quat;
    quat = Eigen::Quaterniond(rotation_matrix);
    cout << "旋转矩阵转换成四元数： " << endl << quat.coeffs() << endl;
}

int main(int argc, char** argv)
{
    // 初始化
    eigenDemo1();

    eigenDemo02();

    eigenDemo03();

    cout << "Hello World!" << endl;
    return 0;
}
