#include <iostream>
#include <Eigen/Core>
#include <sophus/se3.h>
#include <sophus/so3.h>


using namespace std;
//using namespace Eigen;
//using namespace Sophus;



int main(int argc, char** argv)
{
    // 沿着z轴旋转90度的旋转向量
    Eigen::AngleAxisd a1(M_PI / 2, Eigen::Vector3d(0, 0, 1));
    Eigen::Matrix3d r1 = a1.matrix();
    Eigen::Quaterniond q1(a1);

    // 1、初始化李群（so3）s
    //    3位的旋转矩阵 初始化 李群
    Sophus::SO3 so3_r(r1);
    cout << " so(3) so3_r李群 矩阵：  " << so3_r << endl;  // 从输出结果可以看到李群对应的李代数就是旋转角


    //    4位的变化矩阵（四元数） 初始化 李群
    Sophus::SO3 so3_q(q1);
    cout << "so(3) so3_q李群 矩阵：" << so3_q << endl;    // 从输出结果来看， 不管是四元数也是对应李代数的旋转角

    // 2、使用旋转角（轴角） 的各个元素对应的代数值来初始化李群
    //    注意：直接使用旋转角AngleAxis或旋转角度对应的向量(Vector3d=AngleAxis.axis()*AngleAxis.angle())对李群进行初始化是不行的
    //         因为so3李群没有对应的构造函数
    //    错误方法：
//              Sophus::SO3 SO3_a(q1); //直接使用旋转角对李群初始化
//              Sophus::SO3 SO3_a(q1.axis()*a1.angle()); //直接使用旋转角度对应的向量
//        正确方法：
//              对李群进行初始化只能使用旋转角对应的向量的每一个维度进行赋值，
//              对应于SO3的这样一个构造函数SO3(double rot_x, double rot_y, double rot_z);
    //    使用旋转角度对应的向量(Vector3d=AngleAxis.axis()*AngleAxis.angle())中的各个元素对李群进行初始化
    Sophus::SO3 so3_a( (a1.axis() * a1.angle())(0), (a1.axis() * a1.angle())(1), (a1.axis() * a1.angle())(2) );
    cout << "旋转角度 初始化 李群：" << so3_a << endl;

    //    使用旋转角度对应的向量(Vector3d=AngleAxis.axis()*AngleAxis.angle())中的各个元素对李群进行初始化
    Sophus::SO3 so3_a1( M_PI / 2 * 0, M_PI / 2 * 0, M_PI / 2 * 1);
    cout << "旋转角度向量 初始化 李群: " << so3_a1 << endl;

    // 旋转角（轴角）与李代数so(3)对应,所以直接使用旋转角的值获得se(3),进而再通过Sophus::SO3::exp()获得对应的SO(3)
    Eigen::Vector3d v(0, 0, M_PI / 2);
    Sophus::SO3 so3_a2 = Sophus::SO3::exp(v);
    cout << "通过旋转角::exp() 初始化 李群: " << so3_a2 << endl;


    // 3、SO(3)与so(3)的相互转换，以及so3对应的hat和vee操作
    //    so(3)在Sophus(Eigen)中用vector3d表示,使用对数映射获得李群对应的李代数
    Eigen::Vector3d so3_v1 = so3_a2.log();
    cout << "对数映射获取李代数：" << so3_v1.transpose() << endl;

    //    使用指数映射将李代数 转化为 李群
    Sophus::SO3 so3_v2 = Sophus::SO3::exp(so3_v1);
    cout << "对数映射 李代数 转 李群： " << so3_v2 << endl;


    //    hat为向量到其对应的反对称矩阵
    Eigen::Matrix3d m_so3_v1 = Sophus::SO3::hat(so3_v1);
    cout << "向量转到 反对称矩阵： " << m_so3_v1 << endl;

    //    vee为反对称矩阵对应的向量
    Eigen::Vector3d v_m = Sophus::SO3::vee(m_so3_v1);
    cout << "反对称矩阵 转 向量： "  << v_m << endl;

    // 4、增量扰动模型
    Eigen::Vector3d update_so3(1e-4, 0, 0);     // 更新量
    Eigen::Matrix3d update_matrix = Sophus::SO3::exp(update_so3).matrix();      // 将李群转换为旋转矩阵
    cout << "so3 更新： " << update_matrix << endl;

    Sophus::SO3 so3_update = Sophus::SO3::exp(update_so3) * so3_r;
    cout << "so3 update = \n " << so3_update << endl;

    Eigen::Matrix3d so3_update_matrix = so3_update.matrix();    // 将李群转换为旋转矩阵
    cout << "so3 update_matix = \n" << so3_update_matrix << endl;
















    cout << "Hello World!" << endl;
    return 0;
}
