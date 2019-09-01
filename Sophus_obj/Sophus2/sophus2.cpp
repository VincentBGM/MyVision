#include <iostream>
#include <Eigen/Core>
#include <sophus/so3.h>
#include <sophus/se3.h>


using namespace std;



int main(int argc, char** argv)
{

    // 沿着z轴旋转90度 的旋转矩阵
    Eigen::AngleAxisd A1(M_PI/4, Eigen::Vector3d(0, 0, 1));     // 欧拉角
    Eigen::Matrix3d R1 = A1.matrix();   // 旋转矩阵
    Eigen::Quaterniond Q1(A1);          // 四元数
    Eigen::Vector3d T1(1, 2, 3);

    cout << "================================SO(3)空间========================================" << endl;
    // 一、=============================>初始化李群的方式
    //    =====================================================>SO(3)（特殊正交群 只有旋转矩阵）
    // 1、使用旋转矩阵初始化李群 ：SO(3) 对应的李代数so(3) 就是旋转角
    Sophus::SO3 SO3_R(R1);
    cout << "SO3初始化，是旋转矩阵玩为: \n" << SO3_R << endl;

    // 2、使用四元数初始化李群：SO3
    Sophus::SO3 SO3_Q(Q1);
    cout << "SO3初始化， 是四元数为： \n" << SO3_Q << endl;

    // 3、使用旋转角（轴角）对应的向量中的各个元素 初始化李群 SO3      Vector3d=AngleAxis.axis()*AngleAxis.angle()
    Sophus::SO3 SO3_A1( (A1.axis() * A1.angle())(0), (A1.axis() * A1.angle())(1), (A1.axis() * A1.angle())(2));
    cout << "SO3初始化， 是旋转角对应的向量时： \n" << SO3_A1 << endl;

    // 4、使用旋转角（轴角）对应的向量中的各个元素 初始化李群 SO3
    Sophus::SO3 SO3_A2( M_PI/4 * 0, M_PI/4 *0, M_PI/4 * 1);
    cout << "SO3初始化，是旋转角度时： \n" << SO3_A2 << endl;

    // 5、由于旋转角（轴角）与李代数so(3)对应,所以直接使用旋转角的值获得se(3),进而再通过Sophus::SO3::exp()获得对应的SO(3)
    Eigen::Vector3d V1(0, 0, M_PI / 4);
    Sophus::SO3 SO3_V1 = Sophus::SO3::exp(V1);      //
    cout << "SO3初始化，是so3李代数时： \n" << SO3_V1 << endl;

    //    =====================================================>SE(3)（特殊欧式群 有旋转 有平移）
    // 1、使用旋转矩阵 平移向量 初始化SE(3)
    Sophus::SE3 SE3_RT(R1, T1);
    cout << "SE3初始化， 是旋转矩阵和平移向量：\n" << SE3_RT << endl;

    // 二、=============================>SO(3)与so(3)的相互转换，以及so3对应的hat和vee操作
    // 1、李代数so(3) 要用vector3d表示，使用对数映射将 李群转换为李代数  李群=====>李代数
    Eigen::Vector3d so3_V1 = SO3_V1.log();
    cout << "SO3 李群对应的李代数为: \n" << so3_V1 << endl;

    // 2、使用指数映射将，李代数转换为李群  李代数=====>李群
    Sophus::SO3 SO3_V2 = Sophus::SO3::exp(so3_V1);
    cout << "SO3  李代数转换为李群为: \n" << SO3_V2 << endl;

    // 3、hat为向量到其对应的反对称矩阵
    Eigen::Matrix3d M_so3_V1 = Sophus::SO3::hat(so3_V1);
    cout << "so3 向量对应的反对称矩阵 hat= \n" << M_so3_V1 << endl;

    // 4、vee为反对称矩阵对应的向量
    Eigen::Vector3d V_M = Sophus::SO3::vee(M_so3_V1);
    cout << "so3 反对称矩阵到向量 vee= \n" << V_M << endl;

    // 三、===============================>增量扰动模型
    Eigen::Vector3d update_so3(1e-4, 0, 0);   // 需要更新的量

    // 1、李群转换为旋转矩阵
    Eigen::Matrix3d update_matrix = Sophus::SO3::exp(update_so3).matrix();
    cout << "SO3 增益更新矩阵 Matrix= \n" << update_matrix << endl;

    // 2、旋转矩阵到李代数
    Sophus::SO3 SO3_updata = Sophus::exp(update_so3)*SO3_R;
    cout << "so3 旋转矩阵到李代数 SO3_updata= \n" << SO3_updata << endl;

    // 3、将李代数转换为旋转矩阵
    Eigen::Matrix3d SO3_updata_matrix = SO3_updata.matrix();
    cout << "SO3 SO3_updata_matrix= \n" << SO3_updata_matrix << endl;


    cout << "=================================================================================" << endl;
    cout << "================================SE(3)空间========================================" << endl;

    Eigen::AngleAxisd A2(M_PI/2, Eigen::Vector3d(0, 0, 1));
    Eigen::Matrix3d R2 = A2.matrix();
    Eigen::Quaterniond Q2(A2);
    Sophus::SO3 SO3_2(R2);
    Eigen::Vector3d t(1, 0, 0);

    // 一、=============================>初始化李群的方式
    // 1、SE3 使用旋转矩阵 和 平移向量初始化SE3
    //    SE3对应的是一4×4的矩阵，输出时，是一个六维向量，前三位是so3, 后三位是平移向量
    Sophus::SE3 SE_RT(R2, t);
    cout << "SE3 用u旋转矩阵和平移向量初始化李群: \n" << SE_RT << endl;

    // 2、SE3 使用四元数 和 平移向量出是化SE3
    Sophus::SE3 SE_Qt(Q2, t);
    cout << "SE3 使用四元数和平移向量初始化S李群: \n" << SE_Qt << endl;

    // 3、SE3 使用SO3 和 平移向量初始化SE3
    Sophus::SE3 SE_St(SO3_2, t);
    cout << "SE3 使用SO3和平移向量初始化SE3： \n" << SE_St << endl;

    // 二、SE(3) 和 SO(3) 相互转换,
    // 1、se(3)在Sophus中用Vector6d 表示，使用对数映射 获取 李群对应的李代数
    Sophus::Vector6d se3_RT = SE_RT.log();
    cout << "se3李代数 李群转换李代数为: \n" << se3_RT << endl;   // se3输出的是一个六维度向量,其中前3维是平移分量,后3维度是旋转分量

    // 2、SE3 李代数转换为李群
    Sophus::SE3 SE_Rt2 = Sophus::SE3::exp(se3_RT);
    cout << "SE3李群 李代数转换为李群: \n" << SE_Rt2 << endl;

    // 3、SE3 向量的反对称矩阵
    Sophus::Matrix4d M_se3_RT = Sophus::SE3::hat(se3_RT);
    cout << "SE3 hat= \n" << M_se3_RT << endl;

    // 4、SE3 反对称矩阵向量
    Sophus::Vector6d V_M_se3 = Sophus::SE3::vee(M_se3_RT);
    cout << "SE3 vee= \n" << V_M_se3 << endl;

    // 三、增量扰动模型
    Sophus::Vector6d update_se3 = Sophus::Vector6d::Zero();
    update_se3(0) = 1e-4;
    cout << "SE3 更新的增量为: \n" << update_se3.transpose() << endl;

    // 1、将李群转换为旋转矩阵
    Eigen::Matrix4d update_matrix2 = Sophus::SE3::exp(update_se3).matrix();
    cout << "SE3 李群转换为旋转矩阵 update_matrix2= \n" << update_matrix2 << endl;

    //
    Sophus::SE3 SE3_updata = Sophus::SE3::exp(update_se3)*SE_Rt2;
    cout << "SE3 更新量updata= \n" << SE3_updata << endl;

    // 3、将李群转换为旋转矩阵
    Eigen::Matrix4d SE3_updata_matrix = SE3_updata.matrix();
    cout << "SE3 updata matrix= \n" << SE3_updata_matrix << endl;


    cout << "Hello Sophus!" << endl;
    return 0;
}
