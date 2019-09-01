#include <iostream>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <ctime>


using namespace std;


int main(int argc, char** argv)
{

    // 1、eigen vector usage  向量用法
    Eigen::Vector3d pos;
    pos << 0.0, -2.0, 1.0;
    cout << "===========>pos: \n" << pos << endl;

    Eigen::Vector3d pos2 = Eigen::Vector3d(1, -3, 0.5);
    cout << "===========>pos2: \n" << pos2 << endl;

    pos = pos + pos2;
    cout << "===========>pos: \n" << pos << endl;

    Eigen::Vector3d pos3 = Eigen::Vector3d::Zero();
    cout << "===========>pos3: \n" << pos3 << endl;

    // 归一化
    pos.normalize();
//    cout << "===========pos.normalize: \n" << pos << endl;

    // 逆矩阵
    //pos.inverse();
    // cout << "===========pos.inverse: \n" << pos << endl;


    Eigen::Vector3d v(1, 2, 3);
    Eigen::Vector3d w(0, 1, 2);

    // 点乘
    double v_Dot_w = v.dot(w);
    cout << "===========v_Dot_w: \n" << v_Dot_w << endl;

    // 叉乘
    Eigen::Vector3d v_Cross_w = v.cross(w);
    cout << "===========v_Cross_w: \n" << v_Cross_w << endl;

    // 2、位置信息，包括四元数
    Eigen::Quaterniond rot;
    rot.setFromTwoVectors(Eigen::Vector3d(0, 1, 0), pos);
//    cout << "===========rot.setFromTwoVector: \n" << rot << endl;

    Eigen::Matrix<double, 3, 3> rotationMatrix;
    cout << "==========>rotationMatrix: \n" << rotationMatrix << endl;

    rotationMatrix = rot.toRotationMatrix();
    cout << "==========>rot.rotationMatrix: \n" << rotationMatrix << endl;

    Eigen::Quaterniond q(2, 0, 1, -3);
    cout << "实部： " << q.w() << "向量（虚部）： " << endl << q.vec() << endl;

    q.normalize();

    cout << "归一化后，四元数的长度为: " << q.norm() << endl;

    Eigen::Vector3d vec(1, 2, -1);
    Eigen::Quaterniond p;

    p.w() = 0;
    p.vec() = vec;

    Eigen::Quaterniond rotation_p = q * p * q.inverse();
    Eigen::Vector3d rotation_v = rotation_p.vec();

    cout << "旋转一个向量" << endl << vec << "旋转到==========>" << endl << rotation_v << endl;

    // 将四元数转换成旋转矩阵
    Eigen::Matrix3d R = q.toRotationMatrix();

    std::cout << "rotation matrix " << std::endl << R * vec << std::endl;

    Eigen::Quaterniond a = Eigen::Quaterniond::Identity();
    Eigen::Quaterniond b = Eigen::Quaterniond::Identity();
    Eigen::Quaterniond c;


    c.w() = a.w() + b.w();
    c.x() = a.x() + b.x();
    c.y() = a.y() + b.y();
    c.z() = a.z() + b.z();

    Eigen::MatrixXd A(3, 2);
    A << 1, 2,
	 2, 3,
	 3, 4;

    Eigen::MatrixXd B = A.transpose();
	
    Eigen::MatrixXd C = (B * A).inverse();
    C.determinant(); 
    Eigen::Matrix3d D = Eigen::Matrix3d::Identity();

    Eigen::Matrix3d m = Eigen::Matrix3d::Random();
    m = (m + Eigen::Matrix3d::Constant(1.2)) * 50;
    Eigen::Vector3d v2(1,2,3);

    std::cout << "m =" << std::endl << m << std::endl;
    std::cout << "m * v2 =" << std::endl << m * v2 << std::endl;

    Eigen::MatrixXd A2 = Eigen::MatrixXd::Random(7, 9);
    std::cout << "A2 " << A2(3, 6) << std::endl;

    Eigen::MatrixXd B2 = A2.block(1, 2, 3, 3);
    std::cout << "B2" << std::endl << B2 << std::endl;

    Eigen::VectorXd a2 = A2.col(1); 
    Eigen::VectorXd b2 = B2.row(0); 

    Eigen::VectorXd c2 = a2.head(3);
    Eigen::VectorXd d2 = b2.tail(2);


    cout << "Hello Eigen!" << endl;
    return 0;
}
