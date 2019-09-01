

#include "myslam/camera.h"

namespace myslam
{

    Camera::Camera()
    {

//        fx_ = Config::get<float>("camera.fx");
//        fy_ = Config::get<float>("camera.fy");
//        cx_ = Config::get<float>("camera.cx");
//        cy_ = Config::get<float>("camera.cy");
//        depth_scale_ = Config::get<float>("camera.depth_scale");

        fx_ = 517.3;
        fy_ = 516.5;
        cx_ = 325.1;
        cy_ = 249.7;
        depth_scale_ = 5000;
    }

    // 世界坐标系 ===> 相机坐标系
    Eigen::Vector3d Camera::world2Camera( const Eigen::Vector3d& P_w, const Sophus::SE3& T_c_w)
    {

//        cout << "world2Camera function used 111111111 " << endl;
        return T_c_w * P_w;
    }

    // 相机坐标系 ===> 世界坐标系
    Eigen::Vector3d Camera::camera2World( const Eigen::Vector3d& P_c, const Sophus::SE3& T_c_w)
    {

//        cout << "camera2World function used 222222222 " << endl;
        return T_c_w.inverse() * P_c;
    }

    // 相机坐标系 ===> 像素坐标
    Eigen::Vector2d Camera::camera2Pixel( const Eigen::Vector3d& P_c)
    {
        Eigen::Vector2d pixel;
        pixel  = Eigen::Vector2d( fx_ * P_c(0, 0) / P_c(2, 0) + cx_,
                                  fy_ * P_c(1, 0) / P_c(2, 0) + cy_
                );

//        cout << "camera2Pixel function used 33333333 " << endl;
        return pixel;
    }

    // 像素坐标 ====> 相机坐标系
    Eigen::Vector3d Camera::pixel2Camera( const Eigen::Vector2d& P_p, double depth)
    {
        Eigen::Vector3d camera_point;
        camera_point = Eigen::Vector3d( ( P_p(0, 0) - cx_ ) * depth / fx_,
                                        ( P_p(1, 0) - cy_ ) * depth / fy_,
                                        depth
                );

//        cout << "pixel2Camera function used 44444444 " << endl;
        return camera_point;
    }

    // 像素坐标 =====> 世界坐标系
    Eigen::Vector3d Camera::pixel2World( const Eigen::Vector2d& P_p, const Sophus::SE3& T_c_w, double depth)
    {
//        cout << "pixel2World function used 555555555 " << endl;
        return camera2World( pixel2Camera(P_p, depth), T_c_w);
    }

    // 世界坐标系 ===> 像素坐标
    Eigen::Vector2d Camera::world2Pixel( const Eigen::Vector3d& P_w, const Sophus::SE3& T_c_w)
    {
//        cout << "world2Pixel function used 6666666 " << endl;
        return camera2Pixel( world2Camera(P_w, T_c_w) );
    }


}