
#ifndef POINTCLOUD_CAMERA_H
#define POINTCLOUD_CAMERA_H

#include "myslam/common_include.h"

namespace myslam
{

    class Camera
    {
    public:
        typedef std::shared_ptr<Camera> Ptr;

        float fx_, fy_, cx_, cy_, depth_scale_;


        Camera();

//        Camera( float fx, float fy, float cx, float cy, float depth_scale = 0) : fx_(fx), fy_(fy), cx_(cx), cy_(cy), depth_scale_(depth_scale) {}


        Camera ( float fx, float fy, float cx, float cy, float depth_scale=0 ) :
                fx_ ( fx ), fy_ ( fy ), cx_ ( cx ), cy_ ( cy ), depth_scale_ ( depth_scale )
        {}


        // 世界坐标系 ===> 相机坐标系
        Eigen::Vector3d world2Camera( const Eigen::Vector3d& P_w, const Sophus::SE3& T_c_w);

        // 相机坐标系 ===> 世界坐标系
        Eigen::Vector3d camera2World( const Eigen::Vector3d& P_c, const Sophus::SE3& T_c_w);

        // 相机坐标系 ===> 像素坐标
        Eigen::Vector2d camera2Pixel( const Eigen::Vector3d& P_c);

        // 像素坐标 ====> 相机坐标系
        Eigen::Vector3d pixel2Camera( const Eigen::Vector2d& P_p, double depth = 1);

        // 像素坐标 =====> 世界坐标系
        Eigen::Vector3d pixel2World( const Eigen::Vector2d& P_p, const Sophus::SE3& T_c_w, double depth = 1);

        // 世界坐标系 ===> 像素坐标
        Eigen::Vector2d world2Pixel( const Eigen::Vector3d& P_w, const Sophus::SE3& T_c_w);


    };

}

#endif //POINTCLOUD_CAMERA_H
