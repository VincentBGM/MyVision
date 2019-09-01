#include <memory>

#include <utility>

#include <utility>

#include <utility>


#include "myslam/frame.h"


namespace myslam {

    Frame::Frame() : id_(-1), time_stamp_(-1), camera_(nullptr), is_key_frame_(false)
    {

    }

    Frame::Frame ( long id, double time_stamp, const Sophus::SE3& T_w_c, Camera::Ptr camera, cv::Mat color, cv::Mat depth )
            : id_(id), time_stamp_(time_stamp), T_w_c_(T_w_c), camera_(camera), color_(color), depth_(depth), is_key_frame_(false)
    {


    }

    Frame::~Frame() = default;


    Frame::Ptr Frame::creatFrame()
    {
        static long factory_id = 0;
        return Frame::Ptr (new Frame( factory_id++ ) );
    }

    // 获取图像的 深度===========================================================？？？？？？？？？？？？？？？？？？？
    double Frame::findDepth(const cv::KeyPoint& KeyPoint)
    {

//        cvRound()：返回跟参数最接近的整数值，即四舍五入；
//        cvFloor()  ：返回不大于参数的最大整数值，即向下取整；
//        cvCeil()：返回不小于参数的最小整数值，即向上取整；

        int x = cvRound(KeyPoint.pt.x);
        int y = cvRound(KeyPoint.pt.y);

        ushort d = depth_.ptr<ushort >(y)[x];
        if( d != 0 )
        {
            return double( d ) / camera_->depth_scale_;
        }
        else
        {
            int dx[4] = {-1, 0, 1, 0};
            int dy[4] = {0, -1, 0, 1};
            for(auto i = 0; i < 4; i++)
            {
                d = depth_.ptr<ushort >( y + dy[i] )[ x + dx[i] ];
                if ( d != 0 )
                {
                    return double( d ) / camera_->depth_scale_;
                }
            }
        }

        return -1.0;
    }

    // 获取相机的光心 ==============================================================？？？？？？？？？？
    Eigen::Vector3d Frame::getCameraCenter() const
    {
        return T_w_c_.inverse().translation();
    }

    // 判断某个点是不是 在视野内
    bool Frame::isInFrame(const Eigen::Vector3d &pt_world)
    {
        Eigen::Vector3d p_cam = camera_->world2Camera(pt_world, T_w_c_);

        if( p_cam(2, 0) < 0)
        {
            return false;
        }

        Eigen::Vector2d pixel = camera_->world2Pixel(pt_world, T_w_c_ );

        return pixel(0,0)>0 && pixel(1,0)>0 && pixel(0,0)<color_.cols && pixel(1,0)<color_.rows;
    }

}