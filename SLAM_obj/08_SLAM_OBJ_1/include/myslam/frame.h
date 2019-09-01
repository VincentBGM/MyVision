

#ifndef POINTCLOUD_FRAME_H
#define POINTCLOUD_FRAME_H


#include "myslam/common_include.h"
#include "myslam/camera.h"

namespace myslam
{

    class Frame
    {
    public:
        typedef std::shared_ptr<Frame> Ptr;

        unsigned long         id_;
        double        time_stamp_;
        Sophus::SE3        T_w_c_;
        Camera::Ptr       camera_;
        cv::Mat    color_, depth_;

        bool        is_key_frame_;

    public:
        Frame();

        Frame(long id, double time_stamp = 0, const Sophus::SE3& T_w_c = Sophus::SE3(),
                Camera::Ptr camera = nullptr, cv::Mat color = cv::Mat(), cv::Mat depth = cv::Mat());

        ~Frame();

        static Frame::Ptr creatFrame();

        double findDepth(const cv::KeyPoint& depth );

        Eigen::Vector3d getCameraCenter() const;

        bool isInFrame(const Eigen::Vector3d& pt_world);
    };


}


#endif //POINTCLOUD_FRAME_H
