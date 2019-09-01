
#ifndef POINTCLOUD_MAPPOINT_H
#define POINTCLOUD_MAPPOINT_H

#include "myslam/common_include.h"

namespace myslam
{
    class Frame;

    class MapPoint
    {

    public:
        typedef std::shared_ptr<MapPoint> Ptr;

        unsigned long           id_;
        static unsigned long    factory_id_;
        bool good_;                         // good points
        Eigen::Vector3d         pos_;       // 世界坐标系中的位置
        Eigen::Vector3d         norm_;      // 观测方向
        cv::Mat                 descriptor_;


        list<Frame* > observed_frames_;     //

        int                     observed_times_;      // 观测了次数
        int                     matched_times_;       // 匹配的次数
        int                     visible_times_;


        MapPoint();

        MapPoint(unsigned long id, const Eigen::Vector3d& position, const Eigen::Vector3d& norm, Frame* frame= nullptr, const cv::Mat& descriptor = cv::Mat() );


        inline cv::Point3f getPositionCV() const
        {
            return cv::Point3f( pos_(0, 0), pos_(1, 0), pos_(2, 0));
        }


        static MapPoint::Ptr createMapPoint();

        static MapPoint::Ptr createMapPoint( const Eigen::Vector3d& pos_world, const Eigen::Vector3d& norm, const cv::Mat& descriptor, Frame* frame );



    };

}


#endif //POINTCLOUD_MAPPOINT_H
