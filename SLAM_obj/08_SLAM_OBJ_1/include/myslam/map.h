
#ifndef POINTCLOUD_MAP_H
#define POINTCLOUD_MAP_H


#include "myslam/common_include.h"
#include "myslam/frame.h"
#include "myslam/mappoint.h"

namespace myslam
{

    class Map
    {
    public:

        typedef std::shared_ptr<Map> Ptr;

        unordered_map<unsigned long, MapPoint::Ptr> map_points_;    // 所有的路标 landmark
        unordered_map<unsigned long, Frame::Ptr> key_frame_;        // 关键枕

        Map() {}

        void insertKeyFrame(const Frame::Ptr& frame);
        void insertMapPoint(MapPoint::Ptr map_points);
    };
}

#endif //POINTCLOUD_MAP_H
