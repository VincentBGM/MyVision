

#include "myslam/map.h"

namespace myslam
{



    void Map::insertKeyFrame(const Frame::Ptr& frame)
    {
        cout << "pFrame id = " << frame->id_ << endl;
        cout << "关键帧 key Frame size " << key_frame_.size() << endl;

        if( key_frame_.find( frame->id_) ==  key_frame_.end() )
        {
            key_frame_.insert( make_pair(frame->id_, frame));
        }
        else
        {
            key_frame_[ frame->id_ ] = frame;
        }
    }


    void Map::insertMapPoint(MapPoint::Ptr map_points)
    {
        if( map_points_.find( map_points->id_ ) == map_points_.end() )
        {
            map_points_.insert( make_pair( map_points->id_, map_points));
        }
        else
        {
            map_points_[ map_points->id_ ] = map_points;
        }
    }
}

