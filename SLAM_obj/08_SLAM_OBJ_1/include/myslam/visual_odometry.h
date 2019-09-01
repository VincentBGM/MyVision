

#ifndef POINTCLOUD_VISUAL_ODOMETRY_H
#define POINTCLOUD_VISUAL_ODOMETRY_H


#include "myslam/common_include.h"
#include "myslam/map.h"
#include "myslam/frame.h"
#include "myslam/config.h"
#include "myslam/g2o_types.h"


namespace myslam
{

    class VisualOdometry
    {

    public:
        typedef std::shared_ptr<VisualOdometry> Ptr;

        enum VOState
        {
            INITIALIZING = -1,
            OK = 0,
            LOST
        };

        VOState state_;         // 当前VO的状态
        Map::Ptr map_;           // 存储个个关键帧 和 路标点 landmark ,可以随时访问，也可以随时插入 和 删除
        Frame::Ptr ref_;           // 参考帧  第一帧 只存储基本的数据类型：序号 时间戳 深度
        Frame::Ptr cur_;           // 当前帧  第二帧

        cv::Ptr<cv::ORB> orb_;      // 特征匹配 和 描述子方法
        vector<cv::Point3f> points_3d_ref_;     // 存放参考帧的 3D点
        vector<cv::KeyPoint> keypoints_curr_;     //
        cv::Mat descriptor_ref_;
        cv::Mat descriptor_cur_;

        cv::FlannBasedMatcher matcher_flann_;
        vector<MapPoint::Ptr> match_3dpts_;
        vector<int > match_2dkp_index_;


        vector<cv::DMatch> feature_matchers_;   // goodmatchers

        Sophus::SE3 T_c_r_estimated_;
        int num_lost_;
        int num_inliers_;

        int num_of_features_;
        double scale_factor_;
        int level_pyramid_;
        float match_ration_;
        int max_number_lost;
        int min_inliers_;


        double map_point_erase_ratio_;


        double key_frame_min_rot;
        double key_frame_min_trans;

    public:
        VisualOdometry();

        ~VisualOdometry();

        bool addFrame(const Frame::Ptr frame);

    protected:

        void extractKeyPoints();

        void computeDescriptors();

        void featureMatching();

        void poseEstimationPnP();

        void setRef3DPoints();

        void addKeyFrame();
        void optimizeMap();
        void addMapPoints();

        double getViewAngle(Frame::Ptr frame, MapPoint::Ptr point);

        void poseEstimationPnP_BA();   // g2o 非线性优化




        bool checkEstimatedPose();      // 位姿检测

        bool checkKeyFrame();


    };

}


#endif //POINTCLOUD_VISUAL_ODOMETRY_H
