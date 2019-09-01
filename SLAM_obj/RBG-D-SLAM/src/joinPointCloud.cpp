
#include <iostream>
#include <string>
#include <vector>

#include "slamBase.hpp"

using namespace std;

int main(int argc, char** argv)
{
    FRAME frame1, frame2;
    ParameterReader pd;
    CAMERA_INTRINSIC_PARAMETERS camera;
    camera.fx = atof(pd.getData("camera.fx").c_str() );
    camera.fy = atof(pd.getData("camera.fy").c_str() );
    camera.cx = atof(pd.getData("camera.cx").c_str() );
    camera.cy = atof(pd.getData("camera.cy").c_str() );
    camera.scale = atof(pd.getData("camera.scale").c_str() );

    frame1.rgb = cv::imread("../data/rgb/rgb1.png");
    frame1.depth = cv::imread("../data/depth/depth1.png", -1);

    frame2.rgb = cv::imread("../data/rgb/rgb2.png");
    frame2.depth = cv::imread("../data/depth/depth2.png", -1);

    computeKeyPoints(frame1);
    computeKeyPoints(frame2);

    PNP_RESULT result = estimateMotion(frame1, frame2, camera);

    cv::Mat R;
    cv::Rodrigues(result.rvec, R);



    Eigen::Matrix3d r;
    for (auto i = 0; i < 3; i++)
    {
        for (auto j = 0; j < 3; j++)
        {
            r(i, j) = R.at<double>(i,j);
        }
    }



    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    Eigen::AngleAxisd angle( r );
    T = r;
    T(0, 3) = result.tvec.at<double >(0, 0);
    T(1, 3) = result.tvec.at<double >(1, 0);
    T(2, 3) = result.tvec.at<double >(2, 0);

    PointCloud::Ptr cloud1 = image2PointCloud(frame1.rgb, frame1.depth, camera);
    PointCloud::Ptr cloud2 = image2PointCloud(frame2.rgb, frame2.depth, camera);

    PointCloud::Ptr outcloud (new PointCloud() );
    pcl::transformPointCloud(*cloud1, *outcloud, T.matrix() );
    *outcloud += *cloud2;

//    pcl::io::savePCDFile("../data/point_join.pcd", *outcloud);

    pcl::visualization::CloudViewer viewer( "viewer" );
    viewer.showCloud( outcloud );
    while( !viewer.wasStopped() )
    {

    }

    return 0;
}
