
#include "slamBase.hpp"

#include <iostream>


int main(int argc, char** argv)
{
    ParameterReader pd;

    int startIndex = atoi(pd.getData("start_index").c_str());
    int endIndex = atoi(pd.getData("end_index").c_str());

    cout << "Start Initializing ....." << endl;
    int currIndex = startIndex;
    FRAME lastFrame = readFrame(currIndex, pd);

    // 一直比较当前帧 和 上一帧
    CAMERA_INTRINSIC_PARAMETERS camera = getDefaultCamera();
    computeKeyPoints(lastFrame);
    PointCloud::Ptr cloud = image2PointCloud(lastFrame.rgb, lastFrame.depth, camera);
    pcl::visualization::CloudViewer viewer("Point Viewer");

    bool visualize = pd.getData("visualize_pointcloud") == string("yes");

    int min_inliers = atoi( pd.getData("min_inliers").c_str() );
    double max_norm = atof( pd.getData("max_norm").c_str() );

    for ( currIndex = startIndex + 1; currIndex < endIndex; currIndex++)
    {
        cout << "Reading file........." << endl;
        FRAME currFrame = readFrame(currIndex, pd);
        computeKeyPoints(currFrame);

        // 比较currFrame 和 lastFrame
        PNP_RESULT result = estimateMotion(lastFrame, currFrame, camera);
        if ( result.inliers < min_inliers )
        {
            continue;
        }

        // 计算运动范围是否太大
        double norm = normofTransform(result.rvec, result.tvec );
        cout << "norm = " << norm << endl;
        if ( norm > max_norm )
        {
            continue;
        }

        Eigen::Isometry3d T = cvMat2Eigen(result.rvec, result.tvec );
        cloud = joinPointCloud(cloud, currFrame, T, camera);

        if ( visualize == true )
        {
            viewer.showCloud(cloud);
        }

        lastFrame = currFrame;
    }

    pcl::io::savePCDFile( "../data/result.pcd", *cloud );

    return 0;
}