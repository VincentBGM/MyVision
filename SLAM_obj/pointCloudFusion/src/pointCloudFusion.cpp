
#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include "slamBase.hpp"


using namespace std;
using namespace cv;


int main(int argc, char** argv)
{

    // 相机内参
    CAMERA_INTRINSIC_PARAMETERS cameraParams{517.0, 516.0, 318.6, 255.3, 5000.0};

    int frameNum = 3;

    vector<Eigen::Isometry3d> poses;

    PointCloud::Ptr fusedCloud(new PointCloud());

    string path = "./data/";

    string camerPosePath = path + "cameraTrajectory.txt";
    cout << "轨迹路径为： " << camerPosePath << endl;


    readCamerTrajectory(camerPosePath, poses);

    for (int i = 0; i < frameNum; i++)
    {
        string rgbPath = path + "rgb/rgb" + to_string(i) + ".png";
        string depthPath = path + "depth/depth" + to_string(i) + ".png";

        FRAME frm;
        frm.rgb = cv::imread(rgbPath);
        if(frm.rgb.empty())
        {
            cerr << "RGB图像加载失败......" << endl;
            return -1;
        }

        frm.depth = cv::imread(depthPath, -1);
        if(frm.depth.empty())
        {
            cerr << "深度图像加载失败......." << endl;
            return -1;
        }

        if(i == 0)
        {
            fusedCloud = imageToPointcloud(frm.rgb, frm.depth, cameraParams);
        }
        else {
            fusedCloud = pointCloudFusion(fusedCloud, frm, poses[i], cameraParams);
        }
    }

    pcl::io::savePCDFile("./data/fusedCloud.pcd", *fusedCloud);

    cout << "hello SLAM!" << endl;
    return 0;
}