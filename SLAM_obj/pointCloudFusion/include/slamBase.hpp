
# pragma once

#include <iostream>
#include <fstream>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/mls.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/poisson.h>

using namespace std;
using namespace cv;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;

// 相机内参
struct  CAMERA_INTRINSIC_PARAMETERS
{
    double fx, fy, cx, cy, scale;
};

//
struct FRAME
{
    Mat rgb, depth;
};

// 图片转点云
PointCloud::Ptr imageToPointcloud(Mat rgb, Mat depth, CAMERA_INTRINSIC_PARAMETERS camera);

PointCloud::Ptr pointCloudFusion(PointCloud::Ptr& pointcloud, FRAME& newFrame, Eigen::Isometry3d T, CAMERA_INTRINSIC_PARAMETERS camera);

void readCamerTrajectory(string cameraTransFile, vector<Eigen::Isometry3d> &poses);