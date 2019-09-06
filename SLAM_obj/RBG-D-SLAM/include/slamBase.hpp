
#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <list>
#include <memory>
#include <set>
#include <unordered_map>
#include <map>
#include <algorithm>
#include <boost/timer.hpp>
#include <fstream>
#include <sstream>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/viz.hpp>

#include <sophus/se3.h>

#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_levenberg.h>


#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

//#define cout cout << "[" << __FILE__ << ":" << __LINE__ << "]"

using namespace std;
using namespace cv;
// using namespace pcl;

typedef pcl::PointXYZRGBA PointRGB;
typedef pcl::PointCloud<PointRGB> PointCloud;

struct FRAME
{
    int frameID;
    cv::Mat rgb;
    cv::Mat depth;
    cv::Mat descriptor;
    vector<cv::KeyPoint> keyPoints;
};

struct PNP_RESULT
{
    cv::Mat rvec;
    cv::Mat tvec;
    int inliers;
};

struct CAMERA_INTRINSIC_PARAMETERS
{
    double fx;
    double fy;
    double cx;
    double cy;
    double scale;
};

enum CHECK_RESULT
{
    NOT_MATCHED = 0,
    TOO_FAR_AWAY,
    TOO_CLOSE,
    KEYFRAME
};

class ParameterReader
{
public:
    map<string, string> data;

    ParameterReader(string fileName = "../parameters.txt" )
    {
        ifstream filePath( fileName.c_str() );
        if ( !filePath )
        {
            cerr << "Parameters file is not exist......." << endl;
            return ;
        }

        while ( !filePath.eof() )
        {
            string str;
            getline(filePath, str);

            if ( "#" == str.substr(0) )
            {
                continue;
            }

            int pose = str.find("=");
            if ( -1 == pose )
            {
                continue;
            }

            string key = str.substr(0, pose);
            string value = str.substr(pose + 1, str.length() );

            data[key] = value;

            if ( !filePath.good() )
            {
                break;
            }
        }
    }

    string getData(string key)
    {
        map<string, string>::iterator it = data.find(key);
        if ( it == data.end() )
        {
            cerr << "Parameter not find " << key << " data ......." << endl;
            return string("NO_FIND");
        }

        return it->second;
    }
};

inline static CAMERA_INTRINSIC_PARAMETERS getDefaultCamera()
{
    ParameterReader pd;
    CAMERA_INTRINSIC_PARAMETERS camera;

   
    camera.fx = atof( pd.getData("camera.fx").c_str());
    camera.fy = atof( pd.getData("camera.fy").c_str());
    camera.cx = atof( pd.getData("camera.cx").c_str());
    camera.cy = atof( pd.getData("camera.cy").c_str());
    camera.scale = atof( pd.getData("camera.scale").c_str());

    return camera;
}

void computeKeyPoints(FRAME& frame);            // 特征提取

PNP_RESULT estimateMotion(FRAME& frame1, FRAME& frame2, CAMERA_INTRINSIC_PARAMETERS& camera);        // 计算2帧运动，求旋转矩阵 平移向量

Eigen::Isometry3d cvMat2Eigen(cv::Mat& rvec, cv::Mat& tvec);    // 旋转 平移转换为 变换矩阵

cv::Point3f point2DTo3D(cv::Point3f& point, CAMERA_INTRINSIC_PARAMETERS& camera);          // 2D 转3D 点

PointCloud::Ptr image2PointCloud(cv::Mat& rgb, cv::Mat& depth, CAMERA_INTRINSIC_PARAMETERS& camera); // 图像转换为点云

PointCloud::Ptr joinPointCloud(PointCloud::Ptr point, FRAME& new_frame, Eigen::Isometry3d T, CAMERA_INTRINSIC_PARAMETERS& camera);   // 点云融合

FRAME readFrame(int index, ParameterReader& pd);

double normofTransform( cv::Mat rvec, cv::Mat tvec );

CHECK_RESULT checkKeyframes( FRAME& frame1, FRAME& frame2, g2o::SparseOptimizer& opti, bool is_loops=false ); // 匹配当前帧与keyframes里最后一帧

// 回环检测
void checkNearbyLoops(vector<FRAME>& keyFrams, FRAME& currFrame, g2o::SparseOptimizer& optimizer);

void checkRandomLoops(vector<FRAME>& keyFrams, FRAME& currFrame, g2o::SparseOptimizer& optimizer);


//the following are UBUNTU/LINUX ONLY terminal color
#define RESET "\033[0m"
#define BLACK "\033[30m" /* Black */
#define RED "\033[31m" /* Red */
#define GREEN "\033[32m" /* Green */
#define YELLOW "\033[33m" /* Yellow */
#define BLUE "\033[34m" /* Blue */
#define MAGENTA "\033[35m" /* Magenta */
#define CYAN "\033[36m" /* Cyan */
#define WHITE "\033[37m" /* White */
#define BOLDBLACK "\033[1m\033[30m" /* Bold Black */
#define BOLDRED "\033[1m\033[31m" /* Bold Red */
#define BOLDGREEN "\033[1m\033[32m" /* Bold Green */
#define BOLDYELLOW "\033[1m\033[33m" /* Bold Yellow */
#define BOLDBLUE "\033[1m\033[34m" /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m" /* Bold Magenta */
#define BOLDCYAN "\033[1m\033[36m" /* Bold Cyan */
#define BOLDWHITE "\033[1m\033[37m" /* Bold White */





















