
#include <iostream>
#include <string>
#include <vector>

#include "slamBase.hpp"

using namespace std;


int main(int argc, char** argv)
{
    cv::Mat rgb;
    cv::Mat depth;

    ParameterReader pd;
    CAMERA_INTRINSIC_PARAMETERS camera;
    camera.fx = atof(pd.getData("camera.fx").c_str() );     // atof() 转换为双精度的double
    camera.fy = atof(pd.getData("camera.fy").c_str() );
    camera.cx = atof(pd.getData("camera.cx").c_str() );
    camera.cy = atof(pd.getData("camera.cy").c_str() );
    camera.scale = atof(pd.getData("camera.scale").c_str() );

    rgb = cv::imread("../data/rgb/rgb0.png");
    depth = cv::imread("../data/depth/depth0.png", -1);

    PointCloud::Ptr cloud (new PointCloud() );

    for (auto i = 0; i < depth.rows; i++)       // y行
    {
        for (auto j = 0; j < depth.cols; j++)   // x列
        {
            PointRGB point3D;
            // 获取图片深度值，将图像2维点 转成3维点
            ushort d = depth.ptr<ushort >(i)[j];
            if ( 0 == d )
            {
                continue;
            }

            point3D.z = double(d) / camera.scale;
            point3D.x = ( j - camera.cx ) * point3D.z / camera.fx;
            point3D.y = ( i - camera.cy ) * point3D.z / camera.fy;

            point3D.b = rgb.ptr<uchar >(i)[j * 3];
            point3D.g = rgb.ptr<uchar >(i)[j * 3 + 1];
            point3D.r = rgb.ptr<uchar >(i)[j * 3 + 2];

            cloud->points.push_back( point3D );
        }
    }

    cloud->height = 1;
    cloud->width = cloud->points.size();
    cout<<"point cloud size = " << cloud->points.size() << endl;
    cloud->is_dense = false;
    pcl::io::savePCDFile( "../data/pointcloud.pcd", *cloud );

    // 清除数据并退出
    cloud->points.clear();
    cout << "Point cloud saved." <<endl;

    return 0;
}