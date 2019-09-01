
#include "slamBase.hpp"


PointCloud::Ptr imageToPointcloud(Mat rgb, Mat depth, CAMERA_INTRINSIC_PARAMETERS camera)
{
    PointCloud::Ptr cloud(new PointCloud);

    for( int i = 0; i < depth.rows; i++)
    {
        for(int j = 0; j < depth.cols; j++)
        {
            // 获取深度图像（i， j）的值
            ushort d = depth.ptr<ushort>(i)[j];

            if(d == 0)
            {
                continue;
            }

            PointT p;

            // 计算点的空间坐标
            p.z = double(d) / camera.scale;
            p.x = (j - camera.cx) * p.z / camera.fx;
            p.y = (i - camera.cy) * p.z / camera.fy;


            // 从rgb图像中获取颜色
            p.b = rgb.ptr<uchar>(i)[j * 3];
            p.g = rgb.ptr<uchar>(i)[j * 3 + 1];
            p.r = rgb.ptr<uchar>(i)[j * 3 + 2];

            cloud->points.push_back(p);
        }
    }

    cloud->height = 1;
    cloud->width = cloud->points.size();
    cloud->is_dense = false;

    cloud->points.clear();
    cout << "PointCloud saved!" << endl;

    return cloud;
}

// 多帧点云图像融合
PointCloud::Ptr pointCloudFusion(PointCloud::Ptr& pointcloud, FRAME& newFrame, Eigen::Isometry3d T, CAMERA_INTRINSIC_PARAMETERS camera)
{
    // 1、点云叠加融合
    PointCloud::Ptr newPointCloud = imageToPointcloud(newFrame.rgb, newFrame.depth, camera);

    // 点云合并
    PointCloud::Ptr outPointCloud (new PointCloud());

    pcl::transformPointCloud(*pointcloud, *outPointCloud, T.matrix());

    *outPointCloud += *newPointCloud;

    // 2、下采样，保证点云形状特征
    PointCloud::Ptr downSampled_PointCloud (new PointCloud());
    pcl::VoxelGrid<PointT> downSampled;
    downSampled.setInputCloud(outPointCloud);
    downSampled.setLeafSize(0.01f, 0.01f, 0.01f);   // 设置体素体积为1cm 立方体
    downSampled.filter(*downSampled_PointCloud);
    pcl::io::savePCDFile("./downsamled.pcd", *downSampled_PointCloud);

    // 3、统计滤波
    PointCloud::Ptr filter_PointCloud (new PointCloud());
    pcl::StatisticalOutlierRemoval<PointT> statisOutlierRemoval;    // 滤波器对象
    statisOutlierRemoval.setInputCloud(downSampled_PointCloud);
    statisOutlierRemoval.setMeanK(50);                              // 查询点临近点数
    statisOutlierRemoval.setStddevMulThresh(3.0);                   // 离群点的阀值
    statisOutlierRemoval.filter(*filter_PointCloud);
    pcl::io::savePCDFile("./filtered.pcd", *filter_PointCloud);

    // 4、点云重采样，实现点云平滑
    PointCloud::Ptr smooth_PointCloud (new PointCloud());
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);    // 临近搜索法
    pcl::MovingLeastSquares<PointT, PointT> mls;                                // 最小二乘法对象
    mls.setComputeNormals(false);                                               // 最小二乘法中是否设置法线估计
    mls.setInputCloud(filter_PointCloud);
    mls.setPolynomialOrder(2);                                                  // 拟合2阶多项式拟合
    mls.setPolynomialFit(false);                                                // 加速 smooth
    mls.setSearchMethod(tree);
    mls.setSearchRadius(0.05);                                                  // 临近搜索半径
    mls.process(*smooth_PointCloud);
    pcl::io::savePCDFile("./smooth.pcd", *smooth_PointCloud);

    // 5、法线估计
    pcl::PointCloud<pcl::Normal>::Ptr normal_PointCloud (new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<PointT, pcl::Normal> normal;
    pcl::search::KdTree<PointT>::Ptr norml_tree (new pcl::search::KdTree<PointT>);

    normal.setInputCloud(smooth_PointCloud);
    normal.setSearchMethod(norml_tree);
    normal.setKSearch(10);                              // 使用当前周围最近的10个点
    normal.compute(*normal_PointCloud);                 // 计算法线
    pcl::io::savePCDFile("./normals.pcd", *normal_PointCloud);

    // 6、将点云位姿，颜色，法线连接到一起
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normal (new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*smooth_PointCloud, *normal_PointCloud, *cloud_with_normal);
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud(cloud_with_normal);

    // 7、贪心投影三角化
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    pcl::PolygonMesh triangels;                                 // 存储三角化网络模型

    gp3.setSearchRadius(0.1);   // 搜索临近半径
    gp3.setMu(2.5);             // 样本点搜索近临点的最远距离为2.5倍
    gp3.setMaximumNearestNeighbors(100);    // 样本点最多可搜索临域个数 一般在50-100
    gp3.setMinimumAngle(M_PI/18);           // 三角化后得到三角形内角的最小角度为10度
    gp3.setMaximumAngle(2 * M_PI/3);        // 三角化后得到三角形内角的最小角度为120度
    gp3.setMaximumSurfaceAngle(M_PI/4);     // 法线偏离样本点的法线最大角度45度，
    gp3.setNormalConsistency(false);        // 让法线朝向一致
    gp3.setInputCloud(cloud_with_normal);
    gp3.setSearchMethod(tree2);
    gp3.reconstruct(triangels);             // 重建提取三角化


    return smooth_PointCloud;
}


void readCamerTrajectory(string cameraTransFile, vector<Eigen::Isometry3d> &poses)
{
    // 绘制轨迹
    ifstream path(cameraTransFile);
    if(!path)
    {
        cerr << "轨迹信息为空..." << endl;
        return;
    }

    for(int i = 0; i < 5; i++)
    {
        double data[7] = {0};

        for(auto& d : data)
        {
            path >> d;
        }

        // 四元数
        Eigen::Quaterniond q(data[6], data[3], data[4], data[5]);

        // 四元数转换成 变换矩阵
        Eigen::Isometry3d T(q);

        poses.push_back(T);
    }


}