

#include "slamBase.hpp"


// 特征提取
void computeKeyPoints(FRAME& frame)
{
    cv::Ptr<cv::FeatureDetector> _detector;
    cv::Ptr<cv::DescriptorExtractor> _descriptor;

    _detector = cv::ORB::create();
    _descriptor = cv::ORB::create();


    if ( !_detector || !_descriptor )
    {
        cerr << "Unknow detector and descriptor type " << endl;
        return;
    }

    _detector->detect(frame.rgb, frame.keyPoints);
    _descriptor->compute(frame.rgb, frame.keyPoints, frame.descriptor);

    return;
}

// 计算2帧运动，求旋转矩阵 平移向量
PNP_RESULT estimateMotion(FRAME& frame1, FRAME& frame2, CAMERA_INTRINSIC_PARAMETERS& camera)
{
    static ParameterReader pd;
    vector<cv::DMatch> matchs;
    cv::BFMatcher mathcer;

    mathcer.match(frame1.descriptor, frame2.descriptor, matchs);

    PNP_RESULT result;

    vector<cv::DMatch> good_matchs;
    double minDis = 9999;
    double good_match_threshold = atof(pd.getData("good_match_threshold").c_str() );
    for ( auto i = 0; i < matchs.size(); i++)
    {
        if ( matchs[i].distance < minDis )
        {
            minDis = matchs[i].distance;
        }
    }

    if ( minDis < 10 )
    {
        minDis = 10;
    }

    for ( auto i = 0; i < matchs.size(); i++ )
    {
        if ( matchs[i].distance < good_match_threshold * minDis )
        {
            good_matchs.push_back(matchs[i]);
        }
    }

    if (good_matchs.size() <= 5)
    {
        result.inliers = -1;
        return result;
    }

    vector<cv::Point3f> points_3D;  // 第一帧三维点
    vector<cv::Point2f> points_2D;  // 第二帧归一化像素点

    for (auto i = 0; i < good_matchs.size(); i++)
    {
        // 将匹配对第一组2维点，转换为三维点
        cv::Point2f points2 = frame1.keyPoints[good_matchs[i].queryIdx].pt;

        ushort d = frame1.depth.ptr<ushort >(int(points2.y))[int(points2.x)];
        if ( 0 == d)
        {
            continue;
        }

        cv::Point3f cameraPoint3(points2.x, points2.y, d);
        cv::Point3f worldPoint3 = point2DTo3D(cameraPoint3, camera);
        points_3D.push_back(worldPoint3);


        // 获取匹配对的第二组2维点
        points_2D.push_back( cv::Point2f(frame2.keyPoints[good_matchs[i].trainIdx].pt ) );

    }

    if ( points_3D.size() == 0 || points_2D.size() == 0 )
    {
        result.inliers = -1;
        return result;
    }

//    cv::Mat K = ( cv::Mat_<double > (3, 3) << camera.fx, 0, camera.cx,
//                                              0, camera.fy, camera.cy,
//                                              0, 0, 1
//            );

    double camera_matrix_data[3][3] = {
            {camera.fx, 0, camera.cx},
            {0, camera.fy, camera.cy},
            {0, 0, 1}
    };
    cv::Mat K( 3, 3, CV_64F, camera_matrix_data );

    cv::Mat rvec;
    cv::Mat tvec;
    cv::Mat inliers;

    // pnp求救 R，t
    cv::solvePnPRansac(points_3D, points_2D, K, cv::Mat(), rvec, tvec, false, 100, 8.0, 0.99, inliers);

    result.rvec = rvec;
    result.tvec = tvec;
    result.inliers = inliers.rows;

    return result;
}

// 2D 转3D 点
cv::Point3f point2DTo3D(cv::Point3f& point, CAMERA_INTRINSIC_PARAMETERS& camera)
{
    cv::Point3f point3;

    point3.z = double( point.z) / camera.scale;
    point3.x = ( point.x - camera.cx ) * point3.z / camera.fx;
    point3.y = ( point.y - camera.cy ) * point3.z / camera.fy;

    return point3;
}

// 旋转 平移转换为 变换矩阵
Eigen::Isometry3d cvMat2Eigen(cv::Mat& rvec, cv::Mat& tvec)
{
    cv::Mat R;
    cv::Rodrigues(rvec, R);

    Eigen::Matrix3d r;
    for (auto i = 0; i < 3; i++)
    {
        for (auto j = 0; j < 3; j++)
        {
            r(i, j) = R.at<double>(i, j);
        }
    }

    // 平移向量 旋转矩阵 转换为 变换矩阵
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

    Eigen::AngleAxisd angle( r );
//    Eigen::Translation<double,3> trans(tvec.at<double>(0,0), tvec.at<double>(0,1), tvec.at<double>(0,2));
    T = angle;
    T(0, 3) = tvec.at<double >(0, 0);
    T(1, 3) = tvec.at<double >(1, 0);
    T(2, 3) = tvec.at<double >(2, 0);

    return T;
}

// 图像转换为点云
PointCloud::Ptr image2PointCloud(cv::Mat& rgb, cv::Mat& depth, CAMERA_INTRINSIC_PARAMETERS& camera)
{
    PointCloud::Ptr cloud ( new PointCloud );

    for (auto  i = 0; i < depth.rows; i += 2)
    {
        for (auto j = 0; j < depth.cols; j += 2)
        {
            // 获取深度图像的 深度
            ushort d = depth.ptr<double >(i)[j];
            if ( 0 == d )
            {
              continue;
            }
              // 将二维点转换成3维点
            PointRGB p;
            p.z = double(d) / camera.scale;
            p.x = ( j - camera.cx ) * p.z / camera.fx;
            p.y = ( i - camera.cy ) * p.z / camera.fy;

            // 获取点的颜色
            p.b = rgb.ptr<uchar >(i)[j * 3];
            p.g = rgb.ptr<uchar >(i)[j * 3 + 1];
            p.r = rgb.ptr<uchar >(i)[j * 3 + 2];

            // 加入到点云中
            cloud->points.push_back(p);
        }
    }

    cloud->height = 1;
    cloud->width = cloud->points.size();
    cloud->is_dense = false;

    return cloud;
}

// 点云融合
PointCloud::Ptr joinPointCloud(PointCloud::Ptr point, FRAME& new_frame, Eigen::Isometry3d T, CAMERA_INTRINSIC_PARAMETERS& camera)
{
    // 将新来的帧 转换为点云
    PointCloud::Ptr new_pointcloud = image2PointCloud(new_frame.rgb, new_frame.depth, camera);

    // 点云合并
    PointCloud::Ptr output ( new PointCloud());

    pcl::transformPointCloud(*point, *output, T.matrix() );
    *new_pointcloud += *output;

    // Voxel grid 滤波降采样
    static pcl::VoxelGrid<PointRGB> voxel;
    static ParameterReader pd;
    double gridsize = atof( pd.getData("voxel_grid").c_str() );
    voxel.setLeafSize( gridsize, gridsize, gridsize );
    voxel.setInputCloud( new_pointcloud );
    PointCloud::Ptr tmp( new PointCloud() );
    voxel.filter( *tmp );

    return tmp;
}

FRAME readFrame(int index, ParameterReader& pd)
{
    FRAME frame;

    string rgbDir = pd.getData("rgb_dir");
    string depthDir = pd.getData("depth_dir");

    string rgbExt = pd.getData("rgb_extension");
    string depthExt = pd.getData("depth_extension");

    stringstream ss;
    ss << rgbDir << index << rgbExt;

    string fileName;
    ss >> fileName;
    frame.rgb = imread(fileName);

    ss.clear();
    fileName.clear();

    ss << depthDir << index << depthExt;
    ss >> fileName;
    frame.depth = imread(fileName, -1);

    ss.clear();
    fileName.clear();

    return frame;
}

// 计算运动范围是否太大
double normofTransform( cv::Mat rvec, cv::Mat tvec )
{
    return fabs(min(cv::norm(rvec), 2*M_PI-cv::norm(rvec)))+ fabs(cv::norm(tvec));
}

// 匹配当前帧与keyframes里最后一帧
CHECK_RESULT checkKeyframes( FRAME& frame1, FRAME& frame2, g2o::SparseOptimizer& opti, bool is_loops )
{
    static ParameterReader pd;
    static int min_inliers = atoi(pd.getData("min_inliers").c_str() );
    static double max_norm = atof( pd.getData("max_norm").c_str() );
    static double keyframe_threshold = atof( pd.getData("keyframe_threshold").c_str() );
    static double max_norm_lp = atof( pd.getData("max_norm_lp").c_str() );
    static CAMERA_INTRINSIC_PARAMETERS camera = getDefaultCamera();

    // 比较frame1  和 frame2
    PNP_RESULT pnp_result = estimateMotion(frame1, frame2, camera);
    if (  pnp_result.inliers < min_inliers )
    {
        return NOT_MATCHED;
    }

    // 计算运动范围是否太大
    double norm = normofTransform(pnp_result.rvec, pnp_result.tvec);
    if ( is_loops == false )
    {
        if ( norm >= max_norm )
        {
            return TOO_FAR_AWAY;
        }
    }
    else
    {
        if ( norm >= max_norm )
        {
            return TOO_FAR_AWAY;
        }
    }

    if ( norm <= keyframe_threshold )
    {
        return TOO_CLOSE;
    }

    // 向g2o中增加这个顶点与上一帧联系的边
    // 顶点部分
    // 顶点只需设定id即可
    if ( is_loops == false )
    {
        g2o::VertexSE3* v = new g2o::VertexSE3();
        v->setId(frame2.frameID);
        v->setEstimate( Eigen::Isometry3d::Identity() );
        opti.addVertex( v );
    }

    // 增加边
    g2o::EdgeSE3* e = new g2o::EdgeSE3();
    e->setVertex(0, opti.vertex(frame1.frameID));
    e->setVertex(1, opti.vertex(frame2.frameID));
    e->setRobustKernel( new g2o::RobustKernelHuber() );

    // 信息矩阵
    Eigen::Matrix<double, 6, 6>information = Eigen::Matrix<double, 6, 6 >::Identity();
    // 信息矩阵是协方差矩阵的逆，表示我们对边的精度的预先估计
    // 因为pose为6D的，信息矩阵是6*6的阵，假设位置和角度的估计精度均为0.1且互相独立
    // 那么协方差则为对角为0.01的矩阵，信息阵则为100的矩阵
    information(0,0) = information(1,1) = information(2,2) = 100;
    information(3,3) = information(4,4) = information(5,5) = 100;

    // 也可以将角度设大一些，表示对角度的估计更加准确
    e->setInformation( information );
    // 边的估计即是pnp求解之结果
    Eigen::Isometry3d T = cvMat2Eigen( pnp_result.rvec, pnp_result.tvec );
    // edge->setMeasurement( T );
    e->setMeasurement( T.inverse() );
    // 将此边加入图中
    opti.addEdge(e);
    return KEYFRAME;


}

// 回环检测
// 回环检测==============================非常重要
// 当前帧与 关键帧里面 最后几个帧检测
void checkNearbyLoops(vector<FRAME>& keyFrams, FRAME& currFrame, g2o::SparseOptimizer& optimizer)
{
    static ParameterReader pd;
    static int nearby_loops = atoi(pd.getData("nearby_loops").c_str());

    // 当前帧与 关键帧里面 最后几个帧检测
    if ( keyFrams.size() <= nearby_loops )
    {
        // 每个帧都检测
        for (auto i = 0; i < keyFrams.size(); i++)
        {
            checkKeyframes(keyFrams[i], currFrame, optimizer);

        }
    }
    else
    {
        // 至少要检测 后5个
        for (auto i = keyFrams.size() - nearby_loops; i < keyFrams.size(); i++)
        {
            checkKeyframes(keyFrams[i], currFrame, optimizer);
        }
    }


}

// 回环检测，随机选择帧
void checkRandomLoops(vector<FRAME>& keyFrams, FRAME& currFrame, g2o::SparseOptimizer& optimizer)
{
    static ParameterReader pd;
    static int random_loops = atoi(pd.getData("random_loops").c_str());

    srand( (unsigned int) time(NULL) );

    if ( keyFrams.size() < random_loops )
    {
        for ( auto i = 0; i < keyFrams.size(); i++ )
        {
            checkKeyframes(keyFrams[i], currFrame, optimizer);
        }
    }
    else
    {
        for (auto i = 0; i < random_loops; i++ )
        {
            int index = rand()%keyFrams.size();
            checkKeyframes(keyFrams[index],currFrame, optimizer);
        }
    }
}