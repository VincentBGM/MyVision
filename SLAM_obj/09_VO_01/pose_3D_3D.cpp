
#include <iostream>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

#include <chrono>

using namespace std;


string rgb1_path = "./rgb1.png";
string rgb2_path = "./rgb2.png";

string depth1_path = "./depth1.png";
string depth2_path = "./depth2.png";


void find_feature_matches(  cv::Mat& img1, cv::Mat& img2,
                            vector<cv::KeyPoint>& KeyPoints1, vector<cv::KeyPoint>& KeyPoints2,
                            vector<cv::DMatch>& goodMatchers)
{
    cv::Ptr<cv::FeatureDetector> detector;
    cv::Ptr<cv::DescriptorExtractor> descriptor;

    detector = cv::ORB::create();
    descriptor = cv::ORB::create();

    detector->detect(img1, KeyPoints1);
    detector->detect(img2, KeyPoints2);

    cv::Mat descriptor_1, descriptor_2;
    descriptor->compute(img1, KeyPoints1, descriptor_1);
    descriptor->compute(img2, KeyPoints2, descriptor_2);

    vector<cv::DMatch> Matchers;
    cv::BFMatcher Matcher;
    Matcher.match(descriptor_1, descriptor_2, Matchers);

    double minDistance = 10000, maxDistance = 0;
    for(auto i = 0; i < Matchers.size(); i++)
    {
        if( Matchers[i].distance < minDistance )
        {
            minDistance = Matchers[i].distance;
        }
        if( Matchers[i].distance > maxDistance )
        {
            maxDistance = Matchers[i].distance;
        }
    }

    for(auto i = 0; i < Matchers.size(); i++ )
    {
        if ( Matchers[i].distance <= max (2 * minDistance, 30.0) )
        {
            goodMatchers.emplace_back(Matchers[i]);
        }
    }
}


cv::Point2d pixel2cam ( const cv::Point2d& p, const cv::Mat& K )
{
    cv::Point2d newPoints;
    newPoints = cv::Point2d ( (p.x - K.at<double>(0, 2) / K.at<double>(0, 0)),       // (p.x - Cx ) / Fx
                              (p.y - K.at<double>(1, 2) / K.at<double>(1, 1)));      // (p.y - Cy ) / Fy

    return newPoints;
}


void bundleAdjustment( vector<cv::Point3f>& point_3D, vector<cv::Point2f>& point_2D,
                       cv::Mat& R, cv::Mat& t, cv::Mat& K)
{
    // 初始化 g2o
    typedef g2o::BlockSolver< g2o::BlockSolverTraits<6, 3>> Block;
    // 创建 线性 求解器
    Block::LinearSolverType* linearSolver = new g2o::LinearSolverCSparse<Block::PoseMatrixType>();
    // 创建 矩阵块 求解器
    Block* solver_prt = new Block( unique_ptr<Block::LinearSolverType>( linearSolver) );
    // 创建总的求解器
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( unique_ptr<Block>( solver_prt ));
    // 创建稀疏优化器
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm( solver );

    // 创建顶点 相机位姿对象
    g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap();
    // 旋转矩阵的
    Eigen::Matrix3d R1;
    R1 << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
            R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
            R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2);

    cout << "R  = \n" << R  << endl;
    cout << "R1 = \n" << R1 << endl;

    pose->setId( 0 );
    pose->setEstimate( g2o::SE3Quat(R1,
                                    Eigen::Vector3d ( t.at<double>(0, 0), t.at<double>(1, 0), t.at<double>(2, 0))
    ) );

    optimizer.addVertex( pose );

    // 添加顶点
    int index = 1;
    for( auto i = 0; i < point_3D.size(); i++)
    {
        g2o::VertexSBAPointXYZ* point = new g2o::VertexSBAPointXYZ();
        point->setId( index++ );
        point->setEstimate( Eigen::Vector3d (point_3D[i].x, point_3D[i].y, point_3D[i].z));
        point->setMarginalized( true );
        optimizer.addVertex( point );
    }

    // ????????????????????????
    g2o::CameraParameters* camera = new g2o::CameraParameters(
            K.at<double> ( 0,0 ), Eigen::Vector2d ( K.at<double> ( 0,2 ), K.at<double> ( 1,2 ) ), 0
    );
    camera->setId( 0 );
    optimizer.addParameter( camera );


    // 添加边
    index =  1;
    for(auto i = 0; i < point_2D.size(); i++)
    {
        g2o::EdgeProjectXYZ2UV* edge = new g2o::EdgeProjectXYZ2UV();
        edge->setId( index );
        edge->setVertex(0, dynamic_cast<g2o::VertexSBAPointXYZ*>( optimizer.vertex( index )));
        edge->setVertex(1, pose);

        edge->setMeasurement( Eigen::Vector2d (point_2D[i].x, point_2D[i].y));
        edge->setParameterId( 0, 0);
        edge->setInformation(Eigen::Matrix2d::Identity());
        optimizer.addEdge( edge );
        index++;
    }

    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    optimizer.setVerbose ( true );
    optimizer.initializeOptimization();
    optimizer.optimize ( 100 );
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>> ( t2-t1 );
    cout << " 优化时间 optimization costs time: "<< time_used.count() << " seconds." <<endl;

    cout << endl << "after optimization:" << endl;
    cout << "变换矩阵 T = \n" << Eigen::Isometry3d ( pose->estimate() ).matrix() << endl;

}

// SVD 求解 R t
void pose_3D_3D(vector<cv::Point3f>& point3D_1, vector<cv::Point3f>& point3D_2, cv::Mat& R, cv::Mat& t)
{
    // 定义质心 质心为所有点的总和
    cv::Point3f p1, p2;
    for(auto i = 0; i < point3D_1.size(); i++)
    {
        p1 += point3D_1[i];
        p2 += point3D_2[i];
    }

    // 计算2组点的质心
    p1 = p1 / int(point3D_1.size());
    p2 = p2 / int(point3D_1.size());

    // 计算每个点 去质心坐标
    vector<cv::Point3f> new_p1, new_p2;
    for(auto i = 0; i < point3D_1.size(); i++)
    {
        new_p1[i] = point3D_1[i] - p1;
        new_p2[i] = point3D_2[i] - p2;
    }

    // 计算W 矩阵 W = new_p1 * new_p2.T
    Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
    for(auto i = 0; i < point3D_1.size(); i++)
    {
        W += Eigen::Vector3d( new_p1[i].x, new_p1[i].y, new_p1[i].z) * Eigen::Vector3d( new_p2[i].x, new_p2[i].y, new_p2[i].z).transpose();
    }

    // 对W  进行SVD 分解 求U V
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();
    cout << " U = \n" << U << " V = \n " << V << endl;

    // 计算 R = U * V.T
    Eigen::Matrix3d R_ = U * ( V.transpose() );

    // 计算 t = p1 - R * p2
    Eigen::Vector3d t_ = Eigen::Vector3d( p1.x, p2.y, p1.z) - R_ * Eigen::Vector3d( p2.x, p2.y, p2.z);

    cout << " R_  = \n" << R_  << endl;

    R = (cv::Mat_<double>(3, 3) <<
            R_(0, 0), R_(0, 1), R_(0, 2),
            R_(1, 0), R_(1, 1), R_(1, 2),
            R_(2, 0), R_(2, 1), R_(2, 2)
            );

    t = (cv::Mat_<double>(3, 1) <<
            t_(0, 0), t_(1, 0), t_(2, 0)
            );

}

int main(int argc, char** argv)
{
    cv::Mat rgb1 = cv::imread(rgb1_path);
    cv::Mat rgb2 = cv::imread(rgb2_path);

    if(rgb1.empty())
    {
        cerr << "Do not find rgb1.png .............." << endl;
        return -1;
    }
    if(rgb2.empty())
    {
        cerr << "Do not find rgb2.png .............." << endl;
        return -1;
    }

    vector<cv::KeyPoint> keypoints_1, keypoints_2;
    vector<cv::DMatch> goodMatchers;
    find_feature_matches(rgb1, rgb2, keypoints_1, keypoints_2, goodMatchers);
    cout << "匹配点的个数 Size = \n" << goodMatchers.size() << endl;


    // 建立3D点
    cv::Mat depth1 = cv::imread(depth1_path);
    cv::Mat depth2 = cv::imread(depth2_path);

    cv::Mat K = ( cv::Mat_<double > (3, 3) <<  520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1 );

    vector<cv::Point3f> points_3D_1;
    vector<cv::Point3f> points_3D_2;

    for(auto i = 0; i < goodMatchers.size(); i++)
    {
        double depth_1 = depth1.ptr<unsigned short >(int( keypoints_1[goodMatchers[i].queryIdx].pt.y))
        [ int( keypoints_1[goodMatchers[i].trainIdx].pt.x) ];

        double depth_2 = depth2.ptr<unsigned short >(int( keypoints_2[goodMatchers[i].queryIdx].pt.y))
        [int( keypoints_2[goodMatchers[i].trainIdx].pt.x) ];

        if ( depth_1 == 0  || depth_2 == 0)
        {
            continue;
        }

        float dd_1 = depth_1 / 5000;
        float dd_2 = depth_2 / 5000;

        cv::Point2d p1 = pixel2cam( keypoints_1[goodMatchers[i].queryIdx].pt, K);
        cv::Point2d p2 = pixel2cam( keypoints_2[goodMatchers[i].queryIdx].pt, K);


        points_3D_1.emplace_back( cv::Point3f (p1.x * dd_1, p1.y * dd_1, dd_1) );

        points_3D_2.emplace_back( cv::Point3f (p2.x * dd_2, p2.y * dd_2, dd_2));
    }

    // SVD 求解 R t
    cv::Mat R, t;
    pose_3D_3D(points_3D_1, points_3D_2, R, t);



    cout << "Hello slam VO......" << endl;
    return 0;
}