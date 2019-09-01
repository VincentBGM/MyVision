
#include <vector>
#include <fstream>
#include <iostream>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>


#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/types/sba/types_six_dof_expmap.h>



using namespace cv;
using namespace std;


static string p3d_file = "./p3d.txt";
static string p2d_file = "./p2d.txt";

void bundleAdjustment (
        const vector<Point3f> points_3d,
        const vector<Point2f> points_2d,
        Mat& K );


int main(int argc, char **argv)
{


    vector< Point3f > p3d;
    vector< Point2f > p2d;

    // { fx,  0, cx,
    //    0, fy, cy,
    //    0,  0,   1
    // }
    Mat K = ( Mat_<double> ( 3,3 ) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1 );

    // 导入3D点和对应的2D点

    ifstream fp3d(p3d_file);
    if (!fp3d)
    {
        cout<< "No p3d.text file" << endl;
        return -1;
    }
    else
    {
        while (!fp3d.eof())
        {
            double pt3[3] = {0};
            for (auto &p:pt3)
            {
                fp3d >> p;
            }

            p3d.emplace_back(Point3f(pt3[0],pt3[1],pt3[2]));
        }
    }

//    cout << "=====================================================================" << endl;
//    for(auto it = p3d.begin(); it != p3d.end(); it++)
//    {
//        cout << "point3D data : " << *it << endl;
//    }


    ifstream fp2d(p2d_file);
    if (!fp2d)
    {
        cout<< "No p2d.text file" << endl;
        return -1;
    }
    else
    {
        while (!fp2d.eof())
        {
            double pt2[2] = {0};
            for (auto &p:pt2)
            {
                fp2d >> p;
            }

            p2d.emplace_back(Point2f (pt2[0], pt2[1]));
        }
    }

//    cout << "=====================================================================" << endl;
//    for (auto it = p2d.begin(); it != p2d.end(); it++)
//    {
//        cout << "p2D data : " << *it << endl;
//    }


    assert(p3d.size() == p2d.size());

    int iterations = 100;
    double cost = 0, lastCost = 0;
    int nPoints = p3d.size();
    cout << "points: " << nPoints << endl;

    bundleAdjustment ( p3d, p2d, K );
    return 0;
}

void bundleAdjustment ( const vector< Point3f > points_3d, const vector< Point2f > points_2d, Mat& K   )
{

    typedef g2o::BlockSolver< g2o::BlockSolverTraits<6,3> > Block;  // pose 维度为 6, landmark 维度为 3
    // 1、创建一个线性求解器LinearSolver
    Block::LinearSolverType* linearSolver = new g2o::LinearSolverCSparse<Block::PoseMatrixType>();

    // 2、创建BlockSolver。并用上面定义的线性求解器初始化
    Block* solver_ptr = new Block (  std::unique_ptr<Block::LinearSolverType>(linearSolver) );

    // 3、创建总求解器solver。并从GN, LM, DogLeg 中选一个，再用上述块求解器BlockSolver初始化
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( std::unique_ptr<Block>(solver_ptr) );

    // 4、创建稀疏优化器
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm ( solver );
    optimizer.setVerbose(true);

    //  5、计算旋转向量
//    Mat r, t;
//    solvePnP(points_3d, points_2d, K, Mat(), r, t, false);
//
//    Mat R;
//    Rodrigues(r, R);
//
//    Eigen::Matrix3d R_mat;
//    R_mat << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
//             R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
//             R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2);

    // 单位矩阵
    Eigen::Matrix3d R1;
    R1 << 1, 0, 0, 0, 1, 0, 0, 0, 1;


    // 定义图的顶点
    auto* pose = new g2o::VertexSE3Expmap();        // 相机的位姿pose
    pose->setId(0);
//    pose->setEstimate(g2o::SE3Quat(R_mat, Eigen::Vector3d (t.at<double>(0, 0), t.at<double>(1, 0), t.at<double>(2, 0))));
    pose->setEstimate(g2o::SE3Quat(R1, Eigen::Vector3d(0, 0, 0)));
    optimizer.addVertex( pose );


    int index = 1;
    for( const Point3f p:points_3d )    // landmarks
    {
        cout << "p.x = " << p.x << endl;
        cout << "p.y = " << p.y << endl;
        cout << "p.z = " << p.z << endl;


        g2o::VertexSBAPointXYZ* point_3d = new g2o::VertexSBAPointXYZ();
        point_3d->setId( index++ );
        point_3d->setEstimate(Eigen::Vector3d (p.x, p.y, p.z));
        point_3d->setMarginalized(true);
        optimizer.addVertex( point_3d );
    }


    // 6、设置相机内参  fx, (cx , cy), 0
    g2o::CameraParameters* camera = new g2o::CameraParameters (
            K.at<double> ( 0,0 ), Eigen::Vector2d ( K.at<double> ( 0,2 ), K.at<double> ( 1,2 ) ), 0);
    camera->setId ( 0 );
    optimizer.addParameter ( camera );

    // 7、设置边
    index = 1;
    for ( const Point2f p: points_2d )
    {
        cout << "p.x = " << p.x << endl;
        cout << "p.y = " << p.y << endl;

        g2o::EdgeProjectXYZ2UV* edge = new g2o::EdgeProjectXYZ2UV();
        edge->setId ( index );
        edge->setVertex ( 0, dynamic_cast<g2o::VertexSBAPointXYZ*> ( optimizer.vertex ( index ) ) );
        edge->setVertex ( 1, pose );
        edge->setMeasurement ( Eigen::Vector2d ( p.x, p.y ) );  //设置观测值
        edge->setParameterId ( 0,0 );
        edge->setInformation ( Eigen::Matrix2d::Identity() );
        optimizer.addEdge ( edge );
        index++;
    }


    // 7、设置优化参数，开始执行优化
    optimizer.setVerbose (true);
    optimizer.initializeOptimization();
    optimizer.optimize ( 100 );

    // 8、输出优化结果
    cout << endl << "after optimization:" << endl;
    cout << "T = " << endl << Eigen::Isometry3d ( pose->estimate() ).matrix() << endl;
}