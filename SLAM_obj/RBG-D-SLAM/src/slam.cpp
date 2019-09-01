

#include "slamBase.hpp"

#include <iostream>

typedef g2o::BlockSolver_6_3 Block;
typedef g2o::LinearSolverEigen< Block::PoseMatrixType > LinearSolver;


int main(int argc, char** argv)
{
    ParameterReader pd;
    int startIndex = atoi(pd.getData("start_index").c_str() );
    int endIndex = atoi(pd.getData("end_index").c_str() );

    // 存放所有的关键帧
    vector<FRAME> keyFrames;
    int currIndex = startIndex;
    cout << "Start initialzing ...." << endl;
    // 上一帧数据？？？
    FRAME currFrame = readFrame(currIndex, pd);

    CAMERA_INTRINSIC_PARAMETERS camera = getDefaultCamera();
    computeKeyPoints(currFrame);
    PointCloud::Ptr cloud = image2PointCloud(currFrame.rgb, currFrame.depth, camera);

//    bool visualize = pd.getData("visualize_pointcloud") == string("yes");

    // g2o 优化
    cout << "Starts used g2o add Vertex and edge..." << endl;

    LinearSolver* linearSolver = new LinearSolver();
    linearSolver->setBlockOrdering( false );
    Block* solver_ptr = new Block ( std::unique_ptr<Block::LinearSolverType>(linearSolver));
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(std::unique_ptr<Block>(solver_ptr));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm( solver );
    optimizer.setVerbose(false);

    // 添加一个顶点
    g2o::VertexSE3* v = new g2o::VertexSE3();
    v->setId(currIndex);
    v->setEstimate(Eigen::Isometry3d::Identity() );  // 估计为单应矩阵
    v->setFixed(true);
    optimizer.addVertex( v );

    keyFrames.push_back(currFrame );

    double keyFrameThreshold = atof(pd.getData("keyframe_threshold").c_str() );
    bool check_loop_closure = pd.getData("check_loop_closure") == string("yes");

    for (currIndex = startIndex + 1; currIndex < endIndex; currIndex++ )
    {
        cout << "Read file " << currIndex << endl;
        FRAME currFrame = readFrame(currIndex, pd);
        computeKeyPoints(currFrame);

        CHECK_RESULT result = checkKeyframes(keyFrames.back(), currFrame, optimizer);


        // 根据匹配结果选择不同的结果
        switch ( result )
        {
            case NOT_MATCHED:
                cout << RED"Not enough inliers..." << endl;
                break;
            case TOO_FAR_AWAY:
                cout << RED"Too far away, may be an erro.." << endl;
                break;
            case TOO_CLOSE:
                // 太远了，可能出错了
                cout<<RESET"Too close, not a keyframe"<<endl;
                break;
            case KEYFRAME:
                // bu 远不近 ========================================非常重要
                // 检测回环
                if (check_loop_closure)
                {
                    checkNearbyLoops( keyFrames, currFrame, optimizer );
                    checkRandomLoops( keyFrames, currFrame, optimizer );
                }
                keyFrames.push_back( currFrame );

                break;
            default:
                break;

        }

    }

    // g2o 优化
    cout << "Start used g2o optimizer.." << endl;
    cout << RESET"Optimizer pose graph, vertics: " << optimizer.vertices().size() << endl;
    optimizer.save("./result_before.g2o");
    optimizer.initializeOptimization();
    optimizer.optimize(100);
    optimizer.save("./result_after.g2o");
    cout << RESET"Optimizer done..." << endl;

    // 点云拼接
    cout << "Start build PointCloud map..." << endl;
    PointCloud::Ptr outputCloud ( new PointCloud());
    PointCloud::Ptr tmp ( new PointCloud() );

    pcl::VoxelGrid<PointRGB > voxel;    // 网格滤波器， 主要是调整点云地图的分辨率
    pcl::PassThrough<PointRGB> pass;    // 按照z轴滤波， rgb相机有效深度有限， 把太远的去掉
    pass.setFilterFieldName("z");
    pass.setFilterLimits( 0.0, 4.0 );   //4m以上就不要了

    double gridsize = atof(pd.getData("voxel_grid").c_str());
    voxel.setLeafSize( gridsize, gridsize, gridsize );

    for (auto i = 0; i < keyFrames.size(); i++)
    {
        g2o::VertexSE3* vertex = dynamic_cast<g2o::VertexSE3* >(optimizer.vertex(keyFrames[i].frameID ));

        Eigen::Isometry3d pose = vertex->estimate(); //该帧优化后的位姿
        PointCloud::Ptr newCloud = image2PointCloud( keyFrames[i].rgb, keyFrames[i].depth, camera ); //转成点云
        // 以下是滤波
        voxel.setInputCloud( newCloud );
        voxel.filter( *tmp );
        pass.setInputCloud( tmp );
        pass.filter( *newCloud );
        // 把点云变换后加入全局地图中
        pcl::transformPointCloud( *newCloud, *tmp, pose.matrix() );
        *outputCloud += *tmp;

//        if ( visualize == true )
//        {
//            viewer.showCloud(cloud);
//        }

        tmp->clear();
        newCloud->clear();

    }

    voxel.setInputCloud( outputCloud );
    voxel.filter( *tmp );
    //存储
    pcl::io::savePCDFile( "./result.pcd", *tmp );

    cout<<"Final map is saved."<<endl;

    return 0;
}