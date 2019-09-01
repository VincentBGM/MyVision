
#include <iostream>
#include <vector>
#include <stdint.h>
#include <unordered_set>
#include <memory>
#include <stdlib.h>

#include <Eigen/Core>
#include <Eigen/StdVector>

#include <g2o/stuff/sampler.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/batch_stats.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_dogleg.h>

#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/solvers/pcg/linear_solver_pcg.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

#include <g2o/solvers/structure_only/structure_only_solver.h>

#include "common/BundleParams.h"
#include "common/BALProblem.h"
#include "g2o_bal_class.h"

using namespace std;

typedef Eigen::Map<Eigen::VectorXd> vectorRef;
typedef Eigen::Map<const Eigen::VectorXd> constVectorRef;
//给块求解器模板类定义维度并typedef，pose的维度为9维，landmark维度为3维
typedef g2o::BlockSolver<g2o::BlockSolverTraits<9, 3> > blockSolver;


// 对边和节点 进行优化
//问题构建函数，传入一个BALProblem类型指针，稀疏求解器指针，参数类引用BundleParams&
void buildProblem(const BALProblem* bal_problem, g2o::SparseOptimizer* optimizer, const BundleParams& params)
{
    //将bal_problem中的数据读出来
    const int num_points = bal_problem->num_points();
    const int num_camares = bal_problem->num_cameras();
    const int camare_block_size = bal_problem->camera_block_size();
    const int point_block_siez = bal_problem->point_block_size();

    // 初始化相机节点
    //将相机数据的首位置读出，用于后方数据读取
    const double* raw_camaras = bal_problem->cameras();
    for( auto i = 0; i < num_camares; ++i )
    {
        //这里将9维相机位姿从数组中取出来构建成矩阵，用于下面的顶点的初始值赋值
        constVectorRef temVectorCamera( raw_camaras + camare_block_size * i, camare_block_size );
        VertexCameraBAL* pCamare;   //开辟个新的相机顶点类指针
        pCamare->setEstimate(temVectorCamera );             //设定初始值
        pCamare->setId(i);                                  //设定ID

        optimizer->addVertex( pCamare );                     // remeber to add vertex into optimizer..
    }

    // Set point vertex with initial value in the dataset.
    //同样，这里将路标数据的首位置读出，用于后面读取
    const double* raw_points = bal_problem->points();
    for (auto i = 0; i < num_points; ++i)
    {
        //同样，将数组中的路边数据读出构建成矩阵
        constVectorRef temVectorPoints ( raw_points + point_block_siez * i, point_block_siez );
        VertexPointsBAL* pPoint;    //开辟个新的路标顶点指针
        pPoint->setEstimate( temVectorPoints );             //设定初始值
        pPoint->setId( i + num_camares );                   //设定ID，不能跟上面的相机顶点重复，所以加了个相机个数，直接往后排

        pPoint->setMarginalized(true);                       //由于路标要被边缘化计算，所以设置边缘化属性为true
        optimizer->addVertex( pPoint );                     //将顶点添加进优化器
    }

    // 添加边到图中
    const int num_observations = bal_problem->num_observations();   //取出边的个数
    const double* observations = bal_problem->observations();       //取出边数组首位置
    //用观测个数控制循环，来添加所有的边
    for (auto i = 0; i < num_observations; ++i)
    {
        EdgeObservationBAL* pEdge;        //开辟边内存指针

        //由于每条边要定义连接的顶点ID，所以这里将cameraID和pointID取出
        const int camare_id = bal_problem->camera_index()[i];
        const int point_id = bal_problem->point_index()[i] + num_camares;

        // ????????????????????????????????????????????????????????????????????????
        if ( params.robustify )
        {
            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            rk->setDelta(1.0);
            pEdge->setRobustKernel(rk);
        }

        // set the vertex by the ids for an edge observation
        // 这里对每条边进行设置：
        // 连接的两个顶点
        pEdge->setVertex(0, dynamic_cast<VertexCameraBAL* >(optimizer->vertex(camare_id)) );
        pEdge->setVertex(1, dynamic_cast<VertexPointsBAL* >(optimizer->vertex(point_id)) );

        //信息矩阵，这里依旧是单位阵
        pEdge->setInformation(Eigen::Matrix2d::Identity() );

        //设置默认值，就是将观测数据读进去
        pEdge->setMeasurement(Eigen::Vector2d(observations[2 * i + 0], observations[2 * i + 1] ) );

        //将边添加进优化器
        optimizer->addEdge( pEdge );
    }


}


//再看一下程序各个类作用：
//BALProblem跟优化数据txt对接，负责txt的读取、写入，同时还有生成PLY点云文件的功能
//BundleParams类负责优化需要的参数值，默认值设定和用户命令行输入等功能。
//整体这样归类之后，所有优化数据就去BALProblem类对象中询问，参数就去BundleParams类对象询问。

//这个函数的作用是将优化后的结果再写入到BALProblem类中，
//注意，在BALProblem类中，定义的所有读取写入功能都是BALProblem类与txt数据的，并没有优化后的数据与BALProblem的，
//所以这里定义了之后，就会产生优化后的数据类BALProblem，这样再跟txt或者PLY对接的话就很容易了。
//参数很容易理解，被写入的BALProblem*,优化器
void writeToBALProblem(BALProblem* bal_problem, g2o::SparseOptimizer* optimizer )
{
    //将bal_problem中的数据读出来
    const int num_points = bal_problem->num_points();
    const int num_camares = bal_problem->num_cameras();
    const int camare_block_size = bal_problem->camera_block_size();
    const int point_block_siez = bal_problem->point_block_size();

    //用mutable_cameras()函数取得相机首地址，用于后面的数据写入
    double* raw_camaras = bal_problem->mutable_cameras();
    for (auto i = 0; i < num_points; ++i )
    {
        //将相机顶点取出，这里说一下为什么要做这一步指针类型转化，因为optimizer->vertex(i)返回的类型是个vertex*指针类型，
        //需要将其转化成VertexCameraBAL*才能访问估计值，直接像下面的用法会报错：
        //optimizer->vertex(i)-> estimate();
        //原程序是下面这样写的，但是感觉这里用auto比较方便一些，并且也更能体现pCamera仅是个承接的功能。
        //VertexCameraBAL* pCamera = dynamic_cast<VertexCameraBAL*>(optimizer->vertex(i));
        VertexPointsBAL* pPoints = dynamic_cast<VertexPointsBAL* >(optimizer->vertex(i + num_camares ));
        Eigen::Vector3d new_PointVec = pPoints->estimate();

        //取得估计值之后，就可以memcpy()了，这里当然是一个9维的数组，长度上很明显是9*double
        memcpy(raw_camaras + i * camare_block_size, new_PointVec.data(), sizeof(double) * camare_block_size );
    }

    double* raw_points = bal_problem->mutable_points();
    for(int j = 0; j < num_points; ++j)
    {
        VertexPointsBAL* pPoint = dynamic_cast<VertexPointsBAL*>(optimizer->vertex(j + num_camares));
        Eigen::Vector3d NewPointVec = pPoint->estimate();
        memcpy(raw_points + j * point_block_siez, NewPointVec.data(), sizeof(double) * point_block_siez);
    }

}

//this function is  unused yet..
void SetMinimizerOptions(std::shared_ptr<blockSolver>& solver_ptr, const BundleParams& params, g2o::SparseOptimizer* optimizer)
{
    //std::cout<<"Set Minimizer  .."<< std::endl;
    g2o::OptimizationAlgorithmWithHessian* solver;
    if(params.trust_region_strategy == "levenberg_marquardt")
    {
//        g2o::OptimizationAlgorithmLevenberg* solver;
        solver = new g2o::OptimizationAlgorithmLevenberg(std::unique_ptr<blockSolver>(solver_ptr.get()));
    }
    else if(params.trust_region_strategy == "dogleg")
    {
//        g2o::OptimizationAlgorithmDogleg* solver;
        solver = new g2o::OptimizationAlgorithmDogleg(std::unique_ptr<blockSolver>(solver_ptr.get()));
    }
    else
    {
        std::cout << "Please check your trust_region_strategy parameter again.."<< std::endl;
        exit(EXIT_FAILURE);
    }

    optimizer->setAlgorithm(solver);
    //std::cout<<"Set Minimizer  .."<< std::endl;
}

//this function is  unused yet..
void SetLinearSolver(std::shared_ptr<blockSolver>& solver_ptr, const BundleParams& params)
{
    //std::cout<<"Set Linear Solver .."<< std::endl;
    g2o::LinearSolver<blockSolver::PoseMatrixType>* linearSolver = nullptr;

    if(params.linear_solver == "dense_schur" )
    {
        linearSolver = new g2o::LinearSolverDense<blockSolver::PoseMatrixType>();
    }
    else if(params.linear_solver == "sparse_schur")
    {
        linearSolver = new g2o::LinearSolverCholmod<blockSolver::PoseMatrixType>();
        dynamic_cast<g2o::LinearSolverCholmod<blockSolver::PoseMatrixType>* >(linearSolver)->setBlockOrdering(true);  // AMD ordering , only needed for sparse cholesky solver
    }


    solver_ptr = std::make_shared<blockSolver>(linearSolver);
    std::cout <<  "Set Complete.."<< std::endl;
}


//求解设置：使用哪种下降方式，使用哪类线性求解器
/**
 * 设置求解选项,其实核心就是构建一个optimizer
 * @param bal_problem 优化数据
 * @param params 优化参数
 * @param optimizer 稀疏优化器
 */
void setSolverOptionsFromFlags(BALProblem* bal_problem, const BundleParams& params, g2o::SparseOptimizer* optimizer )
{
    blockSolver* solver_ptr;

    g2o::LinearSolver<blockSolver::PoseMatrixType>* linearSolver = nullptr;

    //使用稠密计算方法
    if ( params.linear_solver == "dense_schur")
    {
        linearSolver = new g2o::LinearSolverDense<blockSolver::PoseMatrixType>();
    }
    else if (params.linear_solver == "sparse_schur")
    {
        linearSolver = new g2o::LinearSolverDense<blockSolver::PoseMatrixType>();
        dynamic_cast<g2o::LinearSolverCholmod<blockSolver::PoseMatrixType>* >(linearSolver)->setBlockOrdering(true);
    }

    //将线性求解器对象传入块求解器中，构造块求解器对象
    solver_ptr = new blockSolver(std::unique_ptr<blockSolver::LinearSolverType>(linearSolver ) );

    //将块求解器对象传入下降策略中，构造下降策略对象
    g2o::OptimizationAlgorithmWithHessian* solver;
    //根据参数选择是LM还是DL
    if ( params.trust_region_strategy == "levenberg_marquardt")
    {
        solver =new g2o::OptimizationAlgorithmLevenberg(std::unique_ptr<blockSolver>(solver_ptr) );
    }
    else if (params.trust_region_strategy == "dogleg" )
    {
        solver = new g2o::OptimizationAlgorithmDogleg(std::unique_ptr<blockSolver>(solver_ptr) );
    }
    else
    {
        cout << "Please check your trust_region_strategy parameter again.." << endl;
        exit(EXIT_FAILURE);
    }

    //将下降策略传入优化器的优化逻辑中，至此，一个优化器就构建好了
    optimizer->setAlgorithm(solver );
}


//开始优化，这个优化函数参数就是待优化文件和优化参数
void solverProblem(const char* filename, const BundleParams& params)
{
    BALProblem bal_problem(filename);

    cout << " loading bal file ...." << endl;
    //.num_cameras()返回num_cameras_ 值，显示相机数量
    //.num_points()返回num_points_ 值，显示路标数量
    cout << "bal problem have " << bal_problem.num_cameras() << " camares and " << bal_problem.num_points() << "points" << endl;

    //.num_observations()返回num_observations_ 值，显示观测边的数量
    cout << "Forming" << bal_problem.num_observations() << "observations." << endl;

    // store the initial 3D cloud points and camera pose..
    if (!params.initial_ply.empty() )
    {
        //优化前将BALProblem类中的数据生成一下点云数据，因为优化后，这个类中的数据会被覆盖
        bal_problem.WriteToPLYFile(params.initial_ply);
    }

    cout << " beginning problem..." << endl;

    // add some noise for the intial value
    srand(params.random_seed);
    //这里发现就用到了Normalize()， 对数据的处理，
    bal_problem.Normalize();
    bal_problem.Perturb(params.rotation_sigma, params.translation_sigma, params.point_sigma );

    cout << "Normalize complete....." << endl;

    //创建一个稀疏优化器对象
    g2o::SparseOptimizer optimizer;
    //用SetSolverOptionsFromFlags()对优化器进行设置
    setSolverOptionsFromFlags(&bal_problem, params, &optimizer);
    //设置完后，用BuildProblem()进行优化，参数也很清晰了：数据，优化器，参数
    buildProblem(&bal_problem, &optimizer, params);


    std::cout << "begin optimizaiton .."<< std::endl;
    // perform the optimizaiton
    //开始优化
    optimizer.initializeOptimization();
    //输出优化信息
    optimizer.setVerbose(true);
    optimizer.optimize(params.num_iterations);

    std::cout << "optimization complete.. "<< std::endl;
    // write the optimized data into BALProblem class
    //优化完后，将优化的数据写入BALProblem类，此时这个类中原始数据已经被覆盖，不过没关系，在优化前，它已经生成过PLY点云文件了
    writeToBALProblem(&bal_problem, &optimizer);

    // write the result into a .ply file.
    if(!params.final_ply.empty())
    {
        //优化后，将优化后的数据生成点云文件
        bal_problem.WriteToPLYFile(params.final_ply);
    }

}


int main(int argc, char** argv)
{
    BundleParams params(argc, argv);

    if ( params.input.empty() )
    {
        cout << "Usage: bundle_adjuster-input < path for dataset >";
        return 1;
    }

    solverProblem(params.input.c_str(), params);

    return 0;
}