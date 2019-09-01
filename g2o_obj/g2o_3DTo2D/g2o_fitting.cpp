#include <iostream>
#include <cmath>
#include <chrono>

#include <g2o/core/base_vertex.h>     // 顶点类
#include <g2o/core/base_unary_edge.h> // 边类

#include <g2o/core/block_solver.h>    // 求解器
#include <g2o/solvers/dense/linear_solver_dense.h> // 线性求解器

#include <g2o/core/optimization_algorithm_levenberg.h>  // LM 优化器类
#include <g2o/core/optimization_algorithm_gauss_newton.h>  // GN 高斯牛顿优化器
#include <g2o/core/optimization_algorithm_dogleg.h>  // Powell'sdogleg 优化器

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Core>

#include <opencv2/core/core.hpp>

using namespace std;


// 曲线模型的顶点， 模板参数：优化变量维度和数据类型
class  G2oFittingVertex : public g2o::BaseVertex<3, Eigen::Vector3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    virtual void setToOriginImpl()  // 重置
    {
        _estimate << 0, 0, 0;
    }

    virtual void oplusImpl( const double* update)  // 更新
    {
        _estimate += Eigen::Vector3d(update);
    }

    // 存盘 和 读盘： 留空
    virtual bool read( istream& in)
    {

    }

    virtual bool write(ostream& out) const
    {

    }
};

// 误差模型 模板参数：观测值维度， 类型， 连接顶点类型
class CurveFittingEdge: public g2o::BaseUnaryEdge<1, double, G2oFittingVertex>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    CurveFittingEdge(double x): BaseUnaryEdge(), _x(x)
    {

    }

    // 计算曲线模型误差
    void computeError()
    {
        const G2oFittingVertex* v = static_cast<const G2oFittingVertex*>(_vertices[0]);
        const Eigen::Vector3d abc = v->estimate();
        _error(0, 0) = _measurement - std::exp( abc(0, 0)*_x*_x + abc(1, 0)*_x + abc(2, 0));
    }

    virtual bool read( istream& in) {}
    virtual bool write( ostream& out) const {}

public:
    double _x;  // x 值， y值 为_measurement
};


int main(int argc, char** argv)
{
    double a = 1.0, b = 2.0, c = 1.0;    //
    int N = 100;                         // 数据点
    double w_sigma = 1.0;               // 噪声sigma
    cv::RNG rng;                        // opencv随机数产生器
    double abc[3] = {0, 0, 0};          // abc参数估计

    vector<double> x_data, y_data;      // 数据
    cout << "基础数据: " << endl;
    for(int i = 0; i < N; i++)
    {
        double x = i / 100.0;
        x_data.push_back(x);
        y_data.push_back(
                    exp( a*x*x + b*x + c) + rng.gaussian( w_sigma )
                    );

        cout << x_data[i] << " " << y_data[i] << endl;
    }

    // 构建图 优化
    // 每个误差项 优化变量维度为3, 误差维度为1
    typedef g2o::BlockSolver< g2o::BlockSolverTraits<3, 1> > Block;

    // 1、创建一个线性求解器 LinearSolver
    Block::LinearSolverType* linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>();  // 线性方程式求解器

    // 2、创建BLockSolver求解。使用上面定义的线性求解器初始化
    Block* solver_ptr = new Block(linearSolver);

    // 3、创建总求解器Solver. 选择GN LM DogLeg优化算法。再用BlockSolver求解其 初始化
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

    // 4、创建稀疏优化器 SparseOptimizer
    g2o::SparseOptimizer optimizer;     // 图模型
    optimizer.setAlgorithm(solver);     // 设置求解器
    optimizer.setVerbose(true);         // 打开调试输出

    // 5、定义图的顶点 和 边， 并添加到SparseOptimizer中
    G2oFittingVertex* v = new G2oFittingVertex();  // 往图中增加顶点
    v->setEstimate( Eigen::Vector3d(0, 0, 0) );
    v->setId(0);
    optimizer.addVertex( v );
    for( int i = 0; i < N; i++)  // 往图中加边
    {
        CurveFittingEdge* edge = new CurveFittingEdge( x_data[i]);
        edge->setId(i);
        edge->setVertex(0, v);  // 设置连接的顶点
        edge->setMeasurement( y_data[i]);       // 观测数值
        edge->setInformation( Eigen::Matrix<double, 1, 1>::Identity() * 1 / (w_sigma * w_sigma ));      // 信息矩阵： 斜方差矩阵之逆
        optimizer.addEdge(edge);
    }

    // 6、设置优化参数，开始执行优化
    cout << "开始优化" << endl;
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();

    optimizer.initializeOptimization();
    optimizer.optimize(100);

    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);

    cout << "优化时间为: " << time_used.count() << "秒" << endl;


    // 7、输出优化值
    Eigen::Vector3d abc_estimate = v->estimate();
    cout << "estimated model: " << abc_estimate.transpose() << endl;

    cout << "Hello SALM World!" << endl;
    return 0;
}
