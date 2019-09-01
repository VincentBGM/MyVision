
#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include <g2o/core/block_solver.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_edge.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>

#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>

#include <Eigen/Dense>

#include <sophus/se3.h>
#include <sophus/so3.h>

using namespace std;


typedef Eigen::Matrix<double, 6, 6> Matrix6d;

// 给定误差求J_R^{-1}的近似
Matrix6d JRInv( Sophus::SE3 e )
{
    Matrix6d J;
    J.block(0,0,3,3) = Sophus::SO3::hat(e.so3().log());
    J.block(0,3,3,3) = Sophus::SO3::hat(e.translation());
    J.block(3,0,3,3) = Eigen::Matrix3d::Zero(3,3);
    J.block(3,3,3,3) = Sophus::SO3::hat(e.so3().log());
    J = J*0.5 + Matrix6d::Identity();
    return J;
}



class VertexSE3LieAlgebra : public g2o::BaseVertex<6, Sophus::SE3>
{
public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    bool read(istream& is)
    {
        double data[7];
        for (double & i : data)
        {
            is >> i;
        }

        setEstimate(Sophus::SE3 (
                Eigen::Quaterniond(data[7], data[4], data[5], data[6]),
                Eigen::Vector3d(data[0], data[1], data[2])
                ));
        return true;
    }

    bool write(ostream& os) const
    {
        os << id() << " ";
        Eigen::Quaterniond q  = _estimate.unit_quaternion();

        os << _estimate.translation().transpose() << " ";
        os<<q.coeffs()[0]<<" "<<q.coeffs()[1]<<" "<<q.coeffs()[2]<<" "<<q.coeffs()[3]<<endl;

        return true;

    }

    virtual void setToOriginImpl()
    {
        _estimate = Sophus::SE3();
    }

    virtual void oplusImpl(const double* update)
    {
        Sophus::SE3 up(
                Sophus::SO3(update[3], update[4], update[5]),
                Eigen::Vector3d( update[0], update[1], update[2])
                );

        _estimate = up * _estimate;
    }


};


class EdgeSE3LieAlgebra : public g2o::BaseBinaryEdge<6, Sophus::SE3, VertexSE3LieAlgebra, VertexSE3LieAlgebra>
{
public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    bool read(istream& is)
    {
        double data[7];
        for (auto i : data )
        {
            is >> i;
        }

        Eigen::Quaterniond q(data[6], data[3], data[4], data[5]);
        q.normalize();

        setMeasurement( Sophus::SE3 (q, Eigen::Vector3d(data[0], data[1], data[2]) ) );

        for ( int i=0; i<information().rows() && is.good(); i++ )
        {
            for (int j = i; j < information().cols() && is.good(); j++)
            {
                is >> information()(i, j);
                if (i != j)
                {
                    information()(j, i) = information()(i, j);
                }
            }
        }

        return true;
    }

    bool write(ostream& os) const
    {
        VertexSE3LieAlgebra* v1 = static_cast<VertexSE3LieAlgebra*>(_vertices[0] );
        VertexSE3LieAlgebra* v2 = static_cast<VertexSE3LieAlgebra*>(_vertices[1] );


    }

    virtual void computer()
    {

    }

    virtual void linearizeOplus()
    {

    }



};



int main (int argc, char** argv)
{
    if (argc != 2)
    {
        cout << "Usage: please input g2o_file ............." << endl;
        return 1;
    }

    fstream g2o_file(argv[1]);

    if ( !g2o_file )
    {
        cout << "Waring: Don't find g2o_file....." << endl;
        return 1;
    }


    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 6> > Block;
    Block::LinearSolverType* linearSolver = new g2o::LinearSolverCholmod<Block::PoseMatrixType>;
    Block* solver_ptr = new Block(std::unique_ptr<Block::LinearSolverType>( linearSolver ) );
    g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton(std::unique_ptr<Block>( solver_ptr ));
//    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(std::unique_ptr<Block >(solver_ptr ));
//    g2o::OptimizationAlgorithmDogleg* solver = new g2o::OptimizationAlgorithmDogleg(std::unique_ptr<Blokc >( solver_ptr ));


    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm( solver );




    return 0;
}

