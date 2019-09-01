
#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>

using namespace std;



int main (int argc, char** argv)
{
    if (argc != 2)
    {
        cout << "Usage: Don't find g2o_file, please input g2o_file....." << endl;
        return 1;
    }

    ifstream file(argv[1]);
    if ( !file )
    {
        cout << "File " << argv[1] << "is not find....." << endl;
        return 1;
    }

    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 6> > Block;
    Block::LinearSolverType* linearSolver = new g2o::LinearSolverCholmod<Block::PoseMatrixType>();
    Block* solver_ptr = new Block(std::unique_ptr<Block::LinearSolverType> (linearSolver) );
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(std::unique_ptr<Block >(solver_ptr)); // 梯度下降方法，从GN, LM, DogLeg 中选

    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm( solver );

    int vertex_num = 0;
    int edge_num = 0;

    while ( !file.eof() )
    {
        string name;
        file >> name;

        if ( name == "VERTEX_SE3:QUAT")
        {
            auto* vertex = new g2o::VertexSE3();

            int index = 0;
            file >> index;

            vertex->setId( index );
            vertex->read( file );

            optimizer.addVertex( vertex );

            vertex_num++;
            if (index ==  0 )
            {
                vertex->setFixed( true );
            }

        }
        else if ( name == "EDGE_SE3:QUAT")
        {
            auto* edge = new g2o::EdgeSE3();

            int index_1 = 0;
            int index_2 = 0;
            file >> index_1 >> index_2;

            edge->setId( edge_num++ );

            edge->setVertex(0, optimizer.vertices()[index_1]);
            edge->setVertex(1, optimizer.vertices()[index_2]);

            edge->read( file );

            optimizer.addEdge( edge );

        }

        if ( !file.good() )
        {
            break;
        }

    }


    cout << "Vertex numbers = " << vertex_num << endl << "Edge numbers = " << edge_num << endl;

    optimizer.setVerbose( true );
    optimizer.initializeOptimization();

    cout << " Begin optimizer ..............." << endl;

    optimizer.optimize( 30 );

    cout << "saving optimizer result ...." << endl;
    optimizer.save("./result.g2o");

    cout << "Saving optimizer finished....." << endl;


    return 0;
}