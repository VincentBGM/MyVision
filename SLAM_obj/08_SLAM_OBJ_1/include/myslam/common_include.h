
#ifndef POINTCLOUD_COMMON_INCLUDE_H
#define POINTCLOUD_COMMON_INCLUDE_H


#include <iostream>
#include <vector>
#include <string>
#include <list>
#include <memory>
#include <set>
#include <unordered_map>
#include <map>
#include <algorithm>
#include <boost/timer.hpp>
#include <fstream>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/viz.hpp>

#include <sophus/se3.h>

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>

#define cout cout << "[" << __FILE__ << ":" << __LINE__ << "]"

using namespace std;


#endif //POINTCLOUD_COMMON_INCLUDE_H
