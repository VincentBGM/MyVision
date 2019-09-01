
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
#include <sstream>

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

#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_levenberg.h>


#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

#define cout cout << "[" << __FILE__ << ":" << __LINE__ << "]"

using namespace std;


#endif //POINTCLOUD_COMMON_INCLUDE_H
