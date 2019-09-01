cmake_minimum_required(VERSION 2.8)

project(pose_graph)

SET( CMAKE_CXX_FLAGS "-std=c++11" )#设定编译器
set(CMAKE_BUILD_TYPE "Release")

# 配置opencv
#find_package(OpenCV REQUIRED)
#include_directories(${OpenCV_INCLUDE_DIRS})

# 配置pcl  #
#find_package(PCL REQUIRED)
#include_directories(${PCL_INCLUDE_DIRS})
#ADD_DEFINITIONS( ${PCL_DEFINITIONS} )
#LINK_LIBRARIES( ${PCL_LIBRARY_DIRS} )
#list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

# 配置Egien
include_directories(/usr/include/eigen3)

# 配置g2o库
LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
find_package( G2O REQUIRED )
include_directories(${G2O_INCLUDE_DIRS})


find_package( Cholmod REQUIRED )
include_directories( ${CHOLMOD_INCLUDE_DIR} )

# 配置 csparse   #
#find_package( CSparse REQUIRED )
#include_directories(${CSPARSE_INCLUDE_DIR})

# 配置Sophus 李群 李代数
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})


# 配置 gtsam 
find_package( GTSAMCMakeTools )
find_package( GTSAM REQUIRED )
include_directories( ${GTSAM_INCLUDE_DIR} )

# Ceres 
find_package( Ceres REQUIRED )
include_directories( ${CERES_INCLUDE_DIRS} )

# 配置 Pangolin
#find_package(Pangolin REQUIRED)
#include_directories(${Pangolin_INCLUDE_DIRS})


add_executable( pose_graph_g2o_SE3 pose_graph_g2o_SE3.cpp )
target_link_libraries( pose_graph_g2o_SE3
    		           g2o_core g2o_stuff g2o_types_sba g2o_csparse_extension g2o_types_slam3d
					   ${CHOLMOD_LIBRARIES}
		               ${Sophus_LIBRARY} # Sophus
		             )

add_executable( pose_graph_g2o_lie pose_graph_g2o_lie_algebra.cpp )
target_link_libraries( pose_graph_g2o_lie
    		          g2o_core g2o_stuff g2o_types_sba g2o_csparse_extension g2o_types_slam3d
    		          ${CHOLMOD_LIBRARIES}
                      ${Sophus_LIBRARIES}
		              ${Sophus_LIBRARY} # Sophus
		             )

add_executable( pose_graph_gtsam pose_graph_gtsam.cpp )
target_link_libraries( pose_graph_gtsam
                      ${CHOLMOD_LIBRARIES} #gtsam
		              ${Sophus_LIBRARY} # Sophus
                     )



#target_link_libraries( feature_extraction 
                        #${PCL_LIBRARIES}  # pcl
                        #${OpenCV_LIBS}    # opencv
                        #g2o_core g2o_stuff g2o_types_sba g2o_csparse_extension # g2o
                        #${CSPARSE_LIBRARY} # csparse
                        #${Sophus_LIBRARY} # Sophus
                        #${Pangolin_LIBRARIES}) # Pangolin



