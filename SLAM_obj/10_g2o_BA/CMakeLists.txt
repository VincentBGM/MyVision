cmake_minimum_required(VERSION 2.8)

project(g2o_BA)

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
# list(G20_ROOT /usr/local/include/g2o)
LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
find_package( G2O REQUIRED )
include_directories(${G2O_INCLUDE_DIRS})

find_package( Cholmod REQUIRED )

# 配置 csparse   #
#find_package( CSparse REQUIRED )
#include_directories(${CSPARSE_INCLUDE_DIR})

# 配置Sophus 李群 李代数
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

# 配置 Pangolin
#find_package(Pangolin REQUIRED)
#include_directories(${Pangolin_INCLUDE_DIRS})


include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


SET(G2O_LIBS g2o_cli g2o_ext_freeglut_minimal g2o_simulator g2o_solver_slam2d_linear 
    g2o_types_icp g2o_types_slam2d g2o_types_sba g2o_types_slam3d g2o_core g2o_interface 
    g2o_solver_csparse g2o_solver_structure_only g2o_csparse_extension g2o_opengl_helper g2o_solver_dense 
    g2o_stuff g2o_types_sclam2d g2o_parser g2o_solver_pcg g2o_types_data g2o_types_sim3 cxsparse )


include_directories(${EIGEN3_INCLUDE_DIR} ${CHOLMOD_INCLUDE_DIR} ${PROJECT_SOURCE_DIR}/ceres) 
include_directories(${PROJECT_SOURCE_DIR}/common
                    ${PROJECT_SOURCE_DIR}/common/tools 
                    ${PROJECT_SOURCE_DIR}/common/flags)


message("find g2o libraries in: " ${G2O_INCLUDE_DIR})
message("find g2o lib in " ${G2O_LIBS})
message("find cholmod in " ${CHOLMOD_INCLUDE_DIR})


add_library(BALProblem SHARED ${PROJECT_SOURCE_DIR}/common/BALProblem.cpp)
add_library(ParseCmd SHARED ${PROJECT_SOURCE_DIR}/common/flags/command_args.cpp)

add_executable(${PROJECT_NAME} g2o_bundle.cpp)


target_link_libraries(${PROJECT_NAME} ${G2O_LIBS} ${CHOLMOD_LIBRARIES} BALProblem ParseCmd)

#target_link_libraries( feature_extraction 
                        #${PCL_LIBRARIES}  # pcl
                        #${OpenCV_LIBS}    # opencv
                        #g2o_core g2o_stuff g2o_types_sba g2o_csparse_extension # g2o
                        #${CSPARSE_LIBRARY} # csparse
                        #${Sophus_LIBRARY} # Sophus
                        #${Pangolin_LIBRARIES}) # Pangolin



