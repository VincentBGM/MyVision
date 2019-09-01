cmake_minimum_required(VERSION 3.14.3)

project(Epipolar)


# 配置opencv
find_package(OpenCV REQUIRED)
#set(SOURCE_FILES main.cpp)
include_directories(${OpenCV_INCLUDE_DIRS})

# 配置pcl  #
#find_package(PCL REQUIRED)
#include_directories(${PCL_INCLUDE_DIRS})

# 配置Egien
include_directories(/usr/include/eigen3)

# 配置g2o库
#LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
#find_package( G2O REQUIRED )
#include_directories(${G2O_INCLUDE_DIRS})

# 配置 csparse   #
#find_package( CSparse REQUIRED )
#include_directories(${CSPARSE_INCLUDE_DIR})

# 配置Sophus 李群 李代数
#FIND_PACKAGE(Sophus REQUIRED)
#include_directories(${Sophus_INCLUDE_DIRS})

# 配置 Pangolin
#find_package(Pangolin REQUIRED)
#include_directories(${Pangolin_INCLUDE_DIRS})


add_executable(${PROJECT_NAME} "epipolar.cpp")     # 两两匹配点云

#target_link_libraries (Epipolar ${PCL_LIBRARIES})  # pcl
target_link_libraries (Epipolar  ${OpenCV_LIBS})    # opencv
#target_link_libraries (Epipolar g2o_core g2o_stuff g2o_types_sba g2o_csparse_extension ) # g2o
#target_link_libraries (Epipolar ${CSPARSE_LIBRARY}) # csparse
#target_link_libraries (Epipolar ${Sophus_LIBRARY}) # Sophus
#target_link_libraries(Epipolar ${Pangolin_LIBRARIES}) # Pangolin
#target_compile_features(Epipolar PUBLIC cxx_auto_type)


