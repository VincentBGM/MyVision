cmake_minimum_required(VERSION 2.8)

project(Eigen)


# 配置opencv
#find_package(OpenCV REQUIRED)
#set(SOURCE_FILES main.cpp)
#include_directories(${OpenCV_INCLUDE_DIRS})


# 配置pcl
#find_package(PCL 1.2 REQUIRED)
#include_directories(${PCL_INCLUDE_DIRS})


# 配置Egien
include_directories(/usr/include/eigen3)


add_executable(${PROJECT_NAME} "main.cpp")
#target_link_libraries (Eigen ${PCL_LIBRARIES})  # pcl
#target_link_libraries (Eigen ${OpenCV_LIBS})    # opencv
