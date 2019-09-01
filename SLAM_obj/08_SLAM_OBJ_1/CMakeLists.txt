CMAKE_MINIMUM_REQUIRED( VERSION 2.8 ) #设定版本

PROJECT(myslam) #设定工程名

SET( CMAKE_CXX_FLAGS "-std=c++11 -march=native -O3" )#设定编译器
set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_COMPILER "g++" )
#SET(CMAKE_BUILD_TYPE Debug)

#设定可执行二进制文件的目录
SET( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

#设定存放编译出来的库文件的目录
SET( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

# ========================dependencies===依赖项====
# 配置pcl  #
#find_package(PCL REQUIRED)
#include_directories(${PCL_INCLUDE_DIRS})
#ADD_DEFINITIONS( ${PCL_DEFINITIONS} )
#LINK_LIBRARIES( ${PCL_LIBRARY_DIRS} )
#list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")


# 配置Egien
include_directories(/usr/include/eigen3)

# 配置opencv
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# 配置g2o库
find_package( G2O REQUIRED )
include_directories(${G2O_INCLUDE_DIRS})

# 配置 csparse   #
find_package( CSparse REQUIRED )
include_directories(${CSPARSE_INCLUDE_DIR})

# 配置Sophus 李群 李代数
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

# 配置 Pangolin
#find_package(Pangolin REQUIRED)
#include_directories(${Pangolin_INCLUDE_DIRS})


set( THIRD_PARTY_LIBS 
    #${PCL_LIBRARIES}  # pcl
    ${OpenCV_LIBS}    # opencv
    g2o_core g2o_stuff g2o_types_sba g2o_csparse_extension # g2o
    ${CSPARSE_LIBRARY} # csparse
    ${Sophus_LIBRARY} # Sophus
#    ${Pangolin_LIBRARIES}
)


# =======================================
#设定头文件目录
INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/include)

#并且把该目录设为连接目录
LINK_DIRECTORIES( ${PROJECT_SOURCE_DIR}/lib)

#增加子文件夹，也就是进入源代码文件夹继续构建
add_subdirectory( ${PROJECT_SOURCE_DIR}/src )
add_subdirectory( ${PROJECT_SOURCE_DIR}/test )
