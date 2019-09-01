cmake_minimum_required(VERSION 2.8)

project( BA )

#SET( CMAKE_CXX_FLAGS "-std=c++11" )#设定编译器

# 配置opencv
find_package(OpenCV REQUIRED)
#set(SOURCE_FILES main.cpp)
include_directories(${OpenCV_INCLUDE_DIRS})


# 配置pcl
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})


# 配置Egien
include_directories(/usr/include/eigen3)


# 配置g2o库
# list(G20_ROOT /usr/local/include/g2o)
LIST( APPEND CMAKE_MODULE_PATH /home/vincent/g2o-master/cmake_modules )
#set(G2O_ROOT /home/vincent/g2o-master/g2o)
find_package( G2O REQUIRED )
include_directories(${G2O_INCLUDE_DIRS})

# 配置 csparse
find_package( CSparse REQUIRED )
include_directories(${CSPARSE_INCLUDE_DIR})

# 配置Sophus 李群 李代数
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

# 配置 Pangolin
find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})


add_executable(${PROJECT_NAME} "addEdge.cpp")
target_link_libraries (BA  ${PCL_LIBRARIES})  # pcl
target_link_libraries (BA  ${OpenCV_LIBS})    # opencv
target_link_libraries (BA g2o_core g2o_stuff g2o_types_sba g2o_csparse_extension ) # g2o
target_link_libraries (BA ${CSPARSE_LIBRARY}) # csparse
target_link_libraries (BA ${Sophus_LIBRARY}) # Sophus
target_link_libraries (BA ${Pangolin_LIBRARIES}) # Pangolin