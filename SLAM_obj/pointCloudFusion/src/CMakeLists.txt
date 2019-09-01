#cmake_minimum_required(VERSION 2.8)

#project(Draw_trajectory)


# 配置opencv
find_package(OpenCV REQUIRED)
#set(SOURCE_FILES main.cpp)
include_directories(${OpenCV_INCLUDE_DIRS})

# 配置pcl  #
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
ADD_DEFINITIONS( ${PCL_DEFINITIONS} )
LINK_LIBRARIES( ${PCL_LIBRARY_DIRS} )

# 配置Egien
include_directories(/usr/include/eigen3)

# 配置g2o库
# list(G20_ROOT /usr/local/include/g2o)
LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/src/cmake_modules )
find_package( G2O REQUIRED )
include_directories(${G2O_INCLUDE_DIRS})

# 配置 csparse   #
find_package( CSparse REQUIRED )
include_directories(${CSPARSE_INCLUDE_DIR})

# 配置Sophus 李群 李代数
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

# 配置 Pangolin
find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})


include_directories( ${PROJECT_SOURSE_DIR}/include )

# 头文件
add_library(slambase_shared SHARED slamBase.cpp)
target_link_libraries(slambase_shared
                        ${PCL_LIBRARIES}  # pcl
                        ${OpenCV_LIBS}    # opencv
                        g2o_core g2o_stuff g2o_types_sba g2o_csparse_extension # g2o
                        ${CSPARSE_LIBRARY} # csparse
                        ${Sophus_LIBRARY} # Sophus
                        ${Pangolin_LIBRARIES}) # Pangolin

# cpp文件
add_executable(pointCloudFusion "pointCloudFusion.cpp")     # 两两匹配点云
target_link_libraries (pointCloudFusion slambase_shared
                        ${PCL_LIBRARIES}  # pcl
                        ${OpenCV_LIBS}    # opencv
                        g2o_core g2o_stuff g2o_types_sba g2o_csparse_extension # g2o
                        ${CSPARSE_LIBRARY} # csparse
                        ${Sophus_LIBRARY} # Sophus
                        ${Pangolin_LIBRARIES}) # Pangolin
#target_compile_features(Draw_trajectory PUBLIC cxx_auto_type)

