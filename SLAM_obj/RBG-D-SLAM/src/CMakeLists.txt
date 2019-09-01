#cmake_minimum_required(VERSION 2.8)


SET( CMAKE_CXX_FLAGS "-std=c++11" )#设定编译器

# 配置opencv
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# 配置pcl  #
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
#ADD_DEFINITIONS( ${PCL_DEFINITIONS} )
#LINK_LIBRARIES( ${PCL_LIBRARY_DIRS} )
#list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

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


ADD_EXECUTABLE( main main.cpp )



# 头文件
add_library(slamBase SHARED slamBase.cpp)
target_link_libraries(  slamBase
                        ${PCL_LIBRARIES}  # pcl
                        ${OpenCV_LIBS}    # opencv
                        g2o_core g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension # g2o
                        ${CSPARSE_LIBRARY} # csparse
                        ${Sophus_LIBRARY} # Sophus
                        ${Pangolin_LIBRARIES}) # Pangolin

# cpp文件
add_executable(generate_pointcloud generatePointCloud.cpp)     
target_link_libraries( generate_pointcloud slamBase
                        ${PCL_LIBRARIES}  # pcl
                        ${OpenCV_LIBS}    # opencv
                        g2o_core g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension # g2o
                        ${CSPARSE_LIBRARY} # csparse
                        ${Sophus_LIBRARY} # Sophus
                        ${Pangolin_LIBRARIES}) # Pangolin

add_executable(detectFeatures detecFeatures.cpp)     
target_link_libraries( detectFeatures slamBase
                        ${PCL_LIBRARIES}  # pcl
                        ${OpenCV_LIBS}    # opencv
                        g2o_core g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension # g2o
                        ${CSPARSE_LIBRARY} # csparse
                        ${Sophus_LIBRARY} # Sophus
                        ${Pangolin_LIBRARIES}) # Pangolin

add_executable(joinPointCloud joinPointCloud.cpp)     
target_link_libraries( joinPointCloud slamBase
                        ${PCL_LIBRARIES}  # pcl
                        ${OpenCV_LIBS}    # opencv
                        g2o_core g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension # g2o
                        ${CSPARSE_LIBRARY} # csparse
                        ${Sophus_LIBRARY} # Sophus
                        ${Pangolin_LIBRARIES}) # Pangolin

# 视觉里程计测量
add_executable(visualOdometry visualOdometry.cpp)     
target_link_libraries( visualOdometry slamBase
                        ${PCL_LIBRARIES}  # pcl
                        ${OpenCV_LIBS}    # opencv
                        g2o_core g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension # g2o
                        ${CSPARSE_LIBRARY} # csparse
                        ${Sophus_LIBRARY} # Sophus
                        ${Pangolin_LIBRARIES}) # Pangolin

add_executable(slamEnd slamEnd.cpp)     
target_link_libraries( slamEnd slamBase
                        ${PCL_LIBRARIES}  # pcl
                        ${OpenCV_LIBS}    # opencv
                        g2o_core g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension # g2o
                        ${CSPARSE_LIBRARY} # csparse
                        ${Sophus_LIBRARY} # Sophus
                        ${Pangolin_LIBRARIES}) # Pangolin


add_executable(slam slam.cpp )
target_link_libraries( slam slamBase
                        ${PCL_LIBRARIES}  # pcl
                        ${OpenCV_LIBS}    # opencv
                        g2o_core g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension # g2o
                        ${CSPARSE_LIBRARY} # csparse
                        ${Sophus_LIBRARY} # Sophus
                        ${Pangolin_LIBRARIES}) # Pangolin
