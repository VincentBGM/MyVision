cmake_minimum_required(VERSION 2.8)

project(VO)

SET( CMAKE_CXX_FLAGS "-std=c++11" )#设定编译器

# 配置opencv
find_package(OpenCV REQUIRED)
#set(SOURCE_FILES main.cpp)
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
LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
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




add_executable(feature_extraction feature_extraction.cpp)     
target_link_libraries( feature_extraction 
                        #${PCL_LIBRARIES}  # pcl
                        ${OpenCV_LIBS}    # opencv
                        #g2o_core g2o_stuff g2o_types_sba g2o_csparse_extension # g2o
                        ${CSPARSE_LIBRARY} # csparse
                        #${Sophus_LIBRARY} # Sophus
                        ${Pangolin_LIBRARIES}) # Pangolin

add_executable(pose_2D_2D pose_2D_2D.cpp)     
target_link_libraries( pose_2D_2D 
                        #${PCL_LIBRARIES}  # pcl
                        ${OpenCV_LIBS}    # opencv
                        g2o_core g2o_stuff g2o_types_sba g2o_csparse_extension # g2o
                        ${CSPARSE_LIBRARY} # csparse
                        ${Sophus_LIBRARY} # Sophus
                        ${Pangolin_LIBRARIES}) # Pangolin

add_executable(triangulation triangulation.cpp)     
target_link_libraries( triangulation 
                        #${PCL_LIBRARIES}  # pcl
                        ${OpenCV_LIBS}    # opencv
                        g2o_core g2o_stuff g2o_types_sba g2o_csparse_extension # g2o
                        ${CSPARSE_LIBRARY} # csparse
                        ${Sophus_LIBRARY} # Sophus
                        ${Pangolin_LIBRARIES}) # Pangolin

add_executable(pose_3D_2D pose_3D_2D.cpp)     
target_link_libraries( pose_3D_2D 
                        #${PCL_LIBRARIES}  # pcl
                        ${OpenCV_LIBS}    # opencv
                        g2o_core g2o_stuff g2o_types_sba g2o_csparse_extension # g2o
                        ${CSPARSE_LIBRARY} # csparse
                        ${Sophus_LIBRARY} # Sophus
                        ${Pangolin_LIBRARIES}) # Pangolin

add_executable(pose_3D_3D pose_3D_3D.cpp)     
target_link_libraries( pose_3D_3D 
                        #${PCL_LIBRARIES}  # pcl
                        ${OpenCV_LIBS}    # opencv
                        g2o_core g2o_stuff g2o_types_sba g2o_csparse_extension # g2o
                        ${CSPARSE_LIBRARY} # csparse
                        ${Sophus_LIBRARY} # Sophus
                        ${Pangolin_LIBRARIES}) # Pangolin


