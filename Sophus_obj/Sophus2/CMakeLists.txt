cmake_minimum_required(VERSION 2.8)

project(Sophus2)


# 配置opencv
find_package(OpenCV REQUIRED)
#set(SOURCE_FILES main.cpp)
include_directories(${OpenCV_INCLUDE_DIRS})

# 配置pcl  #
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

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
FIND_PACKAGE(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

# 配置 Pangolin
find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})


add_executable(${PROJECT_NAME} "sophus2.cpp")     # 两两匹配点云
target_link_libraries (Sophus2 ${PCL_LIBRARIES})  # pcl
target_link_libraries (Sophus2  ${OpenCV_LIBS})    # opencv
target_link_libraries (Sophus2 g2o_core g2o_stuff g2o_types_sba g2o_csparse_extension ) # g2o
target_link_libraries (Sophus2 ${CSPARSE_LIBRARY}) # csparse
target_link_libraries (Sophus2 ${Sophus_LIBRARY}) # Sophus
target_link_libraries(Sophus2 ${Pangolin_LIBRARIES}) # Pangolin
target_compile_features(Sophus2 PUBLIC cxx_auto_type)


