cmake_minimum_required(VERSION 2.8.3)
project(lidar_loam)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  std_msgs
  map_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf_conversions
  message_generation
  pcl_ros
  pcl_conversions
)
find_package(Eigen3 REQUIRED QUIET)
find_package(PCL REQUIRED QUIET)
find_package(Ceres REQUIRED QUIET)
find_package(GTSAM REQUIRED QUIET)
find_package(libroptlib REQUIRED QUIET) # When using installed library

message(STATUS "Using catkin version ${catkin_VERSION}")
message(STATUS "Using PCL version ${PCL_VERSION}")
message(STATUS "Using Ceres version ${CERES_VERSION}")

#add_subdirectory(${PROJECT_SOURCE_DIR}/RieOpt) # When using local library

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    # LIBRARIES ${PROJECT_NAME}
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp message_runtime std_msgs nav_msgs geometry_msgs
    DEPENDS PCL
)

###########
## Build ##
###########

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
    ${GTSAM_INCLUDE_DIR}
    ${LIBROPTLIB_INCLUDE_DIRS}
)

link_directories(
    include
    ${PCL_LIBRARY_DIRS}
    ${GTSAM_LIBRARY_DIRS}
)

## Declare a C++ executable
add_executable(LidarPreprocessing src/LidarPreprocessing.cpp)
target_link_libraries(LidarPreprocessing ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(LidarOdometry src/LidarOdometry.cpp src/MinDistProblem.cpp)
target_link_libraries(LidarOdometry ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES} roptlib lapack blas)

add_executable(LidarMapping src/LidarMapping.cpp src/MinDistProblem.cpp)
target_link_libraries(LidarMapping ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES} gtsam roptlib lapack blas)

#For Debug
set(CMAKE_BUILD_TYPE "Debug")
