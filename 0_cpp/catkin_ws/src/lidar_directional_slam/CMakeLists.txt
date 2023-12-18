cmake_minimum_required(VERSION 2.8.3)
project(lidar_directional_slam)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  tf2
  std_msgs
  map_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf_conversions
  message_generation
  libpointmatcher_ros
)

find_package( libpointmatcher REQUIRED )
find_package( Boost REQUIRED COMPONENTS system )
find_package( Eigen3 REQUIRED )
find_package( GTSAM REQUIRED )

message(STATUS "Using libpointmatcher version ${libpointmatcher_VERSION}")
message(STATUS "Using catkin version ${catkin_VERSION}")
message(STATUS "Using Boost version ${Boost_VERSION}")
#message(STATUS "Using PCL version ${PCL_VERSION}")
#message(STATUS "Using VTK version ${VTK_VERSION}")

add_message_files(
  FILES
  Node.msg
  Edge.msg
  PoseGraph.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

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
  LIBRARIES
    ${PROJECT_NAME}
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    libpointmatcher_ros
    message_runtime
    std_msgs
    sensor_msgs
    geometry_msgs
  #LIBRARIES test
  #DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${libpointmatcher_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIRS}
#  ${PCL_INCLUDE_DIRS}
)

## Declare a C++ executable

#add_executable(LidarOdometry src/LidarOdometry.cpp)
#target_link_libraries(LidarOdometry ${catkin_LIBRARIES})

#add_executable(LidarMapping src/LidarMapping.cpp)
#target_link_libraries(LidarMapping ${catkin_LIBRARIES} gtsam)

#add_executable(LidarSLAM src/LidarSLAM.cpp)
#target_link_libraries(LidarSLAM ${catkin_LIBRARIES})

#add_executable(LidarTest src/LidarTest.cpp)
#target_link_libraries(LidarTest ${catkin_LIBRARIES})

#For Debug
set(CMAKE_BUILD_TYPE "Debug")
