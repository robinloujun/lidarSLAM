cmake_minimum_required (VERSION 2.8)
project(lidarDirectionalSLAM)

add_compile_options(-std=c++11)

find_package( libpointmatcher REQUIRED )
find_package( PCL 1.8 REQUIRED )
find_package( catkin REQUIRED COMPONENTS roscpp )
find_package( OpenCV REQUIRED )
find_package( Qt5Widgets REQUIRED QUIET )

include_directories(
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_SOURCE_DIR}
    "/opt/ros/kinetic/include/"
    ${PCL_INCLUDE_DIRS}
    )

message(STATUS "Using libpointmatcher version ${libpointmatcher_VERSION}")
message(STATUS "Using PCL version ${PCL_VERSION}")
message(STATUS "Using catkin version ${catkin_VERSION}")
message(STATUS "Using Boost version ${Boost_VERSION}")
message(STATUS "Using VTK version ${VTK_VERSION}")


set(CPP_FILES
    datasetconfig.cpp
)
set(WRAP_CPP_FILES datasetconfig.h)
set(WRAP_UI_FILES pclvisualization.ui)

add_executable(lidarDirectionalSLAM main.cpp)
target_link_libraries(lidarDirectionalSLAM
    ${libpointmatcher_LIBRARIES}
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    )

add_executable(kitti2pcd kitti_pcd.cpp)
target_link_libraries(kitti2pcd
    ${libpointmatcher_LIBRARIES}
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
    )

add_executable(KittiVisualizer pcl_visual.cpp)
target_link_libraries( KittiVisualizer ${PCL_LIBRARIES} ${OpenCV_LIBS} )

add_executable(registration icp_checker.cpp)
target_link_libraries(registration ${libpointmatcher_LIBRARIES} ${PCL_LIBRARIES} ${catkin_LIBRARIES})
