#ifndef UTILS_H
#define UTILS_H

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/pcl_macros.h>
#include <pcl/range_image/range_image.h>
#include <pcl/registration/icp.h>

#include <cmath>
#include <ctime>
#include <array>
#include <string>
#include <vector>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <thread>
#include <mutex>
#include <queue>
#include <assert.h>

#include <Eigen/Dense>

#include <boost/multi_array.hpp>

using namespace std;

struct PointPoseInfo
{
    double x;
    double y;
    double z;
    double qw;
    double qx;
    double qy;
    double qz;
    int idx;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointPoseInfo,
                                   (double, x, x) (double, y, y) (double, z, z)
                                   (double, qw, qw) (double, qx, qx) (double, qy, qy) (double, qz, qz)
                                   (int, idx, idx) (double, time, time)
)

// PCL point types
using pcl::PointXYZI;

// Matrix
typedef boost::multi_array<float, 2> Mat2f;
typedef boost::multi_array<int, 2> Mat2i;

/* TODO: customized point cloud data structure
template <typename T>
using alignedVector = std::vector<T, Eigen::aligned_allocator<T> >;

typedef Eigen::Vector4f point;
typedef alignedVector<point> PointCloud;
*/

// Configuration for Velodyne VLP-16
//extern const int N_SCAN = 16;
//extern const int Horizon_SCAN = 1800;
//extern const float ang_res_x = 0.2;
//extern const float ang_res_y = 2.0;
//extern const float ang_bottom = 15.0+0.1;
//extern const int groundScanIdx = 7;

// Configuration for Velodyne HDL-64E S2
extern const int N_SCAN = 64;
extern const int Horizon_SCAN = 2083; // points per laser per revolution
extern const float ang_res_x = 360.0 / Horizon_SCAN; // angular resolution (in deg)
extern const float ang_res_y = 26.8/float(N_SCAN-1); // vertical resolution (in deg) ~ 0.4
extern const float ang_res_y_lower = 0.5; // vertical resolution of the lower block
extern const float ang_res_y_upper = 0.3333; // vertical resolution of the upper block
extern const float ang_lower_max = 8.8333; // 26.5/3
extern const float ang_upper_min = 8.3333; // 25.0/3
extern const float ang_bottom = 24.3333; // 73.0/3
extern const int groundScanIdx = 45;

// Configuration for point cloud segmentation
extern const float sensorMountAngle = 0.0;
extern const float segmentTheta = 1.0472; // Seperating threshold for segmentation
extern const int segmentValidPointNum = 5;
extern const int segmentValidLineNum = 3;
extern const float segmentAlphaX = ang_res_x / 180.0 * M_PI; // angular resolution (in rad)
extern const float segmentAlphaY = ang_res_y / 180.0 * M_PI; // vertical resolution (in rad)

extern const float segmentAlphaYLower = ang_res_y_lower / 180.0 * M_PI; // vertical resolution (in rad)
extern const float segmentAlphaYUpper = ang_res_y_upper / 180.0 * M_PI; // vertical resolution (in rad)

extern const float scanPeriod = 0.1;

// Configuration for feature extraction
extern const float edgeThreshold = 0.1; // Be determining the edge feature point
extern const float surfThreshold = 0.1; // By determining the planar feature point
extern const float SqDistThreshold = 25.0; // By looking for nearest point in Corresponding

// Configuration for mapping
extern const double surroundSearchRadius = 50.0;
extern const int   surroundSearchNum = 50;

extern const double historySearchRadius = 7.0;
extern const int   historySearchNum = 25;
extern const float historyFitnessScore = 0.3;

extern const double globalMapSearchRadius = 500.0;

// Storing the smoothness of the local surface in point clouds
struct smoothness
{
    float value;
    size_t idx;
};

// Comparator for sorting the smoothness
struct smoothnessCompare
{
    bool operator()(smoothness const &left, smoothness const &right)
    {
        return left.value < right.value;
    }
};

void checkNanPts(const pcl::PointCloud<PointXYZI>::Ptr& cloudPtr)
{
  for (int i = 0; i < cloudPtr->points.size(); i++)
  {
    if (pcl_isnan(cloudPtr->points[i].x) ||
        pcl_isnan(cloudPtr->points[i].y) ||
        pcl_isnan(cloudPtr->points[i].z))
    {
      ROS_INFO_STREAM("Point " << i << " is a nan pt!!!");
      ROS_INFO_STREAM("The coordinate is ["
                      << cloudPtr->points[i].x << ","
                      << cloudPtr->points[i].y << ","
                      << cloudPtr->points[i].z << "]");
    }
  }
}

// Get parameters from yaml file
template <class class_name>
bool getParameter(const std::string& paramName, class_name& param)
{
  std::string nodeName = ros::this_node::getName();
  std::string paramKey;
  if (!ros::param::search(paramName, paramKey))
  {
    ROS_ERROR("%s: Failed to search for parameter '%s'.", nodeName.c_str(), paramName.c_str());
    return false;
  }

  if (!ros::param::has(paramKey))
  {
    ROS_ERROR("%s: Missing required parameter '%s'.", nodeName.c_str(), paramName.c_str());
    return false;
  }

  if (!ros::param::get(paramKey, param))
  {
    ROS_ERROR("%s: Failed to get parameter '%s'.", nodeName.c_str(), paramName.c_str());
    return false;
  }

  return true;
}

#endif // UTILS_H
