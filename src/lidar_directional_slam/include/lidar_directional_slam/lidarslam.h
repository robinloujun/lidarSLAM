#ifndef LIDARSLAM_H
#define LIDARSLAM_H

#include <ros/ros.h>
#include <lidar_directional_slam/lidarodometry.h>
#include <lidar_directional_slam/lidarmapping.h>
#include <lidar_directional_slam/utils/geometry_utils.h>

class LidarSLAM
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  LidarSLAM();
  ~LidarSLAM();

  bool initialize(const ros::NodeHandle& nh);
  bool process();

private:
  // ros handles
  ros::Subscriber subPointCloud_;
  ros::Publisher pubCurPointCloud_;

  std::string nodeName_;

  // Class objects
  LidarOdometry lidarOdometry_;
  LidarMapping lidarMapping_;
};

#endif // LIDARSLAM_H
