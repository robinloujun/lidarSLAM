#ifndef LIDARMAPPING_H
#define LIDARMAPPING_H
#include <ros/ros.h>
//#include <map_msgs/SaveMap.h>
//#include <map_msgs/GetPointMap.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
//#include <sensor_msgs/point_cloud2_iterator.h>
#include <visualization_msgs/Marker.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <iostream>
#include <fstream>
#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>

#include <Eigen/Dense>

#include <boost/version.hpp>
#include <boost/thread.hpp>
#include <boost/thread/future.hpp>

// For nearest neighbor searching
#include <nabo/nabo.h>

// For point cloud container for ICP and ROS conversions
#include <pointmatcher/PointMatcher.h>
#include <pointmatcher/Timer.h>

#include <pointmatcher_ros/point_cloud.h>
#include <pointmatcher_ros/transform.h>
#include <pointmatcher_ros/get_params_from_server.h>

#include <lidar_directional_slam/utils/basic.h>
#include <lidar_directional_slam/utils/pose_graph.h>

using namespace std;

class LidarMapping
{

  typedef shared_ptr<LidarMapping> Ptr;

  IMPORT_TYPES(float)

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  LidarMapping(ros::NodeHandle& nh);
  ~LidarMapping();

  bool initialize();
  void lidarPointCloudHandler(
      const sensor_msgs::PointCloud2ConstPtr& lidarPointCloudIn);

protected:
  bool loadParameters();
  void processCloud(DPPtr cloud, const ros::Time& stamp, uint32_t seq);
  void processCloudSimple(DPPtr cloud, const ros::Time& stamp, uint32_t seq);
  void pointCloudPreprocessing(
      const sensor_msgs::PointCloud2ConstPtr& lidarPointCloudIn, DP& DP_current,
      struct DPPyramid& DP_pyramid);
  void setMap(DPPtr cloud);
  LidarMapping::DPPtr
  updateMap(DPPtr newCloud, const TransformationParameters T_sensor2map_updated,
            bool mapExists);
  LidarMapping::DPPtr
  updateMapSimple(DPPtr cloud,
                  const TransformationParameters T_sensor2map_updated,
                  bool mapExists);
  LidarMapping::DP getLocalMap();

  void updateICPMap(const DPPtr mapCloud);
  void clearMap();
  void publishLoop(double publishPeriod);
  void publishTransform();
  void publishPoints(const DP& pmCloud, const ros::Publisher& pub);

private:
  // ros elements
  ros::Subscriber subPointCloud_;
  ros::Subscriber subFilteredPointCloud_;

  ros::Publisher pubMap_;
  ros::Publisher pubOdom_;
  ros::Publisher pubOdomError_;
  ros::Publisher pubCloud_;
  ros::Publisher pubCloudLast_;
  ros::Publisher pubEstimatedPath_;
  ros::Publisher pubTruePath_;
  ros::Publisher pubMapInLocal_;
  ros::Publisher pubCloudInGlobal_;
  ros::Publisher pubPathEstimate_;
  ros::Publisher pubPathGroundTruth_;

  ros::Time publishStamp_;
  ros::Time mapCreationStamp_;
  ros::Time lastPointCloudStamp_;
  uint32_t lastPointCloudSeq_;

  DP cloudLast_;
  DPPtr PointCloudMap_;

  DataPointsFilters inputFilters_;
  DataPointsFilters preMapFilters_;
  DataPointsFilters postMapFilters_;
  shared_ptr<PM::DataPointsFilter> localMapFilter_;
  //  PM::DataPointsFilters localFilter_;
  ICP icpOdom_;
  ICPSequence icpMapping_;
  TransformationPtr transformation_;

  TransformationParameters T_odom2map_;
  TransformationParameters T_local2map_;
  TransformationParameters T_sensor2odom_;
  TransformationParameters T_frame2frame_;
  TransformationParameters T_sensor2map_;

  tf::TransformListener tfListener_;
  tf::TransformBroadcaster tfBroadcaster_;

  tf::StampedTransform tfCurrent_;
  tf::StampedTransform tfLast_;

  tf::StampedTransform tfGroundTruthCur_;
  tf::StampedTransform tfGroundTruthFirst_;

  nav_msgs::Path trajectoryEstimate_;
  nav_msgs::Path trajectoryGroundTruth_;

  boost::thread publishThread_;
  boost::mutex publishLock_;
  boost::mutex icpMapLock_;
  boost::mutex addNodeLock_;

  std::string nodeName_;
  std::string sensor_frame_id_;
  std::string world_frame_id_;
  std::string odometry_frame_id_;
  std::string ground_truth_frame_id_;
  std::string map_frame_id_;

  uint nodeIdx_;
  mapping_utils::poseGraph<float> graph_;

  // boolean state whether only LidarOdometry is exeucuted
  bool odometryOnly_;
  bool realtime_;
  bool initialized_;
  bool mapUpdated_;
  bool processingPointCloud_;
  bool processingMapBuilding_;
  bool tfExists_;

  bool testInit_ = false;
  bool registInit_;

  double tfRefreshPeriod_ = 0.01; // tf will be publish at the rate of the
                                  // incoming point cloud if set to zero
  double minOverlap_ = 0.2;
  double maxOverlap_ = 0.9;
  int minNumPts_ = 500;
  const float localMapRange_ = 120; // range of HDL-64E, unit in meter
  const float maxAngle_ = 0.02f;    // maximal angle by NN search
  const float priorDynamic_ =
      0.5f; // ratio. Prior to be dynamic when a new point is added
  const float priorStatic_ =
      0.5f; // ratio. Prior to be static when a new point is added
  const float eps_ = 0.0001f; // a small positive quantity
  const float eps_a_ = 0.05f; // ratio. Error proportional to the laser distance
  const float eps_d_ = 0.02f; // Fix error on the laser distance, unit in meter
  const float alpha_ = 0.99f; // ratio. Propability of staying static given that
                              // the point was dynamic
  const float beta_ = 0.99f; // ratio. Propability of staying dynamic given that
                             // the point was static
  const float maxDynamic_ =
      0.95f; // ratio. Threshold for which a point will stay dynamic
  const float maxDistNewPoint_ =
      0.2f; // distance at which a new point will be added in the global map
  const float maxNNSDist_ = 1.; // maximal distance by NN search
};

#endif // LIDARMAPPING_H
