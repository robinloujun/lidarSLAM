#ifndef LIDARODOMETRY_H
#define LIDARODOMETRY_H
#include <ros/ros.h>
#include <rosbag/bag.h>

#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Transform.h>
#include <visualization_msgs/Marker.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

//#include <geometry_utils/Transform3.h>
//#include <geometry_utils/GeometryUtilsROS.h>

#include <lidar_directional_slam/PoseGraph.h>

#include <iostream>
#include <fstream>
#include <math.h>
#include <time.h>
#include <ctime>
#include <stdio.h>
#include <stdlib.h>
#include <cassert>

#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>

#include <Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_representation.h>
#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>

#include <pcl/filters/random_sample.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <pcl/features/normal_3d.h>

#include <pcl/registration/gicp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include <pcl_ros/point_cloud.h>

#include <pointmatcher/PointMatcher.h>
#include <pointmatcher/Timer.h>
#include <pointmatcher_ros/point_cloud.h>
#include <pointmatcher_ros/transform.h>
#include <pointmatcher_ros/get_params_from_server.h>

#include <lidar_directional_slam/utils/geometry_utils.h>

using namespace std;

typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;
typedef PM::Parameters Parameters;

#define PI 3.14159265

class LidarOdometry
{

public:
  typedef shared_ptr<LidarOdometry> Ptr;

  // structure of the point cloud pyramid
  struct DPPyramid
  {
    DP pointCloudFirst;
    DP pointCloudSecond;
    DP pointCloudThird;
  };

  // structure of point cloud filter pyramid
  struct DataFilterPyramid
  {
    PM::DataPointsFilters inputFiltersFirst;
    PM::DataPointsFilters inputFiltersSecond;
    PM::DataPointsFilters inputFiltersThird;
  };

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  LidarOdometry(ros::NodeHandle& nh);
  ~LidarOdometry();

  bool initialize();
  void lidarPointCloudHandlerDP(
      const sensor_msgs::PointCloud2ConstPtr& lidarPointCloudIn);
  void lidarPointCloudHandlerPCL(
      const sensor_msgs::PointCloud2ConstPtr& lidarPointCloudIn);

private:
  bool loadParameters();
  bool loadICPConfig();
  bool RegisterCallbacks();

  void pointCloudPreprocessing(
      const sensor_msgs::PointCloud2ConstPtr& lidarPointCloudIn, DP& DP_current,
      struct DPPyramid& DP_pyramid);

  void pointCloudPreprocessing(
      const sensor_msgs::PointCloud2ConstPtr& lidarPointCloudIn,
      pcl::PointCloud<pcl::PointXYZ>::Ptr& pclOrigin,
      pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_Pyr_first,
      pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_Pyr_second,
      pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_Pyr_third);

  void applyGICP(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_reading,
                 const pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_refer,
                 const int& iteration, const Eigen::Matrix4f& initial_guess,
                 Eigen::Matrix4f& T_icp,
                 pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_regist);

  void publishPose(const Eigen::Matrix4f& T, const ros::Publisher& pub);
  void publishPoints(const DP& pmCloud, const ros::Publisher& pub);
  void publishPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pclPtr,
                     const ros::Publisher& pub);
  void publishPoseGraph();

  bool registInit_ = false;

  // Point Cloud of libpointmatcher
  // - original point cloud
  DP pointCloudDPLast_;
  DP pointCloudDPCur_;
  DP pointCloudTransfered_;

  // - filtered point clouds for ICP Pyramid
  DPPyramid pointCloudPyrCur_;
  DPPyramid pointCloudPyrLast_;
  unique_ptr<DPPyramid> pointCloudPyrCurPtr_;
  unique_ptr<DPPyramid> pointCloudPyrLastPtr_;

  // Filters for ICP Pyramid
  DataFilterPyramid filterPyr_;

  // For ICP Pyramid
  PM::ICP icpFirst_;
  PM::ICP icpSecond_;
  PM::ICP icpThird_;
  PM::ICP icp_;

  /*
  // Point Cloud of PCL
  pcl::PointCloud<pcl::PointXYZ>::Ptr pclCur_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pclLast_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pclLast1_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pclPyr1_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pclPyr2_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pclPyr3_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pclPyrLast1_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pclPyrLast2_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pclPyrLast3_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pclRegist_;
  */

  std::string nodeName_;

  // frame names
  std::string world_frame_id_;
  std::string odometry_frame_id_;
  std::string map_frame_id_;
  std::string sensor_frame_id_;
  std::string ground_truth_frame_id_;

  // ros elements
  // Subscriber for point cloud input
  ros::Subscriber subPointCloud_;

  // ros publishers
  ros::Publisher pubPointCloudCur_;
  ros::Publisher pubPointCloudLast_;
  ros::Publisher pubPointCloudRegist1_;
  ros::Publisher pubPointCloudRegist2_;
  ros::Publisher pubPointCloudRegist3_;
  ros::Publisher pubPointCloudTransfered_;
  ros::Publisher pubPathEstimate_;
  ros::Publisher pubPathGroundTruth_;
  ros::Publisher pubPoseGraph_;
  ros::Publisher pubGraphNode_;
  ros::Publisher pubGraphEdges_;
  ros::Publisher pubIncrementalEstimate_;
  ros::Publisher pubIntegratedEstimate_;

  ros::Time stamp_;
  // ros::Timer estimate_timer_;

  tf::TransformListener tfListener_;
  tf::TransformBroadcaster tfBroadcaster_;
  tf::StampedTransform tfGroundTruthCur_;
  tf::StampedTransform tfGroundTruthFirst_;

  nav_msgs::Path trajectoryEstimate_;
  nav_msgs::Path trajectoryGroundTruth_;

  /*
  // If use geometry_utils from BLAM
  geometry_utils::Transform3f incrementalEstimate_;
  geometry_utils::Transform3f integratedEstimate_;
  geometry_utils::Transform3f integratedGroundTruth_;
``*/

  // The last estimated transform set as the init transform for next iteration
  PM::TransformationParameters T_initial_guess_;
  PM::TransformationParameters T_world_;
  //  geometry_utils::transformationMatrix T_world_;

  // boolean state whether only LidarOdometry is exeucuted
  bool odometryOnly_;

  /* parameters for PCL
  // for PCL icp
  struct icpParamters
  {
    int iteration1;
    int iteration2;
    int iteration3;
  } icpParam_;

  // for PCL filters
  struct filterParameters
  {
    bool randomFilter;
    float percent1;
    float percent2;
    float percent3;
    bool voxelGridFilter;
    float leafSizeX1;
    float leafSizeY1;
    float leafSizeZ1;
    float leafSizeX2;
    float leafSizeY2;
    float leafSizeZ2;
    float leafSizeX3;
    float leafSizeY3;
    float leafSizeZ3;
    bool statisticalOutlierFilter;
    int meanK;
    int stdDevMulThresh;
    bool radiusOutlierFilter;
    int radius;
    int minNeighbors;
  } filterParam_;
  */
};

#endif // LIDARODOMETRY_H
