#include "lidar_loam/utils.h"
#include "lidar_loam/timer.h"
#include "lidar_loam/lidarfactor.h"
#include "lidar_loam/MinDistProblem.h"

// Linesearch based solvers
#include "roptlib/Solvers/SolversLS.h"
#include "roptlib/Solvers/RSD.h"
#include "roptlib/Solvers/RNewton.h"
#include "roptlib/Solvers/RCG.h"
#include "roptlib/Solvers/RBroydenFamily.h"
#include "roptlib/Solvers/RWRBFGS.h"
#include "roptlib/Solvers/RBFGS.h"
#include "roptlib/Solvers/LRBFGS.h"

// Trust-region based solvers
#include "roptlib/Solvers/SolversTR.h"
#include "roptlib/Solvers/RTRSD.h"
#include "roptlib/Solvers/RTRNewton.h"
#include "roptlib/Solvers/RTRSR1.h"
#include "roptlib/Solvers/LRTRSR1.h"

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/sam/BearingRangeFactor.h>
//#include <gtsam/slam/expressions.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>

class LidarMapping
{
private:
  ros::NodeHandle nh;

  ros::Subscriber subCornerLast;
  ros::Subscriber subSurfLast;
  ros::Subscriber subOutlierLast;
  ros::Subscriber subOdom;
  ros::Subscriber subSegmentedCloud;
  ros::Subscriber subCornerLocal;
  ros::Subscriber subSurfLocal;
  ros::Subscriber subCorner;
  ros::Subscriber subSurf;

  ros::Publisher pubSurround;
  ros::Publisher pubMap;
  ros::Publisher pubOdom;
  ros::Publisher pubOdomHF;
  ros::Publisher pubPath;
  ros::Publisher pubPose;
  ros::Publisher pubCorner;
  ros::Publisher pubSurf;
  ros::Publisher pubCornerMap;
  ros::Publisher pubSurfMap;
  ros::Publisher pubHistory;

  nav_msgs::Odometry odomMapping;
  nav_msgs::Path pathMapping;

  tf::TransformBroadcaster broadcaster;
  tf::StampedTransform mapTransform;

  bool newCornerLast;
  bool newSurfLast;
  bool newOutlierLast;
  bool newOdom;

  double timeNewCornerLast;
  double timeNewSurfLast;
  double timeNewOutlierLast;
  double timeNewOdometry;
  double timeLastProcessing;

  pcl::PointCloud<PointXYZI>::Ptr cornerLast;
  pcl::PointCloud<PointXYZI>::Ptr surfLast;
  pcl::PointCloud<PointXYZI>::Ptr outlierLast;
  pcl::PointCloud<PointXYZI>::Ptr cornerLocalLast;
  pcl::PointCloud<PointXYZI>::Ptr surfLocalLast;

  pcl::PointCloud<PointXYZI>::Ptr cornerCurrent;
  pcl::PointCloud<PointXYZI>::Ptr surfCurrent;
  pcl::PointCloud<PointXYZI>::Ptr surfOutlierLast;

  pcl::PointCloud<PointXYZI>::Ptr cornerLastDS;
  pcl::PointCloud<PointXYZI>::Ptr surfLastDS;
  pcl::PointCloud<PointXYZI>::Ptr outlierLastDS;
  pcl::PointCloud<PointXYZI>::Ptr surfOutlierLastDS;

  pcl::PointCloud<PointXYZI>::Ptr cornerFromMap;
  pcl::PointCloud<PointXYZI>::Ptr surfFromMap;
  pcl::PointCloud<PointXYZI>::Ptr cornerFromMapDS;
  pcl::PointCloud<PointXYZI>::Ptr surfFromMapDS;

  pcl::PointCloud<PointXYZI>::Ptr cornerCurrentPts;
  pcl::PointCloud<PointXYZI>::Ptr cornerMatchJ;
  pcl::PointCloud<PointXYZI>::Ptr cornerMatchL;

  pcl::PointCloud<PointXYZI>::Ptr surfCurrentPts;
  pcl::PointCloud<PointXYZI>::Ptr surfCenter;
  pcl::PointCloud<PointXYZI>::Ptr surfNormal;

  pcl::PointCloud<PointXYZI>::Ptr localMap;
  pcl::PointCloud<PointXYZI>::Ptr localKeyPoses;
  pcl::PointCloud<PointXYZI>::Ptr localKeyPosesDS;

  pcl::PointCloud<PointXYZI>::Ptr surroundMap;
  pcl::PointCloud<PointXYZI>::Ptr surroundKeyPoses;
  pcl::PointCloud<PointXYZI>::Ptr surroundKeyPosesDS;

  pcl::PointCloud<PointXYZI>::Ptr latestKeyFrames;
  pcl::PointCloud<PointXYZI>::Ptr historyKeyFrames;
  pcl::PointCloud<PointXYZI>::Ptr historyKeyFramesDS;

  pcl::PointCloud<PointXYZI>::Ptr poseCloud; // each point as one pose
  // Usage for PointPoseInfo
  // position: x, y, z
  // orientation: qw - w, qx - x, qy - y, qz - z
  // other information: intensity - idx, time - time
  pcl::PointCloud<PointPoseInfo>::Ptr poseInfoCloud;

  pcl::PointCloud<PointXYZI>::Ptr poseLocalCloud;
  pcl::PointCloud<PointPoseInfo>::Ptr poseInterpolated; // Keyframes + interpolated poses

  PointXYZI currentPose, lastPose, firstPoseInLocalGraph;
  PointXYZI ptInLocal, ptInMap;

  pcl::PointCloud<PointXYZI>::Ptr globalPoses;
  pcl::PointCloud<PointXYZI>::Ptr globalPosesDS;
  pcl::PointCloud<PointXYZI>::Ptr globalMap;
  pcl::PointCloud<PointXYZI>::Ptr globalMapDS;

  std::vector<pcl::PointCloud<PointXYZI>::Ptr> cornerKeyFrames;
  std::vector<pcl::PointCloud<PointXYZI>::Ptr> surfKeyFrames;
  std::vector<pcl::PointCloud<PointXYZI>::Ptr> outlierKeyFrames;

  std::deque<pcl::PointCloud<PointXYZI>::Ptr> localCornerKeyFrames;
  std::deque<pcl::PointCloud<PointXYZI>::Ptr> localSurfKeyFrames;
  std::deque<pcl::PointCloud<PointXYZI>::Ptr> localOutlierKeyFrames;
  int firstLocalIdx;

  std::deque<pcl::PointCloud<PointXYZI>::Ptr> recentCornerKeyFrames;
  std::deque<pcl::PointCloud<PointXYZI>::Ptr> recentSurfKeyFrames;
  std::deque<pcl::PointCloud<PointXYZI>::Ptr> recentOutlierKeyFrames;
  int latestFrameIdx;

  std::vector<int> surroundExistingKeyPosesIdx;
  std::deque<pcl::PointCloud<PointXYZI>::Ptr> surroundCornerKeyFrames;
  std::deque<pcl::PointCloud<PointXYZI>::Ptr> surroundSurfKeyFrames;
  std::deque<pcl::PointCloud<PointXYZI>::Ptr> surroundOutlierKeyFrames;

  pcl::KdTreeFLANN<PointXYZI>::Ptr kdTreeCornerFromMap;
  pcl::KdTreeFLANN<PointXYZI>::Ptr kdTreeSurfFromMap;
  pcl::KdTreeFLANN<PointXYZI>::Ptr kdTreeSurroundKeyPoses;
  pcl::KdTreeFLANN<PointXYZI>::Ptr kdTreeHistoryKeyPoses;
  pcl::KdTreeFLANN<PointXYZI>::Ptr kdTreeGlobalMap;

  std::vector<int> pointSearchIdx;
  std::vector<float> pointSearchSqDists;

  pcl::VoxelGrid<PointXYZI> downSizeFilterCorner;
  pcl::VoxelGrid<PointXYZI> downSizeFilterSurf;
  pcl::VoxelGrid<PointXYZI> downSizeFilterOutlier;
  pcl::VoxelGrid<PointXYZI> downSizeFilterSurroundPoses;
  pcl::VoxelGrid<PointXYZI> downSizeFilterHistoryFrames;
  pcl::VoxelGrid<PointXYZI> downSizeFilterGlobalPoses;
  pcl::VoxelGrid<PointXYZI> downSizeFilterGlobalFrames;

  int NumPtsCornerLast;
  int NumPtsSurfLast;
  int NumPtsOutlierLast;
  int NumPtsCornerLastDS;
  int NumPtsSurfLastDS;
  int NumPtsOutlierLastDS;
  int NumPtsSurfOutlierLastDS;
  int NumPtsCornerFromMap;
  int NumPtsSurfFromMap;
  int NumPtsCornerFromMapDS;
  int NumPtsSurfFromMapDS;  
  int cornerResCount;
  int surfResCount;

  int laserCloudCenWidth = 10;
  int laserCloudCenHeight = 10;
  int laserCloudCenDepth = 5;

  // Form of the transformation
  double transformOdom[7]; // transform of odom in current frame
  double transformIncre[7]; // transform between odom and map in each iteration
  double transformMap[7]; // transform of map
  double transformLast[7]; // transform of map in last frame

  std::mutex mutualExclusion;

  int numIter = 0;
  int numOpt = 0;

  // Boolean for functions
  bool printTimeOn;
  bool loopClosureOn;
  bool slidingWindowOn;

  gtsam::NonlinearFactorGraph globalGraph;
  gtsam::Values globalInitialEstimate;
  gtsam::ISAM2 *isam;
  gtsam::Values globalEstimated;

  gtsam::NonlinearFactorGraph localGraph;
  gtsam::Values localInitialEstimate;
  gtsam::ISAM2 *localISAM;
  gtsam::Values localEstimated;

  gtsam::noiseModel::Diagonal::shared_ptr priorNoise;
  gtsam::noiseModel::Diagonal::shared_ptr odometryNoise;
  gtsam::noiseModel::Diagonal::shared_ptr constraintNoise;
  gtsam::noiseModel::Diagonal::shared_ptr measurementNoise;

  // Loop closure detection related
  bool loopToClose;
  double timeSaveLoopClosure;
  int closestHistoryIdx;
  int latestFrameIdxLoop;
  bool loopClosed;

public:
  LidarMapping():
    nh("~")
  {
    gtsam::ISAM2Params isamPara;
    isamPara.relinearizeThreshold = 0.01;
    isamPara.relinearizeSkip = 1;
    isam = new gtsam::ISAM2(isamPara);

    subCornerLast = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_corner_last", 2, &LidarMapping::cornerLastHandler, this);
    subSurfLast = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_surf_last", 2, &LidarMapping::surfaceLastHandler, this);
    subOutlierLast = nh.subscribe<sensor_msgs::PointCloud2>("/outlier_point_cloud", 2, &LidarMapping::outlierLastHandler, this);
    subOdom = nh.subscribe<nav_msgs::Odometry>("/odom", 5, &LidarMapping::odomHandler, this);
    subCornerLocal = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 2, &LidarMapping::cornerLocalHandler, this);
    subSurfLocal = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_flat", 2, &LidarMapping::surfLocalHandler, this);
    subCorner = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 2, &LidarMapping::cornerHandler, this);
    subSurf = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 2, &LidarMapping::surfHandler, this);

    pubSurround = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surround", 2);
    pubMap = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_map", 2);
    pubOdom = nh.advertise<nav_msgs::Odometry>("/odom_mapped", 2);
    pubOdomHF = nh.advertise<nav_msgs::Odometry>("odom_mapped_high_freq", 2);
    pubPath = nh.advertise<nav_msgs::Path>("/path_mapped", 2);
    pubPose = nh.advertise<sensor_msgs::PointCloud2>("/trajectory", 2);
    pubCorner = nh.advertise<sensor_msgs::PointCloud2>("/map_corner_less_sharp", 2);
    pubSurf = nh.advertise<sensor_msgs::PointCloud2>("/map_surf_less_flat", 2);
    pubCornerMap = nh.advertise<sensor_msgs::PointCloud2>("/corner_from_map", 2);
    pubSurfMap = nh.advertise<sensor_msgs::PointCloud2>("/surf_from_map", 2);
    pubHistory = nh.advertise<sensor_msgs::PointCloud2>("/history_point_cloud", 2);

    allocateMemory();
    initializeParameters();
  }

  ~LidarMapping() {}

  void allocateMemory()
  {
    surroundMap.reset(new pcl::PointCloud<PointXYZI>());
    cornerLocalLast.reset(new pcl::PointCloud<PointXYZI>());
    surfLocalLast.reset(new pcl::PointCloud<PointXYZI>());
    cornerLast.reset(new pcl::PointCloud<PointXYZI>());
    surfLast.reset(new pcl::PointCloud<PointXYZI>());
    cornerCurrent.reset(new pcl::PointCloud<PointXYZI>());
    surfCurrent.reset(new pcl::PointCloud<PointXYZI>());
    outlierLast.reset(new pcl::PointCloud<PointXYZI>());
    surfOutlierLast.reset(new pcl::PointCloud<PointXYZI>());
    cornerFromMap.reset(new pcl::PointCloud<PointXYZI>());
    surfFromMap.reset(new pcl::PointCloud<PointXYZI>());
    cornerLastDS.reset(new pcl::PointCloud<PointXYZI>());
    surfLastDS.reset(new pcl::PointCloud<PointXYZI>());
    outlierLastDS.reset(new pcl::PointCloud<PointXYZI>());
    surfOutlierLastDS.reset(new pcl::PointCloud<PointXYZI>());
    cornerFromMapDS.reset(new pcl::PointCloud<PointXYZI>());
    surfFromMapDS.reset(new pcl::PointCloud<PointXYZI>());

    cornerCurrentPts.reset(new pcl::PointCloud<PointXYZI>());
    cornerMatchJ.reset(new pcl::PointCloud<PointXYZI>());
    cornerMatchL.reset(new pcl::PointCloud<PointXYZI>());
    surfCurrentPts.reset(new pcl::PointCloud<PointXYZI>());
    surfCenter.reset(new pcl::PointCloud<PointXYZI>());
    surfNormal.reset(new pcl::PointCloud<PointXYZI>());

    poseCloud.reset(new pcl::PointCloud<PointXYZI>());
    poseInfoCloud.reset(new pcl::PointCloud<PointPoseInfo>());

    poseLocalCloud.reset(new pcl::PointCloud<PointXYZI>());
    poseInterpolated.reset(new pcl::PointCloud<PointPoseInfo>());

    globalPoses.reset(new pcl::PointCloud<PointXYZI>());
    globalPosesDS.reset(new pcl::PointCloud<PointXYZI>());
    globalMap.reset(new pcl::PointCloud<PointXYZI>());
    globalMapDS.reset(new pcl::PointCloud<PointXYZI>());

    surroundKeyPoses.reset(new pcl::PointCloud<PointXYZI>());
    surroundKeyPosesDS.reset(new pcl::PointCloud<PointXYZI>());

    latestKeyFrames.reset(new pcl::PointCloud<PointXYZI>());
    historyKeyFrames.reset(new pcl::PointCloud<PointXYZI>());
    historyKeyFramesDS.reset(new pcl::PointCloud<PointXYZI>());

    kdTreeCornerFromMap.reset(new pcl::KdTreeFLANN<PointXYZI>());
    kdTreeSurfFromMap.reset(new pcl::KdTreeFLANN<PointXYZI>());
    kdTreeSurroundKeyPoses.reset(new pcl::KdTreeFLANN<PointXYZI>());
    kdTreeHistoryKeyPoses.reset(new pcl::KdTreeFLANN<PointXYZI>());
    kdTreeGlobalMap.reset(new pcl::KdTreeFLANN<PointXYZI>());
  }

  void initializeParameters()
  {
    newCornerLast = false;
    newSurfLast = false;
    newOutlierLast = false;
    newOdom = false;

    timeNewCornerLast = 0;
    timeNewSurfLast = 0;
    timeNewOutlierLast = 0;
    timeNewOdometry = 0;
    timeLastProcessing = -1;

    transformOdom[0] = 1;
    transformIncre[0] = 1;
    transformMap[0] = 1;
    transformLast[0] = 1;

    latestFrameIdx = 0;

    for (int i = 1; i < 7; ++i)
    {
      transformOdom[i] = 0;
      transformIncre[i] = 0;
      transformMap[i] = 0;
      transformLast[i] = 0;
    }

    downSizeFilterCorner.setLeafSize(0.2, 0.2, 0.2);
    downSizeFilterSurf.setLeafSize(0.4, 0.4, 0.4);
    downSizeFilterOutlier.setLeafSize(0.4, 0.4, 0.4);
    downSizeFilterGlobalPoses.setLeafSize(1.0, 1.0, 1.0);
    downSizeFilterGlobalFrames.setLeafSize(0.4, 0.4, 0.4);
    downSizeFilterSurroundPoses.setLeafSize(1.0, 1.0, 1.0);
    downSizeFilterHistoryFrames.setLeafSize(0.4, 0.4, 0.4);

    odomMapping.header.frame_id = "/lidar_init";
    odomMapping.child_frame_id = "/lidar_map";

    mapTransform.frame_id_ = "/lidar_init";
    mapTransform.child_frame_id_ = "/lidar_map";

    gtsam::Vector vector6(6);
    vector6 << 1e-6, 1e-6, 1e-6, 1e-8, 1e-8, 1e-6;
    priorNoise = gtsam::noiseModel::Diagonal::Variances(vector6);
    odometryNoise = gtsam::noiseModel::Diagonal::Variances(vector6);
    measurementNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(3)<<0.01,0.03,0.05).finished());

    loopToClose = false;
    loopClosed = false;

    firstLocalIdx = 0;

    // Load parameters from yaml
    if (!getParameter("/mapping/max_num_iter", numIter))
    {
      ROS_WARN("maximal iteration number of mapping optimization not set, use default value: 50");
      numIter = 50;
    }

    if (!getParameter("/mapping/iter_count", numOpt))
    {
      ROS_WARN("number of mapping optimization not set, use default value: 1");
      numOpt = 1;
    }

    if (!getParameter("/mapping/print_time", printTimeOn))
    {
      ROS_WARN("Time print set to false");
      printTimeOn = false;
    }

    if (!getParameter("/mapping/loop_closure", loopClosureOn))
    {
      ROS_WARN("loop closure detection set to false");
      loopClosureOn = false;
    }

    if (!getParameter("/mapping/loop_closure", loopClosureOn))
    {
      ROS_WARN("loop closure detection set to false");
      loopClosureOn = false;
    }
  }

  void cornerLocalHandler(const sensor_msgs::PointCloud2ConstPtr& pointCloudIn)
  {
    cornerLocalLast->clear();
    pcl::fromROSMsg(*pointCloudIn, *cornerLocalLast);
    ROS_INFO_STREAM("corner size: " << cornerLocalLast->points.size());
  }

  void surfLocalHandler(const sensor_msgs::PointCloud2ConstPtr& pointCloudIn)
  {
    surfLocalLast->clear();
    pcl::fromROSMsg(*pointCloudIn, *surfLocalLast);
    ROS_INFO_STREAM("surf size: " << surfLocalLast->points.size());
  }

  void cornerLastHandler(const sensor_msgs::PointCloud2ConstPtr& pointCloudIn)
  {
    cornerLast->clear();
    timeNewCornerLast = pointCloudIn->header.stamp.toSec();
    pcl::fromROSMsg(*pointCloudIn, *cornerLast);
    newCornerLast = true;
  }

  void surfaceLastHandler(const sensor_msgs::PointCloud2ConstPtr& pointCloudIn)
  {
    surfLast->clear();
    timeNewSurfLast = pointCloudIn->header.stamp.toSec();
    pcl::fromROSMsg(*pointCloudIn, *surfLast);
    newSurfLast = true;
  }

  void outlierLastHandler(const sensor_msgs::PointCloud2ConstPtr& pointCloudIn)
  {
    outlierLast->clear();
    timeNewOutlierLast = pointCloudIn->header.stamp.toSec();
    pcl::fromROSMsg(*pointCloudIn, *outlierLast);
    newOutlierLast = true;
  }

  void odomHandler(const nav_msgs::Odometry::ConstPtr& odomIn)
  {
    timeNewOdometry = odomIn->header.stamp.toSec();

    // Get the transformation from the last odom frame to world origin
    transformOdom[0] = odomIn->pose.pose.orientation.w;
    transformOdom[1] = odomIn->pose.pose.orientation.x;
    transformOdom[2] = odomIn->pose.pose.orientation.y;
    transformOdom[3] = odomIn->pose.pose.orientation.z;
    transformOdom[4] = odomIn->pose.pose.position.x;
    transformOdom[5] = odomIn->pose.pose.position.y;
    transformOdom[6] = odomIn->pose.pose.position.z;

//    nav_msgs::Odometry odomHighFreq;
//    odomHighFreq.header.frame_id = "/lidar_init";
//    odomHighFreq.child_frame_id = "/lidar_map";
//    odomHighFreq.header.stamp = odomIn->header.stamp;
//    odomHighFreq.pose.pose.orientation.w = transformMap[0];
//    odomHighFreq.pose.pose.orientation.x = transformMap[1];
//    odomHighFreq.pose.pose.orientation.y = transformMap[2];
//    odomHighFreq.pose.pose.orientation.z = transformMap[3];
//    odomHighFreq.pose.pose.position.x = transformMap[4];
//    odomHighFreq.pose.pose.position.y = transformMap[5];
//    odomHighFreq.pose.pose.position.z = transformMap[6];
//    pubOdomHF.publish(odomHighFreq);

    newOdom = true;
  }

  void cornerHandler(const sensor_msgs::PointCloud2ConstPtr& pointCloudIn)
  {
    cornerCurrent->clear();
    pcl::fromROSMsg(*pointCloudIn, *cornerCurrent);
  }

  void surfHandler(const sensor_msgs::PointCloud2ConstPtr& pointCloudIn)
  {
    surfCurrent->clear();
    pcl::fromROSMsg(*pointCloudIn, *surfCurrent);
  }

  void transformPoint(PointXYZI const *const pi, PointXYZI *const po)
  {
    Eigen::Quaterniond quaternion(transformMap[0],
                                  transformMap[1],
                                  transformMap[2],
                                  transformMap[3]);
    Eigen::Vector3d transition(transformMap[4],
                               transformMap[5],
                               transformMap[6]);

    Eigen::Vector3d ptIn(pi->x, pi->y, pi->z);
    Eigen::Vector3d ptOut = quaternion * ptIn + transition;

    po->x = ptOut.x();
    po->y = ptOut.y();
    po->z = ptOut.z();
    po->intensity = pi->intensity;
  }

  pcl::PointCloud<PointXYZI>::Ptr transformCloud(const pcl::PointCloud<PointXYZI>::Ptr &cloudIn)
  {
    pcl::PointCloud<PointXYZI>::Ptr cloudOut(new pcl::PointCloud<PointXYZI>());

    int numPts = cloudIn->points.size();
    cloudOut->resize(numPts);

    for (int i = 0; i < numPts; ++i)
    {
      PointXYZI ptIn = cloudIn->points[i];
      PointXYZI ptOut;
      transformPoint(&ptIn, &ptOut);
      cloudOut->points[i] = ptOut;
    }

    return cloudOut;
  }

  pcl::PointCloud<PointXYZI>::Ptr transformCloud(const pcl::PointCloud<PointXYZI>::Ptr &cloudIn, PointPoseInfo * PointInfoIn)
  {
    pcl::PointCloud<PointXYZI>::Ptr cloudOut(new pcl::PointCloud<PointXYZI>());

    Eigen::Quaterniond quaternion(PointInfoIn->qw,
                                  PointInfoIn->qx,
                                  PointInfoIn->qy,
                                  PointInfoIn->qz);
    Eigen::Vector3d transition(PointInfoIn->x,
                               PointInfoIn->y,
                               PointInfoIn->z);

    int numPts = cloudIn->points.size();
    cloudOut->resize(numPts);

    for (int i = 0; i < numPts; ++i)
    {
      Eigen::Vector3d ptIn(cloudIn->points[i].x, cloudIn->points[i].y, cloudIn->points[i].z);
      Eigen::Vector3d ptOut = quaternion * ptIn + transition;

      PointXYZI pt;
      pt.x = ptOut.x();
      pt.y = ptOut.y();
      pt.z = ptOut.z();
      pt.intensity = cloudIn->points[i].intensity;

      cloudOut->points[i] = pt;
    }

    return cloudOut;
  }

  void setInitialTransform()
  {
    Eigen::Quaterniond quaternion_odom(transformOdom[0],
                                       transformOdom[1],
                                       transformOdom[2],
                                       transformOdom[3]);
    Eigen::Vector3d transition_odom(transformOdom[4],
                                    transformOdom[5],
                                    transformOdom[6]);

    Eigen::Quaterniond quaternion_incremental(transformIncre[0],
                                              transformIncre[1],
                                              transformIncre[2],
                                              transformIncre[3]);
    Eigen::Vector3d transition_incremental(transformIncre[4],
                                           transformIncre[5],
                                           transformIncre[6]);

    // Set the initial guess for frame-to-model optimization
    // based on the odometry transform of current frame and the odom to map transform of last frame
    Eigen::Quaterniond quaternion_map = quaternion_incremental * quaternion_odom;
    Eigen::Vector3d transition_map = quaternion_incremental * transition_odom + transition_incremental;

    transformMap[0] = quaternion_map.w();
    transformMap[1] = quaternion_map.x();
    transformMap[2] = quaternion_map.y();
    transformMap[3] = quaternion_map.z();
    transformMap[4] = transition_map.x();
    transformMap[5] = transition_map.y();
    transformMap[6] = transition_map.z();
  }

  void computeIncremental()
  {
    Eigen::Quaterniond quaternion_odom(transformOdom[0],
                                       transformOdom[1],
                                       transformOdom[2],
                                       transformOdom[3]);
    Eigen::Vector3d transition_odom(transformOdom[4],
                                    transformOdom[5],
                                    transformOdom[6]);

    Eigen::Quaterniond quaternion_map(transformMap[0],
                                      transformMap[1],
                                      transformMap[2],
                                      transformMap[3]);
    Eigen::Vector3d transition_map(transformMap[4],
                                   transformMap[5],
                                   transformMap[6]);

    // Compute the odom to map transform of last frame which would be used for the next iteration
    Eigen::Quaterniond quaternion_incremental = quaternion_map * quaternion_odom.inverse();
    Eigen::Vector3d transition_incremental = transition_map - quaternion_incremental * transition_odom;

    transformIncre[0] = quaternion_incremental.w();
    transformIncre[1] = quaternion_incremental.x();
    transformIncre[2] = quaternion_incremental.y();
    transformIncre[3] = quaternion_incremental.z();
    transformIncre[4] = transition_incremental.x();
    transformIncre[5] = transition_incremental.y();
    transformIncre[6] = transition_incremental.z();
  }

  void correctIncremental()
  {
    int currentIdx = globalEstimated.size() - 1;

    Eigen::Quaterniond quaternion_odom(transformOdom[0],
                                       transformOdom[1],
                                       transformOdom[2],
                                       transformOdom[3]);
    Eigen::Vector3d transition_odom(transformOdom[4],
                                    transformOdom[5],
                                    transformOdom[6]);

    Eigen::Quaterniond quaternion_map(globalEstimated.at<gtsam::Pose3>(currentIdx).rotation().toQuaternion().w(),
                                      globalEstimated.at<gtsam::Pose3>(currentIdx).rotation().toQuaternion().x(),
                                      globalEstimated.at<gtsam::Pose3>(currentIdx).rotation().toQuaternion().y(),
                                      globalEstimated.at<gtsam::Pose3>(currentIdx).rotation().toQuaternion().z());
    Eigen::Vector3d transition_map(globalEstimated.at<gtsam::Pose3>(currentIdx).translation().x(),
                                   globalEstimated.at<gtsam::Pose3>(currentIdx).translation().y(),
                                   globalEstimated.at<gtsam::Pose3>(currentIdx).translation().z());

    // Compute the odom to map transform of last frame which would be used for the next iteration
    Eigen::Quaterniond quaternion_incremental = quaternion_map * quaternion_odom.inverse();
    Eigen::Vector3d transition_incremental = transition_map - quaternion_incremental * transition_odom;

    transformIncre[0] = quaternion_incremental.w();
    transformIncre[1] = quaternion_incremental.x();
    transformIncre[2] = quaternion_incremental.y();
    transformIncre[3] = quaternion_incremental.z();
    transformIncre[4] = transition_incremental.x();
    transformIncre[5] = transition_incremental.y();
    transformIncre[6] = transition_incremental.z();
  }

  void interpolatePoses()
  {
    Eigen::Quaterniond quaternionLast(transformLast[0], transformLast[1], transformLast[2], transformLast[3]);
    Eigen::Vector3d transitionLast(transformLast[4], transformLast[5], transformLast[6]);

    Eigen::Quaterniond quaternionCurrent(transformMap[0], transformMap[1], transformMap[2], transformMap[3]);
    Eigen::Vector3d transitionCurrent(transformMap[4], transformMap[5], transformMap[6]);

    PointPoseInfo interpolatedPt;

    for (double i = 0.2; i <= 1; i+=0.2)
    {
      Eigen::Quaterniond quaternionInt = quaternionLast.slerp(i, quaternionCurrent);
      Eigen::Vector3d transitionInt = transitionLast + i * (transitionCurrent - transitionLast);

      interpolatedPt.x = transitionInt.x();
      interpolatedPt.y = transitionInt.y();
      interpolatedPt.z = transitionInt.z();
      interpolatedPt.qw = quaternionInt.w();
      interpolatedPt.qx = quaternionInt.x();
      interpolatedPt.qy = quaternionInt.y();
      interpolatedPt.qz = quaternionInt.z();
      interpolatedPt.idx = poseInterpolated->points.size();
      poseInterpolated->push_back(interpolatedPt);

      int idx = poseInterpolated->points.size();

      gtsam::Pose3 poseFrom= gtsam::Pose3(gtsam::Rot3::Quaternion(poseInterpolated->points[idx-2].qw,
                                                                  poseInterpolated->points[idx-2].qx,
                                                                  poseInterpolated->points[idx-2].qy,
                                                                  poseInterpolated->points[idx-2].qz),
                                          gtsam::Point3(poseInterpolated->points[idx-2].x,
                                                        poseInterpolated->points[idx-2].y,
                                                        poseInterpolated->points[idx-2].z));
      gtsam::Pose3 poseTo = gtsam::Pose3(gtsam::Rot3::Quaternion(poseInterpolated->points[idx-1].qw,
                                                                 poseInterpolated->points[idx-1].qx,
                                                                 poseInterpolated->points[idx-1].qy,
                                                                 poseInterpolated->points[idx-1].qz),
                                         gtsam::Point3(poseInterpolated->points[idx-1].x,
                                                       poseInterpolated->points[idx-1].y,
                                                       poseInterpolated->points[idx-1].z));

      localGraph.add(gtsam::BetweenFactor<gtsam::Pose3>(idx - 1,
                                                        idx,
                                                        poseFrom.between(poseTo),
                                                        odometryNoise));
      localInitialEstimate.insert(idx, poseTo);
    }
  }

  // Extract the key frames for building up the map
  void buildLocalMap()
  {
    Timer t_ekf("build the local map");

    // Initialization
    if (poseCloud->points.empty())
    {
      ROS_INFO("Initialization for local map building");
      return;
    }

    // If loop closure detection needed
    if (loopClosureOn == true)
    {
      // If less then 50 frames in recent key frames (model), fetch the last 50 frames
      if (recentCornerKeyFrames.size() < surroundSearchNum)
      {
        recentCornerKeyFrames.clear();
        recentSurfKeyFrames.clear();
        recentOutlierKeyFrames.clear();

        for (int i = poseCloud->points.size() - 1; i >= 0; --i)
        {
          int idx = (int)poseCloud->points[i].intensity;
          recentCornerKeyFrames.push_front(transformCloud(cornerKeyFrames[idx], &poseInfoCloud->points[idx]));
          recentSurfKeyFrames.push_front(transformCloud(surfKeyFrames[idx], &poseInfoCloud->points[idx]));
          recentOutlierKeyFrames.push_front(transformCloud(outlierKeyFrames[idx], &poseInfoCloud->points[idx]));
          if (recentCornerKeyFrames.size() >= surroundSearchNum)
            break;
        }
      }
      // If already more then 50 frames, pop the frames at the beginning
      else
      {
        if (latestFrameIdx != poseCloud->points.size() - 1)
        {
          recentCornerKeyFrames.pop_front();
          recentSurfKeyFrames.pop_front();
          recentOutlierKeyFrames.pop_front();
          latestFrameIdx = poseCloud->points.size() - 1;
          recentCornerKeyFrames.push_back(transformCloud(cornerKeyFrames[latestFrameIdx], &poseInfoCloud->points[latestFrameIdx]));
          recentSurfKeyFrames.push_back(transformCloud(surfKeyFrames[latestFrameIdx], &poseInfoCloud->points[latestFrameIdx]));
          recentOutlierKeyFrames.push_back(transformCloud(outlierKeyFrames[latestFrameIdx], &poseInfoCloud->points[latestFrameIdx]));
        }
      }

      for (int i = 0; i < recentCornerKeyFrames.size(); ++i)
      {
        *cornerFromMap += *recentCornerKeyFrames[i];
        *surfFromMap += *recentSurfKeyFrames[i];
        *surfFromMap += *recentOutlierKeyFrames[i];
      }

//      // Then for the local map for sliding window
//      if (localCornerKeyFrames.size() < surroundSearchNum)
//      {
//        localCornerKeyFrames.clear();
//        localSurfKeyFrames.clear();
//        localOutlierKeyFrames.clear();

//        for (int i = poseLocalCloud->points.size() - 1; i >= 0; --i)
//        {
//          int idx = (int)poseCloud->points[i].intensity;
//          recentCornerKeyFrames.push_front(transformCloud(cornerKeyFrames[idx], &poseInfoCloud->points[idx]));
//          recentSurfKeyFrames.push_front(transformCloud(surfKeyFrames[idx], &poseInfoCloud->points[idx]));
//          recentOutlierKeyFrames.push_front(transformCloud(outlierKeyFrames[idx], &poseInfoCloud->points[idx]));
//          if (recentCornerKeyFrames.size() >= surroundSearchNum)
//            break;
//        }
//      }
//      // If already more then 50 frames, pop the frames at the beginning
//      else
//      {
//        if (latestFrameIdx != poseCloud->points.size() - 1)
//        {
//          recentCornerKeyFrames.pop_front();
//          recentSurfKeyFrames.pop_front();
//          recentOutlierKeyFrames.pop_front();
//          latestFrameIdx = poseCloud->points.size() - 1;
//          recentCornerKeyFrames.push_back(transformCloud(cornerKeyFrames[latestFrameIdx], &poseInfoCloud->points[latestFrameIdx]));
//          recentSurfKeyFrames.push_back(transformCloud(surfKeyFrames[latestFrameIdx], &poseInfoCloud->points[latestFrameIdx]));
//          recentOutlierKeyFrames.push_back(transformCloud(outlierKeyFrames[latestFrameIdx], &poseInfoCloud->points[latestFrameIdx]));
//        }
//      }

//      for (int i = 0; i < recentCornerKeyFrames.size(); ++i)
//      {
//        *cornerFromMap += *recentCornerKeyFrames[i];
//        *surfFromMap += *recentSurfKeyFrames[i];
//        *surfFromMap += *recentOutlierKeyFrames[i];
//      }

    }
    else // No loop closure
    {
      surroundKeyPoses->clear();
      surroundKeyPosesDS->clear();

      // Get the poses within the radius (50m) of last pose
      kdTreeSurroundKeyPoses->setInputCloud(poseCloud);
      kdTreeSurroundKeyPoses->radiusSearch(currentPose, surroundSearchRadius, pointSearchIdx, pointSearchSqDists, 0);

      for (int i = 0; i < pointSearchIdx.size(); ++i)
      {
        surroundKeyPoses->points.push_back(poseCloud->points[pointSearchIdx[i]]);
      }
      downSizeFilterSurroundPoses.setInputCloud(surroundKeyPoses);
      downSizeFilterSurroundPoses.filter(*surroundKeyPosesDS);

      int numSurroundPosesDS = surroundKeyPosesDS->points.size();
      for (int  i = 0; i < surroundExistingKeyPosesIdx.size(); ++i)
      {
        bool poseExists = false;
        for (int j = 0; j < numSurroundPosesDS; ++j)
        {
          if (surroundExistingKeyPosesIdx[i] == int(surroundKeyPosesDS->points[j].intensity))
          {
            poseExists= true;
            break;
          }
        }

        // if the i-th index in surroundExistingKeyPosesIdx does not exist in surroundKeyPoses, delete the key frame
        if (!poseExists)
        {
          surroundExistingKeyPosesIdx.erase(surroundExistingKeyPosesIdx.begin() + i);
          surroundCornerKeyFrames.erase(surroundCornerKeyFrames.begin() + i);
          surroundSurfKeyFrames.erase(surroundSurfKeyFrames.begin() + i);
          surroundOutlierKeyFrames.erase(surroundOutlierKeyFrames.begin() + i);
          --i;
        }
      }

      for (int i = 0; i < numSurroundPosesDS; ++i)
      {
        bool poseExists = false;
        for (auto iter = surroundExistingKeyPosesIdx.begin(); iter != surroundExistingKeyPosesIdx.end(); ++iter)
        {
          if ((*iter) == int(surroundKeyPosesDS->points[i].intensity))
          {
            poseExists = true;
            break;
          }
        }

        if (poseExists)
        {
          continue;
        }
        // If the i-th index in surroundKeyPoses does not exist in surroundExistingKeyPosesIdx, add this frame to key frame
        else
        {
          int idx = int(surroundKeyPosesDS->points[i].intensity);
          surroundExistingKeyPosesIdx.push_back(idx);
          surroundCornerKeyFrames.push_back(transformCloud(cornerKeyFrames[idx], &poseInfoCloud->points[idx]));
          surroundSurfKeyFrames.push_back(transformCloud(surfKeyFrames[idx], &poseInfoCloud->points[idx]));
          surroundOutlierKeyFrames.push_back(transformCloud(outlierKeyFrames[idx], &poseInfoCloud->points[idx]));
        }
      }

      for (int i = 0; i < surroundExistingKeyPosesIdx.size(); ++i)
      {
        *cornerFromMap += *surroundCornerKeyFrames[i];
        *surfFromMap += *surroundSurfKeyFrames[i];
        *surfFromMap += *surroundOutlierKeyFrames[i]; // set the outlier as surface feature points
      }
    }

//    if (printTime)
//      t_ekf.tic_toc();
  }

  void buildLocalMapSlidingWindow()
  {
    // Build up local map based on the indices in poseInterpolated

    // Add the landmark points to the factor graph
    localGraph.emplace_shared<gtsam::BearingRangeFactor<Pose3, Point3> >(x1,
                                                                         l1,
                                                                         bearing11,
                                                                         range11,
                                                                         measurementNoise);


    // optimize
  }

  void downSampleCloud()
  {
    Timer t_dsc("downsampling the point cloud");

    downSizeFilterCorner.setInputCloud(cornerFromMap);
    downSizeFilterCorner.filter(*cornerFromMapDS);
    NumPtsCornerFromMap = cornerFromMap->points.size();
    NumPtsCornerFromMapDS = cornerFromMapDS->points.size();

    downSizeFilterSurf.setInputCloud(surfFromMap);
    downSizeFilterSurf.filter(*surfFromMapDS);
    NumPtsSurfFromMap = surfFromMap->points.size();
    NumPtsSurfFromMapDS = surfFromMapDS->points.size();

    cornerLastDS->clear();
    downSizeFilterCorner.setInputCloud(cornerLast);
    downSizeFilterCorner.filter(*cornerLastDS);
    NumPtsCornerLast = cornerLast->points.size();
    NumPtsCornerLastDS = cornerLastDS->points.size();

    surfLastDS->clear();
    downSizeFilterSurf.setInputCloud(surfLast);
    downSizeFilterSurf.filter(*surfLastDS);
    NumPtsSurfLast = surfLast->points.size();
    NumPtsSurfLastDS = surfLastDS->points.size();

    // Add the planer feature points and the outlier points
    outlierLastDS->clear();
    downSizeFilterOutlier.setInputCloud(outlierLast);
    downSizeFilterOutlier.filter(*outlierLastDS);
    surfOutlierLast->clear();
    surfOutlierLastDS->clear();
    *surfOutlierLast += *surfLastDS;
    *surfOutlierLast += *outlierLastDS;
    downSizeFilterSurf.setInputCloud(surfOutlierLast);
    downSizeFilterSurf.filter(*surfOutlierLastDS);
    NumPtsSurfOutlierLastDS = surfOutlierLastDS->points.size();

    sensor_msgs::PointCloud2 msgs;
    pcl::toROSMsg(*cornerFromMapDS, msgs);
    msgs.header.stamp = ros::Time().fromSec(timeNewOdometry);
    msgs.header.frame_id = "/lidar_init";
    pubCornerMap.publish(msgs);

    pcl::toROSMsg(*surfFromMapDS, msgs);
    msgs.header.stamp = ros::Time().fromSec(timeNewOdometry);
    msgs.header.frame_id = "/lidar_init";
    pubSurfMap.publish(msgs);

//    if (printTime)
//      t_dsc.tic_toc();
  }

  void findCorrespondingCornerFeatures()
  {
    cornerResCount = 0;

    for (int i = 0; i < NumPtsCornerLastDS; ++i)
    {
      ptInLocal = cornerLastDS->points[i];
      transformPoint(&ptInLocal, &ptInMap);
      kdTreeCornerFromMap->nearestKSearch(ptInMap, 5, pointSearchIdx, pointSearchSqDists);

      if (pointSearchSqDists[4] < 1.0)
      {
        std::vector<Eigen::Vector3d> nearCorners;
        Eigen::Vector3d center(0, 0, 0);
        for (int j = 0; j < 5; ++j)
        {
          Eigen::Vector3d pt(cornerFromMapDS->points[pointSearchIdx[j]].x,
                             cornerFromMapDS->points[pointSearchIdx[j]].y,
                             cornerFromMapDS->points[pointSearchIdx[j]].z);
          center = center + pt;
          nearCorners.push_back(pt);
        }
        center /= 5.0;

        // Covariance matrix of distance error
        Eigen::Matrix3d matA1 = Eigen::Matrix3d::Zero();

        for (int j = 0; j < 5; ++j)
        {
          Eigen::Vector3d zeroMean = nearCorners[j] - center;
          matA1 = matA1 + zeroMean * zeroMean.transpose();
        }

        // Computes eigenvalues and eigenvectors of selfadjoint matrices
        // The eigenvalues re sorted in increasing order
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigenSolver(matA1);

        Eigen::Vector3d unitDirection = eigenSolver.eigenvectors().col(2);
        Eigen::Vector3d currentPt(ptInLocal.x, ptInLocal.y, ptInLocal.z);

        // if one eigenvalue is significantly larger than the other two
        if (eigenSolver.eigenvalues()[2] > 3 * eigenSolver.eigenvalues()[1])
        {
          Eigen::Vector3d ptOnLine = center;
          Eigen::Vector3d ptA, ptB;
          ptA = ptOnLine + 0.1 * unitDirection;
          ptB = ptOnLine - 0.1 * unitDirection;

          PointXYZI pointA, pointB;
          pointA.x = ptA.x();
          pointA.y = ptA.y();
          pointA.z = ptA.z();
          pointB.x = ptB.x();
          pointB.y = ptB.y();
          pointB.z = ptB.z();
          cornerCurrentPts->push_back(ptInLocal);
          cornerMatchJ->push_back(pointA);
          cornerMatchL->push_back(pointB);

          ++cornerResCount;
        }
      }
    }
  }

  void findCorrespondingSurfFeatures()
  {
    surfResCount = 0;

    for (int i = 0; i < NumPtsSurfOutlierLastDS; ++i)
    {
      ptInLocal = surfOutlierLastDS->points[i];
      transformPoint(&ptInLocal, &ptInMap);
      kdTreeSurfFromMap->nearestKSearch(ptInMap, 5, pointSearchIdx, pointSearchSqDists);

      Eigen::Matrix<double, 5, 3> matA0;
      Eigen::Matrix<double, 5, 1> matB0 = - Eigen::Matrix<double, 5, 1>::Ones();
      if (pointSearchSqDists[4] < 1.0)
      {
        PointXYZI center;
        for (int j = 0; j < 5; ++j)
        {
          matA0(j, 0) = surfFromMapDS->points[pointSearchIdx[j]].x;
          matA0(j, 1) = surfFromMapDS->points[pointSearchIdx[j]].y;
          matA0(j, 2) = surfFromMapDS->points[pointSearchIdx[j]].z;
        }

        // Get the norm of the plane using linear solver based on QR composition
        Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0);
        double normInverse = 1 / norm.norm();
        norm.normalize(); // get the unit norm

        // Compute the centroid of the plane
        center.x = matA0.col(0).sum() / 5.0;
        center.y = matA0.col(1).sum() / 5.0;
        center.z = matA0.col(2).sum() / 5.0;

        // Make sure that the plan is fit
        bool planeValid = true;
        for (int j = 0; j < 5; ++j)
        {
          if (fabs(norm.x() * surfFromMapDS->points[pointSearchIdx[j]].x +
                   norm.y() * surfFromMapDS->points[pointSearchIdx[j]].y +
                   norm.z() * surfFromMapDS->points[pointSearchIdx[j]].z + normInverse) > 0.2)
          {
            planeValid = false;
            break;
          }
        }

        Eigen::Vector3d currentPt(ptInLocal.x, ptInLocal.y, ptInLocal.z);
        Eigen::Vector3d lastPt(ptInMap.x, ptInMap.y, ptInMap.z);

        // if one eigenvalue is significantly larger than the other two
        if (planeValid)
        {
          float pd = norm.x() * ptInMap.x + norm.y() * ptInMap.y + norm.z() *ptInMap.z + normInverse;
          float weight = 1 - 0.9 * fabs(pd) / sqrt(sqrt(ptInMap.x * ptInMap.x + ptInMap.y * ptInMap.y + ptInMap.z * ptInMap.z));

          PointXYZI normal;
          normal.x = weight * norm.x();
          normal.y = weight * norm.y();
          normal.z = weight * norm.z();
          normal.intensity = weight * normInverse;

          surfCurrentPts->push_back(ptInLocal);
          surfCenter->push_back(center);
          surfNormal->push_back(normal);

          ++surfResCount;
        }
      }
    }
  }

  void updateTransformationWithCeres()
  {
    Timer t_utwc("update the transformation");

    if (NumPtsCornerFromMapDS > 10 && NumPtsSurfFromMapDS > 50)
    {
      Timer t_tree("build the kdtree");
      kdTreeCornerFromMap->setInputCloud(cornerFromMapDS);
      kdTreeSurfFromMap->setInputCloud(surfFromMapDS);
//      t_tree.tic_toc();

      for (int iterCount = 0; iterCount < 2; ++iterCount)
      {
        ceres::LossFunction *lossFunction = new ceres::HuberLoss(0.1);
        ceres::LocalParameterization *quatParameterization = new ceres::QuaternionParameterization();

        ceres::Problem::Options problemOptions;
        ceres::Problem problem(problemOptions);
        problem.AddParameterBlock(transformMap, 4, quatParameterization);
        problem.AddParameterBlock(transformMap + 4, 3);

        findCorrespondingCornerFeatures();
        findCorrespondingSurfFeatures();

        for (int i = 0; i < cornerResCount; ++i)
        {
          Eigen::Vector3d currentPt(cornerCurrentPts->points[i].x,
                                    cornerCurrentPts->points[i].y,
                                    cornerCurrentPts->points[i].z);
          Eigen::Vector3d lastPtJ(cornerMatchJ->points[i].x,
                                  cornerMatchJ->points[i].y,
                                  cornerMatchJ->points[i].z);
          Eigen::Vector3d lastPtL(cornerMatchL->points[i].x,
                                  cornerMatchL->points[i].y,
                                  cornerMatchL->points[i].z);

//          ceres::CostFunction *costFunction = LidarEdgeFactor::Create(currentPt, lastPtJ, lastPtL, 1.0);
          LidarEdgeFactorAnalytic *costFunction = new LidarEdgeFactorAnalytic(currentPt, lastPtJ, lastPtL, 1.0);
          problem.AddResidualBlock(costFunction, lossFunction, transformMap, transformMap + 4);
        }

        for (int i = 0; i < surfResCount; ++i)
        {
          Eigen::Vector3d currentPt(surfCurrentPts->points[i].x,
                                    surfCurrentPts->points[i].y,
                                    surfCurrentPts->points[i].z);
          Eigen::Vector3d norm(surfNormal->points[i].x,
                               surfNormal->points[i].y,
                               surfNormal->points[i].z);
          double normInverse = surfNormal->points[i].intensity;
          Eigen::Vector3d center(surfCenter->points[i].x,
                                 surfCenter->points[i].y,
                                 surfCenter->points[i].z);

//          ceres::CostFunction *costFunction = LidarPlaneNormFactor::Create(currentPt, norm, normInverse);
          LidarPlaneNormFactorAnalytical *costFunction = new LidarPlaneNormFactorAnalytical(currentPt, norm, normInverse, center);
          problem.AddResidualBlock(costFunction, lossFunction, transformMap, transformMap + 4);
        }

        ceres::Solver::Options options;
//        options.minimizer_type = LINE_SEARCH;
        options.linear_solver_type = ceres::DENSE_QR;
        options.max_num_iterations = 4;
        options.minimizer_progress_to_stdout = false;
        options.check_gradients = false;
        options.gradient_check_relative_precision = 1e-4;
        ceres::Solver::Summary summary;

        Timer t_s("solving with ceres");
        ceres::Solve(options, &problem, &summary);
//        if (printTime)
//          t_s.tic_toc();

//        std::cout << summary.BriefReport() << "\n";

        cornerCurrentPts->clear();
        cornerMatchJ->clear();
        cornerMatchL->clear();
        surfCurrentPts->clear();
        surfCenter->clear();
        surfNormal->clear();
      }
    }
    else
    {
      ROS_WARN("Not enough feature points from the map");
    }

    computeIncremental();

//    if (printTime)
//      t_utwc.tic_toc();
  }

  void updateTransformationWithROPTLIB()
  {
    Timer t_utwr("update the transformation");

    if (NumPtsCornerFromMapDS > 10 && NumPtsSurfFromMapDS > 50)
    {
      Timer t_tree("build the kdtree");
      kdTreeCornerFromMap->setInputCloud(cornerFromMapDS);
      kdTreeSurfFromMap->setInputCloud(surfFromMapDS);
//      t_tree.tic_toc();

      findCorrespondingCornerFeatures();
      findCorrespondingSurfFeatures();

      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> cornerMat;
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> surfMat;
      cornerMat.setZero(9, cornerResCount);
      surfMat.setZero(7, surfResCount);

      for (int iterCount = 0; iterCount < 2; ++iterCount)
      {
        // dimension of each manifold in product of manifold
        int dimOfRot = 4;
        int dimOfTrans = 3;

        // Obtain an initial iterate based on S^4 x R3
        ROPTLIB::SphereVariable sphereVar(dimOfRot);
        ROPTLIB::EucVariable EucVar(dimOfTrans);

        // Generate the product element of manifolds
        ROPTLIB::ProductElement SE3Iterate(2, &sphereVar, 1, &EucVar, 1);

        // Allocate new memory without copying the data
        SE3Iterate.NewMemoryOnWrite();

        // Set the pointer to the sphere variable and the euclidean variable
        double *spherePtr = SE3Iterate.GetElement(0)->ObtainWriteEntireData();
        double *eucPtr = SE3Iterate.GetElement(1)->ObtainWriteEntireData();

        spherePtr[0] = transformMap[0];
        spherePtr[1] = transformMap[1];
        spherePtr[2] = transformMap[2];
        spherePtr[3] = transformMap[3];
        eucPtr[0] = transformMap[4];
        eucPtr[1] = transformMap[5];
        eucPtr[2] = transformMap[6];

        // Define the manifolds for SE(3)
        ROPTLIB::Sphere sphereMani(dimOfRot);
        sphereMani.ChooseSphereParamsSet2();
        ROPTLIB::Euclidean EucMani(dimOfTrans);
        ROPTLIB::ProductManifold Domain(2, &sphereMani, 1, &EucMani, 1);

        int col = 0;
        for (int i = 0; i < cornerCurrentPts->points.size(); ++i)
        {
          Eigen::Vector3d currentPt(cornerCurrentPts->points[i].x,
                                    cornerCurrentPts->points[i].y,
                                    cornerCurrentPts->points[i].z);
          Eigen::Vector3d lastPtJ(cornerMatchJ->points[i].x,
                                  cornerMatchJ->points[i].y,
                                  cornerMatchJ->points[i].z);
          Eigen::Vector3d lastPtL(cornerMatchL->points[i].x,
                                  cornerMatchL->points[i].y,
                                  cornerMatchL->points[i].z);

          cornerMat.block<3, 1>(0, col) = currentPt;
          cornerMat.block<3, 1>(3, col) = lastPtJ;
          cornerMat.block<3, 1>(6, col) = lastPtL;

          ++col;
        }

        col = 0;
        for (int i = 0; i < surfCurrentPts->points.size(); ++i)
        {
          Eigen::Vector3d currentPt(surfCurrentPts->points[i].x,
                                    surfCurrentPts->points[i].y,
                                    surfCurrentPts->points[i].z);
          Eigen::Vector3d norm(surfNormal->points[i].x,
                               surfNormal->points[i].y,
                               surfNormal->points[i].z);
          double normInverse = surfNormal->points[i].intensity;

          surfMat.block<3, 1>(0, col) = currentPt;
          surfMat.block<3, 1>(3, col) = norm;
          surfMat(6, col) = normInverse;

          ++col;
        }

        // Define the problem
        mapOptProblem Prob(cornerMat, surfMat, cornerResCount, surfResCount);

        // Set the domain of the problem to be the product manifold
        Prob.SetDomain(&Domain);

        // Apply the Riemannian steepest descent
        ROPTLIB::RSD *solver = new ROPTLIB::RSD(&Prob, &SE3Iterate);

        solver->Max_Iteration = 10;

        // Specify what information will be output in the algorithm.
        solver->Debug = ROPTLIB::NOOUTPUT;
        // enum DEBUGINFO{ NOOUTPUT, FINALRESULT, ITERRESULT, DETAILED, DEBUGLENGTH };

        solver->Run();

        // Obtain the optimized iterates
        const ROPTLIB::Element *xOpt = SE3Iterate.ConstructEmpty();
        xOpt = solver->GetXopt();

        const double *optPtr = xOpt->ObtainReadData();

        transformMap[0] = optPtr[0];
        transformMap[1] = optPtr[1];
        transformMap[2] = optPtr[2];
        transformMap[3] = optPtr[3];
        transformMap[4] = optPtr[4];
        transformMap[5] = optPtr[5];
        transformMap[6] = optPtr[6];

        cornerCurrentPts->clear();
        cornerMatchJ->clear();
        cornerMatchL->clear();
        surfCurrentPts->clear();
        surfCenter->clear();
        surfNormal->clear();
      }
    }
    else
    {
      ROS_WARN("Not enough feature points from the map");
    }

    computeIncremental();

  }

  void saveKeyFramesAndFactors()
  {
    currentPose.x = transformMap[4];
    currentPose.y = transformMap[5];
    currentPose.z = transformMap[6];

    // If the new pose is very near to the last (d < 0.3m), do not save it as a key frame
    bool save = true;

    if (sqrt((lastPose.x - currentPose.x) * (lastPose.x - currentPose.x)
            +(lastPose.y - currentPose.y) * (lastPose.y - currentPose.y)
            +(lastPose.z - currentPose.z) * (lastPose.z - currentPose.z)) < 0.3)
      save = false;

    // Return if the frame should not be regarded as key frame, only when it is the first pose
    if (save == false && !poseCloud->points.empty())
      return;

    // If a new keyframe is to be stored
    lastPose = currentPose;

    if (poseCloud->points.empty())
    {
      gtsam::Rot3 rotation = gtsam::Rot3::Quaternion(transformMap[0], transformMap[1], transformMap[2], transformMap[3]);
      gtsam::Point3 transition = gtsam::Point3(transformMap[4], transformMap[5], transformMap[6]);

      // Initialization for global pose graph
      globalGraph.add(gtsam::PriorFactor<gtsam::Pose3>(0, gtsam::Pose3(rotation, transition), priorNoise));
      globalInitialEstimate.insert(0, gtsam::Pose3(rotation, transition));

      // Initilization for local pose graph
      localGraph.add(gtsam::PriorFactor<gtsam::Pose3>(0, gtsam::Pose3(rotation, transition), priorNoise));
      localInitialEstimate.insert(0, gtsam::Pose3(rotation, transition));

      for (int i = 0; i < 7; ++i)
      {
        transformLast[i] = transformMap[i];
      }
    }
    else
    {
      gtsam::Rot3 rotationLast = gtsam::Rot3::Quaternion(transformLast[0], transformLast[1], transformLast[2], transformLast[3]);
      gtsam::Point3 transitionLast = gtsam::Point3(transformLast[4], transformLast[5], transformLast[6]);

      gtsam::Rot3 rotationCur = gtsam::Rot3::Quaternion(transformMap[0], transformMap[1], transformMap[2], transformMap[3]);
      gtsam::Point3 transitionCur = gtsam::Point3(transformMap[4], transformMap[5], transformMap[6]);

      gtsam::Pose3 poseFrom = gtsam::Pose3(rotationLast, transitionLast);
      gtsam::Pose3 poseTo = gtsam::Pose3(rotationCur, transitionCur);

      globalGraph.add(gtsam::BetweenFactor<gtsam::Pose3>(poseCloud->points.size() - 1,
                                                        poseCloud->points.size(),
                                                        poseFrom.between(poseTo),
                                                        odometryNoise));
      globalInitialEstimate.insert(poseCloud->points.size(), gtsam::Pose3(rotationCur, transitionCur));

      interpolatePoses();
    }

    isam->update(globalGraph, globalInitialEstimate);
    isam->update();

    globalGraph.resize(0);
    globalInitialEstimate.clear();

    PointXYZI latestPose;
    PointPoseInfo latestPoseInfo;
    gtsam::Pose3 latestEstimate;

    globalEstimated = isam->calculateEstimate();
    latestEstimate = globalEstimated.at<gtsam::Pose3>(globalEstimated.size() - 1);

    latestPose.x = latestEstimate.translation().x();
    latestPose.y = latestEstimate.translation().y();
    latestPose.z = latestEstimate.translation().z();
    latestPose.intensity = poseCloud->points.size();
    poseCloud->push_back(latestPose);

    latestPoseInfo.x = latestEstimate.translation().x();
    latestPoseInfo.y = latestEstimate.translation().y();
    latestPoseInfo.z = latestEstimate.translation().z();
    latestPoseInfo.qw = latestEstimate.rotation().toQuaternion().w();
    latestPoseInfo.qx = latestEstimate.rotation().toQuaternion().x();
    latestPoseInfo.qy = latestEstimate.rotation().toQuaternion().y();
    latestPoseInfo.qz = latestEstimate.rotation().toQuaternion().z();
    latestPoseInfo.idx = poseCloud->points.size();
    latestPoseInfo.time = timeNewOdometry;
    poseInfoCloud->push_back(latestPoseInfo);

    if (poseCloud->points.size() > 1)
    {
      transformMap[0] = latestEstimate.rotation().toQuaternion().w();
      transformMap[1] = latestEstimate.rotation().toQuaternion().x();
      transformMap[2] = latestEstimate.rotation().toQuaternion().y();
      transformMap[3] = latestEstimate.rotation().toQuaternion().z();
      transformMap[4] = latestEstimate.translation().x();
      transformMap[5] = latestEstimate.translation().y();
      transformMap[6] = latestEstimate.translation().z();

      for (int i = 0; i < 7; ++i)
      {
        transformLast[i] = transformMap[i];

      }
    }

    pcl::PointCloud<PointXYZI>::Ptr cornerFrame(new pcl::PointCloud<PointXYZI>());
    pcl::PointCloud<PointXYZI>::Ptr surfFrame(new pcl::PointCloud<PointXYZI>());
    pcl::PointCloud<PointXYZI>::Ptr outlierFrame(new pcl::PointCloud<PointXYZI>());

    pcl::copyPointCloud(*cornerLastDS, *cornerFrame);
    pcl::copyPointCloud(*surfLastDS, *surfFrame);
    pcl::copyPointCloud(*outlierLastDS, *outlierFrame);

    cornerKeyFrames.push_back(cornerFrame);
    surfKeyFrames.push_back(surfFrame);
    outlierKeyFrames.push_back(outlierFrame);
  }

  void correctPoses()
  {
    if (loopClosed == true)
    {
      recentCornerKeyFrames.clear();
      recentSurfKeyFrames.clear();
      recentOutlierKeyFrames.clear();

      int numPoses = globalEstimated.size();
      for (int i = 0; i < numPoses; ++i)
      {
        poseCloud->points[i].x = globalEstimated.at<gtsam::Pose3>(i).translation().x();
        poseCloud->points[i].y = globalEstimated.at<gtsam::Pose3>(i).translation().y();
        poseCloud->points[i].z = globalEstimated.at<gtsam::Pose3>(i).translation().z();

        poseInfoCloud->points[i].x = globalEstimated.at<gtsam::Pose3>(i).translation().x();
        poseInfoCloud->points[i].y = globalEstimated.at<gtsam::Pose3>(i).translation().y();
        poseInfoCloud->points[i].z = globalEstimated.at<gtsam::Pose3>(i).translation().z();
        poseInfoCloud->points[i].qw = globalEstimated.at<gtsam::Pose3>(i).rotation().toQuaternion().w();
        poseInfoCloud->points[i].qx = globalEstimated.at<gtsam::Pose3>(i).rotation().toQuaternion().x();
        poseInfoCloud->points[i].qy = globalEstimated.at<gtsam::Pose3>(i).rotation().toQuaternion().y();
        poseInfoCloud->points[i].qz = globalEstimated.at<gtsam::Pose3>(i).rotation().toQuaternion().z();
      }

      // If the poses are to be corrected, so is the incremental transform for next iteration
      correctIncremental();

      loopClosed = false;
    }
  }

  void publishOdometry()
  {
    odomMapping.header.stamp = ros::Time().fromSec(timeNewOdometry);
    odomMapping.pose.pose.orientation.w = transformMap[0];
    odomMapping.pose.pose.orientation.x = transformMap[1];
    odomMapping.pose.pose.orientation.y = transformMap[2];
    odomMapping.pose.pose.orientation.z = transformMap[3];
    odomMapping.pose.pose.position.x = transformMap[4];
    odomMapping.pose.pose.position.y = transformMap[5];
    odomMapping.pose.pose.position.z = transformMap[6];
    pubOdom.publish(odomMapping);

    geometry_msgs::PoseStamped poseStamped;
    poseStamped.header = odomMapping.header;
    poseStamped.pose = odomMapping.pose.pose;
    poseStamped.header.stamp = odomMapping.header.stamp;
    pathMapping.header.stamp = odomMapping.header.stamp;
    pathMapping.poses.push_back(poseStamped);
    pathMapping.header.frame_id = "/lidar_init";
    pubPath.publish(pathMapping);

    mapTransform.stamp_ = odomMapping.header.stamp;
    mapTransform.setRotation(tf::Quaternion(transformMap[1], transformMap[2], transformMap[3], transformMap[0]));
    mapTransform.setOrigin(tf::Vector3(transformMap[4], transformMap[5], transformMap[6]));
    broadcaster.sendTransform(mapTransform);

    sensor_msgs::PointCloud2 msgs;
    if (pubPose.getNumSubscribers())
    {
      pcl::toROSMsg(*poseCloud, msgs);
      msgs.header.stamp = ros::Time().fromSec(timeNewOdometry);
      msgs.header.frame_id = "/lidar_init";
      pubPose.publish(msgs);
    }

    // publish the corner and surf feature points in lidar_init frame
    if (pubCorner.getNumSubscribers())
    {
      for (int i = 0; i < cornerCurrent->points.size(); ++i)
      {
        transformPoint(&cornerCurrent->points[i], &cornerCurrent->points[i]);
      }
      pcl::toROSMsg(*cornerCurrent, msgs);
      msgs.header.stamp = ros::Time().fromSec(timeNewOdometry);
      msgs.header.frame_id = "/lidar_init";
      pubCorner.publish(msgs);
    }

    if (pubSurf.getNumSubscribers())
    {
      for (int i = 0; i < surfCurrent->points.size(); ++i)
      {
        transformPoint(&surfCurrent->points[i], &surfCurrent->points[i]);
      }
      pcl::toROSMsg(*surfCurrent, msgs);
      msgs.header.stamp = ros::Time().fromSec(timeNewOdometry);
      msgs.header.frame_id = "/lidar_init";
      pubSurf.publish(msgs);
    }
  }

  void clearCloud()
  {
    cornerFromMap->clear();
    cornerFromMapDS->clear();
    surfFromMap->clear();
    surfFromMapDS->clear();
  }

  void loopClosureThread()
  {
    if (!loopClosureOn)
      return;

    ros::Rate rate(1);
    while (ros::ok())
    {
      rate.sleep();
      performLoopClosure();
    }
  }

  bool detectLoopClosure()
  {
    latestKeyFrames->clear();
    historyKeyFrames->clear();
    historyKeyFramesDS->clear();

    std::lock_guard<std::mutex> lock(mutualExclusion);

    // Look for the closest key frames
    std::vector<int> pointSearchIdxLoop;
    std::vector<float> pointSearchSqDistsLoop;
    kdTreeHistoryKeyPoses->setInputCloud(poseCloud);
    kdTreeHistoryKeyPoses->radiusSearch(currentPose, historySearchRadius, pointSearchIdxLoop, pointSearchSqDistsLoop, 0);

    closestHistoryIdx = -1;
    for (int i = 0; i < pointSearchIdxLoop.size(); ++i)
    {
      int idx = pointSearchIdxLoop[i];
      if (abs(poseInfoCloud->points[idx].time - timeNewOdometry) > 30.0)
      {
        closestHistoryIdx = idx;
        break;
      }
    }

    if (closestHistoryIdx == -1)
      return false;

    // Combine the corner and surf frames to form the latest frame
    latestFrameIdxLoop = poseCloud->points.size() - 1;
    *latestKeyFrames += *transformCloud(cornerKeyFrames[latestFrameIdxLoop], &poseInfoCloud->points[latestFrameIdxLoop]);
    *latestKeyFrames += *transformCloud(surfKeyFrames[latestFrameIdxLoop], &poseInfoCloud->points[latestFrameIdxLoop]);

    // Form the history frame for loop closure detection
    for (int j = -historySearchNum; j <= historySearchNum; ++j)
    {
      if (closestHistoryIdx + j < 0 || closestHistoryIdx + j > latestFrameIdxLoop)
        continue;

      *historyKeyFrames += *transformCloud(cornerKeyFrames[closestHistoryIdx+j], &poseInfoCloud->points[closestHistoryIdx+j]);
      *historyKeyFrames += *transformCloud(surfKeyFrames[closestHistoryIdx+j], &poseInfoCloud->points[closestHistoryIdx+j]);
    }

    downSizeFilterHistoryFrames.setInputCloud(historyKeyFrames);
    downSizeFilterHistoryFrames.filter(*historyKeyFramesDS);

    if (pubHistory.getNumSubscribers() != 0){
        sensor_msgs::PointCloud2 msgs;
        pcl::toROSMsg(*historyKeyFramesDS, msgs);
        msgs.header.stamp = ros::Time().fromSec(timeNewOdometry);
        msgs.header.frame_id = "/lidar_init";
        pubHistory.publish(msgs);
    }

    return true;
  }

  void performLoopClosure()
  {
    if (poseCloud->points.empty())
      return;

    if (!loopToClose)
    {
      if (detectLoopClosure())
      {
        loopToClose = true;
        timeSaveLoopClosure = timeNewOdometry;
      }
      if (!loopToClose)
        return;
    }

    loopToClose = false;

    pcl::IterativeClosestPoint<PointXYZI, PointXYZI> icp;
    icp.setMaxCorrespondenceDistance(100);
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(0);

    icp.setInputSource(latestKeyFrames);
    icp.setInputTarget(historyKeyFramesDS);
    pcl::PointCloud<PointXYZI>::Ptr alignedCloud(new pcl::PointCloud<PointXYZI>());
    icp.align(*alignedCloud);

    if (!icp.hasConverged() || icp.getFitnessScore() > historyFitnessScore)
      return;

    ROS_INFO("******************* Loop Closure detected! *******************");

    Eigen::Matrix4d correctedTranform;
    correctedTranform = icp.getFinalTransformation().cast<double>();
    Eigen::Quaterniond quaternionIncre(correctedTranform.block<3, 3>(0, 0));
    Eigen::Vector3d transitionIncre(correctedTranform.block<3, 1>(0, 3));
    Eigen::Quaterniond quaternionToCorrect(poseInfoCloud->points[latestFrameIdxLoop].qw,
                                           poseInfoCloud->points[latestFrameIdxLoop].qx,
                                           poseInfoCloud->points[latestFrameIdxLoop].qy,
                                           poseInfoCloud->points[latestFrameIdxLoop].qz);
    Eigen::Vector3d transitionToCorrect(poseInfoCloud->points[latestFrameIdxLoop].x,
                                        poseInfoCloud->points[latestFrameIdxLoop].y,
                                        poseInfoCloud->points[latestFrameIdxLoop].z);

    Eigen::Quaterniond quaternionCorrected = quaternionIncre * quaternionToCorrect;
    Eigen::Vector3d transitionCorrected = quaternionIncre * transitionToCorrect + transitionIncre;

    gtsam::Rot3 rotationFrom = gtsam::Rot3::Quaternion(quaternionCorrected.w(), quaternionCorrected.x(), quaternionCorrected.y(), quaternionCorrected.z());
    gtsam::Point3 transitionFrom = gtsam::Point3(transitionCorrected.x(), transitionCorrected.y(), transitionCorrected.z());

    gtsam::Rot3 rotationTo = gtsam::Rot3::Quaternion(poseInfoCloud->points[closestHistoryIdx].qw,
                                                     poseInfoCloud->points[closestHistoryIdx].qx,
                                                     poseInfoCloud->points[closestHistoryIdx].qy,
                                                     poseInfoCloud->points[closestHistoryIdx].qz);
    gtsam::Point3 transitionTo = gtsam::Point3(poseInfoCloud->points[closestHistoryIdx].x,
                                               poseInfoCloud->points[closestHistoryIdx].y,
                                               poseInfoCloud->points[closestHistoryIdx].z);

    gtsam::Pose3 poseFrom = gtsam::Pose3(rotationFrom, transitionFrom);
    gtsam::Pose3 poseTo = gtsam::Pose3(rotationTo, transitionTo);
    gtsam::Vector vector6(6);
    double noiseScore = icp.getFitnessScore();
    vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
    constraintNoise = gtsam::noiseModel::Diagonal::Variances(vector6);

    std::lock_guard<std::mutex> lock(mutualExclusion);

    globalGraph.add(gtsam::BetweenFactor<gtsam::Pose3>(latestFrameIdxLoop,
                                                      closestHistoryIdx,
                                                      poseFrom.between(poseTo),
                                                      constraintNoise));
    isam->update(globalGraph);
    isam->update();
    globalGraph.resize(0);

    loopClosed = true;
  }

  void publishMap()
  {
    if (pubMap.getNumSubscribers() && !poseCloud->points.empty())
    {
      std::vector<int> pointSearchIdxGlobal;
      std::vector<float> pointSearchSqDistsGlobal;

      mutualExclusion.lock();
      kdTreeGlobalMap->setInputCloud(poseCloud);
      kdTreeGlobalMap->radiusSearch(currentPose, globalMapSearchRadius, pointSearchIdxGlobal, pointSearchSqDistsGlobal, 0);
      mutualExclusion.unlock();

      for (int i = 0; i < pointSearchIdxGlobal.size(); ++i)
        globalPoses->points.push_back(poseCloud->points[pointSearchIdxGlobal[i]]);

      downSizeFilterGlobalPoses.setInputCloud(globalPoses);
      downSizeFilterGlobalPoses.filter(*globalPosesDS);

      for (int i = 0; i < globalPosesDS->points.size(); ++i)
      {
        int idx = int(globalPosesDS->points[i].intensity);
        *globalMap += *transformCloud(cornerKeyFrames[idx], &poseInfoCloud->points[idx]);
        *globalMap += *transformCloud(surfKeyFrames[idx], &poseInfoCloud->points[idx]);
        *globalMap += *transformCloud(outlierKeyFrames[idx], &poseInfoCloud->points[idx]);
      }

      downSizeFilterGlobalFrames.setInputCloud(globalMap);
      downSizeFilterGlobalFrames.filter(*globalMapDS);

      sensor_msgs::PointCloud2 msgs;
      pcl::toROSMsg(*globalMapDS, msgs);
      msgs.header.stamp = ros::Time().fromSec(timeNewOdometry);
      msgs.header.frame_id = "/lidar_init";
      pubMap.publish(msgs);

      globalPoses->clear();
      globalPosesDS->clear();
      globalMap->clear();
      globalMapDS->clear();
    }
  }

  void mapVisualizationThread()
  {
    ros::Rate rate(0.2);
    while (ros::ok())
    {
      rate.sleep();
      ROS_INFO("Publishing the map");
      publishMap();
    }
  }

  void exportPose()
  {
    std::string poseFile = "/home/robin/poses/current.txt";
    std:ofstream openFile;
    openFile.open(poseFile, std::ofstream::out | std::ofstream::trunc);

    for (int i = 0; i < globalEstimated.size(); ++i)
    {
      double tx = globalEstimated.at<gtsam::Pose3>(i).translation().x();
      double ty = globalEstimated.at<gtsam::Pose3>(i).translation().y();
      double tz = globalEstimated.at<gtsam::Pose3>(i).translation().z();
      Eigen::Quaterniond quat(globalEstimated.at<gtsam::Pose3>(i).rotation().toQuaternion().w(),
                              globalEstimated.at<gtsam::Pose3>(i).rotation().toQuaternion().x(),
                              globalEstimated.at<gtsam::Pose3>(i).rotation().toQuaternion().y(),
                              globalEstimated.at<gtsam::Pose3>(i).rotation().toQuaternion().z());

      Eigen::Matrix3d rotateMat = quat.normalized().toRotationMatrix();

      openFile << std::fixed << std::setprecision(9) <<
                  rotateMat(0, 0) << " " << rotateMat(0, 1) << " " << rotateMat(0, 2) << " " << tx << " " <<
                  rotateMat(1, 0) << " " << rotateMat(1, 1) << " " << rotateMat(1, 2) << " " << ty << " " <<
                  rotateMat(2, 0) << " " << rotateMat(2, 1) << " " << rotateMat(2, 2) << " " << tz << std::endl;
    }
    openFile.close();
  }

  void run()
  {
    if (newCornerLast && newSurfLast && newOutlierLast && newOdom &&
        std::abs(timeNewCornerLast - timeNewOdometry) < 0.005 &&
        std::abs(timeNewSurfLast - timeNewOdometry) < 0.005 &&
        std::abs(timeNewOutlierLast - timeNewOdometry) < 0.005)
    {
      newCornerLast = false;
      newSurfLast = false;
      newOutlierLast = false;
      newOdom = false;

      std::lock_guard<std::mutex> lock(mutualExclusion);

      if (timeNewOdometry - timeLastProcessing >= 0.3)
      {
        timeLastProcessing = timeNewOdometry;

        Timer t_map("LidarMapping");

        setInitialTransform();
        buildLocalMap();
        downSampleCloud();
        updateTransformationWithCeres();
//        updateTransformationWithROPTLIB();
        saveKeyFramesAndFactors();
        correctPoses();
        publishOdometry();
        clearCloud();
//        exportPose();
        t_map.tic_toc();
      }
    }
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lidar_loam");

  ROS_INFO("\033[1;32m---->\033[0m Lidar Mapping Started.");

  LidarMapping lm;

  std::thread threadLoopClosure(&LidarMapping::loopClosureThread, &lm);
  std::thread threadMapVisualization(&LidarMapping::mapVisualizationThread, &lm);

  ros::Rate rate(200);

  while (ros::ok())
  {
      ros::spinOnce();

      lm.run();

      rate.sleep();
  }

  ROS_INFO("txt saved");

  threadLoopClosure.join();
  threadMapVisualization.join();

  return 0;
}
