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

class LidarOdometry
{
private:

  ros::NodeHandle nh;

  ros::Subscriber subSegmentedCloud;
  ros::Subscriber subCornerPointsSharp;
  ros::Subscriber subCornerPointsLessSharp;
  ros::Subscriber subSurfPointsFlat;
  ros::Subscriber subSurfPointsLessFlat;

  ros::Publisher pubSegmentedCloud;
  ros::Publisher pubCornerLast;
  ros::Publisher pubSurfLast;
  ros::Publisher pubOdometry;
  ros::Publisher pubPath;
  ros::Publisher pubGroundTruth;
  ros::Publisher pubCloudSurfel;

  std_msgs::Header cloudHeader;
  nav_msgs::Odometry odom;
  nav_msgs::Path path;
  nav_msgs::Path groundTruth;

  pcl::PointCloud<PointXYZI>::Ptr segmentedCloud;
  pcl::PointCloud<PointXYZI>::Ptr cornerPointsSharp;
  pcl::PointCloud<PointXYZI>::Ptr cornerPointsLessSharp;
  pcl::PointCloud<PointXYZI>::Ptr surfPointsFlat;
  pcl::PointCloud<PointXYZI>::Ptr surfPointsLessFlat;

  pcl::PointCloud<PointXYZI>::Ptr laserCloudCornerLast;
  pcl::PointCloud<PointXYZI>::Ptr laserCloudSurfLast;

  int numPtsCornerLast;
  int numPtsSurfLast;

  PointXYZI pointSel;

  pcl::KdTreeFLANN<PointXYZI>::Ptr kdTreeCornerLast;
  pcl::KdTreeFLANN<PointXYZI>::Ptr kdTreeSurfLast;

  std::vector<int> pointSearchIdx;
  std::vector<float> pointSearchSqDists;

  int pointSelCornerIdx[N_SCAN*Horizon_SCAN];
  int pointSearchCornerIdx1[N_SCAN*Horizon_SCAN];
  int pointSearchCornerIdx2[N_SCAN*Horizon_SCAN];

  int pointSelSurfIdx[N_SCAN*Horizon_SCAN];
  int pointSearchSurfIdx1[N_SCAN*Horizon_SCAN];
  int pointSearchSurfIdx2[N_SCAN*Horizon_SCAN];
  int pointSearchSurfIdx3[N_SCAN*Horizon_SCAN];

  tf::TransformListener tfListner;
  tf::TransformBroadcaster tfBroadcaster;
  tf::StampedTransform odomTransform;
  tf::StampedTransform groundTruthTransform;
  tf::StampedTransform groundTruthInit;

  // Form of the transformation
  // transform = [quaternion: w, x, y, z | transition: x, y, z]
  // Notice the quaternion conversion
  // <=> Eigen::Quaternion assignment (w, x, y, z)
  // <=> tf::Quaternion assignment (x, y, z, w)
  double transformCur[7];
  double transformSum[7];

  bool newSegCloud;
  bool newSharpPoints;
  bool newFlatPoints;

  double timeNewSegCloud;
  double timeNewSharpPoints;
  double timeNewFlatPoints;

  int frameCount;
  int skipFrameNum;

  bool systemInitialized;

  int cornerResCount;
  int surfResCount;

  int numIter;
  int numOpt;

  bool distortion;
  bool printTime;
  bool groundTruthExists;

public:
  LidarOdometry():
    nh("~")
  {
    subSegmentedCloud = nh.subscribe<sensor_msgs::PointCloud2>("/segmented_point_cloud", 100, &LidarOdometry::cloudHandler, this);
    subCornerPointsSharp = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 100, &LidarOdometry::laserCloudSharpHandler, this);
    subCornerPointsLessSharp = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 100, &LidarOdometry::laserCloudLessSharpHandler, this);
    subSurfPointsFlat = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_flat", 100, &LidarOdometry::laserCloudFlatHandler, this);
    subSurfPointsLessFlat = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 100, &LidarOdometry::laserCloudLessFlatHandler, this);

    pubSegmentedCloud = nh.advertise<sensor_msgs::PointCloud2>("segmented_point_cloud_2", 100);
    pubCornerLast = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_corner_last", 100);
    pubSurfLast = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surf_last", 100);
    pubOdometry = nh.advertise<nav_msgs::Odometry>("/odom", 100);
    pubPath = nh.advertise<nav_msgs::Path>("/path", 100);
    pubGroundTruth = nh.advertise<nav_msgs::Path>("/ground_truth", 100);

    allocateMemory();
    initializeParameters();
  }

  ~LidarOdometry(){}

  void allocateMemory()
  {
    segmentedCloud.reset(new pcl::PointCloud<PointXYZI>());
    cornerPointsSharp.reset(new pcl::PointCloud<PointXYZI>());
    cornerPointsLessSharp.reset(new pcl::PointCloud<PointXYZI>());
    surfPointsFlat.reset(new pcl::PointCloud<PointXYZI>());
    surfPointsLessFlat.reset(new pcl::PointCloud<PointXYZI>());

    laserCloudCornerLast.reset(new pcl::PointCloud<PointXYZI>());
    laserCloudSurfLast.reset(new pcl::PointCloud<PointXYZI>());

    kdTreeCornerLast.reset(new pcl::KdTreeFLANN<PointXYZI>());
    kdTreeSurfLast.reset(new pcl::KdTreeFLANN<PointXYZI>());
  }

  void initializeParameters()
  {
    segmentedCloud->clear();
    cornerPointsSharp->clear();
    cornerPointsLessSharp->clear();
    surfPointsFlat->clear();
    surfPointsLessFlat->clear();

    skipFrameNum = 1;
    frameCount = skipFrameNum;

    newSegCloud = false;
    newSharpPoints = false;
    newFlatPoints = false;

    timeNewSegCloud = 0;
    timeNewSharpPoints = 0;
    timeNewFlatPoints = 0;

    odom.header.frame_id = "/lidar_init";
    odom.child_frame_id = "/lidar_odom";

    odomTransform.frame_id_ = "/lidar_init";
    odomTransform.child_frame_id_ = "/lidar_odom";

    systemInitialized = false;

    transformCur[0] = 1;
    transformSum[0] = 1;

    for (int i = 1; i < 7; ++i)
    {
      transformCur[i] = 0;
      transformSum[i] = 0;
    }

    cornerResCount = 0;
    surfResCount = 0;

    // Load parameters from yaml
    if (!getParameter("/odometry/max_num_iter", numIter))
    {
      ROS_WARN("maximal iteration number of odometry optimization not set, use default value: 50");
      numIter = 60;
    }

    if (!getParameter("/odometry/iter_count", numOpt))
    {
      ROS_WARN("number of odometry optimization not set, use default value: 1");
      numOpt = 1;
    }

    if (!getParameter("/odometry/print_time", printTime))
    {
      ROS_WARN("Time print set to false");
      printTime = false;
    }

    if (!getParameter("/odometry/distortion", distortion))
    {
      ROS_WARN("distortion set to false");
      distortion = false;
    }

    if (!getParameter("/odometry/ground_truth_exists", groundTruthExists))
    {
      ROS_WARN("ground truth does not exists");
      groundTruthExists = false;
    }
  }

  // Transform the point to the beginning time stamp (from source to reference)
  void transformToStart(PointXYZI const *const pi, PointXYZI *const po)
  {
    // Compute the factor (t_{k,i} - t_k) / (t - t_k) <-  note that t - t_k = 0.1 (10Hz)
    double s = 1.0;
    if (distortion)
    {
      s = (pi->intensity - int(pi->intensity)) / scanPeriod;
    }

    // spherical linear interpolation for quaternion interpolation
    Eigen::Quaterniond quaternion(transformCur[0], transformCur[1], transformCur[2], transformCur[3]);
    Eigen::Vector3d transition(transformCur[4], transformCur[5], transformCur[6]);
    Eigen::Quaterniond interpolatedQuaternion = Eigen::Quaterniond::Identity().slerp(s, quaternion);
    Eigen::Vector3d interpolatedTransition = s * transition;
    Eigen::Vector3d pt(pi->x, pi->y, pi->z);
    Eigen::Vector3d transformedPt = interpolatedQuaternion * pt + interpolatedTransition;

    po->x = transformedPt.x();
    po->y = transformedPt.y();
    po->z = transformedPt.z();
    po->intensity = pi->intensity;
  }

  // Transform the point to the end time stamp (begin of the next sweep)
  void transformToEnd(PointXYZI const *const pi, PointXYZI *const po)
  {
    // First transform to start for undistortion
    PointXYZI ptPCL;
    transformToStart(pi, &ptPCL);

    Eigen::Quaterniond quaternion(transformCur[0], transformCur[1], transformCur[2], transformCur[3]);
    Eigen::Vector3d transition(transformCur[4], transformCur[5], transformCur[6]);
    Eigen::Vector3d PtToStart(ptPCL.x, ptPCL.y, ptPCL.z);
    Eigen::Vector3d ptTransformed = quaternion.inverse() * (PtToStart - transition);

    po->x = ptTransformed.x();
    po->y = ptTransformed.y();
    po->z = ptTransformed.z();
    // Time information no more needed
    po->intensity = int(pi->intensity);
  }

  void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& pointCloudIn)
  {
    // Since they have the same time stamp
    cloudHeader = pointCloudIn->header;
    timeNewSegCloud = cloudHeader.stamp.toSec();
    pcl::fromROSMsg(*pointCloudIn, *segmentedCloud);
    newSegCloud = true;
  }

  void laserCloudSharpHandler(const sensor_msgs::PointCloud2ConstPtr &pointCloudIn)
  {
    timeNewSharpPoints = pointCloudIn->header.stamp.toSec();
    pcl::fromROSMsg(*pointCloudIn, *cornerPointsSharp);
    newSharpPoints = true;
  }

  void laserCloudLessSharpHandler(const sensor_msgs::PointCloud2ConstPtr &pointCloudIn)
  {
    timeNewFlatPoints = pointCloudIn->header.stamp.toSec();
    pcl::fromROSMsg(*pointCloudIn, *cornerPointsLessSharp);
    newFlatPoints = true;
  }

  void laserCloudFlatHandler(const sensor_msgs::PointCloud2ConstPtr &pointCloudIn)
  {
    pcl::fromROSMsg(*pointCloudIn, *surfPointsFlat);
  }

  void laserCloudLessFlatHandler(const sensor_msgs::PointCloud2ConstPtr &pointCloudIn)
  {
    pcl::fromROSMsg(*pointCloudIn, *surfPointsLessFlat);
  }

  void checkInitialization()
  {
    pcl::PointCloud<PointXYZI>::Ptr tempCloud = cornerPointsLessSharp;
    cornerPointsLessSharp = laserCloudCornerLast;
    laserCloudCornerLast = tempCloud;

    tempCloud = surfPointsLessFlat;
    surfPointsLessFlat = laserCloudSurfLast;
    laserCloudSurfLast = tempCloud;

    kdTreeCornerLast->setInputCloud(laserCloudCornerLast);
    kdTreeSurfLast->setInputCloud(laserCloudSurfLast);

    numPtsCornerLast = laserCloudCornerLast->points.size();
    numPtsSurfLast = laserCloudSurfLast->points.size();

    sensor_msgs::PointCloud2 msgs;
    pcl::toROSMsg(*laserCloudCornerLast, msgs);
    msgs.header.stamp = cloudHeader.stamp;
    msgs.header.frame_id = "/base_link";
    pubCornerLast.publish(msgs);

    pcl::toROSMsg(*laserCloudSurfLast, msgs);
    msgs.header.stamp = cloudHeader.stamp;
    msgs.header.frame_id = "/base_link";
    pubSurfLast.publish(msgs);

    if (groundTruthExists)
    {
      geometry_msgs::PoseStamped poseInit;
      poseInit.header = cloudHeader;
      poseInit.pose.orientation.w = transformSum[0];
      poseInit.pose.orientation.x = transformSum[1];
      poseInit.pose.orientation.y = transformSum[2];
      poseInit.pose.orientation.z = transformSum[3];
      poseInit.pose.position.x = transformSum[4];
      poseInit.pose.position.y = transformSum[5];
      poseInit.pose.position.z = transformSum[6];
      groundTruth.header.stamp = cloudHeader.stamp;
      groundTruth.poses.push_back(poseInit);
      groundTruth.header.frame_id = "/lidar_init";
      pubGroundTruth.publish(groundTruth);

      path.header.stamp = cloudHeader.stamp;
      path.poses.push_back(poseInit);
      path.header.frame_id = "/lidar_init";
      pubPath.publish(path);

      tfListner.waitForTransform("world", "base_link", ros::Time(0), ros::Duration(1));
      tfListner.lookupTransform("world", "base_link", ros::Time(0), groundTruthInit);
    }

    systemInitialized = true;
  }

  // Find the (j, l) in the last frame to form the correspindence of i
  void findCorrespondingCornerFeatures()
  {
    int numPtsCornerSharp = cornerPointsSharp->points.size();
    cornerResCount = 0;

    // For each edge feature point i in the current frame
    for (int i = 0; i < numPtsCornerSharp; ++i)
    {
      transformToStart(&cornerPointsSharp->points[i], &pointSel);

      // Find the nearst point of cornerPointsSharp in laserCloudCornerLast using kd-tree
      // Looking for point j (the closest neighbor of i in the last frame)
      kdTreeCornerLast->nearestKSearch(pointSel, 1, pointSearchIdx, pointSearchSqDists);

      int closestPointIdx = -1, minPointIdx2 = -1;

      // If the point found near enough (5^2)
      if (pointSearchSqDists[0] < SqDistThreshold)
      {
        closestPointIdx = pointSearchIdx[0];

        // Get the scan index of point j
        int closestPointScan = int(laserCloudCornerLast->points[closestPointIdx].intensity);

        // Square distance
        double pointSqDist, minPointSqDist2 = SqDistThreshold;

        // Look for point l in the direction of increasing scan line
        // (the closest neighbor of i in the consecutive scan of point j)
        for (int j = closestPointIdx + 1; j < laserCloudCornerLast->points.size(); ++j)
        {
          // If the point is not in the consecutive scans, stop searching
          if (int(laserCloudCornerLast->points[j].intensity) > closestPointScan + 2.5)
          {
            break;
          }

          pointSqDist = (laserCloudCornerLast->points[j].x - pointSel.x) *
                        (laserCloudCornerLast->points[j].x - pointSel.x) +
                        (laserCloudCornerLast->points[j].y - pointSel.y) *
                        (laserCloudCornerLast->points[j].y - pointSel.y) +
                        (laserCloudCornerLast->points[j].z - pointSel.z) *
                        (laserCloudCornerLast->points[j].z - pointSel.z);

          // Make sure that the two points are not on the same laser
          if (int(laserCloudCornerLast->points[j].intensity) > closestPointScan)
          {
            // Update to get the closest point l
            if (pointSqDist < minPointSqDist2)
            {
              minPointSqDist2 = pointSqDist;
              minPointIdx2 = j;
            }
          }
        }
        // Look for point l in the direction of decreasing scan line
        for (int j = closestPointIdx - 1; j >= 0; --j)
        {
          if (int(laserCloudCornerLast->points[j].intensity) < closestPointScan - 2.5)
          {
            break;
          }

          pointSqDist = (laserCloudCornerLast->points[j].x - pointSel.x) *
                        (laserCloudCornerLast->points[j].x - pointSel.x) +
                        (laserCloudCornerLast->points[j].y - pointSel.y) *
                        (laserCloudCornerLast->points[j].y - pointSel.y) +
                        (laserCloudCornerLast->points[j].z - pointSel.z) *
                        (laserCloudCornerLast->points[j].z - pointSel.z);

          if (int(laserCloudCornerLast->points[j].intensity) < closestPointScan)
          {
            if (pointSqDist < minPointSqDist2)
            {
              minPointSqDist2 = pointSqDist;
              minPointIdx2 = j;
            }
          }
        }
      }

      // Get the two points j and l to form the corresponding edge line
      pointSearchCornerIdx1[i] = closestPointIdx;  // point j
      pointSearchCornerIdx2[i] = minPointIdx2;     // point l

      if (minPointIdx2 >= 0)
        ++cornerResCount;
    }
  }

  // Find the (j, l, k) in the last frame to form the correspindence of i
  void findCorrespondingSurfFeatures()
  {
    int numPtsSurfFlat = surfPointsFlat->points.size();
    surfResCount = 0;

    for (int i = 0; i < numPtsSurfFlat; ++i)
    {
      transformToStart(&surfPointsFlat->points[i], &pointSel);

      // Find the nearst point of surfPointsFlat in laserCloudSurfLast using kd-tree
      kdTreeSurfLast->nearestKSearch(pointSel, 1, pointSearchIdx, pointSearchSqDists);

      int closestPointIdx = -1, minPointIdx2 = -1, minPointIdx3 = -1;

      if (pointSearchSqDists[0] < SqDistThreshold)
      {
        closestPointIdx = pointSearchIdx[0];
        int closestPointScan = int(laserCloudSurfLast->points[closestPointIdx].intensity);

        double pointSqDist, minPointSqDist2 = SqDistThreshold, minPointSqDist3 = SqDistThreshold;

        // Look for point l and k in the direction of increasing scan line
        for (int j = closestPointIdx + 1; j < laserCloudSurfLast->points.size(); ++j)
        {
          if (int(laserCloudSurfLast->points[j].intensity) > closestPointScan + 2.5)
          {
            break;
          }

          pointSqDist = (laserCloudSurfLast->points[j].x - pointSel.x) *
                        (laserCloudSurfLast->points[j].x - pointSel.x) +
                        (laserCloudSurfLast->points[j].y - pointSel.y) *
                        (laserCloudSurfLast->points[j].y - pointSel.y) +
                        (laserCloudSurfLast->points[j].z - pointSel.z) *
                        (laserCloudSurfLast->points[j].z - pointSel.z);

          // If the point is in the same scan (with the same scan index) - point l
          if (int(laserCloudSurfLast->points[j].intensity) <= closestPointScan && pointSqDist < minPointSqDist2)
          {
            minPointSqDist2 = pointSqDist;
            minPointIdx2 = j;
          }
          else if (int(laserCloudSurfLast->points[j].intensity) > closestPointScan && pointSqDist < minPointSqDist3)
          {
            minPointSqDist3 = pointSqDist;
            minPointIdx3 = j;
          }
        }

        for (int j = closestPointIdx - 1; j >= 0; --j)
        {
          if (int(laserCloudSurfLast->points[j].intensity) < closestPointScan - 2.5)
          {
            break;
          }

          pointSqDist = (laserCloudSurfLast->points[j].x - pointSel.x) *
                        (laserCloudSurfLast->points[j].x - pointSel.x) +
                        (laserCloudSurfLast->points[j].y - pointSel.y) *
                        (laserCloudSurfLast->points[j].y - pointSel.y) +
                        (laserCloudSurfLast->points[j].z - pointSel.z) *
                        (laserCloudSurfLast->points[j].z - pointSel.z);

          if (int(laserCloudSurfLast->points[j].intensity) >= closestPointScan && pointSqDist < minPointSqDist2)
          {
            minPointSqDist2 = pointSqDist;
            minPointIdx2 = j;
          }
          else if (int(laserCloudSurfLast->points[j].intensity) < closestPointScan && pointSqDist < minPointSqDist3)
          {
            minPointSqDist3 = pointSqDist;
            minPointIdx3 = j;
          }
        }
      }

      pointSearchSurfIdx1[i] = closestPointIdx;
      pointSearchSurfIdx2[i] = minPointIdx2;
      pointSearchSurfIdx3[i] = minPointIdx3;

      if (minPointIdx2 >= 0 && minPointIdx3 >= 0)
        ++surfResCount;
    }
  }

  void updateTransformationWithCeres()
  {
    // Make sure there is enough feature points in the sweep
    if (numPtsCornerLast < 10 || numPtsSurfLast < 100)
      return;

    for (int iterCount = 0; iterCount < numOpt; ++iterCount)
    {
      ceres::LossFunction *lossFunction = new ceres::HuberLoss(0.1);
      ceres::LocalParameterization *quatParameterization = new ceres:: QuaternionParameterization();
      ceres::Problem::Options problemOptions;

      ceres::Problem problem(problemOptions);
      problem.AddParameterBlock(transformCur, 4, quatParameterization);
      problem.AddParameterBlock(transformCur + 4, 3);

      findCorrespondingCornerFeatures();
      findCorrespondingSurfFeatures();

      int corResCnt = 0, surfResCnt = 0;

      for (int i = 0; i < cornerPointsSharp->points.size(); ++i)
      {
        if (pointSearchCornerIdx2[i] >= 0)
        {
          Eigen::Vector3d currentPt(cornerPointsSharp->points[i].x,
                                    cornerPointsSharp->points[i].y,
                                    cornerPointsSharp->points[i].z);
          Eigen::Vector3d lastPtJ(laserCloudCornerLast->points[pointSearchCornerIdx1[i]].x,
                                  laserCloudCornerLast->points[pointSearchCornerIdx1[i]].y,
                                  laserCloudCornerLast->points[pointSearchCornerIdx1[i]].z);
          Eigen::Vector3d lastPtL(laserCloudCornerLast->points[pointSearchCornerIdx2[i]].x,
                                  laserCloudCornerLast->points[pointSearchCornerIdx2[i]].y,
                                  laserCloudCornerLast->points[pointSearchCornerIdx2[i]].z);

          double weight = 1.0;
          if (distortion)
            weight = double((cornerPointsSharp->points[i].intensity - int(cornerPointsSharp->points[i].intensity)) / scanPeriod);

          // Use automatic derivatives
//          ceres::CostFunction *costFunction = LidarEdgeFactor::Create(currentPt, lastPtJ, lastPtL, weight);
          // Use analytic derivatives
          LidarEdgeFactorAnalytic *costFunction = new LidarEdgeFactorAnalytic(currentPt, lastPtJ, lastPtL, weight);
          problem.AddResidualBlock(costFunction, lossFunction, transformCur, transformCur + 4);
          ++corResCnt;
        }
      }

      for (int i = 0; i < surfPointsFlat->points.size(); ++i)
      {
        if (pointSearchSurfIdx2[i] >= 0 && pointSearchSurfIdx3[i] >= 0)
        {
          Eigen::Vector3d currentPt(surfPointsFlat->points[i].x,
                                    surfPointsFlat->points[i].y,
                                    surfPointsFlat->points[i].z);
          Eigen::Vector3d lastPtJ(laserCloudSurfLast->points[pointSearchSurfIdx1[i]].x,
                                  laserCloudSurfLast->points[pointSearchSurfIdx1[i]].y,
                                  laserCloudSurfLast->points[pointSearchSurfIdx1[i]].z);
          Eigen::Vector3d lastPtL(laserCloudSurfLast->points[pointSearchSurfIdx2[i]].x,
                                  laserCloudSurfLast->points[pointSearchSurfIdx2[i]].y,
                                  laserCloudSurfLast->points[pointSearchSurfIdx2[i]].z);
          Eigen::Vector3d lastPtM(laserCloudSurfLast->points[pointSearchSurfIdx3[i]].x,
                                  laserCloudSurfLast->points[pointSearchSurfIdx3[i]].y,
                                  laserCloudSurfLast->points[pointSearchSurfIdx3[i]].z);

          double weight = 1.0;
          if (distortion)
            weight = double((surfPointsFlat->points[i].intensity - int(surfPointsFlat->points[i].intensity)) / scanPeriod);

          // Use automatic derivatives
//          ceres::CostFunction *costFunction = LidarPlaneFactor::Create(currentPt, lastPtJ, lastPtL, lastPtM, weight);
          // Use analytic derivatives
          LidarPlaneFactorAnalytic *costFunction = new LidarPlaneFactorAnalytic(currentPt, lastPtJ, lastPtL, lastPtM, weight);
          problem.AddResidualBlock(costFunction, lossFunction, transformCur, transformCur + 4);
          ++surfResCnt;
        }
      }

//      ROS_INFO_STREAM("Number of corner residuals: " << corResCnt << ", Number of surf residuals: " << surfResCnt);

      ceres::Solver::Options solverOptions;
      solverOptions.linear_solver_type = ceres::DENSE_QR;
      solverOptions.max_num_iterations = numIter;
      solverOptions.minimizer_progress_to_stdout = false;
      solverOptions.check_gradients = false;
      solverOptions.gradient_check_relative_precision = 1e-4;

      ceres::Solver::Summary summary;
      ceres::Solve(solverOptions, &problem, &summary);
//      std::cout << summary.BriefReport() << "\n";
    }

  }

  void updateTransformationWithROPTLIB()
  {
    if (numPtsCornerLast < 10 || numPtsSurfLast < 100)
      return;

    findCorrespondingCornerFeatures();
    findCorrespondingSurfFeatures();

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> cornerMat;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> surfMat;
    cornerMat.setZero(10, cornerResCount);
    surfMat.setZero(13, surfResCount);

    for (int iterCount = 0; iterCount < numOpt; ++iterCount)
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

      spherePtr[0] = transformCur[0];
      spherePtr[1] = transformCur[1];
      spherePtr[2] = transformCur[2];
      spherePtr[3] = transformCur[3];
      eucPtr[0] = transformCur[4];
      eucPtr[1] = transformCur[5];
      eucPtr[2] = transformCur[6];

      // Define the manifolds for SE(3)
      ROPTLIB::Sphere sphereMani(dimOfRot);
      sphereMani.ChooseSphereParamsSet2();
      ROPTLIB::Euclidean EucMani(dimOfTrans);
      ROPTLIB::ProductManifold Domain(2, &sphereMani, 1, &EucMani, 1);


      int col = 0;
      for (int i = 0; i < cornerPointsSharp->points.size(); ++i)
      {
        if (pointSearchCornerIdx2[i] >= 0)
        {
          Eigen::Vector3d currentPt(cornerPointsSharp->points[i].x,
                                    cornerPointsSharp->points[i].y,
                                    cornerPointsSharp->points[i].z);
          Eigen::Vector3d lastPtJ(laserCloudCornerLast->points[pointSearchCornerIdx1[i]].x,
                                  laserCloudCornerLast->points[pointSearchCornerIdx1[i]].y,
                                  laserCloudCornerLast->points[pointSearchCornerIdx1[i]].z);
          Eigen::Vector3d lastPtL(laserCloudCornerLast->points[pointSearchCornerIdx2[i]].x,
                                  laserCloudCornerLast->points[pointSearchCornerIdx2[i]].y,
                                  laserCloudCornerLast->points[pointSearchCornerIdx2[i]].z);

          double weight = 1.0;
          if (distortion)
            weight = double((cornerPointsSharp->points[i].intensity - int(cornerPointsSharp->points[i].intensity)) / scanPeriod);

          cornerMat.block<3, 1>(0, col) = currentPt;
          cornerMat.block<3, 1>(3, col) = lastPtJ;
          cornerMat.block<3, 1>(6, col) = lastPtL;
          cornerMat(9, col) = weight;

          ++col;
        }
      }

      col = 0;
      for (int i = 0; i < surfPointsFlat->points.size(); ++i)
      {

        if (pointSearchSurfIdx2[i] >= 0 && pointSearchSurfIdx3[i] >= 0)
        {
          Eigen::Vector3d currentPt(surfPointsFlat->points[i].x,
                                    surfPointsFlat->points[i].y,
                                    surfPointsFlat->points[i].z);
          Eigen::Vector3d lastPtJ(laserCloudSurfLast->points[pointSearchSurfIdx1[i]].x,
                                  laserCloudSurfLast->points[pointSearchSurfIdx1[i]].y,
                                  laserCloudSurfLast->points[pointSearchSurfIdx1[i]].z);
          Eigen::Vector3d lastPtL(laserCloudSurfLast->points[pointSearchSurfIdx2[i]].x,
                                  laserCloudSurfLast->points[pointSearchSurfIdx2[i]].y,
                                  laserCloudSurfLast->points[pointSearchSurfIdx2[i]].z);
          Eigen::Vector3d lastPtM(laserCloudSurfLast->points[pointSearchSurfIdx3[i]].x,
                                  laserCloudSurfLast->points[pointSearchSurfIdx3[i]].y,
                                  laserCloudSurfLast->points[pointSearchSurfIdx3[i]].z);

          double weight = 1.0;
          if (distortion)
            weight = double((surfPointsFlat->points[i].intensity - int(surfPointsFlat->points[i].intensity)) / scanPeriod);

          surfMat.block<3, 1>(0, col) = currentPt;
          surfMat.block<3, 1>(3, col) = lastPtJ;
          surfMat.block<3, 1>(6, col) = lastPtL;
          surfMat.block<3, 1>(9, col) = lastPtM;
          surfMat(12, col) = weight;

          ++col;
        }
      }

      // Define the problem
      odomOptProblem Prob(cornerMat, surfMat, cornerResCount, surfResCount);

      // Set the domain of the problem to be the product manifold
      Prob.SetDomain(&Domain);

      // Check gradient and Hessian
//      Prob.CheckGradHessian(&SE3Iterate);

      // Output the parameters of the manifold of domain
//      Domain.CheckParams();

      //  enum class lineSearchOptimizer{ RCG, RSD, RBFGS, RBFGS, RWRBFGS, RBroydenFamily};
      //  enum class trustRigionOptimizer{ RTRSD, RTRNewton, RTRSR1, LRTRSR1 };
      // Apply the Riemannian steepest descent
      ROPTLIB::RSD *solver = new ROPTLIB::RSD(&Prob, &SE3Iterate);

      // Set the Line Search algorithm
//      RSDsolver->LineSearch_LS = ROPTLIB::ARMIJO  ; // Back tracking
      // The Armijo-Goldstein condition [DS83 Algorithm A6.3.1]
      // RSDsolver->LineSearch_LS = static_cast<ROPTLIB::LSAlgo> (0); // Another method
      // enum LSAlgo{ ARMIJO, WOLFE, STRONGWOLFE, EXACT, WOLFELP, INPUTFUN, LSALGOLENGTH };

//      solver->Stop_Criterion = ROPTLIB::FUN_REL;  // (f (xi−1) − f (xi ))/f (xi)
//      solver->Tolerance = 1e-10;
      solver->Max_Iteration = numIter;
//      solver->Minstepsize = 1e-2;
//      solver->OutputGap = 5;
//      solver->TimeBound = 0.1;

      // Specify what information will be output in the algorithm.
      solver->Debug = ROPTLIB::NOOUTPUT;
      // enum DEBUGINFO{ NOOUTPUT, FINALRESULT, ITERRESULT, DETAILED, DEBUGLENGTH };

//      RSDsolver->CheckParams();
      solver->Run();

      // Obtain the optimized iterates
      const ROPTLIB::Element *xOpt = SE3Iterate.ConstructEmpty();
      xOpt = solver->GetXopt();
//      xOpt->Print("Current transform");

      const double *optPtr = xOpt->ObtainReadData();

      transformCur[0] = optPtr[0];
      transformCur[1] = optPtr[1];
      transformCur[2] = optPtr[2];
      transformCur[3] = optPtr[3];
      transformCur[4] = optPtr[4];
      transformCur[5] = optPtr[5];
      transformCur[6] = optPtr[6];
    }

  }

  void integrateTransformation()
  {
    Eigen::Quaterniond incrementalQuaternion(transformCur[0], transformCur[1], transformCur[2], transformCur[3]);
    Eigen::Vector3d incrementalTransition(transformCur[4], transformCur[5], transformCur[6]);

    Eigen::Quaterniond integratedQuaternion(transformSum[0], transformSum[1], transformSum[2], transformSum[3]);
    Eigen::Vector3d integratedTransition(transformSum[4], transformSum[5], transformSum[6]);

    integratedTransition = integratedTransition + integratedQuaternion * incrementalTransition;
    integratedQuaternion = integratedQuaternion * incrementalQuaternion;

    transformSum[0] = integratedQuaternion.w();
    transformSum[1] = integratedQuaternion.x();
    transformSum[2] = integratedQuaternion.y();
    transformSum[3] = integratedQuaternion.z();
    transformSum[4] = integratedTransition.x();
    transformSum[5] = integratedTransition.y();
    transformSum[6] = integratedTransition.z();
  }

  void publishOdometry()
  {
    odom.header.stamp = cloudHeader.stamp;
    odom.pose.pose.orientation.w = transformSum[0];
    odom.pose.pose.orientation.x = transformSum[1];
    odom.pose.pose.orientation.y = transformSum[2];
    odom.pose.pose.orientation.z = transformSum[3];
    odom.pose.pose.position.x = transformSum[4];
    odom.pose.pose.position.y = transformSum[5];
    odom.pose.pose.position.z = transformSum[6];
    pubOdometry.publish(odom);

    geometry_msgs::PoseStamped poseStamped;
    poseStamped.header = odom.header;
    poseStamped.pose = odom.pose.pose;
    poseStamped.header.stamp = odom.header.stamp;
    path.header.stamp = odom.header.stamp;
    path.poses.push_back(poseStamped);
    path.header.frame_id = "/lidar_init";
    pubPath.publish(path);

    if (groundTruthExists)
    {
      tfListner.waitForTransform("world", "base_link", ros::Time(0), ros::Duration(1));
      tfListner.lookupTransform("world", "base_link", ros::Time(0), groundTruthTransform);

    poseStamped.header = odom.header;
    poseStamped.pose.orientation.x = groundTruthInit.inverseTimes(groundTruthTransform).getRotation().x();
    poseStamped.pose.orientation.y = groundTruthInit.inverseTimes(groundTruthTransform).getRotation().y();
    poseStamped.pose.orientation.z = groundTruthInit.inverseTimes(groundTruthTransform).getRotation().z();
    poseStamped.pose.orientation.w = groundTruthInit.inverseTimes(groundTruthTransform).getRotation().w();
    poseStamped.pose.position.x = groundTruthInit.inverseTimes(groundTruthTransform).getOrigin().x();
    poseStamped.pose.position.y = groundTruthInit.inverseTimes(groundTruthTransform).getOrigin().y();
    poseStamped.pose.position.z = groundTruthInit.inverseTimes(groundTruthTransform).getOrigin().z();
    groundTruth.header.stamp = odom.header.stamp;
    groundTruth.poses.push_back(poseStamped);
    groundTruth.header.frame_id = "/lidar_init";
    pubGroundTruth.publish(groundTruth);
    }

    odomTransform.stamp_ = cloudHeader.stamp;
    odomTransform.setRotation(tf::Quaternion(transformSum[1], transformSum[2], transformSum[3], transformSum[0]));
    odomTransform.setOrigin(tf::Vector3(transformSum[4], transformSum[5], transformSum[6]));
    tfBroadcaster.sendTransform(odomTransform);
  }

  void publishCloudLast()
  {
    for (int i = 0; i < cornerPointsLessSharp->points.size(); ++i)
    {
      transformToEnd(&cornerPointsLessSharp->points[i], &cornerPointsLessSharp->points[i]);
    }

    for (int i = 0; i < surfPointsLessFlat->points.size(); ++i)
    {
      transformToEnd(&surfPointsLessFlat->points[i], &surfPointsLessFlat->points[i]);
    }

    for (int i = 0; i < segmentedCloud->points.size(); ++i)
    {
      transformToEnd(&segmentedCloud->points[i], &segmentedCloud->points[i]);
    }

    pcl::PointCloud<PointXYZI>::Ptr tempCloud = cornerPointsLessSharp;
    cornerPointsLessSharp = laserCloudCornerLast;
    laserCloudCornerLast = tempCloud;

    tempCloud = surfPointsLessFlat;
    surfPointsLessFlat = laserCloudSurfLast;
    laserCloudSurfLast = tempCloud;

    numPtsCornerLast = laserCloudCornerLast->points.size();
    numPtsSurfLast = laserCloudSurfLast->points.size();

    if (numPtsCornerLast > 10 && numPtsSurfLast > 100)
    {
      kdTreeCornerLast->setInputCloud(laserCloudCornerLast);
      kdTreeSurfLast->setInputCloud(laserCloudSurfLast);
    }

    ++frameCount;

    if (frameCount >= skipFrameNum + 1)
    {
      frameCount = 0;
      sensor_msgs::PointCloud2 msgs;

      pcl::toROSMsg(*segmentedCloud, msgs);
      msgs.header.stamp = cloudHeader.stamp;
      msgs.header.frame_id = "/lidar";
      pubSegmentedCloud.publish(msgs);

      pcl::toROSMsg(*laserCloudCornerLast, msgs);
      msgs.header.stamp = cloudHeader.stamp;
      msgs.header.frame_id = "/lidar";
      pubCornerLast.publish(msgs);

      pcl::toROSMsg(*laserCloudSurfLast, msgs);
      msgs.header.stamp = cloudHeader.stamp;
      msgs.header.frame_id = "/lidar";
      pubSurfLast.publish(msgs);
    }
  }

  void run()
  {
    if (newSegCloud && newSharpPoints && newFlatPoints &&
        std::abs(timeNewSegCloud - timeNewSharpPoints) < 0.05 &&
        std::abs(timeNewSegCloud - timeNewFlatPoints) < 0.05)
    {
      newSegCloud = false;
      newSharpPoints = false;
      newFlatPoints = false;
    }
    else
    {
      return;
    }

    if (!systemInitialized)
    {
      checkInitialization();
      return;
    }

    updateTransformationWithCeres();
//    updateTransformationWithROPTLIB();
    integrateTransformation();
    publishOdometry();
    publishCloudLast();
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test");

  LidarOdometry lo;

  ROS_INFO("\033[1;32m---->\033[0m Lidar Odometry Started.");

  ros::Rate rate(200);

  while (ros::ok())
  {
    ros::spinOnce();
    lo.run();
    rate.sleep();
  }

  ros::spin();
  return 0;
}
