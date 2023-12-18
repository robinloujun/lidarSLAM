#include <lidar_directional_slam/lidarmapping.h>
#include <lidar_directional_slam/utils/parameter_utils.h>
#include <lidar_directional_slam/utils/geometry_utils.h>
#include <lidar_directional_slam/utils/pose_graph.h>

struct BoolSetter
{
public:
  bool toSetValue;
  BoolSetter(bool& target, bool toSetValue)
      : toSetValue(toSetValue), target(target)
  {
  }
  ~BoolSetter() { target = toSetValue; }

protected:
  bool& target;
};

LidarMapping::LidarMapping(ros::NodeHandle& nh)
    : PointCloudMap_(0),
      transformation_(
          PM::get().REG(Transformation).create("RigidTransformation")),
      T_odom2map_(TransformationParameters::Identity(4, 4)),
      T_local2map_(TransformationParameters::Identity(4, 4)),
      T_sensor2odom_(TransformationParameters::Identity(4, 4)),
      T_frame2frame_(TransformationParameters::Identity(4, 4)),
      T_sensor2map_(TransformationParameters::Identity(4, 4)),
      tfListener_(ros::Duration(30)), realtime_(false), initialized_(false),
      processingPointCloud_(false), registInit_(false),
      processingMapBuilding_(false), nodeIdx_(0)
{
  nodeName_ = ros::names::append(nh.getNamespace(), "LidarMapping");

  subPointCloud_ = nh.subscribe("/velodyne_points", 2,
                                &LidarMapping::lidarPointCloudHandler, this);

  pubMap_ = nh.advertise<sensor_msgs::PointCloud2>("point_map", 2);

  pubCloud_ = nh.advertise<sensor_msgs::PointCloud2>("point_cloud", 2);

  pubCloudLast_ = nh.advertise<sensor_msgs::PointCloud2>("point_cloud_last", 2);

  pubOdom_ = nh.advertise<nav_msgs::Odometry>("icp_odometry", 50);

  pubOdomError_ = nh.advertise<nav_msgs::Odometry>("icp_error_odometry", 50);

  //  pubTruePath_ = nh.advertise<nav_msgs::Path>("ground_truth_path", 50);

  pubMapInLocal_ = nh.advertise<sensor_msgs::PointCloud2>("map_in_local", 2);

  pubCloudInGlobal_ =
      nh.advertise<sensor_msgs::PointCloud2>("point_cloud_in_global", 2);

  publishThread_ = boost::thread(
      boost::bind(&LidarMapping::publishLoop, this, tfRefreshPeriod_));

  pubPathEstimate_ = nh.advertise<nav_msgs::Path>("estimated_path", 50, true);

  pubPathGroundTruth_ = nh.advertise<nav_msgs::Path>("ground_truth", 50, true);
}

LidarMapping::~LidarMapping()
{
  //  if (processingMapBuilding_)
  //  {
  //    mapBuildingFuture_.wait();
  //    if (mapBuildingFuture_.has_value())
  //      delete mapBuildingFuture_.get();
  //  }
  //  publishThread_.join();
}

bool LidarMapping::initialize()
{
  ROS_INFO("Initializing the Node LidarMapping ...");
  //  ROS_INFO_STREAM("Version of boost: " << BOOST_VERSION);

  if (!loadParameters())
  {
    ROS_ERROR("%s: Failed to load parameters.", nodeName_.c_str());
    return false;
  }

  initialized_ = true;
  return true;
}

bool LidarMapping::loadParameters()
{
  // Load frame ids.
  if (!parameter_utils::Get("/General/world_frame", world_frame_id_))
    return false;
  if (!parameter_utils::Get("/General/odom_frame", odometry_frame_id_))
    return false;
  if (!parameter_utils::Get("/General/ground_truth_frame",
                            ground_truth_frame_id_))
    return false;
  if (!parameter_utils::Get("/General/map_frame", map_frame_id_))
    return false;
  if (!parameter_utils::Get("/General/sensor_frame", sensor_frame_id_))
    return false;

  // load YAML configuration for input filter
  string configFileName;

  if (ros::param::get("~inputfilter", configFileName))
  {
    ifstream ifs(configFileName.c_str());
    if (ifs.good())
    {
      inputFilters_ = DataPointsFilters(ifs);
    }
    else
    {
      ROS_ERROR_STREAM("Cannot load filter config from YAML file "
                       << configFileName);
    }
  }
  else
  {
    ROS_INFO_STREAM("Found no config file for the input filter");
  }

  // load YAML configuration for premapping filter
  if (ros::param::get("~premappingfilter", configFileName))
  {
    ifstream ifs(configFileName.c_str());
    if (ifs.good())
    {
      preMapFilters_ = DataPointsFilters(ifs);
    }
    else
    {
      ROS_ERROR_STREAM("Cannot load filter config from YAML file "
                       << configFileName);
    }
  }
  else
  {
    ROS_INFO_STREAM("Found no config file for the premapping filter");
  }

  // load YAML configuration for postmapping filter
  if (ros::param::get("~postmappingfilter", configFileName))
  {
    ifstream ifs(configFileName.c_str());
    if (ifs.good())
    {
      postMapFilters_ = DataPointsFilters(ifs);
    }
    else
    {
      ROS_ERROR_STREAM("Cannot load filter config from YAML file "
                       << configFileName);
    }
  }
  else
  {
    ROS_INFO_STREAM("Found no config file for the postmapping filter");
  }

  // load YAML configuration for icp
  if (ros::param::get("~icp_odom", configFileName))
  {
    ifstream ifs(configFileName.c_str());
    if (ifs.good())
    {
      icpOdom_.loadFromYaml(ifs);
    }
    else
    {
      ROS_ERROR_STREAM("Cannot load ICP config from YAML file "
                       << configFileName);
      icpOdom_.setDefault();
    }
  }
  else
  {
    ROS_INFO_STREAM("No ICP config file given, using default");
    icpOdom_.setDefault();
  }

  if (ros::param::get("~icp_map", configFileName))
  {
    ifstream ifs(configFileName.c_str());
    if (ifs.good())
    {
      icpMapping_.loadFromYaml(ifs);
    }
    else
    {
      ROS_ERROR_STREAM("Cannot load ICP config from YAML file "
                       << configFileName);
      icpMapping_.setDefault();
    }
  }
  else
  {
    ROS_INFO_STREAM("No ICP config file given, using default");
    icpMapping_.setDefault();
  }

  // load configuration for local map filter
  Parameters params;
  params["dim"] = "-1";
  params["maxDist"] = PointMatcherSupport::toParam(localMapRange_);

  localMapFilter_ = PM::get().DataPointsFilterRegistrar.create(
      "MaxDistDataPointsFilter", params);

  return true;
}

void LidarMapping::lidarPointCloudHandler(
    const sensor_msgs::PointCloud2ConstPtr& lidarPointCloudIn)
{

  // Convert the point cloud from the form of PointCloud2 to PM::DataPoints
  unique_ptr<DP> cloud(new DP(
      PointMatcher_ros::rosMsgToPointMatcherCloud<float>(*lidarPointCloudIn)));

  // Get the ground truth transform from /tf
  tfListener_.waitForTransform(world_frame_id_, ground_truth_frame_id_,
                               ros::Time(0), ros::Duration(1));
  tfListener_.lookupTransform(world_frame_id_, ground_truth_frame_id_,
                              ros::Time(0), tfGroundTruthCur_);

  //  pubCloud_.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(
  //      *cloud, map_frame_id_, ros::Time::now()));

  ROS_INFO_STREAM("Number of points (original): " << cloud->getNbPoints());

  processCloud(move(cloud), lidarPointCloudIn->header.stamp,
               lidarPointCloudIn->header.seq);

  /* Using container pointCloudIntensity defined in utils/basic.h
    mapping_utils::pointCloudIntensity pointcloud;

    // Using iterator to converse the x, y ,z ,i to mapping_utils format
    for (sensor_msgs::PointCloud2ConstIterator<float> iter(*lidarPointCloudIn,
    "x");
         iter != iter.end(); ++iter)
    {
      if (!std::isfinite(iter[0]) ||
          !std::isfinite(iter[1]) ||
          !std::isfinite(iter[2]))
      {
        continue;
      }
      pointcloud.push_back(mapping_utils::pointI(iter[0], iter[1], iter[2],
    iter[3]));
    }
  */

  //  ROS_INFO("Integrating a point cloud with %lu points.",
  //  pointcloud.getNumPoints());

  // Display the point coordinates
  //    for (auto &c : pointcloud)
  //      cout << c << endl;

  /*
  // Get the ICP-based transform from /tf
  tfListener_.waitForTransform(world_frame_id_, odometry_frame_id_,
                               ros::Time(0), ros::Duration(10));
  tfListener_.lookupTransform(world_frame_id_, odometry_frame_id_,
                              ros::Time(0), tfCurrent_);

  Eigen::Matrix4f T_world;
  T_world = geometry_utils::toEigenMatrix<float>(tfCurrent_);

  ROS_INFO_STREAM("get the tf information: " << tfCurrent_.getOrigin().x() << ",
  " << tfCurrent_.getOrigin().y() << ", " << tfCurrent_.getOrigin().z());

  //  ROS_INFO_STREAM("Transformation matrix from /tf: " << endl << T_world);
  */
}

void LidarMapping::processCloud(DPPtr cloud, const ros::Time& stamp,
                                uint32_t seq)
{
  // Set the state value for point cloud processing
  processingPointCloud_ = true;
  BoolSetter stopProcessingSetter(processingPointCloud_, false);

  // Check the number of points in incoming point cloud
  const size_t getCount(cloud->features.cols());
  if (getCount == 0)
  {
    ROS_ERROR("There is no point in the point cloud");
    return;
  }

  // Init the timer for the process
  timer timer;

  // Apply the input filter for the incoming point cloud
  inputFilters_.apply(*cloud);
  ROS_INFO_STREAM("Filtering the points took " << timer.elapsed() << " s. "
                                               << cloud->features.cols()
                                               << " points left.");

  // If there is no map, initialize the map first
  if (!icpMapping_.hasMap())
  {
    ROS_INFO("[Initialization stage]");
    publishLock_.lock();
    //    T_odom2map_ = PM::TransformationParameters::Identity(4, 4);
    T_local2map_ = TransformationParameters::Identity(4, 4);
    //    T_sensor2odom_ = PM::TransformationParameters::Identity(4, 4);
    //    T_frame2frame_ = PM::TransformationParameters::Identity(4, 4);
    T_sensor2map_ = TransformationParameters::Identity(4, 4);
    publishLock_.unlock();

    // Initialize the map
    mapCreationStamp_ = stamp;
    // Besides update the map, get the T_local2map from last T_sensor2map
    //    setMap(updateMap(cloud, TransformationParameters::Identity(4, 4),
    //    false));

    addNodeLock_.lock();
    graph_.addNode(nodeIdx_, T_sensor2map_, cloud);
    nodeIdx_++;
    addNodeLock_.unlock();

    icpMapLock_.lock();

    // Set the first point cloud as the reference for ICP_mapping
    icpMapping_.setMap(*cloud);
    ROS_INFO_STREAM("Set the first point cloud for ICP_mapping with "
                    << cloud->getNbPoints() << " pts.");
    icpMapLock_.unlock();

    //    ROS_INFO_STREAM("T_local2map = " << endl
    //                                     << T_local2map_);
    //    ROS_INFO_STREAM("T_sensor2map = " << endl
    //                                      << T_sensor2map_);

    tfGroundTruthFirst_ = tfGroundTruthCur_;

    return;
  }
  else
  {
    //    T_odom2map_ = transformation_->correctParameters(T_odom2map_);
    T_local2map_ = transformation_->correctParameters(T_local2map_);
    //    T_sensor2odom_ = transformation_->correctParameters(T_sensor2odom_);
    //    T_frame2frame_ = transformation_->correctParameters(T_frame2frame_);
    T_sensor2map_ = transformation_->correctParameters(T_sensor2map_);
  }

  //  const PM::TransformationParameters T_sensor2map =
  //      transformation_->correctParameters(T_odom2map_ * T_sensor2odom_);

  // Pirnt the transformation matrix
  {
    //    ROS_INFO_STREAM(endl
    //                    << "T_odom_to_sensor(" << odometry_frame_id_ << " to "
    //                    << sensor_frame_id_ << "):\n" << T_odom2sensor_);
    //    ROS_INFO_STREAM(endl
    //                    << "T_odom_to_map(" << odometry_frame_id_ << " to "
    //                    << map_frame_id_ << "):\n" << T_odom2map_);
    //    ROS_INFO_STREAM(endl
    //                    << "T_sensor_to_map (" << sensor_frame_id_ << " to "
    //                    << map_frame_id_ << "):\n" << T_sensor2map);
  }

  //  const PM::TransformationParameters T_sensor2local =
  //      transformation_->correctParameters(T_local2map_.inverse() *
  //      T_sensor2map);

  // Check the dimension of the incoming cloud and the map
  if (cloud->getEuclideanDim() !=
      icpMapping_.getPrefilteredInternalMap().getEuclideanDim())
  {
    ROS_ERROR_STREAM(
        "Dimensionality dismatch: point cloud is "
        << cloud->getEuclideanDim() << "d, while the map is "
        << icpMapping_.getPrefilteredInternalMap().getEuclideanDim() << "d.");
    return;
  }

  // Convert transform between fixed frame and ground_truth frame.
  geometry_msgs::TransformStamped tfGroundTruth;
  tf::Transform transformGroundTruth =
      tfGroundTruthFirst_.inverseTimes(tfGroundTruthCur_);

  // Visualize the Ground Truth Trajectory
  trajectoryGroundTruth_.header.stamp = ros::Time::now();
  trajectoryGroundTruth_.header.frame_id = map_frame_id_;
  geometry_msgs::PoseStamped poseGroundTruth;
  poseGroundTruth.pose.position.x = transformGroundTruth.getOrigin().x();
  poseGroundTruth.pose.position.y = transformGroundTruth.getOrigin().y();
  poseGroundTruth.pose.position.z = transformGroundTruth.getOrigin().z();
  poseGroundTruth.pose.orientation.x = transformGroundTruth.getRotation().x();
  poseGroundTruth.pose.orientation.y = transformGroundTruth.getRotation().y();
  poseGroundTruth.pose.orientation.z = transformGroundTruth.getRotation().z();
  poseGroundTruth.pose.orientation.w = transformGroundTruth.getRotation().w();
  trajectoryGroundTruth_.poses.push_back(poseGroundTruth);

  pubPathGroundTruth_.publish(trajectoryGroundTruth_);

  try
  {
    // Apply frame-to-model ICP
    TransformationParameters T_sensor2local =
        transformation_->correctParameters(T_local2map_.inverse() *
                                           T_sensor2map_);

    TransformationParameters T_sensor2map_updated;
    TransformationParameters T_local2map_updated;
    TransformationParameters T_sensor2local_updated;

    ROS_INFO_STREAM("\nNumber of points in Reading: "
                    << cloud->getNbPoints() << endl
                    << "Number of points in Reference: "
                    << icpMapping_.getPrefilteredInternalMap().getNbPoints());

    // Apply frame-to-model ICP
    // Reading: current point cloud
    // Reference: local map from the global map
    // (local means the pose at last frame)
    icpMapLock_.lock();
    T_sensor2local_updated = icpMapping_(*cloud, T_sensor2local);
    icpMapLock_.unlock();

    ROS_INFO_STREAM("Length of the pose graph: " << graph_.getNumNodes());

    T_local2map_updated = T_sensor2map_;

    T_sensor2map_updated = transformation_->correctParameters(
        T_local2map_updated * T_sensor2local_updated);

    ROS_INFO("[after ICP]");
    ROS_INFO_STREAM("T_local2map_updated:" << endl
                                           << T_local2map_updated);
    ROS_INFO_STREAM("T_sensor2map_updated:" << endl
                                            << T_sensor2map_updated);

    // add the pose and the point cloud to the graph
    addNodeLock_.lock();
    graph_.addNode(nodeIdx_, T_sensor2map_updated, cloud);
    nodeIdx_++;
    addNodeLock_.unlock();

    DP cloudInGlobal = transformation_->compute(*cloud, T_sensor2map_updated);

    pubCloudInGlobal_.publish(
        PointMatcher_ros::pointMatcherCloudToRosMsg<float>(
            cloudInGlobal, map_frame_id_, stamp));

    // Ensure the minimum overlap between frames
    const double overlap = icpMapping_.errorMinimizer->getOverlap();
    ROS_INFO_STREAM("Overlap of frame-to-model ICP: " << overlap);

    T_sensor2map_ = transformation_->correctParameters(T_sensor2map_updated);
    T_local2map_ = transformation_->correctParameters(T_local2map_updated);

    //    graph_.emplace_shared<mapping_utils::poseFactor>();

    ROS_INFO("Adding new points");

    icpMapLock_.lock();

    // Set the local map as the reference for ICP_mapping
    DP localMap =
        transformation_->compute(getLocalMap(), T_sensor2map_updated.inverse());
    localMapFilter_->inPlaceFilter(localMap);
    icpMapping_.setMap(localMap);
    ROS_INFO_STREAM("Set the filtered local map for ICP_mapping with "
                    << localMap.getNbPoints() << " pts.");
    icpMapLock_.unlock();

    if (overlap < minOverlap_)
    {
      ROS_ERROR_STREAM("Estimated overlap too small, ignoring ICP correction");
      return;
    }

    // Compute tf and publish
    publishStamp_ = stamp;
    publishLock_.lock();

    tfBroadcaster_.sendTransform(
        PointMatcher_ros::eigenMatrixToStampedTransform<float>(
            T_sensor2map_updated, map_frame_id_, sensor_frame_id_, stamp));

    publishLock_.unlock();
    processingPointCloud_ = false;

    // Set the estimated Path
    trajectoryEstimate_.header.stamp = stamp;
    trajectoryEstimate_.header.frame_id = map_frame_id_;
    geometry_msgs::PoseStamped poseEstimate;
    poseEstimate.pose = geometry_utils::eigenToRosPose<float>(T_sensor2map_updated);
    trajectoryEstimate_.poses.push_back(poseEstimate);

    pubPathEstimate_.publish(trajectoryEstimate_);


    //    // Check if the new points should be added to the map
    //    if (((overlap < maxOverlap_) ||
    //         (icpMapping_.getPrefilteredInternalMap().features.cols() <
    //          minNumPts_)) &&
    //        (!processingMapBuilding_))
    //    { // process the last available map
    //      mapCreationStamp_ = stamp;

    // Publish map point cloud
    //      publishLock_.lock();
    //      if (pubMap_.getNumSubscribers())
    //      {
    //        ROS_INFO_STREAM("Publishing " << localMap.getNbPoints() << "
    //        pts");
    //      }
    //      publishLock_.unlock();

    //      setMap(updateMap(cloud, T_sensor2map_updated, true));

    //      ROS_INFO("Adding new points in a separate thread");

    //      mapbuildingTask_ = mapBuildingTask(
    //          boost::bind(&LidarMapping::updateMap, this, cloud.release(),
    //                      T_sensor2map_updated, true));
    //      mapBuildingFuture_ = mapbuildingTask_.get_future();
    //      mapBuildingThread_ =
    //          boost::thread(boost::move(boost::ref(mapbuildingTask_)));
    //      mapBuildingThread_.detach();
    //      sched_yield();
    //      processingMapBuilding_ = true;
    //    }
    //    else
    //    {
    //      ROS_ERROR_STREAM("Skipping map"
    //                       << endl
    //                       << "estimated overlap < max. Overlap to merge");
    //      ROS_ERROR_STREAM(
    //          "Number of points in map < minNumPts: "
    //          << icpMapping_.getPrefilteredInternalMap().features.cols() << "
    //          < "
    //          << minNumPts_);
    //      ROS_ERROR_STREAM("processingMapBuilding: " <<
    //      processingMapBuilding_);

    //      bool stateLock = publishLock_.try_lock();
    //      if (stateLock)
    //        publishLock_.unlock();
    //      ROS_ERROR_STREAM("publishLock.try_lock(): " << stateLock);

    //      stateLock = icpMapLock_.try_lock();
    //      if (stateLock)
    //        icpMapLock_.unlock();
    //      ROS_ERROR_STREAM("icpMapLock_.try_lock(): " << stateLock);

    //      return;
    //    }
  }
  catch (PM::ConvergenceError e)
  {
    icpMapLock_.unlock();

    ROS_ERROR_STREAM("Failed to convenge: " << e.what());

    return;
  }

  int realTimeRatio =
      100 * timer.elapsed() / (stamp.toSec() - lastPointCloudStamp_.toSec());
  realTimeRatio *= seq - lastPointCloudSeq_;

  ROS_INFO_STREAM("Total ICP took: " << timer.elapsed() << " s.");
  if (realTimeRatio < 80)
    ROS_INFO_STREAM("Real-time capability: " << realTimeRatio << "%");
  else
    ROS_WARN_STREAM("Real-time capability: " << realTimeRatio << "%");

  lastPointCloudStamp_ = stamp;
  lastPointCloudSeq_ = seq;

  ROS_INFO_STREAM("T_local2map_:" << endl
                                  << T_local2map_);
  ROS_INFO_STREAM("T_sensor2map_:" << endl
                                   << T_sensor2map_);

  ROS_INFO("[processCloud end]");
}

/** For icp pyramid
void LidarMapping::pointCloudPreprocessing(
    const sensor_msgs::PointCloud2ConstPtr& lidarPointCloudIn, DP& DP_current,
    struct DPPyramid& DP_pyramid)
{

  ROS_INFO("Preprocessing for ICP Pyramid ...");
  DP_current =
      PointMatcher_ros::rosMsgToPointMatcherCloud<float>(*lidarPointCloudIn);
  ROS_INFO_STREAM("Number of points (original): " << DP_current.getNbPoints());

  DP_pyramid.pointCloudFirst = DP_current;
  DP_pyramid.pointCloudSecond = DP_current;
  DP_pyramid.pointCloudThird = DP_current;

  filterPyr_.inputFiltersFirst.apply(DP_pyramid.pointCloudFirst);
  filterPyr_.inputFiltersSecond.apply(DP_pyramid.pointCloudSecond);
  filterPyr_.inputFiltersThird.apply(DP_pyramid.pointCloudThird);

  ROS_INFO_STREAM("Number of points (first after filter): "
                  << DP_pyramid.pointCloudFirst.getNbPoints());
  ROS_INFO_STREAM("Number of points (second after filter): "
                  << DP_pyramid.pointCloudSecond.getNbPoints());
  ROS_INFO_STREAM("Number of points (third after filter): "
                  << DP_pyramid.pointCloudThird.getNbPoints());
  ROS_INFO("Finished Preprocessing.");
}
*/

// void LidarMapping::setMap(DPPtr cloud)
//{
//  // Delete old map
//  //  if (PointCloudMap_ && PointCloudMap_ != cloud)
//  //    delete PointCloudMap_;

//  // Set new map
//  PointCloudMap_ = cloud;

//  // Update ICP map
//  updateICPMap(PointCloudMap_);

//  // Publish map point cloud
//  publishLock_.lock();
//  if (pubMap_.getNumSubscribers())
//  {
//    ROS_INFO_STREAM("Publishing " << PointCloudMap_->getNbPoints() << " pts");
//    pubMap_.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(
//        *PointCloudMap_, map_frame_id_, mapCreationStamp_));
//  }
//  publishLock_.unlock();
//}

// void LidarMapping::updateICPMap(const DPPtr mapCloud)
//{
//  try
//  {
//    DP localMap = transformation_->compute(*mapCloud,
//    T_sensor2map_.inverse());

//    // Get the local map by applying a range filter
//    localMapFilter_->inPlaceFilter(localMap);

//    icpMapLock_.lock();

//    // Set the local map as the reference for ICP_mapping
//    icpMapping_.setMap(localMap);
//    ROS_INFO_STREAM("Set the filtered local map for ICP_mapping with "
//                    << localMap.getNbPoints() << " pts.");
//    icpMapLock_.unlock();
//  }
//  catch (...)
//  {
//    ROS_ERROR_STREAM("Unexpected exception... ignoring scan");
//    return;
//  }
//}

//// Simply added the points together
// LidarMapping::DPPtr LidarMapping::updateMapSimple(
//    DPPtr cloud, const PM::TransformationParameters T_sensor2map_updated,
//    bool mapExists)
//{
//  ROS_INFO("Begin to update the map (in a simple way).");

//  timer timer;

//  // Transform the point cloud to the actual position (last)
//  *cloud = transformation_->compute(*cloud, T_sensor2map_updated);
//  preMapFilters_.apply(*cloud);

//  // Publish the cloud
//  pubCloud_.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(
//      *cloud, map_frame_id_, ros::Time::now()));

//  if (mapExists)
//  {
//    cloud->concatenate(*PointCloudMap_);
//    postMapFilters_.apply(*cloud);
//  }

//  ROS_INFO_STREAM("New map available (" << cloud->features.cols()
//                                        << " pts), update took "
//                                        << timer.elapsed() << " s");

//  return cloud;
//}

// LidarMapping::DPPtr
// LidarMapping::updateMap(DPPtr newCloud,
//                        const PM::TransformationParameters
//                        T_sensor2map_updated,
//                        bool mapExists)
//{
//  timer timer;

//  try
//  {
//    if (newCloud->descriptorExists("nearPointExists") == false)
//    {
//      newCloud->addDescriptor("nearPointExists",
//                              PM::Matrix::Zero(1, newCloud->features.cols()));
//    }

//    if (!mapExists)
//    {
//      // If it is the first scene, then transform the cloud
//      // and apply the post map filter

//      *newCloud = transformation_->compute(*newCloud, T_sensor2map_updated);
//      postMapFilters_.apply(*newCloud);

//      ROS_INFO_STREAM("Map initialized, the first map contains "
//                      << newCloud->getNbPoints() << " pts.");

//      return newCloud;
//    }

//    // Get the point number of incoming point cloud and the map
//    const int numPtsCloud(newCloud->getNbPoints());
//    const int numPtsMap(PointCloudMap_->getNbPoints());

//    // Get the point position matrix of incoming points
//    PM::Matrix posCloud = newCloud->features.topRows(3);

//    // Create a nearest-neighbour search for the position matrix
//    std::shared_ptr<NNS> NNSearch;
//    NNSearch.reset(NNS::create(posCloud));

//    // Transform the global map in local coordinates
//    DP mapInLocal = transformation_->compute(*PointCloudMap_,
//                                             T_sensor2map_updated.inverse());

//    pubMapInLocal_.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(
//        mapInLocal, map_frame_id_, ros::Time::now()));

//    // Remove points out of the sensor range to reduce the computation
//    PM::Matrix globalId(1, numPtsMap);

//    // Cut the map point cloud by picking up points within the sensor range
//    // (120m)
//    // Get number of points in the cut cloud
//    int numPtsLocalMap = 0;
//    for (int i = 0; i < numPtsMap; i++)
//    {
//      if (mapInLocal.features.col(i).head(3).norm() < localMapRange_)
//      {
//        mapInLocal.setColFrom(numPtsLocalMap, mapInLocal, i);
//        globalId(0, numPtsLocalMap) = i;
//        numPtsLocalMap++;
//      }
//    }

//    ROS_INFO_STREAM("Number of pts in local map: " << numPtsLocalMap);

//    // Resize the cut cloud
//    mapInLocal.conservativeResize(numPtsLocalMap);

//    // Set Matrice for nearest neighbor search
//    // Idxs is a matrix of indices to columns of position matrix and
//    // dists is a matrix of squared distances corresponding to these indices.
//    PM::Matches::Dists dists(1, numPtsLocalMap);
//    PM::Matches::Ids Idxs(1, numPtsLocalMap);

//    // Look for nearest neighbor (NN = 1) of each point inside the map (global
//    // map in local coordinates) in the incoming point cloud
//    NNSearch->knn(mapInLocal.features.topRows(3), Idxs, dists, 1, 0,
//                  NNS::ALLOW_SELF_MATCH, maxNNSDist_);

//    // Define the view for the descriptor
//    DP::View viewOnNearPointState =
//        newCloud->getDescriptorViewByName("nearPointExists");

//    for (int i = 0; i < numPtsLocalMap; i++)
//    {
//      if (dists(i) != numeric_limits<float>::infinity())
//      {
//        const int readId = Idxs(0, i);
//        const int mapId = globalId(0, i);
//        const Eigen::VectorXd readPt = newCloud->features.col(readId).head(3);
//        const Eigen::VectorXd mapPt = mapInLocal.features.col(mapId).head(3);

//        Eigen::VectorXd runningAverage = (readPt + mapPt) / 2;

//        //        *PointCloudMap_->features.col(mapId).head(3) =
//        //        runningAverage.transform;
//      }
//    }

//    DP tempMap = mapInLocal;
//    tempMap.concatenate(*newCloud);

//    // Look for nearest neighbor (NN = 1) of each point inside the map (global
//    // map in local coordinates) in the incoming point cloud
//    NNSearch.reset(NNS::create(tempMap.features, tempMap.features.rows() - 1,
//                               NNS::KDTREE_LINEAR_HEAP,
//                               NNS::TOUCH_STATISTICS));
//    PM::Matches matchesOverlap(PM::Matches::Dists(1, numPtsCloud),
//                               PM::Matches::Ids(1, numPtsCloud));
//    NNSearch->knn(newCloud->features, matchesOverlap.ids,
//    matchesOverlap.dists,
//                  1, 0);

//    // Prepare the DataPoints for overlap and non-overlap part
//    // First set the size to be same as the incoming point cloud
//    DP overlap(newCloud->createSimilarEmpty());
//    DP noOverlap(newCloud->createSimilarEmpty());

//    int numPtsOut = 0;
//    int numPtsIn = 0;

//    // Split the cloud with a distance threshold (corresponding point pair)
//    for (int i = 0; i < numPtsCloud; ++i)
//    {
//      if (matchesOverlap.dists(i) > maxDistNewPoint_)
//      {
//        noOverlap.setColFrom(numPtsOut, *newCloud, i);
//        numPtsOut++;
//      }
//      else
//      {
//        overlap.setColFrom(numPtsIn, *newCloud, i);
//        numPtsIn++;
//      }
//    }

//    ROS_INFO_STREAM("number of pts in overlap: "
//                    << numPtsIn
//                    << ", number of pts in non-overlap: " << numPtsOut);

//    // Resize the splitted point cloud
//    noOverlap.conservativeResize(numPtsOut);
//    overlap.conservativeResize(numPtsIn);

//    *newCloud = noOverlap;

//    *newCloud = transformation_->compute(*newCloud, T_sensor2map_updated);

//    newCloud->concatenate(*PointCloudMap_);
//    postMapFilters_.apply(*newCloud);
//  }
//  catch (DP::InvalidField e)
//  {
//    ROS_ERROR_STREAM(e.what());
//    abort();
//  }
//  catch (const std::exception& e)
//  {
//    ROS_ERROR_STREAM(e.what());
//    abort();
//  }

//  ROS_INFO_STREAM("New map available with " << newCloud->features.cols()
//                                            << " points, the update took "
//                                            << timer.elapsed() << " s.");

//  return newCloud;
//}

// void LidarMapping::clearMap()
//{
//  ROS_INFO("Clear the map.");

//  T_odom2map_ = TransformationParameters::Identity(4, 4);
//  T_local2map_ = TransformationParameters::Identity(4, 4);
//  T_sensor2odom_ = TransformationParameters::Identity(4, 4);

//  *PointCloudMap_ = DP();
//}

LidarMapping::DP LidarMapping::getLocalMap()
{
  int size = graph_.getNumNodes();
  DP local_map = transformation_->compute(*graph_.pose_graph[size - 1].cloudPtr,
                                          graph_.pose_graph[size - 1].pose);

  for (int i = size - 2; i > size - 11; --i)
  {
    if (i < 0)
    {
      break;
    }
    DP cloudInGlobal = transformation_->compute(*graph_.pose_graph[i].cloudPtr,
                                                graph_.pose_graph[i].pose);
    local_map.concatenate(cloudInGlobal);
  }

  pubMap_.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(
      local_map, map_frame_id_, ros::Time::now()));

  return local_map;
}

void LidarMapping::publishLoop(double publishPeriod)
{
  if (publishPeriod == 0)
    return;
  ros::Rate rate(1.0 / publishPeriod);
  while (ros::ok())
  {
    publishTransform();
    rate.sleep();
  }
}

void LidarMapping::publishTransform()
{
  //  if (processingPointCloud_ == false && initialized_ == true)
  if (initialized_ == true)
  {
    publishLock_.lock();
    ros::Time stamp = ros::Time::now();

    //    std::vector<geometry_msgs::TransformStamped> stampedTransforms;

    //    geometry_msgs::TransformStamped stampedTransform;

    //    stampedTransform = geometry_utils::toRosTransformStamped<float>(
    //        T_odom2map_.inverse(), map_frame_id_, odometry_frame_id_);
    //    stampedTransforms.push_back(stampedTransform);

    //    stampedTransform = geometry_utils::toRosTransformStamped<float>(
    //        T_odom2sensor_, odometry_frame_id_, sensor_frame_id_);
    //    stampedTransforms.push_back(stampedTransform);

    //    tfBroadcaster_.sendTransform(stampedTransforms);

    //    tfBroadcaster_.sendTransform(
    //        PointMatcher_ros::eigenMatrixToStampedTransform<float>(
    //            T_odom2map_, map_frame_id_, odometry_frame_id_, stamp));

    //    tfBroadcaster_.sendTransform(
    //        PointMatcher_ros::eigenMatrixToStampedTransform<float>(
    //            T_sensor2odom_, odometry_frame_id_, sensor_frame_id_, stamp));

    tfBroadcaster_.sendTransform(
        PointMatcher_ros::eigenMatrixToStampedTransform<float>(
            T_sensor2map_, map_frame_id_, sensor_frame_id_, stamp));

    publishLock_.unlock();
  }
}

void LidarMapping::publishPoints(const DP& pmCloud, const ros::Publisher& pub)
{

  sensor_msgs::PointCloud2 msg;
  std::string frame_id = sensor_frame_id_;
  msg = PointMatcher_ros::pointMatcherCloudToRosMsg<float>(pmCloud, frame_id,
                                                           ros::Time::now());
  pub.publish(msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "LidarMapping");
  ros::NodeHandle nh;
  LidarMapping LidarMapping(nh);
  if (!LidarMapping.initialize())
  {
    ROS_ERROR("%s: Failed to initialize Lidar Mapping processor.",
              ros::this_node::getName().c_str());
    return EXIT_FAILURE;
  }
  ros::spin();

  return EXIT_SUCCESS;
}
