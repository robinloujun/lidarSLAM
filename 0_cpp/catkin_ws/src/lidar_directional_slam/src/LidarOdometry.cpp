#include <pointmatcher/PointMatcher.h>
#include <lidar_directional_slam/lidarodometry.h>
#include <lidar_directional_slam/utils/parameter_utils.h>
//#include <lidar_directional_slam/utils/mapping_utils.h>
#include <sensor_msgs/point_cloud2_iterator.h>

LidarOdometry::LidarOdometry(ros::NodeHandle& nh)
    : T_initial_guess_(PM::TransformationParameters::Identity(4, 4)),
      T_world_(PM::TransformationParameters::Identity(4, 4)),
      odometryOnly_(getParam<bool>("odometry_only", false))
{
  nodeName_ = ros::names::append(nh.getNamespace(), "LidarOdometry");

  subPointCloud_ = nh.subscribe<sensor_msgs::PointCloud2>(
      "/velodyne_points", 2, &LidarOdometry::lidarPointCloudHandlerDP, this);

  pubPointCloudCur_ =
      nh.advertise<sensor_msgs::PointCloud2>("point_cloud_current", 2);

  pubPointCloudLast_ =
      nh.advertise<sensor_msgs::PointCloud2>("point_cloud_last", 2);

  pubPointCloudRegist1_ =
      nh.advertise<sensor_msgs::PointCloud2>("point_cloud_registed_1", 2);

  pubPointCloudRegist2_ =
      nh.advertise<sensor_msgs::PointCloud2>("point_cloud_registed_2", 2);

  pubPointCloudRegist3_ =
      nh.advertise<sensor_msgs::PointCloud2>("point_cloud_registed_3", 2);

  pubPointCloudTransfered_ =
      nh.advertise<sensor_msgs::PointCloud2>("point_cloud_transfered", 2);

  pubPathEstimate_ = nh.advertise<nav_msgs::Path>("estimated_path", 50, true);

  pubPathGroundTruth_ = nh.advertise<nav_msgs::Path>("ground_truth", 50, true);

  //  pubPoseGraph_ =
  //      nh.advertise<lidar_directional_slam::PoseGraph>("pose_graph", 10,
  //      false);

  //  pubGraphNode_ =
  //      nh.advertise<visualization_msgs::Marker>("pose_graph_nodes", 10,
  //      false);

  //  pubGraphEdges_ =
  //      nh.advertise<visualization_msgs::Marker>("pose_graph_edges", 10,
  //      false);
}

LidarOdometry::~LidarOdometry() {}

bool LidarOdometry::initialize()
{
  ROS_INFO("Initializing the Node LidarOdometry ...");

  /*
    // Initialize the pointers for PCL
    pclCur_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new
    pcl::PointCloud<pcl::PointXYZ>);
    pclPyr1_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new
    pcl::PointCloud<pcl::PointXYZ>);
    pclPyr2_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new
    pcl::PointCloud<pcl::PointXYZ>);
    pclPyr3_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new
    pcl::PointCloud<pcl::PointXYZ>);
    pclRegist_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new
    pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclLast1_(new
    pcl::PointCloud<pcl::PointXYZ>);
  */

  //  pcl::PointCloud<pcl::PointXYZ>::Ptr pclLast1_(
  //      new pcl::PointCloud<pcl::PointXYZ>);

  if (!loadParameters())
  {
    ROS_ERROR("%s: Failed to load parameters.", nodeName_.c_str());
    return false;
  }

  if (!loadICPConfig())
  {
    ROS_ERROR("%s: Failed to load configuration for ICP (libpointmatcher).",
              nodeName_.c_str());
    return false;
  }

  if (!RegisterCallbacks())
  {
    ROS_ERROR("%s: Failed to register callbacks.", nodeName_.c_str());
    return false;
  }

  return true;
}

bool LidarOdometry::loadParameters()
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

  /*
    // Using geometry_utils for SE(3) representation
    // Load initial position.
    float init_x = 0.0, init_y = 0.0, init_z = 0.0;
    float init_roll = 0.0, init_pitch = 0.0, init_yaw = 0.0;
    if (!parameter_utils::Get("init/position/x", init_x)) return false;
    if (!parameter_utils::Get("init/position/y", init_y)) return false;
    if (!parameter_utils::Get("init/position/z", init_z)) return false;
    if (!parameter_utils::Get("init/orientation/roll", init_roll)) return false;
    if (!parameter_utils::Get("init/orientation/pitch", init_pitch)) return
    false;
    if (!parameter_utils::Get("init/orientation/yaw", init_yaw)) return false;

    geometry_utils::Transform3f init;
    init.translation = geometry_utils::Vec3f(init_x, init_y, init_z);
    init.rotation = geometry_utils::Rot3f(init_roll, init_pitch, init_yaw);
    integratedEstimate_ = init;
  */

  /*
    // Load Parameters for PCL
    // Load parameters for point cloud preprocessing (PCL)
    if (!parameter_utils::Get("PCL/random_filter", filterParam_.randomFilter))
    return false;
    if (!parameter_utils::Get("PCL/filter_first_layer/percentage",
    filterParam_.percent1)) return false;
    if (!parameter_utils::Get("PCL/filter_second_layer/percentage",
    filterParam_.percent2)) return false;
    if (!parameter_utils::Get("PCL/filter_third_layer/percentage",
    filterParam_.percent3)) return false;
    if (!parameter_utils::Get("PCL/grid_filter", filterParam_.voxelGridFilter))
    return false;
    if (!parameter_utils::Get("PCL/filter_first_layer/x",
    filterParam_.leafSizeX1)) return false;
    if (!parameter_utils::Get("PCL/filter_first_layer/y",
    filterParam_.leafSizeY1)) return false;
    if (!parameter_utils::Get("PCL/filter_first_layer/z",
    filterParam_.leafSizeZ1)) return false;
    if (!parameter_utils::Get("PCL/filter_second_layer/x",
    filterParam_.leafSizeX2)) return false;
    if (!parameter_utils::Get("PCL/filter_second_layer/y",
    filterParam_.leafSizeY2)) return false;
    if (!parameter_utils::Get("PCL/filter_second_layer/z",
    filterParam_.leafSizeZ2)) return false;
    if (!parameter_utils::Get("PCL/filter_third_layer/x",
    filterParam_.leafSizeX3)) return false;
    if (!parameter_utils::Get("PCL/filter_third_layer/y",
    filterParam_.leafSizeY3)) return false;
    if (!parameter_utils::Get("PCL/filter_third_layer/z",
    filterParam_.leafSizeZ3)) return false;
    if (!parameter_utils::Get("PCL/statistical_outlier_filter",
    filterParam_.statisticalOutlierFilter)) return false;
    if (!parameter_utils::Get("PCL/number_neighbors", filterParam_.meanK))
    return false;
    if (!parameter_utils::Get("PCL/standard_deviation_mean_distance",
    filterParam_.stdDevMulThresh)) return false;
    if (!parameter_utils::Get("PCL/radius_outlier_filter",
    filterParam_.radiusOutlierFilter)) return false;
    if (!parameter_utils::Get("PCL/radius", filterParam_.radius)) return false;
    if (!parameter_utils::Get("PCL/radius_knn", filterParam_.minNeighbors))
    return false;

    // Load parameters for iterative closet points (PCL)
    if (!parameter_utils::Get("PCL/icp_max_iteration/first_layer",
    icpParam_.iteration1)) return false;
    if (!parameter_utils::Get("PCL/icp_max_iteration/second_layer",
    icpParam_.iteration2)) return false;
    if (!parameter_utils::Get("PCL/icp_max_iteration/third_layer",
    icpParam_.iteration3)) return false;
  */

  ///*
  // Load Parameters for libpointmatcher
  // Load parameters for filters (libpointmatcher)
  string configFileName;
  ros::param::get("~filter1", configFileName);
  ifstream ifs_first(configFileName.c_str());
  filterPyr_.inputFiltersFirst = PM::DataPointsFilters(ifs_first);
  ros::param::get("~filter2", configFileName);
  ifstream ifs_second(configFileName.c_str());
  filterPyr_.inputFiltersSecond = PM::DataPointsFilters(ifs_second);
  ros::param::get("~filter3", configFileName);
  ifstream ifs_third(configFileName.c_str());
  filterPyr_.inputFiltersThird = PM::DataPointsFilters(ifs_third);
  //*/

  return true;
}

bool LidarOdometry::loadICPConfig()

{

  // load YAML configuration
  string configFileName;

  //  ros::param::get("~icp_libpointmatcher", configFileName);
  //  ifstream ifs(configFileName.c_str());
  //  if (ifs.good())
  //  {
  //    icp_.loadFromYaml(ifs);
  //  }
  //  else
  //  {
  //    ROS_ERROR_STREAM("Cannot load ICP config from YAML file " <<
  //    configFileName);
  //    icp_.setDefault();
  //  }

  ros::param::get("~icp1", configFileName);
  ifstream ifs_first(configFileName.c_str());
  if (ifs_first.good())
  {
    icpFirst_.loadFromYaml(ifs_first);
  }
  else
  {
    ROS_ERROR_STREAM(
        "Cannot load ICP config from YAML file for the first layer at "
        << configFileName);
    icpFirst_.setDefault();
  }

  ros::param::get("~icp2", configFileName);
  ifstream ifs_second(configFileName.c_str());
  if (ifs_second.good())
  {
    icpSecond_.loadFromYaml(ifs_second);
  }
  else
  {
    ROS_ERROR_STREAM(
        "Cannot load ICP config from YAML file for the second layer at "
        << configFileName);
    icpSecond_.setDefault();
  }

  ros::param::get("~icp3", configFileName);
  ifstream ifs_third(configFileName.c_str());
  if (ifs_third.good())
  {
    icpThird_.loadFromYaml(ifs_third);
  }
  else
  {
    ROS_ERROR_STREAM(
        "Cannot load ICP config from YAML file for the third layer at "
        << configFileName);
    icpThird_.setDefault();
  }

  return true;
}

bool LidarOdometry::RegisterCallbacks()
{
  //  any needed function
  return true;
}

/*
void LidarOdometry::lidarPointCloudHandlerPCL(const
sensor_msgs::PointCloud2ConstPtr& lidarPointCloudIn)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr pclPyrCur(new
pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pclPyrCur1(new
pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pclPyrCur2(new
pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pclPyrCur3(new
pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pclPyrRegist1(new
pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ> pclCur;
  pcl::PointCloud<pcl::PointXYZ> pcllayer1;

  pointCloudPreprocessing(lidarPointCloudIn, pclPyrCur, pclPyrCur1, pclPyrCur2,
pclPyrCur3);

  ROS_INFO("Finished preprocessing");

  if (registInit_)
  {

    // Get the ground truth
    tfListener_.lookupTransform(world_frame_id_, "base_link", ros::Time(0),
tfMSGGroundTruth_);

    Eigen::Matrix4f T_initial_guess = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f Ticp;
    *pclPyrRegist1 = *pclPyrCur1;

    pclCur = *pclPyrCur;
    pcllayer1 = *pclPyrCur1;
    ROS_INFO_STREAM("Original: Width: " << pclCur.width << " , Height: " <<
pclCur.height);
    ROS_INFO_STREAM("Layer1: Width: " << pcllayer1.width << " , Height: " <<
pcllayer1.height);

    applyGICP(pclPyrCur1, pclLast1_, icpParam_.iteration1, T_initial_guess,
Ticp, pclPyrRegist1);

    incrementalEstimate_.translation = geometry_utils::getTranslation(Ticp);
    incrementalEstimate_.rotation = geometry_utils::getRotation(Ticp);

    T_initial_guess_ = Ticp;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclPyrRegist2(new
pcl::PointCloud<pcl::PointXYZ>);
    applyGICP(pclPyrCur2, pclPyrRegist1, icpParam_.iteration2, T_initial_guess,
Ticp, pclPyrRegist2);

//    incrementalEstimate_.translation = geometry_utils::getTranslation(Ticp);
//    incrementalEstimate_.rotation = geometry_utils::getRotation(Ticp);

//    T_initial_guess_ = Ticp;
//    pcl::PointCloud<pcl::PointXYZ>::Ptr pclPyrRegist3(new
pcl::PointCloud<pcl::PointXYZ>);
//    applyGICP(pclPyrCur1, pclPyrRegist2, icpParam_.iteration3,
T_initial_guess, Ticp, pclPyrRegist3);

//    incrementalEstimate_.translation = geometry_utils::getTranslation(Ticp);
//    incrementalEstimate_.rotation = geometry_utils::getRotation(Ticp);

//    integratedEstimate_ = geometry_utils::PoseUpdate(integratedEstimate_,
incrementalEstimate_);

//    stamp_ = ros::Time::now();

//    // Publish the pose
//    publishPose(incrementalEstimate_, pubIncrementalEstimate_);
//    publishPose(integratedEstimate_, pubIntegratedEstimate_);

//    // Convert transform between fixed frame and odometry frame.
//    geometry_msgs::TransformStamped tf;
//    tf.transform = geometry_utils::ros::ToRosTransform(integratedEstimate_);
//    tf.header.stamp = stamp_;
//    tf.header.frame_id = world_frame_id_;
//    tf.child_frame_id = odometry_frame_id_;
//    tfBroadcaster_.sendTransform(tf);

//    // Set the Path
//    trajectory_.header.stamp = stamp_;
//    trajectory_.header.frame_id = world_frame_id_;
//    poseMSGCurrent_.pose.position.x = integratedEstimate_.translation(0);
//    poseMSGCurrent_.pose.position.y = integratedEstimate_.translation(1);
//    poseMSGCurrent_.pose.position.z = integratedEstimate_.translation(2);
//    trajectory_.poses.push_back(poseMSGCurrent_);

//    pubPath_.publish(trajectory_);

//    publishPoints(pclPyrCur, pubPointCloudCur_);
//    publishPoints(pclLast1_, pubPointCloudLast_);
//    publishPoints(pclPyrRegist1, pubPointCloudRegist1_);
////    publishPoints(pclPyrRegist2, pubPointCloudRegist2_);
////    publishPoints(pclPyrRegist3, pubPointCloudRegist3_);
  }
  else // Initialize for the first loop
  {
    ROS_INFO("Get the First Point Cloud!");
    registInit_ = true;
  }

//  *pclLast_ = *pclCur_;
  pclLast1_ = pclPyrCur1;
}
*/

void LidarOdometry::lidarPointCloudHandlerDP(
    const sensor_msgs::PointCloud2ConstPtr& lidarPointCloudIn)
{

  // Preprocessing for the ICP Pyramid
  // Convert the incoming sensor_msgs to:
  // - the point cloud in the form of PointMatcher::DataPoints(PM::DP) at
  // current frame
  // - a structure of three filtered point clouds in the form of PM::DP at
  // current frame
  pointCloudPreprocessing(lidarPointCloudIn, pointCloudDPCur_,
                          pointCloudPyrCur_);

  // If only LidarOdometry executed, the ground truth path should be read in
  // odom node and should be publish at the frame odom
  if (odometryOnly_)
  {
    // Get the ground truth transform from /tf
    tfListener_.waitForTransform(world_frame_id_, "base_link", ros::Time(0),
                                 ros::Duration(1));
    tfListener_.lookupTransform(world_frame_id_, "base_link", ros::Time(0),
                                tfGroundTruthCur_);
  }

  if (registInit_)
  {
    ROS_INFO("Get the Next Point Cloud!");

    // BLOCK 1
    // set the computed transform matrix as the initial guess of the next layer
    // of ICP Pyramid
    ///*

    clock_t start, diff1, diff2, diff3; // timer to estimate the iteration time

    start = clock();
    // Compute the transformation to express data in ref
    //    PM::TransformationParameters T_first =
    //    icpFirst_(pointCloudPyrCur_.pointCloudFirst,
    //                                                     pointCloudPyrLast_.pointCloudFirst);
    PM::TransformationParameters T_first =
        icpFirst_(pointCloudPyrCur_.pointCloudFirst,
                  pointCloudPyrLast_.pointCloudFirst, T_initial_guess_);
    //    icpFirst_.transformations.apply(pointCloudPyrLast_.pointCloudSecond,
    //    T_first);

    diff1 = clock();
    ROS_INFO_STREAM("The computation for first layer of ICP pyramid takes "
                    << ((float)(diff1 - start)) / CLOCKS_PER_SEC << " s.");

    PM::TransformationParameters T_second =
        icpSecond_(pointCloudPyrCur_.pointCloudSecond,
                   pointCloudPyrLast_.pointCloudSecond, T_first);
    //    icpSecond_.transformations.apply(pointCloudPyrLast_.pointCloudThird,
    //    T_second);

    diff2 = clock();
    ROS_INFO_STREAM("The computation for second layer of ICP pyramid takes "
                    << ((float)(diff2 - diff1)) / CLOCKS_PER_SEC << " s.");

    PM::TransformationParameters T_third =
        icpThird_(pointCloudPyrCur_.pointCloudThird,
                  pointCloudPyrLast_.pointCloudThird, T_second);
    //    icpThird_.transformations.apply(pointCloudPyrLast_.pointCloudThird,
    //    T_third);

    diff3 = clock();
    ROS_INFO_STREAM("The computation for third layer of ICP pyramid takes "
                    << ((float)(diff3 - diff2)) / CLOCKS_PER_SEC << " s.");

    ROS_INFO_STREAM("T_1: " << endl
                            << T_first);
    ROS_INFO_STREAM("T_2: " << endl
                            << T_second);
    ROS_INFO_STREAM("T_3: " << endl
                            << T_third);

    PM::TransformationParameters T_frame2frame = T_third;

    // Set the final transform matrix as the initial guess for next ICP step
    T_initial_guess_ = T_third;

    // Compute the transform matrix from current frame to fixed world frame
    T_world_ = T_world_ * T_third;

    //    ROS_INFO_STREAM("T_world: " << std::endl << T_world_);

    // If use geometry_utils from BLAM
    //    incrementalEstimate_.translation =
    //    geometry_utils::getTranslation(T_frame2frame);
    //    incrementalEstimate_.rotation =
    //    geometry_utils::getRotation(T_frame2frame);

    //    integratedEstimate_ = geometry_utils::PoseUpdate(integratedEstimate_,
    //    incrementalEstimate_);

    // END OF BLOCK 1

    /*
        // BLOCK 2
        // Compute the transformation to express data in ref
        PM::TransformationParameters T_first =
       icpFirst_(pointCloudPyrCur_.pointCloudFirst,
                                                         pointCloudPyrLast_.pointCloudFirst);
        icpFirst_.transformations.apply(pointCloudPyrLast_.pointCloudSecond,
       T_first);

        PM::TransformationParameters T_delta_1 =
       icpSecond_(pointCloudPyrCur_.pointCloudSecond,
                                                           pointCloudPyrLast_.pointCloudSecond);
        icpSecond_.transformations.apply(pointCloudPyrLast_.pointCloudThird,
       T_delta_1);

        PM::TransformationParameters T_delta_2 =
       icpThird_(pointCloudPyrCur_.pointCloudThird,
                                                         pointCloudPyrLast_.pointCloudThird);

        ROS_INFO_STREAM("T_1: " << std::endl << T_first);
        ROS_INFO_STREAM("T_delta_1: " << std::endl << T_delta_1);
        ROS_INFO_STREAM("T_delta_2: " << std::endl << T_delta_2);

        Eigen::Matrix4f T_frame2frame1 = T_first;
        Eigen::Matrix4f T_frame2frame2 = T_delta_1;
        Eigen::Matrix4f T_frame2frame3 = T_delta_2;

        incrementalEstimate_.translation =
       geometry_utils::getTranslation(T_frame2frame1);
        incrementalEstimate_.rotation =
       geometry_utils::getRotation(T_frame2frame1);

        integratedEstimate_ = geometry_utils::PoseUpdate(integratedEstimate_,
       incrementalEstimate_);

        incrementalEstimate_.translation =
       geometry_utils::getTranslation(T_frame2frame2);
        incrementalEstimate_.rotation =
       geometry_utils::getRotation(T_frame2frame2);

        integratedEstimate_ = geometry_utils::PoseUpdate(integratedEstimate_,
       incrementalEstimate_);

        incrementalEstimate_.translation =
       geometry_utils::getTranslation(T_frame2frame3);
        incrementalEstimate_.rotation =
       geometry_utils::getRotation(T_frame2frame3);

        integratedEstimate_ = geometry_utils::PoseUpdate(integratedEstimate_,
       incrementalEstimate_);
    */
    // END OF BLOCK 2

    /*
    // Test the RigidTransformation of libpointmatcher
    shared_ptr<PM::Transformation> rigidTrans;
    rigidTrans = PM::get().REG(Transformation).create("RigidTransformation");

    if (!rigidTrans->checkParameters(T_world_))
    {
      T_world_ = rigidTrans->correctParameters(T_world_);
    }

    DP pointCloudTransfered =
        rigidTrans->compute(pointCloudDPCur_, T_world_.inverse());
    */

    stamp_ = ros::Time::now();

    // Publish the pose
    publishPose(T_third, pubIncrementalEstimate_);
    publishPose(T_world_, pubIntegratedEstimate_);

    // Convert transform between fixed frame and odometry frame.
    geometry_msgs::TransformStamped tfEstimate;
    tfEstimate.transform = geometry_utils::toRosTransform<float>(T_world_);
    tfEstimate.header.stamp = stamp_;
    tfEstimate.header.frame_id = odometry_frame_id_;
    tfEstimate.child_frame_id = sensor_frame_id_;
    tfBroadcaster_.sendTransform(tfEstimate);

    // Set the estimated Path
    trajectoryEstimate_.header.stamp = stamp_;
    trajectoryEstimate_.header.frame_id = odometry_frame_id_;
    geometry_msgs::PoseStamped poseEstimate;
    poseEstimate.pose = geometry_utils::toRosPose<float>(T_world_);
    trajectoryEstimate_.poses.push_back(poseEstimate);

    // if only LidarOdometry executed, visualize the ground truth path
    // The path will be published to odom frame
    if (odometryOnly_)
    {
      // Convert transform between fixed frame and ground_truth frame.
      geometry_msgs::TransformStamped tfGroundTruth;
      tf::Transform transformGroundTruth =
          tfGroundTruthFirst_.inverseTimes(tfGroundTruthCur_);
      tfGroundTruth.transform.translation.x =
          transformGroundTruth.getOrigin().x();
      tfGroundTruth.transform.translation.y =
          transformGroundTruth.getOrigin().y();
      tfGroundTruth.transform.translation.z =
          transformGroundTruth.getOrigin().z();
      tfGroundTruth.transform.rotation.x =
          transformGroundTruth.getRotation().x();
      tfGroundTruth.transform.rotation.y =
          transformGroundTruth.getRotation().y();
      tfGroundTruth.transform.rotation.z =
          transformGroundTruth.getRotation().z();
      tfGroundTruth.transform.rotation.w =
          transformGroundTruth.getRotation().w();
      tfGroundTruth.header.stamp = stamp_;
      tfGroundTruth.header.frame_id = odometry_frame_id_;
      tfGroundTruth.child_frame_id = ground_truth_frame_id_;
      tfBroadcaster_.sendTransform(tfGroundTruth);

      // Visualize the Ground Truth Trajectory
      trajectoryGroundTruth_.header.stamp = stamp_;
      trajectoryGroundTruth_.header.frame_id = odometry_frame_id_;
      geometry_msgs::PoseStamped poseGroundTruth;
      poseGroundTruth.pose.position.x = transformGroundTruth.getOrigin().x();
      poseGroundTruth.pose.position.y = transformGroundTruth.getOrigin().y();
      poseGroundTruth.pose.position.z = transformGroundTruth.getOrigin().z();
      poseGroundTruth.pose.orientation.x =
          transformGroundTruth.getRotation().x();
      poseGroundTruth.pose.orientation.y =
          transformGroundTruth.getRotation().y();
      poseGroundTruth.pose.orientation.z =
          transformGroundTruth.getRotation().z();
      poseGroundTruth.pose.orientation.w =
          transformGroundTruth.getRotation().w();
      trajectoryGroundTruth_.poses.push_back(poseGroundTruth);
    }

    //    pubOdometry_.publish(odom);
    pubPathEstimate_.publish(trajectoryEstimate_);
    if (odometryOnly_)
    {
      pubPathGroundTruth_.publish(trajectoryGroundTruth_);
    }

    publishPoints(pointCloudDPCur_, pubPointCloudCur_);
    publishPoints(pointCloudDPLast_, pubPointCloudLast_);
    publishPoints(pointCloudPyrCur_.pointCloudFirst, pubPointCloudRegist1_);
    publishPoints(pointCloudPyrCur_.pointCloudSecond, pubPointCloudRegist2_);
    publishPoints(pointCloudPyrCur_.pointCloudThird, pubPointCloudRegist3_);

    // Test the RigidTransformation of libpointmatcher
    //    publishPoints(pointCloudTransfered, pubPointCloudTransfered_);

    //    publishPoseGraph();
  }
  else // Initialize for the first loop
  {
    ROS_INFO("Get the First Point Cloud!");

    // if only LidarOdometry executed, initialize the ground truth path
    if (odometryOnly_)
    {
      tfGroundTruthFirst_ = tfGroundTruthCur_;
    }

    // Set the init boolean for registration
    registInit_ = true;
  }

  pointCloudDPLast_ = pointCloudDPCur_;
  pointCloudPyrLast_ = pointCloudPyrCur_;
}

void LidarOdometry::pointCloudPreprocessing(
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

/*
void LidarOdometry::pointCloudPreprocessing(
    const sensor_msgs::PointCloud2ConstPtr& lidarPointCloudIn,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& pclOrigin,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_Pyr_first,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_Pyr_second,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_Pyr_third)
{
  ROS_INFO("Preprocessing the point cloud with a series of filters ...");

  //  pcl::PointCloud<pcl::PointXYZ>::Ptr pclOrigin(new
  //  pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*lidarPointCloudIn, *pclOrigin);

  *pcl_Pyr_first = *pclOrigin;
  *pcl_Pyr_second = *pclOrigin;
  *pcl_Pyr_third = *pclOrigin;

  if (filterParam_.randomFilter)
  {
    const int countPoints1 =
        static_cast<int>(filterParam_.percent1 * pclOrigin->size());
    pcl::RandomSample<pcl::PointXYZ> randomFilter1;
    randomFilter1.setSample(countPoints1);
    randomFilter1.setInputCloud(pcl_Pyr_first);
    randomFilter1.filter(*pcl_Pyr_first);

    const int countPoints2 =
        static_cast<int>(filterParam_.percent2 * pclOrigin->size());
    pcl::RandomSample<pcl::PointXYZ> randomFilter2;
    randomFilter2.setSample(countPoints2);
    randomFilter2.setInputCloud(pcl_Pyr_second);
    randomFilter2.filter(*pcl_Pyr_second);

    const int countPoints3 =
        static_cast<int>(filterParam_.percent3 * pclOrigin->size());
    pcl::RandomSample<pcl::PointXYZ> randomFilter3;
    randomFilter3.setSample(countPoints3);
    randomFilter3.setInputCloud(pcl_Pyr_third);
    randomFilter3.filter(*pcl_Pyr_third);
  }

  if (filterParam_.voxelGridFilter)
  {
    pcl::VoxelGrid<pcl::PointXYZ> voxelGridFilter1;
    voxelGridFilter1.setLeafSize(filterParam_.leafSizeX1,
                                 filterParam_.leafSizeY1,
                                 filterParam_.leafSizeZ1);
    voxelGridFilter1.setInputCloud(pcl_Pyr_first);
    voxelGridFilter1.filter(*pcl_Pyr_first);

    pcl::VoxelGrid<pcl::PointXYZ> voxelGridFilter2;
    voxelGridFilter2.setLeafSize(filterParam_.leafSizeX2,
                                 filterParam_.leafSizeY2,
                                 filterParam_.leafSizeZ2);
    voxelGridFilter2.setInputCloud(pcl_Pyr_second);
    voxelGridFilter2.filter(*pcl_Pyr_second);

    pcl::VoxelGrid<pcl::PointXYZ> voxelGridFilter3;
    voxelGridFilter3.setLeafSize(filterParam_.leafSizeX3,
                                 filterParam_.leafSizeY3,
                                 filterParam_.leafSizeZ3);
    voxelGridFilter3.setInputCloud(pcl_Pyr_third);
    voxelGridFilter3.filter(*pcl_Pyr_third);
  }

  if (filterParam_.statisticalOutlierFilter)
  {
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statisticalOutlierFilter;
    statisticalOutlierFilter.setMeanK(filterParam_.meanK);
    statisticalOutlierFilter.setStddevMulThresh(filterParam_.stdDevMulThresh);

    statisticalOutlierFilter.setInputCloud(pcl_Pyr_first);
    statisticalOutlierFilter.filter(*pcl_Pyr_first);

    statisticalOutlierFilter.setInputCloud(pcl_Pyr_second);
    statisticalOutlierFilter.filter(*pcl_Pyr_second);

    statisticalOutlierFilter.setInputCloud(pcl_Pyr_third);
    statisticalOutlierFilter.filter(*pcl_Pyr_third);
  }

  if (filterParam_.radiusOutlierFilter)
  {
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> radiusOutlierFilter;
    radiusOutlierFilter.setRadiusSearch(filterParam_.radius);
    radiusOutlierFilter.setMinNeighborsInRadius(filterParam_.minNeighbors);

    radiusOutlierFilter.setInputCloud(pcl_Pyr_first);
    radiusOutlierFilter.filter(*pcl_Pyr_first);

    radiusOutlierFilter.setInputCloud(pcl_Pyr_second);
    radiusOutlierFilter.filter(*pcl_Pyr_second);

    radiusOutlierFilter.setInputCloud(pcl_Pyr_third);
    radiusOutlierFilter.filter(*pcl_Pyr_third);
  }

  // Output the number of original and filtered points
  size_t numPointsCur = pclOrigin->size();
  size_t numPointsPyr1 = pcl_Pyr_first->size();
  size_t numPointsPyr2 = pcl_Pyr_second->size();
  size_t numPointsPyr3 = pcl_Pyr_third->size();

  ROS_INFO_STREAM("Number of Points (original): " << numPointsCur);
  ROS_INFO_STREAM("Number of Points (first layer): " << numPointsPyr1);
  ROS_INFO_STREAM("Number of Points (second layer): " << numPointsPyr2);
  ROS_INFO_STREAM("Number of Points (third layer): " << numPointsPyr3);
}
*/

void LidarOdometry::applyGICP(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_reading,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_refer, const int& iteration,
    const Eigen::Matrix4f& initial_guess, Eigen::Matrix4f& T_icp,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_regist)
{
  ROS_INFO("Compute the Surface Normals");
  // Compute surface normals and curvature
  pcl::PointCloud<pcl::PointNormal>::Ptr pcl_reading_normal(
      new pcl::PointCloud<pcl::PointNormal>);
  pcl::PointCloud<pcl::PointNormal>::Ptr pcl_refer_normal(
      new pcl::PointCloud<pcl::PointNormal>);

  pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> normal_estimation;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr kdTree(
      new pcl::search::KdTree<pcl::PointXYZ>());
  normal_estimation.setSearchMethod(kdTree);
  normal_estimation.setKSearch(10);

  normal_estimation.setInputCloud(pcl_reading);
  normal_estimation.compute(*pcl_reading_normal);
  pcl::copyPointCloud(*pcl_reading, *pcl_reading_normal);

  normal_estimation.setInputCloud(pcl_refer);
  normal_estimation.compute(*pcl_refer_normal);
  pcl::copyPointCloud(*pcl_refer, *pcl_refer_normal);

  ROS_INFO("Apply the Generalized ICP ...");
  pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>
      icp; // Using Generalized ICP (GICP)
  //  pcl::IterativeClosestPointNonLinear<pcl::PointNormal, pcl::PointNormal>
  //  icp;
  icp.setTransformationEpsilon(0.0000000001);
  icp.setMaxCorrespondenceDistance(2.0);
  ROS_INFO_STREAM("Max. number of iterations " << iteration);
  icp.setMaximumIterations(iteration);
  //  icp.setRANSACIterations(0);
  //  icp.setMaximumOptimizerIterations ();

  ROS_INFO("Initialized GICP");

  icp.setInputSource(pcl_reading);
  icp.setInputTarget(pcl_refer);

  ROS_INFO("Set the source and target pcl");
  //  *pcl_regist =
  icp.align(*pcl_regist, initial_guess);
  ROS_INFO("Finished the alignment!");
  T_icp = icp.getFinalTransformation();
  ROS_INFO_STREAM("T = " << endl
                         << T_icp);
  ROS_INFO("Get the transformation matrix!");
}

void LidarOdometry::publishPose(const Eigen::Matrix4f& T,
                                const ros::Publisher& pub)
{
  // Check for subscribers
  if (pub.getNumSubscribers() == 0)
    return;

  // Convert from geometry_utils::Transform3 to ROS's PoseStamped type and
  // publish.
  geometry_msgs::PoseStamped msg;
  msg.pose = geometry_utils::toRosPose(T);
  msg.header.frame_id = odometry_frame_id_;
  msg.header.stamp = stamp_;
  pub.publish(msg);
}

void LidarOdometry::publishPoints(const DP& pmCloud, const ros::Publisher& pub)
{

  sensor_msgs::PointCloud2 msg;
  std::string frame_id = sensor_frame_id_;
  msg = PointMatcher_ros::pointMatcherCloudToRosMsg<float>(pmCloud, frame_id,
                                                           stamp_);
  pub.publish(msg);
}

void LidarOdometry::publishPoints(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& pclPtr,
    const ros::Publisher& pub)
{
  pcl::PointCloud<pcl::PointXYZ> pcl;
  pcl = *pclPtr;
  pcl.header.frame_id = sensor_frame_id_;
  pub.publish(pcl);
}

/*
void LidarOdometry::publishPoseGraph()
{
  // Publish the nodes of pose graph
  if (pubGraphNode_.getNumSubscribers() > 0){
    visualization_msgs::Marker nodeMarker;
    nodeMarker.header.frame_id = world_frame_id_;
    nodeMarker.ns = world_frame_id_;
    nodeMarker.id = 0;
    nodeMarker.action = visualization_msgs::Marker::ADD;
    nodeMarker.type = visualization_msgs::Marker::CYLINDER;
    // Set the color to red
    nodeMarker.color.r = 1.0;
    nodeMarker.color.g = 0.0;
    nodeMarker.color.b = 0.0;
    nodeMarker.color.a = 0.8; // default to 0 as invisible
    nodeMarker.scale.x = 0.1;
    nodeMarker.scale.y = 0.1;
    nodeMarker.scale.z = 1.0;
    nodeMarker.points.push_back(geometry_utils::ToRosPoint(position));
    pubGraphNode_.publish(nodeMarker);
    ROS_INFO("Updating the node info");
  }


  // Publish the edges of pose graph
  if (pubGraphEdges_.getNumSubscribers() > 0){
    visualization_msgs::Marker edgeMarker;
    edgeMarker.header.frame_id = world_frame_id_;
    edgeMarker.ns = world_frame_id_;
    edgeMarker.id = 1;
    edgeMarker.action = visualization_msgs::Marker::ADD;
    edgeMarker.type = visualization_msgs::Marker::SPHERE_LIST;
    // Set the color to red
    edgeMarker.color.r = 1.0;
    edgeMarker.color.g = 0.0;
    edgeMarker.color.b = 0.0;
    edgeMarker.color.a = 0.8; // default to 0 as invisible
    edgeMarker.scale.x = 2;
    edgeMarker.scale.y = 0.2;
    edgeMarker.scale.z = 0.2;
    edgeMarker.pose = geometry_utils::ros::ToRosPose(integratedEstimate_);
    pubGraphNode_.publish(edgeMarker);
  }

  // Publish the Pose Graph
  if (pubPoseGraph_.getNumSubscribers() > 0){

    lidar_odometry::PoseGraph poseGraph;

    poseGraph.header.frame_id = world_frame_id_;
    lidar_odometry::Node node;
    node.header.frame_id = world_frame_id_;
    node.pose.position.x =
    node.pose = geometry_utils::ros::ToRosPose(integratedEstimate_);
    poseGraph.nodes.push_back(node);

    lidar_odometry::Edge edge;
    edge.keySourceNode = 0;
    edge.keyTargetNode = 1;
    poseGraph.edges.push_back(edge);

    pubPoseGraph_.publish(poseGraph);
  }

}
*/

int main(int argc, char** argv)
{
  ros::init(argc, argv, "LidarOdometry");

  ros::NodeHandle nh;

  //  ros::Time current_time, last_time;
  //  current_time = ros::Time::now();

  //  ros::Rate r(1.0);

  LidarOdometry LidarOdometry(nh);

  if (!LidarOdometry.initialize())
  {
    ROS_ERROR("%s: Failed to initialize Lidar Odometry processor.",
              ros::this_node::getName().c_str());
    return EXIT_FAILURE;
  }

  ros::spin();

  return EXIT_SUCCESS;
}
