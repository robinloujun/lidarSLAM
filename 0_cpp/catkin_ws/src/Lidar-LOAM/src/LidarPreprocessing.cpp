#include "lidar_loam/utils.h"
#include "lidar_loam/timer.h"

// for customized point cloud data structure
//#include <sensor_msgs/point_cloud2_iterator.h>

class LidarPreprocessing
{
private:
  ros::NodeHandle nh;

  ros::Subscriber subPointCloud;

  ros::Publisher pubOriginalCloud;
  ros::Publisher pubGroundCloud;
  ros::Publisher pubSegmentedCloud;
  ros::Publisher pubOutlierCloud;
  ros::Publisher pubSingleLaser;

  ros::Publisher pubCornerPointsSharp;
  ros::Publisher pubCornerPointsLessSharp;
  ros::Publisher pubSurfPointsFlat;
  ros::Publisher pubSurfPointsLessFlat;

  std_msgs::Header cloudHeader;

  PointXYZI nanPoint;

  pcl::PointCloud<PointXYZI>::Ptr CloudIn;
  pcl::PointCloud<PointXYZI>::Ptr fullCloud;
  pcl::PointCloud<PointXYZI>::Ptr groundCloud;
  pcl::PointCloud<PointXYZI>::Ptr segmentedCloud;
  pcl::PointCloud<PointXYZI>::Ptr outlierCloud;
  pcl::PointCloud<PointXYZI>::Ptr singleLaser;

  pcl::PointCloud<PointXYZI>::Ptr cornerPointsSharp;
  pcl::PointCloud<PointXYZI>::Ptr cornerPointsLessSharp;
  pcl::PointCloud<PointXYZI>::Ptr surfPointsFlat;
  pcl::PointCloud<PointXYZI>::Ptr surfPointsLessFlat;

  pcl::PointCloud<PointXYZI>::Ptr surfPointsLessFlatScan;
  pcl::PointCloud<PointXYZI>::Ptr surfPointsLessFlatScanDS;

  pcl::VoxelGrid<PointXYZI> downSizeFilter;

  float startOrientation, endOrientation, orientationDiff;

  std::vector<int> scanStartIdx;
  std::vector<int> scanEndIdx;

  int labelCount;
  std::vector<std::pair<uint8_t, uint8_t> > neighborIterator;
  uint16_t *allPushedIdxX;
  uint16_t *allPushedIdxY;
  uint16_t *queueIdxX;
  uint16_t *queueIdxY;

  Mat2f rangeMat;
  Mat2i labelMat;
  Mat2i groundMat;

  int numSegPts;

  std::vector<float> cloudSegmentedRange;
  std::vector<int> cloudSegmentedColIdx;
  std::vector<bool> cloudSegmentedGroundFlag;

  std::vector<smoothness> cloudCurvature;
  int cloudNeighborPicked[N_SCAN*Horizon_SCAN];
  int cloudLabel[N_SCAN*Horizon_SCAN];

  bool printTime;

public:
  LidarPreprocessing():
    nh("~")
  {
    subPointCloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1, &LidarPreprocessing::cloudHandler, this);

    pubOriginalCloud = nh.advertise<sensor_msgs::PointCloud2>("/original_point_cloud", 1);
    pubGroundCloud = nh.advertise<sensor_msgs::PointCloud2>("/ground_point_cloud", 1);
    pubSegmentedCloud = nh.advertise<sensor_msgs::PointCloud2>("/segmented_point_cloud", 1);
    pubOutlierCloud = nh.advertise<sensor_msgs::PointCloud2>("/outlier_point_cloud", 1);
    pubSingleLaser = nh.advertise<sensor_msgs::PointCloud2>("/single_laser", 1);

    pubCornerPointsSharp = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 100);
    pubCornerPointsLessSharp = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 100);
    pubSurfPointsFlat = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_flat", 100);
    pubSurfPointsLessFlat = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 100);

    nanPoint.x = std::numeric_limits<float>::quiet_NaN();
    nanPoint.y = std::numeric_limits<float>::quiet_NaN();
    nanPoint.z = std::numeric_limits<float>::quiet_NaN();
    nanPoint.intensity = -1;

    allocateMemory();
    resetParameters();
  }

  ~LidarPreprocessing(){}

  void allocateMemory()
  {
    CloudIn.reset(new pcl::PointCloud<PointXYZI>());
    fullCloud.reset(new pcl::PointCloud<PointXYZI>());
    groundCloud.reset(new pcl::PointCloud<PointXYZI>());
    segmentedCloud.reset(new pcl::PointCloud<PointXYZI>());
    outlierCloud.reset(new pcl::PointCloud<PointXYZI>());
    singleLaser.reset(new pcl::PointCloud<PointXYZI>());

    cornerPointsSharp.reset(new pcl::PointCloud<PointXYZI>());
    cornerPointsLessSharp.reset(new pcl::PointCloud<PointXYZI>());
    surfPointsFlat.reset(new pcl::PointCloud<PointXYZI>());
    surfPointsLessFlat.reset(new pcl::PointCloud<PointXYZI>());

    surfPointsLessFlatScan.reset(new pcl::PointCloud<PointXYZI>());
    surfPointsLessFlatScanDS.reset(new pcl::PointCloud<PointXYZI>());

    fullCloud->points.resize(N_SCAN*Horizon_SCAN);

    scanStartIdx.assign(N_SCAN, 0);
    scanEndIdx.assign(N_SCAN, 0);
    cloudSegmentedGroundFlag.assign(N_SCAN*Horizon_SCAN, false);
    cloudSegmentedColIdx.assign(N_SCAN*Horizon_SCAN, 0);
    cloudSegmentedRange.assign(N_SCAN*Horizon_SCAN, 0);

    std::pair<int8_t, int8_t> neighbor;

    neighbor.first = -1;
    neighbor.second = 0;
    neighborIterator.push_back(neighbor);

    neighbor.first = 0;
    neighbor.second = 1;
    neighborIterator.push_back(neighbor);

    neighbor.first = 0;
    neighbor.second = -1;
    neighborIterator.push_back(neighbor);

    neighbor.first = 1;
    neighbor.second = 0;
    neighborIterator.push_back(neighbor);

    allPushedIdxX = new uint16_t[N_SCAN*Horizon_SCAN];
    allPushedIdxY = new uint16_t[N_SCAN*Horizon_SCAN];

    queueIdxX = new uint16_t[N_SCAN*Horizon_SCAN];
    queueIdxY = new uint16_t[N_SCAN*Horizon_SCAN];

    cloudCurvature.resize(N_SCAN*Horizon_SCAN);
    downSizeFilter.setLeafSize(0.2, 0.2, 0.2);

    // Load parameters from yaml
    if (!getParameter("/preprocessing/print_time", printTime))
    {
      ROS_WARN("Time print set to false");
      printTime = false;
    }
  }

  void resetParameters()
  {
    CloudIn->clear();
    groundCloud->clear();
    segmentedCloud->clear();
    outlierCloud->clear();
    singleLaser->clear();

    rangeMat.resize(boost::extents[N_SCAN][Horizon_SCAN]);
    groundMat.resize(boost::extents[N_SCAN][Horizon_SCAN]);
    labelMat.resize(boost::extents[N_SCAN][Horizon_SCAN]);

    Timer t_init("initialize the range matrix");

    for (int i = 0; i < N_SCAN; ++i)
    {
      for (int j = 0; j < Horizon_SCAN; ++j)
      {
        rangeMat[i][j] = FLT_MAX;
        groundMat[i][j] = 0;
        labelMat[i][j] = 0;
      }
    }

    if (printTime)
      t_init.tic_toc();

    labelCount = 1;
//    numSegPts = 0;

    std::fill(fullCloud->points.begin(), fullCloud->points.end(), nanPoint);
  }

  void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& pointCloudIn)
  {
    Timer t_whole("for Lidar Preprocessing");
    Timer t_comp("for ImageProjection");

    cloudHeader = pointCloudIn->header;
    pcl::fromROSMsg(*pointCloudIn, *CloudIn);

    findStartEndAngle();
    projectPointCloud();
    groundRemoval();
    cloudSegmentation();

    if (printTime)
      t_comp.tic_toc();

    setTimeInterpolation();
    calculateSmoothness();
    markOccludedPoints();
    extractFeatures();
    publishPointCloud();
    resetParameters();

    if (printTime)
      t_whole.tic_toc();
  }

  // Find the first and the last angle of rotation
  void findStartEndAngle()
  {
    Timer t_fsea("find the start and end angle");
    // The rotation angle at the beginning point, additional minus because the velodyne rotates clockwise (cw)
    startOrientation = -atan2(CloudIn->points[0].y, CloudIn->points[0].x);
    // The rotation angle at the end point, add 2*PI to make the rotation period to be 2*PI
    endOrientation = -atan2(CloudIn->points[CloudIn->points.size()-1].y,
                              CloudIn->points[CloudIn->points.size()-1].x) + 2 * M_PI;

    if (endOrientation - startOrientation > 3 * M_PI)
    {
      endOrientation -= 2 * M_PI;
    }
    else if (endOrientation - startOrientation < M_PI)
    {
      endOrientation += 2 * M_PI;
    }

    orientationDiff = endOrientation - startOrientation;

    if (printTime)
      t_fsea.tic_toc();
  }

  // Project the 3D point cloud onto a 2D range image (depth matrix rangeMat)
  // In range matrix
  // FLT_MAX => init => no point at this position
  // range => after projection
  void projectPointCloud()
  {
    Timer t_ppc("project the point cloud");
    float verticalAngle, horizonAngle, range;
    size_t rowIdx, columnIdx, totalIdx, numPts;
    PointXYZI pt;

    numPts = CloudIn->points.size();

    for (size_t i = 0; i < numPts; ++i)
    {
      pt.x = CloudIn->points[i].x;
      pt.y = CloudIn->points[i].y;
      pt.z = CloudIn->points[i].z;

      // Compute the vertical angle to get the index of laser rowIdn
      verticalAngle = atan2(pt.z, sqrt(pt.x * pt.x + pt.y * pt.y)) * 180 / M_PI;

      // cf. LeGO-LOAM
      // Lead to z-axis drift because the laser block of Velodyne HDL-64E S2 not evenly distributed
      // Vertical Field of View:
      // - (  +2  ,  -8.33) @ 1/3 degree spacing
      // - (-8.83 , -24.33) @ 1/2 degree spacing
      // For VLP-16
//      rowIdx = (verticalAngle + ang_bottom) / ang_res_y;

      // cf. A-LOAM
      // For HDL-64e
      if (verticalAngle <= -ang_lower_max)
        rowIdx = int((verticalAngle + ang_bottom) / ang_res_y_lower + 0.5);
      else
        rowIdx = N_SCAN / 2 + int((verticalAngle + ang_upper_min) / ang_res_y_upper + 0.5);

      // Remove the points out of the row range
      if (rowIdx < 0 || rowIdx >= N_SCAN)
        continue;

      // Compute the horizontal angle to get the index of a point in a laser columnIdn
      // The horizontal angle is corresponding to the reletive time
      // horizonAngle clockwise from y-neg (-180 deg) -> y-neg (180 deg)
      horizonAngle = atan2(pt.x, pt.y) * 180 / M_PI;

      // columnIdx counterclockwise from x-neg (0) -> x-neg (Horizon_SCAN)
      columnIdx = -round((horizonAngle - 90.0) / ang_res_x) + Horizon_SCAN / 2;
      if (columnIdx >= Horizon_SCAN)
          columnIdx -= Horizon_SCAN;

      // Remove the points out of the column range
      if (columnIdx < 0 || columnIdx >= Horizon_SCAN)
          continue;

      // Compute the Euclidean distance from a point to the sensor
      range = sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
      rangeMat[rowIdx][columnIdx] = range;

      // Set the index constructor as intensity of a point:
      // integral: row index
      // decimal: column index / 10000
      pt.intensity = (float)rowIdx + (float)columnIdx / 10000.0;

      totalIdx = columnIdx + rowIdx * Horizon_SCAN;
      // fullCloud contains point coordinates [x, y, z] with the index constructor as intensity
      fullCloud->points[totalIdx] = pt;
    }

    if (printTime)
      t_ppc.tic_toc();
  }

  // Remove the ground (actually extract the ground point cloud)
  // In ground matrix
  // -1 => nan point
  // 0  => not set
  // 1  => ground point
  void groundRemoval()
  {
    Timer t_gr("remove the ground");

    size_t lowerIdx, upperIdx;
    float diffX, diffY, diffZ, angle;

    for (size_t j = 0; j < Horizon_SCAN; ++j)
    {
      for (size_t i = 0; i < groundScanIdx; ++i)
      {
        // Visualize a single laser
        if (i == groundScanIdx-1)
          singleLaser->push_back(fullCloud->points[j+i*Horizon_SCAN]);

        lowerIdx = j + ( i )*Horizon_SCAN;
        upperIdx = j + (i+1)*Horizon_SCAN;

        // Make sure that both point are defined (not nan)
        // If not, set the element in ground matrix to -1
        if (fullCloud->points[lowerIdx].intensity == -1 ||
            fullCloud->points[upperIdx].intensity == -1)
        {
          groundMat[i][j] = -1;
          continue;
        }

        diffX = fullCloud->points[upperIdx].x - fullCloud->points[lowerIdx].x;
        diffY = fullCloud->points[upperIdx].y - fullCloud->points[lowerIdx].y;
        diffZ = fullCloud->points[upperIdx].z - fullCloud->points[lowerIdx].z;

        angle = atan2(diffZ, sqrt(diffX*diffX + diffY*diffY)) * 180 / M_PI;

        // When the vertical angle of the two neighboring points smaller than 10 deg
        // regarded as ground (sloped terrain allowed) -> element set to 1
        if (abs(angle - sensorMountAngle) <= 10)
        {
          groundMat[i][j] = 1;
          groundMat[i+1][j] = 1;
        }
      }
    }

    // If it is ground point, set the label to -1
    for (size_t i = 0; i < N_SCAN; ++i)
    {
      for (size_t j = 0; j < Horizon_SCAN; ++j)
      {
        if (groundMat[i][j] == 1 || rangeMat[i][j] == FLT_MAX)
        {
          labelMat[i][j] = -1;
        }
      }
    }

    // Extract the ground point cloud
    if (pubGroundCloud.getNumSubscribers() != 0)
    {
      for (size_t i = 0; i <= groundScanIdx; ++i)
      {
        for (size_t j = 0; j < Horizon_SCAN; ++j)
        {
          if (groundMat[i][j] == 1)
            groundCloud->push_back(fullCloud->points[j+i*Horizon_SCAN]);
        }
      }
    }

    if (printTime)
      t_gr.tic_toc();
  }

  // Segment the point cloud by clustering on the range image [Bogoslavskyi2016]
  void cloudSegmentation()
  {
    Timer t_cs("segment the point cloud");

    // Apply a breadth-first search (BFS) to label every pixel of this component
    for (size_t i = 0; i < N_SCAN; ++i)
      for (size_t j = 0; j < Horizon_SCAN; ++j)
        if (labelMat[i][j] == 0)
          labelComponents(i, j);

    int sizeOfSegCloud = 0;

    // Extract the outlier point cloud
    // Outlier points include the ground points, the points not belonging to big objects
    for (size_t i = 0; i < N_SCAN; ++i)
    {
      scanStartIdx[i] = sizeOfSegCloud - 1 + 5;

      for (size_t j = 0; j < Horizon_SCAN; ++j)
      {
        if (labelMat[i][j] > 0 || groundMat[i][j] == 1) // clustered || ground
        {
          if (labelMat[i][j] == 999999) // infeasible clustering
          {
            if (i > groundScanIdx && j % 5 == 0)
            {
              outlierCloud->push_back(fullCloud->points[j+i*Horizon_SCAN]);
              continue;
            }
            else
            {
              continue;
            }
          }
          if (groundMat[i][j] == 1)
          {
            if (j % 5 != 0 && j > 5 && j < Horizon_SCAN - 5)
              continue;
          }
          cloudSegmentedGroundFlag[sizeOfSegCloud] = (groundMat[i][j] == 1);
          cloudSegmentedRange[sizeOfSegCloud] = rangeMat[i][j];
          cloudSegmentedColIdx[sizeOfSegCloud] = j;
          segmentedCloud->push_back(fullCloud->points[j+i*Horizon_SCAN]);
          ++sizeOfSegCloud;
        }
      }
      scanEndIdx[i] = sizeOfSegCloud - 1 - 5;
    }
    numSegPts = sizeOfSegCloud;

    if (printTime)
      t_cs.tic_toc();
  }

  // cf. [Bogoslavskyi2016]
  // Algorithm1 Range Image Labeling -> procedure LABELCOMPONENTBFS
  // In label matrix
  // -1           => ground point or no point
  // 0            => not set
  // (0, INT_MAX) => belonging to a clustering
  // INT_MAX      =>
  void labelComponents(int row, int col)
  {
    float d1, d2, alpha, angle;
    int fromIdxX, fromIdxY, thisIdxX, thisIdxY;
    bool lineCountFlag[N_SCAN] = {false}; // Save the laser index

    queueIdxX[0] = row;
    queueIdxY[0] = col;
    int queueSize = 1;
    int queueStartIdx = 0;
    int queueEndIdx = 1;

    allPushedIdxX[0] = row;
    allPushedIdxY[0] = col;
    int allPushedIdxSize = 1;

    while(queueSize > 0){
      fromIdxX = queueIdxX[queueStartIdx];
      fromIdxY = queueIdxY[queueStartIdx];
      --queueSize;
      ++queueStartIdx;
      labelMat[fromIdxX][fromIdxY] = labelCount;

      // Search in the N4 Neighborhood
      for (auto iter = neighborIterator.begin(); iter != neighborIterator.end(); ++iter)
      {
        thisIdxX = fromIdxX + (*iter).first;
        thisIdxY = fromIdxY + (*iter).second;

        if (thisIdxX < 0 || thisIdxX >= N_SCAN)
            continue;

        if (thisIdxY < 0)
            thisIdxY = Horizon_SCAN - 1;
        if (thisIdxY >= Horizon_SCAN)
            thisIdxY = 0;

        // Check if the point already belongs to a labeled group
        if (labelMat[thisIdxX][thisIdxY] != 0)
            continue;

        // Determine which point is further
        d1 = std::max(rangeMat[fromIdxX][fromIdxY],
                      rangeMat[thisIdxX][thisIdxY]);
        d2 = std::min(rangeMat[fromIdxX][fromIdxY],
                      rangeMat[thisIdxX][thisIdxY]);

        // For HDL-64E
        if ((*iter).first == 0)
          alpha = segmentAlphaX; // Left and right -> angular resolution (in rad)
        else if (fromIdxX < N_SCAN/2)
          alpha = segmentAlphaYLower; // Up and down -> vertical resolution (in rad)
        else
          alpha = segmentAlphaYUpper;

        // For VLP-16
//        if ((*iter).first == 0)
//            alpha = segmentAlphaX;
//        else
//            alpha = segmentAlphaY;


        // Compute the angle beta to determine if two points lie on the same object
        // trigonometric equation [Bogoslavskyi2016]
        angle = atan2(d2*sin(alpha), (d1 - d2*cos(alpha)));

        // If the angle beta larger than the threshold
        // The neighboring point belongs to the same object and it should be poped into the queue
        if (angle > segmentTheta)
        {
            queueIdxX[queueEndIdx] = thisIdxX;
            queueIdxY[queueEndIdx] = thisIdxY;
            ++queueSize;
            ++queueEndIdx;

            labelMat[thisIdxX][thisIdxY] = labelCount;
            lineCountFlag[thisIdxX] = true;

            allPushedIdxX[allPushedIdxSize] = thisIdxX;
            allPushedIdxY[allPushedIdxSize] = thisIdxY;
            ++allPushedIdxSize;
        }
      }
    }

    // Check whether the segmentation is feasible
    // If there is more than 30 points belonging to a labeled group
    bool feasibleSegment = false;
    if (allPushedIdxSize >= 30)
      feasibleSegment = true;
    // Else if the there are more than 5 points and they are placed across 3 lasers
    else if (allPushedIdxSize >= segmentValidPointNum)
    {
      int lineCount = 0;
      for (size_t i = 0; i < N_SCAN; ++i)
        if (lineCountFlag[i] == true)
        {
          ++lineCount;
        }
      if (lineCount >= segmentValidLineNum)
      {
        feasibleSegment = true;
      }
    }

    // If the segmentation is feasible, label the points with labelCount
    // Otherwise set the label to be 999999 so the point to be outlier
    if (feasibleSegment == true)
      ++labelCount;
    else
    {
      for (size_t i = 0; i < allPushedIdxSize; ++i)
        labelMat[allPushedIdxX[i]][allPushedIdxY[i]] = 999999;
    }
  }

  void setTimeInterpolation()
  {
    Timer t_sti("compute the time interpolation");
    bool halfPassed = false;
    PointXYZI pt;

    for (int i = 0; i < numSegPts; ++i)
    {
      pt.x = segmentedCloud->points[i].x;
      pt.y = segmentedCloud->points[i].y;
      pt.z = segmentedCloud->points[i].z;

      float orientation = -atan2(pt.y, pt.x);
      if (!halfPassed)
      {
        if (orientation < startOrientation - M_PI / 2)
        {
          orientation += 2 * M_PI;
        }
        else if (orientation > startOrientation + M_PI * 3 / 2)
        {
          orientation -= 2 * M_PI;
        }

        if (orientation - startOrientation > M_PI)
        {
          halfPassed = true;
        }
      }
      else
      {
        orientation += 2 * M_PI;
        if (orientation < endOrientation - M_PI * 3 / 2)
        {
          orientation += 2 * M_PI;
        }
        else if (orientation > endOrientation + M_PI / 2)
        {
          orientation -= 2 * M_PI;
        }
      }

      float relTime = (orientation - startOrientation) / orientationDiff;
      pt.intensity = int(segmentedCloud->points[i].intensity) + scanPeriod * relTime;
      segmentedCloud->points[i] = pt;
    }

    if (printTime)
      t_sti.tic_toc();
  }

  void calculateSmoothness()
  {
    Timer t_cs("compute the smoothness");

    for (int i = 5; i < numSegPts - 5; ++i)
    {
      float diffRange = cloudSegmentedRange[i - 5] + cloudSegmentedRange[i - 4]
                      + cloudSegmentedRange[i - 3] + cloudSegmentedRange[i - 2]
                      + cloudSegmentedRange[i - 1] - cloudSegmentedRange[i] * 10
                      + cloudSegmentedRange[i + 1] + cloudSegmentedRange[i + 2]
                      + cloudSegmentedRange[i + 3] + cloudSegmentedRange[i + 4]
                      + cloudSegmentedRange[i + 5];

      cloudCurvature[i].value = diffRange * diffRange;
      cloudCurvature[i].idx = i;
      cloudNeighborPicked[i] = 0;
      cloudLabel[i] = 0;
    }

    if (printTime)
      t_cs.tic_toc();
  }

  void markOccludedPoints()
  {
    Timer t_mop("mark the occluded points");
    for (int i = 5; i < numSegPts - 6; ++i)
    {
      float range1 = cloudSegmentedRange[i];
      float range2 = cloudSegmentedRange[i+1];
      int columnDiff = std::abs(int(cloudSegmentedColIdx[i+1] - cloudSegmentedColIdx[i]));

      if (columnDiff < 10)
      {
        if (range1 - range2 > 0.3)
        {
          cloudNeighborPicked[i - 5] = 1;
          cloudNeighborPicked[i - 4] = 1;
          cloudNeighborPicked[i - 3] = 1;
          cloudNeighborPicked[i - 2] = 1;
          cloudNeighborPicked[i - 1] = 1;
          cloudNeighborPicked[i] = 1;
        }
        else if (range2 - range1 > 0.3)
        {
          cloudNeighborPicked[i + 1] = 1;
          cloudNeighborPicked[i + 2] = 1;
          cloudNeighborPicked[i + 3] = 1;
          cloudNeighborPicked[i + 4] = 1;
          cloudNeighborPicked[i + 5] = 1;
          cloudNeighborPicked[i + 6] = 1;
        }
      }

      float rangeDiff1 = std::abs(cloudSegmentedRange[i-1] - cloudSegmentedRange[i]);
      float rangeDiff2 = std::abs(cloudSegmentedRange[i+1] - cloudSegmentedRange[i]);

      if (rangeDiff1 > 0.02 * cloudSegmentedRange[i] && rangeDiff2 > 0.02 * cloudSegmentedRange[i])
        cloudNeighborPicked[i] = 1;
    }

    if (printTime)
      t_mop.tic_toc();
  }

  void extractFeatures()
  {
    Timer t_ef("extract the features");

    cornerPointsSharp->clear();
    cornerPointsLessSharp->clear();
    surfPointsFlat->clear();
    surfPointsLessFlat->clear();

    for (int i = 0; i < N_SCAN; ++i)
    {
      surfPointsLessFlatScan->clear();

      for (int j = 0; j < 6; ++j)
      {
        int sp = scanStartIdx[i] + (scanEndIdx[i] - scanStartIdx[i]) * j / 6;
        int ep = scanStartIdx[i] + (scanEndIdx[i] - scanStartIdx[i]) * (j + 1) / 6 - 1;

        if (sp >= ep)
          continue;

        std::sort(cloudCurvature.begin()+sp, cloudCurvature.begin()+ep, smoothnessCompare());

        int largestPickedNum = 0;
        for (int k = ep; k >= sp; k--)
        {
          int idx = cloudCurvature[k].idx;

          if (cloudNeighborPicked[idx] == 0 &&
              cloudCurvature[idx].value > edgeThreshold &&
              cloudSegmentedGroundFlag[idx] == false)
          {
            ++largestPickedNum;

            // Each subregion can provide max. 2 edge points
            if (largestPickedNum <= 2)
            {
              cloudLabel[idx] = 2;
              cornerPointsSharp->push_back(segmentedCloud->points[idx]);
              cornerPointsLessSharp->push_back(segmentedCloud->points[idx]);
            }
            else if (largestPickedNum <= 20)
            {
              cloudLabel[idx] = 1;
              cornerPointsLessSharp->push_back(segmentedCloud->points[idx]);
            }
            else
            {
              break;
            }

            cloudNeighborPicked[idx] = 1;

            // Use range difference
            /*
            for (int l = 1; l <= 5; ++l)
            {
              float rangeDiff = cloudSegmentedRange[idx+l] - cloudSegmentedRange[idx+l-1];
              if (rangeDiff * rangeDiff > 0.05)
                break;
              cloudNeighborPicked[idx+l] = 1;
            }

            for (int l = -1; l >= -5; --l)
            {
              float rangeDiff = cloudSegmentedRange[idx+l] - cloudSegmentedRange[idx+l+1];
              if (rangeDiff * rangeDiff > 0.05)
                break;
              cloudNeighborPicked[idx+l] = 1;
            }
            */

            // Use column difference
//            /*
            for (int l = 1; l <= 5; ++l)
            {
              int columnDiff = std::abs(int(cloudSegmentedColIdx[idx+l] - cloudSegmentedColIdx[idx+l-1]));
              if (columnDiff > 10)
                break;
              cloudNeighborPicked[idx+l] = 1;
            }

            for (int l = -1; l >= -5; --l)
            {
              int columnDiff = std::abs(int(cloudSegmentedColIdx[idx+l] - cloudSegmentedColIdx[idx+l+1]));
              if (columnDiff > 10)
                break;
              cloudNeighborPicked[idx+l] = 1;
            }
//            */
          }
        }

        int smallestPickedNum = 0;
        for (int k = sp; k <= ep; ++k)
        {
          int idx = cloudCurvature[k].idx;

          if (cloudNeighborPicked[idx] == 0 &&
              cloudCurvature[idx].value < surfThreshold &&
              cloudSegmentedGroundFlag[idx] == true)
          {
            cloudLabel[idx] = -1;
            surfPointsFlat->push_back(segmentedCloud->points[idx]);

            // Each subregion can provide max. 4 planar points
            ++smallestPickedNum;
            if (smallestPickedNum >= 4)
            {
              break;
            }

            cloudNeighborPicked[idx] = 1;

            // Use range difference
            /*
            for (int l = 1; l <= 5; ++l)
            {
              float rangeDiff = cloudSegmentedRange[idx+l] - cloudSegmentedRange[idx+l-1];
              if (rangeDiff * rangeDiff > 0.05)
                break;
              cloudNeighborPicked[idx+l] = 1;
            }

            for (int l = -1; l >= -5; --l)
            {
              float rangeDiff = cloudSegmentedRange[idx+l] - cloudSegmentedRange[idx+l+1];
              if (rangeDiff * rangeDiff > 0.05)
                break;
              cloudNeighborPicked[idx+l] = 1;
            }
            */

            // Use column difference
//            /*
            for (int l = 1; l <= 5; ++l)
            {
              int columnDiff = std::abs(int(cloudSegmentedColIdx[idx+l] - cloudSegmentedColIdx[idx+l-1]));
              if (columnDiff > 10)
                break;
              cloudNeighborPicked[idx+l] = 1;
            }

            for (int l = -1; l >= -5; --l)
            {
              int columnDiff = std::abs(int(cloudSegmentedColIdx[idx+l] - cloudSegmentedColIdx[idx+l+1]));
              if (columnDiff > 10)
                break;
              cloudNeighborPicked[idx+l] = 1;
            }
//            */
          }
        }

        for (int k = sp; k <= ep; ++k)
        {
          if (cloudLabel[k] <= 0)
          {
            surfPointsLessFlatScan->push_back(segmentedCloud->points[k]);
          }
        }
      }

      // Downsampling the planar points
      surfPointsLessFlatScanDS->clear();

      downSizeFilter.setInputCloud(surfPointsLessFlatScan);
      downSizeFilter.filter(*surfPointsLessFlatScanDS);

      *surfPointsLessFlat += *surfPointsLessFlatScanDS;
    }

    if (printTime)
      t_ef.tic_toc();
  }

  void publishPointCloud()
  {
    Timer t_ppc("publish the point clouds");

    sensor_msgs::PointCloud2 msgs2Pub;

    if (pubOriginalCloud.getNumSubscribers() != 0)
    {
      pcl::toROSMsg(*fullCloud, msgs2Pub);
      msgs2Pub.header.stamp = cloudHeader.stamp;
      msgs2Pub.header.frame_id = "/base_link";
      pubOriginalCloud.publish(msgs2Pub);
    }

    if (pubSegmentedCloud.getNumSubscribers() != 0)
    {
      pcl::toROSMsg(*segmentedCloud, msgs2Pub);
      msgs2Pub.header.stamp = cloudHeader.stamp;
      msgs2Pub.header.frame_id = "/base_link";
      pubSegmentedCloud.publish(msgs2Pub);
    }

    if (pubOutlierCloud.getNumSubscribers() != 0)
    {
      pcl::toROSMsg(*outlierCloud, msgs2Pub);
      msgs2Pub.header.stamp = cloudHeader.stamp;
      msgs2Pub.header.frame_id = "/base_link";
      pubOutlierCloud.publish(msgs2Pub);
    }

    if (pubGroundCloud.getNumSubscribers() != 0)
    {
      pcl::toROSMsg(*groundCloud, msgs2Pub);
      msgs2Pub.header.stamp = cloudHeader.stamp;
      msgs2Pub.header.frame_id = "/base_link";
      pubGroundCloud.publish(msgs2Pub);
    }

    if (pubSingleLaser.getNumSubscribers() != 0)
    {
      pcl::toROSMsg(*singleLaser, msgs2Pub);
      msgs2Pub.header.stamp = cloudHeader.stamp;
      msgs2Pub.header.frame_id = "/base_link";
      pubSingleLaser.publish(msgs2Pub);
    }

    if (pubCornerPointsSharp.getNumSubscribers() != 0){
        pcl::toROSMsg(*cornerPointsSharp, msgs2Pub);
        msgs2Pub.header.stamp = cloudHeader.stamp;
        msgs2Pub.header.frame_id = "/base_link";
        pubCornerPointsSharp.publish(msgs2Pub);
    }

    if (pubCornerPointsLessSharp.getNumSubscribers() != 0){
        pcl::toROSMsg(*cornerPointsLessSharp, msgs2Pub);
        msgs2Pub.header.stamp = cloudHeader.stamp;
        msgs2Pub.header.frame_id = "/base_link";
        pubCornerPointsLessSharp.publish(msgs2Pub);
    }

    if (pubSurfPointsFlat.getNumSubscribers() != 0){
        pcl::toROSMsg(*surfPointsFlat, msgs2Pub);
        msgs2Pub.header.stamp = cloudHeader.stamp;
        msgs2Pub.header.frame_id = "/base_link";
        pubSurfPointsFlat.publish(msgs2Pub);
    }

    if (pubSurfPointsLessFlat.getNumSubscribers() != 0){
        pcl::toROSMsg(*surfPointsLessFlat, msgs2Pub);
        msgs2Pub.header.stamp = cloudHeader.stamp;
        msgs2Pub.header.frame_id = "/base_link";
        pubSurfPointsLessFlat.publish(msgs2Pub);
    }

    if (printTime)
      t_ppc.tic_toc();
  }


  /* TODO: Try to write a customized point cloud data structure
  void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& pointCloudIn){
    std_msgs::Header cloudHeader = pointCloudIn->header;

    PointCloud pc;

    Timer readTimer("reading");

    for(sensor_msgs::PointCloud2ConstIterator<float> iter(*pointCloudIn, "x");
        iter != iter.end(); ++iter)
    {
      if (!std::isfinite(iter[0]) || !std::isfinite(iter[1]) || !std::isfinite(iter[2])){
        continue;
      }
      pc.push_back(point(iter[0], iter[1], iter[2], iter[3]));
    }

    if (printTime)
    readTimer.tic_toc();

  }
  */

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_loam");

    LidarPreprocessing LP;

    ROS_INFO("\033[1;32m---->\033[0m Lidar Preprocessing Started.");

    ros::spin();
    return 0;
}
