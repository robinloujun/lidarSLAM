#include <cassert>
#include <iostream>
#include <fstream>
#include "pointmatcher/IO.h"
#include "pointmatcher/PointMatcher.h"
#include "pointmatcher/PointMatcher.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <sys/stat.h>
#include <dirent.h>
#include <ctime>
#include <Eigen/Dense>

using namespace std;
using namespace PointMatcherSupport;

typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;
typedef PointMatcherIO<float> PMIO;

static vector<string> bin_file_lists;
static vector<string> pcd_file_lists;

#define MAXBUFSIZE ((int) 1e6)

void usage(int argc, char *argv[]);

bool computePairNum(std::string pair1,std::string pair2)
{
  return pair1 < pair2;
}

// Read and sort the files
void read_filelists(const string& dir_path, vector<string>& out_filelsits, string type)
{
  struct dirent *ptr;
  DIR *dir;
  dir = opendir(dir_path.c_str());
  out_filelsits.clear();
  while ((ptr = readdir(dir)) != NULL)
  {
    string tmp_file = ptr->d_name;
    if (tmp_file[0] == '.')continue;
    if (type.size() <= 0){
      out_filelsits.push_back(ptr->d_name);
    }else{
      if (tmp_file.size() < type.size())continue;
      string tmp_cut_type = tmp_file.substr(tmp_file.size() - type.size(),type.size());
      if (tmp_cut_type == type){
        out_filelsits.push_back(ptr->d_name);
      }
    }
  }
  std::sort(out_filelsits.begin(),out_filelsits.end(),computePairNum);
}

void loadBINFile(const std::string &file_name, pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
{
  fstream input(file_name.c_str(), ios::in | ios::binary);
  if(!input.good())
  {
    cerr << "Could not read file: " << file_name << endl;
    exit(EXIT_FAILURE);
  }
  input.seekg(0, ios::beg);

  int i;
  for (i=0; input.good() && !input.eof(); i++)
  {
    pcl::PointXYZI point;
    input.read((char *) &point.x, sizeof(float));
    input.read((char *) &point.y, sizeof(float));
    input.read((char *) &point.z, sizeof(float));
    input.read((char *) &point.intensity, sizeof(float));
    cloud->push_back(point);
  }
  input.close();
//  size_t PointsNum = cloud->size();

}

DP loadedPoints(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud)
{
  size_t PointsNum = cloud->size();
  cout << PointsNum << endl;
  PMIO::Matrix features(4, PointsNum);
  for (int j = 0; j < PointsNum; j++){
    features(0,j) = cloud->points[j].x;
    features(1,j) = cloud->points[j].y;
    features(2,j) = cloud->points[j].z;
    features(3,j) = 1;
  }

  PMIO::LabelGenerator featLabelGen;
  featLabelGen.add((char*) 'x');
  featLabelGen.add((char*) 'y');
  featLabelGen.add((char*) 'z');
  DP loadedPoints(features, featLabelGen.getLabels());
  return loadedPoints;
}

void lidar_visualize(pcl::PointCloud<pcl::PointXYZI>::Ptr &points, const string &frame_index)
{
}

Eigen::MatrixXf LoadPoses(const std::string& filename)
{
  int cols = 0, rows = 0;
  double buff[MAXBUFSIZE];

  // Read numbers from file into buffer
  ifstream infile;
  infile.open(filename);
  while (! infile.eof())
  {
    std::string line;
    getline(infile, line);

    int temp_cols = 0;
    stringstream stream(line);
    while(! stream.eof())
      stream >> buff[cols*rows + temp_cols++];

    if (temp_cols == 0)
      continue;
    if (cols == 0)
      cols = temp_cols;
    rows++;
  }
  infile.close();
  rows--;

  // Populate matrix with numbers
  Eigen::MatrixXf poses(rows, cols);
  for (int i = 0; i < rows; i++)
    for (int j = 0; j < cols; j++)
      poses(i,j) = buff[ cols * i + j];

  return poses;
}

Eigen::Matrix4f tran_para(Eigen::MatrixXf &Poses, const int &index)
{
  Eigen::Matrix<float, 1, 12> pose_row = Poses.row(index);
  Eigen::Matrix4f pose;
  pose.row(0) = pose_row.head(4);
  pose.row(1) = pose_row.segment(4,4);
  pose.row(2) = pose_row.segment(8,4);
  pose.row(3) << 0, 0, 0, 1;
  return pose;
}


void FeatureExtraction (DP &PointCloudRead)
{

}

int main(int argc, char *argv[])
{
//  string bin_path = "/media/robin/Harddisk_thesis/Odometry_Benchmark/dataset/sequences/00/velodyne/000000.bin";
//  string read_path = "/media/robin/Harddisk_thesis/Odometry_Benchmark/dataset/sequences/00/pcd/000000.pcd";
//  pcl::PointCloud<pcl::PointXYZI>::Ptr points (new pcl::PointCloud<pcl::PointXYZI>);
//  DP PMpoints;
//  loadBINFile(bin_path, points);
//  PMpoints = loadedPoints(points);
//  const DP read(PMIO::loadPCD(read_path));

  // Read the frames from the pcd folder (ICP)
  string pcd_path = "/media/robin/Harddisk_thesis/Odometry_Benchmark/dataset/sequences/00/pcd/";
//  string pcd_path = "/home/robin/test/";  // for test
  read_filelists( pcd_path, pcd_file_lists, "pcd" );

  // Read the frames from the bin folder (Viz) (expected)
  string bin_path = "/media/robin/Harddisk_thesis/Odometry_Benchmark/dataset/sequences/00/velodyne/";
//  string bin_path = "/home/robin/test/";  // for test
  read_filelists( bin_path, bin_file_lists, "bin" );

  // Read the ground truth (Viz)
  string gt_path = "/media/robin/Harddisk_thesis/Odometry_Benchmark/dataset/poses/00.txt";
//  string gt_path = "/home/robin/test/poses/00.txt";

  // Load the transformationparameter of ground truth
  Eigen::MatrixXf Pose = LoadPoses(gt_path);

  // Set the first ref point cloud (ICP)
  string refer_path = pcd_path + pcd_file_lists[0];
  const DP ref(PMIO::loadPCD(refer_path));

  // Load the configuration file for ICP
  string configFile = "/home/robin/lidarDirectionalSLAM/2_cpp/icp_config.yaml";

  // Initialize the Visualizer for LiDAR point cloud (Viz)
  pcl::visualization::PCLVisualizer viewer;
  viewer.setWindowName("KITTI LiDAR Visualization");
  viewer.setShowFPS(0);
  viewer.setPosition(10,10);
  viewer.setSize(480, 320);
  viewer.addCoordinateSystem(10.0);
  viewer.setBackgroundColor (0, 0, 0);
  viewer.setCameraPosition(-100, 0, 50, 0, 0, 1, 0);  // set the viewpoint

  // Initialize the point for trajectory (ICP)
//  pcl::PointCloud<pcl::PointXYZ>::Ptr ego_motion (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointXYZ pre_point, cur_point, pre_point_gt, cur_point_gt;
  pre_point.x = pre_point.y = pre_point.z =
      pre_point_gt.x = pre_point_gt.y = pre_point_gt.z = 0;
  Eigen::Vector4f pre_pos(pre_point.x, pre_point.y, pre_point.z, 1);
  Eigen::Vector4f pre_pos_gt = pre_pos;
  Eigen::Vector4f init_pos = pre_pos;
//  ego_motion->push_back(pre_point);

  // Initialize the Visualizer for trajectory (ICP)
  pcl::visualization::PCLVisualizer traj_viewer;
  traj_viewer.setWindowName("KITTI Odometry Visualization");
  traj_viewer.setPosition(600,10);
  traj_viewer.setShowFPS(0);
  traj_viewer.setSize(480, 320);
  traj_viewer.addCoordinateSystem(5.0);
  traj_viewer.setBackgroundColor (0, 0, 0);
  traj_viewer.setCameraPosition(0, -50, 0, 0, 10, 10, 0);  // set the viewpoint

  for (int i = 0; i < bin_file_lists.size(); ++i)
  {
    // Set the timer
    clock_t start, t_load_bin, t_addpc, t_load_pcd, t_icp, finish;
    start = clock();

    // Set the path to byte file for each iteration (Viz)
    string bin_file = bin_path + bin_file_lists[i];
    string frame_index = bin_file_lists[i].substr(0, bin_file_lists[i].length() - 4);

    // Load the byte file from byte file
    pcl::PointCloud<pcl::PointXYZI>::Ptr points (new pcl::PointCloud<pcl::PointXYZI>);

    loadBINFile(bin_file, points);
    t_load_bin = clock();
    cout << "Load the bin file " << bin_file_lists[i] << " in " <<
        ((float)t_load_bin-start)/CLOCKS_PER_SEC << " s" << endl;

    // Visualize the LiDAR point cloud (Viz)
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>
        intensity_distribution(points, "intensity");  // Set the intensity of point cloud
    //  viewer.addPointCloud<pcl::PointXYZI>(points, "cloud"); // without intensity
    viewer.addPointCloud<pcl::PointXYZI>(points, intensity_distribution, "cloud");
    string cloud_title = "Frame " + frame_index;
    viewer.addText(cloud_title, 50, 430, 12, 1, 1, 1, "title"); // RGB [0, 1]
    viewer.spinOnce(1);  // in ms
    t_addpc = clock();
    cout << "Display the frame " << frame_index << " in " <<
        ((float)t_addpc-t_load_bin)/CLOCKS_PER_SEC << " s" << endl;

    // Load the reading pcd file for each iteration (ICP)
    string read_path = pcd_path + pcd_file_lists[i];
    const DP read(PMIO::loadPCD(read_path));
    t_load_pcd = clock();
    cout << "Successfully loaded the pcd file " << pcd_file_lists[i] << " in "
         << ((float)t_load_pcd-t_load_bin)/CLOCKS_PER_SEC << " s." << endl;

    // ICP except for the first loop
    if(i > 0)
    {
      // Create the default ICP algorithm (ICP)
      PM::ICP icp;
      PointMatcherSupport::Parametrizable::Parameters params;
      std::string name;

      // load YAML config
      ifstream ifs(configFile.c_str());
      icp.loadFromYaml(ifs);


/*
      // See the implementation of setDefault() to create a custom ICP algorithm
      // icp.setDefault();

      // Uncomment for console outputs
      setLogger(PM::get().LoggerRegistrar.create("FileLogger"));

//       Prepare reading filters
//        name = "MaxDistDataPointsFilter";
//        params["maxDist"] = "30.0";
//        PM::DataPointsFilter* maxDist_read =
//          PM::get().DataPointsFilterRegistrar.create(name, params);
//        params.clear();

//        name = "MinDistDataPointsFilter";
//        params["minDist"] = "8.0";
//        PM::DataPointsFilter* minDist_read =
//          PM::get().DataPointsFilterRegistrar.create(name, params);
//        params.clear();

//        name = "SurfaceNormalDataPointsFilter";
//        params["knn"] = "20.0";
//        params["keepDensities"] = "1";
//        PM::DataPointsFilter* sur_read =
//          PM::get().DataPointsFilterRegistrar.create(name,params);
//        params.clear();


        name = "SamplingSurfaceNormalDataPointsFilter";
        params["ratio"] = "0.1";
        params["knn"] = "10";
        PM::DataPointsFilter* rand_read =
          PM::get().DataPointsFilterRegistrar.create(name, params);
        params.clear();

        // Prepare reference filters
//        name = "MaxDistDataPointsFilter";
//        params["maxDist"] = "30.0";
//        PM::DataPointsFilter* maxDist_ref =
//          PM::get().DataPointsFilterRegistrar.create(name, params);
//        params.clear();

//        name = "MinDistDataPointsFilter";
//        params["minDist"] = "8.0";
//        PM::DataPointsFilter* minDist_ref =
//          PM::get().DataPointsFilterRegistrar.create(name, params);
//        params.clear();

//        name = "SurfaceNormalDataPointsFilter";
//        params["knn"] = "20.0";
//        params["keepDensities"] = "1";
//        PM::DataPointsFilter* sur_ref =
//          PM::get().DataPointsFilterRegistrar.create(name,params);
//        params.clear();

        name = "SamplingSurfaceNormalDataPointsFilter";
        params["ratio"] = "0.1";
        params["knn"] = "10";
        PM::DataPointsFilter* rand_ref =
          PM::get().DataPointsFilterRegistrar.create(name, params);
        params.clear();

        // Prepare matching function
        name = "KDTreeMatcher";
        params["knn"] = "3";
        params["epsilon"] = "1";
        params["maxDist"] = "2";
        PM::Matcher* kdtree =
          PM::get().MatcherRegistrar.create(name, params);
        params.clear();

        // Prepare outlier filters
//        name = "TrimmedDistOutlierFilter";
//        params["ratio"] = "0.8";
//        PM::OutlierFilter* trimdist =
//          PM::get().OutlierFilterRegistrar.create(name, params);
//        params.clear();

        name = "MaxDistOutlierFilter";
        params["maxDist"] = "1";
        PM::OutlierFilter* maxdist =
          PM::get().OutlierFilterRegistrar.create(name, params);
        params.clear();

        name = "SurfaceNormalOutlierFilter";
        params["maxAngle"] = "1";
        PM::OutlierFilter* surnorm =
          PM::get().OutlierFilterRegistrar.create(name, params);
        params.clear();

        // Prepare error minimization
        name = "PointToPlaneErrorMinimizer";
        PM::ErrorMinimizer* pointToPlane =
          PM::get().ErrorMinimizerRegistrar.create(name);

        // Prepare transformation checker filters
        name = "CounterTransformationChecker";
        params["maxIterationCount"] = "80";
        PM::TransformationChecker* maxIter =
          PM::get().TransformationCheckerRegistrar.create(name, params);
        params.clear();

        name = "DifferentialTransformationChecker";
        params["minDiffRotErr"] = "0.001";
        params["minDiffTransErr"] = "0.01";
//        params["smoothLength"] = "4";
        PM::TransformationChecker* diff =
          PM::get().TransformationCheckerRegistrar.create(name, params);
        params.clear();

        name = "BoundTransformationChecker";
        params["maxRotationNorm"] = "0.8";
        params["maxTranslationNorm"] = "15";
        PM::TransformationChecker* bound =
          PM::get().TransformationCheckerRegistrar.create(name, params);
        params.clear();

        // Prepare inspector
        PM::Inspector* nullInspect =
          PM::get().InspectorRegistrar.create("NullInspector");

        //name = "VTKFileInspector";
          //	params["dumpDataLinks"] = "1";
          //	params["dumpReading"] = "1";
          //	params["dumpReference"] = "1";

        //PM::Inspector* vtkInspect =
        //	PM::get().InspectorRegistrar.create(name, params);
        params.clear();

        // Prepare transformation
        PM::Transformation* rigidTrans =
          PM::get().TransformationRegistrar.create("RigidTransformation");

        // Build ICP solution
//        icp.readingDataPointsFilters.push_back(maxDist_read);
//        icp.readingDataPointsFilters.push_back(minDist_read);
        icp.readingDataPointsFilters.push_back(rand_read);
//        icp.readingDataPointsFilters.push_back(sur_read);

//        icp.referenceDataPointsFilters.push_back(maxDist_ref);
//        icp.referenceDataPointsFilters.push_back(minDist_ref);
        icp.referenceDataPointsFilters.push_back(rand_ref);
//        icp.referenceDataPointsFilters.push_back(sur_ref);

        icp.matcher.reset(kdtree);

//        icp.outlierFilters.push_back(trimdist);
        icp.outlierFilters.push_back(maxdist);
        icp.outlierFilters.push_back(surnorm);

        icp.errorMinimizer.reset(pointToPlane);

        icp.transformationCheckers.push_back(maxIter);
        icp.transformationCheckers.push_back(diff);
        icp.transformationCheckers.push_back(bound);

        // toggle to write vtk files per iteration
        icp.inspector.reset(nullInspect);
        //icp.inspector.reset(vtkInspect);

        icp.transformations.push_back(rigidTrans);
*/
      // Compute the transformation to express data in ref
      PM::TransformationParameters T = icp(read, ref);

      // Compute the current position of LiDAR sensor
      Eigen::Vector4f cur_pos = T * pre_pos;
      cur_point.x = - cur_pos[1];
      cur_point.y = - cur_pos[2];
      cur_point.z = cur_pos[0];
      cout << cur_point << endl;
      Eigen::Matrix4f T_gt = tran_para(Pose, i);
//      cout << T_gt <<endl;
      Eigen::Vector4f cur_pos_gt = T_gt * init_pos;
      cur_point_gt.x = cur_pos_gt[0];
      cur_point_gt.y = cur_pos_gt[1];
      cur_point_gt.z = cur_pos_gt[2];
      cout << cur_point_gt << endl;
      traj_viewer.addLine(pre_point, cur_point, 1.0, 0.0, 0.0, to_string(i));
      traj_viewer.addLine(pre_point_gt, cur_point_gt, 0.0, 1.0, 0.0, "gt" + to_string(i));
      traj_viewer.spinOnce();  // in ms
      pre_pos = cur_pos;
      pre_point = cur_point;
      pre_pos_gt = cur_pos_gt;
      pre_point_gt = cur_point_gt;

//      cout << "Final transformation:" << endl << T << endl;
      cout << "Current position is " << endl << cur_point << endl;
      cout << "Ground Truth position is" << endl << cur_point_gt <<endl;

      t_icp = clock();
      cout << "Finished ICP calculation in " <<
              ((float)t_icp-t_load_pcd)/CLOCKS_PER_SEC << " s." << endl;
    }

    viewer.removePointCloud("cloud");
    viewer.removeText3D("title");

     const DP ref = read;

    finish = clock();
    cout << "Time for this iteration is " <<
            ((float)finish-start)/CLOCKS_PER_SEC << " s." << endl << endl;
  }
  viewer.close();
//  traj_viewer.close();
  cout << "Finished!" << endl;
  return 0;
}

void usage(int argc, char *argv[])
{
  if (!(argc == 3))
  {
    cerr << "Usage " << argv[0] << " [CONFIG.yaml] INPUT.csv/.vtk/.pcd OUTPUT.csv/.vtk/.pcd" << endl;
    cerr << "Example:" << endl;
    cerr << argv[0] << " ./config.yaml ./test/000000.pcd ./test/000001.pcd" << endl << endl;
    abort();
  }
}
