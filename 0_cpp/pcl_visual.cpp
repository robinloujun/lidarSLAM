#include <stdio.h>
#include <string>
#include <iostream>
#include <fstream>
#include <dirent.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <Eigen/Dense>

using namespace std;

static vector<string> file_lists;

#define MAXBUFSIZE ((int) 1e6)

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

void loadBINFile(const std::string &file_name, pcl::PointCloud<pcl::PointXYZI>::Ptr& points)
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
    points->push_back(point);
  }
  input.close();
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

int main()
{
  /*
  // Read the frames from the pcd folder
  string pcd_path = "/media/robin/Harddisk_thesis/Odometry_Benchmark/dataset/sequences/00/pcd/";
  //string pcd_path = "/home/robin/test/";
  read_filelists( pcd_path, file_lists, "pcd" );
  */

  // Read the frames from the bin folder
  string bin_path = "/media/robin/Harddisk_thesis/Odometry_Benchmark/dataset/sequences/00/velodyne/";
  read_filelists( bin_path, file_lists, "bin" );

  // Read the ground truth
  string gt_path = "/media/robin/Harddisk_thesis/Odometry_Benchmark/dataset/poses/00.txt";

  // Load the transformationparameter of ground truth
  Eigen::MatrixXf Pose = LoadPoses(gt_path);

  // Initialize the point for trajectory
  pcl::PointXYZ pre_point_gt, cur_point_gt;
  pre_point_gt.x = pre_point_gt.y = pre_point_gt.z = 0;
  Eigen::Vector4f pre_pos_gt(0, 0, 0, 1), init_pos(0, 0, 0, 1);

  // Initialize the Visualizer for point cloud
  pcl::visualization::PCLVisualizer viewer;
  viewer.setWindowName("KITTI LiDAR Visualization");
  viewer.setShowFPS(1);
  viewer.setSize(480, 320);
  viewer.addCoordinateSystem(10.0);
  viewer.setBackgroundColor (0, 0, 0);
  viewer.setCameraPosition(-100, 0, 50, 0, 0, 1, 0);  // set the viewpoint

  // Initialize the Visualizer for trajectory (ICP)
  pcl::visualization::PCLVisualizer traj_viewer;
  traj_viewer.setWindowName("KITTI Odometry Visualization");
  traj_viewer.setPosition(520,10);
  traj_viewer.setShowFPS(0);
  traj_viewer.setSize(480, 320);
  traj_viewer.addCoordinateSystem(5.0);
  traj_viewer.setBackgroundColor (0, 0, 0);
  traj_viewer.setCameraPosition(0, -50, 0, 0, 10, 10, 0);  // set the viewpoint

  // Display the point cloud sequent sequentially
  for (int i = 0; i < file_lists.size(); ++i)
  {
    string bin_file = bin_path + file_lists[i];
    string frame_index = file_lists[i].substr(0, file_lists[i].length() - 4);
    cout << "Display the frame " << frame_index << endl;
    pcl::PointCloud<pcl::PointXYZI>::Ptr points (new pcl::PointCloud<pcl::PointXYZI>);

    // Load the point cloud from bytefile
    loadBINFile(bin_file, points);

    // Set the intensity and visualize the point cloud
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_distribution(points, "intensity");
    //    viewer.addPointCloud<pcl::PointXYZI>(points, "cloud");
    viewer.addPointCloud<pcl::PointXYZI>(points, intensity_distribution, "cloud");
    string cloud_title = "Frame " + frame_index;
    viewer.addText(cloud_title, 50, 430, 12, 1, 1, 1, "title"); // RGB [0, 1]
    viewer.spinOnce(1);  // in ms
    viewer.removePointCloud("cloud");
    viewer.removeText3D("title");

    if(i > 0)
    {
      // Compute the current position of LiDAR sensor
      cout << "before tran_para"<< endl;
      Eigen::Matrix4f T_gt = tran_para(Pose, i);
      cout << T_gt <<endl;
      Eigen::Vector4f cur_pos_gt = T_gt * init_pos;
      cur_point_gt.x = cur_pos_gt[0];
      cur_point_gt.y = cur_pos_gt[1];
      cur_point_gt.z = cur_pos_gt[2];
      cout << cur_point_gt << endl;
      traj_viewer.addLine(pre_point_gt, cur_point_gt, 1.0, 0.0, 0.0, to_string(i));
      traj_viewer.spinOnce(1);  // in ms
      pre_pos_gt = cur_pos_gt;
      pre_point_gt = cur_point_gt;
    }


  }
  // Display frames in terminal
  cout << "Number of frames: " << file_lists.size() << endl;
  viewer.close();
  cout << "exit!" << endl;

  return 0;
}
