#include "pointmatcher/IO.h"
#include "pointmatcher/PointMatcher.h"
#include "pointmatcher/PointMatcher.h"
#include "pcl/io/pcd_io.h"  // Load PCD
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h> //Visualizer
#include <pcl/filters/random_sample.h> // Random filter
#include <pcl/filters/voxel_grid.h> // Voxel Grid filter
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/gicp.h> // GICP
#include <boost/filesystem.hpp>
#include <Eigen/Dense>
#include <cassert>
#include <fstream>
#include <iostream>
#include <algorithm>
//#include <dirent.h>
#include <ctime>

using namespace std;

typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;
typedef PM::Parameters Parameters;
typedef PointMatcherIO<float> PMIO;

static vector<string> pcd_file_lists;

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

Eigen::MatrixXf LoadPoses(const std::string& filename)
{
  int cols = 0, rows = 0;
  double buff[(int) 1e6];

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

Eigen::Matrix<float, 1, 12> tran_pose(Eigen::Matrix4f &T)
{
  Eigen::Matrix<float, 1, 12> pose;
  pose.head(4) = T.row(0);
  pose.segment(4,4) = T.row(1);
  pose.segment(8,4) = T.row(2);
  return pose;
}

// simpler main function for ICP with libpointmatcher
//int main(int argc, const char *argv[])
//{
//    string pcd1 = "/media/robin/Harddisk_thesis/Odometry_Benchmark/dataset/sequences/00/pcd/000000.pcd";
//    string pcd2 = "/media/robin/Harddisk_thesis/Odometry_Benchmark/dataset/sequences/00/pcd/000001.pcd";
//    string configFile = "/home/robin/lidarDirectionalSLAM/2_cpp/icp_config_mendes.yaml";

//    DP frame1(PMIO::loadPCD(pcd1));
//    DP frame2(PMIO::loadPCD(pcd2));
//}


/*
// main function for ICP with pcl library
int main(int argc, const char *argv[])
{
    string pcd1 = "/media/robin/Harddisk_thesis/Odometry_Benchmark/dataset/sequences/00/pcd/000000.pcd";
    string pcd2 = "/media/robin/Harddisk_thesis/Odometry_Benchmark/dataset/sequences/00/pcd/000001.pcd";

    pcl::PointCloud<pcl::PointXYZ>::Ptr frame1 (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr frame2 (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr regist (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointptr1 (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointptr2 (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointNormal>::Ptr regist_normal (new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointXYZ> pcl_frame1;

    pcl::io::loadPCDFile(pcd1, *frame1);
    pcl::io::loadPCDFile(pcd2, *frame2);

    pcl_frame1 = *frame1;
    cout << "width: " << pcl_frame1.width << " , height: " << pcl_frame1.height << endl;

    pcl::RandomSample<pcl::PointXYZ> randomFilter;
    randomFilter.setKeepOrganized(1);
    int countPoints1 = static_cast<int>(0.1 * frame1->size());
    randomFilter.setSample(countPoints1);
    randomFilter.setInputCloud(frame1);
    randomFilter.filter(*frame1);

    int countPoints2 = static_cast<int>(0.1 * frame2->size());
    randomFilter.setSample(countPoints2);
    randomFilter.setInputCloud(frame2);
    randomFilter.filter(*frame2);

    pcl_frame1 = *frame1;
    cout << "width: " << pcl_frame1.width << " , height: " << pcl_frame1.height << endl;

    pcl::VoxelGrid<pcl::PointXYZ> voxelGridFilter;
    voxelGridFilter.setLeafSize(0.2, 0.2, 0.2);
    voxelGridFilter.setInputCloud(frame1);
    voxelGridFilter.filter(*frame1);
    voxelGridFilter.setInputCloud(frame2);
    voxelGridFilter.filter(*frame2);

    pcl_frame1 = *frame1;
    cout << "width: " << pcl_frame1.width << " , height: " << pcl_frame1.height << endl;

    cout << "last " << frame2->size() << " to current " << frame1->size() << endl;

    cout << "Finished Filtering" << endl;

    cout << "Compute the Surface Normals" << endl;
    // Compute surface normals and curvature
    pcl::PointCloud<pcl::PointNormal>::Ptr pcl_frame1_normal (new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr pcl_frame2_normal (new pcl::PointCloud<pcl::PointNormal>);

    pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> normal_estimation;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdTree (new pcl::search::KdTree<pcl::PointXYZ> ());
    normal_estimation.setSearchMethod (kdTree);
    normal_estimation.setKSearch (10);

    normal_estimation.setInputCloud (frame1);
    normal_estimation.compute(*pcl_frame1_normal);
    pcl::copyPointCloud (*frame1, *pcl_frame1_normal);

    normal_estimation.setInputCloud (frame2);
    normal_estimation.compute (*pcl_frame2_normal);
    pcl::copyPointCloud (*frame2, *pcl_frame2_normal);

//    copyPointCloud(*frame1, *pointptr1);
//    copyPointCloud(*frame2, *pointptr2);

    clock_t start, diff;
    start = clock();

    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;  // Using Generalized ICP (GICP)
    icp.setTransformationEpsilon(0.0000000001);
    icp.setMaxCorrespondenceDistance(2.0);
    icp.setMaximumIterations(10);
    icp.setRANSACIterations(0);

    icp.setInputSource(frame2);
    icp.setInputTarget(frame1);

    icp.align(*regist);
    Eigen::Matrix4f T = icp.getFinalTransformation();
    std::cout << T << std::endl;

//    pcl::IterativeClosestPointNonLinear<pcl::PointNormal, pcl::PointNormal> icp;
//    icp.setTransformationEpsilon(0.0000000001);
//    icp.setMaxCorrespondenceDistance(2.0);
//    icp.setMaximumIterations(10);

//    icp.setInputSource(pcl_frame2_normal);
//    icp.setInputTarget(pcl_frame1_normal);

//    icp.align(*regist_normal);
//    Eigen::Matrix4f T = icp.getFinalTransformation();
//    std::cout << T << std::endl;

    diff = clock() - start;
    std::cout << ((float)diff)/CLOCKS_PER_SEC << " s." << std::endl;

    // Initialize the Visualizer for point cloud
    pcl::visualization::PCLVisualizer viewer;
    viewer.setWindowName("KITTI LiDAR Visualization");
    viewer.setShowFPS(1);
    viewer.setPosition(1000,10);
    viewer.setSize(960, 640);
    viewer.addCoordinateSystem(10.0);
    viewer.setBackgroundColor (0, 0, 0);
    viewer.setCameraPosition(-100, 0, 50, 0, 0, 1, 0);  // set the viewpoint
//    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_distribution(points, "intensity");
    //    viewer.addPointCloud<pcl::PointXYZI>(points, "cloud");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rgb_green(frame1, 0, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rgb_red(frame2, 255, 0, 0);
    viewer.addPointCloudNormals<pcl::PointXYZ, pcl::PointNormal>(frame1, pcl_frame1_normal, 10, 0.05, "normals");
//    viewer.addPointCloud<pcl::PointNormal>(pcl_frame1_normal, rgb_green, "cloud1");
//    viewer.addPointCloud<pcl::PointNormal>(pcl_frame2_normal, rgb_red, "cloud2");
    viewer.addPointCloud<pcl::PointXYZ>(frame1, rgb_green, "cloud1");
//    viewer.addPointCloud<pcl::PointXYZ>(frame2, rgb_red, "cloud2");
    viewer.spin();
}
*/

///*
// main function for ICP with libpointmatcher
int main(int argc, const char *argv[])
{
    string pcd_path = "/media/robin/Harddisk_thesis/Odometry_Benchmark/dataset/sequences/00/pcd/";
    string configFile = "/home/robin/lidarDirectionalSLAM/2_cpp/icp_config_mendes.yaml";
    string outputBaseFile("test");
    string initTranslation("0,0,0");
    string initRotation("1,0,0;0,1,0;0,0,1");
    string gt_path = "/media/robin/Harddisk_thesis/Odometry_Benchmark/dataset/poses/00.txt";
    Eigen::MatrixXf Poses = LoadPoses(gt_path);

    Eigen::Matrix4f T_world = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f T_velo2gt = Eigen::Matrix4f::Zero();
    T_velo2gt << 0.0, -1.0, 0.0, 0.0,
                 0.0, 0.0, -1.0, 0.0,
                 1.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 1.0;

    read_filelists( pcd_path, pcd_file_lists, "pcd" );
    bool init = false;
    DP frame_current, frame_last;
    PM::ICP icp;
    ifstream ifs(configFile.c_str());
    icp.loadFromYaml(ifs);

    ofstream outfile;
    outfile.open("/home/robin/lidarDirectionalSLAM/2_cpp/poses.txt");

//    for (int i = 0; i < pcd_file_lists.size(); ++i)
    for (int i = 0; i < 2; ++i)
    {
        DP frame_current(PMIO::loadPCD(pcd_path+pcd_file_lists[i]));
//        Eigen::Matrix4f T_gt = tran_para(Poses, i);
        Eigen::Matrix4f T_gt_iter = tran_para(Poses, i) * tran_para(Poses, i-1).inverse();
        if (init)
        {
            PM::Matrix matrix = frame_current.features.topRows(3);
            cout << matrix ;
            cout << matrix.topLeftCorner(3,3) << endl;
            clock_t start, diff;
            start = clock();
            PM::TransformationParameters T_frame2frame = icp(frame_current, frame_last);
            diff = clock() - start;

            cout << "tranfrom result: " << endl << T_frame2frame << endl;
            cout << "The ICP iteration takes " << ((float)diff)/CLOCKS_PER_SEC << " s." << endl;

//            T_world = T_frame2frame * T_world;
//            cout << "The calculated transform for frame " << i << " is" << endl << T_velo2gt * T_world << endl;
//            cout << "The ground truth for frame " << i << " is" << endl << T_gt << endl;
//            Eigen::Matrix4f T_inGT = T_velo2gt * T_world;
//            Eigen::Matrix<float, 1, 12> pose = tran_pose(T_inGT);
//            cout << pose << endl;
////            outfile  << pose << "\n";
//            Eigen::Matrix3f Rot_cal;
//            for (int i = 0; i < 3; i++){for(int j = 0; j < 3; j++){Rot_cal(i,j) = T_frame2frame(i,j);}}
//            Eigen::Matrix3f Rot_gt;
//            for (int i = 0; i < 3; i++){for(int j = 0; j < 3; j++){Rot_gt(i,j) = T_gt_iter(i,j);}}
//            Eigen::Vector3f Trans_cal;
//            Trans_cal << T_frame2frame(0,3), T_frame2frame(1,3), T_frame2frame(2,3);
//            Eigen::Vector3f Trans_gt;
//            Trans_gt << T_gt_iter(0,3), T_gt_iter(1,3), T_gt_iter(2,3);
//            cout << "--------- Rotation ---------" << endl;
//            cout << "rotation calculated: " << endl << Rot_cal << endl;
//            cout << "rotation ground truth " << endl << Rot_gt << endl;
//            cout << "--------- Translation ---------" << endl;
//            cout << "translation calculated: " << endl << Trans_cal << endl;
//            cout << "translation ground truth " << endl << Trans_gt << endl;
        }
        else
        {
            init = true;
//            cout << "The ground truth for frame " << i << " is" << endl << T_gt << endl;
        }

        frame_last = frame_current;
    }

    outfile.close();
    return 0;

}
//*/



