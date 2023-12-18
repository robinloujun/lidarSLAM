#include <string>
#include <iostream>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <ctime>
#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>

using namespace std;
using namespace pcl;

static vector<string> seq_lists;
static vector<string> file_lists;

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
}

bool computePairNum(std::string pair1,std::string pair2)
{
  return pair1 < pair2;
}

void sort_filelists(std::vector<std::string>& filists,std::string type)
{
  if (filists.empty())return;

  std::sort(filists.begin(),filists.end(),computePairNum);
}

// source: GitHub - [yanii/kitti-pcl] https://github.com/yanii/kitti-pcl/blob/master/src/kitti2pcd.cpp

void bin2pcd(std::string &infile, std::string& outfile)
{
  // load point cloud
  fstream input(infile.c_str(), ios::in | ios::binary);
  if(!input.good())
  {
    cerr << "Could not read file: " << infile << endl;
    exit(EXIT_FAILURE);
  }
  input.seekg(0, ios::beg);

  pcl::PointCloud<pcl::PointXYZI>::Ptr points (new pcl::PointCloud<pcl::PointXYZI>);

  int i;
  for (i=0; input.good() && !input.eof(); i++)
  {
    PointXYZI point;
    input.read((char *) &point.x, 3*sizeof(float));
    input.read((char *) &point.intensity, sizeof(float));
    points->push_back(point);
  }
  input.close();

  cout << "Read KTTI point cloud with " << i << " points, writing to " << outfile << endl;
  PCDWriter writer;

  // Save DoN features
  writer.write< PointXYZI > (outfile, *points, false);
}

inline bool test_exist(const string &file_name){
  struct stat buffer;
  return (stat (file_name.c_str(), &buffer) == 0);
}

int main(int argc, char **argv)
{
  cout << "Start Converting..." << endl;
  string dataset_path = "/media/robin/Harddisk_thesis/Odometry_Benchmark/dataset/sequences/";

  read_filelists( dataset_path, seq_lists, "" );
  sort_filelists( seq_lists, "" );

  for (int j = 0; j < seq_lists.size(); ++j)
  {
    string pcd_path = dataset_path +  seq_lists[j] + "/pcd/";
    if ( !test_exist(pcd_path) )
    {
      const char* mkdir_pcd_path  = pcd_path.c_str();
      mkdir(mkdir_pcd_path, 0777);
      cout << "Built a folder named pcd to save the pcd files in "
           << pcd_path << " for sequence " << seq_lists[j] << endl;
    }

    string bin_path = dataset_path + seq_lists[j] + "/velodyne/";
    read_filelists( bin_path, file_lists, "bin" );
    sort_filelists( file_lists, "bin" );

    for (int i = 0; i < file_lists.size(); ++i)
    {
        string bin_file = bin_path + file_lists[i];
        string tmp_str = file_lists[i].substr(0, file_lists[i].length() - 4) + ".pcd";
        string pcd_file = pcd_path + tmp_str;
        if ( !test_exist(pcd_file) )
        {
          bin2pcd( bin_file, pcd_file );
        }
    }
    cout << "Finished converting the data of sequence " << seq_lists[j] << " with " << file_lists.size() + 1 << " frames" << endl;
  }
  return 0;
}
