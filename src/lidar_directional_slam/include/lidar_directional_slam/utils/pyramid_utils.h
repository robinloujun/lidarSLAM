#pragma once

#ifndef PYRAMID_UTILS_H
#define PYRAMID_UTILS_H

#include <fstream>
#include <lidar_directional_slam/utils/basic.h>

// namespace lidarslam
//{

template <typename T> class pyramid
{
public:
  using Ptr = std::shared_ptr<pyramid<T>>;

  IMPORT_TYPES(T)

  pyramid() : cloud1(), cloud2(), cloud3() {}
  pyramid(DPPtr cloud) : cloud1(*cloud), cloud2(*cloud), cloud3(*cloud) {}

public:
  void setCloud(DPPtr cloud)
  {
    cloud1 = *cloud;
    cloud2 = *cloud;
    cloud3 = *cloud;
  }

  DP getFirstLayer() { return cloud1; }
  DP getSecondLayer() { return cloud2; }
  DP getThirdLayer() { return cloud3; }

  void setFirstFilter(const std::string& config)
  {
    std::ifstream ifs(config);
    inputFilter1 = DataPointsFilters(ifs);
  }

  void setSecondFilter(const std::string& config)
  {
    std::ifstream ifs(config);
    inputFilter2 = DataPointsFilters(ifs);
  }

  void setThirdFilter(const std::string& config)
  {
    std::ifstream ifs(config);
    inputFilter3 = DataPointsFilters(ifs);
  }

  void applyFilters()
  {
    inputFilter1.apply(cloud1);
    inputFilter2.apply(cloud2);
    inputFilter3.apply(cloud3);
  }

  bool hasMap() { return (icp1.hasMap() && icp2.hasMap() && icp3.hasMap()); }

  void setInitMap()
  {
    icp1.setMap(cloud1);
    icp2.setMap(cloud2);
    icp3.setMap(cloud3);
  }

  void setMap(DP c1, DP c2, DP c3)
  {
    icp1.setMap(c1);
    icp2.setMap(c2);
    icp3.setMap(c3);
  }

private:
  // Three layers containing pointers to point cloud
  DP cloud1;
  DP cloud2;
  DP cloud3;

  // Three layers for filters
  DataPointsFilters inputFilter1;
  DataPointsFilters inputFilter2;
  DataPointsFilters inputFilter3;

public:
  // Three layers for ICP
  ICPSequence icp1;
  ICPSequence icp2;
  ICPSequence icp3;
};

//} // namespace lidarslam

#endif // PYRAMID_UTILS_H
