#ifndef POINT_MAP_H
#define POINT_MAP_H

#include <memory>
#include <vector>
#include <Eigen/Core>

#include <lidar_directional_slam/utils/basic.h>
#include <pointmatcher/PointMatcher.h>

namespace mapping_utils
{
template <typename T>
Eigen::Matrix<T, 3, 1> point;

template <typename T>
struct PointCloud
{
  struct Point
  {
    T  x,y,z;
  };

  std::vector<Point>  pts;

  // Must return the number of data points
  inline size_t kdtree_get_point_count() const { return pts.size(); }
  inline size_t getNumPoints() const { return pts.size(); }

  // Returns the dim'th component of the idx'th point in the class:
  // Since this is inlined and the "dim" argument is typically an immediate value, the
  //  "if/else's" are actually solved at compile time.
  inline T kdtree_get_pt(const size_t idx, const size_t dim) const
  {
    if (dim == 0) return pts[idx].x;
    else if (dim == 1) return pts[idx].y;
    else return pts[idx].z;
  }
};

class pointMap
{


  bool hasMap() const { return (mapPointCloud.getNumPoints() != 0); }

protected:
  PointMatcher<float>::DataPoints mapPointCloudDP;
  PointCloud<float> mapPointCloud;
};

} // namespace mapping_utils

#endif // POINT_MAP_H
