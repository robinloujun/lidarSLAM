#include <Eigen/Core>

#include <lidar_directional_slam/utils/point_map.h>

namespace mapping_utils
{
bool pointMap::hasMap() const
{
  return (mapPointCloud.size() != 0);
}

} // namespace mapping_utils
