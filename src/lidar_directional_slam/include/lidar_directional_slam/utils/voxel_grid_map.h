#pragma once

#ifndef VOXEL_GRID_MAP_H
#define VOXEL_GRID_MAP_H

#include <lidar_directional_slam/utils/basic.h>
#include <lidar_directional_slam/utils/voxel.h>
#include <lidar_directional_slam/utils/layer.h>

namespace mapping_utils
{

/** \brief
 * Represent the world model as a hash of voxel blocks
 * */
class voxelGridMap
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef std::shared_ptr<voxelGridMap> Ptr;

  float voxel_size_ = 0.2f;
  size_t voxels_per_side_ = 8u;

  explicit voxelGridMap():
    layer_(new layer<voxel>(voxel_size_, voxels_per_side_))
  {
    block_size_ = voxel_size_ * voxels_per_side_;
  }

//  explicit voxelGridMap(const layer<voxel>& layer):
//    voxelGridMap(std::allocate_shared<layer<voxel>>(Eigen::aligned_allocator<std::remove_const<Type>::type>(),
//                                                    std::forward<layer<voxel>>(layer))) {};

  explicit voxelGridMap(layer<voxel>::Ptr layer):
    layer_(layer)
  {
    if (!layer)
    {
      throw std::runtime_error(std::string("Null layer<voxel>::Ptr in voxelGridMap constructor"));
    }
    block_size_ = layer->block_size();
  }

  //  std::unordered_map<ulong, > map_points;

  layer<voxel>* getLayerPtr() {return layer_.get();}
  const layer<voxel>& getLayer() const {return *layer_;}

  float getBlockSize() const {return block_size_;}
  float getVoxelSize() const {return voxel_size_;}

protected:
  float block_size_;
  layer<voxel>::Ptr layer_;
}; // class voxelGridMap

} // namespace mapping_utils

#endif // VOXEL_GRID_MAP_H
