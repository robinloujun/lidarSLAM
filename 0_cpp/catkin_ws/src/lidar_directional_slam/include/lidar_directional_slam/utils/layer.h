#pragma once

#ifndef LAYER_H
#define LAYER_H

#include <lidar_directional_slam/utils/basic.h>
#include <lidar_directional_slam/utils/voxel.h>
#include <lidar_directional_slam/utils/block.h>
#include <lidar_directional_slam/utils/voxel_block_hash.h>

namespace mapping_utils
{
/** \brief
 * Layer: a container to store type of voxel_type in blocks.
 */
template <typename voxel_type> class layer
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef std::shared_ptr<layer> Ptr;
  typedef block<voxel_type> blockType;
  typedef typename hashMapType<typename blockType::Ptr>::type blockHashMap;
  typedef typename std::pair<blockIndex, typename blockType::Ptr> blockMapPair;

  explicit layer(float voxel_size, size_t voxels_per_side)
      : voxel_size_(voxel_size), voxels_per_side_(voxels_per_side)
  {
    if (voxel_size_ < 0)
      std::cerr << "Size of voxel must be bigger than zero.";
    voxel_size_inv_ = 1.0 / voxel_size_;
  }

  layer() {}

  inline typename blockType::Ptr getBlockPtrByIndex(const blockIndex& index)
  {
    typename blockHashMap::iterator it = block_map_.find(index);
    if (it != block_map_.end())
    {
      return it->second;
    }
    else
    {
      return typename blockType::Ptr();
    }
  }

  float block_size() const { return block_size_; }
  float block_size_inv() const { return block_size_inv_; }
  float voxel_size() const { return voxel_size_; }
  float voxel_size_inv() const { return voxel_size_inv_; }
  size_t voxels_per_side() const { return voxels_per_side_; }
  float voxels_per_side_inv() const { return voxels_per_side_inv_; }

protected:
  float voxel_size_;
  float block_size_;
  size_t voxels_per_side_;

  float voxel_size_inv_;
  float block_size_inv_;
  float voxels_per_side_inv_;

  blockHashMap block_map_;
};

} // namespace mapping_utils

#endif // LAYER_H
