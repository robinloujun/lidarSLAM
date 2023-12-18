#pragma once

#ifndef BLOCK_H
#define BLOCK_H

#include <atomic>
#include <lidar_directional_slam/utils/basic.h>
//#include <lidar_directional_slam/utils/voxel.h>

namespace mapping_utils
{
/** \brief
 * Block: a container holding voxel. Reference: voxblox
 * Contains 3D position and functions for accessing voxels
 */
template <typename voxel_type>
class block
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef std::shared_ptr<block<voxel_type> > Ptr;
  typedef std::shared_ptr<const block<voxel_type> > constPtr;

  block(size_t voxels_per_side, float voxel_size, const point& origin)
      : includes_point_(false),
        voxels_per_side_(voxels_per_side),
        voxel_size_(voxel_size),
        origin_(origin),
        updated_(false)
  {
    num_voxels_ = voxels_per_side_ * voxels_per_side_ * voxels_per_side_;
    voxel_size_inv_ = 1.0 / voxel_size_;
    block_size_ = voxels_per_side_ * voxel_size_;
    block_size_inv_ = 1.0 / block_size_;
    voxels_.reset(new voxel_type[num_voxels_]);
  }

  ~block() {}

  // Index calculation

  // 3D voxel index -> 1D linear index in a block
  inline size_t
  computeLinearIndexFromVoxelIndex(const voxelIndex& voxel_index) const
  {
    size_t linear_index = static_cast<size_t>(
        voxel_index.x() +
        voxels_per_side_ *
            (voxel_index.y() + voxels_per_side_ * voxel_index.z()));
  }

  // 1D linear index -> 3D voxel index in a block
  inline voxelIndex computeVoxelIndexFromLinearIndex(size_t linear_index) const
  {
    voxelIndex voxel_index;
    std::div_t result_div =
        std::div((int)linear_index, voxels_per_side_ * voxels_per_side_);
    voxel_index.z() = result_div.quot;
    int rem = result_div.rem;
    result_div = std::div(rem, voxels_per_side_);
    voxel_index.y() = result_div.quot;
    voxel_index.x() = result_div.rem;
    return voxel_index;
  }

  // Return the center point of voxel.
  inline point computeCoordinatesFromLinearIndex(size_t linear_index) const
  {
    return computeCoordinatesFromVoxelIndex(
        computeVoxelIndexFromLinearIndex(linear_index));
  }

  // Return the center point of voxel.
  inline point
  computeCoordinatesFromVoxelIndex(const voxelIndex& voxel_index) const
  {
    return origin_ + getCenterPointFromGridIndex(voxel_index, voxel_size_);
  }

  // Accessors to actual blocks.
  inline const voxel_type& getVoxelFromLinearIndex(size_t index) const
  {
    return voxels_[index];
  }

  inline const voxel_type& getVoxelPtrFromLinearIndex(size_t index)
  {
    if (index < num_voxels_)
    {
      return voxels_[index];
    }
  }

  inline voxel_type& getVoxelFromVoxelIndex(const voxelIndex& index)
  {
    return voxels_[computeLinearIndexFromVoxelIndex(index)];
  }

  inline const voxel_type& getVoxelPtrFromVoxelIndex(const voxelIndex& index)
  {
    return voxels_[computeLinearIndexFromVoxelIndex(index)];
  }

  inline bool isValidVoxelIndex(const voxelIndex& index) const
  {
    if (index.x() < 0 || index.x() >= static_cast<int>(voxels_per_side_))
    {
      return false;
    }
    if (index.y() < 0 || index.y() >= static_cast<int>(voxels_per_side_))
    {
      return false;
    }
    if (index.z() < 0 || index.z() >= static_cast<int>(voxels_per_side_))
    {
      return false;
    }
    return true;
  }

  inline bool isValidLinearIndex(size_t index) const
  {
    if (index < 0 || index >= num_voxels_)
    {
      return false;
    }
    return true;
  }

  blockIndex getBlockIndex() const
  {
    return getGridIndexFromOriginPoint<blockIndex>(origin_, block_size_inv_);
  }

  size_t voxels_per_side() const { return voxels_per_side_; }

  float voxel_size() const { return voxel_size_; }

  float voxel_size_inv() const { return voxel_size_inv_; }

  size_t num_voxels() const { return num_voxels_; }

  point origin() const { return origin_; }

  void setOrigin(const point& origin) { origin_ = origin; }

  float block_size() const { return block_size_; }

  bool includes_point() const { return includes_point_; }

  bool updated() const { return updated_; }

  std::atomic<bool>& updated() { return updated_; }

  size_t getMemerySize() const
  {
    size_t size = 0u;

    // Calculate size of members
    size += sizeof(voxels_per_side_);
    size += sizeof(voxel_size_);
    size += sizeof(origin_);
    size += sizeof(num_voxels_);
    size += sizeof(voxel_size_inv_);
    size += sizeof(block_size_);

    size += sizeof(includes_point_);
    size += sizeof(updated_);

    if (num_voxels_ > 0u)
    {
      size += (num_voxels_ * sizeof(voxels_[0]));
    }
    return size;
  }

protected:
  std::unique_ptr<voxel_type[]> voxels_;

  size_t num_voxels_;

  // Will be set to true if any voxel in the block received an update.
  bool includes_point_;

private:
  const size_t voxels_per_side_;
  const float voxel_size_;
  point origin_;

  float voxel_size_inv_;
  float block_size_;
  float block_size_inv_;

  // Will be set to true when data is updated.
  std::atomic<bool> updated_;

}; // class block

} // namespace mapping_utils

#endif // BLOCK_H
