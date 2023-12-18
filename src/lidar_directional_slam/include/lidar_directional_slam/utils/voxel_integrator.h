#pragma once

#ifndef VOXEL_INTEGRATOR_H
#define VOXEL_INTEGRATOR_H

#include <thread>
#include <mutex>

#include <lidar_directional_slam/utils/geometry_utils.h>
#include <lidar_directional_slam/utils/basic.h>
#include <lidar_directional_slam/utils/voxel.h>
#include <lidar_directional_slam/utils/voxel_block_hash.h>
#include <lidar_directional_slam/utils/layer.h>
#include <lidar_directional_slam/utils/integrator_utils.h>
#include <lidar_directional_slam/utils/approx_hash_array.h>

namespace mapping_utils
{

typedef geometry_utils::transformationMatrix transformationMatrix;

/** \brief
 * VoxelIntegratorBase: abstract class for voxel integrators
 * It takes in the point cloud and the pose to update the
 * information in the voxel layer
 */
class voxelIntegrator
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef std::shared_ptr<voxelIntegrator> Ptr;

  voxelIntegrator(layer<voxel>* layer);

  void setLayer(layer<voxel>* layer);

  void integratePointCloud(const transformationMatrix& T,
                           const pointCloud& points);

  void setVoxelGridMap(
      const transformationMatrix& T, const pointCloud& points,
      integratorIndex* index,
      longIndexHashMapType<alignedVector<size_t>>::type* voxel_grid_map);

  void updateVoxelGridMap(
      const transformationMatrix& T, const pointCloud& points,
      longIndexHashMapType<alignedVector<size_t>>::type* voxel_grid_map);

  void integrateVoxels(
      const transformationMatrix& T, const pointCloud& points,
      longIndexHashMapType<alignedVector<size_t>>::type* voxel_grid_map,
      size_t thread_idx);

  void integrateVoxel(
      const transformationMatrix& T, const pointCloud& points,
      const std::pair<globalIndex, alignedVector<size_t>>& it,
      longIndexHashMapType<alignedVector<size_t>>::type* voxel_grid_map);

  void updateVoxel(
      const point& origin_point,
      const point& point_in_global,
      const globalIndex& global_voxel_idx,
      voxel* voxel);

  voxel* allocateStorageAndGetVoxelPtr(const globalIndex& global_voxel_idx,
                                       block<voxel>::Ptr* last_block,
                                       blockIndex* last_block_idx);

protected:
  size_t integrator_threads_ = std::thread::hardware_concurrency();
  //  bool use_const_weight_ = true;
  layer<voxel>* layer_;

  float voxel_size_;
  size_t voxels_per_side_;
  float block_size_;

  // Derived types.
  float voxel_size_inv_;
  float voxels_per_side_inv_;
  float block_size_inv_;

  std::mutex temp_block_mutex_;

  layer<voxel>::blockHashMap temp_block_map_;

  approxHashArray<12, std::mutex, globalIndex, LongIndexHash> mutexes_;
};

} // namespace mapping_utils

#endif // VOXEL_INTEGRATOR_H
