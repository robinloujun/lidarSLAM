#include <iostream>
#include <list>
#include <lidar_directional_slam/utils/voxel_integrator.h>

namespace mapping_utils
{

voxelIntegrator::voxelIntegrator(layer<voxel>* layer)
{
  setLayer(layer);
  if (integrator_threads_ == 0)
  {
    std::cerr << "Automatic core count failed, defaulting to 1 threads"
              << std::endl;
    integrator_threads_ = 1;
  }
}

void voxelIntegrator::setLayer(layer<voxel>* layer)
{
  layer_ = layer;

  voxel_size_ = layer_->voxel_size();
  block_size_ = layer_->block_size();
  voxels_per_side_ = layer_->voxels_per_side();

  voxel_size_inv_ = 1.0 / voxel_size_;
  block_size_inv_ = 1.0 / block_size_;
  voxels_per_side_inv_ = 1.0 / voxels_per_side_;
}

void voxelIntegrator::integratePointCloud(const transformationMatrix& T,
                                          const pointCloud& points)
{
  // Create a hash map: voxel index -> index in point cloud
  longIndexHashMapType<alignedVector<size_t>>::type voxel_grid_map;

  integratorIndex index_getter(points.size());

  setVoxelGridMap(T, points, &index_getter, &voxel_grid_map);

  updateVoxelGridMap(T, points, &voxel_grid_map);
}

void voxelIntegrator::setVoxelGridMap(
    const transformationMatrix& T, const pointCloud& points,
    integratorIndex* index_getter,
    longIndexHashMapType<alignedVector<size_t>>::type* voxel_grid_map)
{
  if (index_getter == nullptr)
    std::cerr << "Failure out of index_getter nullptr!";
  if (voxel_grid_map == nullptr)
    std::cerr << "Failure out of voxel_grid_map nullptr!";

  size_t point_idx;
  while (index_getter->getNextIndex(&point_idx))
  {
    const point& currentPoint = points[point_idx];
    const point pointGlobalCoordinate =
        getPointFromHomogeneous(T * getHomogeneousPoint(currentPoint));

    globalIndex voxel_idx = getGridIndexFromOriginPoint<globalIndex>(
        pointGlobalCoordinate, voxel_size_inv_);

    (*voxel_grid_map)[voxel_idx].push_back(point_idx);
  }
}

void voxelIntegrator::updateVoxelGridMap(
    const transformationMatrix& T, const pointCloud& points,
    longIndexHashMapType<alignedVector<size_t>>::type* voxel_grid_map)
{
  if (integrator_threads_ == 1)
  {
    constexpr size_t thread_idx = 0;
    integrateVoxels(T, points, voxel_grid_map, thread_idx);
  }
  else
  {
    std::list<std::thread> integration_threads;
    for (size_t i = 0; i < integrator_threads_; ++i)
    {
      integration_threads.emplace_back(&voxelIntegrator::integrateVoxels, this,
                                       T, points, voxel_grid_map, i);
    }
    for (std::thread& thread : integration_threads)
    {
      thread.join();
    }
  }
}

void voxelIntegrator::integrateVoxels(
    const transformationMatrix& T, const pointCloud& points,
    longIndexHashMapType<alignedVector<size_t>>::type* voxel_grid_map,
    size_t thread_idx)
{
  longIndexHashMapType<alignedVector<size_t>>::type::const_iterator it;
  size_t map_size;
  it = voxel_grid_map->begin();
  map_size = voxel_grid_map->size();

  for (size_t i = 0; i < map_size; ++i)
  {
    if (((i + thread_idx + 1) % integrator_threads_) == 0)
    {
      integrateVoxel(T, points, *it, voxel_grid_map);
    }
    ++it;
  }
}

void voxelIntegrator::integrateVoxel(
    const transformationMatrix& T, const pointCloud& points,
    const std::pair<globalIndex, alignedVector<size_t>>& it,
    longIndexHashMapType<alignedVector<size_t>>::type* voxel_grid_map)
{
  if (it.second.empty())
  {
    return;
  }

  const point& pointOrigin = geometry_utils::getTranslation<float>(T);
  point mergedPoint = point::Zero();
  //  float mergedWeight = 1.0;

  for (const size_t point_idx : it.second)
  {
    const point& pointCurrent = points[point_idx];
    mergedPoint = (mergedPoint + pointCurrent) / 2.0;
  }

  const point mergedPointInGlobal =
      getPointFromHomogeneous(T * getHomogeneousPoint(mergedPoint));

  globalIndex globalVoxelIndex;
  while (1)
  {
    block<voxel>::Ptr block = nullptr;
    blockIndex block_idx;
    voxel* voxel =
        allocateStorageAndGetVoxelPtr(globalVoxelIndex, &block, &block_idx);

    updateVoxel(pointOrigin, mergedPointInGlobal, globalVoxelIndex, voxel);
  }
}

void voxelIntegrator::updateVoxel(const point& origin_point,
                                  const point& point_in_global,
                                  const globalIndex& global_voxel_idx,
                                  voxel* voxel)
{
  if (voxel == nullptr)
    std::cerr << "Failure out of voxel nullptr";

  const point voxelCenter =
      getCenterPointFromGridIndex(global_voxel_idx, voxel_size_);

  // Lookup the mutex that is responsible for this voxel and lock it
  std::lock_guard<std::mutex> lock(mutexes_.get(global_voxel_idx));

  voxel->point_position =
      voxel::runningAverage(voxel->point_position, point_in_global);
}

voxel* voxelIntegrator::allocateStorageAndGetVoxelPtr(
    const globalIndex& global_voxel_idx, block<voxel>::Ptr* last_block,
    blockIndex* last_block_idx)
{
  if ((last_block == nullptr) || (last_block_idx == nullptr))
    std::cerr << "Failure out of block nullptr!";

  const blockIndex block_idx =
      getBlockIndexFromGlobalVoxelIndex(global_voxel_idx, voxels_per_side_inv_);

  if ((block_idx != *last_block_idx) || (*last_block == nullptr))
  {
    *last_block = layer_->getBlockPtrByIndex(block_idx);
    *last_block_idx = block_idx;
  }

  // If no block at this location currently exists, we allocate a temporary
  // voxel that will be merged into the map later
  if (*last_block == nullptr)
  {
    std::lock_guard<std::mutex> lock(temp_block_mutex_);
    typename layer<voxel>::blockHashMap::iterator it =
        temp_block_map_.find(block_idx);
    if (it != temp_block_map_.end())
    {
      *last_block = it->second;
    }
    else
    {
      auto insert_status = temp_block_map_.emplace(
          block_idx, std::make_shared<block<voxel>>(
                         voxels_per_side_, voxel_size_,
                         getOriginPointFromGridIndex(block_idx, block_size_)));
      if (!insert_status.second)
        std::cerr << "Block already exists when allocating at "
                  << block_idx.transpose();
      *last_block = insert_status.first->second;
    }
  }

  (*last_block)->updated() = true;

  const voxelIndex local_voxel_idx =
      getLocalFromGlobalVoxelIndex(global_voxel_idx, voxels_per_side_);

  return &((*last_block)->getVoxelFromVoxelIndex(local_voxel_idx));
}

} // namespace mapping_utils
