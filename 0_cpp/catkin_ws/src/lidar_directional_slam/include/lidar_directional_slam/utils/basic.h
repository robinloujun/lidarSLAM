#pragma once

#ifndef BASIC_H
#define BASIC_H

#include <set>
#include <list>
#include <vector>
#include <memory>
#include <utility>
#include <cstdlib>
#include <unordered_map>
#include <unordered_set>

#include <Eigen/Core>

#include <nabo/nabo.h>
#include <pointmatcher/PointMatcher.h>
#include <pointmatcher/Timer.h>

// Basic unsigned types
using uchar = unsigned char;
using ushort = unsigned short;
using uint = unsigned int;
using ulong = unsigned long;

// Eigen vector types
using Vector3i = Eigen::Vector3i;
using Vector3f = Eigen::Vector3f;
using Vector3d = Eigen::Vector3d;
using Vortor3s = Eigen::Matrix<short, 3, 1>;
using Vector3l = Eigen::Matrix<int64_t, 3, 1>;
using Vector4i = Eigen::Vector4i;
using Vector4f = Eigen::Vector4f;
using Vector4d = Eigen::Vector4d;

//  using alignedVector = std::vector<T, Eigen::aligned_allocator<T>>;

template <typename T> struct Types
{

  // libnabo
  using NNS = typename Nabo::NearestNeighbourSearch<T>;
  using SearchType = typename NNS::SearchType;

  // libpointmatcher
  using PM = PointMatcher<T>;
  using DP = typename PM::DataPoints;
  using DPPtr = std::shared_ptr<DP>;
  using Parameters = typename PM::Parameters;
  using Matrix = typename PM::Matrix;
  using TransformationParameters = typename PM::TransformationParameters;
  using ICP = typename PM::ICP;
  using ICPSequence = typename PM::ICPSequence;
  using TransformationPtr = std::shared_ptr<typename PM::Transformation>;
  using DataPointsFilters = typename PM::DataPointsFilters;
  using timer = PointMatcherSupport::timer;
};

#define IMPORT_TYPES(TYPE)                                                          \
  using NNS = typename Types<TYPE>::NNS;                                            \
  using SearchType = typename Types<TYPE>::SearchType;                              \
  using PM = typename Types<TYPE>::PM;                                              \
  using DP = typename Types<TYPE>::DP;                                              \
  using DPPtr = typename Types<TYPE>::DPPtr;                                        \
  using Parameters = typename Types<TYPE>::Parameters;                              \
  using Matrix = typename Types<TYPE>::Matrix;                                      \
  using TransformationParameters = typename Types<TYPE>::TransformationParameters;  \
  using ICP = typename Types<TYPE>::ICP;                                            \
  using ICPSequence = typename Types<TYPE>::ICPSequence;                            \
  using TransformationPtr = typename Types<TYPE>::TransformationPtr;                \
  using DataPointsFilters = typename Types<TYPE>::DataPointsFilters;                \
  using timer = typename Types<TYPE>::timer;


/*
// Eigen based point cloud
typedef Vector3f point;
typedef alignedVector<point> pointCloud;
typedef Vector4f pointI;
typedef alignedVector<pointI> pointCloudIntensity;
*/

/*
// Index types
typedef Vector3i generalIndex;
typedef generalIndex voxelIndex;
typedef generalIndex blockIndex;
typedef Vector3l longIndex;
typedef longIndex globalIndex;

// Index list types
typedef std::pair<blockIndex, voxelIndex> voxelKey;
typedef alignedVector<generalIndex> indexVector;
typedef indexVector voxelIndexList;
typedef indexVector blockIndexList;
typedef alignedVector<longIndex> longIndexVector;
typedef longIndexVector globalIndexList;

// For vertex access
typedef size_t vertexIndex;
typedef alignedVector<vertexIndex> vertexIndexList;
*/

/*
// Grid <-> point conversion functions.

// Return the index using round function.
template <typename IndexType>
inline IndexType getGridIndexFromOriginPoint(const point& p, const float
grid_size_inv)
{
  return IndexType(std::round(p.x() * grid_size_inv),
                   std::round(p.y() * grid_size_inv),
                   std::round(p.z() * grid_size_inv));
}

template <typename IndexType>
inline point getCenterPointFromGridIndex(const IndexType& index, float
grid_size)
{
  return point((static_cast<float>(index.x()) + 0.5) * grid_size,
               (static_cast<float>(index.y()) + 0.5) * grid_size,
               (static_cast<float>(index.z()) + 0.5) * grid_size);
}

template <typename IndexType>
inline point getOriginPointFromGridIndex(const IndexType& index, float
grid_size)
{
  return point(static_cast<float>(index.x()) * grid_size,
               static_cast<float>(index.y()) * grid_size,
               static_cast<float>(index.z()) * grid_size);
}

// Return the global index from voxel & block index.
inline globalIndex getGlobalVoxelIndexFromBlockAndVoxelIndex(const blockIndex&
block_index,
                                                             const voxelIndex&
voxel_index,
                                                             int
voxels_per_side)
{
  return globalIndex(block_index.cast<int64_t>() * voxels_per_side +
voxel_index.cast<int64_t>());
}

// Return the block index from global index.
inline blockIndex getBlockIndexFromGlobalVoxelIndex(const globalIndex&
global_voxel_index,
                                                    float voxels_per_side_inv)
{
  return blockIndex(std::floor(static_cast<float>(global_voxel_index.x()) *
voxels_per_side_inv),
                    std::floor(static_cast<float>(global_voxel_index.y()) *
voxels_per_side_inv),
                    std::floor(static_cast<float>(global_voxel_index.z()) *
voxels_per_side_inv));
}

inline voxelIndex getLocalFromGlobalVoxelIndex(
    const globalIndex& global_voxel_index, const int voxels_per_side) {
  // add a big number to the index to make it positive
  constexpr int offset = 1 << (8 * sizeof(int) - 1);

  if ((voxels_per_side & (voxels_per_side -1)))
    std::cerr << "Failure -> voxels_per_side is not power of 2";

  return voxelIndex((global_voxel_index.x() + offset) & (voxels_per_side - 1),
                    (global_voxel_index.y() + offset) & (voxels_per_side - 1),
                    (global_voxel_index.z() + offset) & (voxels_per_side - 1));
}

inline void getBlockAndVoxelIndexFromGlobalVoxelIndex(const globalIndex&
global_voxel_index, const int voxels_per_side,
                                                      blockIndex* block_index,
voxelIndex* voxel_index)
{
  if ((block_index == nullptr) || (voxel_index == nullptr))
    std::cerr << "Failure out of block or voxel nullptr";

  const float voxels_per_side_inv = 1.0 / voxels_per_side;
  *block_index = getBlockIndexFromGlobalVoxelIndex(global_voxel_index,
voxels_per_side_inv);
  *voxel_index = getLocalFromGlobalVoxelIndex(global_voxel_index,
voxels_per_side);
}


// Math functions.
template <typename T>
inline int signFunction(T x) { return (x == 0) ? 0 : x < 0 ? -1 : 1; }

// Return a homogeneous coordinates
inline Vector4f getHomogeneousPoint(const Vector3f& point)
{
  Vector4f homogeneous_point;
  homogeneous_point << point, 1;
  return homogeneous_point;
}

inline Vector3f getPointFromHomogeneous(const Vector4f& homogeneous_point)
{
  Vector3f point = homogeneous_point.head<3>();
  return point;
}
*/

#endif // BASIC_H
