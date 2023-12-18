#pragma once

#ifndef VOXEL_BLOCK_HASH_H
#define VOXEL_BLOCK_HASH_H

#include <lidar_directional_slam/utils/basic.h>

namespace mapping_utils
{
struct hashIndex // hash value evaluated with hash function
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Use an arbitrary prime number
  static constexpr size_t sl = 17191;
  static constexpr size_t sl2 = sl * sl;

  std::size_t operator()(const generalIndex& index) const
  {
    return static_cast<uint>(index.x() + index.y() * sl + index.z() * sl2);
  }

  hashIndex() {}
};

/*
// reference: [Teschner2003], used by KinectFusion and InfiniTAM
struct hashIndex
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  std::size_t operator()(const generalIndex& index) const
  {
    return static_cast<uint>((index.x() * 73856093u) ^ (index.y() * 19349669u) ^ (index.z() * 83492791u) & 0xfffff);
  }
  hashIndex() {}
};
*/

template <typename value_type>
struct hashMapType
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef std::unordered_map<
      generalIndex, value_type, hashIndex, std::equal_to<generalIndex>,
      Eigen::aligned_allocator<std::pair<const generalIndex, value_type> > > type;
};

typedef std::unordered_set<generalIndex, hashIndex, std::equal_to<generalIndex>,
                           Eigen::aligned_allocator<generalIndex> > indexSet;

typedef typename hashMapType<indexVector>::type hierarchicalIndexMap;
typedef typename hashMapType<indexSet>::type hierarchicalIndexSet;
typedef typename hierarchicalIndexMap::value_type hierarchicalIndex;

struct LongIndexHash
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static constexpr size_t sl = 17191;
  static constexpr size_t sl2 = sl * sl;

  std::size_t operator()(const longIndex& index) const
  {
    return static_cast<uint>(index.x() + index.y() * sl + index.z() * sl2);
  }
};

template <typename value_type>
struct longIndexHashMapType
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef std::unordered_map<
    longIndex, value_type, LongIndexHash, std::equal_to<longIndex>,
    Eigen::aligned_allocator<std::pair<const longIndex, value_type> > > type;
};

}  // namespace mapping_utils

#endif // VOXEL_BLOCK_HASH_H
