#pragma once

#ifndef INTEGRATOR_UTILS_H
#define INTEGRATOR_UTILS_H

#include <algorithm>
#include <atomic>
#include <vector>
#include <string>

#include <lidar_directional_slam/utils/basic.h>
#include <lidar_directional_slam/utils/voxel_block_hash.h>

namespace mapping_utils {

class integratorIndex
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit integratorIndex(size_t number_of_points);

  // returns true if index is valid, otherwise false
  bool getNextIndex(size_t* idx);

  void reset();

private:
  size_t getMixedIndex(size_t base_idx);

  std::atomic<size_t> atomic_idx_;
  const size_t number_of_points_;
  const size_t number_of_groups_;

  /// 1024 bins
  static constexpr size_t num_bits = 10;
  static constexpr size_t step_size_ = 1 << num_bits; // 1024
  static constexpr size_t bit_mask_ = step_size_ - 1; // 1023

  static const std::array<size_t, step_size_> offset_lookup_;
};

integratorIndex::integratorIndex(size_t number_of_points)
    : atomic_idx_(0),
      number_of_points_(number_of_points),
      number_of_groups_(number_of_points / step_size_) {}

bool integratorIndex::getNextIndex(size_t *idx)
{
  if (idx == nullptr)
    std::cerr << "Failure out of nullptr";

  size_t sequential_idx = atomic_idx_.fetch_add(1);

  if (sequential_idx >= number_of_points_)
  {
    return false;
  }
  else
  {
    *idx = getMixedIndex(sequential_idx);
    return true;
  }
}

void integratorIndex::reset() { atomic_idx_.store(0); }

size_t integratorIndex::getMixedIndex(size_t base_idx)
{
  if (number_of_groups_ * step_size_ <= base_idx)
  {
    return base_idx;
  }

  const size_t group_num = base_idx % number_of_groups_;
  const size_t position_in_group = base_idx / number_of_groups_;

  return group_num * step_size_ + position_in_group;
}
} // namespace mapping_utils

#endif // INTEGRATOR_UTILS_H
