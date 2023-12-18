#ifndef APPROX_HASH_ARRAY_H
#define APPROX_HASH_ARRAY_H

#include <atomic>
#include <vector>

#include <lidar_directional_slam/utils/basic.h>

/** \brief
 * This class allocate a fixed size array and index it wisth a hash
 * that is masked so that only its first N bits are non zero.
 * Reference: voxblox
 */
namespace mapping_utils
{
template <size_t unmasked_bits, typename storedElement, typename indexType,
          typename indexTypeHasher>
class approxHashArray
{
public:
//  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  storedElement& get(const size_t& hash)
  {
    return pseudo_map_[hash & bit_mask_];
  }

  storedElement& get(const indexType& index, size_t* hash)
  {
    if (hash == nullptr)
      std::cerr << "Failure out of hash nullptr in approxHashArray";
    *hash = hasher_(index);
    return get(*hash);
  }

  storedElement& get(const indexType& index)
  {
    size_t hash = hasher_(index);
    return get(hash);
  }

private:
  static constexpr size_t pseudo_map_size_ = (1 << unmasked_bits);
  static constexpr size_t bit_mask_ = (1 << unmasked_bits) - 1;

  std::array<storedElement, pseudo_map_size_> pseudo_map_;
  indexTypeHasher hasher_;
};
} // namespace mapping_utils

#endif // APPROX_HASH_ARRAY_H
