#pragma once

#ifndef VOXEL_H
#define VOXEL_H

#include <lidar_directional_slam/utils/basic.h>

namespace mapping_utils
{
/** \brief
 * Voxel: a container to store the point cloud and the weight
 * later to store the SDF or color/semantic information
 */
struct voxel
{
  // position of the points in the voxel (12 bytes)
  point point_position;
  // weights of points in voxel (1 byte)
  //  float weight;
  // Threshold for maximum number of points in a voxel (4 bytes)
  //  float threshold_max_num_points; // Set one point in one voxel at the
  //  moment
  // value of the truncated sighned distance transformation (4 bytes) (for later
  // use)
  //  float signedDistanceFunction = 0.0f;

  voxel()
  {
    point_position = Vector3f::Zero();
    //    weight = 0;
    //    threshold_max_num_points = 50.0f;
    //    signedDistanceFunction = 0.0f;
  }

  static point runningAverage(const point& first_point_position,
                              const point& second_point_position)
  {
    point averaged_point_position;
    averaged_point_position.x() = (first_point_position.x() + second_point_position.x()) / 2;
    averaged_point_position.y() = (first_point_position.y() + second_point_position.y()) / 2;
    averaged_point_position.z() = (first_point_position.z() + second_point_position.z()) / 2;

    return averaged_point_position;
  }
}; // struct voxel

} // namespace mapping_utils

#endif // VOXEL_H
