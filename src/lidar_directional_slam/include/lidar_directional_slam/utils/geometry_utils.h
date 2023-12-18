#pragma once

#ifndef GEOMETRY_UTILS_H
#define GEOMETRY_UTILS_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tf_conversions/tf_eigen.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <gtsam/geometry/Pose3.h>

namespace geometry_utils
{

typedef Eigen::Matrix4f transformationMatrix;

template <typename T>
inline Eigen::Matrix<T, 3, 1>
getTranslation(const Eigen::Matrix<T, 4, 4> transform_matrix)
{
  Eigen::Matrix<T, 3, 1> translation;
  translation = transform_matrix.topRightCorner(3, 1);
  return translation;
}

template <typename T>
inline Eigen::Matrix<T, 3, 3>
getRotation(const Eigen::Matrix<T, 4, 4> transform_matrix)
{
  Eigen::Matrix<T, 3, 3> rotation;
  rotation = transform_matrix.topLeftCorner(3, 3);
  return rotation;
}

template <typename T>
inline geometry_msgs::Point eigenToRosPoint(const Eigen::Matrix<T, 3, 1> translation)
{
  geometry_msgs::Point msg;
  msg.x = translation(0);
  msg.y = translation(1);
  msg.z = translation(2);
  return msg;
}

template <typename T>
inline geometry_msgs::Quaternion
eigenToRosQuaternion(const Eigen::Quaternion<T> rotation)
{
  geometry_msgs::Quaternion msg;
  msg.w = rotation.w();
  msg.x = rotation.x();
  msg.y = rotation.y();
  msg.z = rotation.z();
  return msg;
}

template <typename T>
inline geometry_msgs::Quaternion
eigenToRosQuaternion(const Eigen::Matrix<T, 3, 3> rotation)
{
  Eigen::Quaternion<T> quaternion(rotation);
  geometry_msgs::Quaternion msg;
  quaternion.normalize();
  msg.w = quaternion.w();
  msg.x = quaternion.x();
  msg.y = quaternion.y();
  msg.z = quaternion.z();
  return msg;
}

template <typename T>
inline geometry_msgs::Pose
eigenToRosPose(const Eigen::Matrix<T, 4, 4> transform_matrix)
{
  geometry_msgs::Pose msg;
  msg.position = eigenToRosPoint(getTranslation(transform_matrix));
  msg.orientation = eigenToRosQuaternion(getRotation(transform_matrix));
  return msg;
}

template <typename T>
inline geometry_msgs::Vector3 eigenToRosVec(const Eigen::Matrix<T, 3, 1> translation)
{
  geometry_msgs::Vector3 msg;
  msg.x = translation(0);
  msg.y = translation(1);
  msg.z = translation(2);
  return msg;
}

template <typename T>
inline geometry_msgs::Transform
eigenToRosTransform(const Eigen::Matrix<T, 4, 4> transform_matrix)
{
  geometry_msgs::Transform msg;
  msg.translation = eigenToRosVec(getTranslation(transform_matrix));
  msg.rotation = eigenToRosQuaternion(getRotation(transform_matrix));
  return msg;
}

template <typename T>
inline geometry_msgs::TransformStamped
eigenToRosTransformStamped(const Eigen::Matrix<T, 4, 4> transform_matrix,
                      const std::string& frame_id,
                      const std::string& child_frame_id)
{
  geometry_msgs::TransformStamped msg;
  msg.transform.translation = eigenToRosVec(getTranslation(transform_matrix));
  msg.transform.rotation = eigenToRosQuaternion(getRotation(transform_matrix));
  msg.header.frame_id = frame_id;
  msg.child_frame_id = child_frame_id;
  return msg;
}

template <typename T>
inline Eigen::Matrix<T, 4, 4> eigenToEigenMatrix(const geometry_msgs::Transform& msg)
{
  Eigen::Matrix<T, 4, 4> transform_matrix = Eigen::Matrix<T, 4, 4>::Identity();
  Eigen::Quaternion<T> quaternion(msg.rotation.w, msg.rotation.x,
                                  msg.rotation.y, msg.rotation.z);
  transform_matrix.topRightCorner(3, 1) << msg.translation.x, msg.translation.y,
      msg.translation.z;
  transform_matrix.topLeftCorner(3, 3) << quaternion.toRotationMatrix();
  return transform_matrix;
}

template <typename T>
inline Eigen::Matrix<T, 4, 4> eigenToEigenMatrix(const tf::StampedTransform msg)
{
  Eigen::Matrix<T, 4, 4> transform_matrix = Eigen::Matrix<T, 4, 4>::Identity();
  Eigen::Quaternion<T> quaternion(msg.getRotation().w(), msg.getRotation().x(),
                                  msg.getRotation().y(), msg.getRotation().z());
  transform_matrix.topRightCorner(3, 1) << msg.getOrigin().x(),
      msg.getOrigin().y(), msg.getOrigin().z();
  transform_matrix.topLeftCorner(3, 3) << quaternion.toRotationMatrix();
  return transform_matrix;
}

//template <typename T>

}

#endif // GEOMETRY_UTILS_H
