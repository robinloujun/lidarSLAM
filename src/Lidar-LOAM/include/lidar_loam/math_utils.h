#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#include <cmath>
#include <Eigen/Dense>

// Sign function
template <typename T>
T sgnFunc(T val)
{
  return (T(0) < val) - (val < T(0));
}

// Hat (skew) operator
template <typename T>
inline Eigen::Matrix<T, 3, 3> hat(const Eigen::Matrix<T, 3, 1> &vec)
{
  Eigen::Matrix<T, 3, 3> mat;
  mat << 0, -vec(2), vec(1),
         vec(2), 0, -vec(0),
         -vec(1), vec(0), 0;
  return mat;
}

// Left-quaternion-product matrix
//  mat << qua.w(), -qua.x(), -qua.y(), -qua.z(),
//         qua.x(),  qua.w(), -qua.z(),  qua.y(),
//         qua.y(),  qua.z(),  qua.w(), -qua.x(),
//         qua.z(), -qua.y(),  qua.x(),  qua.w();
template <typename T>
inline Eigen::Matrix4d Qleft(const Eigen::Quaternion<T> &qua)
{
  Eigen::Matrix<T, 4, 4> mat;
  mat(0, 0) = qua.w();
  mat.block<1, 3>(0, 1) = -qua.vec().transpose();
  mat.block<3, 1>(1, 0) = qua.vec();
  mat.block<3, 3>(1, 1) = qua.w() * Eigen::Matrix<T, 3, 3>::Identity() + hat(qua.vec());

  return mat;
}

// Right-quaternion-product matrix
//  mat << qua.w(), -qua.x(), -qua.y(), -qua.z(),
//         qua.x(),  qua.w(),  qua.z(), -qua.y(),
//         qua.y(), -qua.z(),  qua.w(), -qua.x(),
//         qua.z(),  qua.y(), -qua.x(),  qua.w();
template <typename T>
inline Eigen::Matrix<T, 4, 4> Qright(const Eigen::Quaternion<T> &qua)
{
  Eigen::Matrix<T, 4, 4> mat;
  mat(0, 0) = qua.w();
  mat.block<1, 3>(0, 1) = -qua.vec().transpose();
  mat.block<3, 1>(1, 0) = qua.vec();
  mat.block<3, 3>(1, 1) = qua.w() * Eigen::Matrix<T, 3, 3>::Identity() - hat(qua.vec());
  return mat;
}

// Convert from quaternion to rotation vector
template <typename T>
inline Eigen::Matrix<T, 3, 1> quaternionToRotationVector(const Eigen::Quaternion<T> &qua)
{
  Eigen::Matrix<T, 3, 3> mat = qua.toRotationMatrix();
  Eigen::Matrix<T, 3, 1> rotation_vec;
  Eigen::AngleAxis<T> angle_axis;
  angle_axis.fromRotationMatrix(mat);
  rotation_vec = angle_axis.angle() * angle_axis.axis();
  return rotation_vec;
}

// Right Jacobian matrix
template <typename T>
inline Eigen::Matrix3d Jright(const Eigen::Quaternion<T> &qua)
{
   Eigen::Matrix<T, 3, 3> mat;
   Eigen::Matrix<T, 3, 1> rotation_vec = quaternionToRotationVector(qua);
   double theta_norm = rotation_vec.norm();
   mat = Eigen::Matrix<T, 3, 3>::Identity()
       - (1 - cos(theta_norm)) / (theta_norm * theta_norm + 1e-10) * hat(rotation_vec)
       + (theta_norm - sin(theta_norm)) / (theta_norm * theta_norm * theta_norm + 1e-10) * hat(rotation_vec) * hat(rotation_vec);
   return mat;
}

// Calculate the Jacobian with respect to the quaternion
template <typename T>
inline Eigen::Matrix<T, 3, 4> quaternionJacobian(const Eigen::Quaternion<T> &qua, const Eigen::Matrix<T, 3, 1> &vec)
{
  Eigen::Matrix<T, 3, 4> mat;
  Eigen::Matrix<T, 3, 1> quaternion_imaginary(qua.x(), qua.y(), qua.z());

  mat.template block<3, 1>(0, 0) = qua.w() * vec + quaternion_imaginary.cross(vec);
  mat.template block<3, 3>(0, 1) = quaternion_imaginary.dot(vec) * Eigen::Matrix<T, 3, 3>::Identity()
                        + quaternion_imaginary * vec.transpose()
                        - vec * quaternion_imaginary.transpose()
                        - qua.w() * hat(vec);
  return T(2) * mat;
}

#endif // MATH_UTILS_H
