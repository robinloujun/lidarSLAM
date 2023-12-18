#ifndef LIDARFACTOR_H
#define LIDARFACTOR_H

#include <iostream>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Dense>
#include <assert.h>
#include <cmath>
#include "math_utils.h"

struct LidarEdgeFactor
{
public:
  LidarEdgeFactor(Eigen::Vector3d current_pt_, Eigen::Vector3d last_pt_a_,
                  Eigen::Vector3d last_pt_b_, double weight_)
    : current_pt(current_pt_), last_pt_a(last_pt_a_), last_pt_b(last_pt_b_), weight(weight_)
  {}

  template <typename T>
  bool operator()(const T *q, const T *t, T *residual) const
  {
    Eigen::Matrix<T, 3, 1> cp(T(current_pt.x()), T(current_pt.y()), T(current_pt.z()));
    Eigen::Matrix<T, 3, 1> lpa(T(last_pt_a.x()), T(last_pt_a.y()), T(last_pt_a.z()));
    Eigen::Matrix<T, 3, 1> lpb(T(last_pt_b.x()), T(last_pt_b.y()), T(last_pt_b.z()));

    Eigen::Quaternion<T> quaternion(q[0], q[1], q[2], q[3]);
//    Eigen::Quaternion<T> quaternion(q[1], q[2], q[3], q[0]);
    quaternion = Eigen::Quaternion<T>::Identity().slerp(T(weight), quaternion);
    Eigen::Matrix<T, 3, 1> transition{T(weight)*t[0], T(weight)*t[1], T(weight)*t[2]};

    Eigen::Matrix<T, 3, 1> lp;
    lp = quaternion * cp + transition;

    Eigen::Matrix<T, 3, 1> nu = (lp - lpa).cross(lp - lpb);
    Eigen::Matrix<T, 3, 1> de = lpa - lpb;

//    residual[0] = nu.x() / de.norm();
//    residual[1] = nu.y() / de.norm();
//    residual[2] = nu.z() / de.norm();

    residual[0] = nu.norm() / de.norm();

    return true;
  }

  static ceres::CostFunction *Create(const Eigen::Vector3d current_pt_, const Eigen::Vector3d last_pt_a_,
                                     const Eigen::Vector3d last_pt_b_, const double weight_)
  {
    return (new ceres::AutoDiffCostFunction<LidarEdgeFactor, 1, 4, 3>(
              new LidarEdgeFactor(current_pt_, last_pt_a_, last_pt_b_, weight_)));
  }

private:
  Eigen::Vector3d current_pt, last_pt_a, last_pt_b;
  double weight;
};

struct LidarPlaneFactor
{
public:
  LidarPlaneFactor(Eigen::Vector3d current_pt_, Eigen::Vector3d last_pt_a_,
                   Eigen::Vector3d last_pt_b_, Eigen::Vector3d last_pt_c_, double weight_)
    : current_pt(current_pt_), last_pt_a(last_pt_a_), last_pt_b(last_pt_b_),
      last_pt_c(last_pt_c_), weight(weight_)
  {
    abc_norm = (last_pt_a - last_pt_b).cross(last_pt_a - last_pt_c);
    abc_norm.normalize();
  }

  template <typename T>
  bool operator()(const T *q, const T *t, T *residual) const
  {
    Eigen::Matrix<T, 3, 1> cp(T(current_pt.x()), T(current_pt.y()), T(current_pt.z()));
    Eigen::Matrix<T, 3, 1> lpa(T(last_pt_a.x()), T(last_pt_a.y()), T(last_pt_a.z()));
    Eigen::Matrix<T, 3, 1> abc(T(abc_norm.x()), T(abc_norm.y()), T(abc_norm.z()));

    Eigen::Quaternion<T> quaternion(q[0], q[1], q[2], q[3]);
    quaternion = Eigen::Quaternion<T>::Identity().slerp(T(weight), quaternion);
    Eigen::Matrix<T, 3, 1> transition{T(weight)*t[0], T(weight)*t[1], T(weight)*t[2]};

    Eigen::Matrix<T, 3, 1> lp;
    lp = quaternion * cp + transition;

    residual[0] = (lp - lpa).dot(abc);

    return true;
  }

  static ceres::CostFunction *Create(const Eigen::Vector3d current_pt_, const Eigen::Vector3d last_pt_a_,
                                     const Eigen::Vector3d last_pt_b_, const Eigen::Vector3d last_pt_c_, const double weight_)
  {
    return (new ceres::AutoDiffCostFunction<LidarPlaneFactor, 1, 4, 3>(
              new LidarPlaneFactor(current_pt_, last_pt_a_, last_pt_b_, last_pt_c_, weight_)));
  }

private:
  Eigen::Vector3d current_pt, last_pt_a, last_pt_b, last_pt_c;
  Eigen::Vector3d abc_norm;
  double weight;
};

struct LidarPlaneNormFactor
{
public:
  LidarPlaneNormFactor(Eigen::Vector3d current_pt_, Eigen::Vector3d abc_norm_, double abc_norm_inverse_)
    : current_pt(current_pt_), abc_norm(abc_norm_), abc_norm_inverse(abc_norm_inverse_)
  {}

  template <typename T>
  bool operator()(const T *q, const T *t, T *residual) const
  {
    Eigen::Matrix<T, 3, 1> lp(T(current_pt.x()), T(current_pt.y()), T(current_pt.z()));
    Eigen::Matrix<T, 3, 1> plane_norm(T(abc_norm.x()), T(abc_norm.y()), T(abc_norm.z()));

    Eigen::Quaternion<T> quaternion(q[0], q[1], q[2], q[3]);
//    Eigen::Quaternion<T> quaternion(q[1], q[2], q[3], q[0]);
    Eigen::Matrix<T, 3, 1> transition{t[0], t[1], t[2]};

    Eigen::Matrix<T, 3, 1> wp;
    wp = quaternion * lp + transition;

    residual[0] = plane_norm.dot(wp) + T(abc_norm_inverse);

    return true;
  }

  static ceres::CostFunction *Create(const Eigen::Vector3d current_pt_, const Eigen::Vector3d abc_norm_, const double abc_norm_inverse_)
  {
    return (new ceres::AutoDiffCostFunction<LidarPlaneNormFactor, 1, 4, 3>(
              new LidarPlaneNormFactor(current_pt_, abc_norm_, abc_norm_inverse_)));
  }

private:
  Eigen::Vector3d current_pt, abc_norm;
  double abc_norm_inverse;
};

class LidarEdgeFactorAnalytic : public ceres::SizedCostFunction<1, 4, 3>
{
public:
  LidarEdgeFactorAnalytic(Eigen::Vector3d current_pt_, Eigen::Vector3d last_pt_a_,
                          Eigen::Vector3d last_pt_b_, double weight_)
    : current_pt(current_pt_), last_pt_a(last_pt_a_), last_pt_b(last_pt_b_), weight(weight_)
  {}

  virtual ~LidarEdgeFactorAnalytic() {}

  virtual bool Evaluate(double const *const *parameters, double *residual, double **jacobians) const
  {
    Eigen::Quaterniond quaternion{parameters[0][0], parameters[0][1], parameters[0][2], parameters[0][3]};
    quaternion = Eigen::Quaterniond::Identity().slerp(weight, quaternion);
    Eigen::Vector3d transition{weight*parameters[1][0], weight*parameters[1][1], weight*parameters[1][2]};

    Eigen::Vector3d lp;
    lp = quaternion * current_pt + transition;

    Eigen::Vector3d nu = (lp - last_pt_a).cross(lp - last_pt_b);
    Eigen::Vector3d de = last_pt_a - last_pt_b;

    residual[0] = nu.norm() / de.norm();

    if (jacobians)
    {
      Eigen::Matrix<double, 1, 3> partialDerivative = nu.transpose() * (hat(last_pt_b) - hat(last_pt_a)) / nu.norm() / de.norm();
      Eigen::Matrix3d rotationVectorJacobian = -quaternion.toRotationMatrix() * hat(current_pt) * Jright(quaternion);

      if (jacobians[0])
      {
        Eigen::Map<Eigen::Matrix<double, 1, 4>> jacobian_matrix(jacobians[0]);
        jacobian_matrix.setZero();
        jacobian_matrix.block<1, 3>(0, 1) = partialDerivative * rotationVectorJacobian;
      }

      if (jacobians[1])
      {
        Eigen::Map<Eigen::Matrix<double, 1, 3>> jacobian_matrix(jacobians[1]);
        jacobian_matrix.setZero();
        jacobian_matrix = partialDerivative;
      }
    }

    return true;
  }

private:
  Eigen::Vector3d current_pt, last_pt_a, last_pt_b;
  double weight;
};

class LidarPlaneFactorAnalytic : public ceres::SizedCostFunction<1, 4, 3>
{
public:
  LidarPlaneFactorAnalytic(Eigen::Vector3d current_pt_, Eigen::Vector3d last_pt_a_,
                           Eigen::Vector3d last_pt_b_, Eigen::Vector3d last_pt_c_, double weight_)
    : current_pt(current_pt_), last_pt_a(last_pt_a_), last_pt_b(last_pt_b_),
      last_pt_c(last_pt_c_), weight(weight_)
  {}

  virtual ~LidarPlaneFactorAnalytic() {}

  virtual bool Evaluate(double const *const *parameters, double *residual, double **jacobians
                        ) const
  {
    Eigen::Quaterniond quaternion{parameters[0][0], parameters[0][1], parameters[0][2], parameters[0][3]};
    quaternion = Eigen::Quaterniond::Identity().slerp(weight, quaternion);
    Eigen::Vector3d transition{weight*parameters[1][0], weight*parameters[1][1], weight*parameters[1][2]};

    Eigen::Vector3d lp;
    lp = quaternion * current_pt + transition;

    Eigen::Vector3d de = (last_pt_a - last_pt_b).cross(last_pt_a - last_pt_c);
    double nu = (lp - last_pt_a).dot(de);

    residual[0] = nu / de.norm();

    if (jacobians)
    {
      Eigen::Matrix<double, 1, 3> partialDerivative = de.normalized().transpose();
      Eigen::Matrix3d rotationVectorJacobian = -quaternion.toRotationMatrix() * hat(current_pt) * Jright(quaternion);

      if (jacobians[0])
      {
        Eigen::Map<Eigen::Matrix<double, 1, 4>> jacobian_matrix(jacobians[0]);
        jacobian_matrix.setZero();
        jacobian_matrix.block<1, 3>(0, 1) = partialDerivative * rotationVectorJacobian;
      }

      if (jacobians[1])
      {
        Eigen::Map<Eigen::Matrix<double, 1, 3>> jacobian_matrix(jacobians[1]);
        jacobian_matrix.setZero();
        jacobian_matrix = partialDerivative;
      }
    }

    return true;
  }

private:
  Eigen::Vector3d current_pt, last_pt_a, last_pt_b, last_pt_c;
  double weight;
};

struct LidarPlaneNormFactorAnalytical : public ceres::SizedCostFunction<1, 4, 3>
{
public:
  LidarPlaneNormFactorAnalytical(Eigen::Vector3d current_pt_, Eigen::Vector3d abc_norm_, double abc_norm_inverse_, Eigen::Vector3d abc_center_)
    : current_pt(current_pt_), abc_norm(abc_norm_), abc_norm_inverse(abc_norm_inverse_), abc_center(abc_center_)
  {}

  virtual ~LidarPlaneNormFactorAnalytical() {}

  virtual bool Evaluate(double const *const *parameters, double *residual, double **jacobians
                        ) const
  {
    Eigen::Quaterniond quaternion{parameters[0][0], parameters[0][1], parameters[0][2], parameters[0][3]};
    Eigen::Vector3d transition{parameters[1][0], parameters[1][1], parameters[1][2]};

    Eigen::Vector3d wp;
    wp = quaternion * current_pt + transition;

    residual[0] = abc_norm.dot(wp) + abc_norm_inverse;

    if (jacobians)
    {
      Eigen::Matrix<double, 1, 3> partialDerivative = abc_norm_inverse*abc_norm.transpose();
//      Eigen::Matrix<double, 1, 3> partialDerivative = Eigen::Matrix<double, 1, 3>::Identity();
      Eigen::Matrix3d rotationVectorJacobian = -quaternion.toRotationMatrix() * hat(current_pt) * Jright(quaternion);

      if (jacobians[0])
      {
        Eigen::Map<Eigen::Matrix<double, 1, 4>> jacobian_matrix(jacobians[0]);
        jacobian_matrix.setZero();
        jacobian_matrix.block<1, 3>(0, 1) = partialDerivative * rotationVectorJacobian;
      }

      if (jacobians[1])
      {
        Eigen::Map<Eigen::Matrix<double, 1, 3>> jacobian_matrix(jacobians[1]);
        jacobian_matrix.setZero();
        jacobian_matrix = partialDerivative;
      }
    }

    return true;
  }

private:
  Eigen::Vector3d current_pt, abc_norm, abc_center;
  double abc_norm_inverse;
};

#endif // LIDARFACTOR_H
