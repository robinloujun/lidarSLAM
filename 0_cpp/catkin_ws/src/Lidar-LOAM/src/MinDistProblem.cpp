#include "lidar_loam/MinDistProblem.h"

odomOptProblem::odomOptProblem(const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &corner_mat_,
               const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &surf_mat_,
               const int &corner_count_, const int &surf_count_)
  : corner_count(corner_count_), surf_count(surf_count_), corner_mat(corner_mat_), surf_mat(surf_mat_)
  {}

odomOptProblem::~odomOptProblem() {}

double odomOptProblem::f(ROPTLIB::Variable *x) const
{
  // The sum of weighted point-to-line/plane distances
  double result = 0;

  ROPTLIB::ProductElement *prodX = dynamic_cast<ROPTLIB::ProductElement *> (x);

  const double *xQ = prodX->GetElement(0)->ObtainReadData();
  const double *xT = prodX->GetElement(1)->ObtainReadData();

  Eigen::Quaterniond quaternion{xQ[0], xQ[1], xQ[2], xQ[3]};
  Eigen::Vector3d transition{xT[0], xT[1], xT[2]};

  // Add residuals of edge feature points
  for (int i = 0; i < corner_count; i++)
  {
    Eigen::Vector3d current_pt = corner_mat.block<3, 1>(0, i);
    Eigen::Vector3d last_pt_j = corner_mat.block<3, 1>(3, i);
    Eigen::Vector3d last_pt_l = corner_mat.block<3, 1>(6, i);
    double weight = corner_mat(9, i);

    quaternion = Eigen::Quaterniond::Identity().slerp(weight, quaternion);
    transition = weight * transition;

    Eigen::Vector3d last_pt;
    last_pt = quaternion * current_pt + transition;

    Eigen::Vector3d nu = (last_pt - last_pt_j).cross(last_pt - last_pt_l);
    Eigen::Vector3d de = last_pt_j - last_pt_l;

    double residual = nu.norm() / de.norm();
    result += residual;
  }

  // Add residuals of planar feature points
  for (int i = 0; i < surf_count; i++)
  {
    Eigen::Vector3d current_pt(surf_mat.block<3, 1>(0, i));
    Eigen::Vector3d last_pt_j(surf_mat.block<3, 1>(3, i));
    Eigen::Vector3d last_pt_l(surf_mat.block<3, 1>(6, i));
    Eigen::Vector3d last_pt_m(surf_mat.block<3, 1>(9, i));
    double weight = surf_mat(12, i);

    quaternion = Eigen::Quaterniond::Identity().slerp(weight, quaternion);
    transition = weight * transition;

    Eigen::Vector3d last_pt;
    last_pt = quaternion * current_pt + transition;

    Eigen::Vector3d de = (last_pt_j - last_pt_l).cross(last_pt_j - last_pt_m);
    double nu = (last_pt - last_pt_j).dot(de);

    double residual = nu / de.norm();
    result += residual;
  }

  return result;
}

void odomOptProblem::EucGrad(ROPTLIB::Variable *x, ROPTLIB::Vector *gf) const
{
  ROPTLIB::ProductElement *prodX = dynamic_cast<ROPTLIB::ProductElement *> (x);

  const double *xQ = prodX->GetElement(0)->ObtainReadData();
  const double *xT = prodX->GetElement(1)->ObtainReadData();

  Eigen::Quaterniond quaternion{xQ[0], xQ[1], xQ[2], xQ[3]};
  Eigen::Vector3d transition{xT[0], xT[1], xT[2]};

  gf->NewMemoryOnWrite();
  // Set the pointer to gradient function
  double *egfPtr = gf->ObtainWriteEntireData();
  Eigen::Map<Eigen::Matrix<double, 1, 4> > jacobian_matrix_quaternion(egfPtr);
  Eigen::Map<Eigen::Matrix<double, 1, 3> > jacobian_matrix_transition(egfPtr + 4);

//  ROPTLIB::ProductElement *prodgf = dynamic_cast<ROPTLIB::ProductElement *> (gf);
//  prodgf->NewMemoryOnWrite();

  // Set the pointer to gradient function
//  double *gfPtr_quaternion = prodgf->GetElement(0)->ObtainWriteEntireData();
//  double *gfPtr_transition = prodgf->GetElement(1)->ObtainWriteEntireData();

//  Eigen::Map<Eigen::Matrix<double, 1, 4>> jacobian_matrix_quaternion(gfPtr_quaternion);
//  Eigen::Map<Eigen::Matrix<double, 1, 3>> jacobian_matrix_transition(gfPtr_transition);
  jacobian_matrix_quaternion.setZero();
  jacobian_matrix_transition.setZero();

  // Add jacobian elements of edge feature points
  for (int i = 0; i < corner_count; i++)
  {
    Eigen::Vector3d current_pt(corner_mat.block<3, 1>(0, i));
    Eigen::Vector3d last_pt_j(corner_mat.block<3, 1>(3, i));
    Eigen::Vector3d last_pt_l(corner_mat.block<3, 1>(6, i));
    double weight = corner_mat(9, i);

    quaternion = Eigen::Quaterniond::Identity().slerp(weight, quaternion);
    transition = weight * transition;

    Eigen::Vector3d last_pt;
    last_pt = quaternion * current_pt + transition;

    Eigen::Vector3d nu = (last_pt - last_pt_j).cross(last_pt - last_pt_l);
    Eigen::Vector3d de = last_pt_j - last_pt_l;

    Eigen::Matrix<double, 1, 3> partialDerivative = nu.transpose() * (hat(last_pt_l) - hat(last_pt_j)) / nu.norm() / de.norm();
    Eigen::Matrix<double, 3, 4> quaternionDerivative = quaternionJacobian(quaternion, current_pt);

    jacobian_matrix_quaternion += partialDerivative * quaternionDerivative;
    jacobian_matrix_transition += partialDerivative;
  }

  // Add jacobian elements of planar feature points
  for (int i = 0; i < surf_count; i++)
  {
    Eigen::Vector3d current_pt(surf_mat.block<3, 1>(0, i));
    Eigen::Vector3d last_pt_j(surf_mat.block<3, 1>(3, i));
    Eigen::Vector3d last_pt_l(surf_mat.block<3, 1>(6, i));
    Eigen::Vector3d last_pt_m(surf_mat.block<3, 1>(9, i));
    double weight = surf_mat(12, i);

    quaternion = Eigen::Quaterniond::Identity().slerp(weight, quaternion);
    transition = weight * transition;

    Eigen::Vector3d last_pt;
    last_pt = quaternion * current_pt + transition;

    Eigen::Vector3d de = (last_pt_j - last_pt_l).cross(last_pt_j - last_pt_m);
//    double nu = (last_pt - last_pt_j).dot(de);

    Eigen::Matrix<double, 1, 3> partialDerivative = de.normalized().transpose();
    Eigen::Matrix<double, 3, 4> quaternionDerivative = quaternionJacobian(quaternion, current_pt);

    jacobian_matrix_quaternion += partialDerivative * quaternionDerivative;
    jacobian_matrix_transition += partialDerivative;
  }

//  ROPTLIB::ProductManifold *prodDomain = dynamic_cast<ROPTLIB::ProductManifold *> (Domain);

}


mapOptProblem::mapOptProblem(const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &corner_mat_,
               const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &surf_mat_,
               const int &corner_count_, const int &surf_count_)
  : corner_count(corner_count_), surf_count(surf_count_), corner_mat(corner_mat_), surf_mat(surf_mat_)
  {}

mapOptProblem::~mapOptProblem() {}

double mapOptProblem::f(ROPTLIB::Variable *x) const
{
  // The sum of weighted point-to-line/plane distances
  double result = 0;

  ROPTLIB::ProductElement *prodX = dynamic_cast<ROPTLIB::ProductElement *> (x);

  const double *xQ = prodX->GetElement(0)->ObtainReadData();
  const double *xT = prodX->GetElement(1)->ObtainReadData();

  Eigen::Quaterniond quaternion{xQ[0], xQ[1], xQ[2], xQ[3]};
  Eigen::Vector3d transition{xT[0], xT[1], xT[2]};

  // Add residuals of edge feature points
  for (int i = 0; i < corner_count; i++)
  {
    Eigen::Vector3d current_pt = corner_mat.block<3, 1>(0, i);
    Eigen::Vector3d last_pt_j = corner_mat.block<3, 1>(3, i);
    Eigen::Vector3d last_pt_l = corner_mat.block<3, 1>(6, i);

    Eigen::Vector3d last_pt;
    last_pt = quaternion * current_pt + transition;

    Eigen::Vector3d nu = (last_pt - last_pt_j).cross(last_pt - last_pt_l);
    Eigen::Vector3d de = last_pt_j - last_pt_l;

    double residual = nu.norm() / de.norm();
    result += residual;
  }

  // Add residuals of planar feature points
  for (int i = 0; i < surf_count; i++)
  {
    Eigen::Vector3d lp(surf_mat.block<3, 1>(0, i));
    Eigen::Vector3d plane_norm(surf_mat.block<3, 1>(3, i));
    double normal_inverse = surf_mat(6, i);

    Eigen::Vector3d wp;
    wp = quaternion * lp + transition;

    double residual = plane_norm.dot(wp) + normal_inverse;
//    result += residual;
  }

  return result;
}

void mapOptProblem::EucGrad(ROPTLIB::Variable *x, ROPTLIB::Vector *gf) const
{
  ROPTLIB::ProductElement *prodX = dynamic_cast<ROPTLIB::ProductElement *> (x);

  const double *xQ = prodX->GetElement(0)->ObtainReadData();
  const double *xT = prodX->GetElement(1)->ObtainReadData();

  Eigen::Quaterniond quaternion{xQ[0], xQ[1], xQ[2], xQ[3]};
  Eigen::Vector3d transition{xT[0], xT[1], xT[2]};

  gf->NewMemoryOnWrite();
  // Set the pointer to gradient function
  double *egfPtr = gf->ObtainWriteEntireData();
  Eigen::Map<Eigen::Matrix<double, 1, 4> > jacobian_matrix_quaternion(egfPtr);
  Eigen::Map<Eigen::Matrix<double, 1, 3> > jacobian_matrix_transition(egfPtr + 4);

  jacobian_matrix_quaternion.setZero();
  jacobian_matrix_transition.setZero();

  // Add jacobian elements of edge feature points
  for (int i = 0; i < corner_count; i++)
  {
    Eigen::Vector3d local_pt(corner_mat.block<3, 1>(0, i));
    Eigen::Vector3d last_pt_j(corner_mat.block<3, 1>(3, i));
    Eigen::Vector3d last_pt_l(corner_mat.block<3, 1>(6, i));

    Eigen::Vector3d world_pt;
    world_pt = quaternion * local_pt + transition;

    Eigen::Vector3d nu = (local_pt - last_pt_j).cross(local_pt - last_pt_l);
    Eigen::Vector3d de = last_pt_j - last_pt_l;

    Eigen::Matrix<double, 1, 3> partialDerivative = nu.transpose() * (hat(last_pt_l) - hat(last_pt_j)) / nu.norm() / de.norm();
    Eigen::Matrix<double, 3, 4> quaternionDerivative = quaternionJacobian(quaternion, local_pt);

    jacobian_matrix_quaternion += partialDerivative * quaternionDerivative;
    jacobian_matrix_transition += partialDerivative;
  }

  // Add jacobian elements of planar feature points
  for (int i = 0; i < surf_count; i++)
  {
    Eigen::Vector3d local_pt(surf_mat.block<3, 1>(0, i));
    Eigen::Vector3d plane_norm(surf_mat.block<3, 1>(3, i));
    double normal_inverse = surf_mat(6, i);

    Eigen::Vector3d world_pt;
    world_pt = quaternion * local_pt + transition;

    Eigen::Matrix<double, 1, 3> partialDerivative = plane_norm.transpose();
    Eigen::Matrix<double, 3, 4> quaternionDerivative = quaternionJacobian(quaternion, local_pt);

//    jacobian_matrix_quaternion += partialDerivative * quaternionDerivative;
//    jacobian_matrix_transition += partialDerivative;
  }


}
