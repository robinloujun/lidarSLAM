#ifndef MINDISTPROBLEM_H
#define MINDISTPROBLEM_H

#include <iostream>
#include <assert.h>
#include <cmath>

#include <Eigen/Dense>

#include <roptlib/Manifolds/SharedSpace.h>
#include <roptlib/Manifolds/Sphere/Sphere.h>
#include <roptlib/Manifolds/Sphere/SphereVector.h>
#include <roptlib/Manifolds/Sphere/SphereVariable.h>
#include <roptlib/Manifolds/Euclidean/Euclidean.h>
#include <roptlib/Manifolds/Euclidean/EucVector.h>
#include <roptlib/Manifolds/Euclidean/EucVariable.h>
#include <roptlib/Manifolds/ProductElement.h>
#include <roptlib/Manifolds/ProductManifold.h>
#include <roptlib/Problems/Problem.h>
#include <roptlib/Others/def.h>

#include "math_utils.h"

class odomOptProblem : public ROPTLIB::Problem
{
public:
  // Default constructor
  odomOptProblem(const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &corner_mat_,
                 const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &surf_mat_,
                 const int &corner_count_, const int &surf_count_);

  virtual ~odomOptProblem();

  /* Overridden pure virtual base class functions of ROPTLIB::Problem */

  // Evaluate the cost function at iterate x.
  virtual double f(ROPTLIB::Variable *x) const;

  // Compute the Riemanian gradient of the cost function at iterate x.
  //  virtual void RieGrad(ROPTLIB::Variable *x, ROPTLIB::Vector *gf) const;

  // Compute the action of the Riemanian Hessian of the cost function at iterate
  // virtual void RieHessianEta(Variable *x, Vector *etax, Vector *xix) const;

  // Compute the Euclidean gradient of f
  virtual void EucGrad(ROPTLIB::Variable *x, ROPTLIB::Vector *gf) const;

  // Compute the action of the Euclidean Hessian,
  // virtual void EucHessianEta(Variable* x, Vector* etax, Vector* exix) const;

  int corner_count, surf_count;
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> corner_mat, surf_mat;

}; // class odomOptProblem

class mapOptProblem : public ROPTLIB::Problem
{
public:
  // Default constructor
  mapOptProblem(const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &corner_mat_,
                const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &surf_mat_,
                const int &corner_count_, const int &surf_count_);

  virtual ~mapOptProblem();

  /* Overridden pure virtual base class functions of ROPTLIB::Problem */

  // Evaluate the cost function at iterate x.
  virtual double f(ROPTLIB::Variable *x) const;

  // Compute the Riemanian gradient of the cost function at iterate x.
  //  virtual void RieGrad(ROPTLIB::Variable *x, ROPTLIB::Vector *gf) const;

  // Compute the action of the Riemanian Hessian of the cost function at iterate
  // virtual void RieHessianEta(Variable *x, Vector *etax, Vector *xix) const;

  // Compute the Euclidean gradient of f
  virtual void EucGrad(ROPTLIB::Variable *x, ROPTLIB::Vector *gf) const;

  // Compute the action of the Euclidean Hessian,
  // virtual void EucHessianEta(Variable* x, Vector* etax, Vector* exix) const;

  int corner_count, surf_count;
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> corner_mat, surf_mat;

}; // class mapOptProblem

#endif // MINDISTPROBLEM_H
