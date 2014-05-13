/*
 * contactForceSolver.h
 *
 *  Created on: Sep 12, 2013
 *      Author: cheonhyeonpark
 */

#ifndef CONTACTFORCESOLVER_H_
#define CONTACTFORCESOLVER_H_

#include <kdl/frames.hpp>

namespace itomp_cio_planner
{
class ContactForceSolver
{
public:
  ContactForceSolver();
  virtual ~ContactForceSolver();

  void operator()(double friction_coeff, std::vector<KDL::Vector>& contact_forces,
      std::vector<KDL::Vector>& contact_positions, const KDL::Wrench& wrench, const std::vector<double>& contact_values,
      const std::vector<KDL::Frame> contact_parent_frames);

  void torqueEval(double *p, double *x, int m, int n);
  void jacTorqueEval(double *p, double *jac, int m, int n);

  static void torqueEvalWrapper(double *p, double *x, int m, int n, void *adata);
  static void jacTorqueEvalWrapper(double *p, double *jac, int m, int n, void *adata);

protected:
  std::vector<KDL::Vector> diffs_;
  std::vector<KDL::Rotation> rotations_;
  std::vector<KDL::Vector> forces_;
};
}

#endif /* CONTACTFORCESOLVER_H_ */
