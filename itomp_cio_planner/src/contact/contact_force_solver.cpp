/*
 * contact_force_solver.cpp
 *
 *  Created on: Sep 12, 2013
 *      Author: cheonhyeonpark
 */

#include <ros/ros.h>
#include <itomp_cio_planner/common.h>
#include <itomp_cio_planner/contact/contact_force_solver.h>
#include <itomp_cio_planner/optimization/f2c.h>
#include <itomp_cio_planner/optimization/clapack.h>

namespace itomp_cio_planner
{
////////////////////////////////////////////////////////////////////////////////
const double w = 0.00001;
const double dz = -0.1;

ContactForceSolver::ContactForceSolver()
{

}

ContactForceSolver::~ContactForceSolver()
{

}

void ContactForceSolver::operator()(double friction_coeff, std::vector<KDL::Vector>& contact_forces,
    std::vector<KDL::Vector>& contact_positions, const KDL::Wrench& wrench, const std::vector<double>& contact_values,
    const std::vector<KDL::Frame> contact_parent_frames)
{
  const double k_0 = 0.01;
  const double k_1 = 0.001;

  int num_contacts = contact_forces.size();

  // compute contact forces using linear programming
  {
    char trans = 'N';
    integer m = 6 + num_contacts * 3;
    integer n = num_contacts * 3;
    integer nrhs = 1;
    integer lda = m;
    integer ldb = m;
    integer lwork = m * n;
    integer info = 0;

    doublereal A[m * n];
    doublereal b[m];
    doublereal work[lwork];

    // set A
    memset(A, 0, sizeof(doublereal) * m * n);
    int row = 0, column = 0;
    for (int i = 0; i < num_contacts; ++i)
    {
      // lapack uses column-major matrices
      // A[r + c * lda]
      A[0 + (column + 0) * lda] = 1;
      A[1 + (column + 1) * lda] = 1;
      A[2 + (column + 2) * lda] = 1;
      column += 3;
    }
    row = 3;
    column = 0;
    for (int i = 0; i < num_contacts; ++i)
    {
      const KDL::Vector& p = contact_parent_frames[i].p;
      A[(row + 0) + (column + 1) * lda] = w * -p.z();
      A[(row + 0) + (column + 2) * lda] = w * p.y();
      A[(row + 1) + (column + 0) * lda] = w * p.z();
      A[(row + 1) + (column + 2) * lda] = w * -p.x();
      A[(row + 2) + (column + 0) * lda] = w * -p.y();
      A[(row + 2) + (column + 1) * lda] = w * p.x();

      column += 3;
    }
    row = 6;
    column = 0;
    for (int i = 0; i < num_contacts; ++i)
    {
      const doublereal contact_value = contact_values[i];
      const doublereal e = k_0 / (contact_value * contact_value + k_1);
      A[(row + 0) + (column + 0) * lda] = e;
      A[(row + 1) + (column + 1) * lda] = e;
      A[(row + 2) + (column + 2) * lda] = e;
      row += 3;
      column += 3;
    }

    // set b
    b[0] = -wrench.force.x();
    b[1] = -wrench.force.y();
    b[2] = -wrench.force.z();
    b[3] = w * -wrench.torque.x();
    b[4] = w * -wrench.torque.y();
    b[5] = w * -wrench.torque.z();
    row = 6;
    for (int i = 0; i < num_contacts; ++i)
    {
      b[row] = 0;
      b[row + 1] = 0;
      b[row + 2] = 0;
      row += 3;
    }

    // solve
    dgels_(&trans, &m, &n, &nrhs, A, &lda, b, &ldb, work, &lwork, &info);

    row = 0;
    for (int i = 0; i < num_contacts; ++i)
    {
      double x = b[row];
      double y = b[row + 1];
      double z = b[row + 2];
      contact_forces[i] = KDL::Vector(x, y, z);
      row += 3;
    }
  }

  ///////////////////////////////////////////////////////////////
  // compute center of pressure (contact positions) using linear programming
  {
    const double CONTACT_MIN_DIR = -0.1;
    const double CONTACT_MAX_DIR = 0.19;
    const double CONTACT_MIN_RIGHT = -0.1;
    const double CONTACT_MAX_RIGHT = 0.1;

    char trans = 'N';
    integer m = 3 + num_contacts * 2;
    integer n = num_contacts * 2;
    integer nrhs = 1;
    integer lda = m;
    integer ldb = m;
    integer lwork = m * n;
    integer info = 0;

    doublereal A[m * n];
    doublereal b[m];
    doublereal work[lwork];

    // set A
    memset(A, 0, sizeof(doublereal) * m * n);
    int row = 0, column = 0;
    for (int i = 0; i < num_contacts; ++i)
    {
      // lapack uses column-major matrices
      // A[r + c * lda]
      A[0 + (column + 1) * lda] = contact_forces[i].z();
      A[1 + (column + 0) * lda] = -contact_forces[i].z();
      A[2 + (column + 0) * lda] = contact_forces[i].y();
      A[2 + (column + 1) * lda] = -contact_forces[i].x();
      column += 2;
    }
    row = 3;
    column = 0;
    for (int i = 0; i < num_contacts; ++i)
    {
      A[(row + 0) + (column + 0) * lda] = w;
      A[(row + 1) + (column + 1) * lda] = w;
      row += 2;
      column += 2;
    }

    // set b
    b[0] = -wrench.torque.x();
    b[1] = -wrench.torque.y();
    b[2] = -wrench.torque.z();
    for (int i = 0; i < num_contacts; ++i)
    {
      const KDL::Vector& p = contact_parent_frames[i].p;
      const KDL::Vector& f = contact_forces[i];
      const KDL::Vector torque = p * f;
      b[0] -= torque.x();
      b[1] -= torque.y();
      b[2] -= torque.z();

      b[0] += dz * f.y();
      b[1] += dz * f.x();
    }
    row = 3;
    for (int i = 0; i < num_contacts; ++i)
    {
      b[row] = 0;
      b[row + 1] = 0;
      row += 2;
    }

    // solve
    dgels_(&trans, &m, &n, &nrhs, A, &lda, b, &ldb, work, &lwork, &info);

    row = 0;
    for (int i = 0; i < num_contacts; ++i)
    {
      const KDL::Rotation& rot = contact_parent_frames[i].M;
      KDL::Vector diff = KDL::Vector(b[row], b[row + 1], dz);
      diff.x(min(diff.x(), CONTACT_MAX_RIGHT));
      diff.x(max(diff.x(), CONTACT_MIN_RIGHT));
      diff.y(min(diff.y(), CONTACT_MAX_DIR));
      diff.y(max(diff.y(), CONTACT_MIN_DIR));
      diff = rot * diff;
      contact_positions[i] = contact_parent_frames[i].p + diff;
      row += 2;
    }
  }
}

}
;
