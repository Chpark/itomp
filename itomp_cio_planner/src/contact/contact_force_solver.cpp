/*
 * contact_force_solver.cpp
 *
 *  Created on: Sep 12, 2013
 *      Author: cheonhyeonpark
 */

#include <ros/ros.h>
#include <itomp_cio_planner/common.h>
#include <itomp_cio_planner/contact/contact_force_solver.h>
#include <itomp_cio_planner/contact/levmar.h>
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

void ContactForceSolver::torqueEvalWrapper(double *p, double *x, int m, int n, void *adata)
{
  ContactForceSolver* solver = (ContactForceSolver*) adata;
  solver->torqueEval(p, x, m, n);
}

void ContactForceSolver::jacTorqueEvalWrapper(double *p, double *jac, int m, int n, void *adata)
{
  ContactForceSolver* solver = (ContactForceSolver*) adata;
  solver->jacTorqueEval(p, jac, m, n);
}

void ContactForceSolver::torqueEval(double *p, double *x, int m, int n)
{
  int num_contacts = m / 2;

  KDL::Vector torque_sum = KDL::Vector::Zero();

  // (p-r) x f + r x f = p x f
  // R * d = (p-r)
  // Rd x f = -W.torque - r x f
  int p_pos = 0;
  for (int i = 0; i < num_contacts; ++i)
  {
    diffs_[i] = KDL::Vector(*(p + p_pos), *(p + p_pos + 1), dz);
    diffs_[i] = rotations_[i] * diffs_[i];
    p_pos += 2;
  }

  // Rd x f
  p_pos = 3;
  for (int i = 0; i < num_contacts; ++i)
  {
    KDL::Vector torque = diffs_[i] * forces_[i];
    torque_sum += torque;
    x[p_pos + 0] = w * torque.x();
    x[p_pos + 1] = w * torque.y();
    x[p_pos + 2] = w * torque.z();
    p_pos += 3;
  }

  x[0] = torque_sum.x();
  x[1] = torque_sum.y();
  x[2] = torque_sum.z();
}

void ContactForceSolver::jacTorqueEval(double *p, double *jac, int m, int n)
{
  int num_contacts = m / 2;

  memset(jac, 0, sizeof(double) * m * n);

  int row = 3;
  int col = 0;
  for (int i = 0; i < num_contacts; ++i)
  {
    const KDL::Rotation& R = rotations_[i];
    const KDL::Vector& f = forces_[i];

    jac[0 * m + (col)] = R(1, 0) * f.z() - R(2, 0) * f.y();
    jac[(0 + 1) * m + (col)] = R(2, 0) * f.x() - R(0, 0) * f.z();
    jac[(0 + 2) * m + (col)] = R(0, 0) * f.y() - R(1, 0) * f.x();

    jac[0 * m + (col + 1)] = R(1, 1) * f.z() - R(2, 1) * f.y();
    jac[(0 + 1) * m + (col + 1)] = R(2, 1) * f.x() - R(0, 1) * f.z();
    jac[(0 + 2) * m + (col)] = R(0, 1) * f.y() - R(1, 1) * f.x();

    jac[row * m + (col)] = w * R(1, 0) * f.z() - R(2, 0) * f.y();
    jac[(row + 1) * m + (col)] = w * R(2, 0) * f.x() - R(0, 0) * f.z();
    jac[(row + 2) * m + (col)] = w * R(0, 0) * f.y() - R(1, 0) * f.x();

    jac[row * m + (col + 1)] = w * R(1, 1) * f.z() - R(2, 1) * f.y();
    jac[(row + 1) * m + (col + 1)] = w * R(2, 1) * f.x() - R(0, 1) * f.z();
    jac[(row + 2) * m + (col)] = w * R(0, 1) * f.y() - R(1, 1) * f.x();

    row += 3;
    col += 2;
  }

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

    ///////////////////////

    /*
     const double CONTACT_MIN_DIR = -0.1;
     const double CONTACT_MAX_DIR = 0.19;
     const double CONTACT_MIN_RIGHT = -0.1;
     const double CONTACT_MAX_RIGHT = 0.1;

     int itMax = 1000;
     double opts[LM_OPTS_SZ], info[LM_INFO_SZ];
     opts[0] = LM_INIT_MU;
     opts[1] = opts[2] = opts[3] = opts[4] = LM_INIT_MU;

     int ret;
     const int m = 2 * num_contacts;
     const int n = 3 + 3 * num_contacts;
     const int k2 = 4 * num_contacts;
     double p[m];
     double x[n];
     double C[k2 * m];
     double d[k2];

     memset(p, 0, sizeof(double) * m);

     x[0] = -wrench.torque.x();
     x[1] = -wrench.torque.y();
     x[2] = -wrench.torque.z();
     for (int i = 0; i < num_contacts; ++i)
     {
     const KDL::Vector& p = contact_parent_frames[i].p;
     const KDL::Vector& f = contact_forces[i];
     const KDL::Vector torque = p * f;
     x[0] -= torque.x();
     x[1] -= torque.y();
     x[2] -= torque.z();
     }
     memset(x + 3, 0, sizeof(double) * (n - 3));

     memset(C, 0, sizeof(double) * k2 * m);
     int row = 0;
     int col = 0;
     for (int i = 0; i < num_contacts; ++i)
     {
     C[(row + 0) * m + (col + 0)] = 1.0;
     C[(row + 1) * m + (col + 0)] = -1.0;
     C[(row + 2) * m + (col + 1)] = 1.0;
     C[(row + 3) * m + (col + 1)] = -1.0;
     row += 4;
     col += 2;
     }
     row = 0;
     for (int i = 0; i < num_contacts; ++i)
     {
     d[row + 0] = CONTACT_MIN_RIGHT;
     d[row + 1] = -CONTACT_MAX_RIGHT;
     d[row + 2] = CONTACT_MIN_DIR;
     d[row + 3] = -CONTACT_MAX_DIR;
     row += 4;
     }

     diffs_.resize(num_contacts);
     rotations_.resize(num_contacts);
     forces_.resize(num_contacts);
     for (int i = 0; i < num_contacts; ++i)
     {
     rotations_[i] = contact_parent_frames[i].M;
     forces_[i] = contact_forces[i];
     }

     ContactForceSolver* pSolver = this;

     ret = dlevmar_lic_der(torqueEvalWrapper, jacTorqueEvalWrapper, p, x, m, n, C, d, k2, itMax, opts, info, NULL, NULL,
     pSolver);

     row = 0;
     for (int i = 0; i < num_contacts; ++i)
     {
     const KDL::Rotation& rot = contact_parent_frames[i].M;
     KDL::Vector diff = KDL::Vector(p[row], p[row + 1], dz);
     diff = rot * diff;
     contact_positions[i] = contact_parent_frames[i].p + diff;
     row += 2;
     }
     */
  }
  /*
   {

   const double CONTACT_MIN_DIR = -0.1;
   const double CONTACT_MAX_DIR = 0.19;
   const double CONTACT_MIN_RIGHT = -0.1;
   const double CONTACT_MAX_RIGHT = 0.1;

   int itMax = 1000;
   double opts[LM_OPTS_SZ], info[LM_INFO_SZ];
   opts[0] = LM_INIT_MU;
   opts[1] = opts[2] = opts[3] = opts[4] = LM_INIT_MU;

   int ret;
   const int m = 2 * num_contacts;
   const int n = 3 + 3 * num_contacts;
   const int k2 = 4 * num_contacts;
   double p[m];
   double x[n];
   double C[k2 * m];
   double d[k2];

   memset(p, 0, sizeof(double) * m);

   x[0] = -wrench.torque.x();
   x[1] = -wrench.torque.y();
   x[2] = -wrench.torque.z();
   for (int i = 0; i < num_contacts; ++i)
   {
   const KDL::Vector& p = contact_parent_frames[i].p;
   const KDL::Vector& f = contact_forces[i];
   const KDL::Vector torque = p * f;
   x[0] -= torque.x();
   x[1] -= torque.y();
   x[2] -= torque.z();
   }
   memset(x + 3, 0, sizeof(double) * (n - 3));

   memset(C, 0, sizeof(double) * k2 * m);
   int row = 0;
   int col = 0;
   for (int i = 0; i < num_contacts; ++i)
   {
   C[(row + 0) * m + (col + 0)] = 1.0;
   C[(row + 1) * m + (col + 0)] = -1.0;
   C[(row + 2) * m + (col + 1)] = 1.0;
   C[(row + 3) * m + (col + 1)] = -1.0;
   row += 4;
   col += 2;
   }
   row = 0;
   for (int i = 0; i < num_contacts; ++i)
   {
   d[row + 0] = CONTACT_MIN_RIGHT;
   d[row + 1] = -CONTACT_MAX_RIGHT;
   d[row + 2] = CONTACT_MIN_DIR;
   d[row + 3] = -CONTACT_MAX_DIR;
   row += 4;
   }

   diffs_.resize(num_contacts);
   rotations_.resize(num_contacts);
   forces_.resize(num_contacts);
   for (int i = 0; i < num_contacts; ++i)
   {
   rotations_[i] = contact_parent_frames[i].M;
   forces_[i] = contact_forces[i];
   }

   ContactForceSolver* pSolver = this;

   ret = dlevmar_lic_der(torqueEvalWrapper, jacTorqueEvalWrapper, p, x, m, n, C, d, k2, itMax, opts, info, NULL, NULL, pSolver);

   row = 0;
   for (int i = 0; i < num_contacts; ++i)
   {
   const KDL::Rotation& rot = contact_parent_frames[i].M;
   KDL::Vector diff = KDL::Vector(p[row], p[row + 1], dz);
   diff = rot * diff;
   contact_positions[i] = contact_parent_frames[i].p + diff;
   row += 2;
   }
   }*/
}

}
;
