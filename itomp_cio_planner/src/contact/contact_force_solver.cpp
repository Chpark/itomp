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

// data : r_1 vec, r_2 vec, R (6 diagonal value)
void contactWrenchEq(double *p, double *x, int m, int n, void *adata)
{
  double* r1 = (double*) adata;
  double* r2 = r1 + 3;
  const double f1x = *p;
  const double f1y = *(p + 1);
  const double f1z = *(p + 2);
  const double f2x = *(p + 3);
  const double f2y = *(p + 4);
  const double f2z = *(p + 5);

  // force
  x[0] = f1x + f2x;
  x[1] = f1y + f2y;
  x[2] = f1z + f2z;

  // torque
  x[3] = r1[1] * f1z - r1[2] * f1y;
  x[4] = r1[2] * f1x - r1[0] * f1z;
  x[5] = r1[0] * f1y - r1[1] * f1x;
  x[3] += r2[1] * f2z - r2[2] * f2y;
  x[4] += r2[2] * f2x - r2[0] * f2z;
  x[5] += r2[0] * f2y - r2[1] * f2x;
}

void contactWrenchEqSingle(double *p, double *x, int m, int n, void *adata)
{
  double* r1 = (double*) adata;
  const double f1x = *p;
  const double f1y = *(p + 1);
  const double f1z = *(p + 2);

  // force
  x[0] = f1x;
  x[1] = f1y;
  x[2] = f1z;

  // torque
  x[3] = r1[1] * f1z - r1[2] * f1y;
  x[4] = r1[2] * f1x - r1[0] * f1z;
  x[5] = r1[0] * f1y - r1[1] * f1x;
}

void solveContactForces2(double friction_coeff, std::vector<KDL::Vector>& contact_forces,
    std::vector<KDL::Vector>& contact_positions, const KDL::Wrench& wrench, const std::vector<double>& contact_values,
    const std::vector<KDL::Frame> contact_frames)
{
  const KDL::Vector robot_model_dir = KDL::Vector(0.0, 1.0, 0.0);
  const KDL::Vector robot_model_right = KDL::Vector(1.0, 0.0, 0.0);
  const double CONTACT_MIN_DIR = -0.05;
  const double CONTACT_MAX_DIR = 0.19;
  const double CONTACT_MIN_RIGHT = -0.1;
  const double CONTACT_MAX_RIGHT = 0.1;
  KDL::Vector dir1 = contact_frames[0].M * robot_model_dir;
  KDL::Vector dir2 = contact_frames[1].M * robot_model_dir;
  KDL::Vector right1 = contact_frames[0].M * robot_model_right;
  KDL::Vector right2 = contact_frames[1].M * robot_model_right;

  int itMax = 1000;
  double opts[LM_OPTS_SZ], info[LM_INFO_SZ];
  opts[0] = LM_INIT_MU;
  opts[1] = 1E-15;
  opts[2] = 1E-15;
  opts[3] = 1E-20;
  opts[4] = LM_DIFF_DELTA; // relevant only if the Jacobian is approximated using finite differences; specifies forward differencing

  if (contact_values[0] > 0 && contact_values[1] > 0)
  {
    int ret;
    const int m = 10;
    const int n = 6;
    const int k2 = 16;
    double p[m] = { contact_forces[0].x(), contact_forces[0].y(), contact_forces[0].z(), 0, 0, contact_forces[1].x(),
        contact_forces[1].y(), contact_forces[1].z(), 0, 0 };
    double x[n] = { -wrench.force.x(), -wrench.force.y(), -wrench.force.z(), -wrench.torque.x(), -wrench.torque.y(),
        -wrench.torque.z() };
    double C[k2 * m] = { -1.0, 0.0, friction_coeff, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, //
        1.0, 0.0, friction_coeff, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, //
        0.0, -1.0, friction_coeff, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, //
        0.0, 1.0, friction_coeff, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, //
        0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, //
        0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, //
        0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, //
        0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, //
        0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, friction_coeff, 0.0, 0.0, //
        0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, friction_coeff, 0.0, 0.0, //
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, friction_coeff, 0.0, 0.0, //
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, friction_coeff, 0.0, 0.0, //
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, //
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, //
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, //
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0 };
    double d[k2] = { 0.0, 0.0, 0.0, 0.0, CONTACT_MIN_RIGHT, -CONTACT_MAX_RIGHT, CONTACT_MIN_DIR, -CONTACT_MAX_DIR, 0.0,
        0.0, 0.0, 0.0, CONTACT_MIN_RIGHT, -CONTACT_MAX_RIGHT, CONTACT_MIN_DIR, -CONTACT_MAX_DIR };
    double adata[6 + 12] = { contact_positions[0].x(), contact_positions[0].y(), contact_positions[0].z(),
        contact_positions[1].x(), contact_positions[1].y(), contact_positions[1].z(), dir1.x(), dir1.y(), dir1.z(),
        dir2.x(), dir2.y(), dir2.z(), right1.x(), right1.y(), right1.z(), right2.x(), right2.y(), right2.z() };

    ret = dlevmar_lic_dif(contactWrenchEq, p, x, m, n, C, d, k2, itMax, opts, info, NULL, NULL, adata);

    /*
     int i;
     printf("Levenberg-Marquardt returned %d in %g iter, reason %g\nSolution: ", ret, info[5], info[6]);
     for (i = 0; i < m; ++i)
     printf("%.7g ", p[i]);
     printf("\n\nMinimization info:\n");
     for (i = 0; i < LM_INFO_SZ; ++i)
     printf("%g ", info[i]);
     printf("\n");
     */

    contact_forces[0] = KDL::Vector(p[0], p[1], p[2]);
    contact_forces[1] = KDL::Vector(p[5], p[6], p[7]);

    contact_positions[0] += p[3] * right1 + p[4] * dir1;
    contact_positions[1] += p[8] * right2 + p[9] * dir2;
  }
  else
  {
    int ret;
    const int m = 5;
    const int n = 6;
    const int k2 = 8;
    double x[n] = { -wrench.force.x(), -wrench.force.y(), -wrench.force.z(), -wrench.torque.x(), -wrench.torque.y(),
        -wrench.torque.z() };
    double C[k2 * m] = { -1.0, 0.0, friction_coeff, 0.0, 0.0, //
        1.0, 0.0, friction_coeff, 0.0, 0.0, //
        0.0, -1.0, friction_coeff, 0.0, 0.0, //
        0.0, 1.0, friction_coeff, 0.0, 0.0, //
        0.0, 0.0, 0.0, 1.0, 0.0, //
        0.0, 0.0, 0.0, -1.0, 0.0, //
        0.0, 0.0, 0.0, 0.0, 1.0, //
        0.0, 0.0, 0.0, 0.0, -1.0 };
    double d[k2] = { 0.0, 0.0, 0.0, 0.0, CONTACT_MIN_RIGHT, -CONTACT_MAX_RIGHT, CONTACT_MIN_DIR, -CONTACT_MAX_DIR };

    if (contact_values[0] > 0)
    {
      double p[m] = { contact_forces[0].x(), contact_forces[0].y(), contact_forces[0].z(), 0, 0 };

      double adata[3 + 6] = { contact_positions[0].x(), contact_positions[0].y(), contact_positions[0].z(), dir1.x(),
          dir1.y(), dir1.z(), right1.x(), right1.y(), right1.z() };

      ret = dlevmar_lic_dif(contactWrenchEqSingle, p, x, m, n, C, d, k2, itMax, opts, info, NULL, NULL, adata);

      contact_forces[0] = KDL::Vector(p[0], p[1], p[2]);
      contact_forces[1] = KDL::Vector(0, 0, 0);

      contact_positions[0] += p[3] * right1 + p[4] * dir1;
    }
    else // assume contact2 > 0
    {
      double p[m] = { contact_forces[1].x(), contact_forces[1].y(), contact_forces[1].z(), 0, 0 };

      double adata[3 + 6] = { contact_positions[1].x(), contact_positions[1].y(), contact_positions[1].z(), dir2.x(),
          dir2.y(), dir2.z(), right2.x(), right2.y(), right2.z() };

      ret = dlevmar_lic_dif(contactWrenchEqSingle, p, x, m, n, C, d, k2, itMax, opts, info, NULL, NULL, adata);

      contact_forces[0] = KDL::Vector(0, 0, 0);
      contact_forces[1] = KDL::Vector(p[0], p[1], p[2]);

      contact_positions[1] += p[3] * right2 + p[4] * dir2;
    }
  }
  /*
   printf("Wrench : (%f %f %f %f %f %f) Solver result : (%f %f %f)(%f %f %f)\n", -wrench.force.x(), -wrench.force.y(),
   -wrench.force.z(), -wrench.torque.x(), -wrench.torque.y(), -wrench.torque.z(), contact_forces[0].x(), contact_forces[0].y(), contact_forces[0].z(), contact_forces[1].x(), contact_forces[1].y(), contact_forces[1].z());
   */
}

////////////////////////////////////////////////////////////////////////////////
static const double w = 0.00001;
static const double dz = -0.1;

static std::vector<KDL::Vector> g_diffs;
static std::vector<KDL::Rotation> g_rotations;
static std::vector<KDL::Vector> g_forces;

void torqueEval(double *p, double *x, int m, int n, void *adata)
{
  int num_contacts = m / 2;

  KDL::Vector torque_sum = KDL::Vector::Zero();

  // (p-r) x f + r x f = p x f
  // R * d = (p-r)
  // Rd x f = -W.torque - r x f
  int p_pos = 0;
  for (int i = 0; i < num_contacts; ++i)
  {
    g_diffs[i] = KDL::Vector(*(p + p_pos), *(p + p_pos + 1), dz);
    g_diffs[i] = g_rotations[i] * g_diffs[i];
    p_pos += 2;
  }

  // Rd x f
  p_pos = 3;
  for (int i = 0; i < num_contacts; ++i)
  {
    KDL::Vector torque = g_diffs[i] * g_forces[i];
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

void jacTorqueEval(double *p, double *jac, int m, int n, void *adata)
{
  int num_contacts = m / 2;

  memset(jac, 0, sizeof(double) * m * n);

  int row = 3;
  int col = 0;
  for (int i = 0; i < num_contacts; ++i)
  {
    const KDL::Rotation& R = g_rotations[i];
    const KDL::Vector& f = g_forces[i];

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

void solveContactForces(double friction_coeff, std::vector<KDL::Vector>& contact_forces,
    std::vector<KDL::Vector>& contact_positions, const KDL::Wrench& wrench, const std::vector<double>& contact_values,
    const std::vector<KDL::Frame> contact_parent_frames)
{
  const double k_0 = 0.01;
  const double k_1 = 0.001;

  /*
  static int time_count = 0;
  static int iterations_count = 0;
  static double time[2] = { 0, 0 };
  double time_start, time_end, time_elapsed;
  */

  int num_contacts = contact_forces.size();

  // compute contact forces using linear programming
  //time_start = ros::Time::now().toSec();
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
  /*
  time_end = ros::Time::now().toSec();
  time_elapsed = time_end - time_start;
  time[0] += time_elapsed;

  time_start = ros::Time::now().toSec();
  */
  // compute center of pressure (contact positions) using levmar
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

    g_diffs.resize(num_contacts);
    g_rotations.resize(num_contacts);
    g_forces.resize(num_contacts);
    for (int i = 0; i < num_contacts; ++i)
    {
      g_rotations[i] = contact_parent_frames[i].M;
      g_forces[i] = contact_forces[i];
    }

    ret = dlevmar_lic_der(torqueEval, jacTorqueEval, p, x, m, n, C, d, k2, itMax, opts, info, NULL, NULL, NULL);

    row = 0;
    for (int i = 0; i < num_contacts; ++i)
    {
      const KDL::Rotation& rot = contact_parent_frames[i].M;
      KDL::Vector diff = KDL::Vector(p[row], p[row + 1], dz);
      diff = rot * diff;
      contact_positions[i] = contact_parent_frames[i].p + diff;
      row += 2;
    }
    //iterations_count += info[5];
  }
  /*
  time_end = ros::Time::now().toSec();
  time_elapsed = time_end - time_start;
  time[1] += time_elapsed;
  if ((++time_count % 1000) == 0)
  {
    printf("Time for 1000 evals (%d itr) : %f %f\n", iterations_count / 1000, time[0], time[1]);
    time[0] = time[1] = 0;
    iterations_count = 0;
  }
  */
}

