#include <itomp_cio_planner/cost/smoothness_cost.h>
#include <itomp_cio_planner/util/differentiation_rules.h>
#include <Eigen/LU>

using namespace std;
using namespace Eigen;

namespace itomp_cio_planner
{

SmoothnessCost::SmoothnessCost(const ItompCIOTrajectory& trajectory, int joint_number,
    const std::vector<double>& derivative_costs, double ridge_factor)
{
  int num_vars_all = trajectory.getNumPoints();
  int num_vars_free = num_vars_all - 2 * (DIFF_RULE_LENGTH - 1);
  MatrixXd diff_matrix = MatrixXd::Zero(num_vars_all, num_vars_all);
  quad_cost_full_ = MatrixXd::Zero(num_vars_all, num_vars_all);

  // construct the quad cost for all variables, as a sum of squared differentiation matrices
  double multiplier = 1.0;
  for (unsigned int i = 0; i < derivative_costs.size(); i++)
  {
    multiplier *= trajectory.getDiscretization();
    diff_matrix = getDiffMatrix(num_vars_all, &DIFF_RULES[i][0]);
    quad_cost_full_ += (derivative_costs[i] * multiplier) * (diff_matrix.transpose() * diff_matrix);
  }
  quad_cost_full_ += MatrixXd::Identity(num_vars_all, num_vars_all) * ridge_factor;

  // extract the quad cost just for the free variables:
  quad_cost_ = quad_cost_full_.block(DIFF_RULE_LENGTH - 1, DIFF_RULE_LENGTH - 1, num_vars_free, num_vars_free);

  // invert the matrix:
  quad_cost_inv_ = quad_cost_.inverse();
}

Eigen::MatrixXd SmoothnessCost::getDiffMatrix(int size, const double* diff_rule) const
{
  MatrixXd matrix = MatrixXd::Zero(size, size);
  for (int i = 0; i < size; i++)
  {
    if (i - DIFF_RULE_LENGTH / 2 < 0 || i + DIFF_RULE_LENGTH / 2 >= size)
      continue;

    for (int j = -DIFF_RULE_LENGTH / 2; j <= DIFF_RULE_LENGTH / 2; j++)
    {
      int index = i + j;
      matrix(i, index) = diff_rule[j + DIFF_RULE_LENGTH / 2];
    }
  }
  return matrix;
}

double SmoothnessCost::getMaxQuadCostInvValue() const
{
  return quad_cost_inv_.maxCoeff();
}

void SmoothnessCost::scale(double scale)
{
  double inv_scale = 1.0 / scale;
  quad_cost_inv_ *= inv_scale;
  quad_cost_ *= scale;
  quad_cost_full_ *= scale;
}

SmoothnessCost::~SmoothnessCost()
{
}

}
