#include <itomp_cio_planner/optimization/improvement_manager_nlp.h>
#include <itomp_cio_planner/util/multivariate_gaussian.h>
#include <omp.h>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <functional>

using namespace Eigen;

namespace itomp_cio_planner
{

ImprovementManagerNLP::ImprovementManagerNLP()
{

}

ImprovementManagerNLP::~ImprovementManagerNLP()
{

}

void ImprovementManagerNLP::initialize(EvaluationManager *evaluation_manager)
{
  ImprovementManager::initialize(evaluation_manager);

  omp_set_num_threads(omp_get_max_threads());
}

bool ImprovementManagerNLP::updatePlanningParameters()
{
  if (!ImprovementManager::updatePlanningParameters())
    return false;

  const ItompCIOTrajectory* group_trajectory = evaluation_manager_->getGroupTrajectory();

  num_dimensions_ = group_trajectory->getNumJoints();
  num_contact_dimensions_ = group_trajectory->getNumContacts();
  num_points_ = group_trajectory->getNumPoints();
  num_contact_phases_ = group_trajectory->getNumContactPhases();

  num_contact_vars_free_ = num_contact_dimensions_ * num_contact_phases_;
  num_pos_variables_ = num_dimensions_ * (num_contact_phases_ - 1);
  num_vel_variables_ = num_dimensions_ * (num_contact_phases_ - 1);
  num_variables_ = num_contact_vars_free_ + num_pos_variables_ + num_vel_variables_;

  parameters_ = group_trajectory->getFreePoints();
  vel_parameters_ = group_trajectory->getFreeVelPoints();
  contact_parameters_ = group_trajectory->getContactTrajectory();
  costs_ = Eigen::VectorXd::Zero(num_points_);
  derivatives_ = Eigen::VectorXd::Zero(num_variables_);

  return true;
}

void ImprovementManagerNLP::runSingleIteration(int iteration)
{
  column_vector variables(num_variables_);
  setVariableVector(variables);

  if (iteration != 0)
    addNoiseToVariables(variables);

  optimize(iteration, variables);
}

void ImprovementManagerNLP::setVariableVector(column_vector& variables)
{
  int writeIndex = 0;

  // positions
  for (int i = 1; i < num_contact_phases_; ++i)
  {
    for (int d = 0; d < num_dimensions_; ++d)
    {
      variables(writeIndex + d, 0) = parameters_(i, d);
    }
    writeIndex += num_dimensions_;
  }

  // velocities
  for (int i = 1; i < num_contact_phases_; ++i)
  {
    for (int d = 0; d < num_dimensions_; ++d)
    {
      variables(writeIndex + d, 0) = vel_parameters_(i, d);
    }
    writeIndex += num_dimensions_;
  }

  // contact variables
  for (int i = 0; i < num_contact_phases_; ++i)
  {
    for (int d = 0; d < num_contact_dimensions_; ++d)
    {
      variables(writeIndex + d, 0) = contact_parameters_(i, d);
    }
    writeIndex += num_contact_dimensions_;
  }
}

void ImprovementManagerNLP::getVariableVector(const column_vector& variables)
{
  int readIndex = 0;

  // positions
  for (int i = 1; i < num_contact_phases_; ++i)
  {
    for (int d = 0; d < num_dimensions_; ++d)
    {
      parameters_(i, d) = variables(readIndex + d, 0);
    }
    readIndex += num_dimensions_;
  }

  // velocities
  for (int i = 1; i < num_contact_phases_; ++i)
  {
    for (int d = 0; d < num_dimensions_; ++d)
    {
      vel_parameters_(i, d) = variables(readIndex + d, 0);
    }
    readIndex += num_dimensions_;
  }

  // contact variables
  for (int i = 0; i < num_contact_phases_; ++i)
  {
    for (int d = 0; d < num_contact_dimensions_; ++d)
    {
      contact_parameters_(i, d) = fabs(variables(readIndex + d, 0));
    }
    readIndex += num_contact_dimensions_;
  }
}

void ImprovementManagerNLP::addNoiseToVariables(column_vector& variables)
{
  MultivariateGaussian noise_generator(VectorXd::Zero(num_variables_),
      MatrixXd::Identity(num_variables_, num_variables_));
  VectorXd noise = VectorXd::Zero(num_variables_);
  noise_generator.sample(noise);
  for (int i = 0; i < num_variables_; ++i)
  {
    variables(i) += 0.01 * noise(i);
  }
}

double ImprovementManagerNLP::evaluate(const column_vector& variables)
{
  getVariableVector(variables);
  return evaluation_manager_->evaluate(parameters_, vel_parameters_, contact_parameters_, costs_);
}

column_vector ImprovementManagerNLP::derivative(const column_vector& variables)
{
  // assume evaluate is called before
  // getVariableVector(variables);

  const double eps = 1E-7;

  column_vector der(variables.size());
  evaluation_manager_->evaluateDerivatives(parameters_, vel_parameters_, contact_parameters_, derivatives_);
  for (int i = 0; i < variables.size(); ++i)
    der(i) = derivatives_(i);

  /*
  column_vector e(variables);

  //#pragma omp parallel for
  for (long i = 0; i < variables.size(); ++i)
  {
    const double old_val = e(i);

    e(i) += eps;
    const double delta_plus = evaluate(e);
    e(i) = old_val - eps;
    const double delta_minus = evaluate(e);

    der(i) = (delta_plus - delta_minus) / (2 * eps);

    e(i) = old_val;
  }
  */

  return der;
}

void ImprovementManagerNLP::optimize(int iteration, column_vector& variables)
{
  dlib::find_min(dlib::bfgs_search_strategy(), dlib::objective_delta_stop_strategy(1e-7).be_verbose(),
      boost::bind(&ImprovementManagerNLP::evaluate, this, _1),
      boost::bind(&ImprovementManagerNLP::derivative, this, _1), variables, -1);
}

}
