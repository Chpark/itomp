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
}

bool ImprovementManagerNLP::updatePlanningParameters()
{
  if (!ImprovementManager::updatePlanningParameters())
    return false;

  num_dimensions_ = evaluation_manager_->getGroupTrajectory()->getNumJoints();
  num_contact_dimensions_ = evaluation_manager_->getGroupTrajectory()->getNumContacts();
  num_points_ = evaluation_manager_->getGroupTrajectory()->getNumPoints();
  num_contact_phases_ = evaluation_manager_->getGroupTrajectory()->getNumContactPhases();

  num_contact_vars_free_ = num_contact_dimensions_ * num_contact_phases_;
  num_pos_variables_ = num_dimensions_ * (num_contact_phases_ - 1);
  num_vel_variables_ = num_dimensions_ * (num_contact_phases_ - 1);
  num_variables_ = num_contact_vars_free_ + num_pos_variables_ + num_vel_variables_;

  variables_.set_size(num_variables_);

  int num_free_points_ = num_contact_phases_ - 1;
  parameters_ = Eigen::MatrixXd(num_contact_phases_ - 1, num_dimensions_);
  vel_parameters_ = Eigen::MatrixXd(num_contact_phases_ - 1, num_dimensions_);
  contact_parameters_ = Eigen::MatrixXd(num_contact_phases_, num_contact_dimensions_);
  costs_ = Eigen::VectorXd::Zero(num_points_);

  return true;
}

void ImprovementManagerNLP::runSingleIteration(int iteration)
{
  setVariableVector();

  if (iteration != 0)
    addNoiseToVariables();

  optimize(iteration);
}

void ImprovementManagerNLP::setVariableVector()
{
  int writeIndex = 0;
  // copy from group_trajectory_:
  const Eigen::MatrixXd& freeTrajectoryBlock = evaluation_manager_->getGroupTrajectory()->getFreePoints();
  const Eigen::MatrixXd& freeVelTrajectoryBlock = evaluation_manager_->getGroupTrajectory()->getFreeVelPoints();
  const Eigen::MatrixXd& contactTrajectoryBlock = evaluation_manager_->getGroupTrajectory()->getContactTrajectory();
  // contact variable for phase 0
  for (int d = 0; d < num_contact_dimensions_; ++d)
  {
    variables_(writeIndex + d, 0) = contactTrajectoryBlock(0, d);
  }
  writeIndex += num_contact_dimensions_;

  for (int i = 1; i < num_contact_phases_; ++i)
  {
    for (int d = 0; d < num_dimensions_; ++d)
    {
      variables_(writeIndex + d, 0) = freeTrajectoryBlock(i, d);
    }
    writeIndex += num_dimensions_;
    for (int d = 0; d < num_dimensions_; ++d)
    {
      variables_(writeIndex + d, 0) = freeVelTrajectoryBlock(i, d);
    }
    writeIndex += num_dimensions_;
    for (int d = 0; d < num_contact_dimensions_; ++d)
    {
      variables_(writeIndex + d, 0) = contactTrajectoryBlock(i, d);
    }
    writeIndex += num_contact_dimensions_;
  }
}

void ImprovementManagerNLP::getVariableVector()
{
  int readIndex = 0;
  // copy to group_trajectory_:
  for (int d = 0; d < num_contact_dimensions_; ++d)
  {
    contact_parameters_(0, d) = abs(variables_(readIndex + d, 0));
  }
  readIndex += num_contact_dimensions_;
  for (int i = 0; i < num_contact_phases_ - 1; ++i)
  {
    for (int d = 0; d < num_dimensions_; ++d)
    {
      parameters_(i, d) = variables_(readIndex + d, 0);
    }
    readIndex += num_dimensions_;
    for (int d = 0; d < num_dimensions_; ++d)
    {
      vel_parameters_(i, d) = variables_(readIndex + d, 0);
    }
    readIndex += num_dimensions_;
    for (int d = 0; d < num_contact_dimensions_; ++d)
    {
      contact_parameters_(i + 1, d) = abs(variables_(readIndex + d, 0));
    }
    readIndex += num_contact_dimensions_;
  }
}

void ImprovementManagerNLP::addNoiseToVariables()
{
  MultivariateGaussian noise_generator(VectorXd::Zero(num_variables_),
      MatrixXd::Identity(num_variables_, num_variables_));
  VectorXd noise = VectorXd::Zero(num_variables_);
  noise_generator.sample(noise);
  for (int i = 0; i < num_variables_; ++i)
  {
    variables_(i) += 0.01 * noise(i);
  }
}

////////////////////////////////////////////////////////////////////////////////

/*
 EvaluationManager* evaluation_manager_;

 int num_dimensions_;
 int num_contact_dimensions_;
 int num_free_points_;
 int num_points_;
 */
/*
 class evaluation_functor
 {

 public:
 evaluation_functor(EvaluationManager* evaluation_manager, int num_dimensions, int num_contact_dimensions,
 int num_free_points, int num_points)
 {
 evaluation_manager_ = evaluation_manager;
 num_dimensions_ = num_dimensions;
 num_contact_dimensions_ = num_contact_dimensions;
 num_free_points_ = num_free_points;
 num_points_ = num_points;

 parameters_ = Eigen::MatrixXd(num_free_points_, num_dimensions_);
 vel_parameters_ = Eigen::MatrixXd(num_free_points_, num_dimensions_);
 contact_parameters_ = Eigen::MatrixXd(num_free_points_ + 1, num_contact_dimensions_);
 costs_ = Eigen::VectorXd::Zero(num_points_);
 }

 double operator()(const column_vector& variables) const
 {
 int readIndex = 0;
 // copy to group_trajectory_:
 for (int d = 0; d < num_contact_dimensions_; ++d)
 {
 contact_parameters_(0, d) = abs(variables(readIndex + d, 0));
 }
 readIndex += num_contact_dimensions_;
 for (int i = 0; i < num_free_points_; ++i)
 {
 for (int d = 0; d < num_dimensions_; ++d)
 {
 parameters_(i, d) = variables(readIndex + d, 0);
 }
 readIndex += num_dimensions_;
 for (int d = 0; d < num_dimensions_; ++d)
 {
 vel_parameters_(i, d) = variables(readIndex + d, 0);
 }
 readIndex += num_dimensions_;
 for (int d = 0; d < num_contact_dimensions_; ++d)
 {
 contact_parameters_(i + 1, d) = abs(variables(readIndex + d, 0));
 }
 readIndex += num_contact_dimensions_;
 }

 return evaluation_manager_->evaluate(parameters_, vel_parameters_, contact_parameters_, costs_);
 }
 };
 */
/*
 column_vector derivative_;

 class derivative_functor
 {

 public:

 derivative_functor(EvaluationManager* evaluation_manager, int num_dimensions, int num_contact_dimensions,
 int num_free_points, int num_points)
 {
 evaluation_manager_ = evaluation_manager;
 num_dimensions_ = num_dimensions;
 num_contact_dimensions_ = num_contact_dimensions;
 num_free_points_ = num_free_points;
 num_points_ = num_points;

 parameters_ = Eigen::MatrixXd(num_free_points_, num_dimensions_);
 vel_parameters_ = Eigen::MatrixXd(num_free_points_, num_dimensions_);
 contact_parameters_ = Eigen::MatrixXd(num_free_points_ + 1, num_contact_dimensions_);
 costs_ = Eigen::VectorXd::Zero(num_points_);
 }

 column_vector operator()(const column_vector& variables) const
 {
 int readIndex = 0;
 // copy to group_trajectory_:
 for (int d = 0; d < num_contact_dimensions_; ++d)
 {
 contact_parameters_(0, d) = abs(variables(readIndex + d, 0));
 }
 readIndex += num_contact_dimensions_;
 for (int i = 0; i < num_free_points_; ++i)
 {
 for (int d = 0; d < num_dimensions_; ++d)
 {
 parameters_(i, d) = variables(readIndex + d, 0);
 }
 readIndex += num_dimensions_;
 for (int d = 0; d < num_dimensions_; ++d)
 {
 vel_parameters_(i, d) = variables(readIndex + d, 0);
 }
 readIndex += num_dimensions_;
 for (int d = 0; d < num_contact_dimensions_; ++d)
 {
 contact_parameters_(i + 1, d) = abs(variables(readIndex + d, 0));
 }
 readIndex += num_contact_dimensions_;
 }

 evaluation_manager_->evaluate(parameters_, vel_parameters_, contact_parameters_, costs_);

 derivative_.set_size(variables.size());
 return derivative_;
 }
 };
 */
double ImprovementManagerNLP::evaluate(const column_vector& variables)
{
  getVariableVector();
  return evaluation_manager_->evaluate(parameters_, vel_parameters_, contact_parameters_, costs_);
}

column_vector ImprovementManagerNLP::derivative(const column_vector& variables)
{
  const double eps = 1E-7;

  column_vector der(variables.size());
  column_vector e(variables);

  for (long i = 0; i < variables.size(); ++i)
  {
    const double old_val = e(i);

    e(i) += eps;
    const double delta_plus = evaluate(e);
    e(i) = old_val - eps;
    const double delta_minus = evaluate(e);

    der(i) = (delta_plus - delta_minus) / (2 * eps);

    // and finally restore the old value of this element
    e(i) = old_val;
  }

  return der;
}

void ImprovementManagerNLP::optimize(int iteration)
{
  /*
  dlib::find_min(dlib::bfgs_search_strategy(), dlib::objective_delta_stop_strategy(1e-7).be_verbose(),
      boost::bind(&ImprovementManagerNLP::evaluate, this, _1),
      boost::bind(&ImprovementManagerNLP::derivative, this, _1), variables_, -1);
      */

  dlib::find_min_using_approximate_derivatives(dlib::bfgs_search_strategy(),
      dlib::objective_delta_stop_strategy(1e-7).be_verbose(), boost::bind(&ImprovementManagerNLP::evaluate, this, _1),
      variables_, -1);

}

}
