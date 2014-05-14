#include <itomp_cio_planner/optimization/improvement_manager_nlp.h>
#include <itomp_cio_planner/util/multivariate_gaussian.h>
#include <omp.h>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <functional>

using namespace Eigen;

namespace itomp_cio_planner
{

static bool verbose = false;

ImprovementManagerNLP::ImprovementManagerNLP()
{
  elapsed_ = 0;
}

ImprovementManagerNLP::~ImprovementManagerNLP()
{

}

void ImprovementManagerNLP::initialize(EvaluationManager *evaluation_manager)
{
  ImprovementManager::initialize(evaluation_manager);

  num_threads_ = omp_get_max_threads();
  omp_set_num_threads(num_threads_);

  derivatives_evaluation_manager_.resize(num_threads_);
  derivatives_evaluation_data_.resize(num_threads_);
  const EvaluationData& default_data = evaluation_manager->getDefaultData();
  for (int i = 0; i < num_threads_; ++i)
  {
    derivatives_evaluation_data_[i].reset(default_data.clone());
    derivatives_evaluation_manager_[i].reset(new EvaluationManager(*evaluation_manager));
    derivatives_evaluation_manager_[i]->setData(derivatives_evaluation_data_[i].get());
  }
}

bool ImprovementManagerNLP::updatePlanningParameters()
{
  if (!ImprovementManager::updatePlanningParameters())
    return false;

  const ItompCIOTrajectory* group_trajectory = evaluation_manager_->getGroupTrajectoryConst();

  num_dimensions_ = group_trajectory->getNumJoints();
  num_contact_dimensions_ = group_trajectory->getNumContacts();
  num_points_ = group_trajectory->getNumPoints();
  num_contact_phases_ = group_trajectory->getNumContactPhases();

  num_contact_vars_free_ = num_contact_dimensions_ * num_contact_phases_;
  num_pos_variables_ = num_dimensions_ * (num_contact_phases_ - 1);
  num_vel_variables_ = num_dimensions_ * (num_contact_phases_ - 1);
  num_variables_ = num_contact_vars_free_ + num_pos_variables_ + num_vel_variables_;

  parameters_.resize(num_threads_);
  vel_parameters_.resize(num_threads_);
  contact_parameters_.resize(num_threads_);
  for (int i = 0; i < num_threads_; ++i)
  {
    parameters_[i] = group_trajectory->getFreePoints();
    vel_parameters_[i] = group_trajectory->getFreeVelPoints();
    contact_parameters_[i] = group_trajectory->getContactTrajectory();
  }
  derivatives_ = Eigen::VectorXd::Zero(num_variables_);

  return true;
}

void ImprovementManagerNLP::runSingleIteration(int iteration)
{
  column_vector variables(num_variables_);

  const ItompCIOTrajectory* group_trajectory = evaluation_manager_->getGroupTrajectoryConst();
  for (int i = 0; i < num_threads_; ++i)
  {
    parameters_[i] = group_trajectory->getFreePoints();
    vel_parameters_[i] = group_trajectory->getFreeVelPoints();
    contact_parameters_[i] = group_trajectory->getContactTrajectory();
  }
  setVariableVector(variables);

  if (iteration != 0)
    addNoiseToVariables(variables);

  optimize(iteration, variables);

  verbose = true;
}

void ImprovementManagerNLP::setVariableVector(column_vector& variables)
{
  const ItompCIOTrajectory* group_trajectory = evaluation_manager_->getGroupTrajectoryConst();
  parameters_[0] = group_trajectory->getFreePoints();
  vel_parameters_[0] = group_trajectory->getFreeVelPoints();
  contact_parameters_[0] = group_trajectory->getContactTrajectory();

  int writeIndex = 0;

  // positions
  for (int i = 1; i < num_contact_phases_; ++i)
  {
    for (int d = 0; d < num_dimensions_; ++d)
    {
      variables(writeIndex + d, 0) = parameters_[0](i, d);
    }
    writeIndex += num_dimensions_;
  }

  // velocities
  for (int i = 1; i < num_contact_phases_; ++i)
  {
    for (int d = 0; d < num_dimensions_; ++d)
    {
      variables(writeIndex + d, 0) = vel_parameters_[0](i, d);
    }
    writeIndex += num_dimensions_;
  }

  // contact variables
  for (int i = 0; i < num_contact_phases_; ++i)
  {
    for (int d = 0; d < num_contact_dimensions_; ++d)
    {
      variables(writeIndex + d, 0) = contact_parameters_[0](i, d);
    }
    writeIndex += num_contact_dimensions_;
  }
}

void ImprovementManagerNLP::getVariableVector(const column_vector& variables, int index)
{
  int readIndex = 0;

  // positions
  for (int i = 1; i < num_contact_phases_; ++i)
  {
    for (int d = 0; d < num_dimensions_; ++d)
    {
      double v = variables(readIndex + d, 0);
      parameters_[index](i, d) = v;
    }
    readIndex += num_dimensions_;
  }

  // velocities
  for (int i = 1; i < num_contact_phases_; ++i)
  {
    for (int d = 0; d < num_dimensions_; ++d)
    {
      vel_parameters_[index](i, d) = variables(readIndex + d, 0);
    }
    readIndex += num_dimensions_;
  }

  // contact variables
  for (int i = 0; i < num_contact_phases_; ++i)
  {
    for (int d = 0; d < num_contact_dimensions_; ++d)
    {
      contact_parameters_[index](i, d) = fabs(variables(readIndex + d, 0));
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
  evaluation_manager_->setTrajectory(parameters_[0], vel_parameters_[0], contact_parameters_[0]);
  return evaluation_manager_->evaluate();
}

column_vector ImprovementManagerNLP::derivative_ref(const column_vector& variables)
{
  const EvaluationData& default_data = evaluation_manager_->getDefaultData();

  const double eps = 1E-7;
  column_vector der(variables.size());

  column_vector e(variables);

  for (long i = 0; i < variables.size(); ++i)
  {
    const double old_val = e(i);

    e(i) += eps;
    getVariableVector(e);

    evaluation_manager_->setTrajectory(parameters_[0], vel_parameters_[0], contact_parameters_[0]);
    const double delta_plus = evaluation_manager_->evaluate();

    e(i) = old_val - eps;
    getVariableVector(e);

    evaluation_manager_->setTrajectory(parameters_[0], vel_parameters_[0], contact_parameters_[0]);
    double delta_minus = evaluation_manager_->evaluate();

    der(i) = (delta_plus - delta_minus) / (2 * eps);

    e(i) = old_val;
  }

  return der;
}

column_vector ImprovementManagerNLP::derivative(const column_vector& variables)
{
  column_vector e(variables);

  // assume evaluate was called before and default_data has the values for 'variables' vector
  const EvaluationData& default_data = evaluation_manager_->getDefaultData();

  const double eps = 1E-7;

  for (int i = 0; i < num_threads_; ++i)
    derivatives_evaluation_data_[i]->deepCopy(default_data);

  //column_vector der_ref = derivative_ref(variables);

  column_vector der(variables.size());

  const int num_positions = (num_contact_phases_ - 1) * num_dimensions_;
  const int num_velocities = (num_contact_phases_ - 1) * num_dimensions_;
  const int num_contact_variables = num_contact_phases_ * num_contact_dimensions_;

  int read_index = 0;

  ros::Time time[10];
  time[0] = ros::Time::now();

#pragma omp parallel for
  for (int index = 0; index < num_positions + num_velocities + num_contact_variables; ++index)
  {
    int thread_num = omp_get_thread_num();
    double value = variables(index);

    int point_index, joint_index;
    EvaluationManager::DERIVATIVE_VARIABLE_TYPE type;
    double value_plus, value_minus;
    int index2;
    if (index < num_positions)
    {
      point_index = (index / num_dimensions_) + 1;
      joint_index = index % num_dimensions_;
      type = EvaluationManager::DERIVATIVE_POSITION_VARIABLE;
      value_plus = value + eps;
      value_minus = value - eps;
    }
    else if (index < num_positions + num_velocities)
    {
      index2 = index - num_positions;
      point_index = (index2 / num_dimensions_) + 1;
      joint_index = index2 % num_dimensions_;
      type = EvaluationManager::DERIVATIVE_VELOCITY_VARIABLE;
      value_plus = value + eps;
      value_minus = value - eps;
    }
    else
    {
      index2 = index - num_positions - num_velocities;
      point_index = (index2 / num_contact_dimensions_);
      joint_index = index2 % num_contact_dimensions_;
      type = EvaluationManager::DERIVATIVE_CONTACT_VARIABLE;
      value_plus = fabs(value + eps);
      value_minus = fabs(value - eps);
    }

    double delta_plus = derivatives_evaluation_manager_[thread_num]->evaluateDerivatives(value_plus, type, point_index,
        joint_index);

    double delta_minus = derivatives_evaluation_manager_[thread_num]->evaluateDerivatives(value_minus, type,
        point_index, joint_index);

    der(index) = (delta_plus - delta_minus) / (2 * eps);
  }

  time[1] = ros::Time::now();
  elapsed_ += (time[1] - time[0]).toSec();

  printf("Elapsed : %f (%f)\n", elapsed_, (time[1] - time[0]).toSec());

  // validation
  /*
   for (int i = 0; i < variables.size(); ++i)
   {
   if (fabs(der(i) - der_ref(i)) > 10.0)
   printf("[%d] %f (%f %f)\n", i, der(i) - der_ref(i), der(i), der_ref(i));
   }
   */

  return der;
}

void ImprovementManagerNLP::optimize(int iteration, column_vector& variables)
{
  dlib::find_min(dlib::lbfgs_search_strategy(10), dlib::objective_delta_stop_strategy(1e-7).be_verbose(),
      boost::bind(&ImprovementManagerNLP::evaluate, this, _1),
      boost::bind(&ImprovementManagerNLP::derivative, this, _1), variables, -1);
}

}
