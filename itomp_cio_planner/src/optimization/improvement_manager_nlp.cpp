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

  num_threads_ = omp_get_max_threads() - 2;
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
  costs_.resize(num_threads_);
  for (int i = 0; i < num_threads_; ++i)
  {
    parameters_[i] = group_trajectory->getFreePoints();
    vel_parameters_[i] = group_trajectory->getFreeVelPoints();
    contact_parameters_[i] = group_trajectory->getContactTrajectory();
    costs_[i] = Eigen::VectorXd::Zero(num_points_);
  }
  derivatives_ = Eigen::VectorXd::Zero(num_variables_);

  return true;
}

void ImprovementManagerNLP::runSingleIteration(int iteration)
{
  column_vector variables(num_variables_);
  setVariableVector(variables);

  if (iteration != 0)
    addNoiseToVariables(variables);

  const ItompCIOTrajectory* group_trajectory = evaluation_manager_->getGroupTrajectoryConst();
  for (int i = 0; i < num_threads_; ++i)
  {
    parameters_[i] = group_trajectory->getFreePoints();
    vel_parameters_[i] = group_trajectory->getFreeVelPoints();
    contact_parameters_[i] = group_trajectory->getContactTrajectory();
    costs_[i] = Eigen::VectorXd::Zero(num_points_);
  }
  setVariableVector(variables);

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
      parameters_[index](i, d) = variables(readIndex + d, 0);
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
  return evaluation_manager_->evaluate(parameters_[0], vel_parameters_[0], contact_parameters_[0], costs_[0]);
}

column_vector ImprovementManagerNLP::derivative(const column_vector& variables)
{
  // assume evaluate was called before and default_data has the values for 'variables' vector
  const EvaluationData& default_data = evaluation_manager_->getDefaultData();

  const double eps = 1E-7;

  for (int i = 0; i < num_threads_; ++i)
    derivatives_evaluation_data_[i]->deepCopy(default_data);

  column_vector der(variables.size());

  const int num_positions = (num_contact_phases_ - 1) * num_dimensions_;
  const int num_velocities = (num_contact_phases_ - 1) * num_dimensions_;
  const int num_contact_variables = num_contact_phases_ * num_contact_dimensions_;

  int read_index = 0;

  ros::Time time[10];
  time[0] = ros::Time::now();

  // positions
//#pragma omp parallel for
  for (int index = 0; index < num_positions; ++index)
  {
    int thread_num = 0;//omp_get_thread_num();

    const int point_index = (index / num_dimensions_) + 1;
    const int joint_index = index % num_dimensions_;

    double value = variables(read_index + index);

    double delta_plus = derivatives_evaluation_manager_[thread_num]->evaluateDerivatives(value + eps,
        EvaluationManager::DERIVATIVE_POSITION_VARIABLE, point_index, joint_index);
    double delta_minus = derivatives_evaluation_manager_[thread_num]->evaluateDerivatives(value - eps,
        EvaluationManager::DERIVATIVE_POSITION_VARIABLE, point_index, joint_index);

    der(read_index + index) = (delta_plus - delta_minus) / (2 * eps);
  }

  read_index += num_positions;

  // velocities
//#pragma omp parallel for
  for (int index = 0; index < num_velocities; ++index)
  {
    int thread_num = 0;// omp_get_thread_num();

    const int point_index = (index / num_dimensions_) + 1;
    const int joint_index = index % num_dimensions_;

    double value = variables(read_index + index);

    double delta_plus = derivatives_evaluation_manager_[thread_num]->evaluateDerivatives(value + eps,
        EvaluationManager::DERIVATIVE_VELOCITY_VARIABLE, point_index, joint_index);
    double delta_minus = derivatives_evaluation_manager_[thread_num]->evaluateDerivatives(value - eps,
        EvaluationManager::DERIVATIVE_VELOCITY_VARIABLE, point_index, joint_index);

    der(read_index + index) = (delta_plus - delta_minus) / (2 * eps);
  }

  read_index += num_velocities;

  // contact variables
//#pragma omp parallel for
  for (int index = 0; index < num_contact_variables; ++index)
  {
    int thread_num = 0;//omp_get_thread_num();

    const int point_index = (index / num_contact_dimensions_);
    const int joint_index = index % num_contact_dimensions_;

    double value = variables(read_index + index);

    double delta_plus = derivatives_evaluation_manager_[thread_num]->evaluateDerivatives(value + eps,
        EvaluationManager::DERIVATIVE_CONTACT_VARIABLE, point_index, joint_index);
    double delta_minus = derivatives_evaluation_manager_[thread_num]->evaluateDerivatives(value - eps,
        EvaluationManager::DERIVATIVE_CONTACT_VARIABLE, point_index, joint_index);

    der(read_index + index) = (delta_plus - delta_minus) / (2 * eps);
  }

  time[1] = ros::Time::now();
  elapsed_ += (time[1] - time[0]).toSec();

  printf("Elapsed : %f (%f)\n", elapsed_, (time[1] - time[0]).toSec());

  return der;
}

column_vector ImprovementManagerNLP::derivative_old(const column_vector& variables)
{
  // assume evaluate was called before
  // getVariableVector(variables);
  const EvaluationData& default_data = evaluation_manager_->getDefaultData();

  const double eps = 1E-7;

  column_vector der(variables.size());

  for (int i = 0; i < num_threads_; ++i)
    derivatives_evaluation_data_[i]->deepCopy(default_data);

  std::vector<column_vector> e(num_threads_);
  for (int i = 0; i < num_threads_; ++i)
    e[i] = variables;

  ros::Time time[10];
  time[0] = ros::Time::now();

#pragma omp parallel for
  for (long i = 0; i < variables.size(); ++i)
  {
    int thread_num = omp_get_thread_num();
    //column_vector e(variables);

    const double old_val = e[thread_num](i);

    e[thread_num](i) += eps;
    getVariableVector(e[thread_num], thread_num);

    const double delta_plus = derivatives_evaluation_manager_[thread_num]->evaluate(parameters_[thread_num],
        vel_parameters_[thread_num], contact_parameters_[thread_num], costs_[thread_num]);

    e[thread_num](i) = old_val - eps;
    getVariableVector(e[thread_num], thread_num);

    double delta_minus = derivatives_evaluation_manager_[thread_num]->evaluate(parameters_[thread_num],
        vel_parameters_[thread_num], contact_parameters_[thread_num], costs_[thread_num]);

    der(i) = (delta_plus - delta_minus) / (2 * eps);

    e[thread_num](i) = old_val;
  }

  time[1] = ros::Time::now();
  elapsed_ += (time[1] - time[0]).toSec();

  printf("Elapsed : %f (%f)\n", elapsed_, (time[1] - time[0]).toSec());

  /*
   for (int i = 0; i < variables.size(); ++i)
   {
   if (verbose)
   {
   int thread_num = 0;

   const double old_val = e[thread_num](i);

   e[thread_num](i) += eps;
   getVariableVector(e[thread_num], thread_num);

   const double delta_plus = derivatives_evaluation_manager_[thread_num]->evaluate(parameters_[thread_num],
   vel_parameters_[thread_num], contact_parameters_[thread_num], costs_[thread_num]);
   e[thread_num](i) = old_val - eps;
   getVariableVector(e[thread_num], thread_num);

   const double delta_minus = derivatives_evaluation_manager_[thread_num]->evaluate(parameters_[thread_num],
   vel_parameters_[thread_num], contact_parameters_[thread_num], costs_[thread_num]);

   double new_der = (delta_plus - delta_minus) / (2 * eps);

   e[thread_num](i) = old_val;

   if (der(i) != new_der)
   printf("[%d] : %f %f = %f - %f / (2 * eps)\n", i, der(i), new_der, delta_plus, delta_minus);
   }
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
