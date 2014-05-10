#include <itomp_cio_planner/optimization/improvement_manager_nlp.h>
#include <itomp_cio_planner/util/multivariate_gaussian.h>
#include "dlib/optimization.h"
#include <omp.h>

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

  return true;
}

void ImprovementManagerNLP::runSingleIteration(int iteration)
{
  bool add_noise = (iteration != 0);
  optimize(add_noise, iteration);
}

////////////////////////////////////////////////////////////////////////////////

typedef dlib::matrix<double, 0, 1> column_vector;
Eigen::MatrixXd parameters_;
Eigen::MatrixXd vel_parameters_;
Eigen::MatrixXd contact_parameters_;
Eigen::VectorXd costs_;
EvaluationManager* evaluation_manager_;

int num_dimensions_;
int num_contact_dimensions_;
int num_free_points_;
int num_points_;

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

void ImprovementManagerNLP::optimize(bool add_noise, int iteration)
{
  int num_contact_vars_free = num_contact_dimensions_ * num_contact_phases_;
  int num_pos_variables = num_dimensions_ * (num_contact_phases_ - 1);
  int num_vel_variables = num_dimensions_ * (num_contact_phases_ - 1);
  int num_variables = num_contact_vars_free + num_pos_variables + num_vel_variables;

  column_vector variables(num_variables);

  int writeIndex = 0;
  // copy from group_trajectory_:
  const Eigen::MatrixXd& freeTrajectoryBlock = evaluation_manager_->getGroupTrajectory()->getFreePoints();
  const Eigen::MatrixXd& freeVelTrajectoryBlock = evaluation_manager_->getGroupTrajectory()->getFreeVelPoints();
  const Eigen::MatrixXd& contactTrajectoryBlock = evaluation_manager_->getGroupTrajectory()->getContactTrajectory();
  // contact variable for phase 0
  for (int d = 0; d < num_contact_dimensions_; ++d)
  {
    variables(writeIndex + d, 0) = contactTrajectoryBlock(0, d);
  }
  writeIndex += num_contact_dimensions_;

  for (int i = 1; i < num_contact_phases_; ++i)
  {
    for (int d = 0; d < num_dimensions_; ++d)
    {
      variables(writeIndex + d, 0) = freeTrajectoryBlock(i, d);
    }
    writeIndex += num_dimensions_;
    for (int d = 0; d < num_dimensions_; ++d)
    {
      variables(writeIndex + d, 0) = freeVelTrajectoryBlock(i, d);
    }
    writeIndex += num_dimensions_;
    for (int d = 0; d < num_contact_dimensions_; ++d)
    {
      variables(writeIndex + d, 0) = contactTrajectoryBlock(i, d);
    }
    writeIndex += num_contact_dimensions_;
  }

  if (add_noise)
  {
    MultivariateGaussian noise_generator(VectorXd::Zero(num_variables),
        MatrixXd::Identity(num_variables, num_variables));
    VectorXd noise = VectorXd::Zero(num_variables);
    noise_generator.sample(noise);
    for (int i = 0; i < num_variables; ++i)
    {
      variables(i) += 0.01 * noise(i);
    }
  }
  /*
   dlib::find_min(dlib::bfgs_search_strategy(), dlib::objective_delta_stop_strategy(1e-7).be_verbose(),
   evaluation_functor(this, num_joints_, num_contacts_, num_contact_phases - 1, num_points_),
   derivative_functor(this, num_joints_, num_contacts_, num_contact_phases - 1, num_points_), variables, -1);
   */
  dlib::find_min_using_approximate_derivatives(dlib::bfgs_search_strategy(),
      dlib::objective_delta_stop_strategy(1e-7).be_verbose(),
      evaluation_functor(evaluation_manager_, num_dimensions_, num_contact_dimensions_, num_contact_phases_ - 1, num_points_), variables, -1);

}


}
