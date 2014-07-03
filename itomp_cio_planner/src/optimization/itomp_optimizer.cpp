#include <ros/ros.h>
#include <itomp_cio_planner/optimization/itomp_optimizer.h>
#include <itomp_cio_planner/contact/ground_manager.h>
#include <itomp_cio_planner/visualization/visualization_manager.h>
#include <itomp_cio_planner/util/planning_parameters.h>
#include <itomp_cio_planner/optimization/improvement_manager_nlp.h>
#include <itomp_cio_planner/optimization/improvement_manager_chomp.h>

using namespace std;

namespace itomp_cio_planner
{

ItompOptimizer::ItompOptimizer(int trajectory_index, ItompCIOTrajectory* trajectory, ItompRobotModel *robot_model,
    const ItompPlanningGroup *planning_group, double planning_start_time, double trajectory_start_time) :
    is_succeed_(false), terminated_(false), trajectory_index_(trajectory_index), planning_start_time_(
        planning_start_time), iteration_(-1), feasible_iteration_(0), last_improvement_iteration_(-1), full_trajectory_(
        trajectory), group_trajectory_(*full_trajectory_, planning_group, DIFF_RULE_LENGTH), evaluation_manager_(
        &iteration_), best_group_trajectory_(group_trajectory_.getTrajectory()), best_group_contact_trajectory_(
        group_trajectory_.getContactTrajectory())
{
  initialize(robot_model, planning_group, trajectory_start_time);
}

void ItompOptimizer::initialize(ItompRobotModel *robot_model, const ItompPlanningGroup *planning_group,
    double trajectory_start_time)
{
  evaluation_manager_.initialize(full_trajectory_, &group_trajectory_, robot_model, planning_group,
      planning_start_time_, trajectory_start_time);

  //improvement_manager_.reset(new ImprovementManagerNLP());
  improvement_manager_.reset(new ImprovementManagerChomp());
  improvement_manager_->initialize(&evaluation_manager_);

  VisualizationManager::getInstance()->clearAnimations();
}

ItompOptimizer::~ItompOptimizer()
{
}

bool ItompOptimizer::optimize()
{
  ros::WallTime start_time = ros::WallTime::now();
  terminated_ = false;
  iteration_ = -1;
  best_group_trajectory_cost_ = numeric_limits<double>::max();

  improvement_manager_->updatePlanningParameters();

  VisualizationManager::getInstance()->render();

  evaluation_manager_.handleJointLimits();
  evaluation_manager_.updateFullTrajectory();
  evaluation_manager_.evaluate();

  updateBestTrajectory(evaluation_manager_.getTrajectoryCost(true));
  ++iteration_;

  while (iteration_ < 1)//PlanningParameters::getInstance()->getMaxIterations())
  {
    improvement_manager_->runSingleIteration(iteration_);
    is_succeed_ = evaluation_manager_.isLastTrajectoryFeasible();
    ROS_INFO("We think trajectory %d is feasible: %s", trajectory_index_, (is_succeed_ ? "True" : "False"));
    bool is_best = updateBestTrajectory(evaluation_manager_.getTrajectoryCost(true));
    if (best_group_trajectory_cost_ < 0.01)
    {
      break;
    }
    if (!is_best)
    {
      group_trajectory_.getTrajectory() = best_group_trajectory_;
      group_trajectory_.getContactTrajectory() = best_group_contact_trajectory_;
    }

    ++iteration_;
  }
  evaluation_manager_.postprocess_ik();

  group_trajectory_.getTrajectory() = best_group_trajectory_;
  group_trajectory_.getContactTrajectory() = best_group_contact_trajectory_;
  evaluation_manager_.updateFullTrajectory();
  evaluation_manager_.evaluate();

  evaluation_manager_.render(trajectory_index_);

  ROS_INFO("Terminated after %d iterations, using path from iteration %d", iteration_, last_improvement_iteration_);
  ROS_INFO("Optimization core finished in %f sec", (ros::WallTime::now() - start_time).toSec());

  evaluation_manager_.getTrajectoryCost(true);

  return is_succeed_;
}

bool ItompOptimizer::updateBestTrajectory(double cost)
{
  if (cost < best_group_trajectory_cost_)
  {
    best_group_trajectory_ = group_trajectory_.getTrajectory();
    best_group_contact_trajectory_ = group_trajectory_.getContactTrajectory();
    best_group_trajectory_cost_ = cost;
    last_improvement_iteration_ = iteration_;
    return true;
  }
  return false;
}

}
