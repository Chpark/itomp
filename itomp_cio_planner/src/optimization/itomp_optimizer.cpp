#include <ros/ros.h>
#include <itomp_cio_planner/optimization/itomp_optimizer.h>
#include <itomp_cio_planner/contact/ground_manager.h>
#include <itomp_cio_planner/visualization/visualization_manager.h>
#include <itomp_cio_planner/util/planning_parameters.h>

using namespace std;

namespace itomp_cio_planner
{

ItompOptimizer::ItompOptimizer(int trajectory_index, ItompCIOTrajectory *trajectory, ItompRobotModel *robot_model,
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
      planning_start_time_, trajectory_start_time, &costAccumulator_);

  improvement_manager_.initialize(&evaluation_manager_);

  VisualizationManager::getInstance()->clearAnimations();

  costAccumulator_.addCost(TrajectoryCost::CreateTrajectoryCost(TrajectoryCost::COST_SMOOTHNESS));
  costAccumulator_.addCost(TrajectoryCost::CreateTrajectoryCost(TrajectoryCost::COST_COLLISION));
  costAccumulator_.addCost(TrajectoryCost::CreateTrajectoryCost(TrajectoryCost::COST_VALIDITY));
  costAccumulator_.addCost(TrajectoryCost::CreateTrajectoryCost(TrajectoryCost::COST_CONTACT_INVARIANT));
  costAccumulator_.addCost(TrajectoryCost::CreateTrajectoryCost(TrajectoryCost::COST_PHYSICS_VIOLATION));
  costAccumulator_.addCost(TrajectoryCost::CreateTrajectoryCost(TrajectoryCost::COST_GOAL_POSE));
  costAccumulator_.addCost(TrajectoryCost::CreateTrajectoryCost(TrajectoryCost::COST_COM));
  costAccumulator_.init(&evaluation_manager_);
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

  improvement_manager_.updatePlanningParameters();

  VisualizationManager::getInstance()->render();

  evaluation_manager_.handleJointLimits();
  evaluation_manager_.updateFullTrajectory();
  evaluation_manager_.performForwardKinematics();
  evaluation_manager_.computeTrajectoryValidity();

  costAccumulator_.compute(&evaluation_manager_);
  costAccumulator_.print(iteration_++);
  updateBestTrajectory(costAccumulator_.getTrajectoryCost());

  bool first = true;
  const int max_interations = 1;
  while (iteration_ < max_interations)
  {
    evaluation_manager_.optimize_nlp(!first);
    costAccumulator_.print(iteration_++);
    is_succeed_ = evaluation_manager_.isLastTrajectoryFeasible();
    ROS_INFO("We think trajectory %d is feasible: %s", trajectory_index_, (is_succeed_ ? "True" : "False"));
    if (!updateBestTrajectory(costAccumulator_.getTrajectoryCost()))
    {
      if (best_group_trajectory_cost_ < 0.1)
        break;
      else
      {
        group_trajectory_.getTrajectory() = best_group_trajectory_;
        group_trajectory_.getContactTrajectory() = best_group_contact_trajectory_;
      }
    }
    first = false;
  }
  evaluation_manager_.postprocess_ik();

  //updateBestTrajectory(costAccumulator_.getTrajectoryCost());

  group_trajectory_.getTrajectory() = best_group_trajectory_;
  group_trajectory_.getContactTrajectory() = best_group_contact_trajectory_;
  evaluation_manager_.updateFullTrajectory();
  evaluation_manager_.performForwardKinematics();
  evaluation_manager_.computeTrajectoryValidity();

  evaluation_manager_.render(trajectory_index_);

  ROS_INFO("Terminated after %d iterations, using path from iteration %d", iteration_, last_improvement_iteration_);
  ROS_INFO("Optimization core finished in %f sec", (ros::WallTime::now() - start_time).toSec());

  costAccumulator_.compute(&evaluation_manager_);
  costAccumulator_.print(iteration_);

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
