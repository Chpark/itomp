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

ItompOptimizer::ItompOptimizer(int trajectory_index, const ItompCIOTrajectoryPtr& trajectory, const ItompRobotModelConstPtr& robot_model,
    const ItompPlanningGroupConstPtr& planning_group, double planning_start_time, double trajectory_start_time,
    const moveit_msgs::Constraints& path_constraints) :
    trajectory_index_(trajectory_index), planning_start_time_(planning_start_time), iteration_(-1), full_trajectory_(
        trajectory), is_best_group_trajectory_feasible_(false), best_group_trajectory_iteration_(
        -1)
{
  initialize(robot_model, planning_group, trajectory_start_time, path_constraints);
}

void ItompOptimizer::initialize(const ItompRobotModelConstPtr& robot_model, const ItompPlanningGroupConstPtr& planning_group,
    double trajectory_start_time, const moveit_msgs::Constraints& path_constraints)
{
  evaluation_manager_ = boost::make_shared<EvaluationManager>(&iteration_);

  improvement_manager_ = boost::make_shared<ImprovementManagerNLP>();
  group_trajectory_ = boost::make_shared<ItompCIOTrajectory>(full_trajectory_, planning_group, 0);
  //improvement_manager_ = boost::make_shared<ImprovementManagerChomp>();
  //group_trajectory_ = boost::make_shared<ItompCIOTrajectory>(full_trajectory_, planning_group, DIFF_RULE_LENGTH);

  best_group_trajectory_ = group_trajectory_->getTrajectory(ItompCIOTrajectory::TRAJECTORY_POSITION);

  evaluation_manager_->initialize(*full_trajectory_, *group_trajectory_, *robot_model, *planning_group,
      planning_start_time_, trajectory_start_time, path_constraints);

  improvement_manager_->initialize(evaluation_manager_);

  VisualizationManager::getInstance()->clearAnimations();
}

ItompOptimizer::~ItompOptimizer()
{
}

bool ItompOptimizer::optimize()
{
  ros::WallTime start_time = ros::WallTime::now();
  iteration_ = -1;
  best_group_trajectory_cost_ = numeric_limits<double>::max();

  improvement_manager_->updatePlanningParameters();

  VisualizationManager::getInstance()->render();

  evaluation_manager_->handleJointLimits();
  evaluation_manager_->updateFullTrajectory();
  evaluation_manager_->evaluate();
  updateBestTrajectory(evaluation_manager_->getTrajectoryCost(true), evaluation_manager_->isLastTrajectoryFeasible());
  ++iteration_;

  int iteration_after_feasible_solution = 0;
  int num_max_iterations = PlanningParameters::getInstance()->getMaxIterations();

  if (!evaluation_manager_->isLastTrajectoryFeasible())
  {
    while (iteration_ < num_max_iterations)
    {
      if (is_best_group_trajectory_feasible_)
        ++iteration_after_feasible_solution;

      improvement_manager_->runSingleIteration(iteration_);
      bool is_feasible = evaluation_manager_->isLastTrajectoryFeasible();
      double cost = evaluation_manager_->getTrajectoryCost(true);
      bool is_cost_reduced = (cost < best_group_trajectory_cost_);
      bool is_updated = updateBestTrajectory(cost, is_feasible);

      // is_cost_reduced : allow moving to non-feasible low-cost solutions
      // is_updated : only moves in feasible solutions
      if (!is_updated)
        group_trajectory_->getTrajectory() = best_group_trajectory_;

      ++iteration_;

      if (iteration_after_feasible_solution > PlanningParameters::getInstance()->getMaxIterationsAfterCollisionFree())
        break;
    }
  }
  group_trajectory_->getTrajectory() = best_group_trajectory_;
  evaluation_manager_->updateFullTrajectory();
  evaluation_manager_->evaluate();
  evaluation_manager_->getTrajectoryCost(true);

  evaluation_manager_->render(trajectory_index_);

  double elpsed_time = (ros::WallTime::now() - start_time).toSec();

  ROS_INFO(
      "Terminated after %d iterations, using path from iteration %d", iteration_, best_group_trajectory_iteration_);
  ROS_INFO(
      "We think trajectory %d is feasible: %s", trajectory_index_, (is_best_group_trajectory_feasible_ ? "True" : "False"));
  ROS_INFO("Optimization core finished in %f sec", elpsed_time);

  planning_info_.time = elpsed_time;
  planning_info_.iterations = iteration_ + 1;
  planning_info_.cost = best_group_trajectory_cost_;
  planning_info_.success = is_best_group_trajectory_feasible_ ? 1 : 0;

  return is_best_group_trajectory_feasible_;
}

bool ItompOptimizer::updateBestTrajectory(double cost, bool is_feasible)
{
  bool update = false;

  if (!is_best_group_trajectory_feasible_)
  {
    if (is_feasible || cost < best_group_trajectory_cost_)
      update = true;
    is_best_group_trajectory_feasible_ = is_feasible;
  }
  else
  {
    if (is_feasible && cost < best_group_trajectory_cost_)
      update = true;
  }

  if (update)
  {
    best_group_trajectory_ = group_trajectory_->getTrajectory();
    best_group_trajectory_cost_ = cost;
    best_group_trajectory_iteration_ = iteration_;
    return true;
  }
  return false;
}

}
