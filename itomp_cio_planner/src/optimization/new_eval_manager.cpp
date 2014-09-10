#include <ros/ros.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/PlanningScene.h>
#include <itomp_cio_planner/optimization/new_eval_manager.h>
#include <itomp_cio_planner/model/itomp_planning_group.h>
#include <itomp_cio_planner/contact/ground_manager.h>
#include <itomp_cio_planner/visualization/visualization_manager.h>
#include <itomp_cio_planner/contact/contact_force_solver.h>
#include <itomp_cio_planner/util/min_jerk_trajectory.h>
#include <itomp_cio_planner/util/planning_parameters.h>
#include <itomp_cio_planner/util/vector_util.h>
#include <itomp_cio_planner/util/multivariate_gaussian.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std;
using namespace Eigen;

namespace itomp_cio_planner
{

NewEvalManager::NewEvalManager()
: last_trajectory_feasible_(false)
{

}

NewEvalManager::~NewEvalManager()
{

}

void NewEvalManager::initialize(const ItompCIOTrajectoryPtr& full_trajectory,
    const ItompRobotModelConstPtr& robot_model, const ItompPlanningGroupConstPtr& planning_group,
    double planning_start_time, double trajectory_start_time, const moveit_msgs::Constraints& path_constraints)
{
  full_trajectory_ = full_trajectory;
  group_trajectory_ = boost::make_shared<ItompCIOTrajectory>(full_trajectory_, planning_group, 0);
  //group_trajectory_ = boost::make_shared<ItompCIOTrajectory>(full_trajectory_, planning_group, DIFF_RULE_LENGTH);

  robot_model_ = robot_model;
  planning_group_ = planning_group;

  planning_start_time_ = planning_start_time;
  trajectory_start_time_ = trajectory_start_time;

  // TODO : rbdl models

  // TODO : cost manager

  // TODO : path_constraints
}

double NewEvalManager::evaluate()
{
  performForwardKinematics();
  performInverseDynamics();

  // TODO: compute different costs

  last_trajectory_feasible_ = cost_acccumulator_.isFeasible();
  return cost_acccumulator_.getTrajectoryCost();
}

double NewEvalManager::evaluateRange(int start, int end)
{
  for (int i = start; i < end; ++i)
  {

  }
}

void NewEvalManager::setTrajectory(const Eigen::MatrixXd& parameters)
{
  // TODO

  // respect joint limits:
  handleJointLimits();

  // copy to full traj:
  updateFullTrajectory();
}

void NewEvalManager::render()
{
}

void NewEvalManager::handleJointLimits()
{
  for (int joint = 0; joint < num_joints_; joint++)
  {
    if (!planning_group_->group_joints_[joint].has_joint_limits_)
      continue;

    double joint_max = planning_group_->group_joints_[joint].joint_limit_max_;
    double joint_min = planning_group_->group_joints_[joint].joint_limit_min_;

    int count = 0;

    for (int i = 1; i < num_points_ - 2; i++)
    {
      if ((*getGroupTrajectory())(i, joint) > joint_max)
      {
        (*getGroupTrajectory())(i, joint) = joint_max;
      }
      else if ((*getGroupTrajectory())(i, joint) < joint_min)
      {
        (*getGroupTrajectory())(i, joint) = joint_min;
      }
    }
  }
}

void NewEvalManager::updateFullTrajectory()
{
  full_trajectory_->copyFromGroupTrajectory(group_trajectory_);
}

void NewEvalManager::performForwardKinematics()
{
}

void NewEvalManager::performInverseDynamics()
{
}

double NewEvalManager::getTrajectoryCost(bool verbose)
{
  if (verbose)
    data_->costAccumulator_.print(*iteration_);
  return data_->costAccumulator_.getTrajectoryCost();
}


}

