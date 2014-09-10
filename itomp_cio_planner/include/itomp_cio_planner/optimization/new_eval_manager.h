#ifndef NEW_EVALUATION_MANAGER_H_
#define NEW_EVALUATION_MANAGER_H_

#include <itomp_cio_planner/common.h>
#include <itomp_cio_planner/optimization/evaluation_data.h>
#include <itomp_cio_planner/model/itomp_robot_model.h>
#include <itomp_cio_planner/trajectory/itomp_cio_trajectory.h>
#include <itomp_cio_planner/cost/smoothness_cost.h>
#include <itomp_cio_planner/cost/trajectory_cost_accumulator.h>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <ros/publisher.h>
#include <moveit/planning_scene/planning_scene.h>

namespace itomp_cio_planner
{
class ItompPlanningGroup;
class NewEvalManager
{
public:
  NewEvalManager();
  virtual ~NewEvalManager();

  void initialize(const ItompCIOTrajectoryPtr& full_trajectory, const ItompRobotModelConstPtr& robot_model,
      const ItompPlanningGroupConstPtr& planning_group, double planning_start_time, double trajectory_start_time,
      const moveit_msgs::Constraints& path_constraints);

  void setTrajectory(const Eigen::MatrixXd& parameters);
  const ItompCIOTrajectoryConstPtr& getFullTrajectory();

  double evaluate();
  double evaluateRange(int start, int end);

  bool isLastTrajectoryFeasible() const;
  double getTrajectoryCost(bool verbose = false);

  void render();

private:
  void handleJointLimits();
  void updateFullTrajectory();

  void performForwardKinematics();
  void performInverseDynamics();

  ItompCIOTrajectoryPtr full_trajectory_;
  ItompCIOTrajectoryPtr group_trajectory_;

  ItompRobotModelConstPtr robot_model_;
  ItompPlanningGroupConstPtr planning_group_;

  double planning_start_time_;
  double trajectory_start_time_;

  bool last_trajectory_feasible_;

  std::vector<RigidBodyDynamics::Model> rbdl_models_;

  TrajectoryCostAccumulator cost_acccumulator_;
  Eigen::MatrixXd evaluation_cost_matrix_;
  Eigen::MatrixXd derivative_cost_matrix_;
};
ITOMP_DEFINE_SHARED_POINTERS(NewEvalManager);

////////////////////////////////////////////////////////////////////////////////

inline bool NewEvalManager::isLastTrajectoryFeasible() const
{
  return last_trajectory_feasible_;
}

}

#endif
