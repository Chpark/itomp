#ifndef ITOMP_PLANNER_NODE_H_
#define ITOMP_PLANNER_NODE_H_

#include <itomp_cio_planner/common.h>
#include <itomp_cio_planner/planner/planning_info_manager.h>
#include <itomp_cio_planner/model/itomp_robot_model.h>
#include <itomp_cio_planner/trajectory/itomp_cio_trajectory.h>
#include <itomp_cio_planner/optimization/itomp_optimizer.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>

namespace itomp_cio_planner
{

class ItompPlannerNode
{
public:
  ItompPlannerNode(const robot_model::RobotModelConstPtr& model);
  virtual ~ItompPlannerNode()
  {
  }

  bool init();

  bool planTrajectory(const planning_scene::PlanningSceneConstPtr& planning_scene,
      const planning_interface::MotionPlanRequest &req, planning_interface::MotionPlanResponse &res);

private:
  bool validateRequest(const planning_interface::MotionPlanRequest &req);
  std::vector<std::string> getPlanningGroups(const std::string& group_name) const;
  void initTrajectory(const sensor_msgs::JointState &joint_state);
  void fillGroupJointTrajectory(const std::string& group_name, const sensor_msgs::JointState& joint_goal_statetate,
      const moveit_msgs::Constraints& path_constraints);
  void
  fillInResult(const robot_state::RobotStatePtr& robot_state, const std::vector<std::string>& planning_groups,
      planning_interface::MotionPlanResponse &res);

  robot_model::RobotModelConstPtr robot_model_;
  ItompRobotModel itomp_robot_model_;

  ItompCIOTrajectoryPtr trajectory_;
  ItompOptimizerPtr optimizer_;
  PlanningInfoManager planning_info_manager_;
};

}

#endif
