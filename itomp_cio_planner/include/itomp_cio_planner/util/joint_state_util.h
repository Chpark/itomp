#ifndef JOINT_STATE_UTIL_H_
#define JOINT_STATE_UTIL_H_

#include <itomp_cio_planner/common.h>
#include <itomp_cio_planner/model/itomp_robot_model.h>

namespace itomp_cio_planner
{
sensor_msgs::JointState jointConstraintsToJointState(const std::vector<moveit_msgs::Constraints> &constraints)
{
  sensor_msgs::JointState state;
  state.name.clear();
  state.position.clear();
  for (unsigned int i = 0; i < constraints.size(); i++)
  {
    const std::vector<moveit_msgs::JointConstraint> &joint_constraints = constraints[i].joint_constraints;

    for (unsigned int j = 0; j < joint_constraints.size(); j++)
    {
      state.name.push_back(joint_constraints[j].joint_name);
      state.position.push_back(joint_constraints[j].position);
    }
  }
  return state;
}

sensor_msgs::JointState getGoalStateFromGoalConstraints(const ItompRobotModel& itomp_robot_model,
    const planning_interface::MotionPlanRequest &req)
{
  sensor_msgs::JointState goal_state;
  sensor_msgs::JointState goal_constraints_joint_state = jointConstraintsToJointState(req.goal_constraints);
  for (unsigned int i = 0; i < goal_constraints_joint_state.name.size(); ++i)
  {
    std::string name = goal_constraints_joint_state.name[i];
    int kdl_number = itomp_robot_model.urdfNameToKdlNumber(name);
    if (kdl_number >= 0)
    {
      goal_state.name[kdl_number] = name;
      goal_state.position[kdl_number] = goal_constraints_joint_state.position[i];
    }
  }
  return goal_state;
}

void jointStateToArray(const ItompRobotModel& itomp_robot_model, const sensor_msgs::JointState &joint_state,
    Eigen::MatrixXd::RowXpr joint_pos_array, Eigen::MatrixXd::RowXpr joint_vel_array,
    Eigen::MatrixXd::RowXpr joint_acc_array)
{
  ROS_INFO("Initial Joint States");
  for (unsigned int i = 0; i < joint_state.name.size(); i++)
  {
    std::string name = joint_state.name[i];
    int kdl_number = itomp_robot_model.urdfNameToKdlNumber(name);
    if (kdl_number >= 0)
    {
      joint_pos_array(kdl_number) = joint_state.position[i];
      joint_vel_array(kdl_number) = joint_state.velocity[i];
      joint_acc_array(kdl_number) = joint_state.effort[i];
      ROS_INFO("%s : %f %f %f", name.c_str(), joint_state.position[i], joint_state.velocity[i], joint_state.effort[i]);
    }
  }
}

}

#endif
