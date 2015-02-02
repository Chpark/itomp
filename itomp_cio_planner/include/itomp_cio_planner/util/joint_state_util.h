#ifndef JOINT_STATE_UTIL_H_
#define JOINT_STATE_UTIL_H_

#include <itomp_cio_planner/common.h>
#include <itomp_cio_planner/model/itomp_robot_model.h>
#include <moveit/planning_interface/planning_request.h>

namespace itomp_cio_planner
{
sensor_msgs::JointState jointConstraintsToJointState(
	const std::vector<moveit_msgs::Constraints> &constraints);

sensor_msgs::JointState getGoalStateFromGoalConstraints(
	const ItompRobotModelConstPtr& itomp_robot_model,
	const planning_interface::MotionPlanRequest &req);

void jointStateToArray(const ItompRobotModelConstPtr& itomp_robot_model,
					   const sensor_msgs::JointState &joint_state,
					   Eigen::MatrixXd::RowXpr joint_pos_array,
					   Eigen::MatrixXd::RowXpr joint_vel_array,
					   Eigen::MatrixXd::RowXpr joint_acc_array);
}

#endif
