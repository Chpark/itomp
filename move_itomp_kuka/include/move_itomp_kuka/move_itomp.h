/*
 * moe_itomp.h
 *
 *  Created on: Sep 23, 2014
 *      Author: chonhyon
 */

#ifndef MOVE_ITOMP_H_
#define MOVE_ITOMP_H_

#include <moveit_msgs/RobotTrajectory.h>

namespace move_itomp
{

class MoveItomp
{
public:
	MoveItomp(const ros::NodeHandle& node_handle);
	~MoveItomp();

	void run(const std::string& group_name);

protected:
	void loadStaticScene();
	bool isStateCollide(const robot_state::RobotState& state);
	bool isStateSingular(robot_state::RobotState& state);

	void plan(planning_interface::MotionPlanRequest& req,
			planning_interface::MotionPlanResponse& res,
			robot_state::RobotState& start_state,
			robot_state::RobotState& goal_state);
	void plan(planning_interface::MotionPlanRequest& req,
			planning_interface::MotionPlanResponse& res,
			robot_state::RobotState& start_state,
			geometry_msgs::PoseStamped& goal_pose,
			const std::string& endeffector_link);

	void displayState(robot_state::RobotState& state);
	void displayStates(robot_state::RobotState& start_state,
			robot_state::RobotState& goal_state);

	void computeIKState(robot_state::RobotState& ik_state,
			const Eigen::Affine3d& end_effector_state);

	void printTrajectory(const moveit_msgs::RobotTrajectory &traj);

	ros::NodeHandle node_handle_;
	robot_model::RobotModelPtr robot_model_;
	planning_scene::PlanningScenePtr planning_scene_;
	planning_interface::PlannerManagerPtr ompl_planner_instance_;
	planning_interface::PlannerManagerPtr itomp_planner_instance_;

	ros::Publisher planning_scene_diff_publisher_;
	ros::Publisher display_publisher_;
	ros::Publisher vis_marker_array_publisher_;

	std::string group_name_;
};

}

#endif /* MOVE_ITOMP_H_ */
