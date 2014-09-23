/*
 * moe_itomp.h
 *
 *  Created on: Sep 23, 2014
 *      Author: chonhyon
 */

#ifndef MOVE_ITOMP_H_
#define MOVE_ITOMP_H_

namespace move_itomp
{

class MoveItomp
{
public:
	MoveItomp(const ros::NodeHandle& node_handle);
	~MoveItomp();

	void init();
	void run();

protected:
	bool isStateSingular(planning_scene::PlanningScenePtr& planning_scene,
			const std::string& group_name, robot_state::RobotState& state);
	void plan(planning_interface::PlannerManagerPtr& planner_instance,
			planning_scene::PlanningScenePtr planning_scene,
			planning_interface::MotionPlanRequest& req,
			planning_interface::MotionPlanResponse& res,
			const std::string& group_name, robot_state::RobotState& start_state,
			robot_state::RobotState& goal_state);
	void plan(planning_interface::PlannerManagerPtr& planner_instance,
			planning_scene::PlanningScenePtr planning_scene,
			planning_interface::MotionPlanRequest& req,
			planning_interface::MotionPlanResponse& res,
			const std::string& group_name, robot_state::RobotState& start_state,
			geometry_msgs::PoseStamped& goal_pose,
			const std::string& endeffector_link);
	void loadStaticScene(ros::NodeHandle& node_handle,
			planning_scene::PlanningScenePtr& planning_scene,
			robot_model::RobotModelPtr& robot_model,
			visualization_msgs::MarkerArray& ma,
			moveit_msgs::PlanningScene& planning_scene_msg);
	void displayState(ros::NodeHandle& node_handle,
			robot_model::RobotModelPtr& robot_model, robot_state::RobotState& state);
	void displayStates(ros::NodeHandle& node_handle,
			robot_model::RobotModelPtr& robot_model,
			robot_state::RobotState& start_state,
			robot_state::RobotState& goal_state);
	bool isCollide(robot_model::RobotModelPtr& robot_model,
			robot_state::RobotState& ik_state,
			planning_scene::PlanningScenePtr& planning_scene,
			ros::Publisher& vis_array_publisher);
	void computeIKState(ros::NodeHandle& node_handle,
			robot_model::RobotModelPtr& robot_model,
			robot_state::RobotState& ik_state,
			planning_scene::PlanningScenePtr& planning_scene,
			const std::string& group_name, Eigen::Affine3d& end_effector_state,
			ros::Publisher& vis_array_publisher);
	void computeIKState(ros::NodeHandle& node_handle,
			robot_model::RobotModelPtr& robot_model,
			robot_state::RobotState& ik_state,
			planning_scene::PlanningScenePtr& planning_scene,
			const std::string& group_name, ros::Publisher& vis_array_publisher,
			double x, double y, double z, double qx, double qy, double qz,
			double qw);
	void transformConstraint(double& x, double& y, double& z, double& qx,
			double& qy, double& qz, double& qw, const Eigen::Affine3d& transform);

	ros::NodeHandle node_handle_;
	robot_model::RobotModelPtr robot_model_;
	planning_scene::PlanningScenePtr planning_scene_;
	planning_interface::PlannerManagerPtr planner_instance_;

	ros::Publisher planning_scene_diff_publisher_;
	ros::Publisher display_publisher_;
	ros::Publisher vis_marker_array_publisher_;
};

}



#endif /* MOVE_ITOMP_H_ */
