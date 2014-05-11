#ifndef ITOMP_PLANNER_NODE_H_
#define ITOMP_PLANNER_NODE_H_

#include <itomp_cio_planner/common.h>
#include <itomp_cio_planner/planner/planning_info.h>
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
	virtual ~ItompPlannerNode();

	bool init();

	int run();

	bool planKinematicPath(const planning_interface::MotionPlanRequest &req,
			planning_interface::MotionPlanResponse &res);

private:
	bool preprocessRequest(const planning_interface::MotionPlanRequest &req);
	void getGoalState(const planning_interface::MotionPlanRequest &req, sensor_msgs::JointState& goalState);
	void initTrajectory(const sensor_msgs::JointState &joint_state);
	void getPlanningGroups(std::vector<std::string>& plannningGroups, const std::string& groupName);
	void fillGroupJointTrajectory(const std::string& groupName, const sensor_msgs::JointState& jointGoalState);
	void multiTrajectoryOptimization(const std::string& groupName, const sensor_msgs::JointState& jointGoalState);
	void updateTrajectoryToBestResult(const std::string& groupName);

	void
	fillInResult(const std::vector<std::string>& planningGroups, planning_interface::MotionPlanResponse &res);

	ItompRobotModel robot_model_;

	ItompCIOTrajectory* trajectory_;
	std::vector<ItompCIOTrajectory*> threadTrajectories_;
	std::vector<ItompOptimizer*> optimizers_;
	double trajectory_start_time_;

	void printTrajectory(ItompCIOTrajectory* trajectory);
	double last_planning_time_;
	int last_min_cost_trajectory_;

	std::vector<std::vector<PlanningInfo> > planning_info_;
	void resetPlanningInfo(int trials, int component);
	void writePlanningInfo(int trials, int component);
	void printPlanningInfoSummary();

	double planning_start_time_;

	Eigen::MatrixXd start_point_velocities_;
	Eigen::MatrixXd start_point_accelerations_;

	robot_state::RobotStatePtr complete_initial_robot_state_;

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
};

}

#endif
