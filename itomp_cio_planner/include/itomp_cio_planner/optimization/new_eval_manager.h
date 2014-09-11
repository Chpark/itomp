#ifndef NEW_EVALUATION_MANAGER_H_
#define NEW_EVALUATION_MANAGER_H_

#include <itomp_cio_planner/common.h>
#include <itomp_cio_planner/model/itomp_robot_model.h>
#include <itomp_cio_planner/trajectory/full_trajectory.h>
#include <itomp_cio_planner/trajectory/parameter_trajectory.h>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <ros/publisher.h>
#include <moveit/planning_scene/planning_scene.h>

namespace itomp_cio_planner
{
ITOMP_FORWARD_DECL(NewEvalManager)
class NewEvalManager
{
public:
	NewEvalManager();
	virtual ~NewEvalManager();

	void initialize(const FullTrajectoryPtr& full_trajectory,
			const ItompRobotModelConstPtr& robot_model,
			const ItompPlanningGroupConstPtr& planning_group,
			double planning_start_time, double trajectory_start_time,
			const moveit_msgs::Constraints& path_constraints);

	const FullTrajectoryConstPtr& getFullTrajectory() const;
	const ParameterTrajectoryConstPtr& getParameterTrajectory() const;

	void getParameters(std::vector<Eigen::MatrixXd>& parameters) const;
	void setParameters(const std::vector<Eigen::MatrixXd>& parameters,
			bool joint_limit_check = true);

	double evaluate();
	void evaluateParameterPoint(int point, int element,
			Eigen::MatrixXd& cost_matrix, int& full_point_begin, int& full_point_end);

	bool isLastTrajectoryFeasible() const;
	double getTrajectoryCost();
	void printTrajectoryCost(int iteration);

	void render();

	NewEvalManager* createClone() const;

private:
	void performForwardKinematics(int point_begin, int point_end);
	void performInverseDynamics(int point_begin, int point_end);

	void setParameterModified(bool needs_joint_limit_check = true);

	bool evaluatePointRange(int point_begin, int point_end, Eigen::MatrixXd& cost_matrix);

	FullTrajectoryPtr full_trajectory_;
	ParameterTrajectoryPtr parameter_trajectory_;
	mutable FullTrajectoryConstPtr full_trajectory_const_;
	mutable ParameterTrajectoryConstPtr parameter_trajectory_const_;

	ItompRobotModelConstPtr robot_model_;
	ItompPlanningGroupConstPtr planning_group_;

	double planning_start_time_;
	double trajectory_start_time_;

	bool last_trajectory_feasible_;

	std::vector<RigidBodyDynamics::Model> rbdl_models_;

	Eigen::MatrixXd evaluation_cost_matrix_;

	bool parameter_modified_;
	bool check_joint_limits_;

	double best_cost_;
};
ITOMP_DEFINE_SHARED_POINTERS(NewEvalManager);

////////////////////////////////////////////////////////////////////////////////

inline const FullTrajectoryConstPtr& NewEvalManager::getFullTrajectory() const
{
	full_trajectory_const_ = full_trajectory_;
	return full_trajectory_const_;
}

inline const ParameterTrajectoryConstPtr& NewEvalManager::getParameterTrajectory() const
{
	parameter_trajectory_const_ = parameter_trajectory_;
	return parameter_trajectory_const_;
}

inline bool NewEvalManager::isLastTrajectoryFeasible() const
{
	return last_trajectory_feasible_;
}

inline void NewEvalManager::setParameterModified(bool needs_joint_limit_check)
{
	parameter_modified_ = true;
	check_joint_limits_ = needs_joint_limit_check;
}

inline double NewEvalManager::getTrajectoryCost()
{
	return evaluation_cost_matrix_.sum();
}

}

#endif
