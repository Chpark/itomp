#ifndef FULL_TRAJECTORY_H_
#define FULL_TRAJECTORY_H_

#include <itomp_cio_planner/common.h>
#include <itomp_cio_planner/trajectory/trajectory.h>
#include <itomp_cio_planner/contact/contact_variables.h>
#include <sensor_msgs/JointState.h>
#include <moveit_msgs/Constraints.h>
#include <moveit_msgs/TrajectoryConstraints.h>

namespace itomp_cio_planner
{
ITOMP_FORWARD_DECL(ParameterTrajectory);
ITOMP_FORWARD_DECL(FullTrajectory)

class FullTrajectory: public Trajectory
{
public:
	enum TRAJECTORY_COMPONENT
	{
		TRAJECTORY_COMPONENT_JOINT = 0,
		TRAJECTORY_COMPONENT_CONTACT_POSITION,
		TRAJECTORY_COMPONENT_CONTACT_FORCE,
		TRAJECTORY_COMPONENT_NUM,
	};

	// Construct a full-DOF state trajectory
	FullTrajectory(const ItompRobotModelConstPtr& robot_model, double duration,
				   double discretization, double keyframe_interval = 0.0,
				   bool has_velocity_and_acceleration = false, bool free_point_end =
					   false, int num_contacts = 0);
	virtual ~FullTrajectory();

	virtual void allocate();

	void updateFromParameterTrajectory(
		const ParameterTrajectoryConstPtr& parameter_trajectory,
		const ItompPlanningGroupConstPtr& planning_group);
	void directChangeForDerivatives(double value,
									const ItompPlanningGroupConstPtr& planning_group, int type,
									int point, int element, int& full_point_begin, int& full_point_end,
									bool backup = true);

	Eigen::Block<Eigen::MatrixXd> getComponentTrajectory(
		TRAJECTORY_COMPONENT component, TRAJECTORY_TYPE type =
			TRAJECTORY_TYPE_POSITION);
	Eigen::MatrixXd getComponentTrajectory(TRAJECTORY_COMPONENT component,
										   TRAJECTORY_TYPE type = TRAJECTORY_TYPE_POSITION) const;

	void setComponentTrajectory(const Eigen::MatrixXd& trajectory,
								TRAJECTORY_COMPONENT component, TRAJECTORY_TYPE type =
									TRAJECTORY_TYPE_POSITION);

	int getComponentSize(TRAJECTORY_COMPONENT component) const;

	void setStartState(const sensor_msgs::JointState& joint_state,
					   const ItompRobotModelConstPtr& robot_model, bool fill_trajectory);
	void setGroupGoalState(const sensor_msgs::JointState& joint_goal_state,
						   const ItompPlanningGroupConstPtr& planning_group,
						   const ItompRobotModelConstPtr& robot_model,
						   const moveit_msgs::TrajectoryConstraints& trajectory_constraints,
						   const moveit_msgs::Constraints& path_constraints,
						   bool fill_trajectory_min_jerk);

	void setContactVariables(int point, const std::vector<ContactVariables>& contact_variables);
	void getContactVariables(int point, std::vector<ContactVariables>& contact_variables);
	void interpolateContactVariables();

	FullTrajectory* createClone() const;

	void backupTrajectories(int point_begin, int point_end, int element);
	void restoreBackupTrajectories();

	int getKeyframeStartIndex() const;
	int getNumKeyframeIntervalPoints() const;
	int getNumKeyframes() const;
	bool hasFreeEndPoint() const;

	Eigen::Vector3d getContactForce(int point, int contact_index) const;

	virtual void printTrajectory(bool position = true, bool velocity = true,
								 bool acceleration = true) const;

protected:
	void copyFromParameterTrajectory(
		const ParameterTrajectoryConstPtr& parameter_trajectory,
		const ItompPlanningGroupConstPtr& planning_group,
		int parameter_point_begin, int parameter_point_end);

	void updateTrajectoryFromKeyframes(int keyframe_begin, int keyframe_end);
	void updateTrajectoryFromKeyframes(int keyframe_begin, int keyframe_end,
									   int element);

	/**
	 * \brief Generates a minimum jerk trajectory from the start index to end index
	 *
	 */
	void fillInMinJerk(const std::set<int>& group_to_full_joint_indices);
	void fillInMinJerkCartesianTrajectory(
		const ItompRobotModelConstPtr& robot_model,
		const ItompPlanningGroupConstPtr& planning_group,
		const moveit_msgs::Constraints& path_constraints);
	void fillInMinJerk(const std::set<int>& groupJointsKDLIndices,
					   const ItompRobotModelConstPtr& robot_model,
					   const ItompPlanningGroupConstPtr& planning_group,
					   const moveit_msgs::TrajectoryConstraints& trajectory_constraints);

	int num_contacts_;
	bool has_free_end_point_;

	int keyframe_start_index_;
	int num_keyframe_interval_points_;
	double keyframe_interval_;
	int num_keyframes_;

	int component_start_indices_[TRAJECTORY_COMPONENT_NUM + 1];

	int backup_point_begin_;
	int backup_point_end_;
	int backup_element_;
	Eigen::MatrixXd backup_trajectory_[TRAJECTORY_TYPE_NUM];

	friend class TrajectoryFactory;
	friend class ParameterTrajectory;
};
ITOMP_DEFINE_SHARED_POINTERS(FullTrajectory);

///////////////////////// inline functions follow //////////////////////
inline Eigen::Block<Eigen::MatrixXd> FullTrajectory::getComponentTrajectory(
	TRAJECTORY_COMPONENT component, TRAJECTORY_TYPE type)
{
	return trajectory_[type].block(0, component_start_indices_[component],
								   trajectory_[type].rows(), getComponentSize(component));
}

inline Eigen::MatrixXd FullTrajectory::getComponentTrajectory(
	TRAJECTORY_COMPONENT component, TRAJECTORY_TYPE type) const
{
	return trajectory_[type].block(0, component_start_indices_[component],
								   trajectory_[type].rows(), getComponentSize(component));
}

inline void FullTrajectory::setComponentTrajectory(
	const Eigen::MatrixXd& trajectory, TRAJECTORY_COMPONENT component,
	TRAJECTORY_TYPE type)
{
	trajectory_[type].block(0, component_start_indices_[component],
							trajectory_[type].rows(), getComponentSize(component)) = trajectory;
}

inline int FullTrajectory::getComponentSize(
	TRAJECTORY_COMPONENT component) const
{
	return component_start_indices_[component + 1]
		   - component_start_indices_[component];
}

inline FullTrajectory* FullTrajectory::createClone() const
{
	FullTrajectory* new_trajectory = new FullTrajectory(*this);
	return new_trajectory;
}

inline int FullTrajectory::getKeyframeStartIndex() const
{
	return keyframe_start_index_;
}

inline int FullTrajectory::getNumKeyframeIntervalPoints() const
{
	return num_keyframe_interval_points_;
}

inline int FullTrajectory::getNumKeyframes() const
{
	return num_keyframes_;
}

inline bool FullTrajectory::hasFreeEndPoint() const
{
	return has_free_end_point_;
}

inline Eigen::Vector3d FullTrajectory::getContactForce(int point,
		int contact_index) const
{
	// mass * gravity
	double scale = 150 * 100;
	Eigen::Vector3d force_param = getComponentTrajectory(
									  TRAJECTORY_COMPONENT_CONTACT_FORCE).block(point, contact_index * 3,
											  1, 3).transpose();
	force_param *= scale;

	if (force_param(2) < 0.0)
		force_param(2) = -force_param(2);
	return force_param;
}

}

#endif /* FULL_TRAJECTORY_H_ */
