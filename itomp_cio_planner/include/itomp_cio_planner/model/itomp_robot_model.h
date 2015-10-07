#ifndef ITOMP_ROBOT_MODEL_H_
#define ITOMP_ROBOT_MODEL_H_

#include <itomp_cio_planner/common.h>
#include <itomp_cio_planner/model/itomp_planning_group.h>
#include <itomp_cio_planner/model/itomp_robot_joint.h>
#include <moveit/robot_model/robot_model.h>
#include <sensor_msgs/JointState.h>
#include <ros/console.h>
#include <rbdl/rbdl.h>

namespace itomp_cio_planner
{

class ItompRobotModel
{
public:

	ItompRobotModel();
	virtual ~ItompRobotModel();

	/**
	 * \brief Initializes the robot models
	 *
	 * \return true if successful, false if not
	 */
	bool init(const robot_model::RobotModelConstPtr& robot_model);

	/**
	 * \brief Gets the planning group corresponding to the group name
	 */
	const ItompPlanningGroupConstPtr& getPlanningGroup(const std::string& group_name) const;

	/**
	 * \brief Gets the number of total joints
	 */
	int getNumJoints() const;

	/**
     * \brief Gets the RBDL joint number from the URDF joint name
	 *
	 * \return -1 if the joint name is not found
	 */
	int jointNameToRbdlNumber(const std::string& joint_name) const;

	/**
     * \brief Gets the URDF joint name from the RBDL joint number
	 *
	 * \return "" if the number does not have a name
	 */
	const std::string rbdlNumberToJointName(int rbdl_number) const;

	const std::string& getReferenceFrame() const;
	const std::string& getRobotName() const;

	const robot_model::RobotModelConstPtr& getMoveitRobotModel() const;
	const RigidBodyDynamics::Model& getRBDLRobotModel() const;

private:
	robot_model::RobotModelConstPtr moveit_robot_model_;
	std::string reference_frame_; /**< Reference frame for all kinematics operations */

	RigidBodyDynamics::Model rbdl_robot_model_;
	int num_rbdl_joints_;

	std::map<std::string, ItompPlanningGroupConstPtr> planning_groups_; /**< Planning group information */
	std::vector<std::string> rbdl_number_to_joint_name_; /**< Mapping from RBDL joint number (1-base) to URDF joint name */
	std::map<std::string, int> joint_name_to_rbdl_number_; /**< Mapping from URDF joint name to RBDL joint number (1-base) */
};
ITOMP_DEFINE_SHARED_POINTERS(ItompRobotModel)

/////////////////////////////// inline functions follow ///////////////////////////////////

inline const ItompPlanningGroupConstPtr& ItompRobotModel::getPlanningGroup(const std::string& group_name) const
{
	std::map<std::string, ItompPlanningGroupConstPtr>::const_iterator it = planning_groups_.find(group_name);
	if (it == planning_groups_.end())
		ROS_ERROR("Planning group %s does not exist!!!", group_name.c_str());
	return it->second;
}

inline int ItompRobotModel::getNumJoints() const
{
	return num_rbdl_joints_;
}

inline int ItompRobotModel::jointNameToRbdlNumber(const std::string& joint_name) const
{
	std::map<std::string, int>::const_iterator it = joint_name_to_rbdl_number_.find(joint_name);
	if (it != joint_name_to_rbdl_number_.end())
		return it->second;
	else
		return -1;
}

inline const std::string ItompRobotModel::rbdlNumberToJointName(int rbdl_number) const
{
	if (rbdl_number < 0 || rbdl_number >= num_rbdl_joints_)
		return std::string("");
	else
		return rbdl_number_to_joint_name_[rbdl_number];
}

inline const std::string& ItompRobotModel::getReferenceFrame() const
{
	return reference_frame_;
}

inline const std::string& ItompRobotModel::getRobotName() const
{
	return moveit_robot_model_->getName();
}

inline const robot_model::RobotModelConstPtr& ItompRobotModel::getMoveitRobotModel() const
{
	return moveit_robot_model_;
}

inline const RigidBodyDynamics::Model& ItompRobotModel::getRBDLRobotModel() const
{
	return rbdl_robot_model_;
}

}
#endif
