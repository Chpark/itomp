#ifndef ITOMP_ROBOT_MODEL_H_
#define ITOMP_ROBOT_MODEL_H_

//#include <ros/ros.h>
#include <itomp_cio_planner/common.h>
#include <itomp_cio_planner/model/itomp_planning_group.h>
#include <itomp_cio_planner/model/itomp_robot_joint.h>
#include <itomp_cio_planner/model/treefksolverjointposaxis.hpp>
#include <itomp_cio_planner/model/treefksolverjointposaxis_partial.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <boost/shared_ptr.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <moveit/robot_model/robot_model.h>

#include <sensor_msgs/JointState.h>
#include <ros/console.h>

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
	bool init(robot_model::RobotModelPtr& robot_model, const std::string& robot_description);

	/**
	 * \brief Gets the planning group corresponding to the group name
	 */
	const ItompPlanningGroup *getPlanningGroup(const std::string& group_name) const;

	/**
	 * \brief Gets the number of joints in the KDL tree
	 */
	int getNumKDLJoints() const;

	/**
	 * \brief Gets the KDL tree
	 */
	const KDL::Tree* getKDLTree() const;
	KDL::Tree* getKDLTree();

	/**
	 * \brief Gets the KDL joint number from the URDF joint name
	 *
	 * \return -1 if the joint name is not found
	 */
	int urdfNameToKdlNumber(const std::string& urdf_name) const;

	/**
	 * \brief Gets the URDF joint name from the KDL joint number
	 *
	 * \return "" if the number does not have a name
	 */
	const std::string kdlNumberToUrdfName(int kdl_number) const;

	const KDL::TreeFkSolverJointPosAxis* getForwardKinematicsSolver() const;

	const std::string& getReferenceFrame() const;

	void jointStateToArray(const sensor_msgs::JointState &joint_state, KDL::JntArray& joint_array);
	void jointStateToArray(const sensor_msgs::JointState &joint_state, Eigen::MatrixXd::RowXpr joint_array,
			Eigen::MatrixXd::RowXpr joint_vel_array, Eigen::MatrixXd::RowXpr joint_acc_array);

	std::vector<std::string> getJointNames() const;

	const std::string& getRobotName() const;

	robot_model::RobotModelPtr getRobotModel();
	const robot_model::RobotModelConstPtr getRobotModel() const;

private:
	robot_model::RobotModelPtr robot_model_;

	KDL::Tree kdl_tree_; /**< The KDL tree of the entire robot */
	int num_kdl_joints_; /**< Total number of joints in the KDL tree */
	std::map<std::string, std::string> joint_segment_mapping_; /**< Joint -> Segment mapping for KDL tree */
	std::map<std::string, std::string> segment_joint_mapping_; /**< Segment -> Joint mapping for KDL tree */
	std::vector<std::string> kdl_number_to_urdf_name_; /**< Mapping from KDL joint number to URDF joint name */
	std::map<std::string, int> urdf_name_to_kdl_number_; /**< Mapping from URDF joint name to KDL joint number */
	std::map<std::string, ItompPlanningGroup> planning_groups_; /**< Planning group information */
	KDL::TreeFkSolverJointPosAxis *fk_solver_; /**< Forward kinematics solver for the tree */
	std::string reference_frame_; /**< Reference frame for all kinematics operations */
};

/////////////////////////////// inline functions follow ///////////////////////////////////

inline const ItompPlanningGroup* ItompRobotModel::getPlanningGroup(const std::string& group_name) const
{
	std::map<std::string, ItompPlanningGroup>::const_iterator it = planning_groups_.find(group_name);
	if (it == planning_groups_.end())
		return NULL;
	else
		return &(it->second);
}

inline int ItompRobotModel::getNumKDLJoints() const
{
	return num_kdl_joints_;
}

inline const KDL::Tree* ItompRobotModel::getKDLTree() const
{
	return &kdl_tree_;
}

inline KDL::Tree* ItompRobotModel::getKDLTree()
{
	return &kdl_tree_;
}

inline int ItompRobotModel::urdfNameToKdlNumber(const std::string& urdf_name) const
{
	std::map<std::string, int>::const_iterator it = urdf_name_to_kdl_number_.find(urdf_name);
	if (it != urdf_name_to_kdl_number_.end())
		return it->second;
	else
		return -1;
}

inline const std::string ItompRobotModel::kdlNumberToUrdfName(int kdl_number) const
{
	if (kdl_number < 0 || kdl_number >= num_kdl_joints_)
		return std::string("");
	else
		return kdl_number_to_urdf_name_[kdl_number];
}

inline const KDL::TreeFkSolverJointPosAxis* ItompRobotModel::getForwardKinematicsSolver() const
{
	return fk_solver_;
}

inline const std::string& ItompRobotModel::getReferenceFrame() const
{
	return reference_frame_;
}

inline void ItompRobotModel::jointStateToArray(const sensor_msgs::JointState &joint_state, KDL::JntArray& joint_array)
{
	for (unsigned int i = 0; i < joint_state.name.size(); i++)
	{
		std::string name = joint_state.name[i];
		int kdl_number = urdfNameToKdlNumber(name);
		if (kdl_number >= 0)
			joint_array(kdl_number) = joint_state.position[i];
	}
}

inline void ItompRobotModel::jointStateToArray(const sensor_msgs::JointState &joint_state,
		Eigen::MatrixXd::RowXpr joint_array, Eigen::MatrixXd::RowXpr joint_vel_array,
		Eigen::MatrixXd::RowXpr joint_acc_array)
{
	ROS_INFO("Initial Joint States");
	for (unsigned int i = 0; i < joint_state.name.size(); i++)
	{
		std::string name = joint_state.name[i];
		int kdl_number = urdfNameToKdlNumber(name);
		if (kdl_number >= 0)
		{
			joint_array(kdl_number) = joint_state.position[i];
			joint_vel_array(kdl_number) = joint_state.velocity[i];
			joint_acc_array(kdl_number) = joint_state.effort[i];
			ROS_INFO("%s : %f %f %f", name.c_str(), joint_state.position[i], joint_state.velocity[i], joint_state.effort[i]);
		}
	}
}

inline std::vector<std::string> ItompRobotModel::getJointNames() const
{
	return kdl_number_to_urdf_name_;
}

inline const std::string& ItompRobotModel::getRobotName() const
{
	return robot_model_->getName();
}

inline const robot_model::RobotModelConstPtr ItompRobotModel::getRobotModel() const
{
	return robot_model_;
}

inline robot_model::RobotModelPtr ItompRobotModel::getRobotModel()
{
	return robot_model_;
}


}
#endif
