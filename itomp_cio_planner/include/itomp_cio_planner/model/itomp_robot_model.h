#ifndef ITOMP_ROBOT_MODEL_H_
#define ITOMP_ROBOT_MODEL_H_

#include <itomp_cio_planner/common.h>
#include <itomp_cio_planner/model/itomp_planning_group.h>
#include <itomp_cio_planner/model/itomp_robot_joint.h>
#include <itomp_cio_planner/model/treefksolverjointposaxis.hpp>
#include <itomp_cio_planner/model/treefksolverjointposaxis_partial.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <moveit/robot_model/robot_model.h>
#include <sensor_msgs/JointState.h>
#include <ros/console.h>
#include <rbdl/rbdl.h>

namespace itomp_cio_planner
{

struct ItompRobotModelIKData
{
    Eigen::Vector3d root_to_hip;
    Eigen::Vector3d hip_to_knee;
    Eigen::Vector3d knee_to_ankle;
    Eigen::Vector3d ankle_to_ee;

    double h1;
    double h2;
    double ph1;
    double ph2;

    double max_stretch;

    Eigen::Affine3d ee_to_root;
};

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
	int jointNameToRbdlNumber(const std::string& joint_name) const;

	/**
	 * \brief Gets the URDF joint name from the KDL joint number
	 *
	 * \return "" if the number does not have a name
	 */
	const std::string rbdlNumberToJointName(int rbdl_number) const;

	const KDL::TreeFkSolverJointPosAxis* getForwardKinematicsSolver() const;

	const std::string& getReferenceFrame() const;

	void jointStateToArray(const sensor_msgs::JointState &joint_state, KDL::JntArray& joint_array);

	std::vector<std::string> getJointNames() const;

	const std::string& getRobotName() const;

	const robot_model::RobotModelConstPtr& getMoveitRobotModel() const;

	const RigidBodyDynamics::Model& getRBDLRobotModel() const;

	const std::set<std::string>& getContactPointNames() const;

    bool getGroupEndeffectorPos(const std::string& group_name, const robot_state::RobotState& robot_state, Eigen::Affine3d& ee_pose) const;
    bool computeStandIKState(robot_state::RobotState& robot_state, Eigen::Affine3d& root_pose, const Eigen::Affine3d& left_foot_pose, const Eigen::Affine3d& right_foot_pose) const;
    bool getRootPose(const std::string& group_name, const Eigen::Affine3d& ee_pose, Eigen::Affine3d& root_pose) const;

private:
    void initializeIKData(const std::string& group_name) const;
    bool computeInverseKinematics(const std::string& group_name, const Eigen::Affine3d& root_pose, const Eigen::Affine3d& dest_pose,
                                  std::vector<double>& joint_values) const;
    bool adjustRootZ(const std::string& group_name, Eigen::Affine3d& root_pose, const Eigen::Affine3d& dest_pose) const;


	robot_model::RobotModelConstPtr moveit_robot_model_;
	std::string reference_frame_; /**< Reference frame for all kinematics operations */

	KDL::Tree kdl_tree_; /**< The KDL tree of the entire robot */
	int num_kdl_joints_; /**< Total number of joints in the KDL tree */
	std::map<std::string, std::string> joint_segment_mapping_; /**< Joint -> Segment mapping for KDL tree */
	std::map<std::string, std::string> segment_joint_mapping_; /**< Segment -> Joint mapping for KDL tree */
	std::vector<std::string> kdl_number_to_urdf_name_; /**< Mapping from KDL joint number to URDF joint name */
	std::map<std::string, int> urdf_name_to_kdl_number_; /**< Mapping from URDF joint name to KDL joint number */

	RigidBodyDynamics::Model rbdl_robot_model_;
	int num_rbdl_joints_;

	std::map<std::string, ItompPlanningGroupConstPtr> planning_groups_; /**< Planning group information */
	std::vector<std::string> rbdl_number_to_joint_name_; /**< Mapping from RBDL joint number (1-base) to URDF joint name */
	std::map<std::string, int> joint_name_to_rbdl_number_; /**< Mapping from URDF joint name to RBDL joint number (1-base) */

    mutable std::map<std::string, ItompRobotModelIKData> ik_data_map_;
};
ITOMP_DEFINE_SHARED_POINTERS(ItompRobotModel);

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

inline void ItompRobotModel::jointStateToArray(const sensor_msgs::JointState &joint_state, KDL::JntArray& joint_array)
{
	for (unsigned int i = 0; i < joint_state.name.size(); i++)
	{
		std::string name = joint_state.name[i];
		int kdl_number = jointNameToRbdlNumber(name);
		if (kdl_number >= 0)
			joint_array(kdl_number) = joint_state.position[i];
	}
}

inline std::vector<std::string> ItompRobotModel::getJointNames() const
{
	return kdl_number_to_urdf_name_;
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
