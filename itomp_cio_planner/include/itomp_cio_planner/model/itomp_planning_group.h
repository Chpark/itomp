#ifndef ITOMP_PLANNING_GROUP_H_
#define ITOMP_PLANNING_GROUP_H_

//#include <ros/ros.h>
#include <itomp_cio_planner/common.h>
#include <kdl/tree.hpp>
#include <itomp_cio_planner/model/itomp_robot_joint.h>
#include <itomp_cio_planner/model/treefksolverjointposaxis_partial.hpp>
#include <itomp_cio_planner/contact/contact_point.h>

namespace itomp_cio_planner
{

class ItompPlanningGroup
{
public:
	std::string name_; /**< Name of the planning group */
	int num_joints_; /**< Number of joints used in planning */
	std::vector<ItompRobotJoint> group_joints_; /**< Joints used in planning */
	std::vector<std::string> link_names_; /**< Links used in planning */
	std::vector<std::string> collision_link_names_; /**< Links used in collision checking */
	boost::shared_ptr<KDL::TreeFkSolverJointPosAxisPartial> fk_solver_; /**< Forward kinematics solver for the group */
	std::vector<ContactPoint> contactPoints_;
	std::map<int, int> kdl_to_group_joint_;

	std::vector<std::string> getJointNames() const;
	int getNumContacts() const;
};

////////////////////////////////////////////////////////////////////////////////

inline int ItompPlanningGroup::getNumContacts() const
{
	return contactPoints_.size();
}

}
#endif
