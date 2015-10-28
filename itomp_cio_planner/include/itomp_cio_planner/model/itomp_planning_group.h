#ifndef ITOMP_PLANNING_GROUP_H_
#define ITOMP_PLANNING_GROUP_H_

//#include <ros/ros.h>
#include <itomp_cio_planner/common.h>
#include <kdl/tree.hpp>
#include <itomp_cio_planner/model/itomp_robot_joint.h>
#include <itomp_cio_planner/contact/contact_point.h>

namespace itomp_cio_planner
{

class ItompPlanningGroup
{
public:
	std::string name_; /**< Name of the planning group */
	int num_joints_; /**< Number of joints used in planning */
	std::vector<ItompRobotJoint> group_joints_; /**< Joints used in planning */
	std::vector<ContactPoint> contact_points_;
	std::map<int, int> rbdl_to_group_joint_;
    mutable std::vector<bool> is_fixed_;

	int getNumContacts() const;
};
ITOMP_DEFINE_SHARED_POINTERS(ItompPlanningGroup)

////////////////////////////////////////////////////////////////////////////////

inline int ItompPlanningGroup::getNumContacts() const
{
	return contact_points_.size();
}

}
#endif
