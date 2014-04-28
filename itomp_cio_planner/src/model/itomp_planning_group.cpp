#include <itomp_cio_planner/model/itomp_planning_group.h>

namespace itomp_cio_planner
{

std::vector<std::string> ItompPlanningGroup::getJointNames() const
{
	std::vector<std::string> ret;
	for (unsigned int i = 0; i < group_joints_.size(); ++i)
		ret.push_back(group_joints_[i].joint_name_);
	return ret;
}

}
