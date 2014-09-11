#include <itomp_cio_planner/trajectory/parameter_trajectory.h>
#include <itomp_cio_planner/trajectory/full_trajectory.h>
#include <itomp_cio_planner/model/itomp_planning_group.h>

namespace itomp_cio_planner
{

ParameterTrajectory::ParameterTrajectory(
		const FullTrajectoryConstPtr& full_trajectory,
		const ItompPlanningGroupConstPtr& planning_group) :
		Trajectory(full_trajectory->discretization_,
				full_trajectory->has_velocity_, false)
{
	duration_ = full_trajectory->duration_;
	num_points_ = full_trajectory->num_keyframes_
			- (full_trajectory->has_free_end_point_ ? 1 : 2);
	num_joints_ = num_elements_ = planning_group->num_joints_;
	group_to_full_joint_indices.resize(num_elements_);
	int i = 0;
	for (; i < num_elements_; ++i)
		group_to_full_joint_indices[i] =
				planning_group->group_joints_[i].rbdl_joint_index_;

	num_elements_ += full_trajectory->getComponentSize(
			FullTrajectory::TRAJECTORY_COMPONENT_CONTACT_POSITION)
			+ full_trajectory->getComponentSize(
					FullTrajectory::TRAJECTORY_COMPONENT_CONTACT_FORCE);
}

ParameterTrajectory::~ParameterTrajectory()
{

}

void ParameterTrajectory::handleJointLimits(
		const ItompPlanningGroupConstPtr& planning_group, int point_begin, int point_end)
{
	for (int joint = 0; joint < num_joints_; ++joint)
	{
		if (!planning_group->group_joints_[joint].has_joint_limits_)
			continue;

		double joint_max = planning_group->group_joints_[joint].joint_limit_max_;
		double joint_min = planning_group->group_joints_[joint].joint_limit_min_;

		int count = 0;

		for (int point = point_begin; point < point_end; ++point)
		{
			if (trajectory_[TRAJECTORY_TYPE_POSITION](point, joint) > joint_max)
			{
				trajectory_[TRAJECTORY_TYPE_POSITION](point, joint) = joint_max;
			}
			else if (trajectory_[TRAJECTORY_TYPE_POSITION](point, joint)
					< joint_min)
			{
				trajectory_[TRAJECTORY_TYPE_POSITION](point, joint) = joint_min;
			}
		}
	}
}

}

