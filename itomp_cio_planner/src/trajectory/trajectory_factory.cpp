#include <itomp_cio_planner/trajectory/trajectory_factory.h>

namespace itomp_cio_planner
{

TrajectoryFactory::TrajectoryFactory()
{

}

TrajectoryFactory::~TrajectoryFactory()
{

}

void TrajectoryFactory::initialize(TRAJECTORY_FACTORY_TYPE type)
{
	type_ = type;
}

FullTrajectory* TrajectoryFactory::CreateFullTrajectory(
		const ItompRobotModelConstPtr& robot_model, double duration,
		double discretization, double keyframe_interval)
{
	FullTrajectory* full_trajectory = NULL;

	// create
	switch (type_)
	{
	case TRAJECTORY_CIO:
		full_trajectory = new FullTrajectory(robot_model,
				duration, discretization, keyframe_interval, true, true,
				robot_model->getContactPointNames().size());
		break;

	case TRAJECTORY_STOMP:
		full_trajectory = new FullTrajectory(robot_model,
				duration, discretization, keyframe_interval, false, false, 0);
		ROS_ERROR("Trajectory for STOMP is not implemented");
		break;

	default:
		ROS_ERROR("Trajectory factory is not initialized");
		break;
	}

	// allocate
	full_trajectory->allocate();

	return full_trajectory;
}

ParameterTrajectory* TrajectoryFactory::CreateParameterTrajectory(
		const FullTrajectoryConstPtr& full_trajectory,
		const ItompPlanningGroupConstPtr& planning_group)
{
	// create
	ParameterTrajectory* parameter_trajectory = new ParameterTrajectory(full_trajectory, planning_group);

	// allocate
	parameter_trajectory->allocate();

	int num_full_joints = full_trajectory->getComponentSize(
			FullTrajectory::TRAJECTORY_COMPONENT_JOINT);
	int num_parameter_joints = parameter_trajectory->num_joints_;

	// copy joint parameters from full to parameter trajectory
	for (int i = 0; i < num_parameter_joints; ++i)
	{
		int full_joint_index =
				parameter_trajectory->group_to_full_joint_indices[i];
		for (int j = 0; j < parameter_trajectory->getNumPoints(); ++j)
		{
			int keyframe_index = full_trajectory->keyframe_start_index_
					+ (j + 1) * full_trajectory->num_keyframe_interval_points_;
			parameter_trajectory->trajectory_[Trajectory::TRAJECTORY_TYPE_POSITION](
					j, i) =
					full_trajectory->trajectory_[Trajectory::TRAJECTORY_TYPE_POSITION](
							keyframe_index, full_joint_index);

			if (parameter_trajectory->has_velocity_)
			{
				parameter_trajectory->trajectory_[Trajectory::TRAJECTORY_TYPE_VELOCITY](
						j, i) =
						full_trajectory->trajectory_[Trajectory::TRAJECTORY_TYPE_VELOCITY](
								keyframe_index, full_joint_index);
			}
		}
	}

	// copy other parameters from full to parameter trajectory
	int copy_size = parameter_trajectory->getNumElements()
			- num_parameter_joints;
	if (copy_size > 0)
	{
		for (int j = 0; j < parameter_trajectory->getNumPoints(); ++j)
		{
			int keyframe_index = full_trajectory->keyframe_start_index_
					+ (j + 1) * full_trajectory->num_keyframe_interval_points_;

			parameter_trajectory->trajectory_[Trajectory::TRAJECTORY_TYPE_POSITION].block(
					j, num_parameter_joints, 1, copy_size) =
					full_trajectory->trajectory_[Trajectory::TRAJECTORY_TYPE_POSITION].block(
							keyframe_index, num_full_joints, 1, copy_size);

			if (parameter_trajectory->has_velocity_)
			{
				parameter_trajectory->trajectory_[Trajectory::TRAJECTORY_TYPE_VELOCITY].block(
						j, num_parameter_joints, 1, copy_size) =
						full_trajectory->trajectory_[Trajectory::TRAJECTORY_TYPE_VELOCITY].block(
								keyframe_index, num_full_joints, 1, copy_size);
			}
		}
	}

	return parameter_trajectory;
}

}
