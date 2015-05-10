#include <itomp_cio_planner/trajectory/trajectory_factory.h>
#include <itomp_cio_planner/trajectory/element_trajectory.h>
#include <itomp_cio_planner/util/planning_parameters.h>

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
		full_trajectory = new FullTrajectory(robot_model, duration,
											 discretization, keyframe_interval, true, false,
											 PlanningParameters::getInstance()->getNumContacts());
		break;

	case TRAJECTORY_STOMP:
		full_trajectory = new FullTrajectory(robot_model, duration,
											 discretization, keyframe_interval, false, false, 0);
		ROS_ERROR("Trajectory for STOMP is not implemented");
		break;

	default:
		ROS_ERROR("Trajectory factory is not initialized");
		break;
	}

	// allocate
	full_trajectory->allocate();

	ROS_INFO(
		"Full trajectory joints : %d", full_trajectory->getComponentSize(FullTrajectory::TRAJECTORY_COMPONENT_JOINT));
	ROS_INFO( "Full trajectory length : %d", full_trajectory->getNumElements());

	return full_trajectory;
}

ParameterTrajectory* TrajectoryFactory::CreateParameterTrajectory(
	const FullTrajectoryConstPtr& full_trajectory,
	const ItompPlanningGroupConstPtr& planning_group)
{
	// create
	ParameterTrajectory* parameter_trajectory = new ParameterTrajectory(
		full_trajectory, planning_group);

	// allocate
	parameter_trajectory->allocate();

	int num_full_joints = full_trajectory->getComponentSize(
							  FullTrajectory::TRAJECTORY_COMPONENT_JOINT);
	int num_parameter_joints = parameter_trajectory->num_joints_;

    if (PlanningParameters::getInstance()->getPrintPlanningInfo())
    {
        ROS_INFO("Parameter trajectory joints : %d/%d", num_parameter_joints, num_full_joints);
        ROS_INFO("Parameter trajectory length : %d", parameter_trajectory->getNumElements());
    }

	// copy joint parameters from full to parameter trajectory
	for (int i = 0; i < num_parameter_joints; ++i)
	{
		int full_joint_index =
			parameter_trajectory->group_to_full_joint_indices[i];
		for (int j = 0; j < parameter_trajectory->getNumPoints(); ++j)
		{
			int keyframe_index = full_trajectory->keyframe_start_index_
								 + j * full_trajectory->num_keyframe_interval_points_;
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
								 + j * full_trajectory->num_keyframe_interval_points_;

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

ItompTrajectory* TrajectoryFactory::CreateItompTrajectory(const ItompRobotModelConstPtr& robot_model, double duration,
        double discretization, double keyframe_interval)
{
    ItompTrajectory* new_trajectory = NULL;

    bool has_velocity_acceleration = true;
    bool has_free_end_point = false;

    // create
    switch (type_)
    {
    case TRAJECTORY_CIO:
        break;

    case TRAJECTORY_STOMP:
        has_velocity_acceleration = false;
        has_free_end_point = false;
        ROS_ERROR("Trajectory for STOMP is not implemented");
        break;

    default:
        ROS_ERROR("Trajectory factory is not initialized");
        break;
    }

    unsigned int num_points = 0;
    unsigned int num_keyframes = 0;
    unsigned int num_keyframe_interval_points = 0;
    computeTrajectoryParameters(num_points, num_keyframes, num_keyframe_interval_points,
                                duration, keyframe_interval, discretization);

    std::string name = "trajectory";
    unsigned int num_joints = robot_model->getNumJoints();
    unsigned int num_contact_positions = PlanningParameters::getInstance()->getNumContacts() * 7; // var + pos(3) + ori(3)
    unsigned int num_contact_forces = PlanningParameters::getInstance()->getNumContacts() * NUM_ENDEFFECTOR_CONTACT_POINTS * 3; // n * force(3)

    std::vector<NewTrajectoryPtr> components(3);
    std::vector<NewTrajectoryPtr> components_sub(3);
    // add pos, vel, acc components
    std::string component_names[] = {"position", "velocity", "acceleration"};
    for (int i = 0; i < ItompTrajectory::COMPONENT_TYPE_NUM; ++i)
    {
        components_sub[0].reset(new ElementTrajectory("joint value", num_points, num_joints));
        components_sub[1].reset(new ElementTrajectory("contact position", num_points, num_contact_positions));
        components_sub[2].reset(new ElementTrajectory("contact force", num_points, num_contact_forces));
        components[i].reset(new CompositeTrajectory(component_names[i], num_points, components_sub));
    }

    new_trajectory = new ItompTrajectory(name, num_points, components, num_keyframes, num_keyframe_interval_points, duration, discretization);

    return new_trajectory;
}

void TrajectoryFactory::computeTrajectoryParameters(unsigned int& num_points, unsigned int& num_keyframes, unsigned int& num_keyframe_interval_points,
        double& duration, double& keyframe_interval, double discretization) const
{
    double new_keyframe_interval = keyframe_interval;
    double new_duration = duration;

    if (keyframe_interval <= discretization)
    {
        num_keyframe_interval_points = 1;
        num_keyframes = num_points = safeDoubleToInt(
                                         duration / discretization) + 1;
        new_duration = (num_points - 1) * discretization;
    }
    else
    {
        num_keyframe_interval_points = safeDoubleToInt(
                                           keyframe_interval / discretization);
        new_keyframe_interval = num_keyframe_interval_points * discretization;
        if (new_keyframe_interval != keyframe_interval)
            ROS_INFO(
                "Trajectory keyframe interval modified : %f -> %f", keyframe_interval, new_keyframe_interval);
        num_keyframes = safeDoubleToInt(duration / new_keyframe_interval) + 1;
        num_points = (num_keyframes - 1) * num_keyframe_interval_points + 1;
        new_duration = (num_keyframes - 1) * new_keyframe_interval;
    }
    if (new_duration != duration)
        ROS_INFO(
            "Trajectory duration modified : %f -> %f", duration, new_duration);

    keyframe_interval = new_keyframe_interval;
    duration = new_duration;
}

}
