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
    unsigned int num_contact_forces = PlanningParameters::getInstance()->getNumContacts() * NUM_ENDEFFECTOR_CONTACT_POINTS * 4; // n * force(3)

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
