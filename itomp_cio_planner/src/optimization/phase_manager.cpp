#include <itomp_cio_planner/optimization/phase_manager.h>

namespace itomp_cio_planner
{

PhaseManager::PhaseManager()
    : phase_(0), num_points_(0)
{

}

PhaseManager::~PhaseManager()
{

}

void PhaseManager::init(int num_points)
{
    num_points_ = num_points;
}

bool PhaseManager::updateParameter(const ItompTrajectoryIndex& index) const
{
    /*
    // for walking
    // Do not update joint values of start/goal points
    if (index.sub_component == ItompTrajectory::SUB_COMPONENT_TYPE_JOINT &&
            (index.point == 0 || index.point == num_points_ - 1))
        return false;
        */

    // for standing up
    // no contact force from foot
    if (index.point == 0)
    {
        if (index.sub_component == ItompTrajectory::SUB_COMPONENT_TYPE_CONTACT_FORCE)
        {
            if (index.element < 24)
                return false;
        }
    }

    // fix only root_x, root_y
    if (index.sub_component == ItompTrajectory::SUB_COMPONENT_TYPE_JOINT &&
            (index.point == 0 || index.point == num_points_ - 1) &&
            (index.element < 2))
        return false;

    /*
    if (PhaseManager::getInstance()->getPhase() != 0 &&
        (index.point == 0 || index.point == num_points_ - 1))
    return false;
    */

    return true;
}

}
