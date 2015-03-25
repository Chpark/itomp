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
    // root x y follow the 2d traj
    if (index.sub_component == ItompTrajectory::SUB_COMPONENT_TYPE_JOINT &&
            (index.element < 2))
        return false;
        */

    // for standup
    /*
    {
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
    }
    */



    // for walking
    // Do not update joint values of start/goal points
    if (index.sub_component == ItompTrajectory::SUB_COMPONENT_TYPE_JOINT &&
            (index.point == 0 || index.point == num_points_ - 1))
        return false;


    /*
    if (PhaseManager::getInstance()->getPhase() != 0 &&
            (index.point == 0 || index.point == num_points_ - 1))
        return false;
        */





    /*
    // overhead bin_all
    // Do not update joint values of start/goal point
    if (index.sub_component == ItompTrajectory::SUB_COMPONENT_TYPE_JOINT && (index.point == 0 || index.point == num_points_ - 1))
        return false;


    // overhead bin_rotating
    if (getPhase() != 0 && index.sub_component == ItompTrajectory::SUB_COMPONENT_TYPE_CONTACT_FORCE &&
            index.point > 0 && index.point < 20 &&
            //index.element >= 12 && index.element < 24) // turn right
            index.element < 12) // turn left
        return false;
    */







    return true;
}

}
