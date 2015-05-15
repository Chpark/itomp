#include <itomp_cio_planner/optimization/phase_manager.h>
#include <itomp_cio_planner/util/planning_parameters.h>

namespace itomp_cio_planner
{

PhaseManager::PhaseManager()
    : phase_(0), num_points_(0)
{
    support_foot_ = 0; // any
    agent_id_ = 0;
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
    int state = (int)(PlanningParameters::getInstance()->getTemporaryVariable(0) + 0.0001);

    switch(state)
    {
    case 0: // standing up
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
    break;

    case 1: // side walking
    case 2:
        break;

    case 3: // luggage turn right
        // overhead bin_all
        // Do not update joint values of start/goal point
        if (index.sub_component == ItompTrajectory::SUB_COMPONENT_TYPE_JOINT && (index.point == 0 || index.point == num_points_ - 1))
            return false;

        // overhead bin_rotating
        if (getPhase() != 0 && index.sub_component == ItompTrajectory::SUB_COMPONENT_TYPE_CONTACT_FORCE &&
                index.point > 0 && index.point < 20 &&
                index.element >= 12 && index.element < 24) // turn right
            //index.element < 12) // turn left
            return false;

        break;

    case 4: // luggage turn left
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

        break;

    case 5: // luggage
        // overhead bin_all
        // Do not update joint values of start/goal point
        if (index.sub_component == ItompTrajectory::SUB_COMPONENT_TYPE_JOINT && (index.point == 0 || index.point == num_points_ - 1))
            return false;
        break;

    case 10: // walking
        // Do not update joint values of start/goal points
        if (index.sub_component == ItompTrajectory::SUB_COMPONENT_TYPE_JOINT &&
                (index.point == 0 || index.point == num_points_ - 1))
            return false;

    case 20: // SCA two-footstep planning

        if (getPhase() == 0 && (index.point != 0 && index.point != num_points_ - 1))
            return false;

        if (getPhase() != 0)
        {
            if (index.sub_component == ItompTrajectory::SUB_COMPONENT_TYPE_CONTACT_POSITION)
            {
                if (index.point == 0 || index.point == num_points_ - 1)
                    return false;
            }
            else if (index.sub_component == ItompTrajectory::SUB_COMPONENT_TYPE_CONTACT_FORCE)
            {
                //if (index.element % 3 != 2)
                  //  return false;

                int parameter_point = index.point / 5;
                if (parameter_point % 4 != 0)
                {
                    if (4 < parameter_point) // left contact
                    {
                        if (support_foot_ & 0x1) // left foot can be support foot
                        {
                            if (index.element < 12)
                            {
                                return false;
                            }
                        }
                        else
                        {
                            if (index.element >= 12 && index.element < 24)
                            {
                                return false;
                            }
                        }
                    }
                    else // right contact
                    {
                        if (support_foot_ & 0x1)
                        {
                            if (index.element >= 12 && index.element < 24)
                            {
                                return false;
                            }
                        }
                        else
                        {
                            if (index.element < 12)
                            {
                                return false;
                            }
                        }
                    }
                }
            }
        }

        break;
    }





    if (state != 0)
    {
        if (PhaseManager::getInstance()->getPhase() == 0 &&
                (index.point == 0 || (index.point == num_points_ - 1 && index.element >= 3)) &&
                index.sub_component == ItompTrajectory::SUB_COMPONENT_TYPE_JOINT)
            return false;

        if (PhaseManager::getInstance()->getPhase() != 0 &&
                (index.point == 0 || (index.point == num_points_ - 1 && index.element < 2)) &&
                index.sub_component == ItompTrajectory::SUB_COMPONENT_TYPE_JOINT)
            return false;
    }



    return true;
}

}
