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
    int state = (int)(PlanningParameters::getInstance()->getTemporaryVariable(0) + ITOMP_EPS);

    switch (getPhase())
    {
    case 0:
    {

        if (index.sub_component == ItompTrajectory::SUB_COMPONENT_TYPE_CONTACT_FORCE)
            return true;
        return false;

    }
        break;
    case 1:
    {
        return false;
    }
        break;
    case 2:
    {
        if (index.component == ItompTrajectory::COMPONENT_TYPE_POSITION)
             if (index.point == 0 || index.point == num_points_ -1)
                return false;

        if (index.sub_component == ItompTrajectory::SUB_COMPONENT_TYPE_CONTACT_POSITION)
        {
            return false;
        }

        if (index.sub_component == ItompTrajectory::SUB_COMPONENT_TYPE_CONTACT_FORCE)
        {
            if (index.point > 4 && index.point < num_points_ - 1 - 4)
                if (index.element < 12 || (index.element >= 24 && index.element < 36))
                    return false;
        }
    }
        break;
    }

    return true;
}

}
