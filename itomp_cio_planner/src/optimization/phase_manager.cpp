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

void PhaseManager::init(int num_points, const ItompPlanningGroupConstPtr& planning_group)
{
    num_points_ = num_points;
    planning_group_ = planning_group;
}

bool PhaseManager::updateParameter(const ItompTrajectoryIndex& index) const
{
    int state = (int)(PlanningParameters::getInstance()->getTemporaryVariable(0) + ITOMP_EPS);

    switch (getPhase())
    {
    case 0:
    {
        return false;
        if (index.component == ItompTrajectory::COMPONENT_TYPE_POSITION)
            if (/*index.point == 0 || */index.point == num_points_ -1)
            return true;

        return false;


    }
        break;
    case 1:
    {
        if (index.component == ItompTrajectory::COMPONENT_TYPE_POSITION)
            if (index.point == 0 || index.point == num_points_ -1)
            return false;

        if (index.sub_component != ItompTrajectory::SUB_COMPONENT_TYPE_JOINT)
            return false;

        if (!planning_group_->is_fixed_[0] && index.element >= 46 && index.element < 54)
            return true;
        if (!planning_group_->is_fixed_[1] && index.element >= 60 && index.element < 68)
            return true;
        if (!planning_group_->is_fixed_[2] && index.element >= 15 && index.element < 20)
            return true;
        if (!planning_group_->is_fixed_[3] && index.element >= 32 && index.element < 37)
            return true;


        return false;
    }
        break;
    case 2:
    {
        //return false;
        if (index.component == ItompTrajectory::COMPONENT_TYPE_POSITION)
            if (index.point == 0 || index.point == num_points_ -1)
            return false;

        if (index.sub_component != ItompTrajectory::SUB_COMPONENT_TYPE_JOINT)
            return false;

        return true;
    }
        break;
    case 3:
    {
        if (index.sub_component == ItompTrajectory::SUB_COMPONENT_TYPE_CONTACT_FORCE)
        {
            if (index.point == 0 || index.point == num_points_ -1)
                return true;

            int contact_id = index.element / 16;
            if (planning_group_->is_fixed_[contact_id])
                return true;
        }
        return false;
    }
        break;
    case 4:
    {
        if (index.component == ItompTrajectory::COMPONENT_TYPE_POSITION)
            if (index.point == 0 || index.point == num_points_ -1)
            return false;

        if (index.sub_component == ItompTrajectory::SUB_COMPONENT_TYPE_CONTACT_POSITION)
            return false;

        if (index.sub_component == ItompTrajectory::SUB_COMPONENT_TYPE_CONTACT_FORCE)
        {
            int contact_id = index.element / 16;
            if (planning_group_->is_fixed_[contact_id])
                return true;

            if (index.point == 0 || index.point == num_points_ -1)
                return true;
            return false;
        }

        return true;
    }
        break;


    }

    return false;
}

}
