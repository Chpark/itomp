#ifndef PHASE_MANAGER_H_
#define PHASE_MANAGER_H_

#include <itomp_cio_planner/common.h>
#include <itomp_cio_planner/trajectory/itomp_trajectory.h>

namespace itomp_cio_planner
{

class PhaseManager : public Singleton<PhaseManager>
{
public:
    PhaseManager();
    virtual ~PhaseManager();

    void init(int num_points, const ItompPlanningGroupConstPtr& planning_group);

    unsigned int getPhase() const;
    void setPhase(unsigned int phase);

    bool updateParameter(const ItompTrajectoryIndex& index) const;

    int agent_id_;
    int support_foot_;
    Eigen::Vector3d initial_goal_pos;

private:
    unsigned int phase_;
    int num_points_;
    ItompPlanningGroupConstPtr planning_group_;
};

inline unsigned int PhaseManager::getPhase() const
{
    return phase_;
}

inline void PhaseManager::setPhase(unsigned int phase)
{
    phase_ = phase;
}

}

#endif
