#ifndef PHASE_MANAGER_H_
#define PHASE_MANAGER_H_

#include <itomp_cio_planner/common.h>

namespace itomp_cio_planner
{

class PhaseManager : public Singleton<PhaseManager>
{
public:
    PhaseManager();
    virtual ~PhaseManager();

    unsigned int getPhase() const;
    void setPhase(unsigned int phase);

private:
    unsigned int phase_;
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
