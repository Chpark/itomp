#ifndef COMPOSITE_TRAJECTORY_H_
#define COMPOSITE_TRAJECTORY_H_

#include <itomp_cio_planner/common.h>
#include <itomp_cio_planner/trajectory/new_trajectory.h>

namespace itomp_cio_planner
{
ITOMP_FORWARD_DECL(CompositeTrajectory)

class CompositeTrajectory : public NewTrajectory
{
public:
    // Construct a trajectory
    CompositeTrajectory(const std::string& name, unsigned int num_points, const std::vector<NewTrajectoryPtr>& components);
    CompositeTrajectory(const CompositeTrajectory& trajectory);

    virtual ~CompositeTrajectory();
    virtual CompositeTrajectory* clone() const;

    NewTrajectoryPtr& getComponent(unsigned int index);
    NewTrajectoryConstPtr getComponent(unsigned int index) const;
    unsigned int getNumComponents() const;

    virtual void printTrajectory() const;

protected:
    std::vector<NewTrajectoryPtr> trajectories_;
};
ITOMP_DEFINE_SHARED_POINTERS(CompositeTrajectory)

///////////////////////// inline functions follow //////////////////////

inline NewTrajectoryPtr& CompositeTrajectory::getComponent(unsigned int index)
{
    return trajectories_[index];
}

inline NewTrajectoryConstPtr CompositeTrajectory::getComponent(unsigned int index) const
{
    return boost::const_pointer_cast<const NewTrajectory>(trajectories_[index]);
}

inline unsigned int CompositeTrajectory::getNumComponents() const
{
    return trajectories_.size();
}

}
#endif
