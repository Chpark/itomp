#include <itomp_cio_planner/trajectory/composite_trajectory.h>
#include <ros/assert.h>

using namespace std;

namespace itomp_cio_planner
{

CompositeTrajectory::CompositeTrajectory(const std::string& name, unsigned int num_points, const std::vector<NewTrajectoryPtr>& components)
    : NewTrajectory(name, num_points), trajectories_(components)
{
    num_elements_ = 0;
    for (unsigned int i = 0; i < getNumComponents(); ++i)
        num_elements_ += getComponent(i)->getNumElements();
}

CompositeTrajectory::CompositeTrajectory(const CompositeTrajectory& trajectory)
    : NewTrajectory(trajectory)
{
    trajectories_.resize(trajectory.getNumComponents());
    for (unsigned int i = 0; i < getNumComponents(); ++i)
    {
        trajectories_[i].reset(trajectory.trajectories_[i]->clone());
    }
}

CompositeTrajectory::~CompositeTrajectory()
{

}

CompositeTrajectory* CompositeTrajectory::clone() const
{
    return new CompositeTrajectory(*this);
}

void CompositeTrajectory::printTrajectory(std::ostream& out_stream) const
{
    out_stream << "Trajectory " << name_ << std::endl;
    for (unsigned int i = 0; i < getNumComponents(); ++i)
    {
        trajectories_[i]->printTrajectory(out_stream);
    }
}

}

