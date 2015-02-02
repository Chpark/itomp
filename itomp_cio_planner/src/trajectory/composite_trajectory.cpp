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

CompositeTrajectory::~CompositeTrajectory()
{

}

void CompositeTrajectory::printTrajectory() const
{
    ROS_INFO("Trajectory %s", name_.c_str());
    for (unsigned int i = 0; i < getNumComponents(); ++i)
    {
        trajectories_[i]->printTrajectory();
    }
}

}

