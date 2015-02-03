#include <itomp_cio_planner/trajectory/new_trajectory.h>

using namespace std;

namespace itomp_cio_planner
{

NewTrajectory::NewTrajectory(const std::string& name, unsigned int num_points)
    : name_(name), num_points_(num_points)
{

}

NewTrajectory::NewTrajectory(const NewTrajectory& trajectory)
    : name_(trajectory.name_),
      num_points_(trajectory.num_points_),
      num_elements_(trajectory.num_elements_)
{

}

NewTrajectory::~NewTrajectory()
{

}

}
