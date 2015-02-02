#include <itomp_cio_planner/trajectory/element_trajectory.h>
#include <ros/assert.h>

using namespace std;

namespace itomp_cio_planner
{

ElementTrajectory::ElementTrajectory(const std::string& name, unsigned int num_points, unsigned int num_elements)
    : NewTrajectory(name, num_points)
{
    num_elements_ = num_elements;
    allocate();
}

ElementTrajectory::~ElementTrajectory()
{

}

void ElementTrajectory::allocate()
{
    ROS_ASSERT(num_points_ != 0 && num_elements_ != 0);

    trajectory_data_ = Eigen::MatrixXd(num_points_,
                                       num_elements_);
    trajectory_data_.setZero(num_points_, num_elements_);
}

void ElementTrajectory::printTrajectory() const
{
    ROS_INFO("Trajectory %s", name_.c_str());
    for (int i = 0; i < num_points_; ++i)
    {
        ROS_INFO("%d : ", i);
        for (int j = 0; j < num_elements_; ++j)
        {
            ROS_INFO("%.10f ", trajectory_data_(i, j));
        }
        ROS_INFO("\n");
    }
}

}
