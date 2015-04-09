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

ElementTrajectory::ElementTrajectory(const ElementTrajectory& trajectory)
    : NewTrajectory(trajectory),
      trajectory_data_(trajectory.trajectory_data_)
{

}

ElementTrajectory::~ElementTrajectory()
{

}

ElementTrajectory* ElementTrajectory::clone() const
{
    return new ElementTrajectory(*this);
}

void ElementTrajectory::allocate()
{
    ROS_ASSERT(num_points_ != 0 && num_elements_ != 0);

    trajectory_data_ = Eigen::MatrixXd(num_points_,
                                       num_elements_);
    trajectory_data_.setZero(num_points_, num_elements_);
}

void ElementTrajectory::printTrajectory(std::ostream& out_stream, int point_start, int point_end) const
{
    out_stream.precision(std::numeric_limits<double>::digits10);
    out_stream << "Trajectory " << name_ << std::endl;
    out_stream << "x : ";
    for (int i = 0; i < num_elements_; ++i)
        out_stream << i << " ";
    out_stream << std::endl;
    int end = (point_end < 0) ? num_points_ : point_end;
    for (int i = point_start; i < end; ++i)
    {
        out_stream << i << " : ";
        for (int j = 0; j < num_elements_; ++j)
        {
            out_stream << fixed << trajectory_data_(i, j) << " ";
        }
        out_stream << std::endl;
    }
}

}
