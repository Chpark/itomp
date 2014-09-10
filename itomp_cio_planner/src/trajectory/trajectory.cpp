#include <itomp_cio_planner/common.h>
#include <itomp_cio_planner/trajectory/trajectory.h>
#include <itomp_cio_planner/util/planning_parameters.h>

using namespace std;

namespace itomp_cio_planner
{

Trajectory::Trajectory(double discretization, bool has_velocity,
		bool has_acceleration) :
		discretization_(discretization), has_velocity_(has_velocity), has_acceleration_(
				has_acceleration), duration_(0.0), num_points_(0), num_elements_(
				0)
{

}

Trajectory::~Trajectory()
{

}

void Trajectory::allocate()
{
	ROS_ASSERT(num_points_ != 0 && num_elements_ != 0);

	trajectory_[TRAJECTORY_TYPE_POSITION] = Eigen::MatrixXd(num_points_,
			num_elements_);
	trajectory_[TRAJECTORY_TYPE_POSITION].setZero(num_points_, num_elements_);

	if (has_velocity_)
	{
		trajectory_[TRAJECTORY_TYPE_VELOCITY] = Eigen::MatrixXd(num_points_,
				num_elements_);
		trajectory_[TRAJECTORY_TYPE_VELOCITY].setZero(num_points_, num_elements_);
	}

	if (has_acceleration_)
	{
		trajectory_[TRAJECTORY_TYPE_ACCELERATION] = Eigen::MatrixXd(num_points_,
				num_elements_);
		trajectory_[TRAJECTORY_TYPE_ACCELERATION].setZero(num_points_, num_elements_);
	}
}

void Trajectory::printTrajectory() const
{
	printf("Position Trajectory\n");
	for (int i = 0; i < num_points_; ++i)
	{
		printf("%d : ", i);
		for (int j = 0; j < num_elements_; ++j)
		{
			printf("%f ", trajectory_[TRAJECTORY_TYPE_POSITION](i, j));
		}
		printf("\n");
	}

	if (has_velocity_)
	{
		printf("Velocity Trajectory\n");
		for (int i = 0; i < num_points_; ++i)
		{
			printf("%d : ", i);
			for (int j = 0; j < num_elements_; ++j)
			{
				printf("%f ", trajectory_[TRAJECTORY_TYPE_VELOCITY](i, j));
			}
			printf("\n");
		}
	}
	if (has_acceleration_)
	{
		printf("Acceleration Trajectory\n");
		for (int i = 0; i < num_points_; ++i)
		{
			printf("%d : ", i);
			for (int j = 0; j < num_elements_; ++j)
			{
				printf("%f ", trajectory_[TRAJECTORY_TYPE_ACCELERATION](i, j));
			}
			printf("\n");
		}
	}
}

}
