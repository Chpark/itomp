#ifndef TRAJECTORY_H_
#define TRAJECTORY_H_

#include <itomp_cio_planner/common.h>
#include <itomp_cio_planner/model/itomp_robot_model.h>
#include <itomp_cio_planner/model/itomp_planning_group.h>
#include <itomp_cio_planner/util/differentiation_rules.h>
#include <ros/assert.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <kdl/jntarray.hpp>

#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/random/mersenne_twister.hpp>

namespace itomp_cio_planner
{
ITOMP_FORWARD_DECL(Trajectory);

class Trajectory
{
public:

	enum TRAJECTORY_TYPE
	{
		TRAJECTORY_TYPE_POSITION = 0,
		TRAJECTORY_TYPE_VELOCITY,
		TRAJECTORY_TYPE_ACCELERATION,
		TRAJECTORY_TYPE_NUM,
	};

	// Construct a trajectory
	Trajectory(double discretization,
			   bool has_velocity = false, bool has_acceleration = false);

	virtual ~Trajectory();

	double& operator()(int traj_point, int element, TRAJECTORY_TYPE type =
						   TRAJECTORY_TYPE_POSITION);
	double operator()(int traj_point, int element, TRAJECTORY_TYPE type =
						  TRAJECTORY_TYPE_POSITION) const;

	Eigen::MatrixXd::RowXpr getTrajectoryPoint(int traj_point,
			TRAJECTORY_TYPE type = TRAJECTORY_TYPE_POSITION);
	Eigen::MatrixXd::ConstRowXpr getTrajectoryPoint(int traj_point,
			TRAJECTORY_TYPE type = TRAJECTORY_TYPE_POSITION) const;

	double getDiscretization() const;
	bool hasVelocity() const;
	bool hasAcceleration() const;
	double getDuration() const;
	int getNumPoints() const;
	int getNumElements() const;

	Eigen::MatrixXd& getTrajectory(TRAJECTORY_TYPE type =
									   TRAJECTORY_TYPE_POSITION);
	const Eigen::MatrixXd& getTrajectory(TRAJECTORY_TYPE type =
			TRAJECTORY_TYPE_POSITION) const;
	void setTrajectory(const Eigen::MatrixXd& trajectory, TRAJECTORY_TYPE type =
						   TRAJECTORY_TYPE_POSITION);

	virtual void printTrajectory(bool position = true, bool velocity = true, bool acceleration = true) const;

protected:
	virtual void allocate(); /**< \brief Allocates memory for the trajectory */

	double discretization_; /**< Discretization of the trajectory */
	bool has_velocity_;
	bool has_acceleration_;

	double duration_; /**< Duration of the trajectory */
	int num_elements_; /**< Number of elements in each trajectory point */
	int num_points_; /**< Number of points in the trajectory */

	Eigen::MatrixXd trajectory_[TRAJECTORY_TYPE_NUM]; /**< Storage for the actual trajectory */

};
ITOMP_DEFINE_SHARED_POINTERS(Trajectory);

///////////////////////// inline functions follow //////////////////////

inline double& Trajectory::operator()(int traj_point, int element,
									  TRAJECTORY_TYPE type)
{
	return trajectory_[type](traj_point, element);
}

inline double Trajectory::operator()(int traj_point, int element,
									 TRAJECTORY_TYPE type) const
{
	return trajectory_[type](traj_point, element);
}

inline Eigen::MatrixXd::RowXpr Trajectory::getTrajectoryPoint(int traj_point,
		TRAJECTORY_TYPE type)
{
	return trajectory_[type].row(traj_point);
}

inline Eigen::MatrixXd::ConstRowXpr Trajectory::getTrajectoryPoint(
	int traj_point, TRAJECTORY_TYPE type) const
{
	return trajectory_[type].row(traj_point);
}

inline int Trajectory::getNumPoints() const
{
	return num_points_;
}

inline int Trajectory::getNumElements() const
{
	return num_elements_;
}

inline double Trajectory::getDiscretization() const
{
	return discretization_;
}

inline double Trajectory::getDuration() const
{
	return duration_;
}

inline bool Trajectory::hasVelocity() const
{
	return has_velocity_;
}

inline bool Trajectory::hasAcceleration() const
{
	return has_acceleration_;
}

inline Eigen::MatrixXd& Trajectory::getTrajectory(TRAJECTORY_TYPE type)
{
	return trajectory_[type];
}

inline const Eigen::MatrixXd& Trajectory::getTrajectory(
	TRAJECTORY_TYPE type) const
{
	return trajectory_[type];
}

inline void Trajectory::setTrajectory(const Eigen::MatrixXd& trajectory,
									  TRAJECTORY_TYPE type)
{
	trajectory_[type] = trajectory;
}

}
#endif
