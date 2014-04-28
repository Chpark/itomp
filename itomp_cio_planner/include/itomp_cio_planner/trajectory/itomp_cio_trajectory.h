#ifndef ITOMP_CIO_TRAJECTORY_H
#define ITOMP_CIO_TRAJECTORY_H

#include <itomp_cio_planner/common.h>
#include <itomp_cio_planner/model/itomp_robot_model.h>
#include <itomp_cio_planner/model/itomp_planning_group.h>
#include <itomp_cio_planner/util/differentiation_rules.h>

#include <trajectory_msgs/JointTrajectory.h>

#include <kdl/jntarray.hpp>

#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/random/mersenne_twister.hpp>

namespace itomp_cio_planner
{

/**
 * \brief Represents a discretized joint-space trajectory
 */
class ItompCIOTrajectory
{
public:
	/**
	 * \brief Constructs a trajectory for a given robot model, trajectory duration, and discretization
	 */
	ItompCIOTrajectory(const ItompRobotModel* robot_model, double duration, double discretization, double num_contacts,
			double contact_phase_duration);

	/**
	 * \brief Creates a new containing only the joints of interest, and adds padding to the start
	 * and end if needed, to have enough trajectory points for the differentiation rules
	 */
	ItompCIOTrajectory(const ItompCIOTrajectory& source_traj, const ItompPlanningGroup* planning_group,
			int diff_rule_length);

	/**
	 * \brief Destructor
	 */
	virtual ~ItompCIOTrajectory();

	double& operator()(int traj_point, int joint);

	double operator()(int traj_point, int joint) const;
	double getContactValue(int phase, int contact) const;

	int getContactPhase(int traj_point) const;
	int getContactPhaseStartPoint(int traj_point) const;
	int getContactPhaseEndPoint(int traj_point) const;
	Eigen::MatrixXd::RowXpr getTrajectoryPoint(int traj_point);
	Eigen::MatrixXd::RowXpr getContactTrajectoryPoint(int phase);
	void getContactPhaseRange(int contactIndex, int& startPoint, int& endPoint);

	void
	getTrajectoryPointKDL(int traj_point, KDL::JntArray& kdl_jnt_array) const;

	Eigen::MatrixXd::ColXpr getJointTrajectory(int joint);
	Eigen::MatrixXd::ConstColXpr getJointTrajectory(int joint) const;

	Eigen::MatrixXd::ColXpr getContactTrajectory(int contact)
	{
		return contact_trajectory_.col(contact);
	}

	void overwriteTrajectory(const trajectory_msgs::JointTrajectory& traj);

	/**
	 * \brief Gets the number of points in the trajectory
	 */
	int getNumPoints() const;
	int getNumContactPhases() const
	{
		return num_contact_phases_;
	}

	/**
	 * \brief Gets the number of points (that are free to be optimized) in the trajectory
	 */
	int getNumFreePoints() const;
	int getNumFreeContactPhases() const
	{
		return num_contact_phases_ - 2;
	}

	/**
	 * \brief Gets the number of joints in each trajectory point
	 */
	int getNumJoints() const;
	int getNumContacts() const
	{
		return num_contacts_;
	}

	/**
	 * \brief Gets the discretization time interval of the trajectory
	 */
	double getDiscretization() const;

	/**
	 * \brief Generates a minimum jerk trajectory from the start index to end index
	 *
	 * Only modifies points from start_index_ to end_index_, inclusive.
	 */
	void fillInMinJerk(const std::set<int>& groupJointsKDLIndices, const Eigen::MatrixXd::RowXpr joint_vel_array,
			const Eigen::MatrixXd::RowXpr joint_acc_array);
	void fillInMinJerkWithMidPoint(const std::vector<double>& midPoint, const std::set<int>& groupJointsKDLIndices,
			int index);

	/**
	 * \brief Sets the start and end index for the modifiable part of the trajectory
	 *
	 * (Everything before the start and after the end index is considered fixed)
	 * The values default to 1 and getNumPoints()-2
	 */
	void setStartEndIndex(int start_index, int end_index);

	/**
	 * \brief Gets the start index
	 */
	int getStartIndex() const;

	/**
	 * \brief Gets the end index
	 */
	int getEndIndex() const;

	/**
	 * \brief Gets the trajectory duration
	 */
	double getDuration() const;

	/**
	 * \brief Gets the entire trajectory matrix
	 */
	Eigen::MatrixXd& getTrajectory();
	Eigen::MatrixXd& getContactTrajectory();

	/**
	 * \brief Gets the entire trajectory matrix
	 */
	void setTrajectory(Eigen::MatrixXd& trajectory);
	void setContactTrajectory(Eigen::MatrixXd& contact_trajectory);

	/**
	 * \brief Gets the block of the trajectory which can be optimized
	 */
	Eigen::Block<Eigen::MatrixXd, Eigen::Dynamic, Eigen::Dynamic> getFreeTrajectoryBlock();

	/**
	 * \brief Gets the block of free (optimizable) trajectory for a single joint
	 */
	Eigen::Block<Eigen::MatrixXd, Eigen::Dynamic, Eigen::Dynamic> getFreeJointTrajectoryBlock(int joint);
	const Eigen::Block<const Eigen::MatrixXd, Eigen::Dynamic, Eigen::Dynamic> getFreeJointTrajectoryBlock(int joint) const;
	Eigen::Block<Eigen::MatrixXd, Eigen::Dynamic, Eigen::Dynamic> getFreeContactTrajectoryBlock(int contact);
	const Eigen::Block<const Eigen::MatrixXd, Eigen::Dynamic, Eigen::Dynamic> getFreeContactTrajectoryBlock(int contact) const;

	/**
	 * \brief Updates the full trajectory (*this) from the group trajectory
	 */
	void updateFromGroupTrajectory(const ItompCIOTrajectory& group_trajectory);

	/**
	 * \brief Gets the index in the full trajectory which was copied to this group trajectory
	 */
	int getFullTrajectoryIndex(int i) const;

	/**
	 * \brief Gets the joint velocities at the given trajectory point
	 */
	template<typename Derived>
	void getJointVelocities(int traj_point, Eigen::MatrixBase<Derived>& velocities);

	/**
	 * \brief Gets the joint accelerations at the given trajectory point
	 */
	template<typename Derived>
	void getJointAccelerations(int traj_point, Eigen::MatrixBase<Derived>& accelerations);

private:

	void init(); /**< \brief Allocates memory for the trajectory */

	const ItompRobotModel* robot_model_; /**< Robot Model */
	const ItompPlanningGroup* planning_group_; /**< Planning group that this trajectory corresponds to, if any */
	int num_points_; /**< Number of points in the trajectory */
	int num_joints_; /**< Number of joints in each trajectory point */
	double discretization_; /**< Discretization of the trajectory */
	double duration_; /**< Duration of the trajectory */
	Eigen::MatrixXd trajectory_; /**< Storage for the actual trajectory */
	int start_index_; /**< Start index (inclusive) of trajectory to be optimized (everything before it will not be modified) */
	int end_index_; /**< End index (inclusive) of trajectory to be optimized (everything after it will not be modified) */
	std::vector<int> full_trajectory_index_; /**< If this is a "group" trajectory, the index from the original traj which each element here was copied */

	Eigen::MatrixXd joint_vel_array_;
	Eigen::MatrixXd joint_acc_array_;

	// contact variables
	int num_contacts_;
	double contact_phase_duration_;
	int num_contact_phases_;
	Eigen::MatrixXd contact_trajectory_;
	std::vector<int> contact_start_points_;
};

///////////////////////// inline functions follow //////////////////////

inline double& ItompCIOTrajectory::operator()(int traj_point, int joint)
{
	return trajectory_(traj_point, joint);
}

inline double ItompCIOTrajectory::operator()(int traj_point, int joint) const
{
	return trajectory_(traj_point, joint);
}

inline double ItompCIOTrajectory::getContactValue(int phase, int contact) const
{
	return contact_trajectory_(phase, contact);
}

inline Eigen::MatrixXd::RowXpr ItompCIOTrajectory::getTrajectoryPoint(int traj_point)
{
	return trajectory_.row(traj_point);
}

inline Eigen::MatrixXd::RowXpr ItompCIOTrajectory::getContactTrajectoryPoint(int phase)
{
	return contact_trajectory_.row(phase);
}

inline Eigen::MatrixXd::ColXpr ItompCIOTrajectory::getJointTrajectory(int joint)
{
	return trajectory_.col(joint);
}

inline Eigen::MatrixXd::ConstColXpr ItompCIOTrajectory::getJointTrajectory(int joint) const
{
	return trajectory_.col(joint);
}

inline int ItompCIOTrajectory::getNumPoints() const
{
	return num_points_;
}

inline int ItompCIOTrajectory::getNumFreePoints() const
{
	return (end_index_ - start_index_) + 1;
}

inline int ItompCIOTrajectory::getNumJoints() const
{
	return num_joints_;
}

inline double ItompCIOTrajectory::getDiscretization() const
{
	return discretization_;
}

inline void ItompCIOTrajectory::setStartEndIndex(int start_index, int end_index)
{
	start_index_ = start_index;
	end_index_ = end_index;
}

inline int ItompCIOTrajectory::getStartIndex() const
{
	return start_index_;
}

inline int ItompCIOTrajectory::getEndIndex() const
{
	return end_index_;
}

inline double ItompCIOTrajectory::getDuration() const
{
	return duration_;
}

inline Eigen::MatrixXd& ItompCIOTrajectory::getTrajectory()
{
	return trajectory_;
}

inline void ItompCIOTrajectory::setTrajectory(Eigen::MatrixXd& trajectory)
{
	trajectory_ = trajectory;
}

inline Eigen::Block<Eigen::MatrixXd, Eigen::Dynamic, Eigen::Dynamic> ItompCIOTrajectory::getFreeTrajectoryBlock()
{
	return trajectory_.block(start_index_, 0, getNumFreePoints(), getNumJoints());
}

inline Eigen::Block<Eigen::MatrixXd, Eigen::Dynamic, Eigen::Dynamic> ItompCIOTrajectory::getFreeJointTrajectoryBlock(
		int joint)
{
	return trajectory_.block(start_index_, joint, getNumFreePoints(), 1);
}

inline const Eigen::Block<const Eigen::MatrixXd, Eigen::Dynamic, Eigen::Dynamic> ItompCIOTrajectory::getFreeJointTrajectoryBlock(
		int joint) const
{
	return trajectory_.block(start_index_, joint, getNumFreePoints(), 1);
}

inline Eigen::Block<Eigen::MatrixXd, Eigen::Dynamic, Eigen::Dynamic> ItompCIOTrajectory::getFreeContactTrajectoryBlock(
		int contact)
{
	return contact_trajectory_.block(1, contact, getNumFreeContactPhases(), 1);
}

inline const Eigen::Block<const Eigen::MatrixXd, Eigen::Dynamic, Eigen::Dynamic> ItompCIOTrajectory::getFreeContactTrajectoryBlock(
		int contact) const
{
	return contact_trajectory_.block(1, contact, getNumFreeContactPhases(), 1);
}


inline void ItompCIOTrajectory::getTrajectoryPointKDL(int traj_point, KDL::JntArray& kdl_jnt_array) const
{
	for (int i = 0; i < num_joints_; i++)
		kdl_jnt_array(i) = trajectory_(traj_point, i);
}

inline int ItompCIOTrajectory::getFullTrajectoryIndex(int i) const
{
	return full_trajectory_index_[i];
}

template<typename Derived>
void ItompCIOTrajectory::getJointVelocities(int traj_point, Eigen::MatrixBase<Derived>& velocities)
{
	velocities.setZero();
	double invTime = 1.0 / discretization_;

	for (int k = -DIFF_RULE_LENGTH / 2; k <= DIFF_RULE_LENGTH / 2; k++)
	{
		velocities += (invTime * DIFF_RULES[0][k + DIFF_RULE_LENGTH / 2]) * trajectory_.row(traj_point + k).transpose();
	}
}

template<typename Derived>
void ItompCIOTrajectory::getJointAccelerations(int traj_point, Eigen::MatrixBase<Derived>& accelerations)
{
	accelerations.setZero();
	double invTime2 = 1.0 / (discretization_ * discretization_);

	for (int k = -DIFF_RULE_LENGTH / 2; k <= DIFF_RULE_LENGTH / 2; k++)
	{
		accelerations += (invTime2 * DIFF_RULES[1][k + DIFF_RULE_LENGTH / 2])
				* trajectory_.row(traj_point + k).transpose();
	}
}

inline Eigen::MatrixXd& ItompCIOTrajectory::getContactTrajectory()
{
	return contact_trajectory_;
}
inline void ItompCIOTrajectory::setContactTrajectory(Eigen::MatrixXd& contact_trajectory)
{
	contact_trajectory_ = contact_trajectory;
}
inline int ItompCIOTrajectory::getContactPhase(int traj_point) const
{
	for (int i = num_contact_phases_ - 1; i > 0; --i)
	{
		if (traj_point >= contact_start_points_[i])
			return i;
	}
	return 0;
}

inline int ItompCIOTrajectory::getContactPhaseStartPoint(int traj_point) const
{
	return contact_start_points_[getContactPhase(traj_point)];
}

inline int ItompCIOTrajectory::getContactPhaseEndPoint(int traj_point) const
{
	return contact_start_points_[getContactPhase(traj_point) + 1] - 1;
}

inline void ItompCIOTrajectory::getContactPhaseRange(int contactIndex, int& startPoint, int& endPoint)
{
	startPoint = contact_start_points_[contactIndex];
	endPoint = contact_start_points_[contactIndex + 1] - 1;
}

}

#endif
