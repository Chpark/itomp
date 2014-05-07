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
  ItompCIOTrajectory(const ItompCIOTrajectory& source_traj, const ItompPlanningGroup* planning_group,
      int diff_rule_length);

  virtual ~ItompCIOTrajectory();

  double& operator()(int traj_point, int joint);
  double operator()(int traj_point, int joint) const;
  double getContactValue(int phase, int contact) const;

  int getContactPhase(int traj_point) const;
  Eigen::MatrixXd::RowXpr getTrajectoryPoint(int traj_point);
  Eigen::MatrixXd::RowXpr getContactTrajectoryPoint(int phase);
  void getContactPhaseRange(int contact_index, int& start_point, int& end_point);

  void getTrajectoryPointKDL(int traj_point, KDL::JntArray& kdl_jnt_array) const;

  Eigen::MatrixXd::ColXpr getJointTrajectory(int joint);
  Eigen::MatrixXd::ConstColXpr getJointTrajectory(int joint) const;
  Eigen::MatrixXd::ColXpr getContactTrajectory(int contact);

  int getNumPoints() const;
  int getNumContactPhases() const;
  int getNumJoints() const;
  int getNumContacts() const;
  int getContactPhaseStride() const;

  double getDiscretization() const;
  double getDuration() const;

  /**
   * \brief Generates a minimum jerk trajectory from the start index to end index
   *
   */
  void fillInMinJerk(const std::set<int>& groupJointsKDLIndices, const Eigen::MatrixXd::RowXpr joint_vel_array,
      const Eigen::MatrixXd::RowXpr joint_acc_array);
  void fillInMinJerkWithMidPoint(const std::vector<double>& midPoint, const std::set<int>& groupJointsKDLIndices,
      int index);

  Eigen::MatrixXd& getTrajectory();
  Eigen::MatrixXd& getContactTrajectory();

  void setTrajectory(Eigen::MatrixXd& trajectory);
  void setContactTrajectory(Eigen::MatrixXd& contact_trajectory);

  void updateFromGroupTrajectory(const ItompCIOTrajectory& group_trajectory);

  Eigen::MatrixXd& getFreePoints();
  Eigen::MatrixXd& getFreeVelPoints();
  void updateTrajectoryFromFreePoints();
  void updateFreePointsFromTrajectory();

  void printTrajectory();

private:
  void init(); /**< \brief Allocates memory for the trajectory */

  const ItompRobotModel* robot_model_; /**< Robot Model */
  const ItompPlanningGroup* planning_group_; /**< Planning group that this trajectory corresponds to, if any */
  int phase_stride_;
  int num_contact_phases_;
  int num_points_; /**< Number of points in the trajectory */
  int num_joints_; /**< Number of joints in each trajectory point */
  double discretization_; /**< Discretization of the trajectory */
  double duration_; /**< Duration of the trajectory */
  Eigen::MatrixXd trajectory_; /**< Storage for the actual trajectory */

  Eigen::MatrixXd free_trajectory_;
  Eigen::MatrixXd free_vel_trajectory_;

  // contact variables
  int num_contacts_;
  double contact_phase_duration_;

  Eigen::MatrixXd contact_trajectory_;

  Eigen::VectorXd vel_start_;
  Eigen::VectorXd acc_start_;
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

inline int ItompCIOTrajectory::getNumJoints() const
{
  return num_joints_;
}

inline double ItompCIOTrajectory::getDiscretization() const
{
  return discretization_;
}

inline double ItompCIOTrajectory::getDuration() const
{
  return duration_;
}

inline int ItompCIOTrajectory::getContactPhaseStride() const
{
  return phase_stride_;
}

inline Eigen::MatrixXd& ItompCIOTrajectory::getTrajectory()
{
  return trajectory_;
}

inline void ItompCIOTrajectory::setTrajectory(Eigen::MatrixXd& trajectory)
{
  trajectory_ = trajectory;
}

inline void ItompCIOTrajectory::getTrajectoryPointKDL(int traj_point, KDL::JntArray& kdl_jnt_array) const
{
  for (int i = 0; i < num_joints_; i++)
    kdl_jnt_array(i) = trajectory_(traj_point, i);
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
  return traj_point / phase_stride_;
}

inline void ItompCIOTrajectory::getContactPhaseRange(int contact_index, int& start_point, int& end_point)
{
  start_point = contact_index * phase_stride_;
  end_point = start_point + phase_stride_;
}

inline Eigen::MatrixXd::ColXpr ItompCIOTrajectory::getContactTrajectory(int contact)
{
  return contact_trajectory_.col(contact);
}

inline int ItompCIOTrajectory::getNumContactPhases() const
{
  return num_contact_phases_;
}

inline int ItompCIOTrajectory::getNumContacts() const
{
  return num_contacts_;
}

inline Eigen::MatrixXd& ItompCIOTrajectory::getFreePoints()
{
  return free_trajectory_;
}

inline Eigen::MatrixXd& ItompCIOTrajectory::getFreeVelPoints()
{
  return free_vel_trajectory_;
}

}

#endif
