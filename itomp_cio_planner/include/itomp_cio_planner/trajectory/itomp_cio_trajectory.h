#ifndef ITOMP_CIO_TRAJECTORY_H
#define ITOMP_CIO_TRAJECTORY_H

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

/**
 * \brief Represents a discretized joint-space trajectory
 */
class ItompCIOTrajectory
{
public:

  enum TRAJECTORY_TYPE
  {
    TRAJECTORY_POSITION = 0, TRAJECTORY_VELOCITY, TRAJECTORY_ACCELERATION, TRAJECTORY_NUM,
  };

  // Construct a full-DOF trajectory
  ItompCIOTrajectory(const ItompRobotModel* robot_model, double duration, double discretization, double num_contacts,
      double contact_phase_duration);
  ItompCIOTrajectory(const ItompRobotModel* robot_model, double duration, double discretization,
      double keyframe_interval = 0.0, bool has_velocity_and_acceleration = false, bool free_end_point = false);

  // Construct a trajectory for a planning group
  ItompCIOTrajectory(const ItompCIOTrajectory& full_trajectory, const ItompPlanningGroup* planning_group,
      int diff_rule_length = 0);

  virtual ~ItompCIOTrajectory();

  double& operator()(int traj_point, int joint, TRAJECTORY_TYPE type = TRAJECTORY_POSITION);
  double operator()(int traj_point, int joint, TRAJECTORY_TYPE type = TRAJECTORY_POSITION) const;

  Eigen::MatrixXd::RowXpr getTrajectoryPoint(int traj_point, TRAJECTORY_TYPE type = TRAJECTORY_POSITION);
  Eigen::MatrixXd::ConstRowXpr getTrajectoryPoint(int traj_point, TRAJECTORY_TYPE type = TRAJECTORY_POSITION) const;
  Eigen::MatrixXd::ColXpr getJointTrajectory(int joint, TRAJECTORY_TYPE type = TRAJECTORY_POSITION);
  Eigen::MatrixXd::ConstColXpr getJointTrajectory(int joint, TRAJECTORY_TYPE type = TRAJECTORY_POSITION) const;

  int getNumPoints() const;
  int getNumJoints() const;
  double getDiscretization() const;
  double getDuration() const;

  // group trajectory index to full trajectory index map
  int getFullTrajectoryIndex(int i) const;

  Eigen::MatrixXd& getTrajectory(TRAJECTORY_TYPE type = TRAJECTORY_POSITION);
  const Eigen::MatrixXd& getTrajectory(TRAJECTORY_TYPE type = TRAJECTORY_POSITION) const;
  void setTrajectory(const Eigen::MatrixXd& trajectory, TRAJECTORY_TYPE type = TRAJECTORY_POSITION);

  void copyFromGroupTrajectory(const ItompCIOTrajectory& group_trajectory);
  void copyFromGroupTrajectory(const ItompCIOTrajectory& group_trajectory, int point_index, int joint_index);
  void copyFromFullTrajectory(const ItompCIOTrajectory& full_trajectory);

  Eigen::MatrixXd& getKeyframes(TRAJECTORY_TYPE type = TRAJECTORY_POSITION);
  const Eigen::MatrixXd& getKeyframes(TRAJECTORY_TYPE type = TRAJECTORY_POSITION) const;
  void setKeyframes(const Eigen::MatrixXd& trajectory, TRAJECTORY_TYPE type = TRAJECTORY_POSITION);

  void updateTrajectoryFromKeyframes();

  /**
   * \brief Generates a minimum jerk trajectory from the start index to end index
   *
   */
  void fillInMinJerk(const std::set<int>& groupJointsKDLIndices, const Eigen::MatrixXd::RowXpr joint_vel_array,
      const Eigen::MatrixXd::RowXpr joint_acc_array);
  void fillInMinJerkCartesianTrajectory(const std::set<int>& groupJointsKDLIndices,
      const Eigen::MatrixXd::RowXpr joint_vel_array, const Eigen::MatrixXd::RowXpr joint_acc_array,
      const moveit_msgs::Constraints& path_constraints, const std::string& group_name);

  void printTrajectory() const;

  // TODO: remove
  Eigen::MatrixXd::RowXpr getContactTrajectoryPoint(int phase);

  void getTrajectoryPointKDL(int traj_point, KDL::JntArray& kdl_jnt_array) const;

  Eigen::MatrixXd::ColXpr getContactTrajectory(int contact);

  Eigen::MatrixXd& getContactTrajectory();
  const Eigen::MatrixXd& getContactTrajectory() const;

  void setContactTrajectory(Eigen::MatrixXd& contact_trajectory);

  Eigen::MatrixXd& getFreePoints();
  const Eigen::MatrixXd& getFreePoints() const;
  Eigen::MatrixXd& getFreeVelPoints();
  const Eigen::MatrixXd& getFreeVelPoints() const;
  void updateTrajectoryFromFreePoints();
  void updateTrajectoryFromFreePoint(int point_index, int joint_index);
  void updateFreePointsFromTrajectory();

  Eigen::Block<Eigen::MatrixXd, Eigen::Dynamic, Eigen::Dynamic> getFreeTrajectoryBlock();
  Eigen::Block<Eigen::MatrixXd, Eigen::Dynamic, Eigen::Dynamic> getFreeJointTrajectoryBlock(int joint);
  const Eigen::Block<const Eigen::MatrixXd, Eigen::Dynamic, Eigen::Dynamic> getFreeJointTrajectoryBlock(
      int joint) const;
  Eigen::Block<Eigen::MatrixXd, Eigen::Dynamic, Eigen::Dynamic> getFreeContactTrajectoryBlock(int contact);
  const Eigen::Block<const Eigen::MatrixXd, Eigen::Dynamic, Eigen::Dynamic> getFreeContactTrajectoryBlock(
      int contact) const;

  int getNumFreePoints() const;

  // contact functions
  double getContactValue(int phase, int contact) const;
  int getNumContactPhases() const;
  int getNumContacts() const;
  int getContactPhaseStride() const;
  int getContactPhase(int traj_point) const;
  int getContactPhaseStartPoint(int traj_point) const;
  int getContactPhaseEndPoint(int traj_point) const;

private:
  void init(); /**< \brief Allocates memory for the trajectory */

  bool hasKeyframes() const;
  void copyKeyframesFromTrajectory();

  bool is_full_trajectory_;

  const ItompRobotModel* robot_model_; /**< Robot Model */
  const ItompPlanningGroup* planning_group_; /**< Planning group that this trajectory corresponds to, if any */

  double discretization_; /**< Discretization of the trajectory */
  bool has_velocity_and_acceleration_;
  bool has_free_end_point_;
  int num_joints_; /**< Number of joints in each trajectory point */
  int keyframe_start_index_;
  int num_keyframe_interval_points_;
  double keyframe_interval_;
  int num_keyframes_;
  int num_points_; /**< Number of points in the trajectory */
  double duration_; /**< Duration of the trajectory */
  int start_index_;
  int end_index_;

  Eigen::MatrixXd trajectory_[TRAJECTORY_NUM]; /**< Storage for the actual trajectory */
  Eigen::MatrixXd keyframes_[2]; // No accelerations

  std::vector<int> full_trajectory_index_;

  // TODO: remove
  Eigen::MatrixXd free_trajectory_;
  Eigen::MatrixXd free_vel_trajectory_;

  Eigen::MatrixXd contact_trajectory_;

  Eigen::VectorXd vel_start_;
  Eigen::VectorXd acc_start_;

  // contact variables
  int num_contacts_;
  double contact_phase_duration_;
  int num_contact_phases_;
  int phase_stride_;
  std::vector<int> contact_start_points_;
};

typedef boost::shared_ptr<ItompCIOTrajectory> ItompCIOTrajectoryPtr;

///////////////////////// inline functions follow //////////////////////

inline double& ItompCIOTrajectory::operator()(int traj_point, int joint, TRAJECTORY_TYPE type)
{
  return trajectory_[type](traj_point, joint);
}

inline double ItompCIOTrajectory::operator()(int traj_point, int joint, TRAJECTORY_TYPE type) const
{
  return trajectory_[type](traj_point, joint);
}

inline Eigen::MatrixXd::RowXpr ItompCIOTrajectory::getTrajectoryPoint(int traj_point, TRAJECTORY_TYPE type)
{
  return trajectory_[type].row(traj_point);
}

inline Eigen::MatrixXd::ConstRowXpr ItompCIOTrajectory::getTrajectoryPoint(int traj_point, TRAJECTORY_TYPE type) const
{
  return trajectory_[type].row(traj_point);
}

inline Eigen::MatrixXd::ColXpr ItompCIOTrajectory::getJointTrajectory(int joint, TRAJECTORY_TYPE type)
{
  return trajectory_[type].col(joint);
}

inline Eigen::MatrixXd::ConstColXpr ItompCIOTrajectory::getJointTrajectory(int joint, TRAJECTORY_TYPE type) const
{
  return trajectory_[type].col(joint);
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

inline int ItompCIOTrajectory::getFullTrajectoryIndex(int i) const
{
  return full_trajectory_index_[i];
}

inline Eigen::MatrixXd& ItompCIOTrajectory::getTrajectory(TRAJECTORY_TYPE type)
{
  return trajectory_[type];
}

inline const Eigen::MatrixXd& ItompCIOTrajectory::getTrajectory(TRAJECTORY_TYPE type) const
{
  return trajectory_[type];
}

inline void ItompCIOTrajectory::setTrajectory(const Eigen::MatrixXd& trajectory, TRAJECTORY_TYPE type)
{
  trajectory_[type] = trajectory;
}

inline Eigen::MatrixXd& ItompCIOTrajectory::getKeyframes(TRAJECTORY_TYPE type)
{
  ROS_ASSERT(!is_full_trajectory_);
  return hasKeyframes() ? keyframes_[type] : trajectory_[type];
}

inline const Eigen::MatrixXd& ItompCIOTrajectory::getKeyframes(TRAJECTORY_TYPE type) const
{
  ROS_ASSERT(!is_full_trajectory_);
  return hasKeyframes() ? keyframes_[type] : trajectory_[type];
}

inline void ItompCIOTrajectory::setKeyframes(const Eigen::MatrixXd& keyframe, TRAJECTORY_TYPE type)
{
  ROS_ASSERT(!is_full_trajectory_);
  if (hasKeyframes())
    keyframes_[type] = keyframe;
  else
    trajectory_[type] = keyframe;
}

inline bool ItompCIOTrajectory::hasKeyframes() const
{
  return num_keyframe_interval_points_ > 1;
}

// TODO: delete below

inline void ItompCIOTrajectory::getTrajectoryPointKDL(int traj_point, KDL::JntArray& kdl_jnt_array) const
{
  for (int i = 0; i < num_joints_; i++)
    kdl_jnt_array(i) = trajectory_[0](traj_point, i);
}

inline Eigen::MatrixXd& ItompCIOTrajectory::getContactTrajectory()
{
  return contact_trajectory_;
}

inline const Eigen::MatrixXd& ItompCIOTrajectory::getContactTrajectory() const
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

inline const Eigen::MatrixXd& ItompCIOTrajectory::getFreePoints() const
{
  return free_trajectory_;
}

inline Eigen::MatrixXd& ItompCIOTrajectory::getFreeVelPoints()
{
  return free_vel_trajectory_;
}

inline const Eigen::MatrixXd& ItompCIOTrajectory::getFreeVelPoints() const
{
  return free_vel_trajectory_;
}

inline int ItompCIOTrajectory::getNumFreePoints() const
{
  return (end_index_ - start_index_) + 1;
}

inline Eigen::Block<Eigen::MatrixXd, Eigen::Dynamic, Eigen::Dynamic> ItompCIOTrajectory::getFreeTrajectoryBlock()
{
  return trajectory_[0].block(start_index_, 0, getNumFreePoints(), getNumJoints());
}

inline Eigen::Block<Eigen::MatrixXd, Eigen::Dynamic, Eigen::Dynamic> ItompCIOTrajectory::getFreeJointTrajectoryBlock(
    int joint)
{
  return trajectory_[0].block(start_index_, joint, getNumFreePoints(), 1);
}

inline const Eigen::Block<const Eigen::MatrixXd, Eigen::Dynamic, Eigen::Dynamic> ItompCIOTrajectory::getFreeJointTrajectoryBlock(
    int joint) const
{
  return trajectory_[0].block(start_index_, joint, getNumFreePoints(), 1);
}

inline Eigen::Block<Eigen::MatrixXd, Eigen::Dynamic, Eigen::Dynamic> ItompCIOTrajectory::getFreeContactTrajectoryBlock(
    int contact)
{
  return contact_trajectory_.block(1, contact, getNumContactPhases() - 1, 1);
}

inline const Eigen::Block<const Eigen::MatrixXd, Eigen::Dynamic, Eigen::Dynamic> ItompCIOTrajectory::getFreeContactTrajectoryBlock(
    int contact) const
{
  return contact_trajectory_.block(1, contact, getNumContactPhases() - 1, 1);
}

inline int ItompCIOTrajectory::getContactPhaseStartPoint(int traj_point) const
{
  return contact_start_points_[getContactPhase(traj_point)];
}

inline int ItompCIOTrajectory::getContactPhaseEndPoint(int traj_point) const
{
  return contact_start_points_[getContactPhase(traj_point) + 1] - 1;
}


inline double ItompCIOTrajectory::getContactValue(int phase, int contact) const
{
  return contact_trajectory_(phase, contact);
}

inline Eigen::MatrixXd::RowXpr ItompCIOTrajectory::getContactTrajectoryPoint(int phase)
{
  return contact_trajectory_.row(phase);
}


inline int ItompCIOTrajectory::getContactPhaseStride() const
{
  return phase_stride_;
}


}

#endif
