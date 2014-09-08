#include <itomp_cio_planner/common.h>
#include <itomp_cio_planner/trajectory/itomp_cio_trajectory.h>
#include <itomp_cio_planner/model/itomp_robot_model.h>
#include <itomp_cio_planner/model/itomp_planning_group.h>
#include <itomp_cio_planner/util/planning_parameters.h>
#include <ros/console.h>
#include <ecl/geometry.hpp>

using namespace std;

namespace itomp_cio_planner
{

inline int safeToInt(double a)
{
  return (int) (a + 1E-7);
}

ItompCIOTrajectory::ItompCIOTrajectory(const ItompRobotModel* robot_model, double duration, double discretization,
    double keyframe_interval, bool has_velocity_and_acceleration, bool free_end_point) :
    is_full_trajectory_(true), robot_model_(robot_model), planning_group_(NULL), discretization_(discretization), has_velocity_and_acceleration_(
        has_velocity_and_acceleration), has_free_end_point_(has_free_end_point_), num_joints_(
        robot_model_->getNumJoints())
{
  keyframe_start_index_ = 0;

  // no keyframe
  if (keyframe_interval <= discretization)
  {
    num_keyframe_interval_points_ = 1;
    num_keyframes_ = num_points_ = safeToInt(duration / discretization) + 1;
    duration_ = (num_points_ - 1) * discretization;
  }
  else
  {
    num_keyframe_interval_points_ = safeToInt(keyframe_interval / discretization_);
    keyframe_interval_ = num_keyframe_interval_points_ * discretization_;
    if (keyframe_interval_ != keyframe_interval)
      ROS_INFO("Trajectory keyframe interval modified : %f -> %f", keyframe_interval, keyframe_interval_);
    num_keyframes_ = safeToInt(duration / keyframe_interval_) + 1;
    num_points_ = (num_keyframes_ - 1) * num_keyframe_interval_points_ + 1;
    duration_ = (num_keyframes_ - 1) * keyframe_interval_;
  }
  if (duration_ != duration)
    ROS_INFO("Trajectory duration modified : %f -> %f", duration, duration_);

  start_index_ = 1;
  end_index_ = has_free_end_point_ ? num_points_ - 1 : num_points_ - 2;

  init();
}

ItompCIOTrajectory::ItompCIOTrajectory(const ItompCIOTrajectory& full_trajectory,
    const ItompPlanningGroup* planning_group, int diff_rule_length) :
    is_full_trajectory_(false), robot_model_(full_trajectory.robot_model_), planning_group_(planning_group), phase_stride_(
        full_trajectory.phase_stride_), discretization_(full_trajectory.discretization_), has_velocity_and_acceleration_(
        full_trajectory.has_velocity_and_acceleration_), has_free_end_point_(full_trajectory.has_free_end_point_), num_keyframe_interval_points_(
        full_trajectory.num_keyframe_interval_points_), keyframe_interval_(full_trajectory.keyframe_interval_), num_keyframes_(
        full_trajectory.num_keyframes_), duration_(full_trajectory.duration_)
{
  num_joints_ = planning_group_->num_joints_;

  if (diff_rule_length == 0)
  {
    num_points_ = full_trajectory.num_points_;
    start_index_ = full_trajectory.start_index_;
    end_index_ = full_trajectory.end_index_;
    keyframe_start_index_ = full_trajectory.keyframe_start_index_;
  }
  else
  {
    int start_extra = (diff_rule_length - 1) - full_trajectory.start_index_;
    int end_extra = (diff_rule_length - 1) - ((full_trajectory.num_points_ - 1) - full_trajectory.end_index_);
    num_points_ = full_trajectory.num_points_ + start_extra + end_extra;
    start_index_ = diff_rule_length - 1;
    end_index_ = (num_points_ - 1) - (diff_rule_length - 1);
    keyframe_start_index_ = full_trajectory.keyframe_start_index_ + start_extra;
  }

  // TODO: only contacts in this group?
  num_contacts_ = full_trajectory.num_contacts_;
  contact_phase_duration_ = full_trajectory.contact_phase_duration_;
  num_contact_phases_ = full_trajectory.num_contact_phases_;

  // allocate the memory:
  init();

  // now copy the trajectories over:
  copyFromFullTrajectory(full_trajectory);

  contact_trajectory_ = full_trajectory.contact_trajectory_;

  full_trajectory_index_.resize(num_points_);
  // now copy the trajectories over:
  int start_extra = start_index_ - full_trajectory.start_index_;
  for (int i = 0; i < num_points_; i++)
  {
    int source_traj_point = i - start_extra;
    if (source_traj_point < 0)
      source_traj_point = 0;
    if (source_traj_point >= full_trajectory.num_points_)
      source_traj_point = full_trajectory.num_points_ - 1;
    full_trajectory_index_[i] = source_traj_point;
  }
}

ItompCIOTrajectory::~ItompCIOTrajectory()
{
}

void ItompCIOTrajectory::init()
{
  trajectory_[TRAJECTORY_POSITION] = Eigen::MatrixXd(num_points_, num_joints_);
  // has at least the first point velocity & acceleration
  trajectory_[TRAJECTORY_VELOCITY] = Eigen::MatrixXd(has_velocity_and_acceleration_ ? num_points_ : 1, num_joints_);
  trajectory_[TRAJECTORY_ACCELERATION] = Eigen::MatrixXd(has_velocity_and_acceleration_ ? num_points_ : 1, num_joints_);

  // use keyframes only if num_keyframe_interval_points > 1
  if (!is_full_trajectory_ && hasKeyframes())
  {
    keyframes_[TRAJECTORY_POSITION] = Eigen::MatrixXd(num_keyframes_, num_joints_);
    if (has_velocity_and_acceleration_)
      keyframes_[TRAJECTORY_VELOCITY] = Eigen::MatrixXd(num_keyframes_, num_joints_);
  }

  // TODO: remove below
  contact_trajectory_ = Eigen::MatrixXd(num_contact_phases_ + 1, num_contacts_);

  free_trajectory_ = Eigen::MatrixXd(num_contact_phases_ + 1, num_joints_);
  free_vel_trajectory_ = Eigen::MatrixXd::Zero(num_contact_phases_ + 1, num_joints_);

  contact_start_points_.clear();
  contact_start_points_.push_back(0);

  for (int i = start_index_; i < end_index_ + 1; i += phase_stride_)
  {
    contact_start_points_.push_back(i);
  }
  contact_start_points_.push_back(end_index_ + 1);
  ROS_ASSERT(contact_start_points_.size() == num_contact_phases_);

  ROS_INFO("Contact Phases");
  for (int i = 0; i < num_contact_phases_; ++i)
  {
    ROS_INFO("Phase %d : %d - %d", i, contact_start_points_[i], contact_start_points_[i + 1] - 1);
  }
}

void ItompCIOTrajectory::copyFromGroupTrajectory(const ItompCIOTrajectory& group_trajectory)
{
  ROS_ASSERT(is_full_trajectory_ && !group_trajectory.is_full_trajectory_);

  int num_vars_free = end_index_ - start_index_ + 1;
  for (int i = 0; i < group_trajectory.planning_group_->num_joints_; i++)
  {
    int target_joint = group_trajectory.planning_group_->group_joints_[i].rbdl_joint_index_;
    trajectory_[TRAJECTORY_POSITION].block(start_index_, target_joint, num_vars_free, 1) =
        group_trajectory.trajectory_[TRAJECTORY_POSITION].block(group_trajectory.start_index_, i, num_vars_free, 1);

    if (has_velocity_and_acceleration_)
    {
      ROS_ASSERT(group_trajectory.has_velocity_and_acceleration_);

      trajectory_[TRAJECTORY_VELOCITY].block(start_index_, target_joint, num_vars_free, 1) =
          group_trajectory.trajectory_[TRAJECTORY_VELOCITY].block(group_trajectory.start_index_, i, num_vars_free, 1);

      trajectory_[TRAJECTORY_ACCELERATION].block(start_index_, target_joint, num_vars_free, 1) =
          group_trajectory.trajectory_[TRAJECTORY_ACCELERATION].block(group_trajectory.start_index_, i, num_vars_free,
              1);
    }
  }

  //contact_trajectory_ = group_trajectory.contact_trajectory_;
  int contact_end_index = num_contact_phases_ - 2;
  int contact_start_index = 1;
  int num_contact_vars_free = contact_end_index - contact_start_index + 1;
  for (int i = 0; i < group_trajectory.planning_group_->getNumContacts(); i++)
  {
    // TODO: need to be changed when multiple groups have contacts;
    int target_contact = i;
    contact_trajectory_.block(contact_start_index, target_contact, num_contact_vars_free, 1) =
        group_trajectory.contact_trajectory_.block(contact_start_index, i, num_contact_vars_free, 1);
  }
}

void ItompCIOTrajectory::copyFromGroupTrajectory(const ItompCIOTrajectory& group_trajectory, int point_index,
    int joint_index)
{
  ROS_ASSERT(is_full_trajectory_ && !group_trajectory.is_full_trajectory_);

  int target_joint = group_trajectory.planning_group_->group_joints_[joint_index].rbdl_joint_index_;

  trajectory_[TRAJECTORY_POSITION](start_index_ + point_index, target_joint) = group_trajectory(
      group_trajectory.start_index_ + point_index, joint_index, TRAJECTORY_POSITION);
}

void ItompCIOTrajectory::copyFromFullTrajectory(const ItompCIOTrajectory& full_trajectory)
{
  ROS_ASSERT(!is_full_trajectory_ && full_trajectory.is_full_trajectory_);

  // now copy the trajectories over:
  int num_vars_free = end_index_ - start_index_ + 1;
  for (int i = 0; i < num_joints_; i++)
  {
    int full_joint = planning_group_->group_joints_[i].rbdl_joint_index_;
    trajectory_[TRAJECTORY_POSITION].block(start_index_, i, num_vars_free, 1) =
        full_trajectory.trajectory_[TRAJECTORY_POSITION].block(full_trajectory.start_index_, full_joint, num_vars_free,
            1);
    if (has_velocity_and_acceleration_)
    {
      ROS_ASSERT(full_trajectory.has_velocity_and_acceleration_);

      trajectory_[TRAJECTORY_VELOCITY].block(start_index_, i, num_vars_free, 1) =
          full_trajectory.trajectory_[TRAJECTORY_VELOCITY].block(full_trajectory.start_index_, full_joint,
              num_vars_free, 1);
      trajectory_[TRAJECTORY_ACCELERATION].block(start_index_, i, num_vars_free, 1) =
          full_trajectory.trajectory_[TRAJECTORY_ACCELERATION].block(full_trajectory.start_index_, full_joint,
              num_vars_free, 1);
    }
  }

  // set start extra points using vel, acc
  if (has_velocity_and_acceleration_)
  {
    for (int j = 0; j < num_joints_; j++)
    {
      int full_joint = planning_group_->group_joints_[j].rbdl_joint_index_;
      double pos = full_trajectory(0, full_joint, TRAJECTORY_POSITION);
      double vel = full_trajectory(0, full_joint, TRAJECTORY_VELOCITY);
      double acc = full_trajectory(0, full_joint, TRAJECTORY_ACCELERATION);

      // only for root pos
      if (full_joint < 6)
      {
        for (int i = start_index_ - 2; i >= 0; --i)
        {
          double new_vel = vel - acc * discretization_;
          double new_pos = pos - vel * discretization_;
          (*this)(i, j, TRAJECTORY_POSITION) = new_pos;
          (*this)(i, j, TRAJECTORY_VELOCITY) = new_vel;
          vel = new_vel;
          pos = new_pos;
        }
      }
    }
  }
}

void ItompCIOTrajectory::copyKeyframesFromTrajectory()
{
  if (!hasKeyframes())
    return;

  int trajectory_start = start_index_ - 1;
  for (int i = 0; i < num_keyframes_; ++i)
  {
    int trajectory_index = trajectory_start + i * num_keyframe_interval_points_;

    keyframes_[TRAJECTORY_POSITION].block(i, 0, 1, num_joints_) = trajectory_[TRAJECTORY_POSITION].block(
        trajectory_index, 0, 1, num_joints_);

    if (has_velocity_and_acceleration_)
      keyframes_[TRAJECTORY_VELOCITY].block(i, 0, 1, num_joints_) = trajectory_[TRAJECTORY_VELOCITY].block(
          trajectory_index, 0, 1, num_joints_);
  }
}

void ItompCIOTrajectory::updateTrajectoryFromKeyframes()
{
  ROS_ASSERT(hasKeyframes());

  if (!has_velocity_and_acceleration_)
    ROS_ERROR("Keyframes with position only trajectory is not supported");

  // cubic interpolation of pos, vel, acc
  // update trajectory between (k, k+1]
  // acc is discontinuous at each keyframe
  for (int j = 0; j < num_joints_; ++j)
  {
    // skip the initial position
    double trajectory_index = keyframe_start_index_ + 1;
    for (int k = 0; k < num_keyframes_; ++k)
    {
      ecl::CubicPolynomial poly;
      poly = ecl::CubicPolynomial::DerivativeInterpolation(0.0, keyframes_[TRAJECTORY_POSITION](k, j),
          keyframes_[TRAJECTORY_VELOCITY](k, j), keyframe_interval_, keyframes_[TRAJECTORY_POSITION](k + 1, j),
          keyframes_[TRAJECTORY_VELOCITY](k + 1, j));
      for (int i = 1; i <= num_keyframe_interval_points_; ++i)
      {
        if (trajectory_index >= start_index_ && trajectory_index <= end_index_)
        {
          trajectory_[TRAJECTORY_POSITION](trajectory_index, j) = poly(discretization_ * i);
          trajectory_[TRAJECTORY_VELOCITY](trajectory_index, j) = poly.derivative(discretization_ * i);
          trajectory_[TRAJECTORY_ACCELERATION](trajectory_index, j) = poly.dderivative(discretization_ * i);
        }
        ++trajectory_index;
      }
    }
  }
}

void ItompCIOTrajectory::fillInMinJerk(const std::set<int>& groupJointsKDLIndices)
{
  int start_index = start_index_ - 1;
  int end_index = end_index_ + 1;
  double duration = (end_index - start_index) * discretization_;

  for (std::set<int>::const_iterator it = groupJointsKDLIndices.begin(); it != groupJointsKDLIndices.end(); ++it)
  {
    int j = *it;

    double x0 = trajectory_[TRAJECTORY_POSITION](start_index, j);
    double v0 = (j < 6) ? trajectory_[TRAJECTORY_VELOCITY](0, j) : 0.0;
    double a0 = (j < 6) ? trajectory_[TRAJECTORY_ACCELERATION](0, j) : 0.0;

    double x1 = trajectory_[TRAJECTORY_POSITION](end_index, j);
    double v1 = 0.0;
    double a1 = 0.0;

    ROS_INFO("Joint %d from %f(%f %f) to %f(%f %f)", j, x0, v0, a0, x1, v1, a1);

    ecl::QuinticPolynomial poly;
    poly = ecl::QuinticPolynomial::Interpolation(start_index * discretization_, x0, v0, a0, end_index * discretization_,
        x1, v1, a1);
    for (int i = start_index + 1; i <= end_index - 1; ++i)
    {
      trajectory_[TRAJECTORY_POSITION](i, j) = poly(i * discretization_);
      if (has_velocity_and_acceleration_)
      {
        trajectory_[TRAJECTORY_VELOCITY](i, j) = poly.derivative(i * discretization_);
        trajectory_[TRAJECTORY_ACCELERATION](i, j) = poly.dderivative(i * discretization_);
      }
    }
  }
}

void ItompCIOTrajectory::fillInMinJerkCartesianTrajectory(const std::set<int>& groupJointsKDLIndices,
    const moveit_msgs::Constraints& path_constraints, const string& group_name)
{
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(robot_model_->getMoveitRobotModel()));
  const robot_state::JointModelGroup* joint_model_group = robot_model_->getMoveitRobotModel()->getJointModelGroup(
      group_name);

  geometry_msgs::Vector3 start_position = path_constraints.position_constraints[0].target_point_offset;
  geometry_msgs::Vector3 goal_position = path_constraints.position_constraints[1].target_point_offset;
  geometry_msgs::Quaternion orientation = path_constraints.orientation_constraints[0].orientation;

  double start_index = start_index_ - 1;
  double end_index = end_index_ + 1;
  double duration = (end_index - start_index) * discretization_;

  // set a state to the start config
  std::vector<double> positions(num_joints_);
  for (std::size_t j = 0; j < num_joints_; j++)
  {
    positions[j] = trajectory_[TRAJECTORY_POSITION](start_index, j);
  }
  kinematic_state->setVariablePositions(&positions[0]);
  kinematic_state->update();

  double T[6]; // powers of the time duration
  T[0] = 1.0;
  T[1] = duration;

  for (int i = 2; i <= 5; i++)
    T[i] = T[i - 1] * T[1];

  // calculate the spline coefficients for 3d space
  const int CARTESIAN_SPACE_DOF = 3;
  double coeff[CARTESIAN_SPACE_DOF][6];

  double x0[3] =
  { start_position.x, start_position.y, start_position.z };
  double x1[3] =
  { goal_position.x, goal_position.y, goal_position.z };
  double v0 = 0.0, v1 = 0.0;
  double a0 = 0.0, a1 = 0.0;
  ROS_INFO(
      "CartesianPos from (%f, %f, %f)(%f %f) to (%f, %f, %f)(%f %f)", x0[0], x0[1], x0[2], v0, a0, x1[0], x1[1], x1[2], v1, a1);

  for (int i = start_index; i <= end_index; ++i)
  {
    ecl::QuinticPolynomial poly[3];
    for (int d = 0; d < 3; ++d)
    {
      poly[d] = ecl::QuinticPolynomial::Interpolation(start_index * discretization_, x0[d], v0, a0,
          end_index * discretization_, x1[d], v1, a1);
    }
    geometry_msgs::Vector3 position;
    position.x = poly[0](i * discretization_);
    position.y = poly[1](i * discretization_);
    position.z = poly[2](i * discretization_);

    // Use IK to compute joint values
    vector<double> ik_solution(num_joints_);

    Eigen::Affine3d end_effector_state = Eigen::Affine3d::Identity();
    Eigen::Quaternion<double> rot(orientation.w, orientation.x, orientation.y, orientation.z);
    Eigen::Vector3d trans(position.x, position.y, position.z);
    Eigen::Matrix3d mat = rot.toRotationMatrix();
    end_effector_state.linear() = mat;
    end_effector_state.translation() = trans;

    kinematics::KinematicsQueryOptions options;
    options.return_approximate_solution = false;
    bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state, 10, 0.1,
        moveit::core::GroupStateValidityCallbackFn(), options);
    if (found_ik)
    {
      //ROS_INFO("IK solution found for waypoint %d", i);

      std::vector<double> group_values;
      kinematic_state->copyJointGroupPositions(joint_model_group, group_values);
      double* state_pos = kinematic_state->getVariablePositions();
      ROS_ASSERT(num_joints_ == kinematic_state->getVariableCount());
      //printf("EE : (%f %f %f)(%f %f %f %f) : ", position.x, position.y, position.z, orientation.x, orientation.y,
      //  orientation.z, orientation.w);
      for (std::size_t k = 0; k < kinematic_state->getVariableCount(); ++k)
      {
        ik_solution[k] = state_pos[k];
        if (i != start_index)
          trajectory_[TRAJECTORY_POSITION](i, k) = state_pos[k];
        //printf("%f ", state_pos[k]);
      }
      //printf("\n");
    }
    else
    {
      ROS_INFO("Could not find IK solution for waypoint %d", i);
    }
  }
}

void ItompCIOTrajectory::printTrajectory() const
{
  printf("Position Trajectory\n");
  for (int i = 0; i < num_points_; ++i)
  {
    printf("%d : ", i);
    for (int j = 0; j < num_joints_; ++j)
    {
      printf("%f ", trajectory_[TRAJECTORY_POSITION](i, j));
    }
    printf("\n");
  }

  if (has_velocity_and_acceleration_)
  {
    printf("Velocity Trajectory\n");
    for (int i = 0; i < num_points_; ++i)
    {
      printf("%d : ", i);
      for (int j = 0; j < num_joints_; ++j)
      {
        printf("%f ", trajectory_[TRAJECTORY_VELOCITY](i, j));
      }
      printf("\n");
    }
    printf("Acceleration Trajectory\n");
    for (int i = 0; i < num_points_; ++i)
    {
      printf("%d : ", i);
      for (int j = 0; j < num_joints_; ++j)
      {
        printf("%f ", trajectory_[TRAJECTORY_ACCELERATION](i, j));
      }
      printf("\n");
    }
  }
}

// TODO: remove below
void ItompCIOTrajectory::updateFreePointsFromTrajectory()
{

}

void ItompCIOTrajectory::updateTrajectoryFromFreePoints()
{

}

void ItompCIOTrajectory::updateTrajectoryFromFreePoint(int point_index, int joint_index)
{
}

}
