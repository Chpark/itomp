#include <itomp_cio_planner/common.h>
#include <itomp_cio_planner/trajectory/itomp_cio_trajectory.h>
#include <itomp_cio_planner/model/itomp_robot_model.h>
#include <itomp_cio_planner/model/itomp_planning_group.h>
#include <ros/console.h>
#include <ros/assert.h>

using namespace std;

namespace itomp_cio_planner
{

inline int safeToInt(double a)
{
  return (int) (a + 1E-7);
}

ItompCIOTrajectory::ItompCIOTrajectory(const ItompRobotModel* robot_model, double duration, double discretization,
    double num_contacts, double contact_phase_duration) :
    robot_model_(robot_model), planning_group_(NULL), phase_stride_(safeToInt(contact_phase_duration / discretization)), num_contact_phases_(
        safeToInt(duration / contact_phase_duration)), num_points_(phase_stride_ * num_contact_phases_ + 1), num_joints_(
        robot_model_->getNumKDLJoints()), discretization_(discretization), duration_(
        (num_points_ - 1) * discretization), num_contacts_(num_contacts), contact_phase_duration_(
        contact_phase_duration), start_index_(1), end_index_(num_points_ - 2)
{
  ROS_ASSERT(duration == duration_);
  init();
}

ItompCIOTrajectory::ItompCIOTrajectory(const ItompCIOTrajectory& source_traj, const ItompPlanningGroup* planning_group,
    int diff_rule_length) :
    robot_model_(source_traj.robot_model_), planning_group_(planning_group), phase_stride_(source_traj.phase_stride_), discretization_(
        source_traj.discretization_)
{
  // TODO: only contacts in this group?
  num_contacts_ = source_traj.num_contacts_;
  contact_phase_duration_ = source_traj.contact_phase_duration_;
  num_contact_phases_ = source_traj.num_contact_phases_;

  num_joints_ = planning_group_->num_joints_;

  int start_extra = (diff_rule_length - 1) - source_traj.start_index_;
  int end_extra = (diff_rule_length - 1) - ((source_traj.num_points_ - 1) - source_traj.end_index_);

  num_points_ = source_traj.num_points_ + start_extra + end_extra;

  start_index_ = diff_rule_length - 1;
  end_index_ = (num_points_ - 1) - (diff_rule_length - 1);

  duration_ = source_traj.duration_;

  // allocate the memory:
  init();

  // now copy the trajectories over:
  copyFromFullTrajectory(source_traj);

  contact_trajectory_ = source_traj.contact_trajectory_;

  full_trajectory_index_.resize(num_points_);
  // now copy the trajectories over:
  for (int i = 0; i < num_points_; i++)
  {
    int source_traj_point = i - start_extra;
    if (source_traj_point < 0)
      source_traj_point = 0;
    if (source_traj_point >= source_traj.num_points_)
      source_traj_point = source_traj.num_points_ - 1;
    full_trajectory_index_[i] = source_traj_point;
  }
}

ItompCIOTrajectory::~ItompCIOTrajectory()
{
}

void ItompCIOTrajectory::copyFromFullTrajectory(const ItompCIOTrajectory& full_trajectory)
{
  /*
   for (int i = 0; i < num_joints_; i++)
   {
   int source_joint = planning_group_->group_joints_[i].kdl_joint_index_;
   trajectory_.block(0, i, num_points_, 1) = full_trajectory.trajectory_.block(0, source_joint, num_points_, 1);
   free_trajectory_.block(0, i, num_contact_phases_ + 1, 1) = full_trajectory.free_trajectory_.block(0, source_joint,
   num_contact_phases_ + 1, 1);
   free_vel_trajectory_.block(0, i, num_contact_phases_ + 1, 1) = full_trajectory.free_vel_trajectory_.block(0,
   source_joint, num_contact_phases_ + 1, 1);
   }
   */

  // now copy the trajectories over:
  int start_extra = (DIFF_RULE_LENGTH - 1) - full_trajectory.start_index_;
  for (int i = 0; i < num_points_; i++)
  {
    int source_traj_point = i - start_extra;
    if (source_traj_point < 0)
      source_traj_point = 0;
    if (source_traj_point >= full_trajectory.num_points_)
      source_traj_point = full_trajectory.num_points_ - 1;
    for (int j = 0; j < num_joints_; j++)
    {
      int source_joint = planning_group_->group_joints_[j].kdl_joint_index_;
      (*this)(i, j) = full_trajectory(source_traj_point, source_joint);
    }
  }
  // set pre start points using vel, acc
  for (int j = 0; j < num_joints_; j++)
  {
    int source_joint = planning_group_->group_joints_[j].kdl_joint_index_;
    double pos = full_trajectory(0, source_joint);
    double vel = full_trajectory.vel_start_(source_joint);
    double acc = full_trajectory.acc_start_(source_joint);

    // only for root pos
    if (source_joint < 6)
    {
      for (int i = start_extra - 1; i >= 0; --i)
      {
        double new_vel = vel - acc * discretization_;
        double new_pos = pos - vel * discretization_;
        (*this)(i, j) = new_pos;
        vel = new_vel;
        pos = new_pos;
      }
    }
  }
}

void ItompCIOTrajectory::init()
{
  trajectory_ = Eigen::MatrixXd(num_points_, num_joints_);
  contact_trajectory_ = Eigen::MatrixXd(num_contact_phases_ + 1, num_contacts_);

  free_trajectory_ = Eigen::MatrixXd(num_contact_phases_ + 1, num_joints_);
  free_vel_trajectory_ = Eigen::MatrixXd::Zero(num_contact_phases_ + 1, num_joints_);

  ROS_INFO("Contact Phases");
  for (int i = 0; i < num_contact_phases_; ++i)
  {
    int start_point, end_point;
    getContactPhaseRange(i, start_point, end_point);
    ROS_INFO("Phase %d : %d - %d", i, start_point, end_point);
  }
}

void ItompCIOTrajectory::updateFromGroupTrajectory(const ItompCIOTrajectory& group_trajectory)
{
  /*
   for (int i = 0; i < group_trajectory.planning_group_->num_joints_; i++)
   {
   int target_joint = group_trajectory.planning_group_->group_joints_[i].kdl_joint_index_;
   trajectory_.block(0, target_joint, num_points_, 1) = group_trajectory.trajectory_.block(0, i, num_points_, 1);
   free_trajectory_.block(0, target_joint, num_contact_phases_ + 1, 1) = group_trajectory.free_trajectory_.block(0, i,
   num_contact_phases_ + 1, 1);
   free_vel_trajectory_.block(0, target_joint, num_contact_phases_ + 1, 1) =
   group_trajectory.free_vel_trajectory_.block(0, i, num_contact_phases_ + 1, 1);
   }

   contact_trajectory_ = group_trajectory.contact_trajectory_;
   */

  int num_vars_free = end_index_ - start_index_ + 1;
  for (int i = 0; i < group_trajectory.planning_group_->num_joints_; i++)
  {
    int target_joint = group_trajectory.planning_group_->group_joints_[i].kdl_joint_index_;
    trajectory_.block(start_index_, target_joint, num_vars_free, 1) = group_trajectory.trajectory_.block(
        group_trajectory.start_index_, i, num_vars_free, 1);
  }

  contact_trajectory_ = group_trajectory.contact_trajectory_;
}

void ItompCIOTrajectory::updateFromGroupTrajectory(const ItompCIOTrajectory& group_trajectory, int point_index,
    int joint_index)
{
  int i = joint_index;
  /*
   {
   int target_joint = group_trajectory.planning_group_->group_joints_[i].kdl_joint_index_;
   trajectory_.block((point_index - 1) * phase_stride_, target_joint, 2 * phase_stride_, 1) =
   group_trajectory.trajectory_.block((point_index - 1) * phase_stride_, i, 2 * phase_stride_, 1);
   }
   */
  int target_joint = group_trajectory.planning_group_->group_joints_[i].kdl_joint_index_;
  trajectory_.block(start_index_ + point_index, target_joint, 1, 1) = group_trajectory.trajectory_.block(
      group_trajectory.start_index_ + point_index, i, 1, 1);
}

void ItompCIOTrajectory::updateFreePointsFromTrajectory()
{
  /*
   for (int i = 0; i < num_joints_; ++i)
   {
   for (int j = 0; j <= num_contact_phases_; ++j)
   {
   free_trajectory_(j, i) = trajectory_(j * phase_stride_, i);
   }
   }
   */

}

void ItompCIOTrajectory::updateTrajectoryFromFreePoints()
{
  /*
   double coeff[4];
   double t[4];

   for (int i = 0; i < num_joints_; ++i)
   {
   int pos = 0;
   for (int j = 0; j < num_contact_phases_; ++j)
   {
   double start_pos = free_trajectory_(j, i);
   double end_pos = free_trajectory_(j + 1, i);
   double start_vel = free_vel_trajectory_(j, i);
   double end_vel = free_vel_trajectory_(j + 1, i);

   coeff[0] = start_pos;
   coeff[1] = start_vel;
   coeff[2] = -end_vel + 3 * end_pos - 2 * start_vel - 3 * start_pos;
   coeff[3] = end_vel - 2 * end_pos + start_vel + 2 * start_pos;

   for (int k = 0; k < phase_stride_; ++k)
   {
   t[0] = 1.0;
   t[1] = ((double) k) / phase_stride_;
   t[2] = t[1] * t[1];
   t[3] = t[2] * t[1];
   (*this)(pos, i) = 0.0;
   for (int l = 0; l <= 3; l++)
   {
   (*this)(pos, i) += t[l] * coeff[l];
   }
   ++pos;
   }
   }
   }
   */
}

void ItompCIOTrajectory::updateTrajectoryFromFreePoint(int point_index, int joint_index)
{
  /*
   double coeff[4];
   double t[4];

   int i = joint_index;
   {
   for (int j = point_index - 1; j < point_index + 1; ++j)
   {
   ROS_ASSERT(j >= 0 && j < num_contact_phases_);
   double start_pos = free_trajectory_(j, i);
   double end_pos = free_trajectory_(j + 1, i);
   double start_vel = free_vel_trajectory_(j, i);
   double end_vel = free_vel_trajectory_(j + 1, i);

   coeff[0] = start_pos;
   coeff[1] = start_vel;
   coeff[2] = -end_vel + 3 * end_pos - 2 * start_vel - 3 * start_pos;
   coeff[3] = end_vel - 2 * end_pos + start_vel + 2 * start_pos;

   int pos = j * phase_stride_;
   for (int k = 0; k < phase_stride_; ++k)
   {
   t[0] = 1.0;
   t[1] = ((double) k) / phase_stride_;
   t[2] = t[1] * t[1];
   t[3] = t[2] * t[1];
   (*this)(pos, i) = 0.0;
   for (int l = 0; l <= 3; l++)
   {
   (*this)(pos, i) += t[l] * coeff[l];
   }
   ++pos;
   }
   }
   }
   */
}

void ItompCIOTrajectory::fillInMinJerk(const std::set<int>& groupJointsKDLIndices,
    const Eigen::MatrixXd::RowXpr joint_vel_array, const Eigen::MatrixXd::RowXpr joint_acc_array)
{
  /*
   vel_start_ = joint_vel_array;
   acc_start_ = joint_acc_array;

   double start_index = 0;
   double end_index = num_points_ - 1;
   double duration = duration_;
   double discretization = discretization_;

   // calculate the spline coefficients for each joint:
   double coeff[num_joints_][6];

   bool hasRotation = false;
   for (std::set<int>::const_iterator it = groupJointsKDLIndices.begin(); it != groupJointsKDLIndices.end(); ++it)
   {
   int i = *it;

   double x0 = (*this)(start_index, i);
   double x1 = (*this)(end_index, i);
   double v0 = joint_vel_array(i);
   double a0 = joint_acc_array(i);
   ROS_INFO("Joint %d from %f(%f %f) to %f", i, x0, v0, a0, x1);

   v0 = v0 * duration;
   a0 = a0 * duration * duration;

   coeff[i][0] = x0;
   coeff[i][1] = v0;
   coeff[i][2] = 0.5 * a0;
   coeff[i][3] = (-1.5 * a0 - 6 * v0 - 10 * x0 + 10 * x1);
   coeff[i][4] = (1.5 * a0 + 8 * v0 + 15 * x0 - 15 * x1);
   coeff[i][5] = (-0.5 * a0 - 3 * v0 - 6 * x0 + 6 * x1);
   }

   // now fill in the joint positions at each time step
   int numPoints = end_index - start_index;
   for (int i = start_index + 1; i < end_index; i++)
   {
   double t[6]; // powers of the time index point
   t[0] = 1.0;
   t[1] = (double) (i - start_index) / (num_points_ - 1);
   for (int k = 2; k <= 5; k++)
   t[k] = t[k - 1] * t[1];

   for (std::set<int>::const_iterator it = groupJointsKDLIndices.begin(); it != groupJointsKDLIndices.end(); ++it)
   {
   int j = *it;

   (*this)(i, j) = 0.0;
   for (int k = 0; k <= 5; k++)
   {
   (*this)(i, j) += t[k] * coeff[j][k];
   }
   }
   }

   updateFreePointsFromTrajectory();
   for (std::set<int>::const_iterator it = groupJointsKDLIndices.begin(); it != groupJointsKDLIndices.end(); ++it)
   {
   int i = *it;
   free_vel_trajectory_(0, i) = joint_vel_array(i);
   free_vel_trajectory_(num_contact_phases_, i) = 0.0;

   for (int j = 1; j < num_contact_phases_; ++j)
   {
   free_vel_trajectory_(j, i) = 0.5 * ((*this)(j * phase_stride_ + 1, i) - (*this)(j * phase_stride_ - 1, i));
   }
   }
   */

  vel_start_ = joint_vel_array;
  acc_start_ = joint_acc_array;
  double start_index = start_index_ - 1;
  double end_index = end_index_ + 1;
  double duration = (end_index - start_index) * discretization_;

  double T[6]; // powers of the time duration
  T[0] = 1.0;
  T[1] = duration;

  for (int i = 2; i <= 5; i++)
    T[i] = T[i - 1] * T[1];

  // calculate the spline coefficients for each joint:
  // (these are for the special case of zero start and end vel and acc)
  double coeff[num_joints_][6];

  bool hasRotation = false;
  for (std::set<int>::const_iterator it = groupJointsKDLIndices.begin(); it != groupJointsKDLIndices.end(); ++it)
  {
    int i = *it;

    // rotation is handled in a special manner
    /*
     if (i == 3)
     {
     hasRotation = true;
     continue;
     }
     */
    double x0 = (*this)(start_index, i);
    double x1 = (*this)(end_index, i);
    double v0 = (i < 6) ? joint_vel_array(i) : 0.0;
    double a0 = (i < 6) ? joint_acc_array(i) : 0.0;
    ROS_INFO("Joint %d from %f(%f %f) to %f", i, x0, v0, a0, x1);

    v0 = v0 * duration;
    a0 = a0 * duration * duration;

    coeff[i][0] = x0;
    coeff[i][1] = v0;
    coeff[i][2] = 0.5 * a0;
    coeff[i][3] = (-1.5 * a0 - 6 * v0 - 10 * x0 + 10 * x1);
    coeff[i][4] = (1.5 * a0 + 8 * v0 + 15 * x0 - 15 * x1);
    coeff[i][5] = (-0.5 * a0 - 3 * v0 - 6 * x0 + 6 * x1);
  }

  // now fill in the joint positions at each time step
  int numPoints = end_index - start_index;
  for (int i = start_index + 1; i < end_index; i++)
  {
    double t[6]; // powers of the time index point
    t[0] = 1.0;
    t[1] = (double) (i - start_index) / numPoints; //(i - start_index) * discretization_;
    for (int k = 2; k <= 5; k++)
      t[k] = t[k - 1] * t[1];

    for (std::set<int>::const_iterator it = groupJointsKDLIndices.begin(); it != groupJointsKDLIndices.end(); ++it)
    {
      int j = *it;

      (*this)(i, j) = 0.0;
      for (int k = 0; k <= 5; k++)
      {
        (*this)(i, j) += t[k] * coeff[j][k];
      }
    }
  }

  /*
   // rotation reaches the goal in the first contact phase
   if (hasRotation)
   {
   double interp_end_index = contact_start_points_[2] - 1;
   double duration = (interp_end_index - start_index) * discretization_;

   double T[6]; // powers of the time duration
   T[0] = 1.0;
   T[1] = duration;

   for (int i = 2; i <= 5; i++)
   T[i] = T[i - 1] * T[1];

   // calculate the spline coefficients for each joint:
   // (these are for the special case of zero start and end vel and acc)
   double coeff[num_joints_][6];

   {
   int i = 3;
   double x0 = (*this)(start_index, i);
   double x1 = (*this)(end_index, i);
   double v0 = joint_vel_array(i);
   double a0 = joint_acc_array(i);
   ROS_INFO("Joint %d from %f(%f %f) to %f", i, x0, v0, a0, x1);

   v0 = v0 * duration;
   a0 = a0 * duration * duration;

   coeff[i][0] = x0;
   coeff[i][1] = v0;
   coeff[i][2] = 0.5 * a0;
   coeff[i][3] = (-1.5 * a0 - 6 * v0 - 10 * x0 + 10 * x1);
   coeff[i][4] = (1.5 * a0 + 8 * v0 + 15 * x0 - 15 * x1);
   coeff[i][5] = (-0.5 * a0 - 3 * v0 - 6 * x0 + 6 * x1);
   }

   // now fill in the joint positions at each time step
   int numPoints = interp_end_index - start_index;
   for (int i = start_index + 1; i < interp_end_index; i++)
   {
   double t[6]; // powers of the time index point
   t[0] = 1.0;
   t[1] = (double) (i - start_index) / numPoints; //(i - start_index) * discretization_;
   for (int k = 2; k <= 5; k++)
   t[k] = t[k - 1] * t[1];

   {
   int j = 3;

   (*this)(i, j) = 0.0;
   for (int k = 0; k <= 5; k++)
   {
   (*this)(i, j) += t[k] * coeff[j][k];
   }
   }
   }
   for (int i = interp_end_index; i < end_index; i++)
   {
   (*this)(i, 3) = (*this)(end_index, 3);
   }
   }*/
}

void ItompCIOTrajectory::fillInMinJerkWithMidPoint(const vector<double>& midPoint,
    const std::set<int>& groupJointsKDLIndices, int index)
{
  /*
   double start_index = start_index_ - 1;
   double end_index = end_index_ + 1;
   double T[7]; // powers of the time duration
   T[0] = 1.0;
   T[1] = (end_index - start_index) * discretization_;

   for (int i = 2; i <= 6; i++)
   T[i] = T[i - 1] * T[1];

   // calculate the spline coefficients for each joint:
   // (these are for the special case of zero start and end vel and acc)
   double coeff[num_joints_][7];
   for (std::set<int>::const_iterator it = groupJointsKDLIndices.begin(); it != groupJointsKDLIndices.end(); ++it)
   {
   int i = *it;
   ROS_INFO("Joint %d from %f to %f", i, (*this)(start_index, i), (*this)(end_index, i));

   if (index != 0)	// || i == 27 || i == 28 || i == 35 || i == 9)

   {
   //int group_joint_index = kdlToGroupJoint.at(i);

   double x0 = (*this)(start_index, i);
   double x1 = (*this)(end_index, i);
   double mid = midPoint[i];

   if (i == 9)
   mid = -0.5;

   if (i == 27)
   mid = 1.2;

   if (i == 28)
   mid = -1.0;

   if (i == 35)
   mid = -0.6;

   coeff[i][0] = x0;
   coeff[i][1] = 0;
   coeff[i][2] = 0;
   coeff[i][3] = (-22 * (x1 - x0) + 64 * (mid - x0)) / T[3];
   coeff[i][4] = (81 * (x1 - x0) - 192 * (mid - x0)) / T[4];
   coeff[i][5] = (-90 * (x1 - x0) + 192 * (mid - x0)) / T[5];
   coeff[i][6] = (32 * (x1 - x0) - 64 * (mid - x0)) / T[6];
   }
   else
   {
   double x0 = (*this)(start_index, i);
   double x1 = (*this)(end_index, i);
   coeff[i][0] = x0;
   coeff[i][1] = 0;
   coeff[i][2] = 0;
   coeff[i][3] = (-20 * x0 + 20 * x1) / (2 * T[3]);
   coeff[i][4] = (30 * x0 - 30 * x1) / (2 * T[4]);
   coeff[i][5] = (-12 * x0 + 12 * x1) / (2 * T[5]);
   coeff[i][6] = 0;
   }
   }

   // now fill in the joint positions at each time step
   for (int i = start_index + 1; i < end_index; i++)
   {
   double t[7]; // powers of the time index point
   t[0] = 1.0;
   t[1] = (i - start_index) * discretization_;
   for (int k = 2; k <= 6; k++)
   t[k] = t[k - 1] * t[1];

   for (std::set<int>::const_iterator it = groupJointsKDLIndices.begin(); it != groupJointsKDLIndices.end(); ++it)
   {
   int j = *it;
   (*this)(i, j) = 0.0;
   for (int k = 0; k <= 6; k++)
   {
   (*this)(i, j) += t[k] * coeff[j][k];
   }
   }
   }
   */
}

void ItompCIOTrajectory::fillInMinJerkCartesianTrajectory(const std::set<int>& groupJointsKDLIndices,
    const Eigen::MatrixXd::RowXpr joint_vel_array, const Eigen::MatrixXd::RowXpr joint_acc_array,
    const moveit_msgs::Constraints& path_constraints, const string& group_name)
{
  vel_start_ = joint_vel_array;
  acc_start_ = joint_acc_array;

  int start_index = start_index_ - 1;
  int end_index = end_index_ + 1;
  int numPoints = end_index - start_index;

  ROS_ASSERT(path_constraints.position_constraints.size() == path_constraints.orientation_constraints.size());
  int num_cartesian_waypoints = path_constraints.position_constraints.size();
  double waypoint_stride = (double) numPoints / (num_cartesian_waypoints + 1);

  // compute waypoint ik solutions
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(robot_model_->getRobotModel()));
  std::vector<double> positions(num_joints_);
  for (int i = 0; i < kinematic_state->getVariableCount(); ++i)
    positions[i] = kinematic_state->getVariablePositions()[i];
  const robot_state::JointModelGroup* joint_model_group = robot_model_->getRobotModel()->getJointModelGroup(group_name);
  vector<vector<double> > cartesian_waypoints_ik_solutions(num_cartesian_waypoints);
  for (int i = 0; i < num_cartesian_waypoints; ++i)
  {
    cartesian_waypoints_ik_solutions[i].resize(num_joints_);
    geometry_msgs::Vector3 position = path_constraints.position_constraints[i].target_point_offset;
    geometry_msgs::Quaternion orientation = path_constraints.orientation_constraints[i].orientation;
    Eigen::Affine3d end_effector_state = Eigen::Affine3d::Identity();
    Eigen::Quaternion<double> rot(orientation.w, orientation.x, orientation.y, orientation.z);
    Eigen::Vector3d trans(position.x, position.y, position.z);
    Eigen::Matrix3d mat = rot.toRotationMatrix();
    end_effector_state.linear() = mat;
    end_effector_state.translation() = trans;

    kinematics::KinematicsQueryOptions options;
    options.return_approximate_solution = false;
    //kinematic_state->setVariablePositions(&positions[0]);
    //kinematic_state->update();
    bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state, 10, 0.1,
        moveit::core::GroupStateValidityCallbackFn(), options);
    if (found_ik)
    {
      ROS_INFO("IK solution found for waypoint %d", i);

      std::vector<double> group_values;
      kinematic_state->copyJointGroupPositions(joint_model_group, group_values);
      double* state_pos = kinematic_state->getVariablePositions();
      ROS_ASSERT(num_joints_ == kinematic_state->getVariableCount());
      printf("EE : (%f %f %f)(%f %f %f %f) : ", position.x, position.y, position.z, orientation.x, orientation.y,
          orientation.z, orientation.w);
      for (std::size_t k = 0; k < kinematic_state->getVariableCount(); ++k)
      {
        cartesian_waypoints_ik_solutions[i][k] = state_pos[k];
        printf("%f ", state_pos[k]);
      }
      printf("\n");
    }
    else
    {
      ROS_INFO("Could not find IK solution for waypoint %d", i);
    }
  }

  // calculate the spline coefficients for each joint:
  double coeff[num_joints_][6];

  for (std::set<int>::const_iterator it = groupJointsKDLIndices.begin(); it != groupJointsKDLIndices.end(); ++it)
  {
    int d = *it;

    int from = start_index;
    int to = std::min((int) (from + waypoint_stride), end_index);
    double duration = (to - from) * discretization_;

    double v0 = 0.0;
    double a0 = 0.0;
    double x0 = (*this)(start_index, d);
    double x1;
    for (int range = 0; range <= num_cartesian_waypoints; ++range)
    {
      // evaluate coefficients
      v0 = v0 * duration;
      a0 = a0 * duration * duration;

      if (range != num_cartesian_waypoints)
      {
        x1 = cartesian_waypoints_ik_solutions[range][d];
        /*
         coeff[d][0] = x0;
         coeff[d][1] = v0;
         coeff[d][2] = 0.5 * a0;
         coeff[d][3] = (-0.5 * a0 - v0 - x0 + x1);
         coeff[d][4] = 0;
         coeff[d][5] = 0;
         */
      }
      else
      {
        to = end_index;
        duration = (to - from) * discretization_;
        x1 = (*this)(end_index, d);
      }

      // endpoint vel, acc = 0
      coeff[d][0] = x0;
      coeff[d][1] = v0;
      coeff[d][2] = 0.5 * a0;
      coeff[d][3] = (-1.5 * a0 - 6 * v0 - 10 * x0 + 10 * x1);
      coeff[d][4] = (1.5 * a0 + 8 * v0 + 15 * x0 - 15 * x1);
      coeff[d][5] = (-0.5 * a0 - 3 * v0 - 6 * x0 + 6 * x1);

      ROS_INFO("Joint %d range %d from %f(%f %f) to %f", d, range, x0, v0, a0, x1);

      // fill in the joint positions at each time step
      for (int i = from; i <= to; ++i)
      {
        double t[6]; // powers of the time index point
        t[0] = 1.0;
        t[1] = (double) (i - from) / (to - from);
        for (int k = 2; k <= 5; k++)
          t[k] = t[k - 1] * t[1];

        (*this)(i, d) = 0.0;
        for (int k = 0; k <= 5; k++)
        {
          (*this)(i, d) += t[k] * coeff[d][k];
        }
      }

      from = to;
      to = std::min((int) (from + waypoint_stride), end_index);
      duration = (to - from) * discretization_;
      x0 = x1;

      // update v0, a0
      /*
       if (range != num_cartesian_waypoints)
       {
       double t[6]; // powers of the time index point
       t[0] = 1.0;
       t[1] = duration;
       for (int k = 2; k <= 5; k++)
       t[k] = t[k - 1] * t[1];

       v0 = 0;
       for (int k = 1; k <= 5; ++k)
       {
       v0 += k * coeff[d][k] * t[k - 1];
       }

       a0 = 0;
       for (int k = 2; k <= 5; ++k)
       {
       a0 += k * (k - 1) * coeff[d][k] * t[k - 2];
       }
       }
       */
    }
  }
  printTrajectory();
}

void ItompCIOTrajectory::printTrajectory() const
{
  printf("Full Trajectory\n");
  for (int i = 0; i < num_points_; ++i)
  {
    printf("%d : ", i);
    for (int j = 0; j < num_joints_; ++j)
    {
      printf("%f ", trajectory_(i, j));
    }
    printf("\n");
  }
  printf("Free Trajectory\n");
  for (int i = 0; i <= num_contact_phases_; ++i)
  {
    printf("%d : ", i);
    for (int j = 0; j < num_joints_; ++j)
    {
      printf("%f ", free_trajectory_(i, j));
    }
    printf("\n");
  }
  printf("Free Velocity Trajectory\n");
  for (int i = 0; i <= num_contact_phases_; ++i)
  {
    printf("%d : ", i);
    for (int j = 0; j < num_joints_; ++j)
    {
      printf("%f ", free_vel_trajectory_(i, j));
    }
    printf("\n");
  }
}

}
