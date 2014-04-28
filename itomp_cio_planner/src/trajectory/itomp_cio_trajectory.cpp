#include <itomp_cio_planner/common.h>
#include <itomp_cio_planner/trajectory/itomp_cio_trajectory.h>
#include <itomp_cio_planner/model/itomp_robot_model.h>
#include <itomp_cio_planner/model/itomp_planning_group.h>
#include <ros/console.h>
#include <ros/assert.h>

using namespace std;

namespace itomp_cio_planner
{

ItompCIOTrajectory::ItompCIOTrajectory(const ItompRobotModel* robot_model, double duration, double discretization,
		double num_contacts, double contact_phase_duration) :
	robot_model_(robot_model), planning_group_(NULL), num_points_((duration / discretization) + 1), num_joints_(
			robot_model_->getNumKDLJoints()), discretization_(discretization), duration_(duration), start_index_(1),
			end_index_(num_points_ - 2), num_contacts_(num_contacts), contact_phase_duration_(contact_phase_duration),
			num_contact_phases_((duration / contact_phase_duration) + 2)
{
	init();
}

ItompCIOTrajectory::ItompCIOTrajectory(const ItompCIOTrajectory& source_traj, const ItompPlanningGroup* planning_group,
		int diff_rule_length) :
	robot_model_(source_traj.robot_model_), planning_group_(planning_group), discretization_(
			source_traj.discretization_)
{
	// TODO: only contacts in this group?
	num_contacts_ = source_traj.num_contacts_;
	contact_phase_duration_ = source_traj.contact_phase_duration_;
	num_contact_phases_ = source_traj.num_contact_phases_;

	num_joints_ = planning_group_->num_joints_;

	// figure out the num_points_:
	// we need diff_rule_length-1 extra points on either side:
	int start_extra = (diff_rule_length - 1) - source_traj.start_index_;
	int end_extra = (diff_rule_length - 1) - ((source_traj.num_points_ - 1) - source_traj.end_index_);

	num_points_ = source_traj.num_points_ + start_extra + end_extra;
	start_index_ = diff_rule_length - 1;
	end_index_ = (num_points_ - 1) - (diff_rule_length - 1);
	duration_ = (num_points_ - 1) * discretization_;

	// allocate the memory:
	init();

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
		for (int j = 0; j < num_joints_; j++)
		{
			int source_joint = planning_group_->group_joints_[j].kdl_joint_index_;
			(*this)(i, j) = source_traj(source_traj_point, source_joint);
		}
	}

	// set pre start points using vel, acc
	for (int j = 0; j < num_joints_; j++)
	{
		int source_joint = planning_group_->group_joints_[j].kdl_joint_index_;
		double pos = source_traj(0, source_joint);
		double vel = source_traj.joint_vel_array_(source_joint);
		double acc = source_traj.joint_acc_array_(source_joint);

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

	contact_trajectory_ = source_traj.contact_trajectory_;
}

ItompCIOTrajectory::~ItompCIOTrajectory()
{
}

void ItompCIOTrajectory::overwriteTrajectory(const trajectory_msgs::JointTrajectory& traj)
{
	std::vector<int> ind;
	for (unsigned int j = 0; j < traj.joint_names.size(); j++)
	{
		int kdl_number = robot_model_->urdfNameToKdlNumber(traj.joint_names[j]);
		if (kdl_number == 0)
		{
			ROS_WARN_STREAM("Can't find kdl index for joint " << traj.joint_names[j]);
		}
		ind.push_back(kdl_number);
	}

	for (unsigned int i = 1; i <= traj.points.size(); i++)
	{
		for (unsigned int j = 0; j < traj.joint_names.size(); j++)
		{
			trajectory_(i, ind[j]) = traj.points[i - 1].positions[j];
		}
	}
}

void ItompCIOTrajectory::init()
{
	trajectory_ = Eigen::MatrixXd(num_points_, num_joints_);
	contact_trajectory_ = Eigen::MatrixXd(num_contact_phases_, num_contacts_);

	contact_start_points_.clear();
	contact_start_points_.push_back(0);
	int phase_stride = (int) (contact_phase_duration_ / discretization_ + 1E-7);
	for (int i = start_index_; i < end_index_ + 1; i += phase_stride)
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

void ItompCIOTrajectory::updateFromGroupTrajectory(const ItompCIOTrajectory& group_trajectory)
{
	int num_vars_free = end_index_ - start_index_ + 1;
	for (int i = 0; i < group_trajectory.planning_group_->num_joints_; i++)
	{
		int target_joint = group_trajectory.planning_group_->group_joints_[i].kdl_joint_index_;
		trajectory_.block(start_index_, target_joint, num_vars_free, 1) = group_trajectory.trajectory_.block(
				group_trajectory.start_index_, i, num_vars_free, 1);
	}

	int contact_end_index = num_contact_phases_ - 2;
	int contact_start_index = 1;
	int num_contact_vars_free = contact_end_index - contact_start_index + 1;
	for (int i = 0; i < group_trajectory.planning_group_->getNumContacts(); i++)
	{
		// TODO: need to be changed when multiple groups have contacts;
		int target_contact = i;
		contact_trajectory_.block(contact_start_index, target_contact, num_contact_vars_free, 1)
				= group_trajectory.contact_trajectory_.block(contact_start_index, i, num_contact_vars_free, 1);
	}
}

void ItompCIOTrajectory::fillInMinJerk(const std::set<int>& groupJointsKDLIndices,
		const Eigen::MatrixXd::RowXpr joint_vel_array, const Eigen::MatrixXd::RowXpr joint_acc_array)
{
	joint_vel_array_ = joint_vel_array;
	joint_acc_array_ = joint_acc_array;
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
		if (i == 3)
		{
			hasRotation = true;
			continue;
		}
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
		t[1] = (double) (i - start_index) / numPoints;//(i - start_index) * discretization_;
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
			t[1] = (double) (i - start_index) / numPoints;//(i - start_index) * discretization_;
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
	}
}

void ItompCIOTrajectory::fillInMinJerkWithMidPoint(const vector<double>& midPoint,
		const std::set<int>& groupJointsKDLIndices, int index)
{
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

		if (index != 0)// || i == 27 || i == 28 || i == 35 || i == 9)

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
}

}
