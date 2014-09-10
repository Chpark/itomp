#include <itomp_cio_planner/trajectory/full_trajectory.h>
#include <itomp_cio_planner/trajectory/parameter_trajectory.h>
#include <itomp_cio_planner/util/joint_state_util.h>
#include <ecl/geometry/polynomial.hpp>

namespace itomp_cio_planner
{

FullTrajectory::FullTrajectory(const ItompRobotModelConstPtr& robot_model,
		double duration, double discretization, double keyframe_interval,
		bool has_velocity_and_acceleration, bool free_end_point,
		int num_contacts) :
		Trajectory(discretization, has_velocity_and_acceleration,
				has_velocity_and_acceleration), has_free_end_point_(
				free_end_point)
{
	// set num_elements & component_start_indices
	component_start_indices_[TRAJECTORY_COMPONENT_JOINT] = 0;
	num_elements_ += robot_model->getNumJoints();
	component_start_indices_[TRAJECTORY_COMPONENT_CONTACT_POSITION] =
			num_elements_;
	num_elements_ += num_contacts * 3;
	component_start_indices_[TRAJECTORY_COMPONENT_CONTACT_FORCE] =
			num_elements_;
	num_elements_ += num_contacts * 3;
	component_start_indices_[TRAJECTORY_COMPONENT_NUM] = num_elements_;

	// set keyframe-related variables
	keyframe_start_index_ = 0;
	// no keyframe
	if (keyframe_interval <= discretization)
	{
		num_keyframe_interval_points_ = 1;
		num_keyframes_ = num_points_ = safeDoubleToInt(
				duration / discretization) + 1;
		duration_ = (num_points_ - 1) * discretization;
	}
	else
	{
		num_keyframe_interval_points_ = safeDoubleToInt(
				keyframe_interval / discretization_);
		keyframe_interval_ = num_keyframe_interval_points_ * discretization_;
		if (keyframe_interval_ != keyframe_interval)
			ROS_INFO(
					"Trajectory keyframe interval modified : %f -> %f", keyframe_interval, keyframe_interval_);
		num_keyframes_ = safeDoubleToInt(duration / keyframe_interval_) + 1;
		num_points_ = (num_keyframes_ - 1) * num_keyframe_interval_points_ + 1;
		duration_ = (num_keyframes_ - 1) * keyframe_interval_;
	}
	if (duration_ != duration)
		ROS_INFO(
				"Trajectory duration modified : %f -> %f", duration, duration_);
}

FullTrajectory::~FullTrajectory()
{

}

void FullTrajectory::allocate()
{
	Trajectory::allocate();

	// set size of the vel/acc trajectory to 1 (used in setStartJointState)
	if (!has_velocity_)
		trajectory_[TRAJECTORY_TYPE_VELOCITY] = Eigen::MatrixXd(1,
				num_elements_);
	if (!has_acceleration_)
		trajectory_[TRAJECTORY_TYPE_ACCELERATION] = Eigen::MatrixXd(1,
				num_elements_);
}

void FullTrajectory::updateFromParameterTrajectory(
		const ParameterTrajectoryConstPtr& parameter_trajectory)
{
	copyFromParameterTrajectory(parameter_trajectory, 0, parameter_trajectory->getNumPoints());
	updateTrajectoryFromKeyframes(0, num_keyframes_);
}

void FullTrajectory::updateFromParameterTrajectory(
		const ParameterTrajectoryConstPtr& parameter_trajectory,
		int parameter_begin_point, int parameter_end_point,
		int& full_begin_point, int& full_end_point)
{
	copyFromParameterTrajectory(parameter_trajectory, parameter_begin_point,
			parameter_end_point);

	int keyframe_begin = std::max((parameter_begin_point + 1) - 1, 0);
	int keyframe_end = std::min((parameter_end_point + 1) + 1, num_keyframes_);

	updateTrajectoryFromKeyframes(keyframe_begin, keyframe_end);

	full_begin_point = keyframe_start_index_ + keyframe_begin * num_keyframe_interval_points_;
	full_end_point = keyframe_start_index_ + keyframe_end * num_keyframe_interval_points_;
}

void FullTrajectory::copyFromParameterTrajectory(
		const ParameterTrajectoryConstPtr& parameter_trajectory,
		int parameter_begin_point, int parameter_end_point)
{
	int num_full_joints = getComponentSize(
			FullTrajectory::TRAJECTORY_COMPONENT_JOINT);
	int num_parameter_joints = parameter_trajectory->num_joints_;

	// copy joint parameters from parameter to full trajectory
	for (int i = 0; i < num_parameter_joints; ++i)
	{
		int full_joint_index =
				parameter_trajectory->group_to_full_joint_indices[i];
		for (int j = parameter_begin_point; j < parameter_end_point; ++j)
		{
			int keyframe_index = keyframe_start_index_
					+ (j + 1) * num_keyframe_interval_points_;

			trajectory_[Trajectory::TRAJECTORY_TYPE_POSITION](keyframe_index,
					full_joint_index) =
					parameter_trajectory->trajectory_[Trajectory::TRAJECTORY_TYPE_POSITION](
							j, i);

			if (parameter_trajectory->has_velocity_)
			{
				trajectory_[Trajectory::TRAJECTORY_TYPE_VELOCITY](
						keyframe_index, full_joint_index) =
						parameter_trajectory->trajectory_[Trajectory::TRAJECTORY_TYPE_VELOCITY](
								j, i);
			}
		}
	}

	// copy other parameters from parameter to full trajectory
	int copy_size = parameter_trajectory->getNumElements()
			- num_parameter_joints;
	if (copy_size > 0)
	{
		for (int j = parameter_begin_point; j < parameter_end_point; ++j)
		{
			int keyframe_index = keyframe_start_index_
					+ (j + 1) * num_keyframe_interval_points_;

			trajectory_[Trajectory::TRAJECTORY_TYPE_POSITION].block(
					keyframe_index, num_full_joints, 1, copy_size) =
					parameter_trajectory->trajectory_[Trajectory::TRAJECTORY_TYPE_POSITION].block(
							j, num_parameter_joints, 1, copy_size);

			if (parameter_trajectory->has_velocity_)
			{
				trajectory_[Trajectory::TRAJECTORY_TYPE_VELOCITY].block(
						keyframe_index, num_full_joints, 1, copy_size) =
						parameter_trajectory->trajectory_[Trajectory::TRAJECTORY_TYPE_VELOCITY].block(
								j, num_parameter_joints, 1, copy_size);
			}
		}
	}
}

void FullTrajectory::updateTrajectoryFromKeyframes(int keyframe_begin, int keyframe_end)
{
	if (num_keyframe_interval_points_ <= 1)
		return;

	ROS_ASSERT(has_velocity_ && has_acceleration_);

	// cubic interpolation of pos, vel, acc
	// update trajectory between (k, k+1]
	// acc is discontinuous at each keyframe
	for (int j = 0; j < num_elements_; ++j)
	{
		// skip the initial position
		double trajectory_index = keyframe_start_index_ + 1;
		for (int k = keyframe_begin; k < keyframe_end - 1; ++k)
		{
			ecl::CubicPolynomial poly;
			int cur_keyframe_index = keyframe_start_index_
					+ k * num_keyframe_interval_points_;
			int next_keyframe_index = cur_keyframe_index
					+ num_keyframe_interval_points_;

			poly = ecl::CubicPolynomial::DerivativeInterpolation(0.0,
					trajectory_[TRAJECTORY_TYPE_POSITION](cur_keyframe_index,
							j),
					trajectory_[TRAJECTORY_TYPE_VELOCITY](cur_keyframe_index,
							j),

					keyframe_interval_,
					trajectory_[TRAJECTORY_TYPE_POSITION](next_keyframe_index,
							j),
					trajectory_[TRAJECTORY_TYPE_VELOCITY](next_keyframe_index,
							j));

			for (int i = 1; i <= num_keyframe_interval_points_; ++i)
			{
				trajectory_[TRAJECTORY_TYPE_POSITION](trajectory_index, j) =
						poly(discretization_ * i);
				trajectory_[TRAJECTORY_TYPE_VELOCITY](trajectory_index, j) =
						poly.derivative(discretization_ * i);
				trajectory_[TRAJECTORY_TYPE_ACCELERATION](trajectory_index, j) =
						poly.dderivative(discretization_ * i);

				++trajectory_index;
			}
		}
	}
}

void FullTrajectory::setStartJointState(
		const sensor_msgs::JointState &joint_state,
		const ItompRobotModelConstPtr& robot_model, bool fill_trajectory)
{
	jointStateToArray(robot_model, joint_state,
			getTrajectoryPoint(0, Trajectory::TRAJECTORY_TYPE_POSITION),
			getTrajectoryPoint(0, Trajectory::TRAJECTORY_TYPE_VELOCITY),
			getTrajectoryPoint(0, Trajectory::TRAJECTORY_TYPE_ACCELERATION));

	if (fill_trajectory)
	{
		for (int i = 1; i < getNumPoints(); ++i)
		{
			getTrajectoryPoint(i) = getTrajectoryPoint(0);
		}
	}
}

void FullTrajectory::setGroupGoalState(
		const sensor_msgs::JointState& joint_goal_state,
		const ItompPlanningGroupConstPtr& planning_group,
		const ItompRobotModelConstPtr& robot_model,
		const moveit_msgs::Constraints& path_constraints,
		bool fill_trajectory_min_jerk)
{
	// set trajectory goal point
	int goal_index = getNumPoints() - 1;
	Eigen::MatrixXd::RowXpr goalPoint = getTrajectoryPoint(goal_index);
	for (int i = 0; i < planning_group->num_joints_; ++i)
	{
		std::string joint_name = planning_group->group_joints_[i].joint_name_;
		int kdl_number = robot_model->jointNameToRbdlNumber(joint_name);
		if (kdl_number >= 0)
		{
			goalPoint(kdl_number) = joint_goal_state.position[kdl_number];
		}
	}

	// interpolate trajectory
	std::set<int> group_to_full_joint_indices;
	for (int i = 0; i < planning_group->num_joints_; ++i)
	{
		group_to_full_joint_indices.insert(
				planning_group->group_joints_[i].rbdl_joint_index_);
	}
	if (path_constraints.position_constraints.size() == 0)
	{
		fillInMinJerk(group_to_full_joint_indices);
	}
	else
	{
		fillInMinJerkCartesianTrajectory(robot_model, planning_group,
				path_constraints);
	}
}

void FullTrajectory::fillInMinJerk(const std::set<int>& groupJointsKDLIndices)
{
	for (std::set<int>::const_iterator it = groupJointsKDLIndices.begin();
			it != groupJointsKDLIndices.end(); ++it)
	{
		int j = *it;

		double x0 = trajectory_[TRAJECTORY_TYPE_POSITION](0, j);
		double v0 = (j < 6) ? trajectory_[TRAJECTORY_TYPE_VELOCITY](0, j) : 0.0;
		double a0 =
				(j < 6) ? trajectory_[TRAJECTORY_TYPE_ACCELERATION](0, j) : 0.0;

		double x1 = trajectory_[TRAJECTORY_TYPE_POSITION](getNumPoints() - 1,
				j);
		double v1 = 0.0;
		double a1 = 0.0;

		ROS_INFO(
				"Joint %d from %f(%f %f) to %f(%f %f)", j, x0, v0, a0, x1, v1, a1);

		ecl::QuinticPolynomial poly;
		poly = ecl::QuinticPolynomial::Interpolation(0, x0, v0, a0, duration_,
				x1, v1, a1);
		for (int i = 1; i < getNumPoints() - 1; ++i)
		{
			trajectory_[TRAJECTORY_TYPE_POSITION](i, j) = poly(
					i * discretization_);
			if (has_velocity_)
			{
				trajectory_[TRAJECTORY_TYPE_VELOCITY](i, j) = poly.derivative(
						i * discretization_);
			}
			if (has_acceleration_)
				trajectory_[TRAJECTORY_TYPE_ACCELERATION](i, j) =
						poly.dderivative(i * discretization_);
		}
	}
}

void FullTrajectory::fillInMinJerkCartesianTrajectory(
		const ItompRobotModelConstPtr& robot_model,
		const ItompPlanningGroupConstPtr& planning_group,
		const moveit_msgs::Constraints& path_constraints)
{

	robot_state::RobotStatePtr kinematic_state(
			new robot_state::RobotState(robot_model->getMoveitRobotModel()));
	const robot_state::JointModelGroup* joint_model_group =
			robot_model->getMoveitRobotModel()->getJointModelGroup(
					planning_group->name_);

	geometry_msgs::Vector3 start_position =
			path_constraints.position_constraints[0].target_point_offset;
	geometry_msgs::Vector3 goal_position =
			path_constraints.position_constraints[1].target_point_offset;
	geometry_msgs::Quaternion orientation =
			path_constraints.orientation_constraints[0].orientation;

	int num_joints = getComponentSize(TRAJECTORY_COMPONENT_JOINT);

	// set a state to the start config
	std::vector<double> positions(num_joints);
	for (std::size_t j = 0; j < num_joints; j++)
	{
		positions[j] = trajectory_[TRAJECTORY_TYPE_POSITION](0, j);
	}
	kinematic_state->setVariablePositions(&positions[0]);
	kinematic_state->update();

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

	for (int i = 0; i < getNumPoints(); ++i)
	{
		ecl::QuinticPolynomial poly[3];
		for (int d = 0; d < 3; ++d)
		{
			poly[d] = ecl::QuinticPolynomial::Interpolation(0, x0[d], v0, a0,
					duration_, x1[d], v1, a1);
		}
		geometry_msgs::Vector3 position;
		position.x = poly[0](i * discretization_);
		position.y = poly[1](i * discretization_);
		position.z = poly[2](i * discretization_);

		// Use IK to compute joint values
		std::vector<double> ik_solution(num_joints);

		Eigen::Affine3d end_effector_state = Eigen::Affine3d::Identity();
		Eigen::Quaternion<double> rot(orientation.w, orientation.x,
				orientation.y, orientation.z);
		Eigen::Vector3d trans(position.x, position.y, position.z);
		Eigen::Matrix3d mat = rot.toRotationMatrix();
		end_effector_state.linear() = mat;
		end_effector_state.translation() = trans;

		kinematics::KinematicsQueryOptions options;
		options.return_approximate_solution = false;
		bool found_ik = kinematic_state->setFromIK(joint_model_group,
				end_effector_state, 10, 0.1,
				moveit::core::GroupStateValidityCallbackFn(), options);
		if (found_ik)
		{
			//ROS_INFO("IK solution found for waypoint %d", i);

			std::vector<double> group_values;
			kinematic_state->copyJointGroupPositions(joint_model_group,
					group_values);
			double* state_pos = kinematic_state->getVariablePositions();
			ROS_ASSERT(num_joints == kinematic_state->getVariableCount());
			//printf("EE : (%f %f %f)(%f %f %f %f) : ", position.x, position.y, position.z, orientation.x, orientation.y,
			//  orientation.z, orientation.w);
			for (std::size_t k = 0; k < kinematic_state->getVariableCount();
					++k)
			{
				ik_solution[k] = state_pos[k];
				if (i != 0)
					trajectory_[TRAJECTORY_TYPE_POSITION](i, k) = state_pos[k];
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

}

