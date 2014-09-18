#include <itomp_cio_planner/trajectory/full_trajectory.h>
#include <itomp_cio_planner/trajectory/parameter_trajectory.h>
#include <itomp_cio_planner/util/joint_state_util.h>
#include <ecl/geometry/polynomial.hpp>
#include <moveit/robot_state/robot_state.h>

namespace itomp_cio_planner
{

FullTrajectory::FullTrajectory(const ItompRobotModelConstPtr& robot_model,
		double duration, double discretization, double keyframe_interval,
		bool has_velocity_and_acceleration, bool free_point_end,
		int num_contacts) :
		Trajectory(discretization, has_velocity_and_acceleration,
				has_velocity_and_acceleration), has_free_end_point_(
				free_point_end)
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

	backup_trajectory_[TRAJECTORY_TYPE_POSITION] = Eigen::MatrixXd(
			getNumPoints(), getNumElements());
	if (has_velocity_)
		backup_trajectory_[TRAJECTORY_TYPE_VELOCITY] = Eigen::MatrixXd(
				getNumPoints(), getNumElements());
	if (has_acceleration_)
		backup_trajectory_[TRAJECTORY_TYPE_ACCELERATION] = Eigen::MatrixXd(
				getNumPoints(), getNumElements());
}

void FullTrajectory::updateFromParameterTrajectory(
		const ParameterTrajectoryConstPtr& parameter_trajectory,
		const ItompPlanningGroupConstPtr& planning_group)
{
	copyFromParameterTrajectory(parameter_trajectory, planning_group, 0,
			parameter_trajectory->getNumPoints());
	updateTrajectoryFromKeyframes(0, num_keyframes_ - 1);
}

void FullTrajectory::directChangeForDerivatives(double value,
		const ItompPlanningGroupConstPtr& planning_group, int type, int point,
		int element, int& full_point_begin, int& full_point_end, bool backup)
{
	int keyframe_begin = std::max(point - 1, 0);
	// keyframe_end is included in the range
	int keyframe_end = std::min(point + 1, num_keyframes_ - 1);

	full_point_begin = keyframe_start_index_
			+ keyframe_begin * num_keyframe_interval_points_;
	full_point_end = std::min(num_points_,
			keyframe_start_index_ + keyframe_end * num_keyframe_interval_points_
					+ 1);

	int full_element = -1;
	int num_full_joints = getComponentSize(
			FullTrajectory::TRAJECTORY_COMPONENT_JOINT);
	int num_parameter_joints = planning_group->num_joints_;
	if (element < num_parameter_joints)
		full_element = planning_group->group_joints_[element].kdl_joint_index_;
	else
		full_element = element + (num_full_joints - num_parameter_joints);

	if (backup)
		backupTrajectories(full_point_begin, full_point_end, full_element);

	int keyframe_index = keyframe_start_index_
			+ point * num_keyframe_interval_points_;

	if (full_element < num_full_joints
			&& (keyframe_index == 0
					|| (keyframe_index == getNumPoints() - 1
							&& !has_free_end_point_)))
		return;

	// set value
	trajectory_[type](keyframe_index, full_element) = value;

	updateTrajectoryFromKeyframes(keyframe_begin, keyframe_end, full_element);
}

void FullTrajectory::copyFromParameterTrajectory(
		const ParameterTrajectoryConstPtr& parameter_trajectory,
		const ItompPlanningGroupConstPtr& planning_group,
		int parameter_point_begin, int parameter_point_end)
{
	int num_full_joints = getComponentSize(
			FullTrajectory::TRAJECTORY_COMPONENT_JOINT);
	int num_parameter_joints = parameter_trajectory->num_joints_;

	// copy joint parameters from parameter to full trajectory
	for (int i = 0; i < num_parameter_joints; ++i)
	{
		const ItompRobotJoint& joint = planning_group->group_joints_[i];
		bool has_joint_limits = joint.has_joint_limits_;
		bool wrap_around = joint.wrap_around_;
		double limit_min, limit_max;
		if (has_joint_limits)
		{
			limit_min = joint.joint_limit_min_;
			limit_max = joint.joint_limit_max_;
		}

		int full_joint_index =
				parameter_trajectory->group_to_full_joint_indices[i];
		for (int j = parameter_point_begin; j < parameter_point_end; ++j)
		{
			// don't copy initial / goal joints
			if (j == 0
					|| (j == parameter_point_end - 1 && !has_free_end_point_))
				continue;

			int keyframe_index = keyframe_start_index_
					+ j * num_keyframe_interval_points_;

			double value =
					parameter_trajectory->trajectory_[Trajectory::TRAJECTORY_TYPE_POSITION](
							j, i);

			if (has_joint_limits)
			{
				value = std::max(value, limit_min);
				value = std::min(value, limit_max);
			}
			if (wrap_around)
			{
				int prev_keyframe_index = keyframe_index
						- num_keyframe_interval_points_;
				double prev_key =
						trajectory_[Trajectory::TRAJECTORY_TYPE_POSITION](
								prev_keyframe_index, full_joint_index);
				while (value - prev_key > M_PI)
					value -= 2 * M_PI;
				while (value - prev_key < -M_PI)
					value += 2 * M_PI;
			}

			trajectory_[Trajectory::TRAJECTORY_TYPE_POSITION](keyframe_index,
					full_joint_index) = value;

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
		for (int j = parameter_point_begin; j < parameter_point_end; ++j)
		{
			int keyframe_index = keyframe_start_index_
					+ j * num_keyframe_interval_points_;

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

void FullTrajectory::updateTrajectoryFromKeyframes(int keyframe_begin,
		int keyframe_end)
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
		int trajectory_index = keyframe_start_index_
				+ keyframe_begin * num_keyframe_interval_points_ + 1;
		for (int k = keyframe_begin; k < keyframe_end; ++k)
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
							j), keyframe_interval_,
					trajectory_[TRAJECTORY_TYPE_POSITION](next_keyframe_index,
							j),
					trajectory_[TRAJECTORY_TYPE_VELOCITY](next_keyframe_index,
							j));

			for (int i = 1; i <= num_keyframe_interval_points_; ++i)
			{
				if (trajectory_index != getNumPoints() - 1
						|| has_free_end_point_)
				{
					double t = i * discretization_;
					trajectory_[TRAJECTORY_TYPE_POSITION](trajectory_index, j) =
							poly(t);
					trajectory_[TRAJECTORY_TYPE_VELOCITY](trajectory_index, j) =
							poly.derivative(t);
					trajectory_[TRAJECTORY_TYPE_ACCELERATION](trajectory_index,
							j) = poly.dderivative(t);
				}

				++trajectory_index;
			}
		}
	}
}

void FullTrajectory::updateTrajectoryFromKeyframes(int keyframe_begin,
		int keyframe_end, int element)
{
	if (num_keyframe_interval_points_ <= 1)
		return;

	ROS_ASSERT(has_velocity_ && has_acceleration_);

	// cubic interpolation of pos, vel, acc
	// update trajectory between (k, k+1]
	// acc is discontinuous at each keyframe

	// skip the initial position
	int trajectory_index = keyframe_start_index_
			+ keyframe_begin * num_keyframe_interval_points_ + 1;
	for (int k = keyframe_begin; k < keyframe_end; ++k)
	{
		ecl::CubicPolynomial poly;
		int cur_keyframe_index = keyframe_start_index_
				+ k * num_keyframe_interval_points_;
		int next_keyframe_index = cur_keyframe_index
				+ num_keyframe_interval_points_;

		poly = ecl::CubicPolynomial::DerivativeInterpolation(0.0,
				trajectory_[TRAJECTORY_TYPE_POSITION](cur_keyframe_index,
						element),
				trajectory_[TRAJECTORY_TYPE_VELOCITY](cur_keyframe_index,
						element), keyframe_interval_,
				trajectory_[TRAJECTORY_TYPE_POSITION](next_keyframe_index,
						element),
				trajectory_[TRAJECTORY_TYPE_VELOCITY](next_keyframe_index,
						element));

		for (int i = 1; i <= num_keyframe_interval_points_; ++i)
		{
			if (trajectory_index != getNumPoints() - 1 || has_free_end_point_)
			{
				double t = i * discretization_;
				trajectory_[TRAJECTORY_TYPE_POSITION](trajectory_index, element) =
						poly(t);
				trajectory_[TRAJECTORY_TYPE_VELOCITY](trajectory_index, element) =
						poly.derivative(t);
				trajectory_[TRAJECTORY_TYPE_ACCELERATION](trajectory_index,
						element) = poly.dderivative(t);
			}

			++trajectory_index;
		}
	}
}

void FullTrajectory::setStartState(const sensor_msgs::JointState &joint_state,
		const ItompRobotModelConstPtr& robot_model, bool fill_trajectory)
{
	ROS_INFO("Set the full trajectory start state");
	jointStateToArray(robot_model, joint_state,
			getTrajectoryPoint(0, Trajectory::TRAJECTORY_TYPE_POSITION),
			getTrajectoryPoint(0, Trajectory::TRAJECTORY_TYPE_VELOCITY),
			getTrajectoryPoint(0, Trajectory::TRAJECTORY_TYPE_ACCELERATION));

	for (unsigned int i = 0; i < joint_state.name.size(); i++)
	{
		std::string name = joint_state.name[i];
		int rbdl_number = robot_model->jointNameToRbdlNumber(name);
		if (rbdl_number >= 0)
		{
			getTrajectoryPoint(0, Trajectory::TRAJECTORY_TYPE_POSITION)(
					rbdl_number) = joint_state.position[i];
			getTrajectoryPoint(0, Trajectory::TRAJECTORY_TYPE_VELOCITY)(
					rbdl_number) = joint_state.velocity[i];
			getTrajectoryPoint(0, Trajectory::TRAJECTORY_TYPE_ACCELERATION)(
					rbdl_number) = joint_state.effort[i];
			ROS_INFO(
					"[%d] %s : %f %f %f", rbdl_number, name.c_str(), joint_state.position[i], joint_state.velocity[i], joint_state.effort[i]);
		}
	}

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
		const moveit_msgs::TrajectoryConstraints& trajectory_constraints,
		const moveit_msgs::Constraints& path_constraints,
		bool fill_trajectory_min_jerk)
{

	ROS_INFO(
			"Set the initial trajectory for planning group : %s", planning_group->name_.c_str());

	// set trajectory goal point
	int goal_index = getNumPoints() - 1;
	Eigen::MatrixXd::RowXpr goalPoint = getTrajectoryPoint(goal_index);
	for (int i = 0; i < planning_group->num_joints_; ++i)
	{
		std::string joint_name = planning_group->group_joints_[i].joint_name_;
		int rbdl_number = robot_model->jointNameToRbdlNumber(joint_name);
		if (rbdl_number >= 0)
		{
			double pos = joint_goal_state.position[rbdl_number];
			// wrap around
			if (planning_group->group_joints_[i].wrap_around_)
			{
				double start_pos = getTrajectoryPoint(0)(rbdl_number);
				while (pos - start_pos > M_PI)
					pos -= 2 * M_PI;
				while (pos - start_pos < -M_PI)
					pos += 2 * M_PI;
			}
			goalPoint(rbdl_number) = pos;
		}
	}

	// interpolate trajectory
	std::set<int> group_rbdl_joint_indices;
	for (int i = 0; i < planning_group->num_joints_; ++i)
	{
		group_rbdl_joint_indices.insert(
				planning_group->group_joints_[i].rbdl_joint_index_);
	}
	if (trajectory_constraints.constraints.size() != 0)
	{
		fillInMinJerk(group_rbdl_joint_indices, robot_model, planning_group,
				trajectory_constraints);
	}
	else if (path_constraints.position_constraints.size() == 0)
	{
		fillInMinJerk(group_rbdl_joint_indices);
	}
	else
	{
		fillInMinJerkCartesianTrajectory(robot_model, planning_group,
				path_constraints);
	}

	// print
	for (unsigned int i = 0; i < joint_goal_state.name.size(); i++)
	{
		std::string joint_name = joint_goal_state.name[i];
		int rbdl_number = robot_model->jointNameToRbdlNumber(joint_name);
		if (rbdl_number >= 0
				&& group_rbdl_joint_indices.find(rbdl_number)
						!= group_rbdl_joint_indices.end())
		{
			ROS_INFO(
					"[%d] %s : (%f %f %f) -> (%f %f %f)", rbdl_number, joint_name.c_str(), trajectory_[TRAJECTORY_TYPE_POSITION](0, rbdl_number), trajectory_[TRAJECTORY_TYPE_VELOCITY](0, rbdl_number), trajectory_[TRAJECTORY_TYPE_ACCELERATION](0, rbdl_number), trajectory_[TRAJECTORY_TYPE_POSITION](goal_index, rbdl_number), trajectory_[TRAJECTORY_TYPE_VELOCITY](goal_index, rbdl_number), trajectory_[TRAJECTORY_TYPE_ACCELERATION](goal_index, rbdl_number));
		}
	}
}

void FullTrajectory::fillInMinJerk(const std::set<int>& groupJointsKDLIndices)
{
	for (std::set<int>::const_iterator it = groupJointsKDLIndices.begin();
			it != groupJointsKDLIndices.end(); ++it)
	{
		int j = *it;

		double x0 = trajectory_[TRAJECTORY_TYPE_POSITION](0, j);
		double v0 = trajectory_[TRAJECTORY_TYPE_VELOCITY](0, j);
		double a0 = trajectory_[TRAJECTORY_TYPE_ACCELERATION](0, j);

		double x1 = trajectory_[TRAJECTORY_TYPE_POSITION](getNumPoints() - 1,
				j);
		double v1 = 0.0;
		double a1 = 0.0;

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

void FullTrajectory::fillInMinJerk(const std::set<int>& groupJointsKDLIndices,
		const ItompRobotModelConstPtr& robot_model,
		const ItompPlanningGroupConstPtr& planning_group,
		const moveit_msgs::TrajectoryConstraints& trajectory_constraints)
{
	int num_points = getNumPoints();
	int num_constraint_points = trajectory_constraints.constraints.size();
	int interval = num_points / (num_constraint_points + 1);

	int group_joint_index = 0;
	for (std::set<int>::const_iterator it = groupJointsKDLIndices.begin();
			it != groupJointsKDLIndices.end(); ++it)
	{
		int j = *it;

		bool has_constraints = false;
		int constraint_index = -1;
		for (int k = 0;
				k
						< trajectory_constraints.constraints[0].joint_constraints.size();
				++k)
		{
			if (trajectory_constraints.constraints[0].joint_constraints[k].joint_name
					== planning_group->group_joints_[group_joint_index].joint_name_)
			{
				has_constraints = true;
				constraint_index = k;
			}
		}

		if (!has_constraints)
		{
			double x0 = trajectory_[TRAJECTORY_TYPE_POSITION](0, j);
			double v0 = trajectory_[TRAJECTORY_TYPE_VELOCITY](0, j);
			double a0 = trajectory_[TRAJECTORY_TYPE_ACCELERATION](0, j);

			double x1 = trajectory_[TRAJECTORY_TYPE_POSITION](
					getNumPoints() - 1, j);
			double v1 = 0.0;
			double a1 = 0.0;

			ecl::QuinticPolynomial poly;
			poly = ecl::QuinticPolynomial::Interpolation(0, x0, v0, a0,
					duration_, x1, v1, a1);
			for (int i = 1; i < getNumPoints() - 1; ++i)
			{
				trajectory_[TRAJECTORY_TYPE_POSITION](i, j) = poly(
						i * discretization_);
				if (has_velocity_)
				{
					trajectory_[TRAJECTORY_TYPE_VELOCITY](i, j) =
							poly.derivative(i * discretization_);
				}
				if (has_acceleration_)
					trajectory_[TRAJECTORY_TYPE_ACCELERATION](i, j) =
							poly.dderivative(i * discretization_);
			}
		}
		else
		{
			// interpolate between waypoints
			for (int k = 0; k <= num_constraint_points; ++k)
			{
				double x0 =
						(k == 0 ?
								trajectory_[TRAJECTORY_TYPE_POSITION](0, j) :
								trajectory_constraints.constraints[k - 1].joint_constraints[constraint_index].position);
				double v0 =
						(k == 0 ?
								trajectory_[TRAJECTORY_TYPE_VELOCITY](0, j) : 0);
				double a0 = (
						k == 0 ?
								trajectory_[TRAJECTORY_TYPE_ACCELERATION](0,
										j) :
								0);

				double x1 =
						(k == num_constraint_points ?
								trajectory_[TRAJECTORY_TYPE_POSITION](
										getNumPoints() - 1, j) :
								trajectory_constraints.constraints[k].joint_constraints[constraint_index].position);
				double v1 = 0.0;
				double a1 = 0.0;

				ecl::QuinticPolynomial poly;
				poly = ecl::QuinticPolynomial::Interpolation(k * interval, x0,
						v0, a0, std::min((k + 1) * interval, num_points - 1),
						x1, v1, a1);
				for (int i = std::max(1, k * interval);
						i < std::min(((k + 1) * interval), getNumPoints() - 1);
						++i)
				{
					trajectory_[TRAJECTORY_TYPE_POSITION](i, j) = poly(i);
					if (has_velocity_)
					{
						trajectory_[TRAJECTORY_TYPE_VELOCITY](i, j) =
								poly.derivative(i);
					}
					if (has_acceleration_)
						trajectory_[TRAJECTORY_TYPE_ACCELERATION](i, j) =
								poly.dderivative(i);
				}

			}
		}
		++group_joint_index;
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
			"Interpolate the initial trajectory with CartesianPos from (%f, %f, %f)(%f %f) to (%f, %f, %f)(%f %f)", x0[0], x0[1], x0[2], v0, a0, x1[0], x1[1], x1[2], v1, a1);

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

void FullTrajectory::backupTrajectories(int point_begin, int point_end,
		int element)
{
	backup_point_begin_ = point_begin;
	backup_point_end_ = point_end;
	backup_element_ = element;
	int point_length = backup_point_end_ - backup_point_begin_;

	backup_trajectory_[Trajectory::TRAJECTORY_TYPE_POSITION].block(
			backup_point_begin_, backup_element_, point_length, 1) =
			trajectory_[Trajectory::TRAJECTORY_TYPE_POSITION].block(
					backup_point_begin_, backup_element_, point_length, 1);

	if (has_velocity_)
		backup_trajectory_[Trajectory::TRAJECTORY_TYPE_VELOCITY].block(
				backup_point_begin_, backup_element_, point_length, 1) =
				trajectory_[Trajectory::TRAJECTORY_TYPE_VELOCITY].block(
						backup_point_begin_, backup_element_, point_length, 1);

	if (has_acceleration_)
		backup_trajectory_[Trajectory::TRAJECTORY_TYPE_ACCELERATION].block(
				backup_point_begin_, backup_element_, point_length, 1) =
				trajectory_[Trajectory::TRAJECTORY_TYPE_ACCELERATION].block(
						backup_point_begin_, backup_element_, point_length, 1);
}

void FullTrajectory::restoreBackupTrajectories()
{
	int point_length = backup_point_end_ - backup_point_begin_;

	trajectory_[Trajectory::TRAJECTORY_TYPE_POSITION].block(backup_point_begin_,
			backup_element_, point_length, 1) =
			backup_trajectory_[Trajectory::TRAJECTORY_TYPE_POSITION].block(
					backup_point_begin_, backup_element_, point_length, 1);

	if (has_velocity_)
		trajectory_[Trajectory::TRAJECTORY_TYPE_VELOCITY].block(
				backup_point_begin_, backup_element_, point_length, 1) =
				backup_trajectory_[Trajectory::TRAJECTORY_TYPE_VELOCITY].block(
						backup_point_begin_, backup_element_, point_length, 1);

	if (has_acceleration_)
		trajectory_[Trajectory::TRAJECTORY_TYPE_ACCELERATION].block(
				backup_point_begin_, backup_element_, point_length, 1) =
				backup_trajectory_[Trajectory::TRAJECTORY_TYPE_ACCELERATION].block(
						backup_point_begin_, backup_element_, point_length, 1);
}

void FullTrajectory::setContactVariables(int point,
		const std::vector<Eigen::Vector3d>& contact_positions,
		const std::vector<Eigen::Vector3d>& contact_forces)
{
	int num_contacts = getComponentSize(
			FullTrajectory::TRAJECTORY_COMPONENT_CONTACT_POSITION) / 3;

	Eigen::MatrixXd::RowXpr row = getTrajectoryPoint(point);
	int position_start =
			component_start_indices_[FullTrajectory::TRAJECTORY_COMPONENT_CONTACT_POSITION];
	int force_start =
			component_start_indices_[FullTrajectory::TRAJECTORY_COMPONENT_CONTACT_FORCE];
	for (int i = 0; i < num_contacts; ++i)
	{
		row(position_start + i * 3) = contact_positions[i](0);
		row(position_start + i * 3 + 1) = contact_positions[i](1);
		row(position_start + i * 3 + 2) = contact_positions[i](2);

		row(force_start + i * 3) = contact_forces[i](0);
		row(force_start + i * 3 + 1) = contact_forces[i](1);
		row(force_start + i * 3 + 2) = contact_forces[i](2);
	}
}
void FullTrajectory::interpolateContactVariables()
{
	int start =
			component_start_indices_[FullTrajectory::TRAJECTORY_COMPONENT_CONTACT_POSITION];
	int end = component_start_indices_[FullTrajectory::TRAJECTORY_COMPONENT_NUM];
	for (int j = start; j < end; ++j)
	{
		double x0 = trajectory_[TRAJECTORY_TYPE_POSITION](0, j);
		double v0 = trajectory_[TRAJECTORY_TYPE_VELOCITY](0, j);
		double a0 = trajectory_[TRAJECTORY_TYPE_ACCELERATION](0, j);

		double x1 = trajectory_[TRAJECTORY_TYPE_POSITION](getNumPoints() - 1,
				j);
		double v1 =
				has_velocity_ ?
						trajectory_[TRAJECTORY_TYPE_VELOCITY](
								getNumPoints() - 1, j) :
						0.0;
		double a1 =
				has_acceleration_ ?
						trajectory_[TRAJECTORY_TYPE_ACCELERATION](
								getNumPoints() - 1, j) :
						0.0;

		ecl::QuinticPolynomial poly;
		poly = ecl::QuinticPolynomial::Interpolation(0, x0, v0, a0, duration_,
				x1, v1, a1);
		for (int i = 1; i < getNumPoints() - 1; ++i)
		{
			trajectory_[TRAJECTORY_TYPE_POSITION](i, j) = poly(0);
			//i * discretization_);
			if (has_velocity_)
			{
				trajectory_[TRAJECTORY_TYPE_VELOCITY](i, j) = poly.derivative(
						0);
				//i * discretization_);
			}
			if (has_acceleration_)
				trajectory_[TRAJECTORY_TYPE_ACCELERATION](i, j) =
						poly.dderivative(0); //i * discretization_);
		}
	}
}

void FullTrajectory::printTrajectory(bool position, bool velocity,
		bool acceleration) const
{
	const char* COMPONENT_NAMES[] =
	{ "Joints", "Contact Positions", "Contact Forces" };
	for (int c = 0; c < TRAJECTORY_COMPONENT_NUM; ++c)
	{
		int begin = component_start_indices_[c];
		int end = component_start_indices_[c + 1];
		printf("%s\n", COMPONENT_NAMES[c]);

		if (position)
		{
			printf("Position Trajectory\n");
			for (int i = 0; i < num_points_; ++i)
			{
				printf("%d : ", i);
				for (int j = begin; j < end; ++j)
				{
					printf("[%d]%.10f ", j,
							trajectory_[TRAJECTORY_TYPE_POSITION](i, j));
				}
				printf("\n");
			}
		}

		if (velocity && has_velocity_)
		{
			printf("Velocity Trajectory\n");
			for (int i = 0; i < num_points_; ++i)
			{
				printf("%d : ", i);
				for (int j = begin; j < end; ++j)
				{
					printf("[%d]%.10f ", j,
							trajectory_[TRAJECTORY_TYPE_VELOCITY](i, j));
				}
				printf("\n");
			}
		}
		if (acceleration && has_acceleration_)
		{
			printf("Acceleration Trajectory\n");
			for (int i = 0; i < num_points_; ++i)
			{
				printf("%d : ", i);
				for (int j = begin; j < end; ++j)
				{
					printf("[%d]%.10f ", j,
							trajectory_[TRAJECTORY_TYPE_ACCELERATION](i, j));
				}
				printf("\n");
			}
		}
	}
}

}

