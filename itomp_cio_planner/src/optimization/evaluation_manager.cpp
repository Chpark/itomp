#include <ros/ros.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/PlanningScene.h>
#include <itomp_cio_planner/optimization/evaluation_manager.h>
#include <itomp_cio_planner/model/itomp_planning_group.h>
#include <itomp_cio_planner/contact/ground_manager.h>
#include <itomp_cio_planner/visualization/visualization_manager.h>
#include <itomp_cio_planner/contact/contact_force_solver.h>
#include <itomp_cio_planner/util/min_jerk_trajectory.h>
#include <itomp_cio_planner/util/planning_parameters.h>
#include <itomp_cio_planner/util/vector_util.h>
#include <itomp_cio_planner/util/multivariate_gaussian.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std;
using namespace Eigen;

const static int NUM_CONTACT_POINTS = 2;

namespace itomp_cio_planner
{
static int LeftLegStart = 0;

enum LEG_LINKS
{
	LEG_LINK_WAIST = 1,
	LEG_LINK_HIP_YAW = 2,
	LEG_LINK_HIP_ROLL = 3,
	LEG_LINK_HIP_PITCH = 4,
	LEG_LINK_KNEE_PITCH = 5,
	LEG_LINK_ANKLE_PITCH = 6,
	LEG_LINK_ANKLE_ROLL = 7,
	LEG_LINK_FOOT = 8,
	LEG_LINK_END_EFFECTOR = 9,
	LEG_LINK_NUM = 10,
};
enum LEG_JOINTS
{
	LEG_JOINT_WAIST = 1,
	LEG_JOINT_HIP_YAW = 2,
	LEG_JOINT_HIP_ROLL = 3,
	LEG_JOINT_HIP_PITCH = 4,
	LEG_JOINT_KNEE_PITCH = 5,
	LEG_JOINT_ANKLE_PITCH = 6,
	LEG_JOINT_ANKLE_ROLL = 7,
	LEG_JOINT_FOOT = 8,
	LEG_JOINT_END_EFFECTOR = 9,
	LEG_JOINT_NUM = 10,
};

static bool STABILITY_COST_VERBOSE = false;

EvaluationManager::EvaluationManager(int* iteration) :
		iteration_(iteration), data_(&default_data_), count_(0)
{
	print_debug_texts_ = false;
}

EvaluationManager::~EvaluationManager()
{

}

void EvaluationManager::initialize(ItompCIOTrajectory *full_trajectory,
		ItompCIOTrajectory *group_trajectory, ItompRobotModel *robot_model,
		const ItompPlanningGroup *planning_group, double planning_start_time,
		double trajectory_start_time,
		const moveit_msgs::Constraints& path_constraints,
		const planning_scene::PlanningSceneConstPtr& planning_scene)
{
	omp_set_num_threads(getNumParallelThreads());

	planning_start_time_ = planning_start_time;
	trajectory_start_time_ = trajectory_start_time;

	robot_model_ = robot_model;
	planning_group_ = planning_group;
	robot_name_ = robot_model_->getRobotName();

	// init some variables:
	num_joints_ = group_trajectory->getNumJoints();
	num_contacts_ = group_trajectory->getNumContacts();
	num_points_ = group_trajectory->getNumPoints();
	num_contact_points_ = group_trajectory->getNumContactPhases() + 1;

	num_vars_full_ = num_points_ - 2 * (DIFF_RULE_LENGTH - 2); // 111 - 2(7 - 2) = 101
	full_vars_start_ = (DIFF_RULE_LENGTH - 2); // 7 - 2 = 5
	full_vars_end_ = num_points_ - (DIFF_RULE_LENGTH - 2); // 111 - (7 - 2) = 106(next var of the last full traj var)

	// set up joint index:
	group_joint_to_kdl_joint_index_.resize(num_joints_);
	for (int i = 0; i < num_joints_; ++i)
		group_joint_to_kdl_joint_index_[i] =
				planning_group_->group_joints_[i].kdl_joint_index_;

	is_collision_free_ = false;
	last_trajectory_collision_free_ = false;

	vis_marker_pub_ =
			VisualizationManager::getInstance()->getVisualizationMarkerPublisher();
	vis_marker_array_pub_ =
			VisualizationManager::getInstance()->getVisualizationMarkerArrayPublisher();

	computeMassAndGravityForce();

	GroundManager::getInstance().init();

	default_data_.initialize(full_trajectory, group_trajectory, robot_model,
			planning_group, this, num_mass_segments_, path_constraints,
			planning_scene);

	timings_.resize(100, 0);
	for (int i = 0; i < 100; ++i)
		timings_[i] = 0;

	for (int i = 0; i < 3; ++i)
	{
		phaseJointArray_[i].resize(robot_model_->getKDLTree()->getNrOfJoints());
	}
}

double EvaluationManager::evaluate()
{
	// do forward kinematics:
	last_trajectory_collision_free_ = performForwardKinematics();

	//handleTrajectoryConstraint();

	computeTrajectoryValidity();
	last_trajectory_collision_free_ &= trajectory_validity_;

	computeWrenchSum();

	computeStabilityCosts();

	// TODO: reuse
	computeCollisionCosts();

	//computeFTRs();
	computeSingularityCosts();

	computeCartesianTrajectoryCosts();

	data_->costAccumulator_.compute(data_);

	last_trajectory_collision_free_ &= data_->costAccumulator_.isFeasible();

	// TODO: if trajectory is changed in handle joint limits,
	// update parameters

	return data_->costAccumulator_.getTrajectoryCost();
}

double EvaluationManager::evaluate(Eigen::VectorXd& costs)
{
	double ret = evaluate();

	//int num_vars_free = getGroupTrajectory()->getFreePoints().rows();

	// TODO
	int num_vars_free = num_points_ - 10 - 2;
	int start = 6;
	for (int i = 0; i < num_vars_free; i++)
	{
		costs(i) = data_->costAccumulator_.getWaypointCost(start + i);
	}

	return ret;
}

double EvaluationManager::evaluate(DERIVATIVE_VARIABLE_TYPE variable_type,
		int free_point_index, int joint_index)
{
	int stride = getGroupTrajectory()->getContactPhaseStride();
	int begin = (free_point_index - 1) * stride;
	int end = (free_point_index + 1) * stride;

	INIT_TIME_MEASUREMENT(10)

	// do forward kinematics:
	if (variable_type != DERIVATIVE_CONTACT_VARIABLE)
		performForwardKinematics(begin, end);

	ADD_TIMER_POINT

	computeTrajectoryValidity();

	ADD_TIMER_POINT

	if (variable_type != DERIVATIVE_CONTACT_VARIABLE)
		computeWrenchSum(begin, end + 1);

	ADD_TIMER_POINT

	computeStabilityCosts(begin, end + 1);

	ADD_TIMER_POINT

	if (variable_type != DERIVATIVE_CONTACT_VARIABLE)
		computeCollisionCosts(begin, end);

	ADD_TIMER_POINT

	computeFTRs(begin, end);

	data_->costAccumulator_.compute(data_);

	UPDATE_TIME
	PRINT_TIME(evaluate, 10000)

	return data_->costAccumulator_.getTrajectoryCost();
}

void EvaluationManager::setTrajectory(const Eigen::MatrixXd& parameters,
		const Eigen::MatrixXd& vel_parameters,
		const Eigen::MatrixXd& contact_parameters)
{
	// copy the parameters into group_trajectory:
	int num_free_points = parameters.rows();
	ROS_ASSERT(getGroupTrajectory()->getFreePoints().rows() == num_free_points);

	getGroupTrajectory()->getFreePoints() = parameters;
	getGroupTrajectory()->getFreeVelPoints() = vel_parameters;
	getGroupTrajectory()->getContactTrajectory() = contact_parameters;

	getGroupTrajectory()->updateTrajectoryFromFreePoints();

	// respect joint limits:
	handleJointLimits();

	// copy to full traj:
	updateFullTrajectory();
}

void EvaluationManager::setTrajectory(
		const std::vector<Eigen::VectorXd>& parameters,
		const std::vector<Eigen::VectorXd>& contact_parameters)
{
	// copy the parameters into group_trajectory:
	int cols = getGroupTrajectory()->getTrajectory().cols();
	for (int d = 0; d < num_joints_; ++d)
	{
		getGroupTrajectory()->getFreeJointTrajectoryBlock(d) = parameters[d];
	}

	cols = num_contact_points_;
	for (int d = 0; d < num_contacts_; ++d)
	{
		//getGroupTrajectory()->getContactTrajectory().block(i, 0, 1, cols) = contact_parameters[i];
		getGroupTrajectory()->getFreeContactTrajectoryBlock(d) =
				contact_parameters[d];
	}

	//getGroupTrajectory()->updateTrajectoryFromFreePoints();

	// respect joint limits:
	handleJointLimits();

	// copy to full traj:
	updateFullTrajectory();
}

void EvaluationManager::backupAndSetVariables(double new_value,
		DERIVATIVE_VARIABLE_TYPE variable_type, int free_point_index,
		int joint_index)
{
	// backup trajectory value
	Eigen::MatrixXd* target_trajectory;
	switch (variable_type)
	{
	case DERIVATIVE_POSITION_VARIABLE:
		target_trajectory = &getGroupTrajectory()->getFreePoints();
		break;

	case DERIVATIVE_VELOCITY_VARIABLE:
		target_trajectory = &getGroupTrajectory()->getFreeVelPoints();
		break;

	case DERIVATIVE_CONTACT_VARIABLE:
		target_trajectory = &getGroupTrajectory()->getContactTrajectory();
		break;
	}
	backup_data_.trajectory_value_ = (*target_trajectory)(free_point_index,
			joint_index);

	ROS_ASSERT(
			variable_type != DERIVATIVE_CONTACT_VARIABLE || new_value >= 0.0);

	// change trajectory variable
	(*target_trajectory)(free_point_index, joint_index) = new_value;

	// update trajectory
	if (variable_type != DERIVATIVE_CONTACT_VARIABLE)
	{
		getGroupTrajectory()->updateTrajectoryFromFreePoint(free_point_index,
				joint_index);
		handleJointLimits();

		// TODO: below function does not update free var trajectory
		updateFullTrajectory(free_point_index, joint_index);
	}

	int stride = getGroupTrajectory()->getContactPhaseStride();
	if (free_point_index == 0)
		free_point_index = 1;
	int begin = (free_point_index - 1) * stride;

	backup_data_.segment_frames_.resize(2 * stride);

	backup_data_.wrenchSum_.resize(2 * stride + 1);
	backup_data_.linkPositions_.resize(num_mass_segments_);
	backup_data_.linkVelocities_.resize(num_mass_segments_);
	backup_data_.linkAngularVelocities_.resize(num_mass_segments_);
	for (int i = 0; i < num_mass_segments_; ++i)
	{
		backup_data_.linkPositions_[i].resize(2 * stride + 1);
		backup_data_.linkVelocities_[i].resize(2 * stride + 1);
		backup_data_.linkAngularVelocities_[i].resize(2 * stride + 1);
	}
	backup_data_.CoMPositions_.resize(2 * stride + 1);
	backup_data_.CoMVelocities_.resize(2 * stride + 1);
	backup_data_.CoMAccelerations_.resize(2 * stride + 1);
	backup_data_.AngularMomentums_.resize(2 * stride + 1);
	backup_data_.Torques_.resize(2 * stride + 1);
	backup_data_.contactViolationVector_.resize(num_contacts_);
	backup_data_.contactPointVelVector_.resize(num_contacts_);
	for (int i = 0; i < num_contacts_; ++i)
	{
		backup_data_.contactViolationVector_[i].resize(2 * stride + 1);
		backup_data_.contactPointVelVector_[i].resize(2 * stride + 1);
	}

	backup_data_.state_contact_invariant_cost_.resize(2 * stride + 1);
	backup_data_.state_physics_violation_cost_.resize(2 * stride + 1);
	backup_data_.state_collision_cost_.resize(2 * stride);
	backup_data_.state_ftr_cost_.resize(2 * stride);

	for (int i = 0; i < 2 * stride; ++i)
	{
		backup_data_.segment_frames_[i] = data_->segment_frames_[begin + i];
	}

	memcpy(&backup_data_.wrenchSum_[0], &data_->wrenchSum_[begin],
			sizeof(KDL::Wrench) * 2 * stride + 1);
	for (int i = 0; i < num_mass_segments_; ++i)
	{
		memcpy(&backup_data_.linkPositions_[i][0],
				&data_->linkPositions_[i][begin],
				sizeof(KDL::Vector) * 2 * stride + 1);
		memcpy(&backup_data_.linkVelocities_[i][0],
				&data_->linkVelocities_[i][begin],
				sizeof(KDL::Vector) * 2 * stride + 1);
		memcpy(&backup_data_.linkAngularVelocities_[i][0],
				&data_->linkAngularVelocities_[i][begin],
				sizeof(KDL::Vector) * 2 * stride + 1);
	}
	memcpy(&backup_data_.CoMPositions_[0], &data_->CoMPositions_[begin],
			sizeof(KDL::Vector) * 2 * stride + 1);
	memcpy(&backup_data_.CoMVelocities_[0], &data_->CoMVelocities_[begin],
			sizeof(KDL::Vector) * 2 * stride + 1);
	memcpy(&backup_data_.CoMAccelerations_[0], &data_->CoMAccelerations_[begin],
			sizeof(KDL::Vector) * 2 * stride + 1);
	memcpy(&backup_data_.AngularMomentums_[0], &data_->AngularMomentums_[begin],
			sizeof(KDL::Vector) * 2 * stride + 1);
	memcpy(&backup_data_.Torques_[0], &data_->Torques_[begin],
			sizeof(KDL::Vector) * 2 * stride + 1);
	for (int i = 0; i < num_contacts_; ++i)
	{
		memcpy(&backup_data_.contactViolationVector_[i][0],
				&data_->contactViolationVector_[i][begin],
				sizeof(Vector4d) * 2 * stride + 1);
		memcpy(&backup_data_.contactPointVelVector_[i][0],
				&data_->contactPointVelVector_[i][begin],
				sizeof(KDL::Vector) * 2 * stride + 1);
	}

	memcpy(&backup_data_.state_contact_invariant_cost_[0],
			&data_->stateContactInvariantCost_[begin],
			sizeof(double) * 2 * stride + 1);
	memcpy(&backup_data_.state_physics_violation_cost_[0],
			&data_->statePhysicsViolationCost_[begin],
			sizeof(double) * 2 * stride + 1);
	memcpy(&backup_data_.state_collision_cost_[0],
			&data_->stateCollisionCost_[begin], sizeof(double) * 2 * stride);
	memcpy(&backup_data_.state_ftr_cost_[0], &data_->stateFTRCost_[begin],
			sizeof(double) * 2 * stride + 1);
}

void EvaluationManager::restoreVariable(DERIVATIVE_VARIABLE_TYPE variable_type,
		int free_point_index, int joint_index)
{
	Eigen::MatrixXd* target_trajectory;
	switch (variable_type)
	{
	case DERIVATIVE_POSITION_VARIABLE:
		target_trajectory = &getGroupTrajectory()->getFreePoints();
		break;

	case DERIVATIVE_VELOCITY_VARIABLE:
		target_trajectory = &getGroupTrajectory()->getFreeVelPoints();
		break;

	case DERIVATIVE_CONTACT_VARIABLE:
		target_trajectory = &getGroupTrajectory()->getContactTrajectory();
		break;
	}

	// restore trajectory value
	(*target_trajectory)(free_point_index, joint_index) =
			backup_data_.trajectory_value_;
	if (variable_type != DERIVATIVE_CONTACT_VARIABLE)
	{
		getGroupTrajectory()->updateTrajectoryFromFreePoint(free_point_index,
				joint_index);
		updateFullTrajectory(free_point_index, joint_index);
	}
	// restore variables
	int stride = getGroupTrajectory()->getContactPhaseStride();
	if (free_point_index == 0)
		free_point_index = 1;
	int begin = (free_point_index - 1) * stride;

	for (int i = 0; i < 2 * stride; ++i)
		data_->segment_frames_[begin + i] = backup_data_.segment_frames_[i];

	memcpy(&data_->wrenchSum_[begin], &backup_data_.wrenchSum_[0],
			sizeof(KDL::Wrench) * 2 * stride + 1);
	for (int i = 0; i < num_mass_segments_; ++i)
	{
		memcpy(&data_->linkPositions_[i][begin],
				&backup_data_.linkPositions_[i][0],
				sizeof(KDL::Vector) * 2 * stride + 1);
		memcpy(&data_->linkVelocities_[i][begin],
				&backup_data_.linkVelocities_[i][0],
				sizeof(KDL::Vector) * 2 * stride + 1);
		memcpy(&data_->linkAngularVelocities_[i][begin],
				&backup_data_.linkAngularVelocities_[i][0],
				sizeof(KDL::Vector) * 2 * stride + 1);
	}
	memcpy(&data_->CoMPositions_[begin], &backup_data_.CoMPositions_[0],
			sizeof(KDL::Vector) * 2 * stride + 1);
	memcpy(&data_->CoMVelocities_[begin], &backup_data_.CoMVelocities_[0],
			sizeof(KDL::Vector) * 2 * stride + 1);
	memcpy(&data_->CoMAccelerations_[begin], &backup_data_.CoMAccelerations_[0],
			sizeof(KDL::Vector) * 2 * stride + 1);
	memcpy(&data_->AngularMomentums_[begin], &backup_data_.AngularMomentums_[0],
			sizeof(KDL::Vector) * 2 * stride + 1);
	memcpy(&data_->Torques_[begin], &backup_data_.Torques_[0],
			sizeof(KDL::Vector) * 2 * stride + 1);
	for (int i = 0; i < num_contacts_; ++i)
	{
		memcpy(&data_->contactViolationVector_[i][begin],
				&backup_data_.contactViolationVector_[i][0],
				sizeof(Vector4d) * 2 * stride + 1);
		memcpy(&data_->contactPointVelVector_[i][begin],
				&backup_data_.contactPointVelVector_[i][0],
				sizeof(KDL::Vector) * 2 * stride + 1);
	}

	memcpy(&data_->stateContactInvariantCost_[begin],
			&backup_data_.state_contact_invariant_cost_[0],
			sizeof(double) * 2 * stride + 1);
	memcpy(&data_->statePhysicsViolationCost_[begin],
			&backup_data_.state_physics_violation_cost_[0],
			sizeof(double) * 2 * stride + 1);
	memcpy(&data_->stateCollisionCost_[begin],
			&backup_data_.state_collision_cost_[0],
			sizeof(double) * 2 * stride);
	memcpy(&data_->stateFTRCost_[begin], &backup_data_.state_ftr_cost_[0],
			sizeof(double) * 2 * stride + 1);
}

double EvaluationManager::evaluateDerivatives(double value,
		DERIVATIVE_VARIABLE_TYPE variable_type, int free_point_index,
		int joint_index)
{
	// backup old values and update trajectory
	backupAndSetVariables(value, variable_type, free_point_index, joint_index);

	// evaluate
	double cost = evaluate(variable_type, free_point_index, joint_index);

	restoreVariable(variable_type, free_point_index, joint_index);

	return cost;
}

void EvaluationManager::render(int trajectory_index, bool is_best)
{
	if (PlanningParameters::getInstance()->getAnimatePath())
	{
		VisualizationManager::getInstance()->animatePath(trajectory_index,
				getFullTrajectoryConst(), is_best);
	}

	if (PlanningParameters::getInstance()->getAnimateEndeffector())
	{
		VisualizationManager::getInstance()->animateEndeffector(
				trajectory_index, full_vars_start_, full_vars_end_,
				data_->segment_frames_, is_best);
		//VisualizationManager::getInstance()->animateCoM(num_vars_full_,
		//	full_vars_start_, data_->CoMPositions_, false);
	}

}

void EvaluationManager::computeMassAndGravityForce()
{
	total_mass_ = 0.0;
	const KDL::SegmentMap& segmentMap =
			robot_model_->getKDLTree()->getSegments();
	num_mass_segments_ = 0;
	for (KDL::SegmentMap::const_iterator it = segmentMap.begin();
			it != segmentMap.end(); ++it)
	{
		const KDL::Segment& segment = it->second.segment;
		double mass = segment.getInertia().getMass();
		if (mass == 0)
			continue;

		total_mass_ += mass;
		masses_.push_back(mass);

		++num_mass_segments_;
	}
	gravity_force_ = total_mass_ * KDL::Vector(0.0, 0.0, -9.8);

	// normalize gravity force to 1.0 and rescale masses
	gravity_force_ = KDL::Vector(0.0, 0.0, -1.0);
	for (int i = 0; i < masses_.size(); ++i)
		masses_[i] /= total_mass_ * 9.8;
	total_mass_ = 1.0 / 9.8;

}

void EvaluationManager::handleJointLimits()
{
	for (int joint = 0; joint < num_joints_; joint++)
	{
		if (!planning_group_->group_joints_[joint].has_joint_limits_)
			continue;

		double joint_max =
				planning_group_->group_joints_[joint].joint_limit_max_;
		double joint_min =
				planning_group_->group_joints_[joint].joint_limit_min_;

		int count = 0;

		for (int i = 1; i < num_points_ - 2; i++)
		{
			if ((*getGroupTrajectory())(i, joint) > joint_max)
			{
				(*getGroupTrajectory())(i, joint) = joint_max;
			}
			else if ((*getGroupTrajectory())(i, joint) < joint_min)
			{
				(*getGroupTrajectory())(i, joint) = joint_min;
			}
		}
	}
}

void EvaluationManager::handleTrajectoryConstraint()
{
	return;

	if (data_->cartesian_waypoints_.size() == 0)
		return;

	// TODO: temp
	// handle cartesian traj
	robot_state::RobotStatePtr kinematic_state(
			new robot_state::RobotState(robot_model_->getRobotModel()));
	const robot_state::JointModelGroup* joint_model_group =
			robot_model_->getRobotModel()->getJointModelGroup("lower_body");

	KDL::Vector start_pos = data_->cartesian_waypoints_[0].p;
	KDL::Vector end_pos = data_->cartesian_waypoints_[1].p;
	KDL::Vector dir = (end_pos - start_pos);
	dir.Normalize();
	KDL::Rotation orientation = data_->cartesian_waypoints_[0].M;

	int start = 6;

	Eigen::Matrix3d mat;
	for (int i = 0; i < 3; ++i)
		for (int j = 0; j < 3; ++j)
			mat(i, j) = orientation(i, j);

	// set state to the start config
	std::vector<double> positions(num_joints_);
	for (std::size_t k = 0; k < num_joints_; k++)
	{
		positions[k] = (*getGroupTrajectory())(5, k);
	}
	kinematic_state->setVariablePositions(&positions[0]);
	kinematic_state->update();

	const int END_EFFECTOR_SEGMENT_INDEX =
			robot_model_->getForwardKinematicsSolver()->segmentNameToIndex(
					"segment_7");
	int num_vars_free = num_points_ - 10 - 2;

	for (int i = start; i < start + num_vars_free; i++)
	{
		int full_traj_index = getGroupTrajectory()->getFullTrajectoryIndex(i);

		for (std::size_t k = 0; k < num_joints_; k++)
			positions[k] = (*getGroupTrajectory())(i, k);
		kinematic_state->setVariablePositions(&positions[0]);
		kinematic_state->update();

		KDL::Frame& frame =
				data_->segment_frames_[i][END_EFFECTOR_SEGMENT_INDEX];
		KDL::Vector proj = start_pos
				+ KDL::dot(dir, (frame.p - start_pos)) * dir;
		if (KDL::dot((proj - start_pos), dir) < 0)
			proj = start_pos;
		if (KDL::dot((proj - end_pos), dir) > 0)
			proj = end_pos;

		proj = start_pos
				+ (end_pos - start_pos) * (double) (i - start) / num_vars_free;

		double dist = (frame.p - proj).Norm();

		// Use IK to compute joint values
		vector<double> ik_solution(num_joints_);

		Eigen::Affine3d end_effector_state = Eigen::Affine3d::Identity();
		Eigen::Vector3d trans(proj.x(), proj.y(), proj.z());

		end_effector_state.linear() = mat;
		end_effector_state.translation() = trans;

		kinematics::KinematicsQueryOptions options;
		options.return_approximate_solution = false;
		bool found_ik = kinematic_state->setFromIK(joint_model_group,
				end_effector_state, 10, 0.1,
				moveit::core::GroupStateValidityCallbackFn(), options);
		if (found_ik)
		{
			std::vector<double> group_values;
			kinematic_state->copyJointGroupPositions(joint_model_group,
					group_values);
			double* state_pos = kinematic_state->getVariablePositions();
			for (std::size_t k = 0; k < kinematic_state->getVariableCount();
					++k)
			{
				ik_solution[k] = state_pos[k];
				(*getGroupTrajectory())(i, k) = state_pos[k];
				(*getFullTrajectory())(full_traj_index, k) = state_pos[k];
			}
		}
		else
		{
			ROS_INFO("Could not find IK solution for waypoint %d", i);
		}
	}
	performForwardKinematics();

	// check
	for (int i = start; i < start + num_vars_free; i++)
	{
		KDL::Frame& frame =
				data_->segment_frames_[i][END_EFFECTOR_SEGMENT_INDEX];
		KDL::Vector proj = start_pos
				+ KDL::dot(dir, (frame.p - start_pos)) * dir;
		double dist = (frame.p - proj).Norm();
		if (dist > 0.1)
			ROS_INFO("error : %d dist : %f", i, dist);
	}
}

void EvaluationManager::updateFullTrajectory()
{
	getFullTrajectory()->updateFromGroupTrajectory(*getGroupTrajectory());
}

void EvaluationManager::updateFullTrajectory(int point_index, int joint_index)
{
	getFullTrajectory()->updateFromGroupTrajectory(*getGroupTrajectory(),
			point_index, joint_index);
}

bool EvaluationManager::performForwardKinematics(int begin, int end)
{
	double invTime = 1.0 / getGroupTrajectory()->getDiscretization();
	double invTimeSq = invTime * invTime;

	is_collision_free_ = true;

	int safe_begin = max(0, begin);
	int safe_end = min(num_points_, end);

	// used in computeBaseFrames
	int full_traj_index = getGroupTrajectory()->getFullTrajectoryIndex(
			num_points_ - 1);
	getFullTrajectory()->getTrajectoryPointKDL(full_traj_index,
			data_->kdl_joint_array_);
	data_->fk_solver_.JntToCartFull(data_->kdl_joint_array_,
			data_->joint_pos_[num_points_ - 1],
			data_->joint_axis_[num_points_ - 1],
			data_->segment_frames_[num_points_ - 1]);

	// for each point in the trajectory
	for (int i = safe_begin; i < safe_end; ++i)
	{
		int full_traj_index = getGroupTrajectory()->getFullTrajectoryIndex(i);
		getFullTrajectory()->getTrajectoryPointKDL(full_traj_index,
				data_->kdl_joint_array_);
		// TODO: ?
		/*
		 // update kdl_joint_array with vel, acc
		 if (i < 1)
		 {
		 for (int j = 0; j < planning_group_->num_joints_; j++)
		 {
		 int target_joint = planning_group_->group_joints_[j].kdl_joint_index_;
		 data_->kdl_joint_array_(target_joint) = (*getGroupTrajectory())(i, j);
		 }
		 }
		 */

		//computeBaseFrames(data_->kdl_joint_array_, i);
		if (i == safe_begin)
			data_->fk_solver_.JntToCartFull(data_->kdl_joint_array_,
					data_->joint_pos_[i], data_->joint_axis_[i],
					data_->segment_frames_[i]);
		else
			data_->fk_solver_.JntToCartFull(data_->kdl_joint_array_,
					data_->joint_pos_[i], data_->joint_axis_[i],
					data_->segment_frames_[i]);
		// TODO: check patrial FK
		/*
		 data_->fk_solver_.JntToCartPartial(data_->kdl_joint_array_,
		 data_->joint_pos_[i], data_->joint_axis_[i],
		 data_->segment_frames_[i]);
		 */

		data_->state_is_in_collision_[i] = false;

		if (data_->state_is_in_collision_[i])
		{
			is_collision_free_ = false;
		}
	}

	return is_collision_free_;
}

void EvaluationManager::computeTrajectoryValidity()
{
	trajectory_validity_ = true;
	const double clearance = 0.001;
	int collisionBV = 8001;
	visualization_msgs::Marker marker;
	visualization_msgs::Marker markerTemp;

	for (int i = 1; i < num_points_ - 1; i++)
	{
		bool valid = true;

		/*
		 dynamic_obstacle_cost_(i) = 0;

		 int full_traj_index = group_trajectory_->getFullTrajectoryIndex(i);
		 full_trajectory_->getTrajectoryPointKDL(full_traj_index, kdl_joint_array_);
		 for (int j = 0; j < full_trajectory_->getNumJoints(); ++j)
		 {
		 robot_state_.joint_state.position[j] = kdl_joint_array_(j);
		 }

		 planning_environment::setRobotStateAndComputeTransforms(robot_state_, *kinematic_state_);

		 collision_space::EnvironmentModel
		 * env_model =
		 const_cast<collision_space::EnvironmentModel*> (collision_proximity_space_->getCollisionModelsInterface()->getOde());
		 env_model->updateRobotModel(&(*kinematic_state_));

		 // check collision points with dynamic obstacles
		 double point_time = full_trajectory_->getDiscretization() * (i - 1);
		 double cur_time = trajectory_start_time_ - planning_start_time_ + point_time;

		 // TODO: dynamic obs
		 double obstacleScale = 1.1 * (1 + cur_time * SENSOR_NOISE);
		 obstacleScale = 1.0;

		 bool inNextExecution = point_time < PlanningParameters::getInstance()->getPlanningStepSize();
		 for (unsigned int j = 0; j < dynamic_obstacles_->size(); ++j)
		 {
		 const pomp_dynamic_obs_msgs::DynamicObstacle& dynamic_obstacle = dynamic_obstacles_->at(j);

		 btVector3 origin(dynamic_obstacle.x, dynamic_obstacle.y, dynamic_obstacle.z);
		 double obstacle_radius = dynamic_obstacle.lengthX * 0.5 * obstacleScale;
		 }
		 */

		data_->state_validity_[i] = valid;
		if (!valid)
			trajectory_validity_ = false;

	}

}

void EvaluationManager::updateCoM(int point)
{
	const KDL::SegmentMap& segmentMap =
			robot_model_->getKDLTree()->getSegments();
	// compute CoM, p_j
	int mass_segment_index = 0;
	data_->CoMPositions_[point] = KDL::Vector::Zero();
	for (KDL::SegmentMap::const_iterator it = segmentMap.begin();
			it != segmentMap.end(); ++it)
	{
		const KDL::Segment& segment = it->second.segment;
		double mass = segment.getInertia().getMass();
		if (mass == 0.0)
			continue;
		mass = masses_[mass_segment_index];

		int sn = robot_model_->getForwardKinematicsSolver()->segmentNameToIndex(
				segment.getName());
		const KDL::Vector& pos = data_->segment_frames_[point][sn]
				* segment.getInertia().getCOG();

		data_->CoMPositions_[point] += pos * mass;
		data_->linkPositions_[mass_segment_index][point] = pos;

		++mass_segment_index;
	}
	data_->CoMPositions_[point] = data_->CoMPositions_[point] / total_mass_;

	if (STABILITY_COST_VERBOSE)
	{
		printf("[%d] CoM Pos : (%f %f %f)\n", point,
				data_->CoMPositions_[point].x(),
				data_->CoMPositions_[point].y(),
				data_->CoMPositions_[point].z());
	}
}

#include <iostream>
void EvaluationManager::computeWrenchSum(int begin, int end)
{
	if (planning_group_->name_ != "lower_body"
			&& planning_group_->name_ != "whole_body")
		return;

	int safe_begin = max(0, begin);
	int safe_end = min(num_points_, end);

	// compute CoM, p_j
	for (int point = safe_begin; point < safe_end; ++point)
	{
		updateCoM(point);
	}

	if (begin == full_vars_start_)
	{
		for (int i = 0; i < begin; ++i)
		{
			data_->CoMPositions_[i] = data_->CoMPositions_[begin];
			for (int j = 0; j < num_mass_segments_; ++j)
				data_->linkPositions_[j][i] = data_->linkPositions_[j][begin];
		}
	}
	if (end == full_vars_end_)
	{
		for (int i = end; i < num_points_; ++i)
		{
			data_->CoMPositions_[i] = data_->CoMPositions_[end - 1];
			for (int j = 0; j < num_mass_segments_; ++j)
				data_->linkPositions_[j][i] = data_->linkPositions_[j][end - 1];
		}
	}

	safe_begin = max(full_vars_start_ + 1, begin);
	safe_end = min(full_vars_end_ - 1, end);

	// compute \dot{CoM} \ddot{CoM}
	itomp_cio_planner::getVectorVelocitiesAndAccelerations(safe_begin,
			safe_end - 1, getGroupTrajectory()->getDiscretization(),
			data_->CoMPositions_, data_->CoMVelocities_,
			data_->CoMAccelerations_, KDL::Vector::Zero());
	// compute \dot{p_j}
	for (int i = 0; i < num_mass_segments_; ++i)
	{
		itomp_cio_planner::getVectorVelocities(safe_begin, safe_end - 1,
				getGroupTrajectory()->getDiscretization(),
				data_->linkPositions_[i], data_->linkVelocities_[i],
				KDL::Vector::Zero());
	}

	// debug
	if (STABILITY_COST_VERBOSE)
	{
		printf("CoMPos CoMVel CoMAcc \n");
		for (int i = safe_begin; i < safe_end; ++i)
		{
			printf("[%d] %f %f %f %f %f %f %f %f %f\n", i,
					data_->CoMPositions_[i].x(), data_->CoMPositions_[i].y(),
					data_->CoMPositions_[i].z(), data_->CoMVelocities_[i].x(),
					data_->CoMVelocities_[i].y(), data_->CoMVelocities_[i].z(),
					data_->CoMAccelerations_[i].x(),
					data_->CoMAccelerations_[i].y(),
					data_->CoMAccelerations_[i].z());
		}
	}

	// TODO: compute angular velocities = (cur-prev)/time
	const KDL::SegmentMap& segment_map =
			robot_model_->getKDLTree()->getSegments();
	const double inv_time = 1.0 / getGroupTrajectory()->getDiscretization();
	for (int point = safe_begin; point < safe_end; ++point)
	{
		int mass_segment_index = 0;
		for (KDL::SegmentMap::const_iterator it = segment_map.begin();
				it != segment_map.end(); ++it)
		{
			const KDL::Segment& segment = it->second.segment;
			double mass = segment.getInertia().getMass();
			if (mass == 0.0)
				continue;
			mass = masses_[mass_segment_index];

			int sn =
					robot_model_->getForwardKinematicsSolver()->segmentNameToIndex(
							segment.getName());
			const KDL::Vector& pos = data_->segment_frames_[point][sn]
					* segment.getInertia().getCOG();
			const KDL::Rotation& prev_rotation = data_->segment_frames_[point
					- 1][sn].M;
			const KDL::Rotation& cur_rotation =
					data_->segment_frames_[point][sn].M;
			const KDL::Rotation& rot_diff = cur_rotation
					* prev_rotation.Inverse();
			data_->linkAngularVelocities_[mass_segment_index][point] =
					rot_diff.GetRot() * inv_time;
			++mass_segment_index;
		}
	}

	// compute angular momentum
	//data_->AngularMomentums_[0] = KDL::Vector(0.0, 0.0, 0.0);
	//data_->AngularMomentums_[num_points_ - 1] = KDL::Vector(0.0, 0.0, 0.0);
	for (int point = safe_begin; point < safe_end; ++point)
	{
		data_->AngularMomentums_[point] = KDL::Vector(0.0, 0.0, 0.0);

		int mass_segment_index = 0;
		for (KDL::SegmentMap::const_iterator it = segment_map.begin();
				it != segment_map.end(); ++it)
		{
			const KDL::Segment& segment = it->second.segment;
			double mass = segment.getInertia().getMass();
			if (mass == 0.0)
				continue;
			mass = masses_[mass_segment_index];

			int sn =
					robot_model_->getForwardKinematicsSolver()->segmentNameToIndex(
							segment.getName());
			KDL::Vector angularVelTerm = (data_->segment_frames_[point][sn]
					* segment.getInertia()).getRotationalInertia()
					* data_->linkAngularVelocities_[mass_segment_index][point];

			data_->AngularMomentums_[point] += mass
					* (data_->linkPositions_[mass_segment_index][point]
							- data_->CoMPositions_[point])
					* data_->linkVelocities_[mass_segment_index][point]
					+ angularVelTerm;
			++mass_segment_index;
		}
	}
	// compute torques
	itomp_cio_planner::getVectorVelocities(safe_begin, safe_end - 1,
			getGroupTrajectory()->getDiscretization(), data_->AngularMomentums_,
			data_->Torques_, KDL::Vector::Zero());

	// compute wrench sum (gravity wrench + inertia wrench)
	for (int point = safe_begin; point < safe_end; ++point)
	{
		data_->wrenchSum_[point].force = gravity_force_;
		data_->wrenchSum_[point].torque = data_->CoMPositions_[point]
				* gravity_force_;

		data_->wrenchSum_[point].force += -total_mass_
				* data_->CoMAccelerations_[point];
		data_->wrenchSum_[point].torque += data_->CoMPositions_[point]
				* (-total_mass_ * data_->CoMAccelerations_[point]);
		//data_->wrenchSum_[point].torque += -data_->Torques_[point];

		if (STABILITY_COST_VERBOSE)
		{
			ROS_INFO(
					"[%d] CoM pos:(%f %f %f)", point, data_->CoMPositions_[point].x(), data_->CoMPositions_[point].y(), data_->CoMPositions_[point].z());
			ROS_INFO(
					"[%d] CoM acc:(%f %f %f)", point, data_->CoMAccelerations_[point].x(), data_->CoMAccelerations_[point].y(), data_->CoMAccelerations_[point].z());
			ROS_INFO(
					"[%d] Ang mon:(%f %f %f)", point, data_->AngularMomentums_[point].x(), data_->AngularMomentums_[point].y(), data_->AngularMomentums_[point].z());
			ROS_INFO(
					"[%d] Com Tor:(%f %f %f)", point, data_->Torques_[point].x(), data_->Torques_[point].y(), data_->Torques_[point].z());
			ROS_INFO(
					"[%d] Wre For:(%f %f %f)", point, data_->wrenchSum_[point].force.x(), data_->wrenchSum_[point].force.y(), data_->wrenchSum_[point].force.z());
			ROS_INFO(
					"[%d] Wre Tor:(%f %f %f)=(%f %f %f)x(%f %f %f)+(%f %f %f)x%f(%f %f %f)-(%f %f %f)", point, data_->wrenchSum_[point].torque.x(), data_->wrenchSum_[point].torque.y(), data_->wrenchSum_[point].torque.z(), data_->CoMPositions_[point].x(), data_->CoMPositions_[point].y(), data_->CoMPositions_[point].z(), gravity_force_.x(), gravity_force_.y(), gravity_force_.z(), data_->CoMPositions_[point].x(), data_->CoMPositions_[point].y(), data_->CoMPositions_[point].z(), total_mass_, data_->CoMAccelerations_[point].x(), data_->CoMAccelerations_[point].y(), data_->CoMAccelerations_[point].z(), data_->Torques_[point].x(), data_->Torques_[point].y(), data_->Torques_[point].z());
		}

	}

	for (int i = 0; i < planning_group_->getNumContacts(); ++i)
	{
		planning_group_->contactPoints_[i].updateContactViolationVector(
				safe_begin, safe_end - 1,
				getGroupTrajectory()->getDiscretization(),
				data_->contactViolationVector_[i],
				data_->contactPointVelVector_[i], data_->segment_frames_,
				data_->planning_scene_);

		for (int point = safe_begin; point < safe_end; ++point)
		{
			if (getGroupTrajectory()->getContactPhaseEndPoint(point) == point)
			{
				data_->contactPointVelVector_[i][point] = KDL::Vector::Zero();
			}
		}
	}
}

void EvaluationManager::computeStabilityCosts(int begin, int end)
{
	int safe_begin = max(full_vars_start_ + 1, begin);
	int safe_end = min(full_vars_end_ - 1, end);
	for (int point = safe_begin; point < safe_end; point++)
	{
		INIT_TIME_MEASUREMENT(10)
		ADD_TIMER_POINT

		double state_contact_invariant_cost = 0.0;
		double state_physics_violation_cost = 0.0;
		if (planning_group_->name_ != "lower_body"
				&& planning_group_->name_ != "whole_body")
		{
			data_->stateContactInvariantCost_[point] =
					state_contact_invariant_cost;
			data_->statePhysicsViolationCost_[point] =
					state_physics_violation_cost;
			continue;
		}

		int num_contacts = planning_group_->getNumContacts();
		if (num_contacts == 0)
			return;

		std::vector<KDL::Frame> contact_parent_frames(num_contacts);
		std::vector<double> contact_values(num_contacts);
		std::vector<KDL::Vector> contact_positions(num_contacts);
		for (int i = 0; i < num_contacts; ++i)
		{
			KDL::SegmentMap::const_iterator it_segment_link =
					robot_model_->getKDLTree()->getSegment(
							planning_group_->contactPoints_[i].getLinkName());
			it_segment_link = it_segment_link->second.parent;
			string parent_segment_name = it_segment_link->first;
			int segment_number =
					robot_model_->getForwardKinematicsSolver()->segmentNameToIndex(
							parent_segment_name);
			contact_parent_frames[i] =
					data_->segment_frames_[point][segment_number];

			planning_group_->contactPoints_[i].getPosition(point,
					contact_positions[i], data_->segment_frames_);
		}

		int phase = getGroupTrajectory()->getContactPhase(point);
		for (int i = 0; i < num_contacts; ++i)
			contact_values[i] = getGroupTrajectory()->getContactValue(phase, i);

		// test
		if (point <= full_vars_start_ || point >= full_vars_end_ - 1)
		{
			contact_values[0] = 10.0;
			contact_values[1] = 10.0;
		}
		else
		{
			contact_values[0] = (phase + LeftLegStart) % 2 == 0 ? 10.0 : 0.0;
			contact_values[1] = (phase + LeftLegStart) % 2 == 0 ? 0.0 : 10.0;
		}

		ADD_TIMER_POINT

		data_->contact_force_solver_(
				PlanningParameters::getInstance()->getFrictionCoefficient(),
				data_->contact_forces_[point], contact_positions,
				data_->wrenchSum_[point], contact_values,
				contact_parent_frames);

		ADD_TIMER_POINT

		for (int i = 0; i < num_contacts; ++i)
		{
			double cost = 0.0;
			for (int j = 0; j < 4; ++j)
				cost += data_->contactViolationVector_[i][point].data_[j]
						* data_->contactViolationVector_[i][point].data_[j];
			cost += 16.0
					* KDL::dot(data_->contactPointVelVector_[i][point],
							data_->contactPointVelVector_[i][point]);
			state_contact_invariant_cost += contact_values[i] * cost;
		}

		KDL::Wrench contactWrench;
		for (int i = 0; i < num_contacts; ++i)
		{
			contactWrench.force += data_->contact_forces_[point][i];
			contactWrench.torque += contact_positions[i]
					* data_->contact_forces_[point][i];
		}

		if (STABILITY_COST_VERBOSE)
		{
			printf("\n");

			KDL::Vector root_pos = data_->segment_frames_[point][3].p;
			printf("%d Root : (%f %f %f) CoM : (%f %f %f)\n", point,
					root_pos.x(), root_pos.y(), root_pos.z(),
					data_->CoMPositions_[point].x(),
					data_->CoMPositions_[point].y(),
					data_->CoMPositions_[point].z());
			for (int i = 0; i < num_contacts; ++i)
			{
				KDL::Vector rel_pos = (contact_positions[i]
						- data_->CoMPositions_[point]);
				KDL::Vector contact_torque = rel_pos
						* data_->contact_forces_[point][i];
				printf(
						"CP %d V:%f F:(%f %f %f) RT:(%f %f %f)xF=(%f %f %f) r:(%f %f %f) p:(%f %f %f)\n",
						i, contact_values[i],
						data_->contact_forces_[point][i].x(),
						data_->contact_forces_[point][0].y(),
						data_->contact_forces_[point][i].z(), rel_pos.x(),
						rel_pos.y(), rel_pos.z(), contact_torque.x(),
						contact_torque.y(), contact_torque.z(),
						contact_parent_frames[i].p.x(),
						contact_parent_frames[i].p.y(),
						contact_parent_frames[i].p.z(),
						contact_positions[i].x(), contact_positions[i].y(),
						contact_positions[i].z());
			}
		}

		KDL::Wrench violation = contactWrench + data_->wrenchSum_[point];
		state_physics_violation_cost = sqrt(
				violation.force.x() * violation.force.x()
						+ violation.force.y() * violation.force.y()
						+ violation.force.z() * violation.force.z()
						+ violation.torque.x() * violation.torque.x()
						+ violation.torque.y() * violation.torque.y()
						+ violation.torque.z() * violation.torque.z());

		if (STABILITY_COST_VERBOSE)
		{
			printf("Gravity Force : (%f %f %f)\n", gravity_force_.x(),
					gravity_force_.y(), gravity_force_.z());
			printf("Inertia Force : (%f %f %f)\n",
					-total_mass_ * data_->CoMAccelerations_[point].x(),
					-total_mass_ * data_->CoMAccelerations_[point].y(),
					-total_mass_ * data_->CoMAccelerations_[point].z());

			printf("Wrench Torque : (%f %f %f)\n",
					data_->wrenchSum_[point].torque.x(),
					data_->wrenchSum_[point].torque.y(),
					data_->wrenchSum_[point].torque.z());

			printf("Violation : (%f %f %f) (%f %f %f)\n", violation.force.x(),
					violation.force.y(), violation.force.z(),
					violation.torque.x(), violation.torque.y(),
					violation.torque.z());

			for (int i = 0; i < num_contacts; ++i)
			{
				printf("CP %d violation (%f %f %f %f) vel (%f %f %f)\n", i,
						data_->contactViolationVector_[i][point].data_[0],
						data_->contactViolationVector_[i][point].data_[1],
						data_->contactViolationVector_[i][point].data_[2],
						data_->contactViolationVector_[i][point].data_[3],
						data_->contactPointVelVector_[i][point].x(),
						data_->contactPointVelVector_[i][point].y(),
						data_->contactPointVelVector_[i][point].z());
			}

			printf("[%d] contactWrench (%f %f %f)(%f %f %f)\n", point,
					contactWrench.force.x(), contactWrench.force.y(),
					contactWrench.force.z(), contactWrench.torque.x(),
					contactWrench.torque.y(), contactWrench.torque.z());
			printf("[%d] violation (%f %f %f)(%f %f %f)\n", point,
					violation.force.x(), violation.force.y(),
					violation.force.z(), violation.torque.x(),
					violation.torque.y(), violation.torque.z());

			printf("[%d]CIcost:%f Pvcost:%f(%f,%f,%f,%f,%f,%f)\n", point,
					state_contact_invariant_cost, state_physics_violation_cost,
					violation.force.x(), violation.force.y(),
					violation.force.z(), violation.torque.x(),
					violation.torque.y(), violation.torque.z());
		}

		data_->stateContactInvariantCost_[point] = state_contact_invariant_cost;
		data_->statePhysicsViolationCost_[point] = state_physics_violation_cost;

ADD_TIMER_POINT	UPDATE_TIME
	PRINT_TIME(stability, 10000)
}

}

void EvaluationManager::computeCollisionCosts(int begin, int end)
{
	int num_all_joints = data_->kinematic_state_[0]->getVariableCount();

	int num_threads = getNumParallelThreads();

	collision_detection::CollisionRequest collision_request;
	collision_request.verbose = false;
	collision_request.contacts = true;
	collision_request.max_contacts = 1000;

	std::vector<collision_detection::CollisionResult> collision_result(
			num_threads);
	std::vector<std::vector<double> > positions(num_threads);

	for (int i = 0; i < num_threads; ++i)
	{
		positions[i].resize(num_all_joints);
	}

	int safe_begin = max(0, begin);
	int safe_end = min(num_points_, end);
#pragma omp parallel for
	for (int i = safe_begin; i < safe_end; ++i)
	{
		int thread_num = omp_get_thread_num();

		double depthSum = 0.0;

		int full_traj_index = getGroupTrajectory()->getFullTrajectoryIndex(i);
		for (std::size_t k = 0; k < num_all_joints; k++)
		{
			positions[thread_num][k] = (*getFullTrajectory())(full_traj_index,
					k);
		}
		data_->kinematic_state_[thread_num]->setVariablePositions(
				&positions[thread_num][0]);
		data_->planning_scene_->checkCollisionUnpadded(collision_request,
				collision_result[thread_num],
				*data_->kinematic_state_[thread_num]);

		const collision_detection::CollisionResult::ContactMap& contact_map =
				collision_result[thread_num].contacts;
		for (collision_detection::CollisionResult::ContactMap::const_iterator it =
				contact_map.begin(); it != contact_map.end(); ++it)
		{
			const collision_detection::Contact& contact = it->second[0];

			depthSum += contact.depth;

			// for debug
			//ROS_INFO("[%d] Collision between %s and %s : %f", i, contact.body_name_1.c_str(), contact.body_name_2.c_str(), contact.depth);

			last_trajectory_collision_free_ = false;
		}
		collision_result[thread_num].clear();
		data_->stateCollisionCost_[i] = depthSum;
	}
}

std::vector<double> computeFTR(const std::string& group_name,
		int contact_point_index, int begin, int end, const EvaluationData* data,
		const ItompPlanningGroup * planning_group)
{
	std::vector<double> positions, trajectory_ftrs;
	int num_joints = data->getFullTrajectory()->getNumJoints();
	positions.resize(num_joints);
	for (int i = begin; i < end; ++i)
	{
		int full_traj_index =
				data->getGroupTrajectory()->getFullTrajectoryIndex(i);
		double cost = 0;
		for (std::size_t k = 0; k < num_joints; k++)
		{
			positions[k] = (*data->getFullTrajectory())(full_traj_index, k);
		}
		data->kinematic_state_[0]->setVariablePositions(&positions[0]);
		robot_model::RobotModelConstPtr robot_model_ptr =
				data->getItompRobotModel()->getRobotModel();
		Eigen::MatrixXd jacobianFull = (data->kinematic_state_[0]->getJacobian(
				robot_model_ptr->getJointModelGroup(group_name)));
		Eigen::MatrixXd jacobian = jacobianFull.block(0, 0, 3,
				jacobianFull.cols());
		Eigen::MatrixXd jacobian_transpose = jacobian.transpose();

		// computing direction, first version as COM velocity between poses
		const KDL::Vector& dir_kdl =
				data->contact_forces_[i][contact_point_index];
		Eigen::Vector3d direction(dir_kdl.x(), dir_kdl.y(), dir_kdl.z());
		if (direction.norm() != 0)
		{
			direction.normalize();
			double ftr = 1
					/ std::sqrt(
							direction.transpose()
									* (jacobian * jacobian_transpose)
									* direction);
			KDL::Vector position, unused, normal;
			planning_group->contactPoints_[contact_point_index].getPosition(i,
					position, data->segment_frames_);
			GroundManager::getInstance().getNearestGroundPosition(position,
					unused, normal, data->planning_scene_); // TODO get more accurate normal
			Eigen::Vector3d normalEigen(normal.x(), normal.y(), normal.z());
			double contact_variable =
					data->getGroupTrajectory()->getContactTrajectory()(
							i
									/ data->getGroupTrajectory()->getContactPhaseStride(),
							contact_point_index);

			ftr *= -direction.dot(normalEigen);
			// bound value btw -10 and 10, then 0 and 1
			ftr = (ftr < -10) ? -10 : ftr;
			ftr = (ftr > 10) ? 10 : ftr;
			ftr = (ftr + 10) / 20;
			cost = dir_kdl.Norm() - ftr;
			cost = (cost < 0) ? 0 : cost;
		}
		trajectory_ftrs.push_back(cost);
	}
	return trajectory_ftrs;
}

void EvaluationManager::computeFTRs(int begin, int end)
{
	int safe_begin = max(0, begin);
	int safe_end = min(num_points_, end);
	std::vector<double> left_leg_cost = computeFTR("left_leg", 0, safe_begin,
			safe_end, data_, planning_group_);
	std::vector<double> right_leg_cost = computeFTR("right_leg", 1, safe_begin,
			safe_end, data_, planning_group_);
	std::vector<double> left_arm_cost = computeFTR("left_arm", 2, safe_begin,
			safe_end, data_, planning_group_);
	std::vector<double> right_arm_cost = computeFTR("right_arm", 3, safe_begin,
			safe_end, data_, planning_group_);
	for (unsigned int i = safe_begin; i < safe_end; ++i)
	{
		int v_index = i - safe_begin;
		data_->stateFTRCost_[i] = (left_leg_cost[v_index]
				+ right_leg_cost[v_index])
				+ 0.5 * (left_arm_cost[v_index] + right_arm_cost[v_index]);
	}

}

void EvaluationManager::printDebugInfo()
{
	if (PlanningParameters::getInstance()->getCartesianTrajectoryCostWeight()
			== 0.0)
		return;

	if (data_->cartesian_waypoints_.size() != 0)
	{
		// position constraint

		const int END_EFFECTOR_SEGMENT_INDEX =
				robot_model_->getForwardKinematicsSolver()->segmentNameToIndex(
						"tcp_2_link");

		data_->costAccumulator_.is_last_trajectory_valid_ = true;

		const double rotation_weight = 0.0;

		int num_vars_free = 100;

		if (data_->cartesian_waypoints_.size() == 0)
			return;

		KDL::Vector start_pos = data_->cartesian_waypoints_[0].p;
		KDL::Vector end_pos = data_->cartesian_waypoints_[1].p;
		KDL::Vector dir = (end_pos - start_pos);
		double dir_max = dir.Normalize();

		double max_dist = 0;

		// TODO: fix
		int point_index = 6;
		for (int i = point_index; i < point_index + num_vars_free; ++i)
		{
			KDL::Frame& frame =
					data_->segment_frames_[i][END_EFFECTOR_SEGMENT_INDEX];

			double proj_dist = KDL::dot(dir, (frame.p - start_pos));
			KDL::Vector proj_pt = proj_dist * dir;
			if (proj_dist < 0)
				proj_pt = KDL::Vector::Zero();
			else if (proj_dist > dir_max)
				proj_pt = dir_max * dir;

			KDL::Vector distToLine = (frame.p - start_pos) - proj_pt;
			double distance = distToLine.Norm();

			printf("[%d] dist: %f (%f %f %f)\n", i, distance, frame.p.x(),
					frame.p.y(), frame.p.z());

			if (distance > max_dist)
				max_dist = distance;
		}
		printf("Max dist : %f\n", max_dist);

	}
}

void EvaluationManager::computeCartesianTrajectoryCosts()
{
	if (PlanningParameters::getInstance()->getCartesianTrajectoryCostWeight()
			== 0.0)
		return;

	if (data_->cartesian_waypoints_.size() != 0)
	{
		// position constraint

		const int END_EFFECTOR_SEGMENT_INDEX =
				robot_model_->getForwardKinematicsSolver()->segmentNameToIndex(
						"tcp_2_link");

		//data_->costAccumulator_.is_last_trajectory_valid_ = true;

		const double rotation_weight = 0.0;

		int num_vars_free = 100;

		for (int i = 0; i < num_points_; ++i)
			data_->stateCartesianTrajectoryCost_[i] = 0;

		if (data_->cartesian_waypoints_.size() == 0)
			return;

		KDL::Vector start_pos = data_->cartesian_waypoints_[0].p;
		KDL::Vector end_pos = data_->cartesian_waypoints_[1].p;
		KDL::Vector dir = (end_pos - start_pos);
		double dir_max = dir.Normalize();

		// TODO: fix
		int point_index = 6;
		for (int i = point_index; i < point_index + num_vars_free; ++i)
		{
			KDL::Frame& frame =
					data_->segment_frames_[i][END_EFFECTOR_SEGMENT_INDEX];

			double proj_dist = KDL::dot(dir, (frame.p - start_pos));
			KDL::Vector proj_pt = proj_dist * dir;
			if (proj_dist < 0)
				proj_pt = KDL::Vector::Zero();
			else if (proj_dist > dir_max)
				proj_pt = dir_max * dir;

			KDL::Vector distToLine = (frame.p - start_pos) - proj_pt;
			double distance = distToLine.Norm();

			double cost = distance;

			if (distance > 0.005)
				last_trajectory_collision_free_ = false;
			//data_->costAccumulator_.is_last_trajectory_valid_ = false;

			data_->stateCartesianTrajectoryCost_[i] = cost * cost;

			//if (distance > 0.1)
			//ROS_INFO("error : %d dist : %f", i, distance);
		}

	}
	else
	{
		if (planning_group_->name_ != "lower_body")
			return;

		// orientation constraint

		// TODO: fix hard-coded values
		const int END_EFFECTOR_SEGMENT_INDEX =
				robot_model_->getForwardKinematicsSolver()->segmentNameToIndex(
						"segment_7");

		data_->costAccumulator_.is_last_trajectory_valid_ = true;

		// orientation constraint
		int num_vars_free = 100;
		int point_index = 5;
		for (int i = point_index; i < point_index + num_vars_free; ++i)
		{
			KDL::Frame& frame =
					data_->segment_frames_[i][END_EFFECTOR_SEGMENT_INDEX];
			KDL::Rotation rot = frame.M;
			KDL::Vector x_dir = rot.UnitX();
			KDL::Vector y_dir = rot.UnitY();
			KDL::Vector z_dir = rot.UnitZ();

			double cost = 0;
			double dot = KDL::dot(y_dir, KDL::Vector(0, 0, -1));
			double angle = (dot > 1.0) ? 0 : acos(dot);
			//angle -= 5.0 * M_PI / 180.0;
			// TODO: ?
			if (angle > 5.0 * M_PI / 180.0)
			{
				//data_->costAccumulator_.is_last_trajectory_valid_ = false;
				last_trajectory_collision_free_ = false;
				cost = angle * angle;
			}
			else
			{
				cost = angle * angle * 0.1;
			}

			data_->stateCartesianTrajectoryCost_[i] = cost;
		}
	}
}

void EvaluationManager::postprocess_ik()
{
	return;

	const double threshold = 0.01;

	const Eigen::MatrixXd& contactTrajectoryBlock =
			getGroupTrajectory()->getContactTrajectory();
	int num_contact_phases = getGroupTrajectory()->getNumContactPhases();
	std::vector<int> ik_ref_point(num_contact_phases);
	for (int j = 0; j < num_contacts_; ++j)
	{
		for (int i = 0; i < num_contact_phases; ++i)
		{
			ik_ref_point[i] = -1;
		}

		for (int i = num_contact_phases - 1; i >= 0; --i)
		{
			double contact_value = contactTrajectoryBlock(i, j);
			if (contact_value > threshold)
			{
				ik_ref_point[i] = num_contact_phases;
			}
			else
				break;
		}
		int ref_point = 0;
		for (int i = 1; i < num_contact_phases; ++i)
		{
			double contact_value = contactTrajectoryBlock(i - 1, j);
			if (contact_value > threshold
					&& ik_ref_point[i] != num_contact_phases)
				ik_ref_point[i] = ref_point;
			else
				ref_point = i;
		}

		/*
		 printf("Contact %d : ", j);
		 for (int i = 0; i < num_contact_phases; ++i)
		 {
		 if (ik_ref_point[i] != -1)
		 printf("%d -> %d ", i, ik_ref_point[i]);
		 }
		 printf("\n");
		 */

		////////////
		// ik
		// TODO:
		string ik_group_name;
		switch (j)
		{
		case 0:
			ik_group_name = "left_leg";
			break;
		case 1:
			ik_group_name = "right_leg";
			break;
		case 2:
			ik_group_name = "left_arm";
			break;
		case 3:
			ik_group_name = "right_arm";
			break;
		}
		for (int i = 1; i < num_contact_phases; ++i)
		{
			KDL::Frame contact_frame;
			if (ik_ref_point[i] != -1)
			{
				planning_group_->contactPoints_[j].getFrame(
						ik_ref_point[i]
								* getGroupTrajectory()->getContactPhaseStride(),
						contact_frame, data_->segment_frames_);

				// set kinematic_state to current joint values
				robot_state::RobotStatePtr kinematic_state(
						new robot_state::RobotState(
								robot_model_->getRobotModel()));
				int num_all_joints = kinematic_state->getVariableCount();
				std::vector<double> positions(num_all_joints);
				for (std::size_t k = 0; k < num_all_joints; k++)
				{
					positions[k] = (*getFullTrajectory())(
							i * getGroupTrajectory()->getContactPhaseStride(),
							k);
				}
				kinematic_state->setVariablePositions(&positions[0]);
				kinematic_state->update();

				// compute ik for ref_point end effector position
				const robot_state::JointModelGroup* joint_model_group =
						robot_model_->getRobotModel()->getJointModelGroup(
								ik_group_name);

				Eigen::Affine3d end_effector_state =
						Eigen::Affine3d::Identity();
				for (int r = 0; r < 3; ++r)
					for (int c = 0; c < 3; ++c)
						end_effector_state(r, c) = contact_frame.M(r, c);
				for (int r = 0; r < 3; ++r)
					end_effector_state(r, 3) = contact_frame.p(r);

				{
					KDL::Frame current_frame;
					planning_group_->contactPoints_[j].getFrame(
							i * getGroupTrajectory()->getContactPhaseStride(),
							current_frame, data_->segment_frames_);
					ROS_INFO(
							"[%d] IK from (%f %f %f) to [%d](%f %f %f", i, current_frame.p.x(), current_frame.p.y(), current_frame.p.z(), ik_ref_point[i], contact_frame.p.x(), contact_frame.p.y(), contact_frame.p.z());
				}

				kinematics::KinematicsQueryOptions options;
				options.return_approximate_solution = true;
				bool found_ik = kinematic_state->setFromIK(joint_model_group,
						end_effector_state, 10, 0.1,
						moveit::core::GroupStateValidityCallbackFn(), options);
				// Now, we can print out the IK solution (if found):
				if (found_ik)
				{
					std::vector<double> group_values;
					kinematic_state->copyJointGroupPositions(joint_model_group,
							group_values);
					kinematic_state->setVariablePositions(&positions[0]);
					kinematic_state->setJointGroupPositions(joint_model_group,
							group_values);

					double* state_pos = kinematic_state->getVariablePositions();
					for (std::size_t k = 0; k < num_all_joints; k++)
					{
						(*getFullTrajectory())(
								i
										* getGroupTrajectory()->getContactPhaseStride(),
								k) = state_pos[k];
					}
					ROS_INFO("[%d:%d] IK solution found", i, j);
				}
				else
				{
					ROS_INFO("[%d:%d] Did not find IK solution", i, j);
				}
			}
		}
	}
	getFullTrajectory()->updateFreePointsFromTrajectory();
	getGroupTrajectory()->copyFromFullTrajectory(*getFullTrajectory());
	getGroupTrajectory()->updateTrajectoryFromFreePoints();
}

double EvaluationManager::getTrajectoryCost(bool verbose)
{
	if (verbose)
		data_->costAccumulator_.print(*iteration_);
	return data_->costAccumulator_.getTrajectoryCost();
}

////////////////////////////////////////////////////////////////////////////////
void computeFrameDebug(int point, const char* label, const char* v1_name,
		KDL::Frame& f1, const char* v2_name, KDL::Frame& f2,
		const char* v3_name, KDL::Frame& f3)
{
	// %.14lf?
	//printf("[%d] %s %s(%.14lf %.14lf %.14lf %f) %s(%.14lf %.14lf %.14lf %f) %s(%.14lf %.14lf %.14lf %f)\n", point, label, v1_name,
	printf("[%d] %s %s(%f %f %f %f) %s(%f %f %f %f) %s(%f %f %f %f)\n", point,
			label, v1_name, f1.p.x(), f1.p.y(), f1.p.z(),
			atan2(f1.M.data[3], f1.M.data[0]) * 180.0 / M_PI, v2_name, f2.p.x(),
			f2.p.y(), f2.p.z(),
			atan2(f2.M.data[3], f2.M.data[0]) * 180.0 / M_PI, v3_name, f3.p.x(),
			f3.p.y(), f3.p.z(),
			atan2(f3.M.data[3], f3.M.data[0]) * 180.0 / M_PI);
}
//#define DEBUG_COMPUTE_FRAME
#ifdef DEBUG_COMPUTE_FRAME
#define COMPUTE_FRAME_DEBUG(point, label, v1_name, v1, v2_name, v2, v3_name, v3) computeFrameDebug(point, label, v1_name, v1, v2_name, v2, v3_name, v3);
#else
#define COMPUTE_FRAME_DEBUG(point, label, v1_name, v1, v2_name, v2, v3_name, v3)
#endif

void EvaluationManager::computeBaseFrames(KDL::JntArray& curJointArray,
		int point)
{
	// HRP4 stand pose hip - ankle distance : (-0.028, 0.01905, -0.698)
	double max_lower_body_stretch = sqrt(
			0.698 * 0.698 + /*0.01905 * 0.01905 +*/0.028 * 0.028);
	double initial_root_height = 0.791;
	double endeffector_to_ankle_height = 0.093;
	double hip_to_knee_right = 0.01905;
	double knee_to_ankle_dir = -0.028;
	KDL::Vector robot_model_dir = KDL::Vector(1.0, 0.0, 0.0);
	KDL::Vector robot_model_right = KDL::Vector(0.0, -1.0, 0.0);
	KDL::Vector robot_model_up = KDL::Vector(0.0, 0.0, 1.0);

	// human
	if (robot_name_.find("human") != std::string::npos)
	{
		max_lower_body_stretch = 1.02;
		initial_root_height = 1.12;
		endeffector_to_ankle_height = 0.1;
		hip_to_knee_right = 0.0;
		knee_to_ankle_dir = 0.0;
		robot_model_dir = KDL::Vector(1.0, 0.0, 0.0);
		robot_model_right = KDL::Vector(0.0, -1.0, 0.0);
		robot_model_up = KDL::Vector(0.0, 0.0, 1.0);
	}

	int nextPhaseStartPoint = getGroupTrajectory()->getContactPhaseStartPoint(
			point + 3);
	if (nextPhaseStartPoint > point)
	{
		phaseJointArray_[nextPhaseStartPoint - point - 1] = curJointArray;
	}

	if (point == 0)
		data_->fk_solver_.JntToCartPartial(curJointArray,
				data_->joint_pos_[point], data_->joint_axis_[point],
				data_->segment_frames_[point]);
	else
		data_->fk_solver_.JntToCartPartial(curJointArray,
				data_->joint_pos_[point], data_->joint_axis_[point],
				data_->segment_frames_[point]);

	if (planning_group_->name_ != "lower_body"
			&& planning_group_->name_ != "unified_body")
		return;

	int full_traj_index = getGroupTrajectory()->getFullTrajectoryIndex(point);

	// fix supporting foot frame
	int supportingLeg =
			(getGroupTrajectory()->getContactPhase(point) + LeftLegStart) % 2
					== 0 ? 0 : 1;
	int floatingLeg = 1 - supportingLeg;
	int supportingLegSegmentNumber[LEG_LINK_NUM];
	int otherLegSegmentNumber[LEG_LINK_NUM];
	int rootSegmentNumber;

	rootSegmentNumber =
			robot_model_->getForwardKinematicsSolver()->segmentNameToIndex(
					PlanningParameters::getInstance()->getLowerBodyRoot());
	KDL::Vector rootPos = data_->segment_frames_[point][rootSegmentNumber].p;

	bool isLeft = (supportingLeg == 0);
	supportingLegSegmentNumber[LEG_LINK_HIP_YAW] = getSegmentIndex(
			LEG_LINK_HIP_YAW, isLeft);
	otherLegSegmentNumber[LEG_LINK_HIP_YAW] = getSegmentIndex(LEG_LINK_HIP_YAW,
			!isLeft);

	supportingLegSegmentNumber[LEG_LINK_END_EFFECTOR] = getSegmentIndex(
			LEG_LINK_END_EFFECTOR, isLeft);
	otherLegSegmentNumber[LEG_LINK_END_EFFECTOR] = getSegmentIndex(
			LEG_LINK_END_EFFECTOR, !isLeft);

	// TODO: cleanup
	int root_sn = rootSegmentNumber;
	int ef_sn = supportingLegSegmentNumber[LEG_LINK_END_EFFECTOR];
	int of_sn = otherLegSegmentNumber[LEG_LINK_END_EFFECTOR];
	KDL::Vector root_pos = rootPos;

	COMPUTE_FRAME_DEBUG(point, "beginning", "root:", data_->segment_frames_[point][root_sn], "ef:",
			data_->segment_frames_[point][ef_sn], "of:", data_->segment_frames_[point][of_sn]);

	int free_vars_start = full_vars_start_ + 1;
	int free_vars_end = full_vars_end_ - 2;

	// debug
	if (point < free_vars_start || point > free_vars_end)
		return;

	// set current supporting foot frame to foot frame of phase start point foot frame.
	KDL::Frame ef_frame =
			data_->segment_frames_[getGroupTrajectory()->getContactPhaseStartPoint(
					point)][ef_sn];
	KDL::Vector ef_ground, ef_normal;
	// phase start point foot frame = last point foot frame of the previous phase.
	if (getGroupTrajectory()->getContactPhaseStartPoint(point) == point)
	{
		ef_frame = data_->segment_frames_[point - 1][ef_sn];
		GroundManager::getInstance().getNearestGroundPosition(ef_frame.p,
				ef_ground, ef_normal, data_->planning_scene_);
		// always on ground
		ef_frame.p.z(ef_ground.z());
		double eulerAngles[3];
		ef_frame.M.GetEulerZYX(eulerAngles[0], eulerAngles[1], eulerAngles[2]);
		ef_frame.M = KDL::Rotation::RotZ(eulerAngles[0]);

	}

	// if current root position is not reachable from the current supporting foot frame,
	// change root position to max reachable position between ankle pos & root pos
	{
		const double EPSILON = 1E-7;
		const double NEAR_ONE = 1.0 - EPSILON;
		bool needUpdate = false;

		KDL::Vector desiredAnklePos = ef_frame.p
				+ endeffector_to_ankle_height * robot_model_up;
		KDL::Vector hipPos =
				data_->segment_frames_[point][supportingLegSegmentNumber[LEG_LINK_HIP_YAW]].p;

		KDL::Vector robotDir =
				data_->segment_frames_[point][rootSegmentNumber].M
						* robot_model_dir;
		KDL::Vector robotRight =
				data_->segment_frames_[point][rootSegmentNumber].M
						* robot_model_right;

		KDL::Vector diffHipToAnkle = hipPos - desiredAnklePos;
		double dist = diffHipToAnkle.Normalize();
		if (dist > max_lower_body_stretch * NEAR_ONE)
		{
			KDL::Vector newHipPos = desiredAnklePos
					+ NEAR_ONE * max_lower_body_stretch * diffHipToAnkle;
			root_pos -= hipPos - newHipPos;
			needUpdate = true;
		}

		if (needUpdate)
		{
			double old_x = curJointArray(0);
			double old_y = curJointArray(1);
			double old_z = curJointArray(2);

			curJointArray(0) = root_pos.x();
			curJointArray(1) = root_pos.y();
			curJointArray(2) = root_pos.z() - initial_root_height;

			// update trajectory
			(*getFullTrajectory())(full_traj_index, 0) = root_pos.x();
			(*getFullTrajectory())(full_traj_index, 1) = root_pos.y();
			(*getFullTrajectory())(full_traj_index, 2) = root_pos.z()
					- initial_root_height;
			(*getGroupTrajectory())(point, 0) = root_pos.x();
			(*getGroupTrajectory())(point, 1) = root_pos.y();
			(*getGroupTrajectory())(point, 2) = root_pos.z()
					- initial_root_height;

			if (getGroupTrajectory()->getContactPhaseStartPoint(point + 1)
					== point + 1)
			{
				double adjusted_x = root_pos.x() - old_x;
				double adjusted_y = root_pos.y() - old_y;
				double adjusted_z = root_pos.z() - initial_root_height - old_z;

				double adjusted_x_vel = adjusted_x
						- (data_->segment_frames_[point - 1][root_sn].p.x()
								- phaseJointArray_[1](0));
				double adjusted_y_vel = adjusted_y
						- (data_->segment_frames_[point - 1][root_sn].p.y()
								- phaseJointArray_[1](1));
				double adjusted_z_vel = adjusted_z
						- (data_->segment_frames_[point - 1][root_sn].p.z()
								- initial_root_height - phaseJointArray_[1](2));

				double adjusted_x_vel2 =
						(data_->segment_frames_[point - 1][root_sn].p.x()
								- phaseJointArray_[1](0))
								- (data_->segment_frames_[point - 2][root_sn].p.x()
										- phaseJointArray_[2](0));
				double adjusted_y_vel2 =
						(data_->segment_frames_[point - 1][root_sn].p.y()
								- phaseJointArray_[1](1))
								- (data_->segment_frames_[point - 2][root_sn].p.y()
										- phaseJointArray_[2](1));
				double adjusted_z_vel2 =
						(data_->segment_frames_[point - 1][root_sn].p.z()
								- phaseJointArray_[1](2))
								- (data_->segment_frames_[point - 2][root_sn].p.z()
										- phaseJointArray_[2](2));

				double adjusted_x_acc = adjusted_x_vel - adjusted_x_vel2;
				double adjusted_y_acc = adjusted_y_vel - adjusted_y_vel2;
				double adjusted_z_acc = adjusted_z_vel - adjusted_z_vel2;

				MinJerkTrajectory xTraj(adjusted_x, adjusted_x_vel,
						adjusted_x_acc, 0.0, 0.0, 0.0);
				MinJerkTrajectory yTraj(adjusted_y, adjusted_y_vel,
						adjusted_y_acc, 0.0, 0.0, 0.0);
				MinJerkTrajectory zTraj(adjusted_z, adjusted_z_vel,
						adjusted_z_acc, 0.0, 0.0, 0.0);

				for (int i = point + 1; i <= free_vars_end; ++i)
				{
					double t = (double) (i - point) / (free_vars_end - point);
					int full_traj_index2 =
							getGroupTrajectory()->getFullTrajectoryIndex(i);

					double xEval = xTraj(t);
					double yEval = yTraj(t);
					double zEval = zTraj(t);

					(*getFullTrajectory())(full_traj_index2, 0) += xEval;
					(*getFullTrajectory())(full_traj_index2, 1) += yEval;
					(*getFullTrajectory())(full_traj_index2, 2) += zEval;
					(*getGroupTrajectory())(i, 0) += xEval;
					(*getGroupTrajectory())(i, 1) += yEval;
					(*getGroupTrajectory())(i, 2) += zEval;
				}
			}

			data_->fk_solver_.JntToCartPartial(curJointArray,
					data_->joint_pos_[point], data_->joint_axis_[point],
					data_->segment_frames_[point]);
		}

		COMPUTE_FRAME_DEBUG(point, "afterroot", "root:", data_->segment_frames_[point][root_sn], "ef:",
				data_->segment_frames_[point][ef_sn], "of:", data_->segment_frames_[point][of_sn]);

	}

	COMPUTE_FRAME_DEBUG(point, "beforeIK1", "root:", data_->segment_frames_[point][root_sn], "ef:",
			data_->segment_frames_[point][ef_sn], "ef_frame:", ef_frame);

	ComputeCollisionFreeLegUsingIK(supportingLeg, root_pos, ef_frame,
			curJointArray, point);

	COMPUTE_FRAME_DEBUG(point, "afterIK1", "root:", data_->segment_frames_[point][root_sn], "ef:",
			data_->segment_frames_[point][ef_sn], "ef_frame:", ef_frame);

	bool needsUpdate = false;
	KDL::Frame of_frame = data_->segment_frames_[point][of_sn];

	// ?

	if (getIteration() <= 0)
	{

		needsUpdate = true;
		ef_frame =
				data_->segment_frames_[getGroupTrajectory()->getContactPhaseStartPoint(
						point)][ef_sn];
		KDL::Frame root_frame = data_->segment_frames_[point][root_sn];
		//KDL::Vector rootDir = root_frame.M * robot_model_dir;
		KDL::Vector rootMoveDir = getSegmentPosition(num_points_ - 1, root_sn)
				- getSegmentPosition(0, root_sn);
		KDL::Vector diff = root_frame.p - ef_frame.p;
		KDL::Vector newOF = root_frame.p + diff;
		newOF.z(0.0);

		// TODO: arbitrary dir
		KDL::Vector goalOF = getSegmentPosition(num_points_ - 1, of_sn);
		goalOF.z(0.0);
		if (dot(newOF, rootMoveDir) > dot(goalOF, rootMoveDir))
		{
			newOF.x(goalOF.x());
			newOF.y(goalOF.y());
		}

		// for different goal orientation
		if (getGroupTrajectory()->getContactPhaseStartPoint(point + 1)
				== point + 1
				|| getGroupTrajectory()->getContactPhase(point)
						!= getGroupTrajectory()->getNumContactPhases() - 2)
		{
			of_frame.p.x(newOF.x());
			of_frame.p.y(newOF.y());
		}

		COMPUTE_FRAME_DEBUG(point, "init0", "root:", data_->segment_frames_[point][root_sn], "ef:", ef_frame, "of:",
				of_frame);
	}

	KDL::Vector of_ground, normal;
	GroundManager::getInstance().getNearestGroundPosition(of_frame.p, of_ground,
			normal, data_->planning_scene_);

	if (of_ground.z() > of_frame.p.z())
	{
		of_frame.p.z(of_ground.z());
		//of_frame.M = data_->segment_frames_[point][root_sn].M;
		double eulerAngles[3];
		of_frame.M.GetEulerZYX(eulerAngles[0], eulerAngles[1], eulerAngles[2]);
		of_frame.M = KDL::Rotation::RotZ(eulerAngles[0]);
		needsUpdate = true;
	}
	if (getGroupTrajectory()->getContactPhaseStartPoint(point + 1) == point + 1) // && getGroupTrajectory()->getContactPhase(point)        % 2 == 1)
	{
		//GroundManager::getInstance().getSafeGroundPosition(of_frame.p, of_ground);
		of_frame.p.z(of_ground.z());
		//of_frame.M = data_->segment_frames_[point][root_sn].M;
		double eulerAngles[3];
		of_frame.M.GetEulerZYX(eulerAngles[0], eulerAngles[1], eulerAngles[2]);
		of_frame.M = KDL::Rotation::RotZ(eulerAngles[0]);

		needsUpdate = true;
	}

	if (needsUpdate)
	{
		COMPUTE_FRAME_DEBUG(point, "beforeIK2", "root:", data_->segment_frames_[point][root_sn], "of:",
				data_->segment_frames_[point][of_sn], "of_frame:", of_frame);

		ComputeCollisionFreeLegUsingIK(floatingLeg, root_pos, of_frame,
				curJointArray, point, false);

		COMPUTE_FRAME_DEBUG(point, "afterIK2", "root:", data_->segment_frames_[point][root_sn], "of:",
				data_->segment_frames_[point][of_sn], "of_frame:", of_frame);
	}

	if (getGroupTrajectory()->getContactPhaseStartPoint(point + 1) == point + 1)
	{
		// test for rotation
		double eulerAngles[3];
		data_->segment_frames_[point][root_sn].M.GetEulerZYX(eulerAngles[0],
				eulerAngles[1], eulerAngles[2]);
		of_frame.M = KDL::Rotation::RotZ(eulerAngles[0]);
		COMPUTE_FRAME_DEBUG(point, "beforeIK3", "root:", data_->segment_frames_[point][root_sn], "of:",
				data_->segment_frames_[point][of_sn], "of_frame:", of_frame);

		ComputeCollisionFreeLegUsingIK(floatingLeg, root_pos, of_frame,
				curJointArray, point, false, true);

		COMPUTE_FRAME_DEBUG(point, "afterIK3", "root:", data_->segment_frames_[point][root_sn], "of:",
				data_->segment_frames_[point][of_sn], "of_frame:", of_frame);
	}

	if (point == 104)
	{
		for (point = 0; point <= 104; ++point)
		{
			COMPUTE_FRAME_DEBUG(point, "final", "root:", data_->segment_frames_[point][root_sn], "ef:",
					data_->segment_frames_[point][ef_sn], "of:", data_->segment_frames_[point][of_sn]);
		}
	}
}

bool fuzzyEquals(double a, double b)
{
	const double eps = 1E-15;
	return abs(a - b) < eps * max(abs(a), abs(b));
}

double solveASinXPlusBCosXIsC(double a, double b, double c)
{
	// solve a sin x + b cos x = c
	double r = sqrt(a * a + b * b);
	double alpha = atan2(a, b);

	// cos (x-alpha) = c / r
	// x = alpha +- acos (c/r)

	double t = c / r;
	if (fuzzyEquals(t, 1.0))
		t = 1.0;
	else if (fuzzyEquals(t, -1.0))
		t = -1.0;

	double v = min(max(t, -1.0), 1.0);
	double x1 = alpha + acos(v);
	double x2 = alpha - acos(v);

	while (x1 > M_PI)
		x1 -= 2 * M_PI;
	while (x1 <= -M_PI)
		x1 += 2 * M_PI;

	while (x2 > M_PI)
		x2 -= 2 * M_PI;
	while (x2 <= -M_PI)
		x2 += 2 * M_PI;

	if (abs(x1) < abs(x2))
		return x1;
	else
		return x2;
}

int EvaluationManager::getSegmentIndex(int link, bool isLeft) const
{
	string segmentName;
	if (robot_name_.find("hrp4") != std::string::npos)
	{
		if (link >= LEG_LINK_HIP_YAW && link <= LEG_LINK_FOOT)
		{
			const string segmentNames[] =
			{ "", "", "HipYaw", "HipRoll", "HipPitch", "KneePitch",
					"AnklePitch", "AnkleRoll", "Foot", "_foot_endeffector" };
			string supportingLegPrefix = isLeft ? "L" : "R";
			segmentName = supportingLegPrefix + segmentNames[link] + "_link";
		}
		else if (link == LEG_LINK_END_EFFECTOR)
		{
			string supportingLegPrefix = isLeft ? "left" : "right";
			segmentName = supportingLegPrefix + "_foot_endeffector_link";
		}
		else
		{
			ROS_ERROR("Unknown Robot Link!!!");
			return -1;
		}
	}
	else if (robot_name_.find("human") != std::string::npos)
	{
		if (link >= LEG_LINK_HIP_YAW && link <= LEG_LINK_END_EFFECTOR)
		{
			const string segmentNamesPre[] =
			{ "", "", "upper_", "upper_", "upper_", "lower_", "", "", "", "" };
			const string segmentNamesPost[] =
			{ "", "", "_leg_z", "_leg_y", "_leg_x", "_leg", "_foot_x",
					"_foot_y", "_foot_z", "_foot_endeffector" };

			string supportingLegPrefix = isLeft ? "left" : "right";
			segmentName = segmentNamesPre[link] + supportingLegPrefix
					+ segmentNamesPost[link] + "_link";
		}
		else
		{
			ROS_ERROR("Unknown Robot Link!!!");
			return -1;
		}
	}
	else
	{
		ROS_ERROR("Unknown Robot Type!!!");
		return -1;
	}

	int segmentIndex =
			robot_model_->getForwardKinematicsSolver()->segmentNameToIndex(
					segmentName);

	return segmentIndex;
}

void EvaluationManager::getJointIndex(int& groupIndex, int& kdlIndex, int joint,
		bool isLeft) const
{
	string jointName;
	if (robot_name_.find("hrp4") != std::string::npos)
	{
		if (joint >= LEG_JOINT_HIP_YAW && joint <= LEG_JOINT_FOOT)
		{
			const string segmentNames[] =
			{ "", "", "HipYaw", "HipRoll", "HipPitch", "KneePitch",
					"AnklePitch", "AnkleRoll", "Foot", "_foot_endeffector" };
			string supportingLegPrefix = isLeft ? "L" : "R";
			jointName = supportingLegPrefix + segmentNames[joint] + "_joint";
		}
		else if (joint == LEG_JOINT_END_EFFECTOR)
		{
			string supportingLegPrefix = isLeft ? "left" : "right";
			jointName = supportingLegPrefix + "_foot_endeffector_joint";
		}
		else
		{
			ROS_ERROR("Unknown Robot Joint!!!");
			return;
		}
	}
	else if (robot_name_.find("human") != std::string::npos)
	{
		if (joint >= LEG_JOINT_HIP_YAW && joint <= LEG_JOINT_END_EFFECTOR)
		{
			const string segmentNamesPre[] =
			{ "", "", "upper_", "upper_", "upper_", "lower_", "", "", "", "" };
			const string segmentNamesPost[] =
			{ "", "", "_leg_z", "_leg_y", "_leg_x", "_leg", "_foot_x",
					"_foot_y", "_foot_z", "_foot_endeffector" };

			string supportingLegPrefix = isLeft ? "left" : "right";
			jointName = segmentNamesPre[joint] + supportingLegPrefix
					+ segmentNamesPost[joint] + "_joint";
		}
		else
		{
			ROS_ERROR("Unknown Robot Joint!!!");
			return;
		}
	}
	else
	{
		ROS_ERROR("Unknown Robot Type!!!");
		return;
	}

	for (int j = 0; j < planning_group_->group_joints_.size(); ++j)
	{
		if (jointName == planning_group_->group_joints_[j].joint_name_)
		{
			groupIndex = j;
			kdlIndex = planning_group_->group_joints_[j].kdl_joint_index_;
			break;
		}
	}
	return;
}

void EvaluationManager::ComputeCollisionFreeLegUsingIK(int legIndex,
		const KDL::Vector& rootPos, const KDL::Frame& destPose,
		KDL::JntArray& curJointArray, int point, bool support, bool updatePhase)
{
	int free_vars_start = full_vars_start_ + 1;
	int free_vars_end = full_vars_end_ - 2;

	// HRP4 stand pose hip - ankle distance : (-0.028, 0.01905, -0.698)
	double max_lower_body_stretch = sqrt(
			0.698 * 0.698 + /*0.01905 * 0.01905 +*/0.028 * 0.028);
	double initial_root_height = 0.791;
	double endeffector_to_ankle_height = 0.093;
	double root_to_hip_right = -0.0875;
	double hip_to_knee_right = 0.01905;
	double hip_to_knee_up = -0.358;
	double knee_to_ankle_dir = -0.028;
	double knee_to_ankle_up = -0.34;
	KDL::Vector robot_model_dir = KDL::Vector(1.0, 0.0, 0.0);
	KDL::Vector robot_model_right = KDL::Vector(0.0, -1.0, 0.0);
	KDL::Vector robot_model_up = KDL::Vector(0.0, 0.0, 1.0);

	// human
	if (robot_name_.find("human") != std::string::npos)
	{
		max_lower_body_stretch = 1.02;
		initial_root_height = 1.12;
		endeffector_to_ankle_height = 0.1;
		root_to_hip_right = -0.1;
		hip_to_knee_right = 0.0;
		hip_to_knee_up = -0.5;
		knee_to_ankle_dir = 0.0;
		knee_to_ankle_up = -0.52;
		robot_model_dir = KDL::Vector(0.0, 1.0, 0.0);
		robot_model_right = KDL::Vector(1.0, 0.0, 0.0);
		robot_model_up = KDL::Vector(0.0, 0.0, 1.0);
	}

	// compute IK for destPose of endeffector
	// clamp to joint limit
	// support foot - interpolate only for next phase
	// other foot - interpolate in the current phase

	int legSegmentNumber[LEG_LINK_NUM];
	KDL::Frame legFrames[LEG_LINK_NUM];
	double legJointAngles[LEG_JOINT_NUM];

	int rootSegmentNumber =
			robot_model_->getForwardKinematicsSolver()->segmentNameToIndex(
					PlanningParameters::getInstance()->getLowerBodyRoot());
	KDL::Frame rootFrame = data_->segment_frames_[point][rootSegmentNumber];

	int jointGroupNumber[LEG_JOINT_NUM];
	int jointKDLNumber[LEG_JOINT_NUM];

	for (int i = LEG_LINK_HIP_YAW; i < LEG_LINK_NUM; ++i)
	{
		int segmentNumber = getSegmentIndex(i, legIndex == 0);
		getJointIndex(jointGroupNumber[i], jointKDLNumber[i], i, legIndex == 0);
	}

	// foot.p = ROOT.p + ROOT.M * (L2 + R2 * R3 * R4 * (L5 + R5 * (L6 + R6 * R7 * R8(I) * L7)))
	// foot.M = ROOT.M * R2 * R3 * R4 * R5 * R6 * R7 * R8(I)
	// set R8 (foot_yaw, foot) = I
	legJointAngles[LEG_JOINT_FOOT] = 0.0;

	// for human
	// foot.p = ROOT.p + ROOT.M * (L2 + Rz2 * Ry3 * Rx4 * (L5 + Rx5 * (L6 + Rx6 * Ry7 * L7)))               (1)
	// foot.M = ROOT.M * Rz2 * Ry3 * Rx4 * Rx5 * Rx6 * Ry7                                                                                  (2)

	// from (1), ROOT.M^-1 * (foot.p - ROOT.p) - L2 = Rz2 * Ry3 * Rx4 * (L5 + Rx5 * (L6 + Rx6 * Ry7 * L7))  (3)
	// from (2), ROOT.M^-1 * foot.M * Ry7^-1 * Rx6^-1 * Rx5^-1 = Rz2 * Ry3 * Rx4                                                    (4)

	// from (3) and (4), (foot.p - ROOT.p) - ROOT.M * L2 = foot.M * Ry7^-1 * Rx6^-1 * Rx5^-1 * (L5 + Rx5 * (L6 + Rx6 * Ry7 * L7))
	//                                                                                                       = foot.M * (Ry7^-1 * Rx6^-1 * (Rx5^-1 * L5 + L6) + L7)                                         (5)
	// from (5), foot.M^-1 * ((foot.p - ROOT.p) - ROOT.M * L2) - L7 = Ry7^-1 * Rx6^-1 * (Rx5^-1 * L5 + L6)                                                  (6)
	// let foot.M^-1 * ((foot.p - ROOT.p) - ROOT.M * L2) - L7 = v
	// v = Ry7^-1 * Rx6^-1 * (Rx5^-1 * L5 + L6)                             (7)
	// Rx5 * Rx6 * Ry7 * v = (L5 * Rx5 * L6)                                (8)
	// length(v) = length(L5 + Rx5 * L6) : get theta5               (9)
	// let Rx5^-1 * L5 + L6=k
	// from (8),
	//   cos7               0               sin7
	// ( sin6sin7   cos6    -sin6cos7 ) ( v ) = k   (10)
	//  -cos6sin7   sin6    cos6cos7
	// solve v.x * cos7 + v.z * sin7 = k.x : get theta7 (11)
	// solve (v.x * sin7 - v.z * cos7) * sin6 + v.y * cos6 = k.y : get theta6 (12)
	// let ROOT.M^-1 * foot.M * Ry7^-1 * Rx6^-1 * Rx5^-1 = M,
	// from (4), Rz2 * Ry3 * Rx4 = M                (13)
	//   c2c3       -s2c4+c2s3s4    s2s4+c2s3c4
	// ( s2c3       c2c4+s2s3s4             -c2s4+s2s3c4 ) = M              (14)
	//   -s3        c3s4                    c3c4
	// -s3 = M(2,0) : get theta3 (15)
	// s2c3 = M(1,0) : get theta2 (16)
	// c3s4 = M(2,1) : get theta4 (17)

	const KDL::Vector L2 =
			(legIndex == 0) ?
					root_to_hip_right * robot_model_right :
					-root_to_hip_right * robot_model_right;
	const KDL::Vector L5 = hip_to_knee_up * robot_model_up
			+ ((legIndex == 0) ?
					hip_to_knee_right * robot_model_right :
					-hip_to_knee_right * robot_model_right);
	const KDL::Vector L6 = knee_to_ankle_dir * robot_model_dir
			+ knee_to_ankle_up * robot_model_up;
	const KDL::Vector L7 = -endeffector_to_ankle_height * robot_model_up;

	const KDL::Vector v = destPose.M.Inverse()
			* ((destPose.p - rootFrame.p) - rootFrame.M * L2) - L7;

	// let foot.M^-1 * ((foot.p - ROOT.p) - ROOT.M * L2) = v
	// length(v) = length(L5 + Rx5 * L6) : get theta5               (9)
	{
		double sqDist = v.x() * v.x() + v.y() * v.y() + v.z() * v.z();
		const double maxSqDist = knee_to_ankle_dir * knee_to_ankle_dir
				+ (hip_to_knee_up + knee_to_ankle_up)
						* (hip_to_knee_up + knee_to_ankle_up);
		sqDist = min(sqDist, maxSqDist);
		// theta5 = thetaPrime + thetaDelta
		const double thetaDelta = atan2(-knee_to_ankle_dir, -knee_to_ankle_up);
		const double sqR = knee_to_ankle_dir * knee_to_ankle_dir
				+ knee_to_ankle_up * knee_to_ankle_up;
		const double r = sqrt(sqR);
		const double q = min(
				max(
						(sqDist - sqR - hip_to_knee_up * hip_to_knee_up)
								/ (2 * r * -hip_to_knee_up), -1.0), 1.0);
		double thetaPrime = acos(q);

		if (robot_name_.find("human") != std::string::npos)
			thetaPrime = -thetaPrime;

		legJointAngles[LEG_JOINT_KNEE_PITCH] = thetaPrime - thetaDelta;
	}
	// let Rx5^-1 * L5 + L6=k
	// solve v.x * cos7 + v.z * sin7 = k.x : get theta7 (11)
	// solve (v.x * sin7 - v.z * cos7) * sin6 + v.y * cos6 = k.y : get theta6 (12)
	{
		KDL::Vector k = KDL::Rotation::RotX(
				-legJointAngles[LEG_JOINT_KNEE_PITCH]) * L5 + L6;
		legJointAngles[LEG_JOINT_ANKLE_ROLL] = solveASinXPlusBCosXIsC(v.z(),
				v.x(), k.x());
		legJointAngles[LEG_JOINT_ANKLE_PITCH] = solveASinXPlusBCosXIsC(
				v.x() * sin(legJointAngles[LEG_JOINT_ANKLE_ROLL])
						- v.z() * cos(legJointAngles[LEG_JOINT_ANKLE_ROLL]),
				v.y(), k.y());
	}
	// let ROOT.M^-1 * foot.M * Ry7^-1 * Rx6^-1 * Rx5^-1 = M,
	// -s3 = M(2,0) : get theta3 (15)
	// s2c3 = M(1,0) : get theta2 (16)
	// c3s4 = M(2,1) : get theta4 (17)
	{
		KDL::Rotation M = rootFrame.M.Inverse() * destPose.M
				* KDL::Rotation::RotY(-legJointAngles[LEG_JOINT_ANKLE_ROLL])
				* KDL::Rotation::RotX(-legJointAngles[LEG_JOINT_ANKLE_PITCH])
				* KDL::Rotation::RotX(-legJointAngles[LEG_JOINT_KNEE_PITCH]);
		legJointAngles[LEG_JOINT_HIP_ROLL] = -asin(M(2, 0));
		legJointAngles[LEG_JOINT_HIP_YAW] = asin(
				M(1, 0) / cos(legJointAngles[LEG_JOINT_HIP_ROLL]));
		legJointAngles[LEG_JOINT_HIP_PITCH] = asin(
				M(2, 1) / cos(legJointAngles[LEG_JOINT_HIP_ROLL]));
	}

	// validate foot pos
	// foot.p = ROOT.p + ROOT.M * (L2 + Rz2 * Ry3 * Rx4 * (L5 + Rx5 * (L6 + Rx6 * Ry7 * Rz8(I) * L7)))
	// foot.M = ROOT.M * Rz2 * Ry3 * Rx4 * Rx5 * Rx6 * Ry7 * Rz8(I)
	KDL::Vector evaluatedPos =
			rootFrame.p
					+ rootFrame.M
							* (L2
									+ KDL::Rotation::RotZ(
											legJointAngles[LEG_JOINT_HIP_YAW])
											* KDL::Rotation::RotY(
													legJointAngles[LEG_JOINT_HIP_ROLL])
											* KDL::Rotation::RotX(
													legJointAngles[LEG_JOINT_HIP_PITCH])
											* (L5
													+ KDL::Rotation::RotX(
															legJointAngles[LEG_JOINT_KNEE_PITCH])
															* (L6
																	+ KDL::Rotation::RotX(
																			legJointAngles[LEG_JOINT_ANKLE_PITCH])
																			* KDL::Rotation::RotY(
																					legJointAngles[LEG_JOINT_ANKLE_ROLL])
																			* KDL::Rotation::RotZ(
																					legJointAngles[LEG_JOINT_FOOT])
																			* L7)));
	KDL::Rotation evaluatedRot = rootFrame.M
			* KDL::Rotation::RotZ(legJointAngles[LEG_JOINT_HIP_YAW])
			* KDL::Rotation::RotY(legJointAngles[LEG_JOINT_HIP_ROLL])
			* KDL::Rotation::RotX(legJointAngles[LEG_JOINT_HIP_PITCH])
			* KDL::Rotation::RotX(legJointAngles[LEG_JOINT_KNEE_PITCH])
			* KDL::Rotation::RotX(legJointAngles[LEG_JOINT_ANKLE_PITCH])
			* KDL::Rotation::RotY(legJointAngles[LEG_JOINT_ANKLE_ROLL])
			* KDL::Rotation::RotZ(legJointAngles[LEG_JOINT_FOOT]);

	// joint limit
	for (int i = LEG_JOINT_HIP_YAW; i <= LEG_JOINT_FOOT; ++i)
	{
		const itomp_cio_planner::ItompRobotJoint& joint =
				planning_group_->group_joints_[jointGroupNumber[i]];
		if (legJointAngles[i] > joint.joint_limit_max_)
		{
			//ROS_INFO("Joint %d clamped from %f to max %f", i, legJointAngles[i], joint.joint_limit_max_);
			legJointAngles[i] = joint.joint_limit_max_;
		}
		if (legJointAngles[i] < joint.joint_limit_min_)
		{
			//ROS_INFO("Joint %d clamped from %f to min %f", i, legJointAngles[i], joint.joint_limit_min_);
			legJointAngles[i] = joint.joint_limit_min_;
		}
	}

	// update trajectories
	int full_traj_index = getGroupTrajectory()->getFullTrajectoryIndex(point);
	for (int j = LEG_JOINT_HIP_YAW; j <= LEG_JOINT_FOOT; ++j)
	{
		(*getFullTrajectory())(full_traj_index, jointKDLNumber[j]) =
				legJointAngles[j];
		(*getGroupTrajectory())(point, jointGroupNumber[j]) = legJointAngles[j];

		double adjusted;
		adjusted = legJointAngles[j] - curJointArray(jointKDLNumber[j]);

		double adjusted_vel, adjusted_vel2, adjusted_acc;
		adjusted_vel = adjusted
				- ((*getGroupTrajectory())(point - 1, jointGroupNumber[j])
						- phaseJointArray_[1](jointKDLNumber[j]));

		adjusted_vel2 = ((*getGroupTrajectory())(point - 1, jointGroupNumber[j])
				- phaseJointArray_[1](jointKDLNumber[j]))
				- ((*getGroupTrajectory())(point - 2, jointGroupNumber[j])
						- phaseJointArray_[2](jointKDLNumber[j]));
		adjusted_acc = adjusted_vel - adjusted_vel2;

		curJointArray(jointKDLNumber[j]) = legJointAngles[j];

		if (support)
		{
			if (getGroupTrajectory()->getContactPhaseStartPoint(point + 1)
					== point + 1)
			{
				MinJerkTrajectory joint(adjusted, adjusted_vel, adjusted_acc,
						0.0, 0.0, 0.0);
				for (int i = point + 1; i <= free_vars_end; ++i)
				{
					double t = (double) (i - point) / (free_vars_end - point);
					int full_traj_index2 =
							getGroupTrajectory()->getFullTrajectoryIndex(i);

					double jointEval = joint(t);

					(*getGroupTrajectory())(i, jointGroupNumber[j]) +=
							jointEval;

					double jointMax =
							planning_group_->group_joints_[jointGroupNumber[j]].joint_limit_max_;
					double jointMin =
							planning_group_->group_joints_[jointGroupNumber[j]].joint_limit_min_;
					(*getGroupTrajectory())(i, jointGroupNumber[j]) = max(
							(*getGroupTrajectory())(i, jointGroupNumber[j]),
							jointMin);
					(*getGroupTrajectory())(i, jointGroupNumber[j]) = min(
							(*getGroupTrajectory())(i, jointGroupNumber[j]),
							jointMax);
					(*getFullTrajectory())(full_traj_index2, jointKDLNumber[j]) =
							(*getGroupTrajectory())(i, jointGroupNumber[j]);
				}
			}
		}
		else // !support
		{
			int phaseStart = getGroupTrajectory()->getContactPhaseStartPoint(
					point);
			int phaseEnd = getGroupTrajectory()->getContactPhaseEndPoint(point);

			if (updatePhase && j == LEG_JOINT_HIP_YAW)
			{
				MinJerkTrajectory jointB(0.0, 0.0, 0.0, adjusted, 0.0, 0.0);
				for (int i = phaseStart; i < point; ++i)
				{
					double t = (double) (i - phaseStart) / (point - phaseStart);
					int full_traj_index2 =
							getGroupTrajectory()->getFullTrajectoryIndex(i);

					double jointEval = jointB(t);

					(*getGroupTrajectory())(i, jointGroupNumber[j]) +=
							jointEval;

					double jointMax =
							planning_group_->group_joints_[jointGroupNumber[j]].joint_limit_max_;
					double jointMin =
							planning_group_->group_joints_[jointGroupNumber[j]].joint_limit_min_;
					(*getGroupTrajectory())(i, jointGroupNumber[j]) = max(
							(*getGroupTrajectory())(i, jointGroupNumber[j]),
							jointMin);
					(*getGroupTrajectory())(i, jointGroupNumber[j]) = min(
							(*getGroupTrajectory())(i, jointGroupNumber[j]),
							jointMax);
					(*getFullTrajectory())(full_traj_index2, jointKDLNumber[j]) =
							(*getGroupTrajectory())(i, jointGroupNumber[j]);
				}
			}

			MinJerkTrajectory jointA(adjusted, 0.0, 0.0, 0.0, 0.0, 0.0);
			for (int i = point + 1; i < phaseEnd; ++i)
			{
				double t = (double) (i - point) / (phaseEnd - point);
				int full_traj_index2 =
						getGroupTrajectory()->getFullTrajectoryIndex(i);

				double jointEval = jointA(t);

				(*getGroupTrajectory())(i, jointGroupNumber[j]) += jointEval;

				double jointMax =
						planning_group_->group_joints_[jointGroupNumber[j]].joint_limit_max_;
				double jointMin =
						planning_group_->group_joints_[jointGroupNumber[j]].joint_limit_min_;
				(*getGroupTrajectory())(i, jointGroupNumber[j]) = max(
						(*getGroupTrajectory())(i, jointGroupNumber[j]),
						jointMin);
				(*getGroupTrajectory())(i, jointGroupNumber[j]) = min(
						(*getGroupTrajectory())(i, jointGroupNumber[j]),
						jointMax);
				(*getFullTrajectory())(full_traj_index2, jointKDLNumber[j]) =
						(*getGroupTrajectory())(i, jointGroupNumber[j]);
			}
		}

	}

	// update data_->segment_frames_
	data_->fk_solver_.JntToCartPartial(curJointArray, data_->joint_pos_[point],
			data_->joint_axis_[point], data_->segment_frames_[point]);
}

void EvaluationManager::computeSingularityCosts(int begin, int end)
{
	const string group_name = "lower_body";

	if (PlanningParameters::getInstance()->getSingularityCostWeight() == 0.0)
		return;

	return;

	double min_singular_value = std::numeric_limits<double>::max();
	int min_singular_value_index = begin;

	std::vector<double> positions;
	int num_joints = data_->getFullTrajectory()->getNumJoints();
	positions.resize(num_joints);
	for (int i = begin; i < end; ++i)
	{
		data_->stateSingularityCost_[i] = 0.0;

		int full_traj_index =
				data_->getGroupTrajectory()->getFullTrajectoryIndex(i);
		double cost = 0;
		for (std::size_t k = 0; k < num_joints; k++)
		{
			positions[k] = (*data_->getFullTrajectory())(full_traj_index, k);
		}
		int sz = data_->kinematic_state_[0]->getVariableCount();
		data_->kinematic_state_[0]->setVariablePositions(&positions[0]);
		data_->kinematic_state_[0]->update();
		robot_model::RobotModelConstPtr robot_model_ptr =
				data_->getItompRobotModel()->getRobotModel();

		const moveit::core::JointModelGroup* jmg =
				robot_model_ptr->getJointModelGroup(group_name);
		Eigen::MatrixXd jacobianFull = (data_->kinematic_state_[0]->getJacobian(
				jmg));

		int rows2 = jacobianFull.rows();
		cout << jacobianFull << endl;
		for (int k = 0; k < num_joints; ++k)
			printf("%f ", positions[k]);
		printf("\n");

		Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobianFull);
		cout << svd.singularValues() << endl;

		/*
		 printf("[%d] svd values : ", i);
		 for (int j = 0; j < svd.singularValues().rows(); ++j)
		 {
		 printf("%f ", svd.singularValues()(j));
		 }
		 printf("\n");
		 */

		int rows = svd.singularValues().rows();
		double value = (rows == 0 ? 0.0 : svd.singularValues()(rows - 1));
		if (value < min_singular_value)
		{
			min_singular_value = value;
			min_singular_value_index = i;
		}
	}
	if (print_debug_texts_)
		printf("Min Singular value : (%d) %f ", min_singular_value_index,
				min_singular_value);
}

}

