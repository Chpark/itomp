#include <ros/ros.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/PlanningScene.h>
#include <itomp_cio_planner/optimization/new_eval_manager.h>
#include <itomp_cio_planner/trajectory/trajectory_factory.h>
#include <itomp_cio_planner/cost/trajectory_cost_manager.h>
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

namespace itomp_cio_planner
{

NewEvalManager::NewEvalManager() :
		last_trajectory_feasible_(false), parameter_modified_(true), check_joint_limits_(
				true), best_cost_(std::numeric_limits<double>::max())
{

}

NewEvalManager::~NewEvalManager()
{

}

void NewEvalManager::initialize(const FullTrajectoryPtr& full_trajectory,
		const ItompRobotModelConstPtr& robot_model,
		const ItompPlanningGroupConstPtr& planning_group,
		double planning_start_time, double trajectory_start_time,
		const moveit_msgs::Constraints& path_constraints)
{
	full_trajectory_ = full_trajectory;
	parameter_trajectory_.reset(
			TrajectoryFactory::getInstance()->CreateParameterTrajectory(
					full_trajectory_, planning_group));

	robot_model_ = robot_model;
	planning_group_ = planning_group;

	planning_start_time_ = planning_start_time;
	trajectory_start_time_ = trajectory_start_time;

	evaluation_cost_matrix_ = Eigen::MatrixXd(full_trajectory_->getNumPoints(),
			TrajectoryCostManager::getInstance()->getNumActiveCostFunctions());

	rbdl_models_.resize(full_trajectory_->getNumPoints(),
			robot_model_->getRBDLRobotModel());

	// TODO : path_constraints
}

NewEvalManager* NewEvalManager::createClone() const
{
	// swallow copy
	NewEvalManager* new_manager = new NewEvalManager(*this);

	// create new trajectories
	new_manager->full_trajectory_.reset(full_trajectory_->createClone());
	new_manager->parameter_trajectory_.reset(
			TrajectoryFactory::getInstance()->CreateParameterTrajectory(
					new_manager->full_trajectory_, planning_group_));

	return new_manager;
}

double NewEvalManager::evaluate()
{
	if (parameter_modified_)
	{
		if (check_joint_limits_)
		{
			parameter_trajectory_->handleJointLimits(planning_group_, 0,
					parameter_trajectory_->getNumPoints());
			check_joint_limits_ = false;
		}
		full_trajectory_->updateFromParameterTrajectory(parameter_trajectory_);
		parameter_modified_ = false;
	}

	performForwardKinematics(0, full_trajectory_->getNumPoints());
	performInverseDynamics(0, full_trajectory_->getNumPoints());

	last_trajectory_feasible_ = evaluatePointRange(0,
			full_trajectory_->getNumPoints(), evaluation_cost_matrix_);

	return getTrajectoryCost();
}

void NewEvalManager::evaluateParameterPoint(int point, int element,
		Eigen::MatrixXd& cost_matrix, int& full_point_begin,
		int& full_point_end)
{
	ROS_ASSERT(parameter_modified_ == false);

	parameter_trajectory_->handleJointLimits(planning_group_, point, point + 1);

	full_trajectory_->updateFromParameterTrajectory(parameter_trajectory_,
			point, point + 1, full_point_begin, full_point_end);

	performForwardKinematics(full_point_begin, full_point_end);
	performInverseDynamics(full_point_begin, full_point_end);

	evaluatePointRange(full_point_begin, full_point_end, cost_matrix);
}

bool NewEvalManager::evaluatePointRange(int point_begin, int point_end,
		Eigen::MatrixXd& cost_matrix)
{
	bool is_feasible = true;

	const std::vector<TrajectoryCostConstPtr>& cost_functions =
			TrajectoryCostManager::getInstance()->getCostFunctionVector();

	// cost weight changed
	if (cost_functions.size() != cost_matrix.cols())
		cost_matrix = Eigen::MatrixXd::Zero(cost_matrix.rows(),
				cost_functions.size());

	for (int i = point_begin; i < point_end; ++i)
	{
		for (int c = 0; c < cost_functions.size(); ++c)
		{
			double cost = 0.0;
			is_feasible &= cost_functions[c]->evaluate(this, full_trajectory_,
					i, cost);

			cost_matrix(i, c) = cost_functions[c]->getWeight() * cost;
		}
	}
	return is_feasible;
}

void NewEvalManager::render()
{
}

void NewEvalManager::performForwardKinematics(int point_begin, int point_end)
{

	int num_joints = full_trajectory_->getComponentSize(
			FullTrajectory::TRAJECTORY_COMPONENT_JOINT);

	printf("name ");
	for (int j = 0; j < num_joints; ++j)
		printf("%s ", robot_model_->rbdlNumberToJointName(j).c_str());
	printf("\n");

	printf("KDL result\n");
	for (int point = point_begin; point < point_end; ++point)
	{
		KDL::JntArray q_in;
		std::vector<KDL::Vector> joint_pos;
		std::vector<KDL::Vector> joint_axis;
		std::vector<KDL::Frame> segment_frames;
		q_in.data = full_trajectory_->getComponentTrajectory(
				FullTrajectory::TRAJECTORY_COMPONENT_JOINT).row(point);
		planning_group_->fk_solver_->JntToCartFull(q_in, joint_pos, joint_axis,
				segment_frames);

		printf("%d x ", point);
		for (int j = 0; j < num_joints; ++j)
			printf("%f ", segment_frames[j + 1].p.x());
		printf("\n");

		printf("%d y ", point);
		for (int j = 0; j < num_joints; ++j)
			printf("%f ", segment_frames[j + 1].p.y());
		printf("\n");

		printf("%d z ", point);
		for (int j = 0; j < num_joints; ++j)
			printf("%f ", segment_frames[j + 1].p.z());
		printf("\n");

		std::vector<Eigen::Quaterniond> rots;
		rots.resize(num_joints);
		for (int j = 0; j < num_joints; ++j)
		{
			//rots[j] = Eigen::Quaterniond(segment_frames[j + 1].M);
			segment_frames[j + 1].M.GetQuaternion(rots[j].x(), rots[j].y(),
					rots[j].z(), rots[j].w());
		}
		printf("%d rx ", point);
		for (int j = 0; j < num_joints; ++j)
			printf("%f ", rots[j].x());
		printf("\n");
		printf("%d ry ", point);
		for (int j = 0; j < num_joints; ++j)
			printf("%f ", rots[j].y());
		printf("\n");
		printf("%d rz ", point);
		for (int j = 0; j < num_joints; ++j)
			printf("%f ", rots[j].z());
		printf("\n");
		printf("%d rw ", point);
		for (int j = 0; j < num_joints; ++j)
			printf("%f ", rots[j].w());
		printf("\n");
	}

	///

	printf("RBDL result\n");
	for (int point = point_begin; point < point_end; ++point)
	{
		Eigen::VectorXd q = full_trajectory_->getComponentTrajectory(
				FullTrajectory::TRAJECTORY_COMPONENT_JOINT).row(point);
		RigidBodyDynamics::UpdateKinematicsCustom(rbdl_models_[point], &q, NULL,
				NULL);

		printf("%d x ", point);
		for (int j = 0; j < num_joints; ++j)
			printf("%f ", rbdl_models_[point].X_base[j + 1].r(0));
		printf("\n");

		printf("%d y ", point);
		for (int j = 0; j < num_joints; ++j)
			printf("%f ", rbdl_models_[point].X_base[j + 1].r(1));
		printf("\n");

		printf("%d z ", point);
		for (int j = 0; j < num_joints; ++j)
			printf("%f ", rbdl_models_[point].X_base[j + 1].r(2));
		printf("\n");

		std::vector<Eigen::Quaterniond> rots;
		rots.resize(num_joints);
		for (int j = 0; j < num_joints; ++j)
		{
			rots[j] = Eigen::Quaterniond(rbdl_models_[point].X_base[j + 1].E);
		}
		printf("%d rx ", point);
		for (int j = 0; j < num_joints; ++j)
			printf("%f ", rots[j].x());
		printf("\n");
		printf("%d ry ", point);
		for (int j = 0; j < num_joints; ++j)
			printf("%f ", rots[j].y());
		printf("\n");
		printf("%d rz ", point);
		for (int j = 0; j < num_joints; ++j)
			printf("%f ", rots[j].z());
		printf("\n");
		printf("%d rw ", point);
		for (int j = 0; j < num_joints; ++j)
			printf("%f ", rots[j].w());
		printf("\n");
	}
}

void NewEvalManager::performInverseDynamics(int point_begin, int point_end)
{
	// TODO
}

void NewEvalManager::getParameters(
		std::vector<Eigen::MatrixXd>& parameters) const
{
	parameters[Trajectory::TRAJECTORY_TYPE_POSITION] =
			parameter_trajectory_->getTrajectory(
					Trajectory::TRAJECTORY_TYPE_POSITION);

	if (parameter_trajectory_->hasVelocity())
		parameters[Trajectory::TRAJECTORY_TYPE_VELOCITY] =
				parameter_trajectory_->getTrajectory(
						Trajectory::TRAJECTORY_TYPE_VELOCITY);
}

void NewEvalManager::setParameters(
		const std::vector<Eigen::MatrixXd>& parameters, bool joint_limit_check)
{
	parameter_trajectory_->getTrajectory(Trajectory::TRAJECTORY_TYPE_POSITION) =
			parameters[Trajectory::TRAJECTORY_TYPE_POSITION];

	if (parameter_trajectory_->hasVelocity())
		parameter_trajectory_->getTrajectory(
				Trajectory::TRAJECTORY_TYPE_VELOCITY) =
				parameters[Trajectory::TRAJECTORY_TYPE_VELOCITY];

	setParameterModified(joint_limit_check);
}

void NewEvalManager::printTrajectoryCost(int iteration)
{
	double cost = evaluation_cost_matrix_.sum();
	if (cost < best_cost_)
		best_cost_ = cost;
	printf("[%d] Trajectory cost : %f/%f (", iteration, cost, best_cost_);

	const std::vector<TrajectoryCostConstPtr>& cost_functions =
			TrajectoryCostManager::getInstance()->getCostFunctionVector();
	for (int c = 0; c < cost_functions.size(); ++c)
	{
		double sub_cost = evaluation_cost_matrix_.col(c).sum();
		printf("%c=%f, ", cost_functions[c]->getName().at(0), sub_cost);
	}

	printf(")\n");
}

}

