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
		last_trajectory_feasible_(false), parameter_modified_(true), best_cost_(
				std::numeric_limits<double>::max())
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
	new_manager->parameter_modified_ = false;

	return new_manager;
}

double NewEvalManager::evaluate()
{
	if (parameter_modified_)
	{
		full_trajectory_->updateFromParameterTrajectory(parameter_trajectory_,
				planning_group_);
		parameter_modified_ = false;
	}

	performForwardKinematics(0, full_trajectory_->getNumPoints());
	performInverseDynamics(0, full_trajectory_->getNumPoints());

	last_trajectory_feasible_ = evaluatePointRange(0,
			full_trajectory_->getNumPoints(), evaluation_cost_matrix_);

	return getTrajectoryCost();
}

void NewEvalManager::computeDerivatives(
		const std::vector<Eigen::MatrixXd>& parameters, int type, int point,
		double* out, double eps)
{
	setParameters(parameters);
	full_trajectory_->updateFromParameterTrajectory(parameter_trajectory_,
			planning_group_);
	parameter_modified_ = false;

	int num_cost_functions =
			TrajectoryCostManager::getInstance()->getNumActiveCostFunctions();

	for (int i = 0; i < parameter_trajectory_->getNumElements(); ++i)
	{
		const double value = parameters[type](point, i);
		int begin, end;

		evaluateParameterPoint(value + eps, type, point, i, begin, end, true);
		const double delta_plus = evaluation_cost_matrix_.block(begin, 0,
				end - begin, num_cost_functions).sum();

		/*
		if (type == 0 && point == 1 && i == 0)
		{
			getFullTrajectory()->printTrajectory();
			std::cout << std::setprecision(10) << evaluation_cost_matrix_
					<< std::endl;
		}
		*/

		evaluateParameterPoint(value - eps, type, point, i, begin, end, false);

		const double delta_minus = evaluation_cost_matrix_.block(begin, 0,
				end - begin, num_cost_functions).sum();

		/*
		if (type == 0 && point == 1 && i == 0)
		{
			getFullTrajectory()->printTrajectory();
			std::cout << std::setprecision(10) << evaluation_cost_matrix_
					<< std::endl;
		}
		*/

		*(out + i) = (delta_plus - delta_minus) / (2 * eps);

		/*
		if (type == 0 && point == 1 && i == 0)
		{
			printf("%.14f = %.14f-%.14f(%.14f) / %.14f\n", *(out + i), delta_plus,
					delta_minus, (delta_plus - delta_minus), 2 * eps);
		}
		*/

		full_trajectory_->restoreBackupTrajectories();
	}
}

void NewEvalManager::evaluateParameterPoint(double value, int type, int point,
		int element, int& full_point_begin, int& full_point_end, bool first)
{
	full_trajectory_->directChangeForDerivatives(value, planning_group_, type,
			point, element, full_point_begin, full_point_end, first);

	performForwardKinematics(full_point_begin, full_point_end);
	performInverseDynamics(full_point_begin, full_point_end);

	evaluatePointRange(full_point_begin, full_point_end,
			evaluation_cost_matrix_);
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
	is_feasible = false;
	return is_feasible;
}

void NewEvalManager::render()
{
}

void NewEvalManager::performForwardKinematics(int point_begin, int point_end)
{
	TIME_PROFILER_START_TIMER(FK);

	for (int point = point_begin; point < point_end; ++point)
	{
		Eigen::VectorXd q = full_trajectory_->getComponentTrajectory(
				FullTrajectory::TRAJECTORY_COMPONENT_JOINT,
				Trajectory::TRAJECTORY_TYPE_POSITION).row(point);

		if (full_trajectory_->hasVelocity()
				&& full_trajectory_->hasAcceleration())
		{
			Eigen::VectorXd q_dot = full_trajectory_->getComponentTrajectory(
					FullTrajectory::TRAJECTORY_COMPONENT_JOINT,
					Trajectory::TRAJECTORY_TYPE_VELOCITY).row(point);
			Eigen::VectorXd q_ddot = full_trajectory_->getComponentTrajectory(
					FullTrajectory::TRAJECTORY_COMPONENT_JOINT,
					Trajectory::TRAJECTORY_TYPE_ACCELERATION).row(point);

			RigidBodyDynamics::UpdateKinematics(rbdl_models_[point], q, q_dot,
					q_ddot);
		}
		else
		{
			RigidBodyDynamics::UpdateKinematicsCustom(rbdl_models_[point], &q,
					NULL, NULL);
		}
	}

	TIME_PROFILER_END_TIMER(FK);
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
		const std::vector<Eigen::MatrixXd>& parameters)
{
	parameter_trajectory_->getTrajectory(Trajectory::TRAJECTORY_TYPE_POSITION) =
			parameters[Trajectory::TRAJECTORY_TYPE_POSITION];

	if (parameter_trajectory_->hasVelocity())
		parameter_trajectory_->getTrajectory(
				Trajectory::TRAJECTORY_TYPE_VELOCITY) =
				parameters[Trajectory::TRAJECTORY_TYPE_VELOCITY];

	setParameterModified();
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

