#include <ros/ros.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/PlanningScene.h>
#include <itomp_cio_planner/optimization/new_eval_manager.h>
#include <itomp_cio_planner/trajectory/trajectory_factory.h>
#include <itomp_cio_planner/cost/trajectory_cost_manager.h>
#include <itomp_cio_planner/model/itomp_planning_group.h>
#include <itomp_cio_planner/model/rbdl_model_util.h>
#include <itomp_cio_planner/contact/ground_manager.h>
#include <itomp_cio_planner/visualization/new_viz_manager.h>
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
				std::numeric_limits<double>::max()), ref_evaluation_manager_(
				NULL)
{

}

NewEvalManager::~NewEvalManager()
{

}

void NewEvalManager::initialize(const FullTrajectoryPtr& full_trajectory,
		const ItompRobotModelConstPtr& robot_model,
		const planning_scene::PlanningSceneConstPtr& planning_scene,
		const ItompPlanningGroupConstPtr& planning_group,
		double planning_start_time, double trajectory_start_time,
		const moveit_msgs::Constraints& path_constraints)
{
	full_trajectory_ = full_trajectory;

	robot_model_ = robot_model;
	planning_scene_ = planning_scene;
	planning_group_ = planning_group;

	planning_start_time_ = planning_start_time;
	trajectory_start_time_ = trajectory_start_time;

	evaluation_cost_matrix_.setZero(full_trajectory_->getNumPoints(),
			TrajectoryCostManager::getInstance()->getNumActiveCostFunctions());

	int num_joints = full_trajectory_->getComponentSize(
			FullTrajectory::TRAJECTORY_COMPONENT_JOINT);
	rbdl_models_.resize(full_trajectory_->getNumPoints(),
			robot_model_->getRBDLRobotModel());
	tau_.resize(full_trajectory_->getNumPoints(), Eigen::VectorXd(num_joints));
	external_forces_.resize(full_trajectory_->getNumPoints(),
			std::vector<RigidBodyDynamics::Math::SpatialVector>(
					robot_model_->getRBDLRobotModel().mBodies.size(),
					RigidBodyDynamics::Math::SpatialVectorZero));

	robot_state_.reset(
			new robot_state::RobotState(robot_model_->getMoveitRobotModel()));

	initializeContactVariables();
	parameter_trajectory_.reset(
			TrajectoryFactory::getInstance()->CreateParameterTrajectory(
					full_trajectory_, planning_group));

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
	new_manager->robot_state_.reset(
			new robot_state::RobotState(robot_model_->getMoveitRobotModel()));

	new_manager->ref_evaluation_manager_ = this;

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

	performFullForwardKinematicsAndDynamics(0,
			full_trajectory_->getNumPoints());
	//performInverseDynamics(0, full_trajectory_->getNumPoints());

	last_trajectory_feasible_ = evaluatePointRange(0,
			full_trajectory_->getNumPoints(), evaluation_cost_matrix_);

	return getTrajectoryCost();
}

void NewEvalManager::computeDerivatives(
		const std::vector<Eigen::MatrixXd>& parameters, int type, int point,
		double* out, double eps, double* d_p, double* d_m)
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

		evaluateParameterPoint(value - eps, type, point, i, begin, end, false);
		const double delta_minus = evaluation_cost_matrix_.block(begin, 0,
				end - begin, num_cost_functions).sum();

		*(out + i) = (delta_plus - delta_minus) / (2 * eps);

		*(d_p + i) = delta_plus;
		*(d_m + i) = delta_minus;

		full_trajectory_->restoreBackupTrajectories();
	}
}

void NewEvalManager::evaluateParameterPoint(double value, int type, int point,
		int element, int& full_point_begin, int& full_point_end, bool first)
{
	full_trajectory_->directChangeForDerivatives(value, planning_group_, type,
			point, element, full_point_begin, full_point_end, first);

	performPartialForwardKinematicsAndDynamics(full_point_begin, full_point_end,
			element);
	//performInverseDynamics(full_point_begin, full_point_end);

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

	/*
	 if (point_begin == 0)
	 ++point_begin;
	 if (point_end == full_trajectory_->getNumPoints()
	 && !full_trajectory_->hasFreeEndPoint())
	 --point_end;
	 */

	for (int i = point_begin; i < point_end; ++i)
	{
		for (int c = 0; c < cost_functions.size(); ++c)
		{
			double cost = 0.0;

			is_feasible &= cost_functions[c]->evaluate(this, i, cost);

			cost_matrix(i, c) = cost_functions[c]->getWeight() * cost;
		}
	}
	is_feasible = false;
	return is_feasible;
}

void NewEvalManager::render()
{
	bool is_best = (getTrajectoryCost() <= best_cost_);
	if (PlanningParameters::getInstance()->getAnimatePath())
		NewVizManager::getInstance()->animatePath(full_trajectory_,
				robot_state_, is_best);

	if (PlanningParameters::getInstance()->getAnimateEndeffector())
	{
		NewVizManager::getInstance()->animateEndeffectors(full_trajectory_,
				rbdl_models_, is_best);
		NewVizManager::getInstance()->animateContactForces(full_trajectory_,
				is_best);
	}
}

void NewEvalManager::performFullForwardKinematicsAndDynamics(int point_begin,
		int point_end)
{
	TIME_PROFILER_START_TIMER(FK);

	if (!full_trajectory_->hasVelocity()
			|| !full_trajectory_->hasAcceleration())
		return;

	int num_contacts = planning_group_->getNumContacts();

	for (int point = point_begin; point < point_end; ++point)
	{
		const Eigen::VectorXd& q = full_trajectory_->getComponentTrajectory(
				FullTrajectory::TRAJECTORY_COMPONENT_JOINT,
				Trajectory::TRAJECTORY_TYPE_POSITION).row(point);
		const Eigen::VectorXd& q_dot = full_trajectory_->getComponentTrajectory(
				FullTrajectory::TRAJECTORY_COMPONENT_JOINT,
				Trajectory::TRAJECTORY_TYPE_VELOCITY).row(point);
		const Eigen::VectorXd& q_ddot =
				full_trajectory_->getComponentTrajectory(
						FullTrajectory::TRAJECTORY_COMPONENT_JOINT,
						Trajectory::TRAJECTORY_TYPE_ACCELERATION).row(point);

		const Eigen::VectorXd& r = full_trajectory_->getComponentTrajectory(
				FullTrajectory::TRAJECTORY_COMPONENT_CONTACT_POSITION,
				Trajectory::TRAJECTORY_TYPE_POSITION).row(point);

		const Eigen::VectorXd& f = full_trajectory_->getComponentTrajectory(
				FullTrajectory::TRAJECTORY_COMPONENT_CONTACT_FORCE,
				Trajectory::TRAJECTORY_TYPE_POSITION).row(point);

		for (int i = 0; i < num_contacts; ++i)
		{
			int rbdl_body_id =
					planning_group_->contact_points_[i].getRBDLBodyId();

			Eigen::Vector3d contact_position;
			Eigen::Vector3d contact_normal;
			GroundManager::getInstance()->getNearestGroundPosition(
					r.block(3 * i, 0, 3, 1), contact_position, contact_normal);

			// test
			int foot_index = i / 4 * 4;
			int ee_index = i % 4;
			contact_position = r.block(3 * foot_index, 0, 3, 1);
			contact_position(2) = 0;
			switch (ee_index)
			{
			case 0:
				contact_position(0) -= 0.05;
				contact_position(1) -= 0.05;
				break;
			case 1:
				contact_position(0) += 0.05;
				contact_position(1) -= 0.05;
				break;
			case 2:
				contact_position(0) += 0.05;
				contact_position(1) += 0.2;
				break;
			case 3:
				contact_position(0) -= 0.05;
				contact_position(1) += 0.2;
				break;
			}

			Eigen::Vector3d contact_force = full_trajectory_->getContactForce(point, i);
			if (contact_force(2) < 0.0)
				contact_force(2) = 0.0;

			Eigen::Vector3d contact_torque = contact_position.cross(
					contact_force);

			RigidBodyDynamics::Math::SpatialVector& ext_force =
					external_forces_[point][rbdl_body_id];
			for (int j = 0; j < 3; ++j)
			{
				ext_force(j) = contact_torque(j);
				ext_force(j + 3) = contact_force(j);
			}

		}

		updateFullKinematicsAndDynamics(rbdl_models_[point], q, q_dot, q_ddot,
				tau_[point], &external_forces_[point]);
	}

	TIME_PROFILER_END_TIMER(FK);
}
void NewEvalManager::performPartialForwardKinematicsAndDynamics(int point_begin,
		int point_end, int parameter_element)
{
	TIME_PROFILER_START_TIMER(FK);

	if (!full_trajectory_->hasVelocity()
			|| !full_trajectory_->hasAcceleration())
		return;

	for (int point = point_begin; point < point_end; ++point)
	{
		rbdl_models_[point] = ref_evaluation_manager_->rbdl_models_[point];
	}

	bool dynamics_only = (parameter_element
			>= parameter_trajectory_->getNumJoints());
	int num_contacts = planning_group_->getNumContacts();

	for (int point = point_begin; point < point_end; ++point)
	{
		const Eigen::VectorXd& q = full_trajectory_->getComponentTrajectory(
				FullTrajectory::TRAJECTORY_COMPONENT_JOINT,
				Trajectory::TRAJECTORY_TYPE_POSITION).row(point);

		const Eigen::VectorXd& q_dot = full_trajectory_->getComponentTrajectory(
				FullTrajectory::TRAJECTORY_COMPONENT_JOINT,
				Trajectory::TRAJECTORY_TYPE_VELOCITY).row(point);
		const Eigen::VectorXd& q_ddot =
				full_trajectory_->getComponentTrajectory(
						FullTrajectory::TRAJECTORY_COMPONENT_JOINT,
						Trajectory::TRAJECTORY_TYPE_ACCELERATION).row(point);

		const Eigen::VectorXd& r = full_trajectory_->getComponentTrajectory(
				FullTrajectory::TRAJECTORY_COMPONENT_CONTACT_POSITION,
				Trajectory::TRAJECTORY_TYPE_POSITION).row(point);

		const Eigen::VectorXd& f = full_trajectory_->getComponentTrajectory(
				FullTrajectory::TRAJECTORY_COMPONENT_CONTACT_FORCE,
				Trajectory::TRAJECTORY_TYPE_POSITION).row(point);

		if (dynamics_only)
		{
			for (int i = 0; i < num_contacts; ++i)
			{
				int rbdl_body_id =
						planning_group_->contact_points_[i].getRBDLBodyId();

				Eigen::Vector3d contact_position;
				Eigen::Vector3d contact_normal;
				GroundManager::getInstance()->getNearestGroundPosition(
						r.block(3 * i, 0, 3, 1), contact_position,
						contact_normal);

				// test
				int foot_index = i / 4 * 4;
				int ee_index = i % 4;
				contact_position = r.block(3 * foot_index, 0, 3, 1);
				contact_position(2) = 0;
				switch (ee_index)
				{
				case 0:
					contact_position(0) -= 0.05;
					contact_position(1) -= 0.05;
					break;
				case 1:
					contact_position(0) += 0.05;
					contact_position(1) -= 0.05;
					break;
				case 2:
					contact_position(0) += 0.05;
					contact_position(1) += 0.2;
					break;
				case 3:
					contact_position(0) -= 0.05;
					contact_position(1) += 0.2;
					break;
				}

				Eigen::Vector3d contact_force = full_trajectory_->getContactForce(point, i);
				if (contact_force(2) < 0.0)
					contact_force(2) = 0.0;

				Eigen::Vector3d contact_torque = contact_position.cross(
						contact_force);

				RigidBodyDynamics::Math::SpatialVector& ext_force =
						external_forces_[point][rbdl_body_id];

				for (int j = 0; j < 3; ++j)
				{
					ext_force(j) = contact_torque(j);
					ext_force(j + 3) = contact_force(j);
				}
			}

			updatePartialDynamics(rbdl_models_[point], q, q_dot, q_ddot,
					tau_[point], &external_forces_[point]);
		}
		else
		{
			tau_[point] = ref_evaluation_manager_->tau_[point];
			external_forces_[point] =
					ref_evaluation_manager_->external_forces_[point];

			updatePartialKinematicsAndDynamics(rbdl_models_[point], q, q_dot,
					q_ddot, tau_[point], &external_forces_[point],
					planning_group_->group_joints_[parameter_element].rbdl_affected_body_ids_);

		}
	}

	TIME_PROFILER_END_TIMER(FK);
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

void NewEvalManager::printTrajectoryCost(int iteration, bool details)
{
	double cost = evaluation_cost_matrix_.sum();
	if (!details && cost >= best_cost_)
		return;

	if (cost < best_cost_)
		best_cost_ = cost;

	const std::vector<TrajectoryCostConstPtr>& cost_functions =
			TrajectoryCostManager::getInstance()->getCostFunctionVector();

	if (!details)
	{
		printf("[%d] Trajectory cost : %.7f (", iteration, best_cost_);
		for (int c = 0; c < cost_functions.size(); ++c)
		{
			double sub_cost = evaluation_cost_matrix_.col(c).sum();
			printf("%c=%.7f, ", cost_functions[c]->getName().at(0), sub_cost);
		}
		printf(")\n");
	}
	else
	{
		printf("[%d] Trajectory cost : %.7f/%.7f\n", iteration, cost,
				best_cost_);
		printf("point ");
		for (int c = 0; c < cost_functions.size(); ++c)
		{
			printf("%s ", cost_functions[c]->getName().c_str());
		}
		printf("\n");

		for (int i = 0; i < evaluation_cost_matrix_.rows(); ++i)
		{
			printf("[%d] ", i);
			for (int c = 0; c < cost_functions.size(); ++c)
			{
				double sub_cost = evaluation_cost_matrix_(i, c);
				printf("%.7f ", sub_cost);
			}
			printf("\n");
		}
	}

}

void NewEvalManager::initializeContactVariables()
{
	//return;

	if (!full_trajectory_->hasVelocity()
			|| !full_trajectory_->hasAcceleration())
		return;

	std::vector<int> init_points;
	init_points.push_back(0);
	if (!full_trajectory_->hasFreeEndPoint())
		init_points.push_back(full_trajectory_->getNumPoints() - 1);

	for (int p = 0; p < init_points.size(); ++p)
	{
		int point = init_points[p];

		const Eigen::VectorXd& q = full_trajectory_->getComponentTrajectory(
				FullTrajectory::TRAJECTORY_COMPONENT_JOINT,
				Trajectory::TRAJECTORY_TYPE_POSITION).row(point);
		const Eigen::VectorXd& q_dot = full_trajectory_->getComponentTrajectory(
				FullTrajectory::TRAJECTORY_COMPONENT_JOINT,
				Trajectory::TRAJECTORY_TYPE_VELOCITY).row(point);
		const Eigen::VectorXd& q_ddot =
				full_trajectory_->getComponentTrajectory(
						FullTrajectory::TRAJECTORY_COMPONENT_JOINT,
						Trajectory::TRAJECTORY_TYPE_ACCELERATION).row(point);

		Eigen::VectorXd tau(q.rows());

		updateFullKinematicsAndDynamics(rbdl_models_[point], q, q_dot, q_ddot,
				tau, NULL);

		/*
		 for (int i = 0; i < rbdl_models_[point].f.size(); ++i)
		 cout << i << " : " << rbdl_models_[point].f[i].transpose() << endl;
		 cout << tau.transpose() << endl;
		 */

		int num_contacts = planning_group_->getNumContacts();

		std::vector<Eigen::MatrixXd> jacobians(num_contacts,
				Eigen::MatrixXd(6, q.rows()));
		std::vector<Eigen::Vector3d> contact_position(num_contacts);
		std::vector<Eigen::Vector3d> contact_force(num_contacts);
		std::vector<Eigen::Vector3d> contact_torque(num_contacts);

		std::vector<RigidBodyDynamics::Math::SpatialVector> ext_forces;
		ext_forces.resize(rbdl_models_[point].mBodies.size(),
				RigidBodyDynamics::Math::SpatialVectorZero);

		for (int i = 0; i < num_contacts; ++i)
		{
			int rbdl_body_id =
					planning_group_->contact_points_[i].getRBDLBodyId();
			CalcFullJacobian(rbdl_models_[point], q, rbdl_body_id,
					Eigen::Vector3d::Zero(), jacobians[i], false);

			contact_position[i] = rbdl_models_[point].X_base[rbdl_body_id].r;
			contact_force[i] = 0.0 / num_contacts * tau.block(0, 0, 3, 1);
			contact_torque[i] = contact_position[i].cross(contact_force[i]);

			if (i == 0 || i == 4)
			{
				contact_position[i](0) += 0.05;
				contact_position[i](1) += 0.05;
			}

			RigidBodyDynamics::Math::SpatialVector& ext_force =
					ext_forces[rbdl_body_id];
			ext_force.set(contact_torque[i].coeff(0),
					contact_torque[i].coeff(1), contact_torque[i].coeff(2),
					contact_force[i].coeff(0), contact_force[i].coeff(1),
					contact_force[i].coeff(2));
		}

		RigidBodyDynamics::InverseDynamics(rbdl_models_[point], q, q_dot,
				q_ddot, tau, &ext_forces);

		/*
		 for (int i = 0; i < rbdl_models_[point].f.size(); ++i)
		 cout << i << " : " << rbdl_models_[point].f[i].transpose() << endl;
		 cout << tau.transpose() << endl;
		 */

		full_trajectory_->setContactVariables(point, contact_position,
				contact_force);
	}
	full_trajectory_->interpolateContactVariables();
}

}

