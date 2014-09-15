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

	evaluation_cost_matrix_ = Eigen::MatrixXd(full_trajectory_->getNumPoints(),
			TrajectoryCostManager::getInstance()->getNumActiveCostFunctions());

	int num_joints = full_trajectory_->getComponentSize(
			FullTrajectory::TRAJECTORY_COMPONENT_JOINT);
	rbdl_models_.resize(full_trajectory_->getNumPoints(),
			robot_model_->getRBDLRobotModel());
	tau_.resize(full_trajectory_->getNumPoints(), Eigen::VectorXd(num_joints));
	external_forces_.resize(full_trajectory_->getNumPoints(),
			std::vector<RigidBodyDynamics::Math::SpatialVector>(robot_model_->getRBDLRobotModel().mBodies.size(),
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

		evaluateParameterPoint(value - eps, type, point, i, begin, end, false);
		const double delta_minus = evaluation_cost_matrix_.block(begin, 0,
				end - begin, num_cost_functions).sum();

		*(out + i) = (delta_plus - delta_minus) / (2 * eps);

		full_trajectory_->restoreBackupTrajectories();
	}
}

void NewEvalManager::evaluateParameterPoint(double value, int type, int point,
		int element, int& full_point_begin, int& full_point_end, bool first)
{
	full_trajectory_->directChangeForDerivatives(value, planning_group_, type,
			point, element, full_point_begin, full_point_end, first);

	int full_element_index =
			planning_group_->group_joints_[element].rbdl_joint_index_;

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
	// TODO: body_ids
	std::vector<unsigned int> body_ids(num_contacts);
	for (int i = 0; i < num_contacts; ++i)
	{
		body_ids[i] = rbdl_models_[0].GetBodyId(
				planning_group_->contactPoints_[i].getLinkName().c_str());
	}

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
			Eigen::Vector3d contact_position;
			Eigen::Vector3d contact_normal;
			GroundManager::getInstance()->getNearestGroundPosition(
					r.block(3 * i, 0, 3, 1), contact_position, contact_normal);
			Eigen::Vector3d contact_force = f.block(3 * i, 0, 3, 1);
			Eigen::Vector3d contact_torque = contact_position.cross(
					contact_force);

			RigidBodyDynamics::Math::SpatialVector& ext_force =
					external_forces_[point][body_ids[i]];
			for (int j = 0; j < 3; ++j)
			{
				ext_force(j) = contact_torque(j);
				ext_force(j + 3) = contact_force(j);
			}

			// TODO: 6D contact_position ?
			RigidBodyDynamics::Math::SpatialTransform lambda(
					RigidBodyDynamics::Math::Matrix3dIdentity,
					contact_position
							- rbdl_models_[point].X_base[body_ids[i]].r);
			ext_force = lambda.applyTranspose(ext_force);
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
	// TODO: body_ids
	std::vector<unsigned int> body_ids(num_contacts);
	for (int i = 0; i < num_contacts; ++i)
	{
		body_ids[i] = rbdl_models_[0].GetBodyId(
				planning_group_->contactPoints_[i].getLinkName().c_str());
	}

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
				Eigen::Vector3d contact_position;
				Eigen::Vector3d contact_normal;
				GroundManager::getInstance()->getNearestGroundPosition(
						r.block(3 * i, 0, 3, 1), contact_position,
						contact_normal);
				Eigen::Vector3d contact_force = f.block(3 * i, 0, 3, 1);
				Eigen::Vector3d contact_torque = contact_position.cross(
						contact_force);

				RigidBodyDynamics::Math::SpatialVector& ext_force =
						external_forces_[point][body_ids[i]];
				for (int j = 0; j < 3; ++j)
				{
					ext_force(j) = contact_torque(j);
					ext_force(j + 3) = contact_force(j);
				}

				// TODO: 6D contact_position ?
				RigidBodyDynamics::Math::SpatialTransform lambda(
						RigidBodyDynamics::Math::Matrix3dIdentity,
						contact_position
								- rbdl_models_[point].X_base[body_ids[i]].r);
				ext_force = lambda.applyTranspose(ext_force);
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

void NewEvalManager::performPartialForwardKinematics(int point_begin,
		int point_end, int parameter_element)
{
	TIME_PROFILER_START_TIMER(FK);

	for (int point = point_begin; point < point_end; ++point)
	{
		rbdl_models_[point] = ref_evaluation_manager_->rbdl_models_[point];
	}

	if (parameter_element < parameter_trajectory_->getNumJoints())
	{
		for (int point = point_begin; point < point_end; ++point)
		{
			const Eigen::VectorXd& q = full_trajectory_->getComponentTrajectory(
					FullTrajectory::TRAJECTORY_COMPONENT_JOINT,
					Trajectory::TRAJECTORY_TYPE_POSITION).row(point);

			if (full_trajectory_->hasVelocity()
					&& full_trajectory_->hasAcceleration())
			{
				const Eigen::VectorXd& q_dot =
						full_trajectory_->getComponentTrajectory(
								FullTrajectory::TRAJECTORY_COMPONENT_JOINT,
								Trajectory::TRAJECTORY_TYPE_VELOCITY).row(
								point);
				const Eigen::VectorXd& q_ddot =
						full_trajectory_->getComponentTrajectory(
								FullTrajectory::TRAJECTORY_COMPONENT_JOINT,
								Trajectory::TRAJECTORY_TYPE_ACCELERATION).row(
								point);

				UpdatePartialKinematics(rbdl_models_[point], q, q_dot, q_ddot,
						planning_group_->group_joints_[parameter_element].rbdl_affected_body_ids_);
			}
			else
			{
				// TODO:
				RigidBodyDynamics::UpdateKinematicsCustom(rbdl_models_[point],
						&q, NULL, NULL);
			}
		}
	}

	TIME_PROFILER_END_TIMER(FK);
}

void NewEvalManager::performInverseDynamics(int point_begin, int point_end)
{
	TIME_PROFILER_START_TIMER(ID);

	if (!full_trajectory_->hasVelocity()
			|| !full_trajectory_->hasAcceleration())
		return;

	int num_contacts = planning_group_->getNumContacts();
	// stand pose only
	ROS_ASSERT(num_contacts == 2);

	int num_joints = full_trajectory_->getComponentSize(
			FullTrajectory::TRAJECTORY_COMPONENT_JOINT);

	std::vector<unsigned int> body_ids(num_contacts);
	std::vector<RigidBodyDynamics::Math::SpatialVector> ext_forces;
	ext_forces.resize(rbdl_models_[0].mBodies.size(),
			RigidBodyDynamics::Math::SpatialVectorZero);

	for (int i = 0; i < num_contacts; ++i)
	{
		body_ids[i] = rbdl_models_[0].GetBodyId(
				planning_group_->contactPoints_[i].getLinkName().c_str());
	}

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
			Eigen::Vector3d contact_position;
			Eigen::Vector3d contact_normal;
			GroundManager::getInstance()->getNearestGroundPosition(
					r.block(3 * i, 0, 3, 1), contact_position, contact_normal);
			Eigen::Vector3d contact_force = f.block(3 * i, 0, 3, 1);
			Eigen::Vector3d contact_torque = contact_position.cross(
					contact_force);

			RigidBodyDynamics::Math::SpatialVector& ext_force =
					ext_forces[body_ids[i]];
			for (int j = 0; j < 3; ++j)
			{
				ext_force(j) = contact_torque(j);
				ext_force(j + 3) = contact_force(j);
			}
			/*
			 cout << contact_position.transpose() << endl;
			 cout << contact_force.transpose() << endl;
			 cout << contact_torque.transpose() << endl;
			 cout << ext_force.transpose() << endl;
			 */

			// TODO: 6D contact_position ?
			RigidBodyDynamics::Math::SpatialTransform lambda(
					RigidBodyDynamics::Math::Matrix3dIdentity,
					contact_position
							- rbdl_models_[point].X_base[body_ids[i]].r);
			ext_force = lambda.applyTranspose(ext_force);

			//cout << lambda << endl;
			//cout << ext_force.transpose() << endl;
		}

		RigidBodyDynamics::InverseDynamics(rbdl_models_[point], q, q_dot,
				q_ddot, tau_[point], &ext_forces);

		/*
		 cout << "a[" << point << "] " << tau_[point].transpose() << endl
		 << endl;
		 */
	}

	TIME_PROFILER_END_TIMER(ID);
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
	printf("[%d] Trajectory cost : %.7f/%.7f (", iteration, cost, best_cost_);

	const std::vector<TrajectoryCostConstPtr>& cost_functions =
			TrajectoryCostManager::getInstance()->getCostFunctionVector();
	for (int c = 0; c < cost_functions.size(); ++c)
	{
		double sub_cost = evaluation_cost_matrix_.col(c).sum();
		printf("%c=%.7f, ", cost_functions[c]->getName().at(0), sub_cost);
	}

	printf(")\n");
}

void NewEvalManager::initializeContactVariables()
{
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

		RigidBodyDynamics::UpdateKinematics(rbdl_models_[point], q, q_dot,
				q_ddot);

		Eigen::VectorXd tau(q.rows());
		RigidBodyDynamics::InverseDynamics(rbdl_models_[point], q, q_dot,
				q_ddot, tau, NULL);
		//cout << tau << endl;

		int num_contacts = planning_group_->getNumContacts();

		// stand pose only
		ROS_ASSERT(num_contacts == 2);

		std::vector<unsigned int> body_ids(num_contacts);
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
			body_ids[i] = rbdl_models_[point].GetBodyId(
					planning_group_->contactPoints_[i].getLinkName().c_str());
			CalcFullJacobian(rbdl_models_[point], q, body_ids[i],
					Eigen::Vector3d::Zero(), jacobians[i], false);

			contact_position[i] = rbdl_models_[point].X_base[body_ids[i]].r;
			contact_force[i] = 1.0 / num_contacts * tau.block(0, 0, 3, 1);
			contact_torque[i] = contact_position[i].cross(contact_force[i]);

			RigidBodyDynamics::Math::SpatialVector& ext_force =
					ext_forces[body_ids[i]];
			ext_force.set(contact_torque[i].coeff(0),
					contact_torque[i].coeff(1), contact_torque[i].coeff(2),
					contact_force[i].coeff(0), contact_force[i].coeff(1),
					contact_force[i].coeff(2));

			/*
			 cout << contact_position[i].transpose() << endl;
			 cout << contact_force[i].transpose() << endl;
			 cout << contact_torque[i].transpose() << endl;
			 cout << ext_force.transpose() << endl << endl;
			 */
		}

		RigidBodyDynamics::InverseDynamics(rbdl_models_[point], q, q_dot,
				q_ddot, tau, &ext_forces);
		//cout << tau << endl;

		full_trajectory_->setContactVariables(point, contact_position,
				contact_force);
	}
	full_trajectory_->interpolateContactVariables();

	//full_trajectory_->printTrajectory();
}

}

