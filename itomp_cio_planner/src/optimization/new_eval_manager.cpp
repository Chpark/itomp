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
	parameter_trajectory_.reset(
			TrajectoryFactory::getInstance()->CreateParameterTrajectory(
					full_trajectory_, planning_group));

	robot_model_ = robot_model;
	planning_scene_ = planning_scene;
	planning_group_ = planning_group;

	planning_start_time_ = planning_start_time;
	trajectory_start_time_ = trajectory_start_time;

	evaluation_cost_matrix_ = Eigen::MatrixXd(full_trajectory_->getNumPoints(),
			TrajectoryCostManager::getInstance()->getNumActiveCostFunctions());

	rbdl_models_.resize(full_trajectory_->getNumPoints(),
			robot_model_->getRBDLRobotModel());

	robot_state_.reset(
			new robot_state::RobotState(robot_model_->getMoveitRobotModel()));

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

	// TODO: partial FK
	int full_element_index =
			planning_group_->group_joints_[element].rbdl_joint_index_;
	performPartialForwardKinematics(full_point_begin, full_point_end, element);

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

Eigen::MatrixXd pseudoInverse(const Eigen::MatrixXd& mat)
{
	Eigen::MatrixXd ret;

	JacobiSVD<MatrixXd> svd(mat, ComputeThinU | ComputeThinV);

	Eigen::MatrixXd U = svd.matrixU();
	Eigen::MatrixXd D = svd.singularValues();
	Eigen::MatrixXd V = svd.matrixU();

	cout << mat << endl << endl;
	cout << U << endl << endl;
	cout << (Eigen::MatrixXd) D.asDiagonal() << endl << endl;
	cout << V.transpose() << endl << endl;
	cout << U * D.asDiagonal() * V.transpose() << endl;

	for (int i = 0; i < D.rows(); ++i)
	{
		if (D(i) < 1e-7)
			D(i) = 0;
		else
			D(i) = 1.0 / D(i);
	}

	ret = V * D.asDiagonal() * U.transpose();

	// test
	cout << endl << mat * ret << endl << endl << ret * mat << endl;

	return ret;
}

void NewEvalManager::performInverseDynamics(int point_begin, int point_end)
{
	TIME_PROFILER_START_TIMER(ID);

	if (!full_trajectory_->hasVelocity()
			|| !full_trajectory_->hasAcceleration())
		return;

	int point = 0;

	const Eigen::VectorXd& q = full_trajectory_->getComponentTrajectory(
			FullTrajectory::TRAJECTORY_COMPONENT_JOINT,
			Trajectory::TRAJECTORY_TYPE_POSITION).row(point);
	const Eigen::VectorXd& q_dot = full_trajectory_->getComponentTrajectory(
			FullTrajectory::TRAJECTORY_COMPONENT_JOINT,
			Trajectory::TRAJECTORY_TYPE_VELOCITY).row(point);
	const Eigen::VectorXd& q_ddot = full_trajectory_->getComponentTrajectory(
			FullTrajectory::TRAJECTORY_COMPONENT_JOINT,
			Trajectory::TRAJECTORY_TYPE_ACCELERATION).row(point);

	cout << q.transpose() << endl << q_dot.transpose() << endl
			<< q_ddot.transpose() << endl << endl;

	Eigen::VectorXd tau(q.rows());
	RigidBodyDynamics::InverseDynamics(rbdl_models_[point], q, q_dot, q_ddot,
			tau, NULL);
	cout << tau << endl;

	unsigned int body_id_1 = rbdl_models_[point].GetBodyId(
			"left_foot_endeffector_link");
	unsigned int body_id_2 = rbdl_models_[point].GetBodyId(
			"right_foot_endeffector_link");
	Eigen::MatrixXd j_1(6, q.rows());
	Eigen::MatrixXd j_2(6, q.rows());

	unsigned int l1 = rbdl_models_[point].lambda[body_id_1];
	unsigned int l2 = rbdl_models_[point].lambda[body_id_1];

	Eigen::Vector3d pt;
	pt.setZero(3);

	CalcFullJacobian(rbdl_models_[point], q, body_id_1, pt, j_1, true);
	CalcFullJacobian(rbdl_models_[point], q, body_id_2, pt, j_2, true);
	cout << j_1 << endl << endl << j_2 << endl << endl;

	Eigen::Vector3d p1, p2;
	p1 = rbdl_models_[point].X_base[body_id_1].r;
	p2 = rbdl_models_[point].X_base[body_id_2].r;

	{
		MatrixXd A(6, 6);
		MatrixXd map(6, 6);
		map.setIdentity();
		map(3, 3) = map(1, 1) = map(2, 2) = -1.0;
		A = (Eigen::MatrixXd) (j_1.block(0, 0, 6, 6)).transpose()
				+ (Eigen::MatrixXd) (j_2.block(0, 0, 6, 6)).transpose() * map;

		VectorXd f = A.colPivHouseholderQr().solve(tau.block(0, 0, 6, 1));

		VectorXd force_1 = f;
		VectorXd force_2 = map * f;

		cout << endl << A << endl << force_1 << endl << endl << force_2 << endl
				<< endl;

		cout << endl << A * f << endl << endl;
	}

	Vector3d force_1 = 0.5 * tau.block(0, 0, 3, 1);
	Vector3d force_2 = force_1;
	Vector3d torque_1 = p1.cross(force_1);
	Vector3d torque_2 = p2.cross(force_2);

	std::vector<RigidBodyDynamics::Math::SpatialVector> ext_forces;

	int num_links = rbdl_models_[point].mBodies.size();
	ext_forces.resize(num_links, RigidBodyDynamics::Math::SpatialVectorZero);

	RigidBodyDynamics::Math::SpatialVector& ext_1 = ext_forces[body_id_1];
	RigidBodyDynamics::Math::SpatialVector& ext_2 = ext_forces[body_id_2];
	ext_1.set(torque_1.coeff(0), torque_1.coeff(1), torque_1.coeff(2),
			force_1.coeff(0), force_1.coeff(1), force_1.coeff(2));
	ext_2.set(torque_2.coeff(0), torque_2.coeff(1), torque_2.coeff(2),
			force_2.coeff(0), force_2.coeff(1), force_2.coeff(2));

	cout << ext_forces[body_id_1] << endl << endl << ext_forces[body_id_2]
			<< endl << endl;

	RigidBodyDynamics::InverseDynamics(rbdl_models_[point], q, q_dot, q_ddot,
			tau, &ext_forces);
	cout << endl << tau << endl;

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

}

