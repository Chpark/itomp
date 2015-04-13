#include <ros/ros.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/PlanningScene.h>
#include <itomp_cio_planner/optimization/new_eval_manager.h>
#include <itomp_cio_planner/optimization/phase_manager.h>
#include <itomp_cio_planner/trajectory/trajectory_factory.h>
#include <itomp_cio_planner/cost/trajectory_cost_manager.h>
#include <itomp_cio_planner/model/itomp_planning_group.h>
#include <itomp_cio_planner/model/rbdl_model_util.h>
#include <itomp_cio_planner/contact/ground_manager.h>
#include <itomp_cio_planner/contact/contact_util.h>
#include <itomp_cio_planner/visualization/new_viz_manager.h>
#include <itomp_cio_planner/util/min_jerk_trajectory.h>
#include <itomp_cio_planner/util/planning_parameters.h>
#include <itomp_cio_planner/util/vector_util.h>
#include <itomp_cio_planner/util/multivariate_gaussian.h>
#include <itomp_cio_planner/util/exponential_map.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std;
using namespace Eigen;

namespace itomp_cio_planner
{

const double PASSIVE_FORCE_RATIO = 1.0;
const NewEvalManager* NewEvalManager::ref_evaluation_manager_ = NULL;

NewEvalManager::NewEvalManager() :
    last_trajectory_feasible_(false),
    parameter_modified_(true),
    best_cost_(std::numeric_limits<double>::max())
{
    if (ref_evaluation_manager_ == NULL)
        ref_evaluation_manager_ = this;
}

NewEvalManager::NewEvalManager(const NewEvalManager& manager)
    : robot_model_(manager.robot_model_),
      planning_scene_(manager.planning_scene_),
      planning_group_(manager.planning_group_),
      planning_start_time_(manager.planning_start_time_),
      trajectory_start_time_(manager.trajectory_start_time_),
      last_trajectory_feasible_(manager.last_trajectory_feasible_),
      parameter_modified_(manager.parameter_modified_),
      best_cost_(manager.best_cost_),
      rbdl_models_(manager.rbdl_models_),
      joint_torques_(manager.joint_torques_),
      external_forces_(manager.external_forces_),
      contact_variables_(manager.contact_variables_),
      evaluation_cost_matrix_(manager.evaluation_cost_matrix_),
      trajectory_constraints_(manager.trajectory_constraints_)
{
    full_trajectory_.reset(manager.full_trajectory_->createClone());
    full_trajectory_const_ = full_trajectory_;
    parameter_trajectory_.reset(TrajectoryFactory::getInstance()->CreateParameterTrajectory(full_trajectory_, planning_group_));
    parameter_trajectory_const_ = parameter_trajectory_;
    itomp_trajectory_.reset(new ItompTrajectory(*manager.getTrajectory()));
    itomp_trajectory_const_ = itomp_trajectory_;

    robot_state_.resize(itomp_trajectory_->getNumPoints());
    for (int i = 0; i < itomp_trajectory_->getNumPoints(); ++i)
        robot_state_[i].reset(new robot_state::RobotState(*manager.robot_state_[i]));

    const collision_detection::WorldPtr world(new collision_detection::World(*planning_scene_->getWorld()));
    collision_world_derivatives_.reset(new CollisionWorldFCLDerivatives(
                                           dynamic_cast<const collision_detection::CollisionWorldFCL&>(*planning_scene_->getCollisionWorld()), world));
    collision_robot_derivatives_.reset(new CollisionRobotFCLDerivatives(
                                           dynamic_cast<const collision_detection::CollisionRobotFCL&>(*planning_scene_->getCollisionRobotUnpadded())));
    collision_robot_derivatives_->constructInternalFCLObject(planning_scene_->getCurrentState());
}

NewEvalManager::~NewEvalManager()
{
}

NewEvalManager& NewEvalManager::operator=(const NewEvalManager& manager)
{
    robot_model_ = manager.robot_model_;
    planning_scene_ = manager.planning_scene_;
    planning_group_ = manager.planning_group_;
    planning_start_time_ = manager.planning_start_time_;
    trajectory_start_time_ = manager.trajectory_start_time_;
    last_trajectory_feasible_ = manager.last_trajectory_feasible_;
    parameter_modified_ = manager.parameter_modified_;
    best_cost_ = manager.best_cost_;
    rbdl_models_ = manager.rbdl_models_;
    joint_torques_ = manager.joint_torques_;
    external_forces_ = manager.external_forces_;
    contact_variables_ = manager.contact_variables_;
    evaluation_cost_matrix_ = manager.evaluation_cost_matrix_;
    trajectory_constraints_ = manager.trajectory_constraints_;

    // allocate
    full_trajectory_.reset(manager.full_trajectory_->createClone());
    full_trajectory_const_ = full_trajectory_;
    parameter_trajectory_.reset(TrajectoryFactory::getInstance()->CreateParameterTrajectory(full_trajectory_, planning_group_));
    parameter_trajectory_const_ = parameter_trajectory_;
    itomp_trajectory_.reset(new ItompTrajectory(*manager.getTrajectory()));
    itomp_trajectory_const_ = itomp_trajectory_;

    robot_state_.resize(itomp_trajectory_->getNumPoints());
    for (int i = 0; i < itomp_trajectory_->getNumPoints(); ++i)
        robot_state_[i].reset(new robot_state::RobotState(*manager.robot_state_[i]));

    const collision_detection::WorldPtr world(new collision_detection::World(*planning_scene_->getWorld()));
    collision_world_derivatives_.reset(new CollisionWorldFCLDerivatives(
                                           dynamic_cast<const collision_detection::CollisionWorldFCL&>(*planning_scene_->getCollisionWorld()), world));
    collision_robot_derivatives_.reset(new CollisionRobotFCLDerivatives(
                                           dynamic_cast<const collision_detection::CollisionRobotFCL&>(*planning_scene_->getCollisionRobotUnpadded())));
    collision_robot_derivatives_->constructInternalFCLObject(planning_scene_->getCurrentState());

    return *this;
}

void NewEvalManager::initialize(const FullTrajectoryPtr& full_trajectory,
                                const ItompTrajectoryPtr& itomp_trajectory,
                                const ItompRobotModelConstPtr& robot_model,
                                const planning_scene::PlanningSceneConstPtr& planning_scene,
                                const ItompPlanningGroupConstPtr& planning_group,
                                double planning_start_time, double trajectory_start_time,
                                const std::vector<moveit_msgs::Constraints>& trajectory_constraints)
{
    full_trajectory_const_ = full_trajectory_ = full_trajectory;
    itomp_trajectory_const_ = itomp_trajectory_ = itomp_trajectory;

	robot_model_ = robot_model;
	planning_scene_ = planning_scene;
	planning_group_ = planning_group;

	planning_start_time_ = planning_start_time;
	trajectory_start_time_ = trajectory_start_time;

	TrajectoryCostManager::getInstance()->buildActiveCostFunctions(this);
    evaluation_cost_matrix_.setZero(full_trajectory_->getNumPoints(), TrajectoryCostManager::getInstance()->getNumActiveCostFunctions());

    int num_joints = full_trajectory_->getComponentSize(FullTrajectory::TRAJECTORY_COMPONENT_JOINT);
    rbdl_models_.resize(full_trajectory_->getNumPoints(), robot_model_->getRBDLRobotModel());
    joint_torques_.resize(full_trajectory_->getNumPoints(), Eigen::VectorXd(num_joints));
	external_forces_.resize(full_trajectory_->getNumPoints(),
                            std::vector<RigidBodyDynamics::Math::SpatialVector>(robot_model_->getRBDLRobotModel().mBodies.size(), RigidBodyDynamics::Math::SpatialVectorZero));

	robot_state_.resize(full_trajectory_->getNumPoints());
	for (int i = 0; i < full_trajectory_->getNumPoints(); ++i)
        robot_state_[i].reset(new robot_state::RobotState(robot_model_->getMoveitRobotModel()));

	initializeContactVariables();

    parameter_trajectory_.reset(TrajectoryFactory::getInstance()->CreateParameterTrajectory(full_trajectory_, planning_group));
    parameter_trajectory_const_ = parameter_trajectory_;

    itomp_trajectory_->computeParameterToTrajectoryIndexMap(robot_model, planning_group);
    itomp_trajectory_->interpolateKeyframes(planning_group);

    const collision_detection::WorldPtr world(new collision_detection::World(*planning_scene_->getWorld()));
    collision_world_derivatives_.reset(new CollisionWorldFCLDerivatives(
                                           dynamic_cast<const collision_detection::CollisionWorldFCL&>(*planning_scene_->getCollisionWorld()), world));
    collision_robot_derivatives_.reset(new CollisionRobotFCLDerivatives(
                                           dynamic_cast<const collision_detection::CollisionRobotFCL&>(*planning_scene_->getCollisionRobotUnpadded())));
    collision_robot_derivatives_->constructInternalFCLObject(planning_scene_->getCurrentState());

    trajectory_constraints_ = trajectory_constraints;
}

double NewEvalManager::evaluate()
{
	if (parameter_modified_)
	{
        full_trajectory_->updateFromParameterTrajectory(parameter_trajectory_, planning_group_);
		parameter_modified_ = false;
	}

    performFullForwardKinematicsAndDynamics(0, full_trajectory_->getNumPoints());

    std::vector<TrajectoryCostPtr>& cost_functions = TrajectoryCostManager::getInstance()->getCostFunctionVector();
	for (int c = 0; c < cost_functions.size(); ++c)
	{
		cost_functions[c]->preEvaluate(this);
	}

    last_trajectory_feasible_ = evaluatePointRange(0, full_trajectory_->getNumPoints(), evaluation_cost_matrix_);

	for (int c = 0; c < cost_functions.size(); ++c)
	{
		cost_functions[c]->postEvaluate(this);
	}

	return getTrajectoryCost();
}

void NewEvalManager::computeDerivatives(
	const std::vector<Eigen::MatrixXd>& parameters, int type, int point,
	double* out, double eps, double* d_p, double* d_m, std::vector<std::vector<double> >* cost_der)
{
	//debug_ = true;

	setParameters(parameters);
    full_trajectory_->updateFromParameterTrajectory(parameter_trajectory_, planning_group_);
    parameter_modified_ = false;

    int num_cost_functions = TrajectoryCostManager::getInstance()->getNumActiveCostFunctions();

	for (int i = 0; i < parameter_trajectory_->getNumElements(); ++i)
	{
		const double value = parameters[type](point, i);
		int begin, end;

		evaluateParameterPoint(value + eps, type, point, i, begin, end, true);
        const double delta_plus = (evaluation_cost_matrix_.block(begin, 0, end - begin, num_cost_functions).sum());

		if (cost_der)
		{
			for (int j = 0; j < num_cost_functions; ++j)
			{
				double dp = evaluation_cost_matrix_.block(begin, j, end - begin, 1).sum();
				(*cost_der)[j][i] = dp;
			}
		}

		evaluateParameterPoint(value - eps, type, point, i, begin, end, false);
        const double delta_minus = (evaluation_cost_matrix_.block(begin, 0, end - begin, num_cost_functions).sum());


		*(out + i) = (delta_plus - delta_minus) / (2 * eps);

		*(d_p + i) = delta_plus;
		*(d_m + i) = delta_minus;

		if (cost_der)
		{
			for (int j = 0; j < num_cost_functions; ++j)
			{
				double dp = (*cost_der)[j][i];
				double dm = evaluation_cost_matrix_.block(begin, j, end - begin, 1).sum();
				(*cost_der)[j][i] = (dp - dm) / (2 * eps);
			}
		}

		full_trajectory_->restoreBackupTrajectories();
	}

	//debug_ = false;
}

void NewEvalManager::computeDerivatives(int parameter_index, const ItompTrajectory::ParameterVector& parameters,
                                        double* derivative_out, double eps)
{
    int num_cost_functions = TrajectoryCostManager::getInstance()->getNumActiveCostFunctions();

    unsigned int point_begin, point_end;
    const double value = parameters(parameter_index, 0);

    double derivative = 0.0;

    const ItompTrajectoryIndex& index = itomp_trajectory_->getTrajectoryIndex(parameter_index);
    if (PhaseManager::getInstance()->updateParameter(index))
    {
        evaluateParameterPointItomp(value + eps, parameter_index, point_begin, point_end, true);
        const double delta_plus = (evaluation_cost_matrix_.block(point_begin, 0, point_end - point_begin, num_cost_functions).sum());

        evaluateParameterPointItomp(value - eps, parameter_index, point_begin, point_end, false);
        const double delta_minus = (evaluation_cost_matrix_.block(point_begin, 0, point_end - point_begin, num_cost_functions).sum());

        derivative = (delta_plus - delta_minus) / (2 * eps);

        itomp_trajectory_->restoreTrajectory();
    }

    *(derivative_out + parameter_index) = derivative;
}

void NewEvalManager::computeCostDerivatives(int parameter_index, const ItompTrajectory::ParameterVector& parameters,
        double* derivative_out, std::vector<double*>& cost_derivative_out, double eps)
{
    int num_cost_functions = TrajectoryCostManager::getInstance()->getNumActiveCostFunctions();

    unsigned int point_begin, point_end;
    const double value = parameters(parameter_index, 0);

    std::vector<double> cost_delta_plus(num_cost_functions, 0.0);
    std::vector<double> cost_delta_minus(num_cost_functions, 0.0);
    double derivative = 0.0;

    const ItompTrajectoryIndex& index = itomp_trajectory_->getTrajectoryIndex(parameter_index);
    if (PhaseManager::getInstance()->updateParameter(index))
    {
        evaluateParameterPointItomp(value + eps, parameter_index, point_begin, point_end, true);
        const double delta_plus = (evaluation_cost_matrix_.block(point_begin, 0, point_end - point_begin, num_cost_functions).sum());
        for (int i = 0; i < num_cost_functions; ++i)
            cost_delta_plus[i] = (evaluation_cost_matrix_.block(point_begin, i, point_end - point_begin, 1).sum());

        evaluateParameterPointItomp(value - eps, parameter_index, point_begin, point_end, false);
        const double delta_minus = (evaluation_cost_matrix_.block(point_begin, 0, point_end - point_begin, num_cost_functions).sum());
        for (int i = 0; i < num_cost_functions; ++i)
            cost_delta_minus[i] = (evaluation_cost_matrix_.block(point_begin, i, point_end - point_begin, 1).sum());

        derivative = (delta_plus - delta_minus) / (2 * eps);

        itomp_trajectory_->restoreTrajectory();
    }

    *(derivative_out + parameter_index) = derivative;
    for (int i = 0; i < num_cost_functions; ++i)
        *(cost_derivative_out[i] + parameter_index) = (cost_delta_plus[i] - cost_delta_minus[i]) / (2 * eps);

}

void NewEvalManager::evaluateParameterPoint(double value, int type, int point,
		int element, int& full_point_begin, int& full_point_end, bool first)
{
	full_trajectory_->directChangeForDerivatives(value, planning_group_, type,
			point, element, full_point_begin, full_point_end, first);

	performPartialForwardKinematicsAndDynamics(full_point_begin, full_point_end,
			element);

	evaluatePointRange(full_point_begin, full_point_end,
					   evaluation_cost_matrix_, type, element);
}

void NewEvalManager::evaluateParameterPointItomp(double value, int parameter_index,
        unsigned int& point_begin, unsigned int& point_end, bool first)
{
    itomp_trajectory_->directChangeForDerivativeComputation(parameter_index, value, point_begin, point_end, first);

    const ItompTrajectoryIndex& index = itomp_trajectory_->getTrajectoryIndex(parameter_index);

    if (index.component == ItompTrajectory::COMPONENT_TYPE_POSITION && index.sub_component == ItompTrajectory::SUB_COMPONENT_TYPE_JOINT &&
            index.element < 2)
        itomp_trajectory_->avoidNeighbors(trajectory_constraints_);

    if (index.point == point_end)
        ++point_end;

    performPartialForwardKinematicsAndDynamics(point_begin, point_end, index);

    evaluatePointRange(point_begin, point_end, evaluation_cost_matrix_, index);
}

bool NewEvalManager::evaluatePointRange(int point_begin, int point_end,
                                        Eigen::MatrixXd& cost_matrix,
                                        const ItompTrajectoryIndex& index)
{
    bool is_feasible = true;

    const std::vector<TrajectoryCostPtr>& cost_functions =
        TrajectoryCostManager::getInstance()->getCostFunctionVector();

    // cost weight changed
    if (cost_functions.size() != cost_matrix.cols())
        cost_matrix = Eigen::MatrixXd::Zero(cost_matrix.rows(),	cost_functions.size());

    for (int c = 0; c < cost_functions.size(); ++c)
    {
        if (cost_functions[c]->isInvariant(this, index))
        {
            for (int i = point_begin; i < point_end; ++i)
                cost_matrix(i, c) = 0.0;
        }
        else
        {
            for (int i = point_begin; i < point_end; ++i)
            {
                double cost = 0.0;

                is_feasible &= cost_functions[c]->evaluate(this, i, cost);

                cost_matrix(i, c) = cost_functions[c]->getWeight() * cost;
            }
        }
    }
    is_feasible = false;
    return is_feasible;
}

bool NewEvalManager::evaluatePointRange(int point_begin, int point_end,
										Eigen::MatrixXd& cost_matrix, int type, int element)
{
	bool is_feasible = true;

	const std::vector<TrajectoryCostPtr>& cost_functions =
		TrajectoryCostManager::getInstance()->getCostFunctionVector();

	// cost weight changed
	if (cost_functions.size() != cost_matrix.cols())
        cost_matrix = Eigen::MatrixXd::Zero(cost_matrix.rows(),	cost_functions.size());

	for (int c = 0; c < cost_functions.size(); ++c)
	{
        /*
		if (cost_functions[c]->isInvariant(this, type, element))
		{
			for (int i = point_begin; i < point_end; ++i)
				cost_matrix(i, c) = 0.0;
		}
		else
        */
		{
			for (int i = point_begin; i < point_end; ++i)
			{
				double cost = 0.0;

				is_feasible &= cost_functions[c]->evaluate(this, i, cost);

				cost_matrix(i, c) = cost_functions[c]->getWeight() * cost;
			}
		}
	}
	is_feasible = false;
	return is_feasible;
}

void NewEvalManager::render()
{
	bool is_best = (getTrajectoryCost() <= best_cost_);
	if (PlanningParameters::getInstance()->getAnimatePath())
    {
        NewVizManager::getInstance()->animatePath(itomp_trajectory_, robot_state_[0], is_best);

        if (is_best)
            NewVizManager::getInstance()->displayTrajectory(itomp_trajectory_);
    }

	if (PlanningParameters::getInstance()->getAnimateEndeffector())
	{
        NewVizManager::getInstance()->animateEndeffectors(itomp_trajectory_, rbdl_models_, is_best);
        NewVizManager::getInstance()->animateContacts(itomp_trajectory_, contact_variables_, rbdl_models_, is_best);
	}
}

void NewEvalManager::performFullForwardKinematicsAndDynamics(int point_begin, int point_end)
{
	TIME_PROFILER_START_TIMER(FK);

	int num_contacts = planning_group_->getNumContacts();
    int num_joints = itomp_trajectory_->getElementTrajectory(ItompTrajectory::COMPONENT_TYPE_POSITION,
                     ItompTrajectory::SUB_COMPONENT_TYPE_JOINT)->getNumElements();

	for (int point = point_begin; point < point_end; ++point)
	{
        const Eigen::VectorXd& q = itomp_trajectory_->getElementTrajectory(ItompTrajectory::COMPONENT_TYPE_POSITION,
                                   ItompTrajectory::SUB_COMPONENT_TYPE_JOINT)->getTrajectoryPoint(point);

        const Eigen::VectorXd& q_dot = itomp_trajectory_->getElementTrajectory(ItompTrajectory::COMPONENT_TYPE_VELOCITY,
                                       ItompTrajectory::SUB_COMPONENT_TYPE_JOINT)->getTrajectoryPoint(point);

        const Eigen::VectorXd& q_ddot = itomp_trajectory_->getElementTrajectory(ItompTrajectory::COMPONENT_TYPE_ACCELERATION,
                                        ItompTrajectory::SUB_COMPONENT_TYPE_JOINT)->getTrajectoryPoint(point);

        if (PlanningParameters::getInstance()->getCIEvaluationOnPoints())
        {
            // compute contact variables
            itomp_trajectory_->getContactVariables(point, contact_variables_[point]);

            // compute external forces
            for (int i = 0; i < num_contacts; ++i)
            {
                const Eigen::Vector3d contact_position = contact_variables_[point][i].getPosition();
                const Eigen::Vector3d contact_orientation = contact_variables_[point][i].getOrientation();

                Eigen::Vector3d contact_normal, proj_position, proj_orientation;

                proj_position = contact_position;
                proj_orientation = contact_orientation;

                contact_variables_[point][i].ComputeProjectedPointPositions(proj_position, proj_orientation,
                        rbdl_models_[point], planning_group_->contact_points_[i]);

                for (int c = 0; c < NUM_ENDEFFECTOR_CONTACT_POINTS; ++c)
                {
                    Eigen::Vector3d& point_position = contact_variables_[point][i].projected_point_positions_[c];
                    Eigen::Vector3d point_orientation;
                    GroundManager::getInstance()->getNearestContactPosition(point_position, proj_orientation,
                            point_position, point_orientation, contact_normal, i < 2);

                    int rbdl_point_id = planning_group_->contact_points_[i].getContactPointRBDLIds(c);

                    Eigen::Vector3d contact_force = contact_variables_[point][i].getPointForce(c);

                    Eigen::Vector3d contact_torque = point_position.cross(contact_force);

                    RigidBodyDynamics::Math::SpatialVector& ext_force = external_forces_[point][rbdl_point_id];
                    for (int j = 0; j < 3; ++j)
                    {
                        ext_force(j) = contact_torque(j);
                        ext_force(j + 3) = contact_force(j);
                    }
                }
            }
        }
        else
        {
            // compute contact variables
            itomp_trajectory_->getContactVariables(point, contact_variables_[point]);
            for (int i = 0; i < num_contacts; ++i)
            {
                const Eigen::Vector3d contact_position = contact_variables_[point][i].getPosition();
                const Eigen::Vector3d contact_orientation = contact_variables_[point][i].getOrientation();

                Eigen::Vector3d contact_normal, proj_position, proj_orientation;
                GroundManager::getInstance()->getNearestContactPosition(contact_position, contact_orientation,
                        proj_position, proj_orientation, contact_normal, i < 2);

                contact_variables_[point][i].ComputeProjectedPointPositions(proj_position, proj_orientation,
                        rbdl_models_[point], planning_group_->contact_points_[i]);
            }

            // compute external forces
            for (int i = 0; i < num_contacts; ++i)
            {
                for (int c = 0; c < NUM_ENDEFFECTOR_CONTACT_POINTS; ++c)
                {
                    int rbdl_point_id = planning_group_->contact_points_[i].getContactPointRBDLIds(c);

                    Eigen::Vector3d point_position = contact_variables_[point][i].projected_point_positions_[c];

                    Eigen::Vector3d contact_force = contact_variables_[point][i].getPointForce(c);

                    Eigen::Vector3d contact_torque = point_position.cross(contact_force);

                    RigidBodyDynamics::Math::SpatialVector& ext_force = external_forces_[point][rbdl_point_id];
                    for (int j = 0; j < 3; ++j)
                    {
                        ext_force(j) = contact_torque(j);
                        ext_force(j + 3) = contact_force(j);
                    }
                }
            }
        }

        // passive forces
        const double K_P = 50.0 * PASSIVE_FORCE_RATIO;
        const double K_D = 1.0 * PASSIVE_FORCE_RATIO;
        std::vector<double> passive_forces(num_joints + 1, 0.0);
        for (int i = 1; i <= num_joints; ++i)
        {
            int q_index = rbdl_models_[point].mJoints[i].q_index;

            if (q_index < 6)
            {

            }
            else if (q_index < 12)
            {
                passive_forces[i] = -K_P * q(q_index) -K_D * q_dot(q_index);
                //passive_force = -K_P * q(q_index);
            }
            else
            {
                //passive_force = -K_D * q_dot(q_index);
            }
        }

        updateFullKinematicsAndDynamics(rbdl_models_[point], q, q_dot, q_ddot, joint_torques_[point], &external_forces_[point], &passive_forces);
	}

	TIME_PROFILER_END_TIMER(FK);
}
void NewEvalManager::performPartialForwardKinematicsAndDynamics(int point_begin, int point_end, int parameter_element)
{
    ROS_ASSERT(false);
	TIME_PROFILER_START_TIMER(FK);
    /*

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

		if (dynamics_only)
		{
			// compute contact variables
			full_trajectory_->getContactVariables(point,
												  contact_variables_[point]);
			for (int i = 0; i < num_contacts; ++i)
			{
				const Eigen::Vector3d contact_position =
					contact_variables_[point][i].getPosition();
				const Eigen::Vector3d contact_orientation =
					contact_variables_[point][i].getOrientation();

				Eigen::Vector3d contact_normal, proj_position, proj_orientation;
				GroundManager::getInstance()->getNearestGroundPosition(
					contact_position, contact_orientation, proj_position,
					proj_orientation, contact_normal);

				int rbdl_endeffector_id =
					planning_group_->contact_points_[i].getRBDLBodyId();

				contact_variables_[point][i].ComputeProjectedPointPositions(
					proj_position, proj_orientation, rbdl_models_[point],
					planning_group_->contact_points_[i]);
			}

			// compute external forces
			for (int i = 0; i < num_contacts; ++i)
			{
				for (int c = 0; c < NUM_ENDEFFECTOR_CONTACT_POINTS; ++c)
				{
					int rbdl_point_id =
						planning_group_->contact_points_[i].getContactPointRBDLIds(
							c);

					Eigen::Vector3d point_position =
						contact_variables_[point][i].projected_point_positions_[c];

					Eigen::Vector3d contact_force =
						contact_variables_[point][i].getPointForce(c);

					Eigen::Vector3d contact_torque = point_position.cross(
														 contact_force);

					RigidBodyDynamics::Math::SpatialVector& ext_force =
						external_forces_[point][rbdl_point_id];
					for (int j = 0; j < 3; ++j)
					{
						ext_force(j) = contact_torque(j);
						ext_force(j + 3) = contact_force(j);
					}
				}
			}

			updatePartialDynamics(rbdl_models_[point], q, q_dot, q_ddot,
                                  joint_torques_[point], &external_forces_[point]);
		}
		else
		{
			contact_variables_[point] =
				ref_evaluation_manager_->contact_variables_[point];
            joint_torques_[point] = ref_evaluation_manager_->joint_torques_[point];
			external_forces_[point] =
				ref_evaluation_manager_->external_forces_[point];

			updatePartialKinematicsAndDynamics(rbdl_models_[point], q, q_dot,
                                               q_ddot, joint_torques_[point], &external_forces_[point], NULL,
											   planning_group_->group_joints_[parameter_element].rbdl_affected_body_ids_);

		}
	}
    */

	TIME_PROFILER_END_TIMER(FK);
}

void NewEvalManager::performPartialForwardKinematicsAndDynamics(int point_begin, int point_end, const ItompTrajectoryIndex& index)
{
    TIME_PROFILER_START_TIMER(FK);

    bool dynamics_only = (index.sub_component != ItompTrajectory::SUB_COMPONENT_TYPE_JOINT);
    int num_contacts = planning_group_->getNumContacts();
    int num_joints = itomp_trajectory_->getElementTrajectory(ItompTrajectory::COMPONENT_TYPE_POSITION,
                     ItompTrajectory::SUB_COMPONENT_TYPE_JOINT)->getNumElements();

    // copy only variables will be updated
    for (int point = point_begin; point < point_end; ++point)
    {
        rbdl_models_[point].f = ref_evaluation_manager_->rbdl_models_[point].f;

        if (!dynamics_only)
        {
            rbdl_models_[point].X_lambda = ref_evaluation_manager_->rbdl_models_[point].X_lambda;
            rbdl_models_[point].X_base = ref_evaluation_manager_->rbdl_models_[point].X_base;
            rbdl_models_[point].v = ref_evaluation_manager_->rbdl_models_[point].v;
            rbdl_models_[point].a = ref_evaluation_manager_->rbdl_models_[point].a;
            rbdl_models_[point].c = ref_evaluation_manager_->rbdl_models_[point].c;
        }
    }

    const ElementTrajectoryPtr& pos_trajectory = itomp_trajectory_->getElementTrajectory(ItompTrajectory::COMPONENT_TYPE_POSITION,
            ItompTrajectory::SUB_COMPONENT_TYPE_JOINT);
    const ElementTrajectoryPtr& vel_trajectory = itomp_trajectory_->getElementTrajectory(ItompTrajectory::COMPONENT_TYPE_VELOCITY,
            ItompTrajectory::SUB_COMPONENT_TYPE_JOINT);
    const ElementTrajectoryPtr& acc_trajectory = itomp_trajectory_->getElementTrajectory(ItompTrajectory::COMPONENT_TYPE_ACCELERATION,
            ItompTrajectory::SUB_COMPONENT_TYPE_JOINT);

    for (int point = point_begin; point < point_end; ++point)
    {
        const Eigen::VectorXd& q = pos_trajectory->getTrajectoryPoint(point);
        const Eigen::VectorXd& q_dot = vel_trajectory->getTrajectoryPoint(point);
        const Eigen::VectorXd& q_ddot = acc_trajectory->getTrajectoryPoint(point);

        if (dynamics_only)
        {
            if (PlanningParameters::getInstance()->getCIEvaluationOnPoints())
            {
                // compute contact variables
                itomp_trajectory_->getContactVariables(point, contact_variables_[point]);

                // compute external forces
                for (int i = 0; i < num_contacts; ++i)
                {
                    const Eigen::Vector3d contact_position = contact_variables_[point][i].getPosition();
                    const Eigen::Vector3d contact_orientation = contact_variables_[point][i].getOrientation();

                    Eigen::Vector3d contact_normal, proj_position, proj_orientation;

                    proj_position = contact_position;
                    proj_orientation = contact_orientation;

                    contact_variables_[point][i].ComputeProjectedPointPositions(proj_position, proj_orientation,
                            rbdl_models_[point], planning_group_->contact_points_[i]);

                    for (int c = 0; c < NUM_ENDEFFECTOR_CONTACT_POINTS; ++c)
                    {
                        Eigen::Vector3d& point_position = contact_variables_[point][i].projected_point_positions_[c];
                        Eigen::Vector3d point_orientation;
                        GroundManager::getInstance()->getNearestContactPosition(point_position, proj_orientation,
                                point_position, point_orientation, contact_normal, i < 2);

                        int rbdl_point_id = planning_group_->contact_points_[i].getContactPointRBDLIds(c);

                        Eigen::Vector3d contact_force = contact_variables_[point][i].getPointForce(c);

                        Eigen::Vector3d contact_torque = point_position.cross(contact_force);

                        RigidBodyDynamics::Math::SpatialVector& ext_force = external_forces_[point][rbdl_point_id];
                        for (int j = 0; j < 3; ++j)
                        {
                            ext_force(j) = contact_torque(j);
                            ext_force(j + 3) = contact_force(j);
                        }
                    }
                }
            }
            else
            {
                // compute contact variables
                itomp_trajectory_->getContactVariables(point, contact_variables_[point]);
                for (int i = 0; i < num_contacts; ++i)
                {
                    const Eigen::Vector3d contact_position = contact_variables_[point][i].getPosition();
                    const Eigen::Vector3d contact_orientation = contact_variables_[point][i].getOrientation();

                    Eigen::Vector3d contact_normal, proj_position, proj_orientation;
                    GroundManager::getInstance()->getNearestContactPosition(contact_position, contact_orientation,
                            proj_position, proj_orientation, contact_normal, i < 2);

                    int rbdl_endeffector_id = planning_group_->contact_points_[i].getRBDLBodyId();

                    contact_variables_[point][i].ComputeProjectedPointPositions(proj_position, proj_orientation,
                            rbdl_models_[point], planning_group_->contact_points_[i]);
                }

                // compute external forces
                for (int i = 0; i < num_contacts; ++i)
                {
                    for (int c = 0; c < NUM_ENDEFFECTOR_CONTACT_POINTS; ++c)
                    {
                        int rbdl_point_id = planning_group_->contact_points_[i].getContactPointRBDLIds(c);

                        Eigen::Vector3d point_position = contact_variables_[point][i].projected_point_positions_[c];

                        Eigen::Vector3d contact_force = contact_variables_[point][i].getPointForce(c);

                        Eigen::Vector3d contact_torque = point_position.cross(contact_force);

                        RigidBodyDynamics::Math::SpatialVector& ext_force = external_forces_[point][rbdl_point_id];
                        for (int j = 0; j < 3; ++j)
                        {
                            ext_force(j) = contact_torque(j);
                            ext_force(j + 3) = contact_force(j);
                        }
                    }
                }
            }

            updatePartialDynamics(rbdl_models_[point], q, q_dot, q_ddot, joint_torques_[point], &external_forces_[point]);
        }
        else
        {
            contact_variables_[point] = ref_evaluation_manager_->contact_variables_[point];
            joint_torques_[point] = ref_evaluation_manager_->joint_torques_[point];
            external_forces_[point] = ref_evaluation_manager_->external_forces_[point];



            // passive forces
            const double K_P = 50.0 * PASSIVE_FORCE_RATIO;
            const double K_D = 1.0 * PASSIVE_FORCE_RATIO;
            std::vector<double> passive_forces(num_joints + 1, 0.0);

            int i = index.element + 1;
            int q_index = rbdl_models_[point].mJoints[i].q_index;
            // TODO: needs reverse map if q_index != index.element
            ROS_ASSERT(q_index == index.element);
            double passive_force = 0.0;
            if (q_index < 6)
            {

            }
            else if (q_index < 12)
            {
                passive_forces[i] = -K_P * q(q_index) -K_D * q_dot(q_index);
                //passive_force = -K_P * q(q_index);
            }
            else
            {
                //passive_force = -K_D * q_dot(q_index);
            }

            updatePartialKinematicsAndDynamics(rbdl_models_[point], q, q_dot,
                                               q_ddot, joint_torques_[point], &external_forces_[point], &passive_forces,
                                               planning_group_->group_joints_[itomp_trajectory_->getParameterJointIndex(index.element)].rbdl_affected_body_ids_);

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

void NewEvalManager::getParameters(ItompTrajectory::ParameterVector& parameters) const
{
    itomp_trajectory_->getParameters(parameters);
}

void NewEvalManager::setParameters(const ItompTrajectory::ParameterVector& parameters)
{
    itomp_trajectory_->setParameters(parameters, planning_group_);
    itomp_trajectory_->avoidNeighbors(trajectory_constraints_);
}

void NewEvalManager::printTrajectoryCost(int iteration, bool details)
{
	double cost = evaluation_cost_matrix_.sum();

    double old_best = best_cost_;

	bool is_best = cost < best_cost_;
	if (is_best)
		best_cost_ = cost;

    return;

    const std::vector<TrajectoryCostPtr>& cost_functions = TrajectoryCostManager::getInstance()->getCostFunctionVector();

	if (!details || !is_best)
	{
	}
	else
	{
        cout.precision(std::numeric_limits<double>::digits10);
        cout << "[" << iteration << "] Trajectory cost : " << fixed << old_best << " -> " << fixed << best_cost_ << std::endl;

        int max_cost_name_length = 0;
        for (int c = 0; c < cost_functions.size(); ++c)
            if (cost_functions[c]->getName().size() > max_cost_name_length)
                max_cost_name_length = cost_functions[c]->getName().size();


        for (int c = 0; c < cost_functions.size(); ++c)
		{
            double sub_cost = evaluation_cost_matrix_.col(c).sum();
            cout << setw(max_cost_name_length) << cost_functions[c]->getName();
            cout << " : " << fixed << sub_cost << std::endl;
        }

        cout.precision(3);
        for (int c = 0; c < cost_functions.size(); ++c)
        {
            cout << setw(max_cost_name_length) << cost_functions[c]->getName() << " : ";
            for (int i = 0; i < itomp_trajectory_->getNumPoints(); ++i)
            {
                double cost = evaluation_cost_matrix_(i, c);
                std::cout << fixed << cost << " ";
            }
            std::cout << std::endl;
		}


	}
}

void NewEvalManager::initializeContactVariables()
{
	int num_contacts = planning_group_->getNumContacts();
    ROS_ASSERT(num_contacts == PlanningParameters::getInstance()->getNumContacts());

	// allocate
    contact_variables_.resize(itomp_trajectory_->getNumPoints());
	for (int i = 0; i < contact_variables_.size(); ++i)
	{
		contact_variables_[i].resize(num_contacts);
	}

    for (int point = 0; point < itomp_trajectory_->getNumPoints(); ++point)
	{
        const Eigen::VectorXd& q = itomp_trajectory_->getElementTrajectory(ItompTrajectory::COMPONENT_TYPE_POSITION,
                                   ItompTrajectory::SUB_COMPONENT_TYPE_JOINT)->getTrajectoryPoint(point);

        const Eigen::VectorXd& q_dot = itomp_trajectory_->getElementTrajectory(ItompTrajectory::COMPONENT_TYPE_VELOCITY,
                                       ItompTrajectory::SUB_COMPONENT_TYPE_JOINT)->getTrajectoryPoint(point);

        const Eigen::VectorXd& q_ddot = itomp_trajectory_->getElementTrajectory(ItompTrajectory::COMPONENT_TYPE_ACCELERATION,
                                        ItompTrajectory::SUB_COMPONENT_TYPE_JOINT)->getTrajectoryPoint(point);

		Eigen::VectorXd tau(q.rows());

        updateFullKinematicsAndDynamics(rbdl_models_[point], q, q_dot, q_ddot, tau, NULL, NULL);

		/*
		 for (int i = 0; i < rbdl_models_[point].f.size(); ++i)
		 cout << i << " : " << rbdl_models_[point].f[i].transpose() << endl;
		 cout << tau.transpose() << endl;
		 */

		std::vector<RigidBodyDynamics::Math::SpatialVector> ext_forces;
        ext_forces.resize(rbdl_models_[point].mBodies.size(), RigidBodyDynamics::Math::SpatialVectorZero);

		for (int i = 0; i < num_contacts; ++i)
		{
            int rbdl_body_id = planning_group_->contact_points_[i].getRBDLBodyId();

			contact_variables_[point][i].setVariable(0.0);
            contact_variables_[point][i].setPosition(rbdl_models_[point].X_base[rbdl_body_id].r);
            contact_variables_[point][i].setOrientation(exponential_map::RotationToExponentialMap(rbdl_models_[point].X_base[rbdl_body_id].E));

			for (int j = 0; j < NUM_ENDEFFECTOR_CONTACT_POINTS; ++j)
			{
				Eigen::MatrixXd jacobian(6, q.rows());
				Eigen::Vector3d contact_position;
				Eigen::Vector3d contact_force;
				Eigen::Vector3d contact_torque;

                int rbdl_body_id = planning_group_->contact_points_[i].getContactPointRBDLIds(j);
                CalcFullJacobian(rbdl_models_[point], q, rbdl_body_id, Eigen::Vector3d::Zero(), jacobian, false);

				contact_position = rbdl_models_[point].X_base[rbdl_body_id].r;
				// set forces to 0
				contact_force = 0.0 / num_contacts * tau.block(0, 0, 3, 1);


                // TODO: set initial forces to feet
                if (i < 2)
                {
                    const double VINCENT_CP_WEIGHT[] = {0.107, 0.107, 0.393, 0.393};

                    if (point == 0 || point == itomp_trajectory_->getNumPoints() - 1)
                        //if (point % 20 == 0)
                    {
                        contact_force = VINCENT_CP_WEIGHT[j] / 2.0 * tau.block(0, 0, 3, 1);
                    }
                    /*
                    else
                    {
                        if ((point / 20) % 2 == 0)
                        {
                            if (i == 0)
                            {
                                contact_force = VINCENT_CP_WEIGHT[j] * tau.block(0, 0, 3, 1);
                            }
                        }
                        else
                        {
                            if (i == 1)
                            {
                                contact_force = VINCENT_CP_WEIGHT[j] * tau.block(0, 0, 3, 1);
                            }
                        }
                    }
                    */
                }

				contact_torque = contact_position.cross(contact_force);

                RigidBodyDynamics::Math::SpatialVector& ext_force = ext_forces[rbdl_body_id];
                ext_force.set(contact_torque.coeff(0), contact_torque.coeff(1), contact_torque.coeff(2),
                              contact_force.coeff(0), contact_force.coeff(1), contact_force.coeff(2));

				contact_variables_[point][i].setPointForce(j, contact_force);
			}
		}

		// to validate
        RigidBodyDynamics::InverseDynamics(rbdl_models_[point], q, q_dot, q_ddot, tau, &ext_forces);

		/*
		 for (int i = 0; i < rbdl_models_[point].f.size(); ++i)
		 cout << i << " : " << rbdl_models_[point].f[i].transpose() << endl;
		 cout << tau.transpose() << endl;
		 */

		full_trajectory_->setContactVariables(point, contact_variables_[point]);
        itomp_trajectory_->setContactVariables(point, contact_variables_[point]);
	}
	full_trajectory_->interpolateContactVariables();
    itomp_trajectory_->interpolateStartEnd(ItompTrajectory::SUB_COMPONENT_TYPE_CONTACT_POSITION);
    //itomp_trajectory_->interpolateStartEnd(ItompTrajectory::SUB_COMPONENT_TYPE_CONTACT_FORCE);

    /*
    std::vector<unsigned int> left_foot_cp(7);
    std::vector<unsigned int> right_foot_cp(7);
    for (int i = 0; i < 7; ++i)
    {
        left_foot_cp[i] = i;
        right_foot_cp[i] = i + 7;
    }
    itomp_trajectory_->interpolate(20, 60, ItompTrajectory::SUB_COMPONENT_TYPE_CONTACT_POSITION, &left_foot_cp);
    itomp_trajectory_->copy(60, 40, ItompTrajectory::SUB_COMPONENT_TYPE_CONTACT_POSITION, &right_foot_cp);
    itomp_trajectory_->interpolate(0, 40, ItompTrajectory::SUB_COMPONENT_TYPE_CONTACT_POSITION, &right_foot_cp);
    for (int i = 40; i >= 20; --i)
    {
        itomp_trajectory_->copy(i, i + 20, ItompTrajectory::SUB_COMPONENT_TYPE_CONTACT_POSITION, &right_foot_cp);
        itomp_trajectory_->copy(20, i, ItompTrajectory::SUB_COMPONENT_TYPE_CONTACT_POSITION, &right_foot_cp);
    }
    */
}

void NewEvalManager::resetBestTrajectoryCost()
{
    best_cost_ = std::numeric_limits<double>::max();
}

void NewEvalManager::printLinkTransforms() const
{
    std::ofstream trajectory_file;
    trajectory_file.open("link_transforms.txt");


    for (int i = 0; i < itomp_trajectory_->getNumPoints(); ++i)
    {
        trajectory_file.precision(3);
        trajectory_file << "Time : " << i * itomp_trajectory_->getDiscretization() << endl;

        trajectory_file.precision(std::numeric_limits<double>::digits10);

        const RigidBodyDynamics::Model& model = rbdl_models_[i];
        for (int j = 0; j < model.mBodies.size(); ++j)
        {
            //cout << model.GetBodyName(j) << endl << model.X_base[j] << endl;
            trajectory_file << model.GetBodyName(j) << " X.E ";
            for (int r = 0; r < 3; ++r)
                for (int c = 0; c < 3; ++c)
                    trajectory_file << model.X_base[j].E(r, c) << " ";
            trajectory_file << " X.r ";
            for (int r = 0; r < 3; ++r)
                trajectory_file << model.X_base[j].r(r) << " ";
            trajectory_file << endl;
        }
    }
    trajectory_file.close();
}

}

