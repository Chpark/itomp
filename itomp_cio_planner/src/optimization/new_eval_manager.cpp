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
#include <ecl/geometry/polynomial.hpp>
#include <ecl/geometry.hpp>

using namespace std;
using namespace Eigen;

namespace itomp_cio_planner
{

const NewEvalManager* NewEvalManager::ref_evaluation_manager_ = NULL;

NewEvalManager::NewEvalManager() :
    last_trajectory_feasible_(false),
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
      best_cost_(manager.best_cost_),
      rbdl_models_(manager.rbdl_models_),
      joint_torques_(manager.joint_torques_),
      external_forces_(manager.external_forces_),
      contact_variables_(manager.contact_variables_),
      evaluation_cost_matrix_(manager.evaluation_cost_matrix_),
      trajectory_constraints_(manager.trajectory_constraints_)
{
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
    best_cost_ = manager.best_cost_;
    rbdl_models_ = manager.rbdl_models_;
    joint_torques_ = manager.joint_torques_;
    external_forces_ = manager.external_forces_;
    contact_variables_ = manager.contact_variables_;
    evaluation_cost_matrix_ = manager.evaluation_cost_matrix_;
    trajectory_constraints_ = manager.trajectory_constraints_;

    // allocate
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

void NewEvalManager::initialize(const ItompTrajectoryPtr& itomp_trajectory,
                                const ItompRobotModelConstPtr& robot_model,
                                const planning_scene::PlanningSceneConstPtr& planning_scene,
                                const ItompPlanningGroupConstPtr& planning_group,
                                double planning_start_time, double trajectory_start_time,
                                const std::vector<moveit_msgs::Constraints>& trajectory_constraints)
{
    itomp_trajectory_const_ = itomp_trajectory_ = itomp_trajectory;

	robot_model_ = robot_model;
	planning_scene_ = planning_scene;
	planning_group_ = planning_group;

	planning_start_time_ = planning_start_time;
	trajectory_start_time_ = trajectory_start_time;

    int num_points = itomp_trajectory_->getNumPoints();
    int num_joints = itomp_trajectory_->getNumJoints();

	TrajectoryCostManager::getInstance()->buildActiveCostFunctions(this);
    evaluation_cost_matrix_.setZero(num_points, TrajectoryCostManager::getInstance()->getNumActiveCostFunctions());


    rbdl_models_.resize(num_points, robot_model_->getRBDLRobotModel());
    joint_torques_.resize(num_points, Eigen::VectorXd(num_joints));
    external_forces_.resize(num_points,
                            std::vector<RigidBodyDynamics::Math::SpatialVector>(robot_model_->getRBDLRobotModel().mBodies.size(), RigidBodyDynamics::Math::SpatialVectorZero));

    robot_state_.resize(num_points);
    for (int i = 0; i < num_points; ++i)
        robot_state_[i].reset(new robot_state::RobotState(robot_model_->getMoveitRobotModel()));

	initializeContactVariables();

    itomp_trajectory_->computeParameterToTrajectoryIndexMap(robot_model, planning_group);
    //itomp_trajectory_->interpolateKeyframes(planning_group);
    itomp_trajectory_->interpolateStartEnd(ItompTrajectory::SUB_COMPONENT_TYPE_ALL);
    itomp_trajectory_->getElementTrajectory(0, 0)->printTrajectory(std::cout);

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
    int num_points = itomp_trajectory_->getNumPoints();

    performFullForwardKinematicsAndDynamics(0, num_points);

    std::vector<TrajectoryCostPtr>& cost_functions = TrajectoryCostManager::getInstance()->getCostFunctionVector();
    // cost weight changed
    if (cost_functions.size() != evaluation_cost_matrix_.cols())
        evaluation_cost_matrix_ = Eigen::MatrixXd::Zero(evaluation_cost_matrix_.rows(),	cost_functions.size());

    last_trajectory_feasible_ = true;
    for (int c = 0; c < cost_functions.size(); ++c)
    {
        cost_functions[c]->preEvaluate(this);
        for (int i = 0; i < num_points; ++i)
        {
            double cost = 0.0;
            last_trajectory_feasible_ &= cost_functions[c]->evaluate(this, i, cost);
            evaluation_cost_matrix_(i, c) = cost_functions[c]->getWeight() * cost;
        }
        cost_functions[c]->postEvaluate(this);
    }
    last_trajectory_feasible_ = false;

	return getTrajectoryCost();
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
        evaluateParameterPoint(value + eps, parameter_index, point_begin, point_end, true);
        const double delta_plus = (evaluation_cost_matrix_.block(point_begin, 0, point_end - point_begin, num_cost_functions).sum());

        evaluateParameterPoint(value - eps, parameter_index, point_begin, point_end, false);
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
        evaluateParameterPoint(value + eps, parameter_index, point_begin, point_end, true);
        const double delta_plus = (evaluation_cost_matrix_.block(point_begin, 0, point_end - point_begin, num_cost_functions).sum());
        for (int i = 0; i < num_cost_functions; ++i)
            cost_delta_plus[i] = (evaluation_cost_matrix_.block(point_begin, i, point_end - point_begin, 1).sum());

        evaluateParameterPoint(value - eps, parameter_index, point_begin, point_end, false);
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

void NewEvalManager::evaluateParameterPoint(double value, int parameter_index,
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

bool NewEvalManager::evaluatePointRange(int point_begin, int point_end, Eigen::MatrixXd& cost_matrix, const ItompTrajectoryIndex& index)
{
    bool is_feasible = true;

    const std::vector<TrajectoryCostPtr>& cost_functions = TrajectoryCostManager::getInstance()->getCostFunctionVector();

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

void NewEvalManager::render()
{
	bool is_best = (getTrajectoryCost() <= best_cost_);
	if (PlanningParameters::getInstance()->getAnimatePath())
    {
        NewVizManager::getInstance()->animatePath(itomp_trajectory_, robot_state_[0], is_best);

        //if (is_best)
            NewVizManager::getInstance()->displayTrajectory(itomp_trajectory_);
    }

	if (PlanningParameters::getInstance()->getAnimateEndeffector())
	{
        NewVizManager::getInstance()->animateEndeffectors(itomp_trajectory_, rbdl_models_, is_best);
        NewVizManager::getInstance()->animateContacts(itomp_trajectory_, contact_variables_, rbdl_models_, is_best);
	}

    if (is_best)
    {
        NewVizManager::getInstance()->animateInternalForces(itomp_trajectory_, rbdl_models_, true, true);
        NewVizManager::getInstance()->animateCenterOfMass(itomp_trajectory_, rbdl_models_);
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
        std::vector<double> passive_forces(num_joints + 1, 0.0);
        computePassiveForces(point, q, q_dot, passive_forces);

        updateFullKinematicsAndDynamics(rbdl_models_[point], q, q_dot, q_ddot, joint_torques_[point], &external_forces_[point], &passive_forces);
	}

	TIME_PROFILER_END_TIMER(FK);
}

void NewEvalManager::performPartialForwardKinematicsAndDynamics(int point_begin, int point_end, const ItompTrajectoryIndex& index)
{
    TIME_PROFILER_START_TIMER(FK);

    bool dynamics_only = (index.sub_component != ItompTrajectory::SUB_COMPONENT_TYPE_JOINT);
    int num_contacts = planning_group_->getNumContacts();
    int num_joints = itomp_trajectory_->getNumJoints();

    // copy only variables will be updated
    for (int point = point_begin; point < point_end; ++point)
    {
        rbdl_models_[point].f = ref_evaluation_manager_->rbdl_models_[point].f;
        rbdl_models_[point].X_lambda = ref_evaluation_manager_->rbdl_models_[point].X_lambda;
        rbdl_models_[point].X_base = ref_evaluation_manager_->rbdl_models_[point].X_base;
        rbdl_models_[point].v = ref_evaluation_manager_->rbdl_models_[point].v;
        rbdl_models_[point].a = ref_evaluation_manager_->rbdl_models_[point].a;
        rbdl_models_[point].c = ref_evaluation_manager_->rbdl_models_[point].c;
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
            std::vector<double> passive_forces(num_joints + 1, 0.0);
            computePassiveForces(point, q, q_dot, passive_forces);

            updatePartialDynamics(rbdl_models_[point], q, q_dot, q_ddot, joint_torques_[point], &external_forces_[point], &passive_forces);
        }
        else
        {
            contact_variables_[point] = ref_evaluation_manager_->contact_variables_[point];
            joint_torques_[point] = ref_evaluation_manager_->joint_torques_[point];
            external_forces_[point] = ref_evaluation_manager_->external_forces_[point];

            // passive forces
            std::vector<double> passive_forces(num_joints + 1, 0.0);
            computePassiveForces(point, q, q_dot, passive_forces);

            updatePartialKinematicsAndDynamics(rbdl_models_[point], q, q_dot,
                                               q_ddot, joint_torques_[point], &external_forces_[point], &passive_forces,
                                               planning_group_->group_joints_[itomp_trajectory_->getParameterJointIndex(index.element)].rbdl_affected_body_ids_);

        }
    }

    TIME_PROFILER_END_TIMER(FK);
}

void NewEvalManager::getParameters(ItompTrajectory::ParameterVector& parameters) const
{
    itomp_trajectory_->getParameters(parameters);
}

void NewEvalManager::setParameters(const ItompTrajectory::ParameterVector& parameters)
{
    itomp_trajectory_->setParameters(parameters, planning_group_);
    //itomp_trajectory_->avoidNeighbors(trajectory_constraints_);
}

void NewEvalManager::printTrajectoryCost(int iteration, bool details)
{
	double cost = evaluation_cost_matrix_.sum();

    double old_best = best_cost_;

	bool is_best = cost < best_cost_;
	if (is_best)
		best_cost_ = cost;

    //return;

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


        /*
        cout.precision(3);
        for (int c = 0; c < cost_functions.size(); ++c)
        {
            cout << cost_functions[c]->getName() << " ";
        }
        cout << endl;
        for (int i = 0; i < itomp_trajectory_->getNumPoints(); ++i)
        {
            cout << i << " : ";
            for (int c = 0; c < cost_functions.size(); ++c)
            {
                double cost = evaluation_cost_matrix_(i, c);
                std::cout << fixed << cost << " ";
            }
            std::cout << std::endl;
        }
        */



        for (int c = 0; c < cost_functions.size(); ++c)
        {
            double sub_cost = evaluation_cost_matrix_.col(c).sum();
            cout << setw(max_cost_name_length) << cost_functions[c]->getName();
            cout << " : " << fixed << sub_cost << std::endl;
        }

        /*
        std::cout << "torques\n";
        for (int point = 0; point < itomp_trajectory_->getNumPoints(); ++point)
        {
            std::cout << "[" << point << "] : ";
            for (int i = 0; i < joint_torques_[point].rows(); ++i)
            {
                // actuated joints
                double joint_torque = joint_torques_[point](i);
                std::cout << joint_torque << " ";
            }
            std::cout << std::endl;
        }
        std::cout << std::endl;
        */
	}
}

void NewEvalManager::initializeContactVariables()
{
	int num_contacts = planning_group_->getNumContacts();
    ROS_ASSERT(num_contacts == PlanningParameters::getInstance()->getNumContacts());

    planning_group_->is_fixed_.resize(num_contacts);
    for (int i = 0; i < num_contacts; ++i)
    {
        planning_group_->is_fixed_[i] = true;
    }

	// allocate
    contact_variables_.resize(itomp_trajectory_->getNumPoints());
	for (int i = 0; i < contact_variables_.size(); ++i)
	{
		contact_variables_[i].resize(num_contacts);
	}

    for (int point = 0; point < itomp_trajectory_->getNumPoints(); ++point)
	{
        Eigen::VectorXd q = itomp_trajectory_->getElementTrajectory(ItompTrajectory::COMPONENT_TYPE_POSITION,
                                   ItompTrajectory::SUB_COMPONENT_TYPE_JOINT)->getTrajectoryPoint(point);

        const Eigen::VectorXd& q_dot = itomp_trajectory_->getElementTrajectory(ItompTrajectory::COMPONENT_TYPE_VELOCITY,
                                       ItompTrajectory::SUB_COMPONENT_TYPE_JOINT)->getTrajectoryPoint(point);

        const Eigen::VectorXd& q_ddot = itomp_trajectory_->getElementTrajectory(ItompTrajectory::COMPONENT_TYPE_ACCELERATION,
                                        ItompTrajectory::SUB_COMPONENT_TYPE_JOINT)->getTrajectoryPoint(point);

		Eigen::VectorXd tau(q.rows());

        updateFullKinematicsAndDynamics(rbdl_models_[point], q, q_dot, q_ddot, tau, NULL, NULL);

		std::vector<RigidBodyDynamics::Math::SpatialVector> ext_forces;
        ext_forces.resize(rbdl_models_[point].mBodies.size(), RigidBodyDynamics::Math::SpatialVectorZero);

        if (point == 0 || point == itomp_trajectory_->getNumPoints() - 1)
        {
            std::vector<unsigned int> body_ids;
            std::vector<RigidBodyDynamics::Math::Vector3d> target_positions;
            std::vector<RigidBodyDynamics::Math::Matrix3d> target_orientations;

            for (int i = 0; i < num_contacts; ++i)
            {
                int rbdl_body_id = planning_group_->contact_points_[i].getRBDLBodyId();

                Eigen::Vector3d contact_normal, proj_position, proj_orientation;
                //Eigen::Matrix3d proj_orientation;
                GroundManager::getInstance()->getNearestContactPosition(rbdl_models_[point].X_base[rbdl_body_id].r,
                                                                        exponential_map::RotationToExponentialMap(rbdl_models_[point].X_base[rbdl_body_id].E),
                                                                        proj_position, proj_orientation, contact_normal);

                Eigen::Vector3d proj_dir = proj_position - rbdl_models_[point].X_base[rbdl_body_id].r;
                double d = proj_dir.dot(contact_normal);
                proj_position = rbdl_models_[point].X_base[rbdl_body_id].r + d * contact_normal;

                Matrix3d proj_orientation_mat = exponential_map::ExponentialMapToRotation(proj_orientation);


                Eigen::Vector3d diff = rbdl_models_[point].X_base[rbdl_body_id].r - proj_position;
                if (diff.norm() > 0.15)
                {
                    cout << "[" << point << ":" << i << "] not corrected using IK : " << diff.norm() << endl;

                    planning_group_->is_fixed_[i] = false;
                    continue;
                }
                else
                {
                    if (point == itomp_trajectory_->getNumPoints() - 1)
                    {
                        Eigen::Vector3d normal_start = rbdl_models_[0].X_base[rbdl_body_id].E * Eigen::Vector3d(0, 0, 1.0);
                        Eigen::Vector3d normal_goal = rbdl_models_[itomp_trajectory_->getNumPoints() - 1].X_base[rbdl_body_id].E * Eigen::Vector3d(0, 0, 1.0);
                        double dot_normal = normal_start.dot(normal_goal);

                        double dist = (proj_position - rbdl_models_[0].X_base[rbdl_body_id].r).norm();
                        cout << "[" << point << ":" << i << "] start-end dist : " << dist << " ori normal dot : " << dot_normal << endl;
                        if ((dist < 0.05 && dot_normal > 0.8))
                        {
                            proj_position = rbdl_models_[0].X_base[rbdl_body_id].r;
                            proj_orientation_mat = rbdl_models_[0].X_base[rbdl_body_id].E;
                            cout << "[" << point << ":" << i << "] set to the start position" << endl;
                        }
                    }
                }

                cout << "[" << point << ":" << i << "] corrected using IK" << endl;

                RigidBodyDynamics::Math::Matrix3d cur_ori = rbdl_models_[point].X_base[rbdl_body_id].E;

                body_ids.push_back(rbdl_body_id);
                RigidBodyDynamics::Math::Vector3d target_pos(proj_position);
                target_positions.push_back(target_pos);
                RigidBodyDynamics::Math::Matrix3d target_orientation = proj_orientation_mat;

                target_orientations.push_back(target_orientation);
            }

            std::map<int, int> rbdl_to_group_joint = planning_group_->rbdl_to_group_joint_;
            // erase root, torso, ...
            /*
            for (int i=0; i<26; i++)
                rbdl_to_group_joint.erase(i);
                */

            if (!itomp_cio_planner::InverseKinematics6D(rbdl_models_[point], q, body_ids, target_positions, target_orientations, q, rbdl_to_group_joint))
                ROS_INFO("IK failed");
            updateFullKinematicsAndDynamics(rbdl_models_[point], q, q_dot, q_ddot, tau, NULL, NULL);
            itomp_trajectory_->getElementTrajectory(ItompTrajectory::COMPONENT_TYPE_POSITION,
                                               ItompTrajectory::SUB_COMPONENT_TYPE_JOINT)->getTrajectoryPoint(point) = q;

        }

        int num_fixed_contacts = 0;
        for (int i = 0; i < num_contacts; ++i)
        {
            if (planning_group_->is_fixed_[i] == true)
                ++num_fixed_contacts;
        }

		for (int i = 0; i < num_contacts; ++i)
		{
            int rbdl_body_id = planning_group_->contact_points_[i].getRBDLBodyId();

			contact_variables_[point][i].setVariable(0.0);
            contact_variables_[point][i].setPosition(rbdl_models_[point].X_base[rbdl_body_id].r);

            const Eigen::Vector3d prev_orientation = (point == 0) ? Eigen::Vector3d::Zero() : contact_variables_[point - 1][i].getOrientation();
            const Eigen::Vector3d* prev_orientation_p = (point == 0) ? NULL : &prev_orientation;
            contact_variables_[point][i].setOrientation(exponential_map::RotationToExponentialMap(rbdl_models_[point].X_base[rbdl_body_id].E, prev_orientation_p));


			for (int j = 0; j < NUM_ENDEFFECTOR_CONTACT_POINTS; ++j)
			{
				Eigen::MatrixXd jacobian(6, q.rows());
				Eigen::Vector3d contact_position;
				Eigen::Vector3d contact_force;
				Eigen::Vector3d contact_torque;

                int rbdl_body_id = planning_group_->contact_points_[i].getContactPointRBDLIds(j);
                RigidBodyDynamics::CalcBodySpatialJacobian(rbdl_models_[point], q, rbdl_body_id, jacobian, false);

				contact_position = rbdl_models_[point].X_base[rbdl_body_id].r;
				// set forces to 0
                if (num_fixed_contacts > 0 && planning_group_->is_fixed_[i] == true)
                    contact_force = -1.0 / num_fixed_contacts / NUM_ENDEFFECTOR_CONTACT_POINTS * tau.block(0, 0, 3, 1);
                else
                    contact_force = Eigen::Vector3d::Zero();

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

        itomp_trajectory_->setContactVariables(point, contact_variables_[point]);
	}


    // check fixed contacts

    for (int i = 0; i < num_contacts; ++i)
    {
        int rbdl_body_id = planning_group_->contact_points_[i].getRBDLBodyId();
        Eigen::Vector3d pos_start = rbdl_models_[0].X_base[rbdl_body_id].r;
        Eigen::Vector3d pos_goal = rbdl_models_[itomp_trajectory_->getNumPoints() - 1].X_base[rbdl_body_id].r;

        Eigen::Vector3d normal_start = rbdl_models_[0].X_base[rbdl_body_id].E * Eigen::Vector3d(0, 0, 1.0);
        Eigen::Vector3d normal_goal = rbdl_models_[itomp_trajectory_->getNumPoints() - 1].X_base[rbdl_body_id].E * Eigen::Vector3d(0, 0, 1.0);

        double dot_normal = normal_start.dot(normal_goal);

        double pos_diff = (pos_start - pos_goal).squaredNorm();

        if (planning_group_->is_fixed_[i] == true && pos_diff < 0.05 && dot_normal > 0.8)
        {
            cout << "Contact " << i << " fixed" << endl;
        }
        else
        {
            cout << "Contact " << i << " not fixed : " << pos_diff << " : " << dot_normal << endl;
            planning_group_->is_fixed_[i] = false;
        }
    }

}

void NewEvalManager::correctContacts(bool update_kinematics)
{
    correctContacts(1, itomp_trajectory_->getNumPoints() - 1, update_kinematics);
}

void NewEvalManager::correctContacts(int point_begin, int point_end, bool update_kinematics)
{
    int num_contacts = planning_group_->getNumContacts();
    for (int point = std::max(point_begin, 1); point < std::min((unsigned int)point_end, itomp_trajectory_->getNumPoints() - 1); ++point)
    {
        std::vector<unsigned int> body_ids;
        std::vector<RigidBodyDynamics::Math::Vector3d> target_positions;
        std::vector<RigidBodyDynamics::Math::Matrix3d> target_orientations;

        ecl::QuinticPolynomial poly;
        poly = ecl::QuinticPolynomial::Interpolation(0, 0.0, 0.0, 0.0,
                                                     itomp_trajectory_->getNumPoints() - 1, 1.0, 0.0, 0.0);
        double t = poly(point);

        for (int i = 0; i < num_contacts; ++i)
        {
            if (planning_group_->is_fixed_[i])
            {
                int rbdl_body_id = planning_group_->contact_points_[i].getRBDLBodyId();
                body_ids.push_back(rbdl_body_id);

                RigidBodyDynamics::Math::Vector3d start_pos(rbdl_models_[0].X_base[rbdl_body_id].r);
                RigidBodyDynamics::Math::Vector3d goal_pos(rbdl_models_[itomp_trajectory_->getNumPoints() - 1].X_base[rbdl_body_id].r);
                RigidBodyDynamics::Math::Vector3d target_pos(start_pos * (1.0 - t) + goal_pos * t);

                Quaterniond start_ori(rbdl_models_[0].X_base[rbdl_body_id].E);
                Quaterniond end_ori(rbdl_models_[itomp_trajectory_->getNumPoints() - 1].X_base[rbdl_body_id].E);
                RigidBodyDynamics::Math::Matrix3d target_orientation(Eigen::Matrix3d(start_ori.slerp(t, end_ori)));

                target_positions.push_back(target_pos);
                target_orientations.push_back(target_orientation);
            }
        }

        Eigen::VectorXd q = itomp_trajectory_->getElementTrajectory(ItompTrajectory::COMPONENT_TYPE_POSITION,
                                   ItompTrajectory::SUB_COMPONENT_TYPE_JOINT)->getTrajectoryPoint(point);


        std::map<int, int> rbdl_to_group_joint = planning_group_->rbdl_to_group_joint_;
        // erase root, torso, ...
        /*
        for (int i=0; i<26; i++)
            rbdl_to_group_joint.erase(i);
            */


        if (!itomp_cio_planner::InverseKinematics6D(rbdl_models_[point], q, body_ids, target_positions, target_orientations, q, rbdl_to_group_joint))
            ROS_INFO("IK failed");

        // repeat above
        itomp_trajectory_->getElementTrajectory(ItompTrajectory::COMPONENT_TYPE_POSITION,
                                           ItompTrajectory::SUB_COMPONENT_TYPE_JOINT)->getTrajectoryPoint(point) = q;

        if (update_kinematics)
        {
            //RigidBodyDynamics::UpdateKinematicsCustom(rbdl_models_[point], &q, NULL, NULL);



            const Eigen::VectorXd& q_dot = itomp_trajectory_->getElementTrajectory(ItompTrajectory::COMPONENT_TYPE_VELOCITY,
                                           ItompTrajectory::SUB_COMPONENT_TYPE_JOINT)->getTrajectoryPoint(point);

            const Eigen::VectorXd& q_ddot = itomp_trajectory_->getElementTrajectory(ItompTrajectory::COMPONENT_TYPE_ACCELERATION,
                                            ItompTrajectory::SUB_COMPONENT_TYPE_JOINT)->getTrajectoryPoint(point);

            Eigen::VectorXd tau(q.rows());

            updateFullKinematicsAndDynamics(rbdl_models_[point], q, q_dot, q_ddot, tau, NULL, NULL);

            std::vector<RigidBodyDynamics::Math::SpatialVector> ext_forces;
            ext_forces.resize(rbdl_models_[point].mBodies.size(), RigidBodyDynamics::Math::SpatialVectorZero);

            for (int i = 0; i < num_contacts; ++i)
            {
                if (!getPlanningGroup()->is_fixed_[i])
                    continue;

                int rbdl_body_id = planning_group_->contact_points_[i].getRBDLBodyId();

                contact_variables_[point][i].setVariable(0.0);
                contact_variables_[point][i].setPosition(rbdl_models_[point].X_base[rbdl_body_id].r);

                const Eigen::Vector3d prev_orientation = (point == 0) ? Eigen::Vector3d::Zero() : contact_variables_[point - 1][i].getOrientation();
                const Eigen::Vector3d* prev_orientation_p = (point == 0) ? NULL : &prev_orientation;
                contact_variables_[point][i].setOrientation(exponential_map::RotationToExponentialMap(rbdl_models_[point].X_base[rbdl_body_id].E, prev_orientation_p));
            }

            // to validate
            //RigidBodyDynamics::InverseDynamics(rbdl_models_[point], q, q_dot, q_ddot, tau, &ext_forces);

            itomp_trajectory_->setContactVariables(point, contact_variables_[point]);
        }
    }
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

void NewEvalManager::computePassiveForces(int point,
                                          const RigidBodyDynamics::Math::VectorNd &q,
                                          const RigidBodyDynamics::Math::VectorNd &q_dot,
                                          std::vector<double>& passive_forces)
{
    const double K_P = 50.0 * PlanningParameters::getInstance()->getPassiveForceRatio();
    const double K_D = 1.0 * PlanningParameters::getInstance()->getPassiveForceRatio();

    int num_joints = itomp_trajectory_->getElementTrajectory(ItompTrajectory::COMPONENT_TYPE_POSITION,
                     ItompTrajectory::SUB_COMPONENT_TYPE_JOINT)->getNumElements();

    // rocket_box
    /*
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
    */

    // beta

    for (int i = 1; i <= num_joints; ++i)
    {
        int q_index = rbdl_models_[point].mJoints[i].q_index;

        if ((q_index >= 3 && q_index <= 5) ||
            (q_index >= 46 && q_index <= 54) ||
            (q_index >= 65 && q_index <= 70))
        {
            passive_forces[i] = -K_P * q(q_index) -K_D * q_dot(q_index);
        }
        else
        {
            //passive_forces[i] = -K_D * q_dot(q_index);
        }
    }

}

}

