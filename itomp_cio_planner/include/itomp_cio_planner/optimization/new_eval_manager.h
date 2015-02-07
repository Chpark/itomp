#ifndef NEW_EVALUATION_MANAGER_H_
#define NEW_EVALUATION_MANAGER_H_

#include <itomp_cio_planner/common.h>
#include <itomp_cio_planner/model/itomp_robot_model.h>
#include <itomp_cio_planner/trajectory/full_trajectory.h>
#include <itomp_cio_planner/trajectory/itomp_trajectory.h>
#include <itomp_cio_planner/trajectory/parameter_trajectory.h>
#include <itomp_cio_planner/contact/contact_variables.h>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <ros/publisher.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/robot_state.h>
#include <itomp_cio_planner/collision/collision_world_fcl_derivatives.h>
#include <itomp_cio_planner/collision/collision_robot_fcl_derivatives.h>

namespace itomp_cio_planner
{
ITOMP_FORWARD_DECL(NewEvalManager)

class NewEvalManager
{
public:
	NewEvalManager();
    NewEvalManager(const NewEvalManager& manager);
	virtual ~NewEvalManager();

    NewEvalManager& operator=(const NewEvalManager& manager);

	void initialize(const FullTrajectoryPtr& full_trajectory,
                    const ItompTrajectoryPtr& itomp_trajectory,
					const ItompRobotModelConstPtr& robot_model,
					const planning_scene::PlanningSceneConstPtr& planning_scene,
					const ItompPlanningGroupConstPtr& planning_group,
					double planning_start_time, double trajectory_start_time,
					const moveit_msgs::Constraints& path_constraints);

	const FullTrajectoryConstPtr& getFullTrajectory() const;
	const ParameterTrajectoryConstPtr& getParameterTrajectory() const;
    const ItompTrajectoryConstPtr& getTrajectory() const;

	void getParameters(std::vector<Eigen::MatrixXd>& parameters) const;
	void setParameters(const std::vector<Eigen::MatrixXd>& parameters);
    void getParameters(ItompTrajectory::ParameterVector& parameters) const;
    void setParameters(const ItompTrajectory::ParameterVector& parameters);

	double evaluate();
	void evaluateParameterPoint(double value, int type, int point, int element,
								int& full_point_begin, int& full_point_end, bool first);
    void evaluateParameterPointItomp(double value, int parameter_index,
                                unsigned int& point_begin, unsigned int& point_end, bool first);

	void computeDerivatives(const std::vector<Eigen::MatrixXd>& parameters,
							int type, int point, double* out, double eps, double* d_p, double* d_m, std::vector<std::vector<double> >* cost_der = NULL);
    void computeDerivatives(int parameter_index, const ItompTrajectory::ParameterVector& parameters,
                            double* derivative_out, double eps);

	bool isLastTrajectoryFeasible() const;
	double getTrajectoryCost() const;
	void printTrajectoryCost(int iteration, bool details = false);

	void render();

	void updateFromParameterTrajectory();

	const planning_scene::PlanningSceneConstPtr& getPlanningScene() const;
	const RigidBodyDynamics::Model& getRBDLModel(int point) const;
	const ItompPlanningGroupConstPtr& getPlanningGroup() const;
	const ItompRobotModelConstPtr& getItompRobotModel() const;
	const robot_state::RobotStatePtr& getRobotState(int point) const;

    const CollisionWorldFCLDerivativesPtr& getCollisionWorldFCLDerivatives() const;
    const CollisionRobotFCLDerivativesPtr& getCollisionRobotFCLDerivatives() const;

private:
	void initializeContactVariables();

	void performFullForwardKinematicsAndDynamics(int point_begin, int point_end);
	void performPartialForwardKinematicsAndDynamics(int point_begin, int point_end, int parameter_element);
    void performPartialForwardKinematicsAndDynamics(int point_begin, int point_end, const ItompTrajectoryIndex& index);

	void setParameterModified();

	bool evaluatePointRange(int point_begin, int point_end,
							Eigen::MatrixXd& cost_matrix, int type = -1, int element = -1);
    bool evaluatePointRange(int point_begin, int point_end,
                            Eigen::MatrixXd& cost_matrix,
                            const ItompTrajectoryIndex& index);

	bool isDerivative() const;

    // shared constant pointer members
    ItompRobotModelConstPtr robot_model_;
    planning_scene::PlanningSceneConstPtr planning_scene_;
    ItompPlanningGroupConstPtr planning_group_;

    // non-pointer members
	double planning_start_time_;
	double trajectory_start_time_;
	bool last_trajectory_feasible_;
    bool parameter_modified_;
    double best_cost_;

	std::vector<RigidBodyDynamics::Model> rbdl_models_;
    std::vector<Eigen::VectorXd> joint_torques_; // computed from inverse dynamics
	std::vector<std::vector<RigidBodyDynamics::Math::SpatialVector> > external_forces_;
	std::vector<std::vector<ContactVariables> > contact_variables_;

	Eigen::MatrixXd evaluation_cost_matrix_;

    static const NewEvalManager* ref_evaluation_manager_;

    // non-shared pointer members
    FullTrajectoryPtr full_trajectory_;
    ParameterTrajectoryPtr parameter_trajectory_;
    FullTrajectoryConstPtr full_trajectory_const_;
    ParameterTrajectoryConstPtr parameter_trajectory_const_;
    ItompTrajectoryPtr itomp_trajectory_;
    ItompTrajectoryConstPtr itomp_trajectory_const_;
    std::vector<robot_state::RobotStatePtr> robot_state_;
    CollisionWorldFCLDerivativesPtr collision_world_derivatives_;
    CollisionRobotFCLDerivativesPtr collision_robot_derivatives_;

	friend class TrajectoryCostContactInvariant;
	friend class TrajectoryCostObstacle;
	friend class TrajectoryCostPhysicsViolation;
	friend class TrajectoryCostTorque;
	friend class TrajectoryCostFTR;
	friend class TrajectoryCostFrictionCone;
	friend class TrajectoryCostCOM;
	friend class TrajectoryCostEndeffectorVelocity;
	friend class TrajectoryCostROM;
};
ITOMP_DEFINE_SHARED_POINTERS(NewEvalManager)

////////////////////////////////////////////////////////////////////////////////

inline const FullTrajectoryConstPtr& NewEvalManager::getFullTrajectory() const
{
	return full_trajectory_const_;
}

inline const ParameterTrajectoryConstPtr& NewEvalManager::getParameterTrajectory() const
{
	return parameter_trajectory_const_;
}

inline const ItompTrajectoryConstPtr& NewEvalManager::getTrajectory() const
{
    return itomp_trajectory_const_;
}

inline bool NewEvalManager::isLastTrajectoryFeasible() const
{
	return last_trajectory_feasible_;
}

inline void NewEvalManager::setParameterModified()
{
	parameter_modified_ = true;
}

inline double NewEvalManager::getTrajectoryCost() const
{
	return evaluation_cost_matrix_.sum();
}

inline const planning_scene::PlanningSceneConstPtr& NewEvalManager::getPlanningScene() const
{
	return planning_scene_;
}

inline const ItompPlanningGroupConstPtr& NewEvalManager::getPlanningGroup() const
{
	return planning_group_;
}

inline const RigidBodyDynamics::Model& NewEvalManager::getRBDLModel(int point) const
{
	return rbdl_models_[point];
}

inline const ItompRobotModelConstPtr& NewEvalManager::getItompRobotModel() const
{
	return robot_model_;
}

inline const robot_state::RobotStatePtr& NewEvalManager::getRobotState(int point) const
{
	return robot_state_[point];
}

inline const CollisionWorldFCLDerivativesPtr& NewEvalManager::getCollisionWorldFCLDerivatives() const
{
    return collision_world_derivatives_;
}

inline const CollisionRobotFCLDerivativesPtr& NewEvalManager::getCollisionRobotFCLDerivatives() const
{
    return collision_robot_derivatives_;
}

}

#endif
