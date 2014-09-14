#include <itomp_cio_planner/cost/trajectory_cost.h>

namespace itomp_cio_planner
{

TrajectoryCost::TrajectoryCost(int index, std::string name, double weight) :
		index_(index), name_(name), weight_(weight)
{

}

TrajectoryCost::~TrajectoryCost()
{

}

bool TrajectoryCostSmoothness::evaluate(
		const NewEvalManager* evaluation_manager, int point, double& cost) const
{
	TIME_PROFILER_START_TIMER(Smoothness);

	const FullTrajectoryConstPtr trajectory =
			evaluation_manager->getFullTrajectory();
	ROS_ASSERT(trajectory->hasAcceleration());

	cost = 0;
	double value;
	const Eigen::MatrixXd mat_acc = trajectory->getTrajectory(
			Trajectory::TRAJECTORY_TYPE_ACCELERATION);
	for (int i = 0; i < mat_acc.cols(); ++i)
	{
		value = std::abs(mat_acc(point, i));
		cost += value * value;
	}

	// normalize cost (independent to # of joints)
	cost /= trajectory->getComponentSize(
			FullTrajectory::TRAJECTORY_COMPONENT_JOINT);

	TIME_PROFILER_END_TIMER(Smoothness);

	return true;
}

bool TrajectoryCostObstacle::evaluate(const NewEvalManager* evaluation_manager,
		int point, double& cost) const
{
	TIME_PROFILER_START_TIMER(Obstacle);

	bool is_feasible = true;
	cost = 0;

	const FullTrajectoryConstPtr trajectory =
			evaluation_manager->getFullTrajectory();
	robot_state::RobotStatePtr robot_state = evaluation_manager->robot_state_;
	const planning_scene::PlanningSceneConstPtr planning_scene =
			evaluation_manager->getPlanningScene();

	ROS_ASSERT(
			robot_state->getVariableCount() == trajectory->getComponentSize(FullTrajectory::TRAJECTORY_COMPONENT_JOINT));

	collision_detection::CollisionRequest collision_request;
	collision_detection::CollisionResult collision_result;
	collision_request.verbose = false;
	collision_request.contacts = true;
	collision_request.max_contacts = 1000;

	const Eigen::MatrixXd mat = trajectory->getTrajectory(
			Trajectory::TRAJECTORY_TYPE_POSITION).row(point);
	robot_state->setVariablePositions(mat.data());

	const double self_collision_scale = 0.1;

	planning_scene->checkCollisionUnpadded(collision_request, collision_result,
			*robot_state);

	const collision_detection::CollisionResult::ContactMap& contact_map =
			collision_result.contacts;
	for (collision_detection::CollisionResult::ContactMap::const_iterator it =
			contact_map.begin(); it != contact_map.end(); ++it)
	{
		const collision_detection::Contact& contact = it->second[0];

		if (contact.body_type_1 != collision_detection::BodyTypes::WORLD_OBJECT
				&& contact.body_type_2
						!= collision_detection::BodyTypes::WORLD_OBJECT)
			cost += self_collision_scale * contact.depth;
		else
			cost += contact.depth;

	}
	collision_result.clear();

	is_feasible = (cost == 0.0);

	TIME_PROFILER_END_TIMER(Obstacle);

	return is_feasible;
}

bool TrajectoryCostValidity::evaluate(const NewEvalManager* evaluation_manager,
		int point, double& cost) const
{
	bool is_feasible = true;
	cost = 0;

	// implement

	return is_feasible;
}

bool TrajectoryCostContactInvariant::evaluate(
		const NewEvalManager* evaluation_manager, int point, double& cost) const
{
	bool is_feasible = true;
	cost = 0;

	// implement

	return is_feasible;
}

bool TrajectoryCostPhysicsViolation::evaluate(
		const NewEvalManager* evaluation_manager, int point, double& cost) const
{
	bool is_feasible = true;
	cost = 0;

	// implement

	return is_feasible;
}

bool TrajectoryCostGoalPose::evaluate(const NewEvalManager* evaluation_manager,
		int point, double& cost) const
{
	bool is_feasible = true;
	cost = 0;

	// implement

	return is_feasible;
}

bool TrajectoryCostCOM::evaluate(const NewEvalManager* evaluation_manager,
		int point, double& cost) const
{
	bool is_feasible = true;
	cost = 0;

	// implement

	return is_feasible;
}

bool TrajectoryCostEndeffectorVelocity::evaluate(
		const NewEvalManager* evaluation_manager, int point, double& cost) const
{
	bool is_feasible = true;
	cost = 0;

	// implement

	return is_feasible;
}

bool TrajectoryCostTorque::evaluate(const NewEvalManager* evaluation_manager,
		int point, double& cost) const
{
	bool is_feasible = true;
	cost = 0;

	// implement

	return is_feasible;
}

bool TrajectoryCostRVO::evaluate(const NewEvalManager* evaluation_manager,
		int point, double& cost) const
{
	bool is_feasible = true;
	cost = 0;

	// implement

	return is_feasible;
}

bool TrajectoryCostFTR::evaluate(const NewEvalManager* evaluation_manager,
		int point, double& cost) const
{
	bool is_feasible = true;
	cost = 0;

	// implement

	return is_feasible;
}

bool TrajectoryCostCartesianTrajectory::evaluate(
		const NewEvalManager* evaluation_manager, int point, double& cost) const
{
	bool is_feasible = true;
	cost = 0;

	// implement

	return is_feasible;
}

bool TrajectoryCostSingularity::evaluate(
		const NewEvalManager* evaluation_manager, int point, double& cost) const
{
	bool is_feasible = true;
	cost = 0;

	// implement

	return is_feasible;
}

}
