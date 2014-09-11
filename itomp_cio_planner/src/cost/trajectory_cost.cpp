#include <itomp_cio_planner/cost/trajectory_cost.h>

namespace itomp_cio_planner
{

TrajectoryCost::TrajectoryCost(int index, std::string name,
		double weight) :
		index_(index), name_(name), weight_(weight)
{

}

TrajectoryCost::~TrajectoryCost()
{

}

bool TrajectoryCostSmoothness::evaluate(
		const NewEvalManager* evaluation_manager,
		const FullTrajectoryConstPtr& trajectory, int point, double& cost) const
{
	bool is_feasible = true;
	cost = 0;

	// implement

	return is_feasible;
}

bool TrajectoryCostCollision::evaluate(
		const NewEvalManager* evaluation_manager,
		const FullTrajectoryConstPtr& trajectory, int point, double& cost) const
{
	bool is_feasible = true;
	cost = 0;

	// implement

	return is_feasible;
}

bool TrajectoryCostValidity::evaluate(
		const NewEvalManager* evaluation_manager,
		const FullTrajectoryConstPtr& trajectory, int point, double& cost) const
{
	bool is_feasible = true;
	cost = 0;

	// implement

	return is_feasible;
}

bool TrajectoryCostContactInvariant::evaluate(
		const NewEvalManager* evaluation_manager,
		const FullTrajectoryConstPtr& trajectory, int point, double& cost) const
{
	bool is_feasible = true;
	cost = 0;

	// implement

	return is_feasible;
}

bool TrajectoryCostPhysicsViolation::evaluate(
		const NewEvalManager* evaluation_manager,
		const FullTrajectoryConstPtr& trajectory, int point, double& cost) const
{
	bool is_feasible = true;
	cost = 0;

	// implement

	return is_feasible;
}

bool TrajectoryCostGoalPose::evaluate(
		const NewEvalManager* evaluation_manager,
		const FullTrajectoryConstPtr& trajectory, int point, double& cost) const
{
	bool is_feasible = true;
	cost = 0;

	// implement

	return is_feasible;
}

bool TrajectoryCostCOM::evaluate(
		const NewEvalManager* evaluation_manager,
		const FullTrajectoryConstPtr& trajectory, int point, double& cost) const
{
	bool is_feasible = true;
	cost = 0;

	// implement

	return is_feasible;
}

bool TrajectoryCostEndeffectorVelocity::evaluate(
		const NewEvalManager* evaluation_manager,
		const FullTrajectoryConstPtr& trajectory, int point, double& cost) const
{
	bool is_feasible = true;
	cost = 0;

	// implement

	return is_feasible;
}

bool TrajectoryCostTorque::evaluate(
		const NewEvalManager* evaluation_manager,
		const FullTrajectoryConstPtr& trajectory, int point, double& cost) const
{
	bool is_feasible = true;
	cost = 0;

	// implement

	return is_feasible;
}

bool TrajectoryCostRVO::evaluate(
		const NewEvalManager* evaluation_manager,
		const FullTrajectoryConstPtr& trajectory, int point, double& cost) const
{
	bool is_feasible = true;
	cost = 0;

	// implement

	return is_feasible;
}

bool TrajectoryCostFTR::evaluate(
		const NewEvalManager* evaluation_manager,
		const FullTrajectoryConstPtr& trajectory, int point, double& cost) const
{
	bool is_feasible = true;
	cost = 0;

	// implement

	return is_feasible;
}

bool TrajectoryCostCartesianTrajectory::evaluate(
		const NewEvalManager* evaluation_manager,
		const FullTrajectoryConstPtr& trajectory, int point, double& cost) const
{
	bool is_feasible = true;
	cost = 0;

	// implement

	return is_feasible;
}

bool TrajectoryCostSingularity::evaluate(
		const NewEvalManager* evaluation_manager,
		const FullTrajectoryConstPtr& trajectory, int point, double& cost) const
{
	bool is_feasible = true;
	cost = 0;

	// implement

	return is_feasible;
}

}
