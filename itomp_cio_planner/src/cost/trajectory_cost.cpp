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
		const NewEvalManager* evaluation_manager,
		const FullTrajectoryConstPtr& trajectory, int point, double& cost) const
{
	ROS_ASSERT(trajectory->hasAcceleration());

	//cost = trajectory->getTrajectory(Trajectory::TRAJECTORY_TYPE_ACCELERATION).row(point).array().abs().sum();
	cost = 0;
	double value;
	const Eigen::MatrixXd mat_acc = trajectory->getTrajectory(Trajectory::TRAJECTORY_TYPE_ACCELERATION);
	for (int i = 0; i < mat_acc.cols(); ++i)
	{
		value = std::abs(mat_acc(point, i));
		cost += value * value;
	}

	// normalize cost (independent to # of joints)
	cost /= trajectory->getComponentSize(FullTrajectory::TRAJECTORY_COMPONENT_JOINT);

	return true;
}

bool TrajectoryCostObstacle::evaluate(const NewEvalManager* evaluation_manager,
		const FullTrajectoryConstPtr& trajectory, int point, double& cost) const
{
	bool is_feasible = true;
	cost = 0;

	// implement

	return is_feasible;
}

bool TrajectoryCostValidity::evaluate(const NewEvalManager* evaluation_manager,
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

bool TrajectoryCostGoalPose::evaluate(const NewEvalManager* evaluation_manager,
		const FullTrajectoryConstPtr& trajectory, int point, double& cost) const
{
	bool is_feasible = true;
	cost = 0;

	// implement

	return is_feasible;
}

bool TrajectoryCostCOM::evaluate(const NewEvalManager* evaluation_manager,
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

bool TrajectoryCostTorque::evaluate(const NewEvalManager* evaluation_manager,
		const FullTrajectoryConstPtr& trajectory, int point, double& cost) const
{
	bool is_feasible = true;
	cost = 0;

	// implement

	return is_feasible;
}

bool TrajectoryCostRVO::evaluate(const NewEvalManager* evaluation_manager,
		const FullTrajectoryConstPtr& trajectory, int point, double& cost) const
{
	bool is_feasible = true;
	cost = 0;

	// implement

	return is_feasible;
}

bool TrajectoryCostFTR::evaluate(const NewEvalManager* evaluation_manager,
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
