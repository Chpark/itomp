#include <itomp_cio_planner/cost/trajectory_cost_manager.h>
#include <itomp_cio_planner/util/planning_parameters.h>

namespace itomp_cio_planner
{

TrajectoryCostManager::TrajectoryCostManager()
{

}

TrajectoryCostManager::~TrajectoryCostManager()
{

}

void TrajectoryCostManager::buildActiveCostFunctions(const NewEvalManager* evaluation_manager)
{
	cost_function_vector_.clear();
	int index = 0;

	ITOMP_TRAJECTORY_COST_ADD(Smoothness)
	ITOMP_TRAJECTORY_COST_ADD(Obstacle)
	ITOMP_TRAJECTORY_COST_ADD(Validity)
	ITOMP_TRAJECTORY_COST_ADD(ContactInvariant)
	ITOMP_TRAJECTORY_COST_ADD(PhysicsViolation)
	ITOMP_TRAJECTORY_COST_ADD(GoalPose)
	ITOMP_TRAJECTORY_COST_ADD(COM)
	ITOMP_TRAJECTORY_COST_ADD(EndeffectorVelocity)
	ITOMP_TRAJECTORY_COST_ADD(Torque)
	ITOMP_TRAJECTORY_COST_ADD(RVO)
	ITOMP_TRAJECTORY_COST_ADD(FTR)
	ITOMP_TRAJECTORY_COST_ADD(ROM)
	ITOMP_TRAJECTORY_COST_ADD(CartesianTrajectory)
	ITOMP_TRAJECTORY_COST_ADD(Singularity)
	ITOMP_TRAJECTORY_COST_ADD(FrictionCone)

    ROS_INFO("Loaded trajectory cost functions");
    for (int i = 0; i < cost_function_vector_.size(); ++i)
        ROS_INFO("[%d] %s : %f", i, cost_function_vector_[i]->getName().c_str(), cost_function_vector_[i]->getWeight());
}

}
