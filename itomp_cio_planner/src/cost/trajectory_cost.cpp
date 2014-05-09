/*
 * trajectoryCost.cpp
 *
 *  Created on: Oct 23, 2013
 *      Author: cheonhyeonpark
 */

#include <itomp_cio_planner/optimization/evaluation_manager.h>
#include <itomp_cio_planner/cost/trajectory_cost.h>
#include <itomp_cio_planner/util/planning_parameters.h>

using namespace std;

namespace itomp_cio_planner
{
//static int LeftLegStart = 0;

TrajectoryCost* TrajectoryCost::CreateTrajectoryCost(COST_TYPE type)
{
	TrajectoryCost* newCost = NULL;
	switch (type)
	{
	case COST_SMOOTHNESS:
		newCost = new TrajectorySmoothnessCost();
		break;

	case COST_COLLISION:
		newCost = new TrajectoryCollisionCost();
		break;

	case COST_VALIDITY:
		newCost = new TrajectoryValidityCost();
		break;

	case COST_CONTACT_INVARIANT:
		newCost = new TrajectoryContactInvariantCost();
		break;

	case COST_PHYSICS_VIOLATION:
		newCost = new TrajectoryPhysicsViolationCost();
		break;

	case COST_GOAL_POSE:
		newCost = new TrajectoryGoalPoseCost();
		break;

	case COST_COM:
		newCost = new TrajectoryCoMCost();
		break;

	case COST_ENDEFFECTOR_VELOCITY:
	case COST_TORQUE:
	case COST_RVO:
	default:
		assert(false);
		break;
	}

	return newCost;
}

////////////////////////////////////////////////////////////////////////////////

void TrajectoryCost::init(const EvaluationManager* evaluator)
{
	costs_ = Eigen::VectorXd::Zero(evaluator->num_points_);
}

void TrajectoryCost::computeCostSum(const EvaluationManager* evaluator)
{
	costSum_ = 0.0;
	for (int i = 1; i <= evaluator->num_points_ - 2; i++)
	{
		costSum_ += costs_(i);
	}
}

void TrajectoryCost::compute(const EvaluationManager* evaluator)
{
	doCompute(evaluator);
	computeCostSum(evaluator);
}
////////////////////////////////////////////////////////////////////////////////

void TrajectorySmoothnessCost::doCompute(const EvaluationManager* evaluator)
{
	double smoothness_cost = 0.0;
	// joint costs:
	for (int i = 0; i < evaluator->num_joints_; i++)
		smoothness_cost += evaluator->joint_costs_[i].getCost(evaluator->group_trajectory_->getJointTrajectory(i));

	costs_(1) = smoothness_cost;
}

double TrajectorySmoothnessCost::getWeight() const
{
	return PlanningParameters::getInstance()->getSmoothnessCostWeight();
}
////////////////////////////////////////////////////////////////////////////////

void TrajectoryCollisionCost::doCompute(const EvaluationManager* evaluator)
{
  for (int i = 1; i <= evaluator->num_points_ - 2; i++)
    {
      costs_(i) = evaluator->stateCollisionCost_[i];
    }
}

double TrajectoryCollisionCost::getWeight() const
{
	return PlanningParameters::getInstance()->getObstacleCostWeight();
}
////////////////////////////////////////////////////////////////////////////////

void TrajectoryValidityCost::doCompute(const EvaluationManager* evaluator)
{
  for (int i = 1; i <= evaluator->num_points_ - 2; i++)
	{
		double state_validity_cost = 0.0;
		if (!evaluator->state_validity_[i])
			state_validity_cost = 1.0;

		costs_(i) = state_validity_cost;
	}
}

double TrajectoryValidityCost::getWeight() const
{
	return PlanningParameters::getInstance()->getStateValidityCostWeight();
}
////////////////////////////////////////////////////////////////////////////////

void TrajectoryContactInvariantCost::doCompute(const EvaluationManager* evaluator)
{
  for (int i = 1; i <= evaluator->num_points_ - 2; i++)
	{
		costs_(i) = evaluator->stateContactInvariantCost_[i];
	}
}

double TrajectoryContactInvariantCost::getWeight() const
{
	return PlanningParameters::getInstance()->getContactInvariantCostWeight();
}
////////////////////////////////////////////////////////////////////////////////

void TrajectoryPhysicsViolationCost::doCompute(const EvaluationManager* evaluator)
{
  for (int i = 1; i <= evaluator->num_points_ - 2; i++)
	{
		costs_(i) = evaluator->statePhysicsViolationCost_[i];
	}
}

double TrajectoryPhysicsViolationCost::getWeight() const
{
	return PlanningParameters::getInstance()->getPhysicsViolationCostWeight();
}
////////////////////////////////////////////////////////////////////////////////

void TrajectoryGoalPoseCost::doCompute(const EvaluationManager* evaluator)
{
  /*
	double goal_pose_cost = 0.0;
	if (evaluator->planning_group_->name_ == "lower_body" || evaluator->planning_group_->name_ == "whole_body")
	{
		goal_pose_cost += (evaluator->getSegmentPosition(evaluator->free_vars_end_,
				PlanningParameters::getInstance()->getLowerBodyRoot()) - evaluator->getSegmentPosition(
				evaluator->free_vars_end_ + 1, PlanningParameters::getInstance()->getLowerBodyRoot())).Norm();
		goal_pose_cost += (evaluator->getSegmentPosition(evaluator->free_vars_end_, "left_foot_endeffector_link")
				- evaluator->getSegmentPosition(evaluator->free_vars_end_ + 1, "left_foot_endeffector_link")).Norm();

		KDL::Vector rightFoot =
				(evaluator->getSegmentPosition(evaluator->free_vars_end_, "right_foot_endeffector_link")
						- evaluator->getSegmentPosition(evaluator->free_vars_end_ + 1, "right_foot_endeffector_link"));
		goal_pose_cost += rightFoot.Norm();
	}

	for (int i = 0; i < evaluator->num_points_ - 2; i++)
	{
		costs_(i) = goal_pose_cost / evaluator->num_points_ - 2;
	}
	*/
}

double TrajectoryGoalPoseCost::getWeight() const
{
	return PlanningParameters::getInstance()->getGoalPoseCostWeight();
}
////////////////////////////////////////////////////////////////////////////////

void TrajectoryCoMCost::doCompute(const EvaluationManager* evaluator)
{
  /*
	for (int i = evaluator->free_vars_start_; i <= evaluator->free_vars_end_; i++)
	{
		double state_CoM_cost = 0.0;
		if (evaluator->planning_group_->name_ == "lower_body" || evaluator->planning_group_->name_ == "whole_body")
		{
			state_CoM_cost += evaluator->CoMAccelerations_[i].Norm();
			if (evaluator->CoMVelocities_[i].x() < 0.0)
				state_CoM_cost += -evaluator->CoMVelocities_[i].x()
						* PlanningParameters::getInstance()->getTemporaryVariable(0);

			// check CoM is in two legs
			const string contact_foot = (evaluator->group_trajectory_->getContactPhase(i)/2
					+ LeftLegStart) % 2 == 0 ? "left_foot_endeffector_link"
					: "right_foot_endeffector_link";
			const string other_foot = (evaluator->group_trajectory_->getContactPhase(i)/2
					+ LeftLegStart) % 2 == 1 ? "left_foot_endeffector_link"
					: "right_foot_endeffector_link";
			int ef_sn = evaluator->robot_model_->getForwardKinematicsSolver()->segmentNameToIndex(contact_foot);
			int of_sn = evaluator->robot_model_->getForwardKinematicsSolver()->segmentNameToIndex(other_foot);
			KDL::Vector ef = evaluator->segment_frames_[i][ef_sn].p;
			KDL::Vector of = evaluator->segment_frames_[i][of_sn].p;
			double maxX = max(ef.x(), of.x()) - 0.028;
			double minX = min(ef.x(), of.x()) - 0.028;
			double centerX = (ef.x() + of.x()) * 0.5f - 0.028;
			state_CoM_cost += abs(evaluator->CoMPositions_[i].x() - centerX)
					* PlanningParameters::getInstance()->getTemporaryVariable(1);

			state_CoM_cost /= (evaluator->free_vars_end_ - evaluator->free_vars_start_ + 1);
		}

		costs_(i - evaluator->free_vars_start_) = state_CoM_cost;
	}
	*/
}

double TrajectoryCoMCost::getWeight() const
{
	return PlanningParameters::getInstance()->getCoMCostWeight();
}

}
