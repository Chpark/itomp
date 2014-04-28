/*
 * trajectoryCostAccumulator.cpp
 *
 *  Created on: Oct 23, 2013
 *      Author: cheonhyeonpark
 */

#include <itomp_cio_planner/cost/trajectory_cost_accumulator.h>
#include <itomp_cio_planner/optimization/evaluation_manager.h>
#include <ros/console.h>

namespace itomp_cio_planner
{

TrajectoryCostAccumulator::TrajectoryCostAccumulator()
{

}

TrajectoryCostAccumulator::~TrajectoryCostAccumulator()
{
	for (std::map<TrajectoryCost::COST_TYPE, TrajectoryCost*>::iterator it = costMap_.begin(); it != costMap_.end();
			++it)
	{
		delete it->second;
	}
	costMap_.clear();
}

void TrajectoryCostAccumulator::addCost(TrajectoryCost* cost)
{
	if (cost)
	{
		costMap_[cost->getType()] = cost;
	}
}

void TrajectoryCostAccumulator::init(const EvaluationManager* evaluator)
{
	for (std::map<TrajectoryCost::COST_TYPE, TrajectoryCost*>::iterator it = costMap_.begin(); it != costMap_.end();
			++it)
	{
		it->second->init(evaluator);
	}
}

void TrajectoryCostAccumulator::compute(const EvaluationManager* evaluator)
{
	// TODO:
	//const_cast<EvaluationManager*>(evaluator)->computeTrajectoryValidity();
	const_cast<EvaluationManager*>(evaluator)->computeWrenchSum();
	const_cast<EvaluationManager*>(evaluator)->computeStabilityCosts();

	for (std::map<TrajectoryCost::COST_TYPE, TrajectoryCost*>::iterator it = costMap_.begin(); it != costMap_.end();
			++it)
	{
		if (it->second->getWeight() != 0.0)
			it->second->compute(evaluator);
	}
}

double TrajectoryCostAccumulator::getWaypointCost(int waypoint) const
{
	double accumulatedCost = 0.0;
	for (std::map<TrajectoryCost::COST_TYPE, TrajectoryCost*>::const_iterator it = costMap_.begin();
			it != costMap_.end(); ++it)
	{
		accumulatedCost += it->second->getWaypointCost(waypoint) * it->second->getWeight();
	}
	return accumulatedCost;
}

double TrajectoryCostAccumulator::getWaypointCost(int waypoint, TrajectoryCost::COST_TYPE type) const
{
	std::map<TrajectoryCost::COST_TYPE, TrajectoryCost*>::const_iterator it = costMap_.find(type);
	return it->second->getWaypointCost(waypoint) * it->second->getWeight();
}

double TrajectoryCostAccumulator::getTrajectoryCost(TrajectoryCost::COST_TYPE type) const
{
	std::map<TrajectoryCost::COST_TYPE, TrajectoryCost*>::const_iterator it = costMap_.find(type);
	if (it != costMap_.end())
		return it->second->getTrajectoryCost() * it->second->getWeight();
	return 0.0;
}

double TrajectoryCostAccumulator::getTrajectoryCost() const
{
	double accumulatedCost = 0.0;
	for (std::map<TrajectoryCost::COST_TYPE, TrajectoryCost*>::const_iterator it = costMap_.begin();
			it != costMap_.end(); ++it)
	{
		accumulatedCost += it->second->getTrajectoryCost() * it->second->getWeight();
	}
	return accumulatedCost;
}

void TrajectoryCostAccumulator::print(int number) const
{
	ROS_INFO(
			"[%d] Trajectory cost : %f (s=%f, c=%f, v=%f, ci=%f pv=%f gp=%f com=%f)", number, getTrajectoryCost(), getTrajectoryCost(TrajectoryCost::COST_SMOOTHNESS), getTrajectoryCost(TrajectoryCost::COST_COLLISION), getTrajectoryCost(TrajectoryCost::COST_VALIDITY), getTrajectoryCost(TrajectoryCost::COST_CONTACT_INVARIANT), getTrajectoryCost(TrajectoryCost::COST_PHYSICS_VIOLATION), getTrajectoryCost(TrajectoryCost::COST_GOAL_POSE), getTrajectoryCost(TrajectoryCost::COST_COM));

}

bool TrajectoryCostAccumulator::isFeasible() const
{
	//if (getTrajectoryCost(TrajectoryCost::COST_CONTACT_INVARIANT) > 1E-7)
	//return false;

	return true;
}

}
