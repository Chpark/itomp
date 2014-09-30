/*
 * trajectoryCostAccumulator.cpp
 *
 *  Created on: Oct 23, 2013
 *      Author: cheonhyeonpark
 */

#include <itomp_cio_planner/cost/trajectory_cost_accumulator.h>
#include <itomp_cio_planner/optimization/evaluation_data.h>
#include <ros/console.h>

namespace itomp_cio_planner
{

TrajectoryCostAccumulator::TrajectoryCostAccumulator() :
		is_last_trajectory_valid_(true)
{
	best_cost_ = std::numeric_limits<double>::max();
}

TrajectoryCostAccumulator::~TrajectoryCostAccumulator()
{

}

void TrajectoryCostAccumulator::addCost(TrajectoryCostPtr cost)
{
	if (cost != NULL)
	{
		costMap_[cost->getType()] = cost;
	}
}

void TrajectoryCostAccumulator::init(const EvaluationData* data)
{
	for (std::map<TrajectoryCost::COST_TYPE, TrajectoryCostPtr>::iterator it =
			costMap_.begin(); it != costMap_.end(); ++it)
	{
		it->second->init(data);
		costDataMap_[it->first] = Eigen::VectorXd::Zero(data->getNumPoints());
		costSumMap_[it->first] = 0.0;
	}
}

void TrajectoryCostAccumulator::compute(const EvaluationData* data)
{
	for (std::map<TrajectoryCost::COST_TYPE, TrajectoryCostPtr>::iterator it =
			costMap_.begin(); it != costMap_.end(); ++it)
	{
		if (it->second->getWeight() != 0.0)
			it->second->compute(data, costDataMap_[it->first],
					costSumMap_[it->first]);
	}
}

double TrajectoryCostAccumulator::getWaypointCost(int waypoint) const
{
	double accumulatedCost = 0.0;
	for (std::map<TrajectoryCost::COST_TYPE, TrajectoryCostPtr>::const_iterator it =
			costMap_.begin(); it != costMap_.end(); ++it)
	{
		const Eigen::VectorXd& costData = costDataMap_.find(it->first)->second;
		accumulatedCost += it->second->getWaypointCost(waypoint, costData)
				* it->second->getWeight();
	}
	return accumulatedCost;
}

double TrajectoryCostAccumulator::getWaypointCost(int waypoint,
		TrajectoryCost::COST_TYPE type) const
{
	std::map<TrajectoryCost::COST_TYPE, TrajectoryCostPtr>::const_iterator it =
			costMap_.find(type);
	const Eigen::VectorXd& costData = costDataMap_.find(type)->second;
	return it->second->getWaypointCost(waypoint, costData)
			* it->second->getWeight();
}

double TrajectoryCostAccumulator::getTrajectoryCost(
		TrajectoryCost::COST_TYPE type) const
{
	std::map<TrajectoryCost::COST_TYPE, TrajectoryCostPtr>::const_iterator it =
			costMap_.find(type);
	if (it != costMap_.end())
	{
		double costSum = costSumMap_.find(it->first)->second;
		return costSum * it->second->getWeight();
	}
	return 0.0;
}

double TrajectoryCostAccumulator::getTrajectoryCost() const
{
	double accumulatedCost = 0.0;
	for (std::map<TrajectoryCost::COST_TYPE, TrajectoryCostPtr>::const_iterator it =
			costMap_.begin(); it != costMap_.end(); ++it)
	{
		double costSum = costSumMap_.find(it->first)->second;
		accumulatedCost += costSum * it->second->getWeight();
	}
	return accumulatedCost;
}

void TrajectoryCostAccumulator::print(int number) const
{
	double cost = getTrajectoryCost();
	if (cost < best_cost_)
	{
		best_cost_ = cost;
		printf("[%d] Trajectory cost : %f/%f (s=%f, c=%f, ci=%f pv=%f ct=%f)\n",
				number, cost, best_cost_,
				getTrajectoryCost(TrajectoryCost::COST_SMOOTHNESS),
				getTrajectoryCost(TrajectoryCost::COST_COLLISION),
				getTrajectoryCost(TrajectoryCost::COST_CONTACT_INVARIANT),
				getTrajectoryCost(TrajectoryCost::COST_PHYSICS_VIOLATION),
				getTrajectoryCost(TrajectoryCost::COST_CARTESIAN_TRAJECTORY));
	}
}

bool TrajectoryCostAccumulator::isFeasible() const
{
	//if (getTrajectoryCost(TrajectoryCost::COST_CONTACT_INVARIANT) > 1E-7)
	//return false;

	//if (getTrajectoryCost() - getTrajectoryCost(TrajectoryCost::COST_SMOOTHNESS) < 0.01)
	//return true;

	if (!is_last_trajectory_valid_)
		return false;

	if (getTrajectoryCost(TrajectoryCost::COST_COLLISION) < 1E-7)
		return true;

	return false;
}

}
