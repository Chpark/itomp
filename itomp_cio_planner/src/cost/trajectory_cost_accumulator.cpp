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

TrajectoryCostAccumulator::TrajectoryCostAccumulator()
{

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
  for (std::map<TrajectoryCost::COST_TYPE, TrajectoryCostPtr>::iterator it = costMap_.begin(); it != costMap_.end(); ++it)
  {
    it->second->init(data);
    costDataMap_[it->first] = Eigen::VectorXd::Zero(data->getNumPoints());
    costSumMap_[it->first] = 0.0;
  }
}

void TrajectoryCostAccumulator::compute(const EvaluationData* data)
{
  for (std::map<TrajectoryCost::COST_TYPE, TrajectoryCostPtr>::iterator it = costMap_.begin(); it != costMap_.end(); ++it)
  {
    if (it->second->getWeight() != 0.0)
      it->second->compute(data, costDataMap_[it->first], costSumMap_[it->first]);
  }
}

double TrajectoryCostAccumulator::getWaypointCost(int waypoint) const
{
  double accumulatedCost = 0.0;
  for (std::map<TrajectoryCost::COST_TYPE, TrajectoryCostPtr>::const_iterator it = costMap_.begin(); it != costMap_.end();
      ++it)
  {
    const Eigen::VectorXd& costData = costDataMap_.find(it->first)->second;
    accumulatedCost += it->second->getWaypointCost(waypoint, costData) * it->second->getWeight();
  }
  return accumulatedCost;
}

double TrajectoryCostAccumulator::getWaypointCost(int waypoint, TrajectoryCost::COST_TYPE type) const
{
  std::map<TrajectoryCost::COST_TYPE, TrajectoryCostPtr>::const_iterator it = costMap_.find(type);
  const Eigen::VectorXd& costData = costDataMap_.find(type)->second;
  return it->second->getWaypointCost(waypoint, costData) * it->second->getWeight();
}

double TrajectoryCostAccumulator::getTrajectoryCost(TrajectoryCost::COST_TYPE type) const
{
  std::map<TrajectoryCost::COST_TYPE, TrajectoryCostPtr>::const_iterator it = costMap_.find(type);
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
  for (std::map<TrajectoryCost::COST_TYPE, TrajectoryCostPtr>::const_iterator it = costMap_.begin(); it != costMap_.end();
      ++it)
  {
    double costSum = costSumMap_.find(it->first)->second;
    accumulatedCost += costSum * it->second->getWeight();
  }
  return accumulatedCost;
}

void TrajectoryCostAccumulator::print(int number) const
{
  printf("[%d] Trajectory cost : %f (s=%f, c=%f, v=%f, ci=%f pv=%f gp=%f com=%f)\n", number, getTrajectoryCost(),
      getTrajectoryCost(TrajectoryCost::COST_SMOOTHNESS), getTrajectoryCost(TrajectoryCost::COST_COLLISION),
      getTrajectoryCost(TrajectoryCost::COST_VALIDITY), getTrajectoryCost(TrajectoryCost::COST_CONTACT_INVARIANT),
      getTrajectoryCost(TrajectoryCost::COST_PHYSICS_VIOLATION), getTrajectoryCost(TrajectoryCost::COST_GOAL_POSE),
      getTrajectoryCost(TrajectoryCost::COST_COM));

}

bool TrajectoryCostAccumulator::isFeasible() const
{
  //if (getTrajectoryCost(TrajectoryCost::COST_CONTACT_INVARIANT) > 1E-7)
  //return false;

  return true;
}

}
