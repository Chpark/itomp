/*
 * TrajectoryCostAccumulator.h
 *
 *  Created on: Oct 23, 2013
 *      Author: cheonhyeonpark
 */

#ifndef TRAJECTORYCOSTACCUMULATOR_H_
#define TRAJECTORYCOSTACCUMULATOR_H_

#include <itomp_cio_planner/common.h>
#include <itomp_cio_planner/cost/trajectory_cost.h>

namespace itomp_cio_planner
{
class EvaluationManager;
class TrajectoryCostAccumulator
{
public:
	TrajectoryCostAccumulator();
	virtual ~TrajectoryCostAccumulator();

	void init(const EvaluationManager* evaluator);
	void compute(const EvaluationManager* evaluator);

	void addCost(TrajectoryCostPtr cost);

	double getWaypointCost(int waypoint) const;
	double getWaypointCost(int waypoint, TrajectoryCost::COST_TYPE type) const;
	double getTrajectoryCost(TrajectoryCost::COST_TYPE type) const;
	double getTrajectoryCost() const;

	bool isFeasible() const;

	void print(int number) const;

protected:
	std::map<TrajectoryCost::COST_TYPE, TrajectoryCostPtr> costMap_;
};

}


#endif /* TRAJECTORYCOSTACCUMULATOR_H_ */
