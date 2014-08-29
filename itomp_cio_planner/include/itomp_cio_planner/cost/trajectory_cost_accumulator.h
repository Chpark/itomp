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
class EvaluationData;
class TrajectoryCostAccumulator
{
public:
	TrajectoryCostAccumulator();
	virtual ~TrajectoryCostAccumulator();

	void init(const EvaluationData* data);
	void compute(const EvaluationData* data);

	void addCost(TrajectoryCostPtr cost);

	double getWaypointCost(int waypoint) const;
	double getWaypointCost(int waypoint, TrajectoryCost::COST_TYPE type) const;
	double getTrajectoryCost(TrajectoryCost::COST_TYPE type) const;
	double getTrajectoryCost() const;

	bool isFeasible() const;

	void print(int number) const;

protected:
	std::map<TrajectoryCost::COST_TYPE, TrajectoryCostPtr> costMap_;
	std::map<TrajectoryCost::COST_TYPE, Eigen::VectorXd> costDataMap_;
	std::map<TrajectoryCost::COST_TYPE, double> costSumMap_;

	mutable double best_cost_;
};

}


#endif /* TRAJECTORYCOSTACCUMULATOR_H_ */
