/*
 * best_cost_manager.h
 *
 *  Created on: Sep 23, 2014
 *      Author: chonhyon
 */

#ifndef BEST_COST_MANAGER_H_
#define BEST_COST_MANAGER_H_

#include <itomp_cio_planner/common.h>

namespace itomp_cio_planner
{
class BestCostManager
{
public:
	BestCostManager();
	~BestCostManager();

	void reset();

	bool updateBestCost(int trajectory_index, double cost, bool feasible);
	int getBestCostTrajectoryIndex() const;

	bool isSolutionFound() const;

protected:
	boost::mutex mtx_;

	double best_cost;
	bool is_feasible_;

	unsigned int best_trajectory_index_;
};

inline BestCostManager::BestCostManager() :
		best_cost(std::numeric_limits<double>::max()), best_trajectory_index_(
				0), is_feasible_(false)
{

}

inline BestCostManager::~BestCostManager()
{

}

inline void BestCostManager::reset()
{
	best_cost = std::numeric_limits<double>::max();
	best_trajectory_index_ = 0;
	is_feasible_ = false;
}

inline bool BestCostManager::updateBestCost(int trajectory_index, double cost,
		bool feasible)
{
	if (!feasible && is_feasible_)
		return false;

	if (cost < best_cost || (feasible && !is_feasible_))
	{
		boost::lock_guard<boost::mutex> guard(mtx_);
		if (cost < best_cost || (feasible && !is_feasible_))
		{
			best_cost = cost;
			best_trajectory_index_ = trajectory_index;
			is_feasible_ = feasible;
		}
	}
	return trajectory_index == best_trajectory_index_;
}

inline int BestCostManager::getBestCostTrajectoryIndex() const
{
	return best_trajectory_index_;
}

inline bool BestCostManager::isSolutionFound() const
{
	return is_feasible_;
}

}

#endif /* BEST_COST_MANAGER_H_ */
