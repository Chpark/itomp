#ifndef TRAJECTORY_COST_BUILDER_H_
#define TRAJECTORY_COST_BUILDER_H_

#include <itomp_cio_planner/common.h>
#include <itomp_cio_planner/util/singleton.h>
#include <itomp_cio_planner/cost/trajectory_cost.h>

namespace itomp_cio_planner
{
class TrajectoryCostManager: public Singleton<TrajectoryCostManager>
{
public:
	TrajectoryCostManager();
	virtual ~TrajectoryCostManager();

	void buildActiveCostFunctions();

	const std::vector<TrajectoryCostConstPtr>& getCostFunctionVector() const;
	int getNumActiveCostFunctions();

protected:
	std::vector<TrajectoryCostConstPtr> cost_function_vector_;
};

inline const std::vector<TrajectoryCostConstPtr>& TrajectoryCostManager::getCostFunctionVector() const
{
	return cost_function_vector_;
}

inline int TrajectoryCostManager::getNumActiveCostFunctions()
{
	if (cost_function_vector_.size() == 0)
		buildActiveCostFunctions();

	return cost_function_vector_.size();
}

}

#endif /* TRAJECTORY_COST_BUILDER_H_ */
