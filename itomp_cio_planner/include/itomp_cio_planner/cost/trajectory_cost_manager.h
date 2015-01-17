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

	void buildActiveCostFunctions(const NewEvalManager* evaluation_manager);

	std::vector<TrajectoryCostPtr>& getCostFunctionVector();
	int getNumActiveCostFunctions();

protected:
	std::vector<TrajectoryCostPtr> cost_function_vector_;
};

inline std::vector<TrajectoryCostPtr>& TrajectoryCostManager::getCostFunctionVector()
{
	return cost_function_vector_;
}

inline int TrajectoryCostManager::getNumActiveCostFunctions()
{
	return cost_function_vector_.size();
}

}

#endif /* TRAJECTORY_COST_BUILDER_H_ */
