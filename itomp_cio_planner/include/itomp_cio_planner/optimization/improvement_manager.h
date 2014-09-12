#ifndef IMPROVEMENT_MANAGER_H_
#define IMPROVEMENT_MANAGER_H_

#include <itomp_cio_planner/common.h>
#include <itomp_cio_planner/optimization/new_eval_manager.h>

namespace itomp_cio_planner
{

class ImprovementManager
{
public:
	ImprovementManager();
	virtual ~ImprovementManager();

	virtual void initialize(const NewEvalManagerPtr& evaluation_manager,
			const ItompPlanningGroupConstPtr& planning_group);
	virtual bool updatePlanningParameters();
	virtual void runSingleIteration(int iteration) = 0;

protected:
	NewEvalManagerPtr evaluation_manager_;
	ItompPlanningGroupConstPtr planning_group_;

	int last_planning_parameter_index_;
};
ITOMP_DEFINE_SHARED_POINTERS(ImprovementManager);

}
;

#endif
