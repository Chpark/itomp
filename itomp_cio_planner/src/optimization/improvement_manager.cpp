#include <itomp_cio_planner/optimization/improvement_manager.h>
#include <itomp_cio_planner/util/planning_parameters.h>

namespace itomp_cio_planner
{

ImprovementManager::ImprovementManager() :
	last_planning_parameter_index_(-1)
{

}

ImprovementManager::~ImprovementManager()
{

}

void ImprovementManager::initialize(const NewEvalManagerPtr& evaluation_manager,
									const ItompPlanningGroupConstPtr& planning_group)
{
	evaluation_manager_ = evaluation_manager;
	planning_group_ = planning_group;
}

bool ImprovementManager::updatePlanningParameters()
{
	if (last_planning_parameter_index_
			== PlanningParameters::getInstance()->getUpdateIndex())
		return false;
	last_planning_parameter_index_ =
		PlanningParameters::getInstance()->getUpdateIndex();
	return true;
}

}
