#ifndef IMPROVEMENT_MANAGER_H_
#define IMPROVEMENT_MANAGER_H_

#include <itomp_cio_planner/common.h>
#include <itomp_cio_planner/optimization/evaluation_manager.h>

namespace itomp_cio_planner
{

class ImprovementManager
{
public:
  ImprovementManager();
  virtual ~ImprovementManager();

  virtual void initialize(EvaluationManager *evaluation_manager);
  virtual bool updatePlanningParameters();
  virtual void runSingleIteration(int iteration) = 0;

protected:
  EvaluationManager *evaluation_manager_;
  int last_planning_parameter_index_;
};

typedef boost::shared_ptr<ImprovementManager> ImprovementManagerPtr;
}
;

#endif
