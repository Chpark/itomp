#ifndef IMPROVEMENT_MANAGER_NLP_H_
#define IMPROVEMENT_MANAGER_NLP_H_

#include <itomp_cio_planner/optimization/improvement_manager.h>
#include <itomp_cio_planner/common.h>
#include <itomp_cio_planner/optimization/evaluation_manager.h>

namespace itomp_cio_planner
{

class ImprovementManagerNLP : public ImprovementManager
{
public:
  ImprovementManagerNLP();
  virtual ~ImprovementManagerNLP();

  virtual void initialize(EvaluationManager *evaluation_manager);
  virtual bool updatePlanningParameters();
  virtual void runSingleIteration(int iteration);

protected:
  void optimize(bool add_noise, int iteration);

  int num_dimensions_;
  int num_contact_dimensions_;
  int num_points_;
  int num_contact_phases_;

};

}
;

#endif
