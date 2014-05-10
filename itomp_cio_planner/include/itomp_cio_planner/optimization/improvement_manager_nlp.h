#ifndef IMPROVEMENT_MANAGER_NLP_H_
#define IMPROVEMENT_MANAGER_NLP_H_

#include <itomp_cio_planner/optimization/improvement_manager.h>
#include <itomp_cio_planner/common.h>
#include <itomp_cio_planner/optimization/evaluation_manager.h>
#include "dlib/optimization.h"

namespace itomp_cio_planner
{

typedef dlib::matrix<double, 0, 1> column_vector;

class ImprovementManagerNLP: public ImprovementManager
{
public:
  ImprovementManagerNLP();
  virtual ~ImprovementManagerNLP();

  virtual void initialize(EvaluationManager *evaluation_manager);
  virtual bool updatePlanningParameters();
  virtual void runSingleIteration(int iteration);

protected:
  void setVariableVector();
  void getVariableVector();
  void addNoiseToVariables();

  double evaluate(const column_vector& variables);
  column_vector derivative(const column_vector& variables);

  void optimize(int iteration);

  int num_dimensions_;
  int num_contact_dimensions_;
  int num_points_;
  int num_contact_phases_;

  int num_contact_vars_free_;
  int num_pos_variables_;
  int num_vel_variables_;
  int num_variables_;

  Eigen::MatrixXd parameters_;
  Eigen::MatrixXd vel_parameters_;
  Eigen::MatrixXd contact_parameters_;
  Eigen::VectorXd costs_;

  column_vector variables_;

};

}
;

#endif
