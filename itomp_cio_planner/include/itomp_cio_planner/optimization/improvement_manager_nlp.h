#ifndef IMPROVEMENT_MANAGER_NLP_H_
#define IMPROVEMENT_MANAGER_NLP_H_

#include <itomp_cio_planner/optimization/improvement_manager.h>
#include <itomp_cio_planner/common.h>
#include <itomp_cio_planner/optimization/new_eval_manager.h>
#include "dlib/optimization.h"

namespace itomp_cio_planner
{

typedef dlib::matrix<double, 0, 1> column_vector;

class ImprovementManagerNLP: public ImprovementManager
{
public:
	ImprovementManagerNLP();
	virtual ~ImprovementManagerNLP();

	virtual void initialize(const NewEvalManagerPtr& evaluation_manager, const ItompPlanningGroupConstPtr& planning_group);
	virtual bool updatePlanningParameters();
	virtual void runSingleIteration(int iteration);

protected:
	void addNoiseToVariables(column_vector& variables);

	double evaluate(const column_vector& variables);
	column_vector derivative(const column_vector& variables);
	column_vector derivative_ref(const column_vector& variables);

	void optimize(int iteration, column_vector& variables);

    void computeEvaluationOrder(long variable_size);

	int num_threads_;
	std::vector<NewEvalManagerPtr> derivatives_evaluation_manager_;

	std::vector<Eigen::MatrixXd> evaluation_cost_matrices_;

	double best_cost_;
    column_vector best_param_;

	double eps_;

	ros::Time start_time_;
	int evaluation_count_;

    std::vector<long> evaluation_order_;
};

}
;

#endif
