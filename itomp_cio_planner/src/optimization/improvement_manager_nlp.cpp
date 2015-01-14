#include <itomp_cio_planner/optimization/improvement_manager_nlp.h>
#include <itomp_cio_planner/cost/trajectory_cost_manager.h>
#include <itomp_cio_planner/util/multivariate_gaussian.h>
#include <itomp_cio_planner/util/planning_parameters.h>
#include <omp.h>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <functional>
#include <itomp_cio_planner/util/jacobian.h>

using namespace Eigen;

double getROSWallTime()
{
	return ros::WallTime::now().toSec();
}

namespace itomp_cio_planner
{

ImprovementManagerNLP::ImprovementManagerNLP()
{
	evaluation_count_ = 0;
	eps_ = 1e-7;
	best_cost_ = std::numeric_limits<double>::max();
}

ImprovementManagerNLP::~ImprovementManagerNLP()
{

}

void ImprovementManagerNLP::initialize(
		const NewEvalManagerPtr& evaluation_manager,
		const ItompPlanningGroupConstPtr& planning_group)
{
	start_time_ = ros::Time::now();

	ImprovementManager::initialize(evaluation_manager, planning_group);

	num_threads_ = omp_get_max_threads();
	// TODO: change num_threads_

	omp_set_num_threads(num_threads_);
	ROS_INFO("Use %d threads", num_threads_);

	if (num_threads_ < 1)
		ROS_ERROR("0 threads!!!");

	TIME_PROFILER_INIT(getROSWallTime, num_threads_);TIME_PROFILER_ADD_ENTRY(FK);

	const ParameterTrajectoryConstPtr& parameter_trajectory =
			evaluation_manager_->getParameterTrajectory();
	num_parameter_types_ = parameter_trajectory->hasVelocity() ? 2 : 1;
	num_parameter_points_ = parameter_trajectory->getNumPoints();
	num_parameter_elements_ = parameter_trajectory->getNumElements();
	int num_points = evaluation_manager_->getFullTrajectory()->getNumPoints();

	int num_costs =
			TrajectoryCostManager::getInstance()->getNumActiveCostFunctions();

	derivatives_evaluation_manager_.resize(num_threads_);
	evaluation_parameters_.resize(num_threads_);
	evaluation_cost_matrices_.resize(num_threads_);
	for (int i = 0; i < num_threads_; ++i)
	{
		derivatives_evaluation_manager_[i].reset(
				evaluation_manager->createClone());

		evaluation_parameters_[i].resize(Trajectory::TRAJECTORY_TYPE_NUM,
				Eigen::MatrixXd(num_parameter_points_,
						num_parameter_elements_));
		evaluation_cost_matrices_[i] = Eigen::MatrixXd(num_points, num_costs);
	}
	best_parameter_.resize(Trajectory::TRAJECTORY_TYPE_NUM,
			Eigen::MatrixXd(num_parameter_points_, num_parameter_elements_));
}

bool ImprovementManagerNLP::updatePlanningParameters()
{
	if (!ImprovementManager::updatePlanningParameters())
		return false;

	TrajectoryCostManager::getInstance()->buildActiveCostFunctions(evaluation_manager_.get());

	return true;
}

void ImprovementManagerNLP::runSingleIteration(int iteration)
{
	int num_variables = num_parameter_elements_ * num_parameter_points_
			* num_parameter_types_;

	column_vector variables(num_variables);

	evaluation_manager_->getParameters(evaluation_parameters_[0]);
	writeToOptimizationVariables(variables, evaluation_parameters_[0]);

	//if (iteration != 0)
	//addNoiseToVariables(variables);

	optimize(iteration, variables);

	evaluation_manager_->printTrajectoryCost(iteration);

	printf("Elapsed : %f\n", (ros::Time::now() - start_time_).toSec());
}

void ImprovementManagerNLP::writeToOptimizationVariables(
		column_vector& variables,
		const std::vector<Eigen::MatrixXd>& evaluation_parameter)
{
	int write_index = 0;

	for (int k = 0; k < num_parameter_types_; ++k)
	{
		for (int j = 0; j < num_parameter_points_; ++j)
		{
			for (int i = 0; i < num_parameter_elements_; ++i)
			{
				variables(write_index++, 0) = evaluation_parameter[k](j, i);
			}
		}
	}
}

void ImprovementManagerNLP::readFromOptimizationVariables(
		const column_vector& variables,
		std::vector<Eigen::MatrixXd>& evaluation_parameter)
{
	int read_index = 0;

	double value;
	for (int k = 0; k < num_parameter_types_; ++k)
	{
		for (int j = 0; j < num_parameter_points_; ++j)
		{
			for (int i = 0; i < num_parameter_elements_; ++i)
			{
				evaluation_parameter[k](j, i) = variables(read_index++, 0);
			}
		}
	}
}

double ImprovementManagerNLP::evaluate(const column_vector& variables)
{
	readFromOptimizationVariables(variables, evaluation_parameters_[0]);
	evaluation_manager_->setParameters(evaluation_parameters_[0]);

	double cost = evaluation_manager_->evaluate();

	evaluation_manager_->render();

	evaluation_manager_->printTrajectoryCost(++evaluation_count_, true);
	if (evaluation_count_ % 100 == 0)
	{
		printf("Elapsed (in eval) : %f\n",
				(ros::Time::now() - start_time_).toSec());
	}

	if (cost < best_cost_)
	{
		best_cost_ = cost;
		best_parameter_ = evaluation_parameters_[0];
	}

	return cost;
}

column_vector ImprovementManagerNLP::derivative_ref(
		const column_vector& variables)
{
	column_vector der(variables.size());
	column_vector e = variables;

	column_vector delta_plus_vec(variables.size());
	column_vector delta_minus_vec(variables.size());

	for (long i = 0; i < variables.size(); ++i)
	{
		const double old_val = e(i);

		e(i) += eps_;
		readFromOptimizationVariables(e, evaluation_parameters_[0]);
		evaluation_manager_->setParameters(evaluation_parameters_[0]);
		const double delta_plus = evaluation_manager_->evaluate();

		e(i) = old_val - eps_;
		readFromOptimizationVariables(e, evaluation_parameters_[0]);
		evaluation_manager_->setParameters(evaluation_parameters_[0]);
		double delta_minus = evaluation_manager_->evaluate();

		der(i) = (delta_plus - delta_minus) / (2 * eps_);

		e(i) = old_val;

		delta_plus_vec(i) = delta_plus;
		delta_minus_vec(i) = delta_minus;
	}

	printf("Der_ref : ");
	for (long i = 0; i < variables.size(); ++i)
		printf("%f ", der(i));
	printf("\n");
	printf("Der_ref_p : ");
	for (long i = 0; i < variables.size(); ++i)
		printf("%.14f ", delta_plus_vec(i));
	printf("\n");
	printf("Der_ref_m : ");
	for (long i = 0; i < variables.size(); ++i)
		printf("%.14f ", delta_minus_vec(i));
	printf("\n");

	return der;
}

column_vector ImprovementManagerNLP::derivative(const column_vector& variables)
{
	column_vector delta_plus_vec(variables.size());
	column_vector delta_minus_vec(variables.size());
	double d_p, d_m;

	// assume evaluate was called before

	TIME_PROFILER_START_ITERATION;

	int num_cost_functions =
			TrajectoryCostManager::getInstance()->getNumActiveCostFunctions();

	std::vector<column_vector> der(num_cost_functions + 1);
	for (int i = 0; i < 1/*der.size()*/; ++i)
		der[i].set_size(variables.size());

	readFromOptimizationVariables(variables, evaluation_parameters_[0]);
	for (int i = 1; i < num_threads_; ++i)
		evaluation_parameters_[i] = evaluation_parameters_[0];

	std::vector<std::vector<double> > cost_der(num_cost_functions, std::vector<double>(num_parameter_elements_));

	int parameter_index = 0;
	for (int k = 0; k < num_parameter_types_; ++k)
	{
#pragma omp parallel for
		for (int j = 0; j < num_parameter_points_; ++j)
		{
			int thread_index = omp_get_thread_num();
			int thread_variable_index = parameter_index
					+ j * num_parameter_elements_;

			derivatives_evaluation_manager_[thread_index]->computeDerivatives(
					evaluation_parameters_[thread_index], k, j,
					der[0].begin() + thread_variable_index, eps_,
					delta_plus_vec.begin() + thread_variable_index,
					delta_minus_vec.begin() + thread_variable_index, NULL);//&cost_der);
#ifdef VALIDATE_DERIVATIVE
			for (int i = 0; i < num_cost_functions; ++i)
			{
				for (int e = 0; e < num_parameter_elements_; ++e)
					*(der[i + 1].begin() + thread_variable_index + e) = cost_der[i][e];
			}
#endif
		}
		parameter_index += num_parameter_points_ * num_parameter_elements_;
	}

	TIME_PROFILER_PRINT_ITERATION_TIME();

//#define VALIDATE_DERIVATIVE
#ifdef VALIDATE_DERIVATIVE
	// validate
	printf("[%d] Der : \n", evaluation_count_);
	for (long i = 0; i < variables.size(); ++i)
	{
		printf("%d %f ", i, variables(i));
		for (int c = 0; c <= num_cost_functions; ++c)
			printf("%f ", der[c](i));
		printf("\n");
	}
	printf("\n");
#endif

	return der[0];
}

void ImprovementManagerNLP::optimize(int iteration, column_vector& variables)
{
	//addNoiseToVariables(variables);

	Jacobian::evaluation_manager_ = evaluation_manager_.get();

	dlib::find_min(dlib::lbfgs_search_strategy(10),
			dlib::objective_delta_stop_strategy(eps_,
					PlanningParameters::getInstance()->getMaxIterations()).be_verbose(),
			boost::bind(&ImprovementManagerNLP::evaluate, this, _1),
			boost::bind(&ImprovementManagerNLP::derivative, this, _1),
			variables, 0.0);

	evaluation_manager_->setParameters(best_parameter_);
	evaluation_manager_->evaluate();
	evaluation_manager_->printTrajectoryCost(0, true);
	evaluation_manager_->render();
}

void ImprovementManagerNLP::addNoiseToVariables(column_vector& variables)
{
	int num_variables = variables.size();
	MultivariateGaussian noise_generator(VectorXd::Zero(num_variables),
			MatrixXd::Identity(num_variables, num_variables));
	VectorXd noise = VectorXd::Zero(num_variables);
	noise_generator.sample(noise);
	for (int i = 0; i < num_variables; ++i)
	{
		variables(i) += 1e-2 * noise(i);
	}
}

}
