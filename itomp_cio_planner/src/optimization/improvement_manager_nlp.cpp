#include <itomp_cio_planner/optimization/improvement_manager_nlp.h>
#include <itomp_cio_planner/cost/trajectory_cost_manager.h>
#include <omp.h>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <functional>

using namespace Eigen;

namespace itomp_cio_planner
{

ImprovementManagerNLP::ImprovementManagerNLP()
{
	evaluation_count_ = 0;
	eps_ = 1e-7;
}

ImprovementManagerNLP::~ImprovementManagerNLP()
{

}

void ImprovementManagerNLP::initialize(
		const NewEvalManagerPtr& evaluation_manager)
{
	start_time_ = ros::Time::now();

	ImprovementManager::initialize(evaluation_manager);

	num_threads_ = omp_get_max_threads();
	// TODO: change num_threads_
	num_threads_ = 1;
	omp_set_num_threads(num_threads_);
	if (num_threads_ < 1)
		ROS_ERROR("0 threads!!!");

	const ParameterTrajectoryConstPtr& parameter_trajectory =
			evaluation_manager_->getParameterTrajectory();
	num_parameter_types_ = parameter_trajectory->hasVelocity() ? 2 : 1;
	num_parameter_points_ = parameter_trajectory->getNumPoints();
	num_parameter_elements_ = parameter_trajectory->getNumElements();

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
		evaluation_cost_matrices_[i] = Eigen::MatrixXd(num_parameter_points_,
				num_costs);
	}
}

bool ImprovementManagerNLP::updatePlanningParameters()
{
	if (!ImprovementManager::updatePlanningParameters())
		return false;

	TrajectoryCostManager::getInstance()->buildActiveCostFunctions();

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

void ImprovementManagerNLP::addNoiseToVariables(column_vector& variables)
{
	/*
	 MultivariateGaussian noise_generator(VectorXd::Zero (num_variables_),
	 MatrixXd::Identity(num_variables_, num_variables_));
	 VectorXd noise = VectorXd::Zero(num_variables_);
	 noise_generator.sample(noise);
	 for (int i = 0; i < num_variables_; ++i)
	 {
	 variables(i) += 0.001 * noise(i);
	 }
	 */
}

double ImprovementManagerNLP::evaluate(const column_vector& variables)
{
	readFromOptimizationVariables(variables, evaluation_parameters_[0]);
	evaluation_manager_->setParameters(evaluation_parameters_[0]);

	double cost = evaluation_manager_->evaluate();

	if (++evaluation_count_ % 100 == 0)
	{
		evaluation_manager_->printTrajectoryCost(evaluation_count_);

		printf("Elapsed : %f\n", (ros::Time::now() - start_time_).toSec());
	}

	return cost;
}

column_vector ImprovementManagerNLP::derivative_ref(
		const column_vector& variables)
{
	column_vector der(variables.size());
	column_vector e = variables;

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
	}

	return der;
}

column_vector ImprovementManagerNLP::derivative(const column_vector& variables)
{
	// assume evaluate was called before

	column_vector der(variables.size());
	readFromOptimizationVariables(variables, evaluation_parameters_[0]);
	for (int i = 1; i < num_threads_; ++i)
		evaluation_parameters_[i] = evaluation_parameters_[0];

	int parameter_index = 0;
	for (int k = 0; k < num_parameter_types_; ++k)
	{
#pragma omp parallel for
		for (int j = 0; j < num_parameter_points_; ++j)
		{
			int thread_index = omp_get_thread_num();
			int thread_variable_index = parameter_index
					+ j * num_parameter_elements_;
			for (int i = 0; i < num_parameter_elements_; ++i)
			{
				const double old_val = evaluation_parameters_[thread_index][k](
						j, i);
				int begin, end;

				evaluation_parameters_[thread_index][k](j, i) += eps_;
				derivatives_evaluation_manager_[thread_index]->evaluateParameterPoint(
						j, i, evaluation_cost_matrices_[thread_index], begin,
						end);
				const double delta_plus =
						evaluation_cost_matrices_[thread_index].block(begin, 1,
								end - begin, num_parameter_elements_).sum();

				evaluation_parameters_[thread_index][k](j, i) = old_val - eps_;
				derivatives_evaluation_manager_[thread_index]->evaluateParameterPoint(
						j, i, evaluation_cost_matrices_[thread_index], begin,
						end);
				const double delta_minus =
						evaluation_cost_matrices_[thread_index].block(begin, 1,
								end - begin, num_parameter_elements_).sum();

				evaluation_parameters_[thread_index][k](j, i) = old_val;

				der(thread_variable_index++) = (delta_plus - delta_minus)
						/ (2 * eps_);
			}
		}
		parameter_index += num_parameter_points_ * num_parameter_elements_;
	}

	return der;
}

void ImprovementManagerNLP::optimize(int iteration, column_vector& variables)
{
	dlib::find_min(dlib::lbfgs_search_strategy(10),
			dlib::objective_delta_stop_strategy(eps_).be_verbose(),
			boost::bind(&ImprovementManagerNLP::evaluate, this, _1),
			boost::bind(&ImprovementManagerNLP::derivative, this, _1),
			variables, 0.0);
}

}
