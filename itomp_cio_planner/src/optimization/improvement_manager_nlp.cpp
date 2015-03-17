#include <itomp_cio_planner/optimization/improvement_manager_nlp.h>
#include <itomp_cio_planner/cost/trajectory_cost_manager.h>
#include <itomp_cio_planner/util/multivariate_gaussian.h>
#include <itomp_cio_planner/util/planning_parameters.h>
#include <omp.h>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <functional>
#include <itomp_cio_planner/util/jacobian.h>
#include <iostream>

using namespace Eigen;

double getROSWallTime()
{
	return ros::WallTime::now().toSec();
}

namespace itomp_cio_planner
{

const bool READ_TRAJECTORY_FILE = true;
const bool WRITE_TRAJECTORY_FILE = true;

ImprovementManagerNLP::ImprovementManagerNLP()
{
	evaluation_count_ = 0;
    eps_ = ITOMP_EPS;
	best_cost_ = std::numeric_limits<double>::max();
}

ImprovementManagerNLP::~ImprovementManagerNLP()
{
    TrajectoryCostManager::getInstance()->destroy();
    PerformanceProfiler::getInstance()->destroy();

    for (int i = 0; i < derivatives_evaluation_manager_.size(); ++i)
        derivatives_evaluation_manager_[i].reset();
}

void ImprovementManagerNLP::initialize(const NewEvalManagerPtr& evaluation_manager,
                                       const ItompPlanningGroupConstPtr& planning_group)
{
	start_time_ = ros::Time::now();

	ImprovementManager::initialize(evaluation_manager, planning_group);

    num_threads_ = omp_get_max_threads();

	omp_set_num_threads(num_threads_);
    ROS_INFO("Use %d threads on %d processors", num_threads_, omp_get_num_procs());

	if (num_threads_ < 1)
		ROS_ERROR("0 threads!!!");

	TIME_PROFILER_INIT(getROSWallTime, num_threads_);
	TIME_PROFILER_ADD_ENTRY(FK);

    const ParameterTrajectoryConstPtr& parameter_trajectory = evaluation_manager_->getParameterTrajectory();
	num_parameter_types_ = parameter_trajectory->hasVelocity() ? 2 : 1;
	num_parameter_points_ = parameter_trajectory->getNumPoints();
	num_parameter_elements_ = parameter_trajectory->getNumElements();
	int num_points = evaluation_manager_->getFullTrajectory()->getNumPoints();

    int num_costs =	TrajectoryCostManager::getInstance()->getNumActiveCostFunctions();

	derivatives_evaluation_manager_.resize(num_threads_);
	evaluation_parameters_.resize(num_threads_);
	evaluation_cost_matrices_.resize(num_threads_);
	for (int i = 0; i < num_threads_; ++i)
	{
        derivatives_evaluation_manager_[i].reset(new NewEvalManager(*evaluation_manager));
        evaluation_parameters_[i].resize(Trajectory::TRAJECTORY_TYPE_NUM, Eigen::MatrixXd(num_parameter_points_, num_parameter_elements_));
		evaluation_cost_matrices_[i] = Eigen::MatrixXd(num_points, num_costs);
	}
    best_parameter_.resize(Trajectory::TRAJECTORY_TYPE_NUM, Eigen::MatrixXd(num_parameter_points_, num_parameter_elements_));
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
    best_cost_ = std::numeric_limits<double>::max();

    int num_variables = num_parameter_elements_ * num_parameter_points_ * num_parameter_types_;

	column_vector variables(num_variables);

    evaluation_manager_->getParameters(variables);

    // read from file
    if (READ_TRAJECTORY_FILE)
    {
        std::ifstream trajectory_file;
        std::stringstream ss;
        ss << "trajectory_" << iteration << ".txt";
        trajectory_file.open(ss.str().c_str());
        if (trajectory_file.is_open())
        {
            trajectory_file >> variables;
            trajectory_file.close();

            evaluation_manager_->setParameters(variables);
        }
    }

	//if (iteration != 0)
	//addNoiseToVariables(variables);

    optimize(iteration, variables);

	evaluation_manager_->printTrajectoryCost(iteration);

	printf("Elapsed : %f\n", (ros::Time::now() - start_time_).toSec());

    evaluation_manager_->getParameters(variables);

    // write to file
    if (WRITE_TRAJECTORY_FILE)
    {
        std::ofstream trajectory_file;
        std::stringstream ss;
        ss << "trajectory_" << iteration << ".txt";
        trajectory_file.open(ss.str().c_str());
        trajectory_file.precision(std::numeric_limits<double>::digits10);
        trajectory_file << variables;
        trajectory_file.close();
    }
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
    evaluation_manager_->setParameters(variables);

	double cost = evaluation_manager_->evaluate();

	evaluation_manager_->render();

	evaluation_manager_->printTrajectoryCost(++evaluation_count_, true);
    if (evaluation_count_ % 1000 == 0)
	{
		printf("Elapsed (in eval) : %f\n",
			   (ros::Time::now() - start_time_).toSec());
	}

	if (cost < best_cost_)
	{
		best_cost_ = cost;
		best_parameter_ = evaluation_parameters_[0];
        best_param_ = variables;
	}

	return cost;
}

column_vector ImprovementManagerNLP::derivative_ref(const column_vector& variables)
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
        evaluation_manager_->setParameters(e);
		const double delta_plus = evaluation_manager_->evaluate();

        e(i) = old_val - eps_;
		readFromOptimizationVariables(e, evaluation_parameters_[0]);
		evaluation_manager_->setParameters(evaluation_parameters_[0]);
        evaluation_manager_->setParameters(e);
		double delta_minus = evaluation_manager_->evaluate();

        der(i) = (delta_plus - delta_minus) / (2 * eps_);

		e(i) = old_val;

		delta_plus_vec(i) = delta_plus;
		delta_minus_vec(i) = delta_minus;
	}

	return der;
}

static bool COMPUTE_COST_DERIVATIVE = false;
column_vector ImprovementManagerNLP::derivative(const column_vector& variables)
{
	// assume evaluate was called before

	TIME_PROFILER_START_ITERATION;

    column_vector der;
    der.set_size(variables.size());

    // for cost debug
    std::vector<column_vector> cost_der(TrajectoryCostManager::getInstance()->getNumActiveCostFunctions());
    for (int i = 0; i < cost_der.size(); ++i)
        cost_der[i].set_size(variables.size());
    std::vector<double*> cost_der_ptr(cost_der.size());
    for (int i = 0; i < cost_der.size(); ++i)
        cost_der_ptr[i] = cost_der[i].begin();

    #pragma omp parallel for
    for (int i = 0; i < num_threads_; ++i)
    {
        derivatives_evaluation_manager_[i]->setParameters(variables);
    }

    #pragma omp parallel for
    for (int i = 0; i < variables.size(); ++i)
    {
        int thread_index = omp_get_thread_num();

        //  for cost debug
        //derivatives_evaluation_manager_[thread_index]->computeDerivatives(i, variables, der.begin(), eps_);
        derivatives_evaluation_manager_[thread_index]->computeCostDerivatives(i, variables, der.begin(), cost_der_ptr, eps_);
    }


    TIME_PROFILER_PRINT_ITERATION_TIME();

    // print derivatives per costs
    if (COMPUTE_COST_DERIVATIVE)
    {
        const std::vector<TrajectoryCostPtr>& cost_functions = TrajectoryCostManager::getInstance()->getCostFunctionVector();
        std::cout.precision(3);
        std::cout << "component sub_component point element ";
        for (int c = 0; c < cost_functions.size(); ++c)
        {
            std::cout << cost_functions[c]->getName() << " ";
        }
        std::cout << std::endl;
        for (int i = 0; i < variables.size(); ++i)
        {
            const ItompTrajectoryIndex& index = evaluation_manager_->getTrajectory()->getTrajectoryIndex(i);
            std::cout << index.component << " " << index.sub_component << " " << index.point << " " << index.element << " ";
            for (int j = 0; j < cost_der.size(); ++j)
                std::cout << cost_der[j](i) << " ";
            std::cout << std::endl;
        }
    }

    // validation with der_ref
    /*
    column_vector der_reference = derivative_ref(variables);
    ROS_INFO("Vaildate computed derivative with reference");
    double max_der = 0.0;
    for (int i = 0; i < variables.size(); ++i)
    {
        double abs_der = std::abs(der(i));
        if (abs_der > max_der)
            max_der = abs_der;
    }
    for (int i = 0; i < variables.size(); ++i)
    {
        if (std::abs(der(i) - der_reference(i)) > max_der * 0.1)
        {
            const ItompTrajectoryIndex& index = evaluation_manager_->getTrajectory()->getTrajectoryIndex(i);

            ROS_INFO("Error at %d(%d %d %d %d) : %.14f (%.14f vs %.14f) max_der : %f", i,
                     index.component, index.sub_component, index.point, index.element,
                     std::abs(der(i) - der_reference(i)),
                     der(i), der_reference(i), max_der);
        }
    }
    */

    return der;
}

void ImprovementManagerNLP::optimize(int iteration, column_vector& variables)
{
	//addNoiseToVariables(variables);

	Jacobian::evaluation_manager_ = evaluation_manager_.get();

    std::vector<double> group_joint_min(planning_group_->group_joints_.size());
    std::vector<double> group_joint_max(planning_group_->group_joints_.size());
    for (int j = 0; j < planning_group_->group_joints_.size(); ++j)
    {
        const ItompRobotJoint& joint = planning_group_->group_joints_[j];
        int group_index = joint.group_joint_index_;
        group_joint_min[group_index] = joint.joint_limit_min_;
        group_joint_max[group_index] = joint.joint_limit_max_;
    }

    column_vector x_lower, x_upper;
    x_lower.set_size(variables.size());
    x_upper.set_size(variables.size());
    for (int i = 0; i < variables.size(); ++i)
    {
        ItompTrajectoryIndex index = evaluation_manager_->getTrajectory()->getTrajectoryIndex(i);

        x_lower(i) = -30.0;
        x_upper(i) = 30.0;


        if (index.component == ItompTrajectory::COMPONENT_TYPE_POSITION)
        {
            switch (index.sub_component)
            {
            case ItompTrajectory::SUB_COMPONENT_TYPE_JOINT:
            {
                int parameter_joint_index = evaluation_manager_->getTrajectory()->getParameterJointIndex(index.element);
                if (parameter_joint_index != -1)
                {
                    x_lower(i) = group_joint_min[parameter_joint_index];
                    x_upper(i) = group_joint_max[parameter_joint_index];
                }
                /*
                if (parameter_joint_index == 3 || parameter_joint_index == 4)
                {
                    x_lower(i) = -0.001;
                    x_upper(i) = 0.001;
                }
                */
            }
            break;

            case ItompTrajectory::SUB_COMPONENT_TYPE_CONTACT_POSITION:
                switch(index.element % 7)
                {
                case 0:
                    break;

                case 1:
                    x_lower(i) = group_joint_min[0];
                    x_upper(i) = group_joint_max[0];
                    break;

                case 2:
                    x_lower(i) = group_joint_min[1];
                    x_upper(i) = group_joint_max[1];
                    break;

                case 3:
                    x_lower(i) = group_joint_min[2];
                    x_upper(i) = group_joint_max[2];
                    break;

                case 4:
                case 5:
                case 6:
                    x_lower(i) = -2.0 * M_PI;
                    x_upper(i) = 2.0 * M_PI;
                    break;
                }
                break;

            case ItompTrajectory::SUB_COMPONENT_TYPE_CONTACT_FORCE:
                x_lower(i) = -1.0;
                x_upper(i) = 1.0;

                break;

            }
        }
    }


    dlib::find_min_box_constrained(dlib::lbfgs_search_strategy(10),
                                   dlib::objective_delta_stop_strategy(eps_,
                                           PlanningParameters::getInstance()->getMaxIterations()).be_verbose(),
                                   boost::bind(&ImprovementManagerNLP::evaluate, this, _1),
                                   boost::bind(&ImprovementManagerNLP::derivative, this, _1),
                                   variables, x_lower, x_upper);


    /*
        dlib::find_min(dlib::lbfgs_search_strategy(10),
                       dlib::objective_delta_stop_strategy(eps_,
                               PlanningParameters::getInstance()->getMaxIterations()).be_verbose(),
                       boost::bind(&ImprovementManagerNLP::evaluate, this, _1),
                       boost::bind(&ImprovementManagerNLP::derivative, this, _1),
                       variables, 0.0);
                       */

	evaluation_manager_->setParameters(best_parameter_);
    evaluation_manager_->setParameters(variables);
	evaluation_manager_->evaluate();
	evaluation_manager_->printTrajectoryCost(0, true);
	evaluation_manager_->render();

    COMPUTE_COST_DERIVATIVE = true;
    derivative(best_param_);
    COMPUTE_COST_DERIVATIVE = false;
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
        variables(i) += ITOMP_EPS * noise(i);
	}
}

}
