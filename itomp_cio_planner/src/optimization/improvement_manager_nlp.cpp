#include <itomp_cio_planner/optimization/improvement_manager_nlp.h>
#include <itomp_cio_planner/optimization/phase_manager.h>
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

const bool READ_TRAJECTORY_FILE = false;
const bool WRITE_TRAJECTORY_FILE = false;

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
    if (PlanningParameters::getInstance()->getPrintPlanningInfo())
        ROS_INFO("Use %d threads on %d processors", num_threads_, omp_get_num_procs());

    if (num_threads_ < 1)
        ROS_ERROR("0 threads!!!");

    TIME_PROFILER_INIT(getROSWallTime, num_threads_);
    TIME_PROFILER_ADD_ENTRY(FK);

    int num_points = evaluation_manager_->getTrajectory()->getNumPoints();

    int num_costs =	TrajectoryCostManager::getInstance()->getNumActiveCostFunctions();

    derivatives_evaluation_manager_.resize(num_threads_);
    evaluation_cost_matrices_.resize(num_threads_);
    for (int i = 0; i < num_threads_; ++i)
    {
        derivatives_evaluation_manager_[i].reset(new NewEvalManager(*evaluation_manager));
        evaluation_cost_matrices_[i] = Eigen::MatrixXd(num_points, num_costs);
	}
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

    int num_variables = evaluation_manager_->getTrajectory()->getNumParameters();

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

    std::ofstream trajectory_file;
    std::stringstream ss;
    ss << "trajectory_out_phase_" << iteration << ".txt";
    trajectory_file.open(ss.str().c_str());
    evaluation_manager_->getTrajectory()->printTrajectory(trajectory_file);
    trajectory_file.close();
}

double ImprovementManagerNLP::evaluate(const column_vector& variables)
{
    evaluation_manager_->setParameters(variables);

    double cost = evaluation_manager_->evaluate();

    evaluation_manager_->render();

    evaluation_manager_->printTrajectoryCost(++evaluation_count_, true);
    if (evaluation_count_ % 1000 == 0)
    {
        double elapsed_time = (ros::Time::now() - start_time_).toSec();
        printf("Elapsed (in eval) : %f cost : %f\n", elapsed_time, cost);

        // Assume a planning has problem if it takes more than 600 seconds.
        if (elapsed_time > 600)
            evaluation_manager_->getTrajectory()->printTrajectory(std::cout);

    }

    if (cost < best_cost_)
	{
        best_cost_ = cost;
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
        evaluation_manager_->setParameters(e);
        const double delta_plus = evaluation_manager_->evaluate();

        e(i) = old_val - eps_;
        evaluation_manager_->setParameters(e);
        double delta_minus = evaluation_manager_->evaluate();

        der(i) = (delta_plus - delta_minus) / (2 * eps_);

        e(i) = old_val;

        delta_plus_vec(i) = delta_plus;
        delta_minus_vec(i) = delta_minus;
	}

    return der;
}

//#define COMPUTE_COST_DERIVATIVE
column_vector ImprovementManagerNLP::derivative(const column_vector& variables)
{
    // assume evaluate was called before

    TIME_PROFILER_START_ITERATION;

    column_vector der;
    der.set_size(variables.size());

    // for cost debug
#ifdef COMPUTE_COST_DERIVATIVE
    std::vector<column_vector> cost_der(TrajectoryCostManager::getInstance()->getNumActiveCostFunctions());
    for (int i = 0; i < cost_der.size(); ++i)
        cost_der[i].set_size(variables.size());
    std::vector<double*> cost_der_ptr(cost_der.size());
    for (int i = 0; i < cost_der.size(); ++i)
        cost_der_ptr[i] = cost_der[i].begin();
#endif

    #pragma omp parallel for
    for (int i = 0; i < num_threads_; ++i)
    {
        derivatives_evaluation_manager_[i]->setParameters(variables);
    }

    #pragma omp parallel for
    for (int i = 0; i < variables.size(); ++i)
    {
        int thread_index = omp_get_thread_num();

        /*
        {
            std::stringstream ss;
            ss << thread_index << " begin " << i << "\n";
            std::cout << ss.str().c_str();
        }
        */

        int order = evaluation_order_[i];

        //  for cost debug
#ifndef COMPUTE_COST_DERIVATIVE
        derivatives_evaluation_manager_[thread_index]->computeDerivatives(order, variables, der.begin(), eps_);
#else
        derivatives_evaluation_manager_[thread_index]->computeCostDerivatives(order, variables, der.begin(), cost_der_ptr, eps_);
#endif

        /*
        {
            std::stringstream ss;
            ss << thread_index << " end " << i << "\n";
            std::cout << ss.str().c_str();
        }
        */
    }

    TIME_PROFILER_PRINT_ITERATION_TIME();

    // print derivatives per costs
#ifdef COMPUTE_COST_DERIVATIVE
    {
        const std::vector<TrajectoryCostPtr>& cost_functions = TrajectoryCostManager::getInstance()->getCostFunctionVector();
        std::cout.precision(3);
        std::cout.precision(std::numeric_limits<double>::digits10);
        std::cout << "component sub_component point element ";
        for (int c = 0; c < cost_functions.size(); ++c)
        {
            std::cout << cost_functions[c]->getName() << " ";
        }
        std::cout << "sum " << std::endl;
        for (int i = 0; i < variables.size(); ++i)
        {
            const ItompTrajectoryIndex& index = evaluation_manager_->getTrajectory()->getTrajectoryIndex(i);
            std::cout << i << " " << index.component << " " << index.sub_component << " " << index.point << " " << index.element << " ";
            for (int j = 0; j < cost_der.size(); ++j)
                std::cout << cost_der[j](i) << " ";
            std::cout << der(i);
            std::cout << std::endl;
        }
    }
#endif

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
        if (std::abs(der(i) - der_reference(i)) > 0.001)
        {
            const ItompTrajectoryIndex& index = evaluation_manager_->getTrajectory()->getTrajectoryIndex(i);

            ROS_INFO("Error at %d(%d %d %d %d) : %.14f (%.14f vs %.14f) max_der : %f", i,
                     index.component, index.sub_component, index.point, index.element,
                     std::abs(der(i) - der_reference(i)),
                     der(i), der_reference(i), max_der);
        }
    }
    */

    /*
    double der_max[2][3];
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 3; ++j)
        {
            der_max[i][j] = 0.0;
        }
*/
    for (int i = 0; i < der.size(); ++i)
    {
        const ItompTrajectoryIndex& index = evaluation_manager_->getTrajectory()->getTrajectoryIndex(i);
        double max_scale = 1e10;

        if (index.sub_component == ItompTrajectory::SUB_COMPONENT_TYPE_JOINT)
            max_scale = 1e6;

        if (der(i) > max_scale)
            der(i) = max_scale;
        if (der(i) < -max_scale)
            der(i) = -max_scale;

  //      if (std::abs(der(i)) > der_max[index.component][index.sub_component])
    //        der_max[index.component][index.sub_component] = std::abs(der(i)) ;
    }
    /*
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 3; ++j)
        {
            std::cout << der_max[i][j] << " ";
        }
    std::cout << std::endl;
*/
    return der;
}

void ImprovementManagerNLP::optimize(int iteration, column_vector& variables)
{
    computeEvaluationOrder(variables.size());
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


                // for walking
                if (parameter_joint_index == 3 || parameter_joint_index == 4)
                        //|| parameter_joint_index == 8 || parameter_joint_index == 11)
                {
                    //x_lower(i) = -0.001;
                    //x_upper(i) = 0.001;
                }

                if (parameter_joint_index < 2)
                {
                    x_lower(i) = PlanningParameters::getInstance()->getWorkspaceMin()[parameter_joint_index];
                    x_upper(i) = PlanningParameters::getInstance()->getWorkspaceMax()[parameter_joint_index];
                }


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
                
                x_lower(i) = -10.0;
                x_upper(i) = 10.0;
                if (index.element % 4 == 3)
                {
                    x_lower(i) = 0.0;
                    x_upper(i) = 10.0;
                }

                break;

            }
        }
        else // VELOCITY
        {
        }
    }

    evaluation_manager_->render();

    int max_iterations = PlanningParameters::getInstance()->getMaxIterations();
    if (PhaseManager::getInstance()->getPhase() > 2)
        max_iterations *= 10;
    itomp_cio_planner::find_min_box_constrained(dlib::cg_search_strategy(),//itomp_cio_planner::lbfgs_search_strategy(10),
                                   dlib::objective_delta_stop_strategy(eps_, max_iterations).be_verbose(),
                                   boost::bind(&ImprovementManagerNLP::evaluate, this, _1),
                                   boost::bind(&ImprovementManagerNLP::derivative, this, _1),
                                   variables, x_lower, x_upper);

    evaluation_manager_->setParameters(variables);
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
        const ItompTrajectoryIndex& index = evaluation_manager_->getTrajectory()->getTrajectoryIndex(i);
        if (index.point != 0)
            variables(i) += 1e-3 * noise(i);
    }
}

void ImprovementManagerNLP::computeEvaluationOrder(long variable_size)
{
    evaluation_order_.resize(variable_size);

    std::vector<long> indices_of_joint_param; // slow due to collision checking
    std::vector<long> indices_of_non_joint_param;
    indices_of_joint_param.reserve(variable_size);
    indices_of_non_joint_param.reserve(variable_size);
    for (long i = 0; i < variable_size; ++i)
    {
        const ItompTrajectoryIndex& index = evaluation_manager_->getTrajectory()->getTrajectoryIndex(i);
        if (index.sub_component == ItompTrajectory::SUB_COMPONENT_TYPE_JOINT)
            indices_of_joint_param.push_back(i);
        else
            indices_of_non_joint_param.push_back(i);
    }
    // maximize chunk size
    long write_index = 0;
    for (int i = 0; i < num_threads_; ++i)
    {
        std::copy(indices_of_joint_param.begin() + i * indices_of_joint_param.size() / num_threads_,
                  indices_of_joint_param.begin() + (i + 1) * indices_of_joint_param.size() / num_threads_,
                  evaluation_order_.begin() + write_index);
        write_index += (i + 1) * indices_of_joint_param.size() / num_threads_ - i * indices_of_joint_param.size() / num_threads_;

        std::copy(indices_of_non_joint_param.begin() + i * indices_of_non_joint_param.size() / num_threads_,
                  indices_of_non_joint_param.begin() + (i + 1) * indices_of_non_joint_param.size() / num_threads_,
                  evaluation_order_.begin() + write_index);
        write_index += (i + 1) * indices_of_non_joint_param.size() / num_threads_ - i * indices_of_non_joint_param.size() / num_threads_;
    }
    ROS_ASSERT(write_index == variable_size);
}

}
