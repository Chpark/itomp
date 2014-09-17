#include <ros/ros.h>
#include <itomp_cio_planner/optimization/itomp_optimizer.h>
#include <itomp_cio_planner/contact/ground_manager.h>
#include <itomp_cio_planner/visualization/new_viz_manager.h>
#include <itomp_cio_planner/util/planning_parameters.h>
#include <itomp_cio_planner/optimization/improvement_manager_nlp.h>
//#include <itomp_cio_planner/optimization/improvement_manager_chomp.h>

using namespace std;

namespace itomp_cio_planner
{

ItompOptimizer::ItompOptimizer(int trajectory_index,
		const FullTrajectoryPtr& full_trajectory,
		const ItompRobotModelConstPtr& robot_model,
		const planning_scene::PlanningSceneConstPtr& planning_scene,
		const ItompPlanningGroupConstPtr& planning_group,
		double planning_start_time, double trajectory_start_time,
		const moveit_msgs::Constraints& path_constraints) :
		trajectory_index_(trajectory_index), planning_start_time_(
				planning_start_time), iteration_(-1), best_parameter_cost_(
				std::numeric_limits<double>::max()), is_best_parameter_feasible_(
				false), best_parameter_iteration_(-1)
{
	initialize(full_trajectory, robot_model, planning_scene, planning_group,
			trajectory_start_time, path_constraints);
}

void ItompOptimizer::initialize(const FullTrajectoryPtr& full_trajectory,
		const ItompRobotModelConstPtr& robot_model,
		const planning_scene::PlanningSceneConstPtr& planning_scene,
		const ItompPlanningGroupConstPtr& planning_group,
		double trajectory_start_time,
		const moveit_msgs::Constraints& path_constraints)
{
	improvement_manager_ = boost::make_shared<ImprovementManagerNLP>();
	//improvement_manager_ = boost::make_shared<ImprovementManagerChomp>();

	NewVizManager::getInstance()->setPlanningGroup(planning_group);
	GroundManager::getInstance()->initialize();

	evaluation_manager_ = boost::make_shared<NewEvalManager>();
	evaluation_manager_->initialize(full_trajectory, robot_model,
			planning_scene, planning_group, planning_start_time_,
			trajectory_start_time, path_constraints);
	improvement_manager_->initialize(evaluation_manager_, planning_group);

	best_parameter_trajectory_.resize(Trajectory::TRAJECTORY_TYPE_NUM);
}

ItompOptimizer::~ItompOptimizer()
{
}

bool ItompOptimizer::optimize()
{
	ros::WallTime start_time = ros::WallTime::now();
	iteration_ = -1;
	best_parameter_cost_ = numeric_limits<double>::max();
	is_best_parameter_feasible_ = false;
	best_parameter_iteration_ = -1;

	improvement_manager_->updatePlanningParameters();

	NewVizManager::getInstance()->renderOneTime();

	evaluation_manager_->evaluate();

	evaluation_manager_->render();
	updateBestTrajectory();
	++iteration_;

	int iteration_after_feasible_solution = 0;
	int num_max_iterations =
			PlanningParameters::getInstance()->getMaxIterations();

	if (!evaluation_manager_->isLastTrajectoryFeasible())
	{
		while (iteration_ < num_max_iterations)
		{
			if (is_best_parameter_feasible_)
				++iteration_after_feasible_solution;

			improvement_manager_->runSingleIteration(iteration_);
			evaluation_manager_->printTrajectoryCost(iteration_);

			//bool is_cost_reduced = (evaluation_manager_->getTrajectoryCost() < best_parameter_cost_);
			bool is_updated = updateBestTrajectory();
			// is_cost_reduced : allow moving to non-feasible low-cost solutions
			// is_updated : only moves in feasible solutions
			if (!is_updated)
				evaluation_manager_->setParameters(best_parameter_trajectory_);

			++iteration_;

			if (iteration_after_feasible_solution
					> PlanningParameters::getInstance()->getMaxIterationsAfterCollisionFree())
				break;
		}
	}
	evaluation_manager_->setParameters(best_parameter_trajectory_);
	evaluation_manager_->evaluate();
	evaluation_manager_->printTrajectoryCost(iteration_, true);

	evaluation_manager_->render();
	evaluation_manager_->getFullTrajectory()->printTrajectory();

	double elpsed_time = (ros::WallTime::now() - start_time).toSec();

	ROS_INFO(
			"Terminated after %d iterations, using path from iteration %d", iteration_, best_parameter_iteration_);
	ROS_INFO(
			"We think trajectory %d is feasible: %s", trajectory_index_, (is_best_parameter_feasible_ ? "True" : "False"));
	ROS_INFO("Optimization core finished in %f sec", elpsed_time);

	planning_info_.time = elpsed_time;
	planning_info_.iterations = iteration_ + 1;
	planning_info_.cost = best_parameter_cost_;
	planning_info_.success = is_best_parameter_feasible_ ? 1 : 0;

	return is_best_parameter_feasible_;
}

bool ItompOptimizer::updateBestTrajectory()
{
	double cost = evaluation_manager_->getTrajectoryCost();
	bool is_feasible = evaluation_manager_->isLastTrajectoryFeasible();

	bool update = false;

	if (!is_best_parameter_feasible_)
	{
		if (is_feasible || cost < best_parameter_cost_)
			update = true;
		is_best_parameter_feasible_ = is_feasible;
	}
	else
	{
		if (is_feasible && cost < best_parameter_cost_)
			update = true;
	}

	if (update)
	{
		evaluation_manager_->getParameters(best_parameter_trajectory_);
		best_parameter_cost_ = cost;
		best_parameter_iteration_ = iteration_;
		return true;
	}
	return false;
}

}
