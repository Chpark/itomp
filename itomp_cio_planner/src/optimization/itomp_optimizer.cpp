#include <ros/ros.h>
#include <itomp_cio_planner/optimization/itomp_optimizer.h>
#include <itomp_cio_planner/optimization/phase_manager.h>
#include <itomp_cio_planner/contact/ground_manager.h>
#include <itomp_cio_planner/visualization/new_viz_manager.h>
#include <itomp_cio_planner/util/planning_parameters.h>
#include <itomp_cio_planner/util/itomp_debug.h>
#include <itomp_cio_planner/optimization/improvement_manager_nlp.h>
//#include <itomp_cio_planner/optimization/improvement_manager_chomp.h>

using namespace std;

namespace itomp_cio_planner
{

ItompOptimizer::ItompOptimizer(int trajectory_index,
                               const ItompTrajectoryPtr& itomp_trajectory,
							   const ItompRobotModelConstPtr& robot_model,
							   const planning_scene::PlanningSceneConstPtr& planning_scene,
							   const ItompPlanningGroupConstPtr& planning_group,
							   double planning_start_time, double trajectory_start_time,
                               const std::vector<moveit_msgs::Constraints>& trajectory_constraints) :
    trajectory_index_(trajectory_index), planning_start_time_(planning_start_time), iteration_(-1),
    best_parameter_cost_(std::numeric_limits<double>::max()), is_best_parameter_feasible_(false), best_parameter_iteration_(-1)
{
    initialize(itomp_trajectory, robot_model, planning_scene, planning_group,
               trajectory_start_time, trajectory_constraints);
}

void ItompOptimizer::initialize(const ItompTrajectoryPtr& itomp_trajectory,
								const ItompRobotModelConstPtr& robot_model,
								const planning_scene::PlanningSceneConstPtr& planning_scene,
								const ItompPlanningGroupConstPtr& planning_group,
								double trajectory_start_time,
                                const std::vector<moveit_msgs::Constraints>& trajectory_constraints)
{
	improvement_manager_ = boost::make_shared<ImprovementManagerNLP>();
	//improvement_manager_ = boost::make_shared<ImprovementManagerChomp>();

	NewVizManager::getInstance()->setPlanningGroup(planning_group);

    NewEvalManager::ref_evaluation_manager_ = NULL;
	evaluation_manager_ = boost::make_shared<NewEvalManager>();
    evaluation_manager_->initialize(itomp_trajectory, robot_model,
									planning_scene, planning_group, planning_start_time_,
                                    trajectory_start_time, trajectory_constraints);
	improvement_manager_->initialize(evaluation_manager_, planning_group);

    PhaseManager::getInstance()->init(itomp_trajectory->getNumPoints(), planning_group);

    best_parameter_trajectory_.set_size(itomp_trajectory->getNumParameters(), 1);
}

ItompOptimizer::~ItompOptimizer()
{
    improvement_manager_.reset();
    evaluation_manager_.reset();
}

bool ItompOptimizer::optimize()
{
	ros::WallTime start_time = ros::WallTime::now();
	iteration_ = -1;
	best_parameter_cost_ = numeric_limits<double>::max();
	is_best_parameter_feasible_ = false;
	best_parameter_iteration_ = -1;

	improvement_manager_->updatePlanningParameters();

	evaluation_manager_->evaluate();

	evaluation_manager_->render();
	updateBestTrajectory();
	++iteration_;

	int iteration_after_feasible_solution = 0;
    int num_max_iterations = 5;

    ros::WallTime phase_start_time = ros::WallTime::now();

	if (!evaluation_manager_->isLastTrajectoryFeasible())
	{
		while (iteration_ < num_max_iterations)
		{
            ROS_INFO("Optimization phase %d started", iteration_);

			if (is_best_parameter_feasible_)
				++iteration_after_feasible_solution;

            PhaseManager::getInstance()->setPhase(iteration_);
            if (iteration_ != 0)
            {
                best_parameter_cost_ = numeric_limits<double>::max();
                evaluation_manager_->resetBestTrajectoryCost();
            }

            ROS_INFO("Planning Phase %d...", iteration_);

			improvement_manager_->runSingleIteration(iteration_);
			evaluation_manager_->printTrajectoryCost(iteration_);

			//bool is_cost_reduced = (evaluation_manager_->getTrajectoryCost() < best_parameter_cost_);
			bool is_updated = updateBestTrajectory();
			// is_cost_reduced : allow moving to non-feasible low-cost solutions
			// is_updated : only moves in feasible solutions
			if (!is_updated)
				evaluation_manager_->setParameters(best_parameter_trajectory_);

			++iteration_;

            if (iteration_after_feasible_solution > PlanningParameters::getInstance()->getMaxIterationsAfterCollisionFree())
				break;

            if (iteration_ == 1)
                evaluation_manager_->correctEndPointContacts();
            if (iteration_ == 1)
                evaluation_manager_->getTrajectoryNonConst()->interpolateStartEnd(ItompTrajectory::SUB_COMPONENT_TYPE_JOINT);
            evaluation_manager_->correctContacts(evaluation_manager_->getTrajectory()->getNumPoints() - 1, evaluation_manager_->getTrajectory()->getNumPoints(), true);
            evaluation_manager_->render();

            /*
            if (iteration_ == 1)
            {

                ItompTrajectoryIndex idx;
                idx.component = 0;
                idx.sub_component = 0;
                idx.element = 62;

                ElementTrajectoryPtr& et = evaluation_manager_->getTrajectoryNonConst()->getElementTrajectory(idx.component, idx.sub_component);


                for (int i = 4; i < 32; i += 4)
                {
                    idx.point = i;
                    Eigen::MatrixXd::RowXpr row = et->getTrajectoryPoint(idx.point);
                    if (i < 16)
                        row(idx.element) -= 0.1 * M_PI * ((double)i / 16);
                    else
                        row(idx.element) -= 0.1 * M_PI * (2 - (double)i / 16);
                }
            }
            */

            if (iteration_ == 3 || iteration_ == 5)
            {
                double phase_time = (ros::WallTime::now() - phase_start_time).toSec();
                ros::NodeHandle node_handle("/move_itomp");
                std::string motion_name;
                if (node_handle.getParam("/move_itomp/motion_name", motion_name))
                {
                    std::ofstream trajectory_file;
                    trajectory_file.open("performance.txt", ios::out | ios::app);
                    trajectory_file.precision(std::numeric_limits<double>::digits10);
                    trajectory_file << motion_name << (iteration_ == 3 ? " P2 " : " P3 " ) << phase_time << " ";

                    TIME_PROFILER_PRINT_TOTAL_TIME(trajectory_file, false);
                    if (iteration_ == 5)
                        trajectory_file << "\n";
                    trajectory_file.close();
                    TIME_PROFILER_RESET
                }
                phase_start_time = ros::WallTime::now();
            }
		}
	}

	evaluation_manager_->setParameters(best_parameter_trajectory_);
    evaluation_manager_->correctContacts();
	evaluation_manager_->evaluate();
	evaluation_manager_->printTrajectoryCost(iteration_);

	evaluation_manager_->render();

	double elpsed_time = (ros::WallTime::now() - start_time).toSec();

    //ROS_INFO("Terminated after %d iterations, using path from iteration %d", iteration_, best_parameter_iteration_);
    //ROS_INFO("We think trajectory %d is feasible: %s", trajectory_index_, (is_best_parameter_feasible_ ? "True" : "False"));
    //ROS_INFO("Optimization core finished in %f sec", elpsed_time);

	planning_info_.time = elpsed_time;
	planning_info_.iterations = iteration_ + 1;
	planning_info_.cost = best_parameter_cost_;
	planning_info_.success = is_best_parameter_feasible_ ? 1 : 0;

    evaluation_manager_->printLinkTransforms();

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
