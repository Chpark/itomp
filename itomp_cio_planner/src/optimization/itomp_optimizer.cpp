#include <ros/ros.h>
#include <itomp_cio_planner/optimization/itomp_optimizer.h>
#include <itomp_cio_planner/optimization/phase_manager.h>
#include <itomp_cio_planner/contact/ground_manager.h>
#include <itomp_cio_planner/visualization/new_viz_manager.h>
#include <itomp_cio_planner/util/planning_parameters.h>
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

	if (!evaluation_manager_->isLastTrajectoryFeasible())
	{
		while (iteration_ < num_max_iterations)
		{
            // initialize contact positions
            if (false)//iteration_ == 0)
            {
                const int num_contacts = evaluation_manager_->contact_variables_[0].size();

                for (int contact_id = 0; contact_id < num_contacts; contact_id++)
                {
                    for (int i=0; i<4; i++)
                    {
                        evaluation_manager_->contact_variables_[ 5][contact_id].setPosition( evaluation_manager_->contact_variables_[ 0][contact_id].getPosition() );
                        evaluation_manager_->contact_variables_[15][contact_id].setPosition( evaluation_manager_->contact_variables_[20][contact_id].getPosition() );
                        evaluation_manager_->contact_variables_[25][contact_id].setPosition( evaluation_manager_->contact_variables_[20][contact_id].getPosition() );
                        evaluation_manager_->contact_variables_[35][contact_id].setPosition( evaluation_manager_->contact_variables_[40][contact_id].getPosition() );
                    }

                    evaluation_manager_->itomp_trajectory_->setContactVariables( 5, evaluation_manager_->contact_variables_[ 5]);
                    evaluation_manager_->itomp_trajectory_->setContactVariables(15, evaluation_manager_->contact_variables_[15]);
                    evaluation_manager_->itomp_trajectory_->setContactVariables(25, evaluation_manager_->contact_variables_[25]);
                    evaluation_manager_->itomp_trajectory_->setContactVariables(35, evaluation_manager_->contact_variables_[35]);
                }

                evaluation_manager_->getTrajectoryNonConst()->interpolate( 0,  5, ItompTrajectory::SUB_COMPONENT_TYPE_CONTACT_POSITION);
                evaluation_manager_->getTrajectoryNonConst()->interpolate( 5, 15, ItompTrajectory::SUB_COMPONENT_TYPE_CONTACT_POSITION);
                evaluation_manager_->getTrajectoryNonConst()->interpolate(15, 20, ItompTrajectory::SUB_COMPONENT_TYPE_CONTACT_POSITION);
            }


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

            /*
            if (iteration_ == 1)
            {
                const int num_points = evaluation_manager_->getTrajectory()->getNumPoints();
                evaluation_manager_->getTrajectoryNonConst()->interpolate(0, num_points / 2, ItompTrajectory::SUB_COMPONENT_TYPE_JOINT);
                evaluation_manager_->getTrajectoryNonConst()->interpolate(num_points / 2, num_points - 1, ItompTrajectory::SUB_COMPONENT_TYPE_JOINT);
            }
            */
            //if (iteration_ == 1)
            {
                //evaluation_manager_->correctContacts();
            }

            /*
            if (iteration_ == 1)
            {
                // set contact forces to 0
                const int num_contacts = evaluation_manager_->contact_variables_[0].size();

                for (int contact_id = 0; contact_id < num_contacts; contact_id++)
                {
                    if (!evaluation_manager_->planning_group_->is_fixed_[contact_id])
                    {
                        for (int point = 5; point <= 15; point += 5)
                        {
                            for (int i=0; i<4; i++)
                            {
                                const Eigen::Vector3d zero(0.0, 0.0, 0.0);
                                evaluation_manager_->contact_variables_[point][contact_id].setPointForce(i, zero);
                            }

                            evaluation_manager_->itomp_trajectory_->setContactVariables(point, evaluation_manager_->contact_variables_[point]);
                        }
                    }
                }

                evaluation_manager_->getTrajectoryNonConst()->interpolate( 0,  5, ItompTrajectory::SUB_COMPONENT_TYPE_CONTACT_FORCE);
                evaluation_manager_->getTrajectoryNonConst()->interpolate( 5, 15, ItompTrajectory::SUB_COMPONENT_TYPE_CONTACT_FORCE);
                evaluation_manager_->getTrajectoryNonConst()->interpolate(15, 20, ItompTrajectory::SUB_COMPONENT_TYPE_CONTACT_FORCE);
            }
            */

            evaluation_manager_->render();
		}
	}

	evaluation_manager_->setParameters(best_parameter_trajectory_);
    //evaluation_manager_->correctContacts();
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
