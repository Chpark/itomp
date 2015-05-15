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
							   const FullTrajectoryPtr& full_trajectory,
                               const ItompTrajectoryPtr& itomp_trajectory,
							   const ItompRobotModelConstPtr& robot_model,
							   const planning_scene::PlanningSceneConstPtr& planning_scene,
							   const ItompPlanningGroupConstPtr& planning_group,
							   double planning_start_time, double trajectory_start_time,
                               const std::vector<moveit_msgs::Constraints>& trajectory_constraints) :
    trajectory_index_(trajectory_index), planning_start_time_(planning_start_time), iteration_(-1),
    best_parameter_cost_(std::numeric_limits<double>::max()), is_best_parameter_feasible_(false), best_parameter_iteration_(-1)
{
    initialize(full_trajectory, itomp_trajectory, robot_model, planning_scene, planning_group,
               trajectory_start_time, trajectory_constraints);
}

void ItompOptimizer::initialize(const FullTrajectoryPtr& full_trajectory,
                                const ItompTrajectoryPtr& itomp_trajectory,
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
    evaluation_manager_->initialize(full_trajectory, itomp_trajectory, robot_model,
									planning_scene, planning_group, planning_start_time_,
                                    trajectory_start_time, trajectory_constraints);
	improvement_manager_->initialize(evaluation_manager_, planning_group);

    PhaseManager::getInstance()->init(itomp_trajectory->getNumPoints());

	best_parameter_trajectory_.resize(Trajectory::TRAJECTORY_TYPE_NUM);
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
    int num_max_iterations = 3;
	//PlanningParameters::getInstance()->getMaxIterations(); // for CHOMP optimization

    //++iteration_;

	if (!evaluation_manager_->isLastTrajectoryFeasible())
	{
		while (iteration_ < num_max_iterations)
		{
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

            if (iteration_ >= 2)
            {
                for (int i = 1; i < evaluation_manager_->getTrajectory()->getNumPoints(); ++i)
                {
                    robot_state::RobotState robot_state(*evaluation_manager_->getRobotState(i));
                    const std::vector<const robot_model::LinkModel*>& robot_link_models = evaluation_manager_->getItompRobotModel()->getMoveitRobotModel()->getLinkModels();

                    Eigen::Affine3d root_pose;
                    Eigen::Affine3d left_foot_pose;
                    Eigen::Affine3d right_foot_pose;

                    ElementTrajectoryPtr& joint_traj = evaluation_manager_->getTrajectoryNonConst()->getElementTrajectory(ItompTrajectory::COMPONENT_TYPE_POSITION,
                                                       ItompTrajectory::SUB_COMPONENT_TYPE_JOINT);

                    for (unsigned int j = 0; j < robot_state.getVariableCount(); ++j)
                        robot_state.getVariablePositions()[j] = joint_traj->getTrajectoryPoint(i)(j);
                    robot_state.update(true);
                    root_pose = robot_state.getGlobalLinkTransform(robot_link_models[6]);

                    evaluation_manager_->getItompRobotModel()->getGroupEndeffectorPos("left_leg", robot_state, left_foot_pose);
                    evaluation_manager_->getItompRobotModel()->getGroupEndeffectorPos("right_leg", robot_state, right_foot_pose);

                    Eigen::Vector3d normal_out, pos_out, orientation_out;
                    GroundManager::getInstance()->getNearestContactPosition(left_foot_pose.translation(), exponential_map::RotationToExponentialMap(left_foot_pose.linear()),
                                                                      pos_out, orientation_out, normal_out);
                    //GroundManager::getInstance()->getNearestZPosition(left_foot_pose.translation(), exponential_map::RotationToExponentialMap(left_foot_pose.linear()),
                      //                                                pos_out, orientation_out, normal_out);

                    if ((i == 40 || normal_out.dot(left_foot_pose.translation() - pos_out) < 0.0) ||
                        (PhaseManager::getInstance()->support_foot_ == 1 && i <= 20) ||
                        (PhaseManager::getInstance()->support_foot_ != 1 && i >= 20))
                    {
                        left_foot_pose.translation() = pos_out;
                        left_foot_pose.linear() = exponential_map::ExponentialMapToRotation(orientation_out);
                    }
                    GroundManager::getInstance()->getNearestContactPosition(right_foot_pose.translation(), exponential_map::RotationToExponentialMap(right_foot_pose.linear()),
                                                                      pos_out, orientation_out, normal_out);
                    if ((i == 40 || normal_out.dot(right_foot_pose.translation() - pos_out) < 0.0) ||
                        (PhaseManager::getInstance()->support_foot_ == 2 && i <= 20) ||
                        (PhaseManager::getInstance()->support_foot_ != 2 && i >= 20))
                    {
                        right_foot_pose.translation() = pos_out;
                        right_foot_pose.linear() = exponential_map::ExponentialMapToRotation(orientation_out);
                    }

                    evaluation_manager_->getItompRobotModel()->computeStandIKState(robot_state, root_pose, left_foot_pose, right_foot_pose);

                    Eigen::MatrixXd::RowXpr traj_point = joint_traj->getTrajectoryPoint(i);
                    for (int k = 0; k < robot_state.getVariableCount(); ++k)
                        traj_point(k) = robot_state.getVariablePosition(k);

                }
            }
		}
	}

	evaluation_manager_->setParameters(best_parameter_trajectory_);
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
