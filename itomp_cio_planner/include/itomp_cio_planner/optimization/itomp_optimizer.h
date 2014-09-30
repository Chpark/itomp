#ifndef ITOMP_OPTIMIZER_H_
#define ITOMP_OPTIMIZER_H_

#include <itomp_cio_planner/common.h>
#include <itomp_cio_planner/trajectory/itomp_cio_trajectory.h>
#include <itomp_cio_planner/optimization/evaluation_manager.h>
#include <itomp_cio_planner/optimization/improvement_manager.h>
#include <itomp_cio_planner/optimization/best_cost_manager.h>

namespace itomp_cio_planner
{
class ItompRobotModel;
class ItompPlanningGroup;
class ItompOptimizer
{
public:
	ItompOptimizer() {};
	ItompOptimizer(int trajectory_index, ItompCIOTrajectory* trajectory, ItompRobotModel *robot_model,
			const ItompPlanningGroup *planning_group, double planning_start_time, double trajectory_start_time,
			const moveit_msgs::Constraints& path_constraints, BestCostManager* best_cost_manager,
			const planning_scene::PlanningSceneConstPtr& planning_scene);
	virtual ~ItompOptimizer();

	bool optimize();
	double getBestCost() const;
	bool isSucceed() const;
	int getLastIteration() const;

private:
	void initialize(ItompRobotModel *robot_model, const ItompPlanningGroup *planning_group,
			double trajectory_start_time, const moveit_msgs::Constraints& path_constraints,
			const planning_scene::PlanningSceneConstPtr& planning_scene);
	bool updateBestTrajectory(double cost);

	bool is_feasible;
	bool terminated_;
	int trajectory_index_;
	double planning_start_time_;

	int iteration_;
	int feasible_iteration_;
	int last_improvement_iteration_;

	ItompCIOTrajectory* full_trajectory_;
	ItompCIOTrajectory group_trajectory_;

	EvaluationManager evaluation_manager_;
	ImprovementManagerPtr improvement_manager_;

	Eigen::MatrixXd best_group_trajectory_;
	Eigen::MatrixXd best_group_contact_trajectory_;
	double best_group_trajectory_cost_;

	BestCostManager* best_cost_manager_;
};

typedef boost::shared_ptr<ItompOptimizer> ItompOptimizerPtr;

////////////////////////////////////////////////////////////////////////////////

inline double ItompOptimizer::getBestCost() const
{
	return best_group_trajectory_cost_;
}

inline bool ItompOptimizer::isSucceed() const
{
	return is_feasible;
}

inline int ItompOptimizer::getLastIteration() const
{
	return iteration_;
}

}

#endif
