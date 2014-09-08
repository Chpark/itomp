#ifndef ITOMP_OPTIMIZER_H_
#define ITOMP_OPTIMIZER_H_

#include <itomp_cio_planner/common.h>
#include <itomp_cio_planner/trajectory/itomp_cio_trajectory.h>
#include <itomp_cio_planner/optimization/evaluation_manager.h>
#include <itomp_cio_planner/optimization/improvement_manager.h>
#include <itomp_cio_planner/planner/planning_info_manager.h>

namespace itomp_cio_planner
{
class ItompRobotModel;
class ItompPlanningGroup;
class ItompOptimizer
{
public:
	ItompOptimizer(int trajectory_index, ItompCIOTrajectory* trajectory, ItompRobotModel *robot_model,
			const ItompPlanningGroup *planning_group, double planning_start_time, double trajectory_start_time,
			const moveit_msgs::Constraints& path_constraints);
	virtual ~ItompOptimizer();

	bool optimize();

	const PlanningInfo& getPlanningInfo() const;

private:
	void initialize(ItompRobotModel *robot_model, const ItompPlanningGroup *planning_group,
			double trajectory_start_time, const moveit_msgs::Constraints& path_constraints);
	bool updateBestTrajectory(double cost, bool is_feasible);

	int trajectory_index_;
	double planning_start_time_;

	int iteration_;

	ItompCIOTrajectory* full_trajectory_;
	ItompCIOTrajectoryPtr group_trajectory_;

	EvaluationManager evaluation_manager_;
	ImprovementManagerPtr improvement_manager_;

	Eigen::MatrixXd best_group_trajectory_;
	double best_group_trajectory_cost_;
	bool is_best_group_trajectory_feasible_;
	int best_group_trajectory_iteration_;

	PlanningInfo planning_info_;
};

typedef boost::shared_ptr<ItompOptimizer> ItompOptimizerPtr;

////////////////////////////////////////////////////////////////////////////////

inline const PlanningInfo& ItompOptimizer::getPlanningInfo() const
{
  return planning_info_;
}

}

#endif
