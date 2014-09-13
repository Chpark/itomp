#ifndef ITOMP_OPTIMIZER_H_
#define ITOMP_OPTIMIZER_H_

#include <itomp_cio_planner/common.h>
#include <itomp_cio_planner/trajectory/full_trajectory.h>
#include <itomp_cio_planner/trajectory/parameter_trajectory.h>
#include <itomp_cio_planner/optimization/new_eval_manager.h>
#include <itomp_cio_planner/optimization/improvement_manager.h>
#include <itomp_cio_planner/planner/planning_info_manager.h>

namespace itomp_cio_planner
{
class ItompRobotModel;
class ItompPlanningGroup;
class ItompOptimizer
{
public:
	ItompOptimizer(int trajectory_index,
			const FullTrajectoryPtr& full_trajectory,
			const ItompRobotModelConstPtr& robot_model,
			const planning_scene::PlanningSceneConstPtr& planning_scene,
			const ItompPlanningGroupConstPtr& planning_group,
			double planning_start_time, double trajectory_start_time,
			const moveit_msgs::Constraints& path_constraints);
	virtual ~ItompOptimizer();

	bool optimize();

	const PlanningInfo& getPlanningInfo() const;

private:
	void initialize(const FullTrajectoryPtr& full_trajectory,
			const ItompRobotModelConstPtr& robot_model,
			const planning_scene::PlanningSceneConstPtr& planning_scene,
			const ItompPlanningGroupConstPtr& planning_group,
			double trajectory_start_time,
			const moveit_msgs::Constraints& path_constraints);

	bool updateBestTrajectory();

	int trajectory_index_;
	double planning_start_time_;

	int iteration_;

	NewEvalManagerPtr evaluation_manager_;
	ImprovementManagerPtr improvement_manager_;

	std::vector<Eigen::MatrixXd> best_parameter_trajectory_;
	double best_parameter_cost_;
	bool is_best_parameter_feasible_;
	int best_parameter_iteration_;

	PlanningInfo planning_info_;
};
ITOMP_DEFINE_SHARED_POINTERS(ItompOptimizer);

////////////////////////////////////////////////////////////////////////////////

inline const PlanningInfo& ItompOptimizer::getPlanningInfo() const
{
	return planning_info_;
}

}

#endif
