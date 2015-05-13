#ifndef ITOMP_PLANNER_NODE_H_
#define ITOMP_PLANNER_NODE_H_

#include <itomp_cio_planner/common.h>
#include <itomp_cio_planner/planner/planning_info_manager.h>
#include <itomp_cio_planner/model/itomp_robot_model.h>
#include <itomp_cio_planner/trajectory/full_trajectory.h>
#include <itomp_cio_planner/trajectory/itomp_trajectory.h>
#include <itomp_cio_planner/optimization/itomp_optimizer.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>

namespace itomp_cio_planner
{

enum eFOOT_INDEX
{
    NO_FOOT = 0,
    LEFT_FOOT = 1,
    RIGHT_FOOT = 2,
    ANY_FOOT = 3,
};

class ItompPlannerNode
{
public:
	ItompPlannerNode(const robot_model::RobotModelConstPtr& model);
    virtual ~ItompPlannerNode();

	bool init();

    bool planTrajectory(const planning_scene::PlanningSceneConstPtr& planning_scene,
                        const planning_interface::MotionPlanRequest &req,
                        planning_interface::MotionPlanResponse &res);

private:
	bool validateRequest(const planning_interface::MotionPlanRequest &req);
    std::vector<std::string> getPlanningGroups(const std::string& group_name) const;
    void fillInResult(const robot_state::RobotStatePtr& robot_state,
                      planning_interface::MotionPlanResponse &res);

    bool adjustStartGoalPositions(robot_state::RobotState& initial_state, robot_state::RobotState& goal_state, bool read_start_state_from_previous_step);
    bool applySideStepping(const robot_state::RobotState& initial_state, robot_state::RobotState& goal_state);
    eFOOT_INDEX getFrontFoot(const robot_state::RobotState& initial_state, eFOOT_INDEX& support_foot);

    bool readWaypoint(robot_state::RobotStatePtr& robot_state);
    void writeWaypoint();
    void deleteWaypointFiles();
    void writeTrajectory();

    void readMocapData(const std::string& file_name, Eigen::MatrixXd& mocap_trajectory);

	robot_model::RobotModelConstPtr robot_model_;
	ItompRobotModelPtr itomp_robot_model_;

	FullTrajectoryPtr trajectory_;
    ItompTrajectoryPtr itomp_trajectory_;
	ItompOptimizerPtr optimizer_;
	PlanningInfoManager planning_info_manager_;
};
ITOMP_DEFINE_SHARED_POINTERS(ItompPlannerNode)

inline double normalizeAngle(double angle)
{
    while (angle > M_PI)
        angle -= 2.0 * M_PI;
    while (angle <= -M_PI)
        angle += 2.0 * M_PI;
    return angle;
}

}

#endif
