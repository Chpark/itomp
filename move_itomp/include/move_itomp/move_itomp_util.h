#ifndef MOVE_ITOMP_UTIL_H_
#define MOVE_ITOMP_UTIL_H_

#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/PlanningScene.h>

namespace move_itomp_util
{

void initializePlanner(boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager> >& planner_plugin_loader,
                       planning_interface::PlannerManagerPtr& planner_instance,
                       ros::NodeHandle& node_handle,
                       robot_model::RobotModelPtr& robot_model);

void loadStaticScene(ros::NodeHandle& node_handle,
                     planning_scene::PlanningScenePtr& planning_scene,
                     robot_model::RobotModelPtr& robot_model,
                     ros::Publisher& planning_scene_diff_publisher);

void displayStates(robot_state::RobotState& start_state,
                   robot_state::RobotState& goal_state,
                   ros::NodeHandle& node_handle,
                   robot_model::RobotModelPtr& robot_model);

void doPlan(const std::string& group_name,
            planning_interface::MotionPlanRequest& req,
            planning_interface::MotionPlanResponse& res,
            robot_state::RobotState& start_state,
            robot_state::RobotState& goal_state,
            planning_scene::PlanningScenePtr& planning_scene,
            planning_interface::PlannerManagerPtr& planner_instance);

void visualizeResult(planning_interface::MotionPlanResponse& res,
                     ros::NodeHandle& node_handle,
                     int repeat_last,
                     double sleep_time);

}
#endif

