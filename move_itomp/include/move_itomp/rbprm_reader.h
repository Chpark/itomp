#ifndef RBPRM_READER_H_
#define RBPRM_READER_H_

#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/PlanningScene.h>

namespace rbprm_reader
{

std::vector<std::string> InitTrajectoryFromFile(std::vector<Eigen::VectorXd>& waypoints, std::vector<Eigen::MatrixXd>& contactPoints, const std::string& filepath);

void displayInitialWaypoints(robot_state::RobotState& state,
                             ros::NodeHandle& node_handle,
                             robot_model::RobotModelPtr& robot_model,
                             const std::vector<std::string>& hierarchy,
                             const std::vector<Eigen::VectorXd>& waypoints);

moveit_msgs::Constraints setRootJointConstraint(moveit_msgs::Constraints& c,
                                                const std::vector<std::string>& hierarchy,
                                                const Eigen::VectorXd& transform);

moveit_msgs::Constraints setContactPointConstraint(moveit_msgs::Constraints& c,
                                                   const std::vector<std::string>& hierarchy,
                                                   const Eigen::MatrixXd& transform);

moveit_msgs::Constraints setRootJointAndContactPointConstraints(const std::vector<std::string>& hierarchy,
                                                                const Eigen::VectorXd& joint_transform,
                                                                const Eigen::MatrixXd& contacts);

void setRobotStateFrom(robot_state::RobotState& state,
                       const std::vector<std::string>& hierarchy,
                       const std::vector<Eigen::VectorXd>& waypoints,
                       int index);
}
#endif

