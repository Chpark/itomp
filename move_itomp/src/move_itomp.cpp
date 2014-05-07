/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta */

// Original code from pr2_moveit_tutorials::motion_planning_api_tutorial.cpp
#include <pluginlib/class_loader.h>
#include <ros/ros.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_itomp");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle node_handle("~");

  // Setting up to start using a planner is pretty easy. Planners are 
  // setup as plugins in MoveIt! and you can use the ROS pluginlib
  // interface to load any planner that you want to use. Before we 
  // can load the planner, we need two objects, a RobotModel 
  // and a PlanningScene.
  // We will start by instantiating a
  // `RobotModelLoader`_
  // object, which will look up
  // the robot description on the ROS parameter server and construct a
  // :moveit_core:`RobotModel` for us to use.
  //
  // .. _RobotModelLoader: http://docs.ros.org/api/moveit_ros_planning/html/classrobot__model__loader_1_1RobotModelLoader.html
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();

  // Using the :moveit_core:`RobotModel`, we can construct a
  // :planning_scene:`PlanningScene` that maintains the state of 
  // the world (including the robot). 
  planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));

  // We will now construct a loader to load a planner, by name. 
  // Note that we are using the ROS pluginlib library here.
  boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager> > planner_plugin_loader;
  planning_interface::PlannerManagerPtr planner_instance;
  std::string planner_plugin_name;

  // We will get the name of planning plugin we want to load
  // from the ROS param server, and then load the planner
  // making sure to catch all exceptions.
  if (!node_handle.getParam("planning_plugin", planner_plugin_name))
    ROS_FATAL_STREAM("Could not find planner plugin name");
  try
  {
    planner_plugin_loader.reset(
        new pluginlib::ClassLoader<planning_interface::PlannerManager>("moveit_core",
            "planning_interface::PlannerManager"));
  }
  catch (pluginlib::PluginlibException& ex)
  {
    ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
  }
  try
  {
    planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
    if (!planner_instance->initialize(robot_model, node_handle.getNamespace()))
      ROS_FATAL_STREAM("Could not initialize planner instance");
    ROS_INFO_STREAM("Using planning interface '" << planner_instance->getDescription() << "'");
  }
  catch (pluginlib::PluginlibException& ex)
  {
    const std::vector<std::string> &classes = planner_plugin_loader->getDeclaredClasses();
    std::stringstream ss;
    for (std::size_t i = 0; i < classes.size(); ++i)
      ss << classes[i] << " ";
    ROS_ERROR_STREAM(
        "Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl << "Available plugins: " << ss.str());
  }

  /* Sleep a little to allow time to startup rviz, etc. */
  ros::WallDuration sleep_time(1.0);
  sleep_time.sleep();

  // We will now create a motion plan request
  // specifying the desired pose of the end-effector as input.
  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;
  robot_state::RobotState& start_state = planning_scene->getCurrentStateNonConst();

  // Set start_state
  const robot_state::JointModelGroup* joint_model_group = start_state.getJointModelGroup("whole_body");
  std::map<std::string, double> values;
  joint_model_group->getVariableDefaultPositions("pressup", values);
  //joint_model_group->getVariableDefaultPositions("standup", values);
  start_state.setVariablePositions(values);

  // Copy from start_state to req.start_state
  unsigned int num_joints = start_state.getVariableCount();
  req.start_state.joint_state.name = start_state.getVariableNames();
  req.start_state.joint_state.position.resize(num_joints);
  req.start_state.joint_state.velocity.resize(num_joints);
  req.start_state.joint_state.effort.resize(num_joints);
  memcpy(&req.start_state.joint_state.position[0], start_state.getVariablePositions(), sizeof(double) * num_joints);
  if (start_state.hasVelocities())
    memcpy(&req.start_state.joint_state.velocity[0], start_state.getVariableVelocities(), sizeof(double) * num_joints);
  else
    memset(&req.start_state.joint_state.velocity[0], 0, sizeof(double) * num_joints);
  if (start_state.hasAccelerations())
    memcpy(&req.start_state.joint_state.effort[0], start_state.getVariableAccelerations(), sizeof(double) * num_joints);
  else
    memset(&req.start_state.joint_state.effort[0], 0, sizeof(double) * num_joints);

  // Now, setup a goal state
  robot_state::RobotState goal_state(start_state);
  joint_model_group->getVariableDefaultPositions("pressup2", values);
  //joint_model_group->getVariableDefaultPositions("standup", values);
  goal_state.setVariablePositions(values);
  double jointValue = 1.0;
  //goal_state.setJointPositions("base_prismatic_joint_y", &jointValue);
  req.group_name = "whole_body";
  moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);
  req.goal_constraints.push_back(joint_goal);

  // We now construct a planning context that encapsulate the scene,
  // the request and the response. We call the planner using this 
  // planning context
  planning_interface::PlanningContextPtr context = planner_instance->getPlanningContext(planning_scene, req,
      res.error_code_);
  context->solve(res);
  if (res.error_code_.val != res.error_code_.SUCCESS)
  {
    ROS_ERROR("Could not compute plan successfully");
    return 0;
  }

  // Visualize the result
  // ^^^^^^^^^^^^^^^^^^^^
  ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>(
      "/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;

  /* Visualize the trajectory */

  ROS_INFO("Visualizing the trajectory");
  moveit_msgs::MotionPlanResponse response;
  res.getMessage(response);

  display_trajectory.trajectory_start = response.trajectory_start;
  display_trajectory.trajectory.push_back(response.trajectory);
  display_publisher.publish(display_trajectory);

  sleep_time.sleep();
  ROS_INFO("Done");
  planner_instance.reset();

  return 0;
}
