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
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/PositionConstraint.h>
#include <moveit_msgs/OrientationConstraint.h>
#include <boost/variant/get.hpp>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shapes.h>

void addWaypoint(planning_interface::MotionPlanRequest& req, double x, double y, double z, double qx, double qy,
    double qz, double qw)
{
  moveit_msgs::PositionConstraint pos_constraint;
  moveit_msgs::OrientationConstraint ori_constraint;
  pos_constraint.target_point_offset.x = x;
  pos_constraint.target_point_offset.y = y;
  pos_constraint.target_point_offset.z = z;
  ori_constraint.orientation.x = qx;
  ori_constraint.orientation.y = qy;
  ori_constraint.orientation.z = qz;
  ori_constraint.orientation.w = qw;
  req.path_constraints.position_constraints.push_back(pos_constraint);
  req.path_constraints.orientation_constraints.push_back(ori_constraint);
}

void plan(planning_interface::PlannerManagerPtr& planner_instance, planning_scene::PlanningScenePtr planning_scene,
    planning_interface::MotionPlanRequest& req, planning_interface::MotionPlanResponse& res,
    const std::string& group_name, robot_state::RobotState& start_state, robot_state::RobotState& goal_state)
{
  const robot_state::JointModelGroup* joint_model_group = start_state.getJointModelGroup(group_name);
  req.group_name = group_name;

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

  // goal state
  moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);
  req.goal_constraints.clear();
  req.goal_constraints.push_back(joint_goal);

  planning_interface::PlanningContextPtr context = planner_instance->getPlanningContext(planning_scene, req,
      res.error_code_);
  context->solve(res);
  if (res.error_code_.val != res.error_code_.SUCCESS)
  {
    ROS_ERROR("Could not compute plan successfully");
    return;
  }
}

void loadStaticScene(ros::NodeHandle& node_handle, planning_scene::PlanningScenePtr& planning_scene,
    robot_model::RobotModelPtr& robot_model)
{
  std::string environment_file;
  std::vector<double> environment_position;

  node_handle.param<std::string>("/itomp_planner/environment_model", environment_file, "");

  if (!environment_file.empty())
  {
    double scale;
    node_handle.param("/itomp_planner/environment_model_scale", scale, 1.0);
    environment_position.resize(3, 0);
    if (node_handle.hasParam("/itomp_planner/environment_model_position"))
    {
      XmlRpc::XmlRpcValue segment;
      node_handle.getParam("/itomp_planner/environment_model_position", segment);
      if (segment.getType() == XmlRpc::XmlRpcValue::TypeArray)
      {
        int size = segment.size();
        for (int i = 0; i < size; ++i)
        {
          double value = segment[i];
          environment_position.push_back(value);
        }
      }
    }

    // Collision object
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = robot_model->getModelFrame();
    collision_object.id = "environment";
    geometry_msgs::Pose pose;
    pose.position.x = environment_position[0];
    pose.position.y = environment_position[1];
    pose.position.z = environment_position[2];
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;

    shapes::Mesh* shape = shapes::createMeshFromResource(environment_file);
    shapes::ShapeMsg mesh_msg;
    shapes::constructMsgFromShape(shape, mesh_msg);
    shape_msgs::Mesh mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

    collision_object.meshes.push_back(mesh);
    collision_object.mesh_poses.push_back(pose);

    collision_object.operation = collision_object.ADD;
    moveit_msgs::PlanningScene planning_scene_msg;
    planning_scene_msg.world.collision_objects.push_back(collision_object);
    planning_scene_msg.is_diff = true;
    planning_scene->setPlanningSceneDiffMsg(planning_scene_msg);
  }
}

void displayStates(ros::NodeHandle& node_handle, robot_model::RobotModelPtr& robot_model,
    robot_state::RobotState& start_state, robot_state::RobotState& goal_state)
{
  // display start / goal states
  int num_variables = start_state.getVariableNames().size();
  ros::Publisher start_state_display_publisher = node_handle.advertise<moveit_msgs::DisplayRobotState>(
      "/move_itomp/display_start_state", 1, true);
  moveit_msgs::DisplayRobotState disp_start_state;
  disp_start_state.state.joint_state.header.frame_id = robot_model->getModelFrame();
  disp_start_state.state.joint_state.name = start_state.getVariableNames();
  disp_start_state.state.joint_state.position.resize(num_variables);
  memcpy(&disp_start_state.state.joint_state.position[0], start_state.getVariablePositions(),
      sizeof(double) * num_variables);
  disp_start_state.highlight_links.clear();
  const std::vector<std::string>& link_model_names = robot_model->getLinkModelNames();
  for (int i = 0; i < link_model_names.size(); ++i)
  {
    std_msgs::ColorRGBA color;
    color.a = 0.5;
    color.r = 0.0;
    color.g = 1.0;
    color.b = 0.5;
    moveit_msgs::ObjectColor obj_color;
    obj_color.id = link_model_names[i];
    obj_color.color = color;
    disp_start_state.highlight_links.push_back(obj_color);
  }
  start_state_display_publisher.publish(disp_start_state);

  ros::Publisher goal_state_display_publisher = node_handle.advertise<moveit_msgs::DisplayRobotState>(
      "/move_itomp/display_goal_state", 1, true);
  moveit_msgs::DisplayRobotState disp_goal_state;
  disp_goal_state.state.joint_state.header.frame_id = robot_model->getModelFrame();
  disp_goal_state.state.joint_state.name = goal_state.getVariableNames();
  disp_goal_state.state.joint_state.position.resize(num_variables);
  memcpy(&disp_goal_state.state.joint_state.position[0], goal_state.getVariablePositions(),
      sizeof(double) * num_variables);
  disp_goal_state.highlight_links.clear();
  for (int i = 0; i < link_model_names.size(); ++i)
  {
    std_msgs::ColorRGBA color;
    color.a = 0.5;
    color.r = 0.0;
    color.g = 0.5;
    color.b = 1.0;
    moveit_msgs::ObjectColor obj_color;
    obj_color.id = link_model_names[i];
    obj_color.color = color;
    disp_goal_state.highlight_links.push_back(obj_color);
  }
  goal_state_display_publisher.publish(disp_goal_state);
}

bool isCollide(robot_state::RobotState& ik_state, planning_scene::PlanningScenePtr& planning_scene)
{
  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;
  collision_request.verbose = true;
  collision_request.contacts = false;

  collision_detection::AllowedCollisionMatrix acm = planning_scene->getAllowedCollisionMatrix();
  acm.setEntry("environment", "segment_0", true);

  planning_scene->checkCollisionUnpadded(collision_request, collision_result, ik_state, acm);
  return collision_result.collision;
}

void computeIKState(robot_state::RobotState& ik_state, planning_scene::PlanningScenePtr& planning_scene,
    const std::string& group_name, Eigen::Affine3d& end_effector_state)
{
  // compute waypoint ik solutions

  const robot_state::JointModelGroup* joint_model_group = ik_state.getJointModelGroup(group_name);

  int num_joints = ik_state.getVariableCount();

  kinematics::KinematicsQueryOptions options;
  options.return_approximate_solution = false;
  bool found_ik = false;

  robot_state::RobotState org_start(ik_state);
  int i = 0;
  while(true)
  {
    found_ik = ik_state.setFromIK(joint_model_group, end_effector_state, 10, 0.1,
        moveit::core::GroupStateValidityCallbackFn(), options);
    found_ik &= !isCollide(ik_state, planning_scene);
    if (found_ik)
      break;

    ++i;

    double dist = log(-3 + 0.001 * i) / log(10);

    ik_state.setToRandomPositionsNearBy(joint_model_group, org_start, dist);
  }

  if (found_ik)
  {
    ROS_INFO("IK solution found after %d trials", i + 1);
  }
  else
  {
    ROS_INFO("Could not find IK solution");
  }
}

void computeIKState(robot_state::RobotState& ik_state, planning_scene::PlanningScenePtr& planning_scene,
    const std::string& group_name, double x, double y, double z, double qx, double qy, double qz, double qw)
{
  Eigen::Affine3d end_effector_state = Eigen::Affine3d::Identity();
  Eigen::Quaternion<double> rot(qw, qx, qy, qz);
  Eigen::Vector3d trans(x, y, z);
  Eigen::Matrix3d mat = rot.toRotationMatrix();
  end_effector_state.linear() = mat;
  end_effector_state.translation() = trans;

  computeIKState(ik_state, planning_scene, group_name, end_effector_state);
}

void transformConstraint(double& x, double& y, double& z, double& qx, double& qy, double& qz, double& qw,
    const Eigen::Affine3d& transform)
{
  Eigen::Affine3d end_effector_state = Eigen::Affine3d::Identity();

  Eigen::Quaternion<double> rot(qw, qx, qy, qz);
  Eigen::Vector3d trans(x, y, z);
  Eigen::Matrix3d mat = rot.toRotationMatrix();
  end_effector_state.linear() = mat;
  end_effector_state.translation() = trans;

  Eigen::Matrix3d parent_mat = mat * transform.rotation().inverse();
  Eigen::Vector3d parent_trans = trans - parent_mat * transform.translation();

  Eigen::Matrix3d temp = transform.rotation().inverse();
  Eigen::Vector3d temp2 = temp * transform.translation();


  rot = Eigen::Quaternion<double>(parent_mat);

  x = parent_trans.data()[0];
  y = parent_trans.data()[1];
  z = parent_trans.data()[2];

  qw = rot.w();
  qx = rot.x();
  qy = rot.y();
  qz = rot.z();

}

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
  } catch (pluginlib::PluginlibException& ex)
  {
    ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
  }
  try
  {
    planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
    if (!planner_instance->initialize(robot_model, node_handle.getNamespace()))
      ROS_FATAL_STREAM("Could not initialize planner instance");
    ROS_INFO_STREAM("Using planning interface '" << planner_instance->getDescription() << "'");
  } catch (pluginlib::PluginlibException& ex)
  {
    const std::vector<std::string> &classes = planner_plugin_loader->getDeclaredClasses();
    std::stringstream ss;
    for (std::size_t i = 0; i < classes.size(); ++i)
      ss << classes[i] << " ";
    ROS_ERROR_STREAM(
        "Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl << "Available plugins: " << ss.str());
  }

  ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>(
      "/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;
  moveit_msgs::MotionPlanResponse response;

  loadStaticScene(node_handle, planning_scene, robot_model);

  /* Sleep a little to allow time to startup rviz, etc. */
  ros::WallDuration sleep_time(1.0);
  sleep_time.sleep();

  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;

  // Set start_state
  robot_state::RobotState& start_state = planning_scene->getCurrentStateNonConst();
  const robot_state::JointModelGroup* joint_model_group = start_state.getJointModelGroup("lower_body");
  std::map<std::string, double> values;
  //joint_model_group->getVariableDefaultPositions("pose_1", values);
  joint_model_group->getVariableDefaultPositions("idle", values);
  start_state.setVariablePositions(values);
  double jointValue = 0.0;
  //start_state.setJointPositions("base_prismatic_joint_y", &jointValue);

  // Setup a goal state
  robot_state::RobotState goal_state(start_state);
  //joint_model_group->getVariableDefaultPositions("pose_2", values);
  joint_model_group->getVariableDefaultPositions("idle", values);
  goal_state.setVariablePositions(values);
  jointValue = 2.5;
  //goal_state.setJointPositions("base_prismatic_joint_y", &jointValue);

  // setup cartesian trajectory waypoints
  /*
   addWaypoint(req, -2.5514, 0, 9.5772, 0, 0.707, 0, 0.707);
   addWaypoint(req,  0, 2.5514, 9.5772, 0.5, 0.5, -0.5, 0.5);
   addWaypoint(req, 2.5514, 0, 9.5772, 0.707, 0, -0.707, 0);
   addWaypoint(req,  0, -2.5514, 9.5772, -0.5, 0.5, 0.5, 0.5);
   addWaypoint(req, -2.5514, 0, 9.5772, 0, 0.707, 0, 0.707);
   */
  displayStates(node_handle, robot_model, start_state, goal_state);

  const double INV_SQRT_2 = 1.0 / sqrt(2.0);
  double EE_CONSTRAINTS[][7] =
  {
  /*
   { -3, 5, 3.0, 0, INV_SQRT_2, INV_SQRT_2, 0 },
   { -3, 5, 7.0, 0, INV_SQRT_2, INV_SQRT_2, 0 },
   { 3, 5, 7.0, 0, INV_SQRT_2, INV_SQRT_2, 0 },
   { 3, 5, 3.0, 0, INV_SQRT_2, INV_SQRT_2, 0 }
   */

  { 2, 0.5, 12, -0.5, -0.5, 0.5, 0.5 },
  { 2, 2, 8.5, 0, -INV_SQRT_2, INV_SQRT_2, 0 },
  { 2, 1.0, 12, -0.5, -0.5, 0.5, 0.5 },
  { 1.5, 2, 8.5, 0, -INV_SQRT_2, INV_SQRT_2, 0 },
  { 2, 1.5, 12, -0.5, -0.5, 0.5, 0.5 },
  { 1, 2, 8.5, 0, -INV_SQRT_2, INV_SQRT_2, 0 },

  };

  // transform from tcp to arm end-effector
  Eigen::Affine3d transform_1 = robot_model->getLinkModel("tcp_1_link")->getJointOriginTransform();
  Eigen::Affine3d transform_2 = robot_model->getLinkModel("tcp_2_link")->getJointOriginTransform();
  for (int i = 0; i < 6; ++i)
  {
    transformConstraint(EE_CONSTRAINTS[i][0], EE_CONSTRAINTS[i][1], EE_CONSTRAINTS[i][2], EE_CONSTRAINTS[i][3],
        EE_CONSTRAINTS[i][4], EE_CONSTRAINTS[i][5], EE_CONSTRAINTS[i][6], (i % 2 == 0) ? transform_1 : transform_2);
  }

  robot_state::RobotState from_state(start_state);
  robot_state::RobotState to_state(start_state);

  isCollide(from_state, planning_scene);
  for (int i = 0; i < 6; ++i)
  {
    ROS_INFO("*** Planning Sequence %d ***", i);

    computeIKState(to_state, planning_scene, "lower_body", EE_CONSTRAINTS[i][0], EE_CONSTRAINTS[i][1],
        EE_CONSTRAINTS[i][2], EE_CONSTRAINTS[i][3], EE_CONSTRAINTS[i][4], EE_CONSTRAINTS[i][5], EE_CONSTRAINTS[i][6]);

    // for KUKA
    if (i == 0)
    {
      from_state = to_state;
      goal_state = to_state;
    }

    if (i != 0)
    {
      req.path_constraints.position_constraints.clear();
      req.path_constraints.orientation_constraints.clear();
      addWaypoint(req, EE_CONSTRAINTS[i - 1][0], EE_CONSTRAINTS[i - 1][1], EE_CONSTRAINTS[i - 1][2],
          EE_CONSTRAINTS[i - 1][3], EE_CONSTRAINTS[i - 1][4], EE_CONSTRAINTS[i - 1][5], EE_CONSTRAINTS[i - 1][6]);
      addWaypoint(req, EE_CONSTRAINTS[i][0], EE_CONSTRAINTS[i][1], EE_CONSTRAINTS[i][2], EE_CONSTRAINTS[i][3],
          EE_CONSTRAINTS[i][4], EE_CONSTRAINTS[i][5], EE_CONSTRAINTS[i][6]);
    }

    plan(planner_instance, planning_scene, req, res, "lower_body", from_state, to_state);
    res.getMessage(response);
    if (i == 0)
      display_trajectory.trajectory_start = response.trajectory_start;
    display_trajectory.trajectory.push_back(response.trajectory);
    display_publisher.publish(display_trajectory);
    from_state = to_state;

    // use the last configuration of prev trajectory
    int num_joints = from_state.getVariableCount();
    std::vector<double> positions(num_joints);
    const robot_state::RobotState& last_state = res.trajectory_->getLastWayPoint();
    from_state.setVariablePositions(last_state.getVariablePositions());
    from_state.update();
  }

  // return to the initial position
  req.path_constraints.position_constraints.clear();
  req.path_constraints.orientation_constraints.clear();
  plan(planner_instance, planning_scene, req, res, "lower_body", from_state, goal_state);
  res.getMessage(response);
  display_trajectory.trajectory.push_back(response.trajectory);

  // publish trajectory
  display_publisher.publish(display_trajectory);

  sleep_time.sleep();
  ROS_INFO("Done");
  planner_instance.reset();

  return 0;
}
