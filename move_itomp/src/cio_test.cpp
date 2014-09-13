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
#include <map>
#include <cmath>

const std::string GROUP_NAME = "lower_body";
const double INV_SQRT_2 = 1.0 / std::sqrt((long double)2.0);

void renderHierarchicalTrajectory(robot_trajectory::RobotTrajectoryPtr& robot_trajectory, ros::NodeHandle& node_handle,
    robot_model::RobotModelPtr& robot_model)
{
  static ros::Publisher vis_marker_array_publisher_ = node_handle.advertise<visualization_msgs::MarkerArray>(
      "itomp_planner/trajectory", 10);
  visualization_msgs::MarkerArray ma;
  std::vector < std::string > link_names = robot_model->getLinkModelNames();
  std_msgs::ColorRGBA color;
  std::string ns = "robot";
  ros::Duration dur(100.0);

  std::map<std::string, std_msgs::ColorRGBA> colorMap;
  color.a = 0.3;
  color.r = 0.5;
  color.g = 1.0;
  color.b = 1.0;
  colorMap["lower_body"] = color;
  color.a = 0.3;
  color.r = 0.5;
  color.g = 1.0;
  color.b = 0.3;
  colorMap["torso"] = color;
  color.g = 0.3;
  color.b = 1.0;
  colorMap["head"] = color;
  color.r = 0.9;
  color.g = 0.5;
  color.b = 0.3;
  colorMap["left_arm"] = color;
  color.r = 0.9;
  color.g = 0.3;
  color.b = 0.5;
  colorMap["right_arm"] = color;
  color.r = 1.0;
  color.g = 1.0;
  color.b = 1.0;
  colorMap["object"] = color;

  const robot_state::JointModelGroup* joint_model_group;
  std::map < std::string, std::vector<std::string> > group_links_map;

  group_links_map["lower_body"] = robot_model->getJointModelGroup("lower_body")->getLinkModelNames();
  group_links_map["torso"] = robot_model->getJointModelGroup("torso")->getLinkModelNames();
  group_links_map["head"] = robot_model->getJointModelGroup("head")->getLinkModelNames();
  group_links_map["left_arm"] = robot_model->getJointModelGroup("left_arm")->getLinkModelNames();
  group_links_map["right_arm"] = robot_model->getJointModelGroup("right_arm")->getLinkModelNames();

  group_links_map["object"].clear();
  if (robot_model->hasLinkModel("right_hand_object_link"))
    group_links_map["object"].push_back("right_hand_object_link");

  int num_waypoints = robot_trajectory->getWayPointCount();
  for (int i = 0; i < num_waypoints; ++i)
  {
    ma.markers.clear();
    robot_state::RobotStatePtr state = robot_trajectory->getWayPointPtr(i);

    for (std::map<std::string, std::vector<std::string> >::iterator it = group_links_map.begin();
        it != group_links_map.end(); ++it)
    {
      std::string ns = "robot_" + it->first;
      state->getRobotMarkers(ma, group_links_map[it->first], colorMap[it->first], ns, dur);
    }
    vis_marker_array_publisher_.publish(ma);

    double time = (i == 0 || i == num_waypoints - 1) ? 2.0 : 0.05;
    ros::WallDuration timer(time);
    timer.sleep();
  }
  for (int i = 0; i < 10; ++i)
  {
    ma.markers.clear();
    robot_state::RobotStatePtr state = robot_trajectory->getWayPointPtr(num_waypoints - 1);

    for (std::map<std::string, std::vector<std::string> >::iterator it = group_links_map.begin();
        it != group_links_map.end(); ++it)
    {
      std::string ns = "robot_" + it->first;
      state->getRobotMarkers(ma, group_links_map[it->first], colorMap[it->first], ns, dur);
    }
    vis_marker_array_publisher_.publish(ma);

    double time = 0.05;
    ros::WallDuration timer(time);
    timer.sleep();
  }

}

void renderEnvironment(const std::string& environment_file, robot_model::RobotModelPtr& robot_model,
    const std::string& ns, std_msgs::ColorRGBA& color)
{
  ros::NodeHandle node_handle;
  ros::Publisher vis_marker_array_publisher_ = node_handle.advertise<visualization_msgs::MarkerArray>(
      "itomp_planner/visualization_marker_array", 10);
  visualization_msgs::MarkerArray ma;
  visualization_msgs::Marker msg;
  msg.header.frame_id = robot_model->getModelFrame();
  msg.header.stamp = ros::Time::now();
  msg.ns = ns;
  msg.type = visualization_msgs::Marker::MESH_RESOURCE;
  msg.action = visualization_msgs::Marker::ADD;
  msg.scale.x = 1.0;
  msg.scale.y = 1.0;
  msg.scale.z = 1.0;
  msg.id = 0;
  msg.pose.position.x = 0.0;
  msg.pose.position.y = 0.0;
  msg.pose.position.z = 0.0;
  msg.pose.orientation.x = 0.0;
  msg.pose.orientation.y = 0.0;
  msg.pose.orientation.z = 0.0;
  msg.pose.orientation.w = 1.0;
  msg.color = color;
  /*
   msg.color.a = 1.0;
   msg.color.r = 0.5;
   msg.color.g = 0.5;
   msg.color.b = 0.5;
   */
  msg.mesh_resource = environment_file;
  ma.markers.push_back(msg);
  vis_marker_array_publisher_.publish(ma);
}

void computeIKState(robot_state::RobotState& ik_state, const std::string& group_name, double x, double y, double z,
    double qx, double qy, double qz, double qw)
{
  // compute waypoint ik solutions

  const robot_state::JointModelGroup* joint_model_group = ik_state.getJointModelGroup(group_name);

  int num_joints = ik_state.getVariableCount();

  Eigen::Affine3d end_effector_state = Eigen::Affine3d::Identity();
  Eigen::Quaternion<double> rot(qw, qx, qy, qz);
  Eigen::Vector3d trans(x, y, z);
  Eigen::Matrix3d mat = rot.toRotationMatrix();
  end_effector_state.linear() = mat;
  end_effector_state.translation() = trans;

  kinematics::KinematicsQueryOptions options;
  options.return_approximate_solution = false;
  bool found_ik = ik_state.setFromIK(joint_model_group, end_effector_state, 10, 0.1,
      moveit::core::GroupStateValidityCallbackFn(), options);
  if (found_ik)
  {
    //ROS_INFO("IK solution found");
  }
  else
  {
    ROS_INFO("Could not find IK solution");
  }
}

void visualizeResult(planning_interface::MotionPlanResponse& res, ros::NodeHandle& node_handle, int repeat_last,
    double sleep_time)
{
  // Visualize the result
  // ^^^^^^^^^^^^^^^^^^^^
  static ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>(
      "/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;

  ROS_INFO("Visualizing the trajectory");
  moveit_msgs::MotionPlanResponse response;

  for (int i = 0; i < repeat_last; ++i)
    res.trajectory_->addSuffixWayPoint(res.trajectory_->getLastWayPoint(), 5000);
  res.getMessage(response);

  display_trajectory.trajectory_start = response.trajectory_start;
  display_trajectory.trajectory.push_back(response.trajectory);
  display_publisher.publish(display_trajectory);
  ros::WallDuration timer(sleep_time);
  timer.sleep();
}

void doPlan(const std::string& group_name, planning_interface::MotionPlanRequest& req,
    planning_interface::MotionPlanResponse& res, robot_state::RobotState& start_state,
    robot_state::RobotState& goal_state, planning_scene::PlanningScenePtr& planning_scene,
    planning_interface::PlannerManagerPtr& planner_instance)
{
  const robot_state::JointModelGroup* joint_model_group = goal_state.getJointModelGroup("whole_body");

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

  req.group_name = group_name;
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
    exit(0);
  }
}

void displayStates(robot_state::RobotState& start_state, robot_state::RobotState& goal_state,
    ros::NodeHandle& node_handle, robot_model::RobotModelPtr& robot_model)
{
  static ros::Publisher start_state_display_publisher = node_handle.advertise<moveit_msgs::DisplayRobotState>(
      "/move_itomp/display_start_state", 1, true);
  static ros::Publisher goal_state_display_publisher = node_handle.advertise<moveit_msgs::DisplayRobotState>(
      "/move_itomp/display_goal_state", 1, true);

  int num_variables = start_state.getVariableNames().size();

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

void setWalkingStates(robot_state::RobotState& start_state, robot_state::RobotState& goal_state,
    Eigen::Vector3d& start_trans, Eigen::Vector3d& goal_trans, double start_rot = 0, double end_rot = 0)
{
  std::map<std::string, double> values;
  double jointValue = 0.0;

  const robot_state::JointModelGroup* joint_model_group = start_state.getJointModelGroup("whole_body");

  joint_model_group->getVariableDefaultPositions("standup", values);
  start_state.setVariablePositions(values);
  jointValue = start_trans(0);
  start_state.setJointPositions("base_prismatic_joint_x", &jointValue);
  jointValue = start_trans(1);
  start_state.setJointPositions("base_prismatic_joint_y", &jointValue);
  jointValue = start_trans(2);
  start_state.setJointPositions("base_prismatic_joint_z", &jointValue);
  jointValue = start_rot;
  start_state.setJointPositions("base_revolute_joint_z", &jointValue);

  goal_state = start_state;
  joint_model_group->getVariableDefaultPositions("standup", values);
  goal_state.setVariablePositions(values);
  jointValue = goal_trans(0);
  goal_state.setJointPositions("base_prismatic_joint_x", &jointValue);
  jointValue = goal_trans(1);
  goal_state.setJointPositions("base_prismatic_joint_y", &jointValue);
  jointValue = goal_trans(2);
  goal_state.setJointPositions("base_prismatic_joint_z", &jointValue);
  jointValue = end_rot;
  goal_state.setJointPositions("base_revolute_joint_z", &jointValue);

  /*
  // test
  //double temp[] = {0.1,0.2,0.3,0.4,0.5,0.6};
  double temp[] = {0,0,0,0,0,3.14};
  start_state.setJointPositions("base_prismatic_joint_x", &temp[0]);
  start_state.setJointPositions("base_prismatic_joint_y", &temp[1]);
  start_state.setJointPositions("base_prismatic_joint_z", &temp[2]);
  start_state.setJointPositions("base_revolute_joint_x", &temp[3]);
  start_state.setJointPositions("base_revolute_joint_y", &temp[4]);
  start_state.setJointPositions("base_revolute_joint_z", &temp[5]);
  temp[5] = -3.14;
  goal_state.setJointPositions("base_revolute_joint_z", &temp[5]);
  */
}

void setDoorOpeningStates(robot_state::RobotState& pre_state, robot_state::RobotState& start_state,
    robot_state::RobotState& goal_state, robot_state::RobotState& post_state)
{
  std::map<std::string, double> values;
  double jointValue = 0.0;

  const robot_state::JointModelGroup* joint_model_group = start_state.getJointModelGroup("whole_body");

  // pre state
  joint_model_group->getVariableDefaultPositions("standup", values);
  pre_state.setVariablePositions(values);
  jointValue = -0.55;
  pre_state.setJointPositions("base_prismatic_joint_x", &jointValue);
  jointValue = -0.5;
  pre_state.setJointPositions("base_prismatic_joint_y", &jointValue);

  joint_model_group->getVariableDefaultPositions("door_1_new_whole", values);
  start_state.setVariablePositions(values);
  jointValue = -0.55;
  start_state.setJointPositions("base_prismatic_joint_x", &jointValue);
  jointValue = -0.5;
  start_state.setJointPositions("base_prismatic_joint_y", &jointValue);

  const double INV_SQRT_2 = 1.0 / std::sqrt((long double)2.0);
  computeIKState(start_state, "right_arm", -0.5, -0.065, 1.6, INV_SQRT_2, -INV_SQRT_2, 0, 0);

  // Now, setup a goal state
  goal_state = start_state;
  joint_model_group->getVariableDefaultPositions("door_2_new_whole", values);
  goal_state.setVariablePositions(values);
  jointValue = -0.55;
  goal_state.setJointPositions("base_prismatic_joint_x", &jointValue);
  jointValue = 0.5;
  goal_state.setJointPositions("base_prismatic_joint_y", &jointValue);
  computeIKState(goal_state, "right_arm", 0.0 - 0.065, 0.5, 1.6, 0, 1, 0, 0);

  // post state
  joint_model_group->getVariableDefaultPositions("standup", values);
  post_state.setVariablePositions(values);
  jointValue = -0.55;
  post_state.setJointPositions("base_prismatic_joint_x", &jointValue);
  jointValue = 0.5;
  post_state.setJointPositions("base_prismatic_joint_y", &jointValue);

}

void setDrawerStates(robot_state::RobotState& pre_state, robot_state::RobotState& start_state,
    robot_state::RobotState& goal_state, robot_state::RobotState& post_state)
{
  double jointValue = 0.0;

  // Set start_state
  const robot_state::JointModelGroup* joint_model_group = start_state.getJointModelGroup("whole_body");
  std::map<std::string, double> values;

  joint_model_group->getVariableDefaultPositions("stand", values);
  pre_state.setVariablePositions(values);
  jointValue = -2.71;
  pre_state.setJointPositions("base_prismatic_joint_x", &jointValue);
  jointValue = 5.3 - 0.23;
  pre_state.setJointPositions("base_prismatic_joint_y", &jointValue);
  jointValue = M_PI * 0.5;
  pre_state.setJointPositions("base_revolute_joint_z", &jointValue);

  joint_model_group->getVariableDefaultPositions("open_whole_3_new", values);
  start_state.setVariablePositions(values);
  jointValue = -2.71;
  start_state.setJointPositions("base_prismatic_joint_x", &jointValue);
  jointValue = 5.3 - 0.23;
  start_state.setJointPositions("base_prismatic_joint_y", &jointValue);
  jointValue = M_PI * 0.5;
  start_state.setJointPositions("base_revolute_joint_z", &jointValue);
  start_state.updateLinkTransforms();
  Eigen::Matrix3d mat;
  Eigen::Affine3d transform = start_state.getFrameTransform("right_hand_endeffector_link");
  {
    Eigen::Affine3d transform = start_state.getFrameTransform("right_hand_endeffector_link");
    Eigen::Quaternion<double> rot(0.33, 0, std::sqrt((long double)(1.0 - 0.33 * 0.33)), 0);
    mat = rot.toRotationMatrix();
    transform.linear() = mat;
    kinematics::KinematicsQueryOptions options;
    options.return_approximate_solution = false;
    const robot_state::JointModelGroup* joint_model_group = start_state.getJointModelGroup("right_arm");
    bool found_ik = start_state.setFromIK(joint_model_group, transform, 10, 0.1,
        moveit::core::GroupStateValidityCallbackFn(), options);
    if (found_ik)
    {
      ROS_INFO("IK solution found");
    }
    else
    {
      ROS_INFO("Could not find IK solution");
    }
  }

  joint_model_group->getVariableDefaultPositions("open_whole_3_new", values);
  goal_state.setVariablePositions(values);
  jointValue = -2.71;
  goal_state.setJointPositions("base_prismatic_joint_x", &jointValue);
  jointValue = 5.3 - 0.23;
  goal_state.setJointPositions("base_prismatic_joint_y", &jointValue);
  jointValue = M_PI * 0.5;
  goal_state.setJointPositions("base_revolute_joint_z", &jointValue);
  goal_state.updateLinkTransforms();
  {
    Eigen::Affine3d transform = goal_state.getFrameTransform("right_hand_endeffector_link");
    transform.translation()(0) += 0.2;
    transform.linear() = mat;
    kinematics::KinematicsQueryOptions options;
    options.return_approximate_solution = false;
    const robot_state::JointModelGroup* joint_model_group = goal_state.getJointModelGroup("right_arm");
    bool found_ik = goal_state.setFromIK(joint_model_group, transform, 10, 0.1,
        moveit::core::GroupStateValidityCallbackFn(), options);
    if (found_ik)
    {
      ROS_INFO("IK solution found");
    }
    else
    {
      ROS_INFO("Could not find IK solution");
    }
  }

  post_state = pre_state;
}

int main(int argc, char **argv)
{
  int motion = 0;
  if (argc >= 2)
  {
    motion = atoi(argv[1]);
  }
  printf("%d Motion %d\n", argc, motion);

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

  /* Sleep a little to allow time to startup rviz, etc. */
  ros::WallDuration sleep_time(1.0);
  sleep_time.sleep();

  std_msgs::ColorRGBA color;
  color.a = 1.0;
  color.r = 0.5;
  color.g = 0.5;
  color.b = 0.5;
  renderEnvironment("package://move_itomp/meshes/merged_empty.dae", robot_model, "environment", color);
  color.r = 1.0;
  color.g = 1.0;
  color.b = 1.0;
  renderEnvironment("package://move_itomp/meshes/drawer2.dae", robot_model, "drawer", color);
  //renderEnvironment("package://move_itomp/meshes/drawer2_opened.dae", robot_model, "drawer_opened", color);
  //renderEnvironment("package://move_itomp/meshes/door.dae", robot_model, "door", color);
  renderEnvironment("package://move_itomp/meshes/door_opened.dae", robot_model, "door_opened", color);

  // We will now create a motion plan request
  // specifying the desired pose of the end-effector as input.
  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;

  std::vector<robot_state::RobotState> robot_states;

  int state_index = 0;

  motion = 9;
  switch (motion)
  {
  case 0:
    //walking
  {
    robot_states.push_back(planning_scene->getCurrentStateNonConst());
    robot_states.push_back(robot_states.back());
    Eigen::Vector3d start_trans(-0.55, -2.7, -0.6);
    Eigen::Vector3d goal_trans(-0.55, -0.5, 0);
    setWalkingStates(robot_states[state_index], robot_states[state_index + 1], start_trans, goal_trans);
    doPlan("lower_body", req, res, robot_states[state_index], robot_states[state_index + 1], planning_scene,
        planner_instance);
    renderHierarchicalTrajectory(res.trajectory_, node_handle, robot_model);
  }
    break;

  case 1:
    // move hand
  {
    robot_states.push_back(planning_scene->getCurrentStateNonConst());
    robot_states.push_back(robot_states.back());
    robot_states.push_back(robot_states.back());
    robot_states.push_back(robot_states.back());

    // opening a door states
    setDoorOpeningStates(robot_states[state_index], robot_states[state_index + 1], robot_states[state_index + 2],
        robot_states[state_index + 3]);

    doPlan("right_arm", req, res, robot_states[state_index], robot_states[state_index + 1], planning_scene,
        planner_instance);
  }
    break;

  case 2: // open door
  {
    robot_states.push_back(planning_scene->getCurrentStateNonConst());
    robot_states.push_back(robot_states.back());
    robot_states.push_back(robot_states.back());
    robot_states.push_back(robot_states.back());

    // opening a door states
    setDoorOpeningStates(robot_states[state_index], robot_states[state_index + 1], robot_states[state_index + 2],
        robot_states[state_index + 3]);

    ++state_index;

    doPlan("lower_body", req, res, robot_states[state_index], robot_states[state_index + 1], planning_scene,
        planner_instance);
    {
      // hack
      double x = 0.5;  // - 0.05;
      double y = 0.065;  // + 0.1 + 0.065 - 0.02;
      int num_waypoints = res.trajectory_->getWayPointCount();
      const double radius = std::sqrt((long double)(x * x + y * y));
      const double theta_start = std::atan2(-(long double)y, -(long double)x) + 2 * M_PI;
      const double theta_end = std::atan2((long double)x, -(long double)y);  // - 0.3;
      const Eigen::Quaternion<double> rot_start(0, INV_SQRT_2, -INV_SQRT_2, 0);
      const Eigen::Quaternion<double> rot_end(0, 0, 1, 0);
      double start_y = res.trajectory_->getFirstWayPoint().getVariablePosition(1);
      double end_y = res.trajectory_->getLastWayPoint().getVariablePosition(1);

      const robot_state::JointModelGroup* joint_model_group2 = res.trajectory_->getFirstWayPoint().getJointModelGroup(
          "right_arm");
      const std::vector<std::string> joint_model_names = joint_model_group2->getJointModelNames();

      std::map<std::string, double> joint_val_map;
      for (int i = 0; i < num_waypoints; ++i)
      {
        robot_state::RobotStatePtr state = res.trajectory_->getWayPointPtr(i);
        double cy = state->getVariablePosition(1);
        //double t = (double)i / (num_waypoints - 1);
        double t = (cy - start_y) / (end_y - start_y);
        //ROS_INFO("T : %f", t);
        double theta_interp = theta_start + t * (theta_end - theta_start);

        double x = std::cos((long double)theta_interp) * radius;
        double y = std::sin((long double)theta_interp) * radius;

        const Eigen::Quaternion<double> q = rot_start.slerp(t, rot_end);

        if (i != 0)
        {
          state->setVariablePositions(joint_val_map);
        }

        computeIKState(*state, "right_arm", x, y, 1.6, q.x(), q.y(), q.z(), q.w());

        for (int j = 0; j < joint_model_names.size(); ++j)
        {
          const std::string name = joint_model_names[j];
          double v = state->getVariablePosition(name);
          joint_val_map[name] = v;
        }
      }
    }
  }
    break;

  case 3: // move hand
  {
    robot_states.push_back(planning_scene->getCurrentStateNonConst());
    robot_states.push_back(robot_states.back());
    robot_states.push_back(robot_states.back());
    robot_states.push_back(robot_states.back());

    // opening a door states
    setDoorOpeningStates(robot_states[state_index], robot_states[state_index + 1], robot_states[state_index + 2],
        robot_states[state_index + 3]);

    state_index += 2;

    doPlan("right_arm", req, res, robot_states[state_index], robot_states[state_index + 1], planning_scene,
        planner_instance);
  }
    break;

  case 4: //walking
  {
    robot_states.push_back(planning_scene->getCurrentStateNonConst());
    robot_states.push_back(robot_states.back());
    Eigen::Vector3d start_trans(-0.55, 0.5, 0);
    Eigen::Vector3d goal_trans(-0.55, 3.5, 0);
    setWalkingStates(robot_states[state_index], robot_states[state_index + 1], start_trans, goal_trans, 0, 0);
    doPlan("lower_body", req, res, robot_states[state_index], robot_states[state_index + 1], planning_scene,
        planner_instance);
  }
    break;

  case 5: // walking
  {
    robot_states.push_back(planning_scene->getCurrentStateNonConst());
    robot_states.push_back(robot_states.back());
    Eigen::Vector3d start_trans(-0.55, 3.5, 0);
    Eigen::Vector3d goal_trans(-2.71, 5.3 - 0.23, 0);
    setWalkingStates(robot_states[state_index], robot_states[state_index + 1], start_trans, goal_trans, 0, M_PI * 0.5);

    doPlan("decomposed_body", req, res, robot_states[state_index], robot_states[state_index + 1], planning_scene,
        planner_instance);
  }
    break;

  case 6: // move hand
  {
    robot_states.push_back(planning_scene->getCurrentStateNonConst());
    robot_states.push_back(robot_states.back());
    robot_states.push_back(robot_states.back());
    robot_states.push_back(robot_states.back());
    setDrawerStates(robot_states[state_index], robot_states[state_index + 1], robot_states[state_index + 2],
        robot_states[state_index + 3]);

    doPlan("decomposed_body", req, res, robot_states[state_index], robot_states[state_index + 1], planning_scene,
        planner_instance);
  }
    break;

  case 7: // pull drawer
  {
    robot_states.push_back(planning_scene->getCurrentStateNonConst());
    robot_states.push_back(robot_states.back());
    robot_states.push_back(robot_states.back());
    robot_states.push_back(robot_states.back());
    setDrawerStates(robot_states[state_index], robot_states[state_index + 1], robot_states[state_index + 2],
        robot_states[state_index + 3]);

    ++state_index;

    doPlan("right_arm", req, res, robot_states[state_index], robot_states[state_index + 1], planning_scene,
        planner_instance);
  }
    break;

  case 8: // move hand
  {
    robot_states.push_back(planning_scene->getCurrentStateNonConst());
    robot_states.push_back(robot_states.back());
    robot_states.push_back(robot_states.back());
    robot_states.push_back(robot_states.back());
    setDrawerStates(robot_states[state_index], robot_states[state_index + 1], robot_states[state_index + 2],
        robot_states[state_index + 3]);

    state_index += 2;

    doPlan("decomposed_body", req, res, robot_states[state_index], robot_states[state_index + 1], planning_scene,
        planner_instance);
  }
    break;

    // test state
  case 9: // walking
   {
	   robot_states.push_back(planning_scene->getCurrentStateNonConst());
	   robot_states.push_back(robot_states.back());
	   Eigen::Vector3d start_trans(3.0, 4.0, 0);
	   Eigen::Vector3d goal_trans(3.0, 9.0, 0);
	   setWalkingStates(robot_states[state_index], robot_states[state_index + 1], start_trans, goal_trans);
	   doPlan("lower_body", req, res, robot_states[state_index], robot_states[state_index + 1], planning_scene,
		   planner_instance);
	   renderHierarchicalTrajectory(res.trajectory_, node_handle, robot_model);
   }
     break;
  }

  displayStates(robot_states[state_index], robot_states[state_index + 1], node_handle, robot_model);

  visualizeResult(res, node_handle, 0, 1.0);
  //renderHierarchicalTrajectory(res.trajectory_, node_handle, robot_model);

  ROS_INFO("Done");
  planner_instance.reset();

  return 0;
}
