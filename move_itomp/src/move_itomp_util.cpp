#include <move_itomp/move_itomp_util.h>
#include <boost/variant/get.hpp>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shapes.h>

namespace move_itomp_util
{

void initializePlanner(boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager> >& planner_plugin_loader,
                       planning_interface::PlannerManagerPtr& planner_instance,
                       ros::NodeHandle& node_handle,
                       robot_model::RobotModelPtr& robot_model)
{
    std::string planner_plugin_name;

    if (!node_handle.getParam("planning_plugin", planner_plugin_name))
        ROS_FATAL_STREAM("Could not find planner plugin name");
    try
    {
        planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>("moveit_core", "planning_interface::PlannerManager"));
    }
    catch (pluginlib::PluginlibException& ex)
    {
        ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
    }
    try
    {
        cpu_set_t mask;
        if (sched_getaffinity(0, sizeof(cpu_set_t), &mask) != 0)
            ROS_ERROR("sched_getaffinity failed");
        planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
        if (sched_setaffinity(0, sizeof(cpu_set_t), &mask) != 0)
            ROS_ERROR("sched_setaffinity failed");
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
        ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl << "Available plugins: " << ss.str());
    }
}

void loadStaticScene(ros::NodeHandle& node_handle,
                     planning_scene::PlanningScenePtr& planning_scene,
                     robot_model::RobotModelPtr& robot_model,
                     ros::Publisher& planning_scene_diff_publisher)
{
    std::string environment_file;
    std::vector<double> environment_position;

    node_handle.param<std::string>("/itomp_planner/environment_model", environment_file, "");

    if (!environment_file.empty())
    {
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
                    environment_position[i] = value;
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
        ROS_INFO("Env col pos : (%f %f %f)", environment_position[0], environment_position[1], environment_position[2]);
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

        planning_scene_diff_publisher.publish(planning_scene_msg);
    }
}

void doPlan(const std::string& group_name,
            planning_interface::MotionPlanRequest& req,
            planning_interface::MotionPlanResponse& res,
            robot_state::RobotState& start_state,
            robot_state::RobotState& goal_state,
            planning_scene::PlanningScenePtr& planning_scene,
            planning_interface::PlannerManagerPtr& planner_instance)
{
    const robot_state::JointModelGroup* joint_model_group = goal_state.getJointModelGroup("whole_body");

    // Copy from start_state to req.start_state
    unsigned int num_joints = start_state.getVariableCount();
    robot_state::robotStateToRobotStateMsg(start_state, req.start_state);

    req.group_name = group_name;
    moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);
    req.goal_constraints.push_back(joint_goal);

    // We now construct a planning context that encapsulate the scene,
    // the request and the response. We call the planner using this
    // planning context
    planning_interface::PlanningContextPtr context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
    context->solve(res);
    if (res.error_code_.val != res.error_code_.SUCCESS)
    {
        ROS_ERROR("Could not compute plan successfully");
        exit(0);
    }
}

void visualizeResult(planning_interface::MotionPlanResponse& res, ros::NodeHandle& node_handle, int repeat_last, double sleep_time)
{
    // Visualize the result
    // ^^^^^^^^^^^^^^^^^^^^
    static ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
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

void displayStates(robot_state::RobotState& start_state,
                   robot_state::RobotState& goal_state,
                   ros::NodeHandle& node_handle,
                   robot_model::RobotModelPtr& robot_model)
{
    static ros::Publisher start_state_display_publisher = node_handle.advertise<moveit_msgs::DisplayRobotState>("/move_itomp/display_start_state", 1, true);
    static ros::Publisher goal_state_display_publisher = node_handle.advertise<moveit_msgs::DisplayRobotState>("/move_itomp/display_goal_state", 1, true);

    int num_variables = start_state.getVariableNames().size();

    moveit_msgs::DisplayRobotState disp_start_state;
    disp_start_state.state.joint_state.header.frame_id = robot_model->getModelFrame();
    disp_start_state.state.joint_state.name = start_state.getVariableNames();
    disp_start_state.state.joint_state.position.resize(num_variables);
    memcpy(&disp_start_state.state.joint_state.position[0], start_state.getVariablePositions(), sizeof(double) * num_variables);
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
    memcpy(&disp_goal_state.state.joint_state.position[0], goal_state.getVariablePositions(), sizeof(double) * num_variables);
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

}
