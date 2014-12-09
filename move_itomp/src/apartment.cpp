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
#include <boost/variant/get.hpp>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shapes.h>

//file handling
#include <string>
#include <sstream>
#include <fstream>

const double INV_SQRT_2 = 1.0 / std::sqrt((long double) 2.0);

void loadStaticScene(ros::NodeHandle& node_handle,
		planning_scene::PlanningScenePtr& planning_scene,
		robot_model::RobotModelPtr& robot_model,
		ros::Publisher& planning_scene_diff_publisher)
{
	std::string environment_file;
	std::vector<double> environment_position;

	node_handle.param<std::string>("/itomp_planner/environment_model",
			environment_file, "");

	if (!environment_file.empty())
	{
		environment_position.resize(3, 0);
		if (node_handle.hasParam("/itomp_planner/environment_model_position"))
		{
			XmlRpc::XmlRpcValue segment;
			node_handle.getParam("/itomp_planner/environment_model_position",
					segment);
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
		ROS_INFO(
				"Env col pos : (%f %f %f)", environment_position[0], environment_position[1], environment_position[2]);
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

void renderStaticScene(ros::NodeHandle& node_handle,
		planning_scene::PlanningScenePtr& planning_scene,
		robot_model::RobotModelPtr& robot_model)
{
	std::string environment_file;
	std::vector<double> environment_position;

	static ros::Publisher vis_marker_array_publisher_ = node_handle.advertise<
			visualization_msgs::MarkerArray>("visualization_marker_array", 10);

	ros::WallDuration(1.0).sleep();

	node_handle.param<std::string>("/itomp_planner/environment_model",
			environment_file, "");

	if (!environment_file.empty())
	{
		environment_position.resize(3, 0);
		if (node_handle.hasParam("/itomp_planner/environment_model_position"))
		{
			XmlRpc::XmlRpcValue segment;
			node_handle.getParam("/itomp_planner/environment_model_position",
					segment);
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

		visualization_msgs::MarkerArray ma;
		visualization_msgs::Marker msg;
		msg.header.frame_id = robot_model->getModelFrame();
		msg.header.stamp = ros::Time::now();
		msg.ns = "environment";
		msg.type = visualization_msgs::Marker::MESH_RESOURCE;
		msg.action = visualization_msgs::Marker::ADD;
		msg.scale.x = 1.0;
		msg.scale.y = 1.0;
		msg.scale.z = 1.0;
		msg.id = 0;
		msg.pose.position.x = environment_position[0];
		msg.pose.position.y = environment_position[1];
		msg.pose.position.z = environment_position[2];
		ROS_INFO(
				"Env render pos : (%f %f %f)", environment_position[0], environment_position[1], environment_position[2]);
		msg.pose.orientation.x = 0.0;
		msg.pose.orientation.y = 0.0;
		msg.pose.orientation.z = 0.0;
		msg.pose.orientation.w = 1.0;
		msg.color.a = 1.0;
		msg.color.r = 0.5;
		msg.color.g = 0.5;
		msg.color.b = 0.5;
		msg.mesh_resource = environment_file;
		ma.markers.push_back(msg);

		ros::WallDuration(1.0).sleep();
		vis_marker_array_publisher_.publish(ma);
	}
}

void renderHierarchicalTrajectory(
		robot_trajectory::RobotTrajectoryPtr& robot_trajectory,
		ros::NodeHandle& node_handle, robot_model::RobotModelPtr& robot_model)
{
	static ros::Publisher vis_marker_array_publisher_ = node_handle.advertise<
			visualization_msgs::MarkerArray>("itomp_planner/trajectory", 10);
	visualization_msgs::MarkerArray ma;
	std::vector<std::string> link_names = robot_model->getLinkModelNames();
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
	std::map<std::string, std::vector<std::string> > group_links_map;

	group_links_map["lower_body"] = robot_model->getJointModelGroup(
			"lower_body")->getLinkModelNames();
	group_links_map["torso"] =
			robot_model->getJointModelGroup("torso")->getLinkModelNames();
	group_links_map["head"] =
			robot_model->getJointModelGroup("head")->getLinkModelNames();
	group_links_map["left_arm"] =
			robot_model->getJointModelGroup("left_arm")->getLinkModelNames();
	group_links_map["right_arm"] =
			robot_model->getJointModelGroup("right_arm")->getLinkModelNames();

	group_links_map["object"].clear();
	if (robot_model->hasLinkModel("right_hand_object_link"))
		group_links_map["object"].push_back("right_hand_object_link");

	int num_waypoints = robot_trajectory->getWayPointCount();
	for (int i = 0; i < num_waypoints; ++i)
	{
		ma.markers.clear();
		robot_state::RobotStatePtr state = robot_trajectory->getWayPointPtr(i);

		for (std::map<std::string, std::vector<std::string> >::iterator it =
				group_links_map.begin(); it != group_links_map.end(); ++it)
		{
			std::string ns = "robot_" + it->first;
			state->getRobotMarkers(ma, group_links_map[it->first],
					colorMap[it->first], ns, dur);
		}
		vis_marker_array_publisher_.publish(ma);

		double time = (i == 0 || i == num_waypoints - 1) ? 2.0 : 0.05;
		ros::WallDuration timer(time);
		timer.sleep();
	}
	for (int i = 0; i < 10; ++i)
	{
		ma.markers.clear();
		robot_state::RobotStatePtr state = robot_trajectory->getWayPointPtr(
				num_waypoints - 1);

		for (std::map<std::string, std::vector<std::string> >::iterator it =
				group_links_map.begin(); it != group_links_map.end(); ++it)
		{
			std::string ns = "robot_" + it->first;
			state->getRobotMarkers(ma, group_links_map[it->first],
					colorMap[it->first], ns, dur);
		}
		vis_marker_array_publisher_.publish(ma);

		double time = 0.05;
		ros::WallDuration timer(time);
		timer.sleep();
	}

}

void renderEnvironment(const std::string& environment_file,
		robot_model::RobotModelPtr& robot_model, const std::string& ns,
		std_msgs::ColorRGBA& color)
{
	return;

	ros::NodeHandle node_handle;
	ros::Publisher vis_marker_array_publisher_ = node_handle.advertise<
			visualization_msgs::MarkerArray>(
			"move_itomp/visualization_marker_array", 10);
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

void visualizeResult(planning_interface::MotionPlanResponse& res,
		ros::NodeHandle& node_handle, int repeat_last, double sleep_time)
{
	// Visualize the result
	// ^^^^^^^^^^^^^^^^^^^^
	static ros::Publisher display_publisher = node_handle.advertise<
			moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path",
			1, true);
	moveit_msgs::DisplayTrajectory display_trajectory;

	ROS_INFO("Visualizing the trajectory");
	moveit_msgs::MotionPlanResponse response;

	for (int i = 0; i < repeat_last; ++i)
		res.trajectory_->addSuffixWayPoint(res.trajectory_->getLastWayPoint(),
				5000);
	res.getMessage(response);

	display_trajectory.trajectory_start = response.trajectory_start;
	display_trajectory.trajectory.push_back(response.trajectory);
	display_publisher.publish(display_trajectory);
	ros::WallDuration timer(sleep_time);
	timer.sleep();
}

void doPlan(const std::string& group_name,
		planning_interface::MotionPlanRequest& req,
		planning_interface::MotionPlanResponse& res,
		robot_state::RobotState& start_state,
		robot_state::RobotState& goal_state,
		planning_scene::PlanningScenePtr& planning_scene,
		planning_interface::PlannerManagerPtr& planner_instance)
{
	const robot_state::JointModelGroup* joint_model_group =
			goal_state.getJointModelGroup("whole_body");

	// Copy from start_state to req.start_state
	unsigned int num_joints = start_state.getVariableCount();
	req.start_state.joint_state.name = start_state.getVariableNames();
	req.start_state.joint_state.position.resize(num_joints);
	req.start_state.joint_state.velocity.resize(num_joints);
	req.start_state.joint_state.effort.resize(num_joints);
	memcpy(&req.start_state.joint_state.position[0],
			start_state.getVariablePositions(), sizeof(double) * num_joints);
	if (start_state.hasVelocities())
		memcpy(&req.start_state.joint_state.velocity[0],
				start_state.getVariableVelocities(),
				sizeof(double) * num_joints);
	else
		memset(&req.start_state.joint_state.velocity[0], 0,
				sizeof(double) * num_joints);
	if (start_state.hasAccelerations())
		memcpy(&req.start_state.joint_state.effort[0],
				start_state.getVariableAccelerations(),
				sizeof(double) * num_joints);
	else
		memset(&req.start_state.joint_state.effort[0], 0,
				sizeof(double) * num_joints);

	req.group_name = group_name;
	moveit_msgs::Constraints joint_goal =
			kinematic_constraints::constructGoalConstraints(goal_state,
					joint_model_group);
	req.goal_constraints.push_back(joint_goal);

	// We now construct a planning context that encapsulate the scene,
	// the request and the response. We call the planner using this
	// planning context
	planning_interface::PlanningContextPtr context =
			planner_instance->getPlanningContext(planning_scene, req,
					res.error_code_);
	context->solve(res);
	if (res.error_code_.val != res.error_code_.SUCCESS)
	{
		ROS_ERROR("Could not compute plan successfully");
		exit(0);
	}
}

void displayStates(robot_state::RobotState& start_state,
		robot_state::RobotState& goal_state, ros::NodeHandle& node_handle,
		robot_model::RobotModelPtr& robot_model)
{
	static ros::Publisher start_state_display_publisher = node_handle.advertise<
			moveit_msgs::DisplayRobotState>("/move_itomp/display_start_state",
			1, true);
	static ros::Publisher goal_state_display_publisher = node_handle.advertise<
			moveit_msgs::DisplayRobotState>("/move_itomp/display_goal_state", 1,
			true);

	int num_variables = start_state.getVariableNames().size();

	moveit_msgs::DisplayRobotState disp_start_state;
	disp_start_state.state.joint_state.header.frame_id =
			robot_model->getModelFrame();
	disp_start_state.state.joint_state.name = start_state.getVariableNames();
	disp_start_state.state.joint_state.position.resize(num_variables);
	memcpy(&disp_start_state.state.joint_state.position[0],
			start_state.getVariablePositions(), sizeof(double) * num_variables);
	disp_start_state.highlight_links.clear();
	const std::vector<std::string>& link_model_names =
			robot_model->getLinkModelNames();
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
	disp_goal_state.state.joint_state.header.frame_id =
			robot_model->getModelFrame();
	disp_goal_state.state.joint_state.name = goal_state.getVariableNames();
	disp_goal_state.state.joint_state.position.resize(num_variables);
	memcpy(&disp_goal_state.state.joint_state.position[0],
			goal_state.getVariablePositions(), sizeof(double) * num_variables);
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

moveit_msgs::Constraints setRootJointConstraint(
        const std::vector<std::string>& hierarchy,
        const Eigen::VectorXd& transform)
{
	moveit_msgs::Constraints c;
    moveit_msgs::JointConstraint jc;
    moveit_msgs::OrientationConstraint oc;

    int id = 0;
    for(std::vector<std::string>::const_iterator cit= hierarchy.begin();
        cit != hierarchy.end(); ++cit, ++id)
    {
        jc.joint_name = *cit;
        jc.position = transform(id);
        c.joint_constraints.push_back(jc);
    }
	return c;
}

void setWalkingStates(robot_state::RobotState& start_state,
        robot_state::RobotState& goal_state, Eigen::VectorXd& start_transf,
        Eigen::VectorXd& goal_transf, const std::vector<std::string>& hierarchy)
{
	std::map<std::string, double> values;
	double jointValue = 0.0;

	const robot_state::JointModelGroup* joint_model_group =
			start_state.getJointModelGroup("whole_body");

	joint_model_group->getVariableDefaultPositions("standup", values);
	start_state.setVariablePositions(values);
    int id = 0;
    for(std::vector<std::string>::const_iterator cit = hierarchy.begin();
        cit != hierarchy.end(); ++cit, ++id)
    {
        jointValue = start_transf(id);
        start_state.setJointPositions(*cit, &jointValue);
    }

    goal_state = start_state;
    joint_model_group->getVariableDefaultPositions("standup", values);
    goal_state.setVariablePositions(values);

    id = 0;
    for(std::vector<std::string>::const_iterator cit = hierarchy.begin();
        cit != hierarchy.end(); ++cit, ++id)
    {
        jointValue = goal_transf(id);
        goal_state.setJointPositions(*cit, &jointValue);
    }
}

//load initial path
// file has following form (like a bvh)
// HIERARCHY
// Ordered list of all joints for which there are constraints
// MOTION
// Frames:	16
// Frame Time: 1
// "X Y Z Rx Ry Rz J1 J2... Jn" one line per state
std::vector<std::string> InitTrajectoryFromFile(std::vector<Eigen::VectorXd>& waypoints, const std::string& filepath)
{
    std::vector<std::string> res;
    bool hierarchy = false;
    bool motion = false;
    double offset [3] = {0,0,0};
    std::ifstream myfile (filepath.c_str());
    if (myfile.is_open())
    {
        std::string line;
        while (myfile.good())
        {
            getline(myfile, line);
            if(line.find("HIERARCHY") != std::string::npos)
            {
                hierarchy = true;
            }
            else if(line.find("OFFSET ") != std::string::npos)
            {
                hierarchy = false;
                line = line.substr(7);
                char *endptr;
                int h = 0;
                offset[h++] = strtod(line.c_str(), &endptr);
                for(; h< 3; ++h)
                {
                    double tg = strtod(endptr, &endptr);
                    offset[h] = tg; // strtod(endptr, &endptr);
                }
            }
            else if(line.find("MOTION") != std::string::npos)
            {
                 hierarchy = false;
            }
            else if(line.find("Frame Time") != std::string::npos)
            {
                 motion = true;
            }
            else if(!line.empty())
            {
                if(hierarchy)
                {
                    res.push_back(line);
                }
                else if(motion)
                {
                    Eigen::VectorXd waypoint(res.size());
                    char *endptr;
                    int h = 0;
                    waypoint[h++] = strtod(line.c_str(), &endptr);
                    for(; h< res.size(); ++h)
                    {
                        waypoint[h] = strtod(endptr, &endptr);
                    }
                    for(int i =0; i<3; ++i)
                    {
                        waypoint[i] += offset[i];
                    }
                    waypoint[2] -= 1.2;
                    waypoints.push_back(waypoint);
                }
            }
        }
        myfile.close();
    }
    else
    {
        std::cout << "can not find initial path file" << filepath << std::endl;
    }
    return res;
}

int main(int argc, char **argv)
{
	int motion = 0;
    std::string initialpath = "";
	if (argc >= 2)
	{
		motion = atoi(argv[1]);
        if(argc >=3)
        {
            initialpath = argv[2];
        }
	}
	printf("%d Motion %d\n", argc, motion);

	ros::init(argc, argv, "move_itomp");
	ros::AsyncSpinner spinner(1);
	spinner.start();
	ros::NodeHandle node_handle("~");

	robot_model_loader::RobotModelLoader robot_model_loader(
			"robot_description");
	robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();

	planning_scene::PlanningScenePtr planning_scene(
			new planning_scene::PlanningScene(robot_model));

	ros::Publisher planning_scene_diff_publisher;
	planning_scene_diff_publisher = node_handle.advertise<
			moveit_msgs::PlanningScene>("/planning_scene", 1);
	while (planning_scene_diff_publisher.getNumSubscribers() < 1)
	{
		ros::WallDuration sleep_t(0.5);
		sleep_t.sleep();
		ROS_INFO("Waiting planning_scene subscribers");
	}

	boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager> > planner_plugin_loader;
	planning_interface::PlannerManagerPtr planner_instance;
	std::string planner_plugin_name;

	if (!node_handle.getParam("planning_plugin", planner_plugin_name))
		ROS_FATAL_STREAM("Could not find planner plugin name");
	try
	{
		planner_plugin_loader.reset(
				new pluginlib::ClassLoader<planning_interface::PlannerManager>(
						"moveit_core", "planning_interface::PlannerManager"));
	} catch (pluginlib::PluginlibException& ex)
	{
		ROS_FATAL_STREAM(
				"Exception while creating planning plugin loader " << ex.what());
	}
	try
	{
		planner_instance.reset(
				planner_plugin_loader->createUnmanagedInstance(
						planner_plugin_name));
		if (!planner_instance->initialize(robot_model,
				node_handle.getNamespace()))
			ROS_FATAL_STREAM("Could not initialize planner instance");
		ROS_INFO_STREAM(
				"Using planning interface '" << planner_instance->getDescription() << "'");
	} catch (pluginlib::PluginlibException& ex)
	{
		const std::vector<std::string> &classes =
				planner_plugin_loader->getDeclaredClasses();
		std::stringstream ss;
		for (std::size_t i = 0; i < classes.size(); ++i)
			ss << classes[i] << " ";
		ROS_ERROR_STREAM(
				"Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl << "Available plugins: " << ss.str());
	}

	loadStaticScene(node_handle, planning_scene, robot_model, planning_scene_diff_publisher);

	/* Sleep a little to allow time to startup rviz, etc. */
	ros::WallDuration sleep_time(1.0);
	sleep_time.sleep();

	//renderStaticScene(node_handle, planning_scene, robot_model);

	// We will now create a motion plan request
	// specifying the desired pose of the end-effector as input.
	planning_interface::MotionPlanRequest req;
	planning_interface::MotionPlanResponse res;

	std::vector<robot_state::RobotState> robot_states;

	int state_index = 0;

	robot_states.push_back(planning_scene->getCurrentStateNonConst());
    robot_states.push_back(robot_states.back());
    Eigen::VectorXd start_trans, goal_trans;

	// set trajectory constraints
    std::vector<Eigen::VectorXd> waypoints;
    std::vector<std::string> hierarchy;
	// internal waypoints between start and goal

    if(initialpath.empty())
    {
        hierarchy.push_back("base_prismatic_joint_x");
        hierarchy.push_back("base_prismatic_joint_y");
        hierarchy.push_back("base_prismatic_joint_z");
        hierarchy.push_back("base_revolute_joint_z");
        hierarchy.push_back("base_revolute_joint_y");
        hierarchy.push_back("base_revolute_joint_x");
        Eigen::VectorXd vec1;
		start_trans = Eigen::VectorXd(6);
        start_trans = Eigen::VectorXd(6); start_trans << 0.0, 1.0, 0.0,0,0,0;
        goal_trans =  Eigen::VectorXd(6); goal_trans << 0.0, 2.5, 0.0,0,0,0;
		goal_trans << 0.0, 2.5, 0.0, 0, 0, 0;
        vec1 = Eigen::VectorXd(6); vec1 << 0.0, 1.5, 1.0,0,0,0;
		vec1 << 0.0, 1.5, 1.0, 0, 0, 0;
        waypoints.push_back(vec1);
        vec1 = Eigen::VectorXd(6); vec1 << 0.0, 2.0, 2.0,0,0,0;
		vec1 << 0.0, 2.0, 2.0, 0, 0, 0;
        waypoints.push_back(vec1);
        vec1 = Eigen::VectorXd(6); vec1 << 0.0, 2.5, 1.0,0,0,0;
		vec1 << 0.0, 2.5, 1.0, 0, 0, 0;
        waypoints.push_back(vec1);
    }
    else
    {
        hierarchy = InitTrajectoryFromFile(waypoints, initialpath);
        start_trans = waypoints.front();
        goal_trans =  waypoints.back();
    }
    setWalkingStates(robot_states[state_index], robot_states[state_index + 1],
            start_trans, goal_trans, hierarchy);
	for (int i = 0; i < waypoints.size(); ++i)
	{
		moveit_msgs::Constraints configuration_constraint =
                setRootJointConstraint(hierarchy, waypoints[i]);
        req.trajectory_constraints.constraints.push_back(
				configuration_constraint);
	}

	displayStates(robot_states[state_index], robot_states[state_index + 1],
				node_handle, robot_model);

	doPlan("whole_body", req, res, robot_states[state_index],
			robot_states[state_index + 1], planning_scene, planner_instance);


	visualizeResult(res, node_handle, 0, 1.0);

	ROS_INFO("Done");
	planner_instance.reset();

	return 0;
}
