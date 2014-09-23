// Original code from pr2_moveit_tutorials::motion_planning_api_tutorial.cpp
#include <pluginlib/class_loader.h>
#include <ros/ros.h>

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
#include <move_itomp_kuka/move_itomp.h>

using namespace std;

namespace move_itomp
{

MoveItomp::MoveItomp(const ros::NodeHandle& node_handle) :
		node_handle_(node_handle)
{

}

MoveItomp::~MoveItomp()
{

}

void MoveItomp::run(const std::string& group_name)
{
	group_name_ = group_name;

	robot_model_loader::RobotModelLoader robot_model_loader(
			"robot_description");
	robot_model_ = robot_model_loader.getModel();

	planning_scene_.reset(new planning_scene::PlanningScene(robot_model_));

	planning_scene_diff_publisher_ = node_handle_.advertise<
			moveit_msgs::PlanningScene>("/planning_scene", 1);
	while (planning_scene_diff_publisher_.getNumSubscribers() < 1)
	{
		ros::WallDuration sleep_t(0.5);
		sleep_t.sleep();
		ROS_INFO("Waiting planning_scene subscribers");
	}

	collision_detection::AllowedCollisionMatrix& acm =
			planning_scene_->getAllowedCollisionMatrixNonConst();
	acm.setEntry("environment", "segment_0", true);
	acm.setEntry("environment", "segment_1", true);

	boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager> > planner_plugin_loader;
	std::string planner_plugin_name;
	if (!node_handle_.getParam("planning_plugin", planner_plugin_name))
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
	planner_plugin_name = "ompl_interface/OMPLPlanner";
	try
	{
		planner_instance_.reset(
				planner_plugin_loader->createUnmanagedInstance(
						planner_plugin_name));
		if (!planner_instance_->initialize(robot_model_,
				node_handle_.getNamespace()))
			ROS_FATAL_STREAM("Could not initialize planner instance");
		ROS_INFO_STREAM(
				"Using planning interface '" << planner_instance_->getDescription() << "'");
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

	display_publisher_ = node_handle_.advertise<moveit_msgs::DisplayTrajectory>(
			"/move_group/display_planned_path", 1, true);
	vis_marker_array_publisher_ = node_handle_.advertise<
			visualization_msgs::MarkerArray>("visualization_marker_array", 10,
			true);

	loadStaticScene();

	ros::WallDuration sleep_time(0.01);
	sleep_time.sleep();

	///////////////////////////////////////////////////////

	moveit_msgs::DisplayTrajectory display_trajectory;
	moveit_msgs::MotionPlanResponse response;
	visualization_msgs::MarkerArray ma;
	planning_interface::MotionPlanRequest req;
	planning_interface::MotionPlanResponse res;

	req.workspace_parameters.min_corner.x =
			req.workspace_parameters.min_corner.y =
					req.workspace_parameters.min_corner.z = -10.0;
	req.workspace_parameters.max_corner.x =
			req.workspace_parameters.max_corner.y =
					req.workspace_parameters.max_corner.z = 10.0;

	// Set start_state
	robot_state::RobotState& start_state =
			planning_scene_->getCurrentStateNonConst();
	const robot_state::JointModelGroup* joint_model_group =
			start_state.getJointModelGroup(group_name_);

	std::map<std::string, double> values;
	joint_model_group->getVariableDefaultPositions("idle", values);
	start_state.setVariablePositions(values);
	start_state.update();

	// Setup a goal state
	robot_state::RobotState goal_state(start_state);
	joint_model_group->getVariableDefaultPositions("idle", values);
	goal_state.setVariablePositions(values);
	goal_state.update();

	const double INV_SQRT_2 = 1.0 / sqrt(2.0);

	const double EE_CONSTRAINTS[][7] =
	{
	{ 2, 0.5, 12, -0.5, -0.5, 0.5, 0.5 },
	{ 2, 2, 8.5 + 1.0, 0, -INV_SQRT_2, INV_SQRT_2, 0 },
	{ 2, 1.0, 12, -0.5, -0.5, 0.5, 0.5 },
	{ 1.5, 2, 8.5 + 1.0, 0, -INV_SQRT_2, INV_SQRT_2, 0 },
	{ 2, 1.5, 12, -0.5, -0.5, 0.5, 0.5 },
	{ 1, 2, 8.5 + 1.0, 0, -INV_SQRT_2, INV_SQRT_2, 0 }, };

	Eigen::Affine3d goal_transform[6];
	for (int i = 0; i < 6; ++i)
	{
		Eigen::Vector3d trans = Eigen::Vector3d(EE_CONSTRAINTS[i][0],
				EE_CONSTRAINTS[i][1], EE_CONSTRAINTS[i][2]);
		Eigen::Quaterniond rot = Eigen::Quaterniond(EE_CONSTRAINTS[i][6],
				EE_CONSTRAINTS[i][3], EE_CONSTRAINTS[i][4],
				EE_CONSTRAINTS[i][5]);

		goal_transform[i].linear() = rot.toRotationMatrix();
		goal_transform[i].translation() = trans;
	}

	// transform from tcp to arm end-effector
	Eigen::Affine3d transform_1_inv =
			robot_model_->getLinkModel("tcp_1_link")->getJointOriginTransform().inverse();
	Eigen::Affine3d transform_2_inv =
			robot_model_->getLinkModel("tcp_2_link")->getJointOriginTransform().inverse();

	for (int i = 0; i < 6; ++i)
	{
		goal_transform[i] = goal_transform[i]
				* ((i % 2 == 0) ? transform_1_inv : transform_2_inv);
	}

	start_state.update();
	ROS_ASSERT(isStateCollide(start_state) == false);

	std::vector<robot_state::RobotState> states(6, start_state);
	for (int i = 0; i < 6; ++i)
	{
		states[i].update();
		computeIKState(states[i], goal_transform[i]);
	}

	for (int i = 0; i < 6; ++i)
	{
		//vis_marker_array_publisher_.publish(ma);

		ROS_INFO("*** Planning Sequence %d ***", i);

		robot_state::RobotState& from_state = states[i];
		robot_state::RobotState& to_state = states[(i + 1) % 6];

		displayStates(node_handle_, from_state, to_state);
		sleep_time.sleep();

		const Eigen::Affine3d& transform = goal_transform[(i + 1) % 6];
		Eigen::Vector3d trans = transform.translation();
		Eigen::Quaterniond rot = Eigen::Quaterniond(transform.linear());

		geometry_msgs::PoseStamped goal_pose;
		goal_pose.header.frame_id = robot_model_->getModelFrame();
		goal_pose.pose.position.x = trans(0);
		goal_pose.pose.position.y = trans(1);
		goal_pose.pose.position.z = trans(2);
		goal_pose.pose.orientation.x = rot.x();
		goal_pose.pose.orientation.y = rot.y();
		goal_pose.pose.orientation.z = rot.z();
		goal_pose.pose.orientation.w = rot.w();
		std::string endeffector_name = "end_effector_link";

		plan(planner_instance_, planning_scene_, req, res, "lower_body",
				from_state, goal_pose, endeffector_name);

		res.getMessage(response);

		if (res.trajectory_->getWayPointCount() == 0)
		{
			--i;
			continue;
		}

		if (i == 0)
		{
			display_trajectory.trajectory_start = response.trajectory_start;
		}

		display_trajectory.trajectory.push_back(response.trajectory);
		//display_publisher.publish(display_trajectory);

		//from_state = to_state;

		// use the last configuration of prev trajectory
		if (i != 5)
		{
			int num_joints = from_state.getVariableCount();
			std::vector<double> positions(num_joints);
			const robot_state::RobotState& last_state =
					res.trajectory_->getLastWayPoint();
			to_state.setVariablePositions(last_state.getVariablePositions());
			to_state.update();
		}
	}

	// publish trajectory
	display_publisher_.publish(display_trajectory);

	sleep_time.sleep();
	ROS_INFO("Done");
	planner_instance_.reset();

	int num_joints =
			display_trajectory.trajectory[0].joint_trajectory.points[0].positions.size();
	int num_trajectories = display_trajectory.trajectory.size();
	for (int k = 0; k < num_joints; ++k)
	{
		std::cout
				<< display_trajectory.trajectory[0].joint_trajectory.joint_names[k]
				<< " ";
	}
	std::cout << std::endl;
	for (int i = 0; i < num_trajectories; ++i)
	{
		int num_points =
				display_trajectory.trajectory[i].joint_trajectory.points.size();
		for (int j = 0; j < num_points; ++j)
		{
			std::cout << "[" << j << "] ";
			for (int k = 0; k < num_joints; ++k)
			{
				double value =
						display_trajectory.trajectory[i].joint_trajectory.points[j].positions[k];
				std::cout << value << " ";
			}
			std::cout << std::endl;
		}
		std::cout << std::endl;
	}

}

bool MoveItomp::isStateSingular(
		planning_scene::PlanningScenePtr& planning_scene,
		const std::string& group_name, robot_state::RobotState& state)
{
	// check singularity
	Eigen::MatrixXd jacobianFull = (state.getJacobian(
			planning_scene_->getRobotModel()->getJointModelGroup(group_name)));
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobianFull);
	int rows = svd.singularValues().rows();
	double min_value = svd.singularValues()(rows - 1);

	const double threshold = 1e-3;
	if (min_value < threshold)
		return true;
	else
		return false;
}

void MoveItomp::plan(planning_interface::PlannerManagerPtr& planner_instance,
		planning_scene::PlanningScenePtr planning_scene,
		planning_interface::MotionPlanRequest& req,
		planning_interface::MotionPlanResponse& res,
		const std::string& group_name, robot_state::RobotState& start_state,
		robot_state::RobotState& goal_state)
{
	const robot_state::JointModelGroup* joint_model_group =
			start_state.getJointModelGroup(group_name);
	req.group_name = group_name;
	req.allowed_planning_time = 300.0;

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

	// goal state
	moveit_msgs::Constraints joint_goal =
			kinematic_constraints::constructGoalConstraints(goal_state,
					joint_model_group);
	req.goal_constraints.clear();
	req.goal_constraints.push_back(joint_goal);

	planning_interface::PlanningContextPtr context =
			planner_instance->getPlanningContext(planning_scene_, req,
					res.error_code_);
	context->solve(res);
	if (res.error_code_.val != res.error_code_.SUCCESS)
	{
		ROS_ERROR("Could not compute plan successfully");
		return;
	}
}

void MoveItomp::plan(planning_interface::PlannerManagerPtr& planner_instance,
		planning_scene::PlanningScenePtr planning_scene,
		planning_interface::MotionPlanRequest& req,
		planning_interface::MotionPlanResponse& res,
		const std::string& group_name, robot_state::RobotState& start_state,
		geometry_msgs::PoseStamped& goal_pose,
		const std::string& endeffector_link)
{
	const robot_state::JointModelGroup* joint_model_group =
			start_state.getJointModelGroup(group_name);
	req.group_name = group_name;
	req.allowed_planning_time = 300.0;

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

	planning_scene_->getCurrentStateNonConst().update();

	// goal state
	std::vector<double> tolerance_pose(3, 0.01);
	std::vector<double> tolerance_angle(3, 0.01);
	moveit_msgs::Constraints pose_goal =
			kinematic_constraints::constructGoalConstraints(endeffector_link,
					goal_pose, tolerance_pose, tolerance_angle);
	req.goal_constraints.clear();
	req.goal_constraints.push_back(pose_goal);

	planning_interface::PlanningContextPtr context =
			planner_instance->getPlanningContext(planning_scene_, req,
					res.error_code_);
	context->solve(res);
	if (res.error_code_.val != res.error_code_.SUCCESS)
	{
		ROS_ERROR("Could not compute plan successfully");
		return;
	}
}

void MoveItomp::loadStaticScene()
{
	moveit_msgs::PlanningScene planning_scene_msg;
	std::string environment_file;
	std::vector<double> environment_position;

	node_handle_.param<std::string>("/itomp_planner/environment_model",
			environment_file, "");

	if (!environment_file.empty())
	{
		double scale;
		node_handle_.param("/itomp_planner/environment_model_scale", scale,
				1.0);
		environment_position.resize(3, 0);
		if (node_handle_.hasParam("/itomp_planner/environment_model_position"))
		{
			XmlRpc::XmlRpcValue segment;
			node_handle_.getParam("/itomp_planner/environment_model_position",
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
		collision_object.header.frame_id = robot_model_->getModelFrame();
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
		//moveit_msgs::PlanningScene planning_scene_msg;
		planning_scene_msg.world.collision_objects.push_back(collision_object);
		planning_scene_msg.is_diff = true;
		planning_scene_->setPlanningSceneDiffMsg(planning_scene_msg);
	}

	planning_scene_diff_publisher_.publish(planning_scene_msg);
}

void MoveItomp::displayState(ros::NodeHandle& node_handle,
		robot_state::RobotState& state)
{
	std_msgs::ColorRGBA color;
	color.a = 0.5;
	color.r = 1.0;
	color.g = 0.5;
	color.b = 0.5;

	int num_variables = state.getVariableNames().size();
	static ros::Publisher state_display_publisher = node_handle_.advertise<
			moveit_msgs::DisplayRobotState>("/move_itomp/display_state", 1,
			true);
	moveit_msgs::DisplayRobotState disp_state;
	disp_state.state.joint_state.header.frame_id =
			robot_model_->getModelFrame();
	disp_state.state.joint_state.name = state.getVariableNames();
	disp_state.state.joint_state.position.resize(num_variables);
	memcpy(&disp_state.state.joint_state.position[0],
			state.getVariablePositions(), sizeof(double) * num_variables);
	disp_state.highlight_links.clear();
	const std::vector<std::string>& link_model_names =
			robot_model_->getLinkModelNames();
	for (unsigned int i = 0; i < link_model_names.size(); ++i)
	{
		moveit_msgs::ObjectColor obj_color;
		obj_color.id = link_model_names[i];
		obj_color.color = color;
		disp_state.highlight_links.push_back(obj_color);
	}
	state_display_publisher.publish(disp_state);
}

void MoveItomp::displayStates(ros::NodeHandle& node_handle,
		robot_state::RobotState& start_state,
		robot_state::RobotState& goal_state)
{
	// display start / goal states
	int num_variables = start_state.getVariableNames().size();
	static ros::Publisher start_state_display_publisher =
			node_handle_.advertise<moveit_msgs::DisplayRobotState>(
					"/move_itomp/display_start_state", 1, true);
	moveit_msgs::DisplayRobotState disp_start_state;
	disp_start_state.state.joint_state.header.frame_id =
			robot_model_->getModelFrame();
	disp_start_state.state.joint_state.name = start_state.getVariableNames();
	disp_start_state.state.joint_state.position.resize(num_variables);
	memcpy(&disp_start_state.state.joint_state.position[0],
			start_state.getVariablePositions(), sizeof(double) * num_variables);
	disp_start_state.highlight_links.clear();
	const std::vector<std::string>& link_model_names =
			robot_model_->getLinkModelNames();
	for (unsigned int i = 0; i < link_model_names.size(); ++i)
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

	static ros::Publisher goal_state_display_publisher = node_handle_.advertise<
			moveit_msgs::DisplayRobotState>("/move_itomp/display_goal_state", 1,
			true);
	moveit_msgs::DisplayRobotState disp_goal_state;
	disp_goal_state.state.joint_state.header.frame_id =
			robot_model_->getModelFrame();
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

bool MoveItomp::isStateCollide(const robot_state::RobotState& state)
{
	visualization_msgs::MarkerArray ma;
	visualization_msgs::Marker msg;
	msg.header.frame_id = robot_model_->getModelFrame();
	msg.header.stamp = ros::Time::now();
	msg.ns = "collision";
	msg.type = visualization_msgs::Marker::SPHERE_LIST;
	msg.action = visualization_msgs::Marker::ADD;
	msg.scale.x = 0.2;
	msg.scale.y = 0.2;
	msg.scale.z = 0.2;
	msg.color.a = 1.0;
	msg.color.r = 1.0;
	msg.color.g = 1.0;
	msg.color.b = 0.0;
	msg.id = 0;

	collision_detection::CollisionRequest collision_request;
	collision_detection::CollisionResult collision_result;
	collision_request.verbose = false;
	collision_request.contacts = true;

	planning_scene_->checkCollisionUnpadded(collision_request, collision_result,
			state);

	const collision_detection::CollisionResult::ContactMap& contact_map =
			collision_result.contacts;
	for (collision_detection::CollisionResult::ContactMap::const_iterator it =
			contact_map.begin(); it != contact_map.end(); ++it)
	{
		for (int i = 0; i < it->second.size(); ++i)
		{
			const collision_detection::Contact& contact = it->second[i];
			geometry_msgs::Point point;
			point.x = contact.pos(0);
			point.y = contact.pos(1);
			point.z = contact.pos(2);
			msg.points.push_back(point);
		}
	}
	ma.markers.push_back(msg);
	vis_marker_array_publisher_.publish(ma);
	ros::WallDuration sleep_time(0.01);
	sleep_time.sleep();

	return collision_result.collision;
}

void MoveItomp::computeIKState(robot_state::RobotState& ik_state,
		const Eigen::Affine3d& end_effector_state)
{

	ros::WallDuration sleep_time(0.01);

	// compute waypoint ik solutions

	const robot_state::JointModelGroup* joint_model_group =
			ik_state.getJointModelGroup(group_name_);

	int num_joints = ik_state.getVariableCount();

	kinematics::KinematicsQueryOptions options;
	options.return_approximate_solution = false;
	bool found_ik = false;

	robot_state::RobotState org_start(ik_state);
	int i = 0;
	while (true)
	{
		found_ik = ik_state.setFromIK(joint_model_group, end_effector_state, 10,
				0.1, moveit::core::GroupStateValidityCallbackFn(), options);
		ik_state.update();

		found_ik &= !isStateCollide(ik_state);

		if (found_ik && isStateSingular(planning_scene_, group_name_, ik_state))
			found_ik = false;

		double* pos = ik_state.getVariablePositions();
		printf("IK result :");
		for (int i = 0; i < ik_state.getVariableCount(); ++i)
			printf("%f ", pos[i]);
		printf("\n");
		if (found_ik)
			break;

		displayState(node_handle_, ik_state);
		sleep_time.sleep();

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

void MoveItomp::transform(Eigen::Affine3d& t, const Eigen::Affine3d& transform)
{
	Eigen::Matrix3d mat = t.rotation();

	Eigen::Matrix3d parent_mat = mat * transform.rotation().inverse();
	Eigen::Vector3d parent_trans = t.translation()
			- parent_mat * transform.translation();

	t.linear() = parent_mat;
	t.translation() = parent_trans;
}

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "move_itomp");
	ros::AsyncSpinner spinner(1);
	spinner.start();
	ros::NodeHandle node_handle("~");

	move_itomp::MoveItomp* move_itomp = new move_itomp::MoveItomp(node_handle);
	move_itomp->run("lower_body");
	delete move_itomp;

	return 0;
}
