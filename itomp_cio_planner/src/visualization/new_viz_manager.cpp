#include <itomp_cio_planner/visualization/new_viz_manager.h>
#include <itomp_cio_planner/util/planning_parameters.h>
#include <visualization_msgs/MarkerArray.h>
#include <ros/node_handle.h>
#include <boost/lexical_cast.hpp>

using namespace std;

namespace itomp_cio_planner
{

NewVizManager::NewVizManager()
{
}

NewVizManager::~NewVizManager()
{
}

void NewVizManager::initialize(const ItompRobotModelConstPtr& robot_model)
{
	ros::NodeHandle node_handle;
	vis_marker_array_publisher_ = node_handle.advertise<
			visualization_msgs::MarkerArray>(
			"itomp_planner/visualization_marker_array", 10);
	vis_marker_publisher_ = node_handle.advertise<visualization_msgs::Marker>(
			"itomp_planner/visualization_marker", 10);

	robot_model_ = robot_model;
	reference_frame_ = robot_model->getReferenceFrame();
}

void NewVizManager::setPlanningGroup(
		const ItompPlanningGroupConstPtr& planning_group)
{
	planning_group_ = planning_group;

	const multimap<string, string>& endeffector_names =
			PlanningParameters::getInstance()->getAnimateEndeffectorSegment();
	std::pair<multimap<string, string>::const_iterator,
			multimap<string, string>::const_iterator> ret =
			endeffector_names.equal_range(planning_group_->name_);
	endeffector_rbdl_indices_.clear();
	for (multimap<string, string>::const_iterator it = ret.first;
			it != ret.second; ++it)
	{
		string endeffector_name = it->second;
		unsigned int rbdl_link_index =
				robot_model_->getRBDLRobotModel().GetBodyId(
						endeffector_name.c_str());
		endeffector_rbdl_indices_.push_back(rbdl_link_index);
	}
}

void NewVizManager::renderOneTime()
{
	//renderGround();
	//renderEnvironment(); // rendered in move_itomp
}

void NewVizManager::renderEnvironment()
{
	string environment_file =
			PlanningParameters::getInstance()->getEnvironmentModel();
	if (environment_file.empty())
		return;

	vector<double> environment_position =
			PlanningParameters::getInstance()->getEnvironmentModelPosition();
	double scale =
			PlanningParameters::getInstance()->getEnvironmentModelScale();
	environment_position.resize(3, 0);

	visualization_msgs::MarkerArray ma;
	visualization_msgs::Marker msg;
	msg.header.frame_id = reference_frame_;
	msg.header.stamp = ros::Time::now();
	msg.ns = "environment";
	msg.type = visualization_msgs::Marker::MESH_RESOURCE;
	msg.action = visualization_msgs::Marker::ADD;
	msg.scale.x = scale;
	msg.scale.y = scale;
	msg.scale.z = scale;
	msg.id = 0;
	msg.pose.position.x = environment_position[0];
	msg.pose.position.y = environment_position[1];
	msg.pose.position.z = environment_position[2];
	/*
	 msg.pose.orientation.x = sqrt(0.5);
	 msg.pose.orientation.y = 0.0;
	 msg.pose.orientation.z = 0.0;
	 msg.pose.orientation.w = sqrt(0.5);
	 */
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
	vis_marker_array_publisher_.publish(ma);
}

void NewVizManager::renderGround()
{
}

void NewVizManager::animateEndeffectors(
		const FullTrajectoryConstPtr& full_trajectory,
		const std::vector<RigidBodyDynamics::Model>& models, bool is_best)
{
	const double scale_keyframe = 0.05;
	const double scale_line = 0.01;
	visualization_msgs::Marker::_color_type BLUE, GREEN, MAGENTA, YELLOW;
	BLUE.a = 1.0;
	BLUE.r = 0.0;
	BLUE.g = 0.0;
	BLUE.b = 1.0;
	GREEN.a = 1.0;
	GREEN.r = 0.0;
	GREEN.g = 1.0;
	GREEN.b = 0.0;
	MAGENTA.a = 1.0;
	MAGENTA.r = 1.0;
	MAGENTA.g = 0.0;
	MAGENTA.b = 1.0;
	YELLOW.a = 1.0;
	YELLOW.r = 1.0;
	YELLOW.g = 1.0;
	YELLOW.b = 0.0;
	geometry_msgs::Point point;

	visualization_msgs::Marker msg;
	msg.header.frame_id = reference_frame_;
	msg.header.stamp = ros::Time::now();
	msg.ns = is_best ? "itomp_best_ee" : "itomp_ee";
	msg.action = visualization_msgs::Marker::ADD;
	msg.pose.orientation.w = 1.0;

	// render keyframe endeffectors
	msg.type = visualization_msgs::Marker::SPHERE_LIST;
	msg.scale.x = scale_keyframe;
	msg.scale.y = scale_keyframe;
	msg.scale.z = scale_keyframe;
	msg.points.resize(0);

	msg.color = is_best ? MAGENTA : BLUE;

	msg.id = 0;
	for (int i = full_trajectory->getKeyframeStartIndex();
			i < full_trajectory->getNumPoints();
			i += full_trajectory->getNumKeyframeIntervalPoints())
	{
		for (int j = 0; j < endeffector_rbdl_indices_.size(); ++j)
		{
			int rbdl_index = endeffector_rbdl_indices_[j];

			Eigen::Vector3d endeffector_pos = models[i].X_base[rbdl_index].r;
			point.x = endeffector_pos(0);
			point.y = endeffector_pos(1);
			point.z = endeffector_pos(2);
			msg.points.push_back(point);
		}
	}
	vis_marker_publisher_.publish(msg);

	// render endeffector path
	msg.type = visualization_msgs::Marker::LINE_STRIP;
	msg.scale.x = scale_line;
	msg.scale.y = scale_line;
	msg.scale.z = scale_line;
	msg.color = is_best ? YELLOW : GREEN;

	for (int j = 0; j < endeffector_rbdl_indices_.size(); ++j)
	{
		++msg.id;
		msg.points.resize(0);

		int rbdl_index = endeffector_rbdl_indices_[j];
		for (int i = 0; i < full_trajectory->getNumPoints(); ++i)
		{
			Eigen::Vector3d endeffector_pos = models[i].X_base[rbdl_index].r;
			point.x = endeffector_pos(0);
			point.y = endeffector_pos(1);
			point.z = endeffector_pos(2);
			msg.points.push_back(point);
		}

		vis_marker_publisher_.publish(msg);
	}
}

void NewVizManager::animatePath(const FullTrajectoryConstPtr& full_trajectory,
		robot_state::RobotStatePtr& robot_state, bool is_best)
{
	if (!is_best)
		return;

	visualization_msgs::Marker::_color_type WHITE;
	WHITE.a = 0.1;
	WHITE.r = 1.0;
	WHITE.g = 1.0;
	WHITE.b = 1.0;

	geometry_msgs::Point point;

	visualization_msgs::MarkerArray ma;
	std::vector<std::string> link_names =
			robot_model_->getMoveitRobotModel()->getLinkModelNames();
	std_msgs::ColorRGBA color = WHITE;
	ros::Duration dur(100.0);

	for (int point = full_trajectory->getKeyframeStartIndex();
			point < full_trajectory->getNumPoints();
			point += full_trajectory->getNumKeyframeIntervalPoints())
	{
		ma.markers.clear();
		const Eigen::MatrixXd mat = full_trajectory->getTrajectory(
				Trajectory::TRAJECTORY_TYPE_POSITION).row(point);
		robot_state->setVariablePositions(mat.data());
		std::string ns = "kf_" + boost::lexical_cast<std::string>(point);
		robot_state->getRobotMarkers(ma, link_names, color, ns, dur);
		vis_marker_array_publisher_.publish(ma);
	}
}

void NewVizManager::animateContactForces(
		const FullTrajectoryConstPtr& full_trajectory, bool is_best)
{
	if (!is_best)
		return;

	const double scale_line = 0.01;
	visualization_msgs::Marker::_color_type MAGENTA;
	MAGENTA.a = 1.0;
	MAGENTA.r = 1.0;
	MAGENTA.g = 0.0;
	MAGENTA.b = 1.0;
	geometry_msgs::Point point_from, point_to;

	const Eigen::MatrixXd& contact_positions =
			full_trajectory->getComponentTrajectory(
					FullTrajectory::TRAJECTORY_COMPONENT_CONTACT_POSITION);
	const Eigen::MatrixXd& contact_forces =
			full_trajectory->getComponentTrajectory(
					FullTrajectory::TRAJECTORY_COMPONENT_CONTACT_FORCE);

	int num_contacts = contact_positions.cols() / 3;

	visualization_msgs::Marker msg;
	msg.header.frame_id = reference_frame_;
	msg.header.stamp = ros::Time::now();
	msg.ns = "itomp_best_cf";
	msg.action = visualization_msgs::Marker::ADD;
	msg.pose.orientation.w = 1.0;

	msg.type = visualization_msgs::Marker::LINE_LIST;
	msg.scale.x = scale_line;
	msg.scale.y = scale_line;
	msg.scale.z = scale_line;
	msg.color = MAGENTA;

	msg.id = 0;
	msg.points.resize(0);

	for (int point = full_trajectory->getKeyframeStartIndex();
			point < full_trajectory->getNumPoints();
			point += full_trajectory->getNumKeyframeIntervalPoints())
	{
		for (int contact_index = 0; contact_index < num_contacts * 3;
				contact_index += 3)
		{
			point_from.x = contact_positions(point, contact_index);
			point_from.y = contact_positions(point, contact_index + 1);
			point_from.z = contact_positions(point, contact_index + 2);

			point_to.x = contact_forces(point, contact_index) + point_from.x;
			point_to.y = contact_forces(point, contact_index + 1)
					+ point_from.y;
			point_to.z = contact_forces(point, contact_index + 2)
					+ point_from.z;

			msg.points.push_back(point_from);
			msg.points.push_back(point_to);
		}
	}
	vis_marker_publisher_.publish(msg);
}

}
