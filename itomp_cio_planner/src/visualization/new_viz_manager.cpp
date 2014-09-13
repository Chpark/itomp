#include <itomp_cio_planner/visualization/new_viz_manager.h>
#include <itomp_cio_planner/util/planning_parameters.h>
#include <visualization_msgs/MarkerArray.h>
#include <ros/node_handle.h>

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
	renderEnvironment();
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
		const std::vector<RigidBodyDynamics::Model>& models)
{
	const double scale_keyframe = 0.05;
	const double scale_line = 0.01;
	visualization_msgs::Marker::_color_type BLUE, GREEN;
	BLUE.a = 1.0;
	BLUE.r = 0.0;
	BLUE.g = 0.0;
	BLUE.b = 1.0;
	GREEN.a = 1.0;
	GREEN.r = 0.0;
	GREEN.g = 1.0;
	GREEN.b = 0.0;
	geometry_msgs::Point point;

	visualization_msgs::Marker msg;
	msg.header.frame_id = reference_frame_;
	msg.header.stamp = ros::Time::now();
	msg.ns = "itomp_endeffector";
	msg.action = visualization_msgs::Marker::ADD;
	msg.pose.orientation.w = 1.0;

	// render keyframe endeffectors
	msg.type = visualization_msgs::Marker::SPHERE_LIST;
	msg.scale.x = scale_keyframe;
	msg.scale.y = scale_keyframe;
	msg.scale.z = scale_keyframe;
	msg.points.resize(0);
	msg.color = BLUE;

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
	msg.color = GREEN;

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

}
