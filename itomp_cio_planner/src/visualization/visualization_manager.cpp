/*
 * visualizationManager.cpp
 *
 *  Created on: Sep 11, 2013
 *      Author: cheonhyeonpark
 */

#include <itomp_cio_planner/visualization/visualization_manager.h>
#include <itomp_cio_planner/model/itomp_robot_model.h>
#include <itomp_cio_planner/util/planning_parameters.h>
#include <visualization_msgs/MarkerArray.h>
#include <ros/ros.h>

using namespace std;

namespace itomp_cio_planner
{

VisualizationManager::VisualizationManager()
{
}

VisualizationManager::~VisualizationManager()
{
}

void VisualizationManager::render()
{
	//renderGround();
	renderEnvironment();

}

void VisualizationManager::renderEnvironment()
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

void VisualizationManager::renderGround()
{
	//return;

	if (PlanningParameters::getInstance()->getTemporaryVariable(2) == 1.0)
	{
		// hrp4
		// stair

		visualization_msgs::MarkerArray ma;
		visualization_msgs::Marker::_color_type FLOOR_COLOR;
		visualization_msgs::Marker::_color_type STAIR_COLOR;
		visualization_msgs::Marker::_color_type STAIR_COLOR2;

		FLOOR_COLOR.a = 1.0;
		FLOOR_COLOR.r = 1.0;
		FLOOR_COLOR.g = 1.0;
		FLOOR_COLOR.b = 1.0;

		visualization_msgs::Marker msg;
		msg.header.frame_id = reference_frame_;
		msg.header.stamp = ros::Time::now();
		msg.ns = "floor";
		msg.type = visualization_msgs::Marker::CUBE;
		msg.action = visualization_msgs::Marker::ADD;

		msg.scale.x = 80.0;
		msg.scale.y = 80.0;
		msg.scale.z = 0.1;

		msg.points.resize(0);

		msg.color = FLOOR_COLOR;
		msg.id = 30000;

		msg.pose.position.z = -0.1;
		ma.markers.push_back(msg);

		msg.points.resize(0);
		STAIR_COLOR.a = 1.0;
		STAIR_COLOR.r = 0.5;
		STAIR_COLOR.g = 0.5;
		STAIR_COLOR.b = 0.8;
		STAIR_COLOR2.a = 1.0;
		STAIR_COLOR2.r = 0.5;
		STAIR_COLOR2.g = 0.8;
		STAIR_COLOR2.b = 0.5;

		msg.color = STAIR_COLOR;
		++msg.id;
		msg.ns = "floor1";
		msg.scale.x = 0.4;
		msg.scale.y = 40.0;
		msg.scale.z = 0.15;
		msg.pose.position.x = -0.8;
		msg.pose.position.z = 0.075;
		ma.markers.push_back(msg);

		msg.color = STAIR_COLOR2;
		++msg.id;
		msg.pose.position.x += 0.4;
		msg.pose.position.z += 0.075;
		msg.scale.z += 0.15;
		ma.markers.push_back(msg);

		msg.color = STAIR_COLOR;
		++msg.id;
		msg.pose.position.x += 0.4;
		msg.pose.position.z += 0.075;
		msg.scale.z += 0.15;
		ma.markers.push_back(msg);

		msg.color = STAIR_COLOR2;
		++msg.id;
		msg.pose.position.x += 0.4;
		msg.pose.position.z -= 0.075;
		msg.scale.z -= 0.15;
		ma.markers.push_back(msg);

		msg.color = STAIR_COLOR;
		++msg.id;
		msg.pose.position.x += 0.4;
		msg.pose.position.z -= 0.075;
		msg.scale.z -= 0.15;
		ma.markers.push_back(msg);

		vis_marker_array_publisher_.publish(ma);
	}
	/*
	 {
	 // human
	 // stair

	 visualization_msgs::MarkerArray ma;
	 visualization_msgs::Marker::_color_type FLOOR_COLOR;
	 visualization_msgs::Marker::_color_type STAIR_COLOR;
	 visualization_msgs::Marker::_color_type STAIR_COLOR2;

	 FLOOR_COLOR.a = 1.0;
	 FLOOR_COLOR.r = 0.5;
	 FLOOR_COLOR.g = 0.5;
	 FLOOR_COLOR.b = 0.5;

	 visualization_msgs::Marker msg;
	 msg.header.frame_id = reference_frame_;
	 msg.header.stamp = ros::Time::now();
	 msg.ns = "floor";
	 msg.type = visualization_msgs::Marker::CUBE;
	 msg.action = visualization_msgs::Marker::ADD;

	 msg.scale.x = 2.0;
	 msg.scale.y = 10.0;
	 msg.scale.z = 0.1;

	 msg.points.resize(0);

	 msg.color = FLOOR_COLOR;
	 msg.id = 30000;

	 msg.pose.position.z = -0.1;
	 ma.markers.push_back(msg);

	 msg.points.resize(0);
	 STAIR_COLOR.a = 1.0;
	 STAIR_COLOR.r = 0.5;
	 STAIR_COLOR.g = 0.5;
	 STAIR_COLOR.b = 0.8;
	 STAIR_COLOR2.a = 1.0;
	 STAIR_COLOR2.r = 0.5;
	 STAIR_COLOR2.g = 0.8;
	 STAIR_COLOR2.b = 0.5;

	 msg.color = STAIR_COLOR;
	 ++msg.id;
	 msg.ns = "floor1";
	 msg.scale.x = 2.0;
	 msg.scale.y = 0.5;
	 msg.scale.z = 0.2;
	 msg.pose.position.y = -1.0;
	 msg.pose.position.z = 0.1;
	 ma.markers.push_back(msg);

	 msg.color = STAIR_COLOR2;
	 ++msg.id;
	 msg.pose.position.y += 0.5;
	 msg.pose.position.z += 0.1;
	 msg.scale.z += 0.2;
	 ma.markers.push_back(msg);

	 msg.color = STAIR_COLOR;
	 ++msg.id;
	 msg.pose.position.y += 0.5;
	 msg.pose.position.z += 0.1;
	 msg.scale.z += 0.2;
	 ma.markers.push_back(msg);

	 msg.color = STAIR_COLOR2;
	 ++msg.id;
	 msg.pose.position.y += 0.5;
	 msg.pose.position.z -= 0.1;
	 msg.scale.z -= 0.2;
	 ma.markers.push_back(msg);

	 msg.color = STAIR_COLOR;
	 ++msg.id;
	 msg.pose.position.y += 0.5;
	 msg.pose.position.z -= 0.1;
	 msg.scale.z -= 0.2;
	 ma.markers.push_back(msg);

	 vis_marker_array_pub_->publish(ma);
	 }
	 */

	if (PlanningParameters::getInstance()->getTemporaryVariable(2) == 11.0)
	{
		// WAFR circle

		visualization_msgs::MarkerArray ma;
		visualization_msgs::Marker::_color_type FLOOR_COLOR;
		visualization_msgs::Marker::_color_type STAIR_COLOR;
		visualization_msgs::Marker::_color_type STAIR_COLOR2;

		FLOOR_COLOR.a = 1.0;
		FLOOR_COLOR.r = 1.0;
		FLOOR_COLOR.g = 1.0;
		FLOOR_COLOR.b = 1.0;

		visualization_msgs::Marker msg;
		msg.header.frame_id = reference_frame_;
		msg.header.stamp = ros::Time::now();
		msg.ns = "floor";
		msg.type = visualization_msgs::Marker::CUBE;
		msg.action = visualization_msgs::Marker::ADD;

		msg.scale.x = 80.0;
		msg.scale.y = 80.0;
		msg.scale.z = 0.1;

		msg.points.resize(0);

		msg.color = FLOOR_COLOR;
		msg.id = 30000;

		msg.pose.position.z = -0.1;
		ma.markers.push_back(msg);

		msg.points.resize(0);
		STAIR_COLOR.a = 1.0;
		STAIR_COLOR.r = 0.5;
		STAIR_COLOR.g = 0.5;
		STAIR_COLOR.b = 0.8;
		STAIR_COLOR2.a = 1.0;
		STAIR_COLOR2.r = 0.5;
		STAIR_COLOR2.g = 0.8;
		STAIR_COLOR2.b = 0.5;

		msg.scale.x = 80.0;

		msg.color = STAIR_COLOR;
		++msg.id;
		msg.ns = "floor1";
		msg.scale.y = 11.5;
		//		msg.scale.x = 5.0;
		msg.scale.z = 0.15;
		msg.pose.position.z = 0.075;
		ma.markers.push_back(msg);

		msg.color = STAIR_COLOR2;
		++msg.id;
		msg.pose.position.z += 0.075;
		msg.scale.y -= 4.0;
		msg.scale.z += 0.15;
		msg.scale.x -= 1.0;
		//msg.scale.x = 30.0;

		// hack
		msg.scale.y += 0.2;
		msg.pose.position.y -= 0.1;
		ma.markers.push_back(msg);
		msg.scale.y -= 0.2;
		msg.pose.position.y += 0.1;

		msg.color = STAIR_COLOR;
		++msg.id;
		msg.pose.position.z += 0.075;
		msg.scale.y -= 4.0;
		msg.scale.z += 0.15;
		msg.scale.x -= 1.0;
		//msg.scale.x = 20.0;
		ma.markers.push_back(msg);

		vis_marker_array_publisher_.publish(ma);
	}
	if (PlanningParameters::getInstance()->getTemporaryVariable(2) == 12.0)
	{
		// WAFR dynamic obstacles
		visualization_msgs::MarkerArray ma;
		visualization_msgs::Marker::_color_type ROAD_COLOR;
		visualization_msgs::Marker::_color_type SIDEWALK_COLOR;
		visualization_msgs::Marker::_color_type YELLOW_COLOR;
		visualization_msgs::Marker::_color_type WHITE_COLOR;

		WHITE_COLOR.a = 1.0;
		WHITE_COLOR.r = 1.0;
		WHITE_COLOR.g = 1.0;
		WHITE_COLOR.b = 1.0;

		ROAD_COLOR.a = 1.0;
		ROAD_COLOR.r = 0.1;
		ROAD_COLOR.g = 0.1;
		ROAD_COLOR.b = 0.1;

		SIDEWALK_COLOR.a = 1.0;
		SIDEWALK_COLOR.r = 0.8;
		SIDEWALK_COLOR.g = 0.6;
		SIDEWALK_COLOR.b = 0.6;

		YELLOW_COLOR.a = 1.0;
		YELLOW_COLOR.r = 1.0;
		YELLOW_COLOR.g = 1.0;
		YELLOW_COLOR.b = 0.0;

		visualization_msgs::Marker msg;
		msg.header.frame_id = reference_frame_;
		msg.header.stamp = ros::Time::now();
		msg.ns = "road";
		msg.type = visualization_msgs::Marker::CUBE;
		msg.action = visualization_msgs::Marker::ADD;

		msg.scale.x = 80.0;
		msg.scale.y = 8.0;
		msg.scale.z = 0.1;
		msg.color = ROAD_COLOR;
		msg.id = 0;
		msg.pose.position.z = -0.1;
		ma.markers.push_back(msg);

		msg.scale.x = 2.0;
		msg.scale.y = 80.0;
		msg.scale.z = 0.1;
		msg.pose.position.x = 3.0;
		msg.color = ROAD_COLOR;
		msg.id = 1;
		msg.pose.position.z = -0.1;
		ma.markers.push_back(msg);

		msg.scale.x = 42.0;
		msg.scale.y = 0.05;
		msg.scale.z = 0.1;
		msg.color = YELLOW_COLOR;
		msg.id = 2;
		msg.pose.position.z = -0.09;
		msg.pose.position.x = -19.0;
		ma.markers.push_back(msg);

		msg.scale.x = 36.0;
		msg.scale.y = 0.05;
		msg.scale.z = 0.1;
		msg.color = YELLOW_COLOR;
		msg.id = 3;
		msg.pose.position.z = -0.09;
		msg.pose.position.x = 22.0;
		ma.markers.push_back(msg);

		msg.scale.x = 42.0;
		msg.scale.y = 0.05;
		msg.scale.z = 0.1;
		msg.color = WHITE_COLOR;
		msg.id = 4;
		msg.pose.position.z = -0.09;
		msg.pose.position.x = -19.0;
		msg.pose.position.y = -2.0;
		ma.markers.push_back(msg);

		msg.scale.x = 36.0;
		msg.scale.y = 0.05;
		msg.scale.z = 0.1;
		msg.color = WHITE_COLOR;
		msg.id = 5;
		msg.pose.position.z = -0.09;
		msg.pose.position.x = 22.0;
		msg.pose.position.y = -2.0;
		ma.markers.push_back(msg);

		msg.scale.x = 42.0;
		msg.scale.y = 0.05;
		msg.scale.z = 0.1;
		msg.color = WHITE_COLOR;
		msg.id = 6;
		msg.pose.position.z = -0.09;
		msg.pose.position.x = -19.0;
		msg.pose.position.y = 2.0;
		ma.markers.push_back(msg);

		msg.scale.x = 36.0;
		msg.scale.y = 0.05;
		msg.scale.z = 0.1;
		msg.color = WHITE_COLOR;
		msg.id = 7;
		msg.pose.position.z = -0.09;
		msg.pose.position.x = 22.0;
		msg.pose.position.y = 2.0;
		ma.markers.push_back(msg);

		msg.scale.x = 42.0;
		msg.scale.y = 36.0;
		msg.scale.z = 0.15;
		msg.color = SIDEWALK_COLOR;
		msg.id = 8;
		msg.pose.position.x = -19.0;
		msg.pose.position.y = -22.0;
		msg.pose.position.z = 0.075;
		ma.markers.push_back(msg);

		msg.scale.x = 42.0;
		msg.scale.y = 36.0;
		msg.scale.z = 0.15;
		msg.color = SIDEWALK_COLOR;
		msg.id = 9;
		msg.pose.position.x = -19.0;
		msg.pose.position.y = 22.0;
		msg.pose.position.z = 0.075;
		ma.markers.push_back(msg);

		msg.scale.x = 36.0;
		msg.scale.y = 36.0;
		msg.scale.z = 0.15;
		msg.color = SIDEWALK_COLOR;
		msg.id = 10;
		msg.pose.position.x = 22.0;
		msg.pose.position.y = -22.0;
		msg.pose.position.z = 0.075;
		ma.markers.push_back(msg);

		msg.scale.x = 36.0;
		msg.scale.y = 36.0;
		msg.scale.z = 0.15;
		msg.color = SIDEWALK_COLOR;
		msg.id = 11;
		msg.pose.position.x = 22.0;
		msg.pose.position.y = 22.0;
		msg.pose.position.z = 0.075;
		ma.markers.push_back(msg);

		vis_marker_array_publisher_.publish(ma);
	}
	if (PlanningParameters::getInstance()->getTemporaryVariable(2) == 13.0)
	{
		// wafr narrow passage
		visualization_msgs::MarkerArray ma;

		visualization_msgs::Marker msg;
		msg.header.frame_id = reference_frame_;
		msg.header.stamp = ros::Time::now();
		msg.ns = "road";
		msg.type = visualization_msgs::Marker::CUBE;
		msg.action = visualization_msgs::Marker::ADD;
		visualization_msgs::Marker::_color_type WALL_COLOR;
		WALL_COLOR.a = 1.0;
		WALL_COLOR.r = 0.5;
		WALL_COLOR.g = 0.5;
		WALL_COLOR.b = 0.5;

		msg.scale.x = 0.2;
		msg.scale.y = 2.0;
		msg.scale.z = 1.2;
		msg.color = WALL_COLOR;
		msg.id = 0;
		msg.pose.position.x = -0.5;
		msg.pose.position.y = 0.0;
		msg.pose.position.z = 0.6;
		ma.markers.push_back(msg);

		msg.id = 1;
		msg.pose.position.x = 0.5;
		ma.markers.push_back(msg);

		msg.id = 2;
		msg.scale.x = 10.0 * sqrt(2.0);
		msg.scale.y = 0.2;
		msg.pose.position.x = -0.4 - 5.0 - 0.1 / sqrt(2.0);
		msg.pose.position.y = -1.0 - 5.0 + 0.1 / sqrt(2.0);
		msg.pose.orientation.x = 0.0;
		msg.pose.orientation.y = 0.0;
		msg.pose.orientation.z = sin(M_PI_4 * 0.5);
		msg.pose.orientation.w = cos(M_PI_4 * 0.5);
		ma.markers.push_back(msg);

		msg.id = 3;
		msg.scale.x = 10.0 * sqrt(2.0);
		msg.scale.y = 0.2;
		msg.pose.position.x = 0.4 + 5.0 + 0.1 / sqrt(2.0);
		msg.pose.position.y = -1.0 - 5.0 + 0.1 / sqrt(2.0);
		msg.pose.orientation.x = 0.0;
		msg.pose.orientation.y = 0.0;
		msg.pose.orientation.z = sin(-M_PI_4 * 0.5);
		msg.pose.orientation.w = cos(-M_PI_4 * 0.5);
		ma.markers.push_back(msg);

		vis_marker_array_publisher_.publish(ma);
	}

}

void VisualizationManager::initialize(
		const itomp_cio_planner::ItompRobotModel& robot_model)
{
	ros::NodeHandle node_handle;
	vis_marker_array_publisher_ = node_handle.advertise<
			visualization_msgs::MarkerArray>(
			"pomp_planner/visualization_marker_array", 10);
	vis_marker_publisher_ = node_handle.advertise<visualization_msgs::Marker>(
			"pomp_planner/visualization_marker", 10);

	reference_frame_ = robot_model.getReferenceFrame();

	robot_model_ = &robot_model;

	int num_traj = PlanningParameters::getInstance()->getNumTrajectories();
	robot_states_.resize(num_traj);
	for (int i = 0; i < num_traj; ++i)
		robot_states_[i].reset(
				new robot_state::RobotState(robot_model_->getRobotModel()));
}

void VisualizationManager::setPlanningGroup(
		const itomp_cio_planner::ItompRobotModel& robot_model,
		const std::string& groupName)
{
	const multimap<string, string>& endeffectorSegments =
			PlanningParameters::getInstance()->getAnimateEndeffectorSegment();

	multimap<string, string>::const_iterator it;
	for (it = endeffectorSegments.begin(); it != endeffectorSegments.end();
			++it)
	{
		if (it->first == groupName)
		{
			int segmentIndex =
					robot_model.getForwardKinematicsSolver()->segmentNameToIndex(
							it->second);
			if (segmentIndex == -1)
			{
				ROS_INFO(
						"Invalid endeffector segment name %s for %s", it->second.c_str(), it->first.c_str());
			}
			else
			{
				animate_endeffector_segment_numbers_.push_back(segmentIndex);
			}
		}
	}

	root_segment_number_ =
			robot_model.getForwardKinematicsSolver()->segmentNameToIndex(
					PlanningParameters::getInstance()->getLowerBodyRoot());
}

void VisualizationManager::animateEndeffector(int trajectory_index,
		int point_start, int point_end,
		const vector<vector<KDL::Frame> >& segmentFrames, bool best)
{
	const double trajectory_color_diff = 0.33;
	const double scale = 0.005;
	const int marker_step = 1;

	visualization_msgs::Marker::_color_type YELLOW, LIGHT_YELLOW;
	visualization_msgs::Marker::_color_type RED, LIGHT_RED;
	RED.a = 1.0;
	RED.r = 1.0;
	RED.g = 0.0;
	RED.b = 0.0;
	YELLOW.a = 1.0;
	YELLOW.r = 1.0;
	YELLOW.g = 1.0;
	YELLOW.b = 0.0;
	LIGHT_RED = RED;
	LIGHT_RED.g = 0.5;
	LIGHT_RED.b = 0.5;
	LIGHT_YELLOW = YELLOW;
	LIGHT_YELLOW.b = 0.5;

	visualization_msgs::Marker msg;
	msg.header.frame_id = reference_frame_;
	msg.header.stamp = ros::Time::now();
	msg.ns = "itomp_endeffector";
	msg.type = visualization_msgs::Marker::SPHERE_LIST;
	msg.action = visualization_msgs::Marker::ADD;

	msg.scale.x = scale;
	msg.scale.y = scale;
	msg.scale.z = scale;

	msg.points.resize(0);

	for (unsigned int index = 0;
			index < animate_endeffector_segment_numbers_.size(); ++index)
	{
		if (index != 0)
			break;

		msg.id = trajectory_index * animate_endeffector_segment_numbers_.size()
				+ index;

		msg.color = best ? (index == 0 ? YELLOW : LIGHT_YELLOW) : (index == 0 ? RED : LIGHT_RED);

		int sn = animate_endeffector_segment_numbers_[index];
		if (sn <= 0)
			continue;

		geometry_msgs::Point prevPt;

		for (int j = point_start; j < point_end; j += marker_step)
		{
			geometry_msgs::Point point;
			point.x = segmentFrames[j][sn].p.x();
			point.y = segmentFrames[j][sn].p.y();
			point.z = segmentFrames[j][sn].p.z();
			msg.points.push_back(point);
		}
		publish(msg);

	}
}

void VisualizationManager::animateRoot(int numFreeVars, int freeVarStartIndex,
		const std::vector<std::vector<KDL::Frame> >& segmentFrames, bool best)
{
	const double scale = 0.05;

	visualization_msgs::Marker::_color_type RED;

	RED.a = 1.0;
	RED.r = 0.8;
	RED.g = 0.6;
	RED.b = 0.6;

	visualization_msgs::Marker msg;
	msg.header.frame_id = reference_frame_;
	msg.header.stamp = ros::Time::now();
	msg.ns = "root_link";
	msg.type = visualization_msgs::Marker::SPHERE_LIST;
	msg.action = visualization_msgs::Marker::ADD;

	msg.scale.x = scale;
	msg.scale.y = scale;
	msg.scale.z = scale;

	msg.points.resize(0);

	msg.color = RED;
	msg.id = 12;

	if (best)
	{
		msg.ns = "root_link_best";
		msg.id = 13;
		msg.color.a = 1.0;
		msg.color.r = 1.0;
		msg.color.g = 1.0;
		msg.color.b = 0.6;
	}

	int sn = root_segment_number_;
	if (sn > 0)
	{
		int marker_step = 1;
		for (int i = 0; i < numFreeVars; i = i + marker_step)
		{
			int j = i + freeVarStartIndex;
			geometry_msgs::Point point;
			point.x = segmentFrames[j][sn].p.x();
			point.y = segmentFrames[j][sn].p.y();
			point.z = segmentFrames[j][sn].p.z();

			msg.points.push_back(point);

		}
		vis_marker_publisher_.publish(msg);
	}
}

void VisualizationManager::animateCoM(int numFreeVars, int freeVarStartIndex,
		const std::vector<KDL::Vector>& CoM, bool best)
{
	const double scale = 0.05;

	visualization_msgs::Marker::_color_type RED;

	RED.a = 1.0;
	RED.r = 1.0;
	RED.g = 0.5;
	RED.b = 0.5;

	visualization_msgs::Marker msg;
	msg.header.frame_id = reference_frame_;
	msg.header.stamp = ros::Time::now();
	msg.ns = "itomp_CoM";
	msg.type = visualization_msgs::Marker::SPHERE_LIST;
	msg.action = visualization_msgs::Marker::ADD;

	msg.scale.x = scale;
	msg.scale.y = scale;
	msg.scale.z = scale;

	msg.points.resize(0);

	msg.color = RED;
	msg.id = 11;

	if (best)
	{
		msg.ns = "CoM_best";
		msg.id = 14;
		/*
		 msg.color.a = 1.0;
		 msg.color.r = 1.0;
		 msg.color.g = 1.0;
		 msg.color.b = 0.6;
		 */
	}

	int marker_step = 1;

	geometry_msgs::Point prevPt;
	for (int i = 0; i < numFreeVars; i = i + marker_step)
	{
		int j = i + freeVarStartIndex;
		geometry_msgs::Point point;
		point.x = CoM[j].x();
		point.y = CoM[j].y();
		point.z = CoM[j].z();
		msg.points.push_back(point);
		if (i != 0)
		{
			geometry_msgs::Point diff;
			diff.x = point.x - prevPt.x;
			diff.y = point.y - prevPt.y;
			diff.z = point.z - prevPt.z;
			double dist = sqrt(
					diff.x * diff.x + diff.y * diff.y + diff.z * diff.z);
			int numNeeded = (int) (dist / scale * 2);
			for (int k = 0; k < numNeeded; ++k)
			{
				geometry_msgs::Point dummy;
				dummy.x = prevPt.x + diff.x * (k + 1) / numNeeded;
				dummy.y = prevPt.y + diff.y * (k + 1) / numNeeded;
				dummy.z = prevPt.z + diff.z * (k + 1) / numNeeded;
				msg.points.push_back(dummy);
			}
		}
		prevPt = point;
	}
	vis_marker_publisher_.publish(msg);
}

void VisualizationManager::animatePath(int trajectory_index,
		const ItompCIOTrajectory* traj, bool is_best)
{
	if (!is_best)
		return;

	std::vector<std::string> link_names =
			robot_model_->getRobotModel()->getJointModelGroup("lower_body")->getLinkModelNames();
	std::vector<std::string> link_names2 =
			robot_model_->getRobotModel()->getJointModelGroup("gripper")->getLinkModelNames();
	link_names.insert(link_names.end(), link_names2.begin(), link_names2.end());

	std_msgs::ColorRGBA WHITE, YELLOW, RED;
	WHITE.a = 1.0;
	WHITE.r = 1.0;
	WHITE.g = 1.0;
	WHITE.b = 1.0;
	YELLOW.a = 1.0;
	YELLOW.r = 0.0;
	YELLOW.g = 1.0;
	YELLOW.b = 1.0;
	RED.a = 0.5;
	RED.r = 1.0;
	RED.g = 0.0;
	RED.b = 0.0;
	ros::Duration dur(100.0);

	visualization_msgs::MarkerArray ma;
	for (int point = 0; point < traj->getNumPoints(); point += 10)
	{
		visualization_msgs::MarkerArray ma_point;
		const Eigen::MatrixXd& mat = traj->getTrajectoryPoint(point);
		robot_states_[trajectory_index]->setVariablePositions(mat.data());
		robot_states_[trajectory_index]->updateLinkTransforms();
		std::string ns = "wp_" + boost::lexical_cast<std::string>(point);
		robot_states_[trajectory_index]->getRobotMarkers(ma_point, link_names,
				WHITE, ns, dur);
		ma.markers.insert(ma.markers.end(), ma_point.markers.begin(),
				ma_point.markers.end());
	}
	int marker_size = ma.markers.size();
	/*
	 for (int i = 0; i < marker_size; ++i)
	 {
	 ma.markers[i].id += trajectory_index * marker_size;
	 }
	 */
	vis_marker_array_publisher_.publish(ma);

}

}
