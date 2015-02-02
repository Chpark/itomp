#include <itomp_cio_planner/visualization/new_viz_manager.h>
#include <itomp_cio_planner/util/planning_parameters.h>
#include <itomp_cio_planner/contact/ground_manager.h>
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

	colors_.resize(8);
	for (int i = 0; i < 8; ++i)
	{
		colors_[i].a = 1.0;
		colors_[i].b = (i % 2 == 0) ? 0.0 : 1.0;
		colors_[i].g = ((i / 2) % 2 == 0) ? 0.0 : 1.0;
		colors_[i].r = ((i / 4) % 2 == 0) ? 0.0 : 1.0;
	}
}

void NewVizManager::setPlanningGroup(
	const ItompPlanningGroupConstPtr& planning_group)
{
	planning_group_ = planning_group;

	const multimap<string, string>& endeffector_names =
		PlanningParameters::getInstance()->getGroupEndeffectorNames();
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
	const double scale_keyframe = 0.03;
	const double scale_line = 0.005;
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

	msg.color = is_best ? colors_[GREEN] : colors_[BLUE];

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
	msg.color = is_best ? colors_[GREEN] : colors_[BLUE];

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
								const robot_state::RobotStatePtr& robot_state, bool is_best)
{
	if (!is_best)
		return;

	geometry_msgs::Point point;

	visualization_msgs::MarkerArray ma;
	std::vector<std::string> link_names =
		robot_model_->getMoveitRobotModel()->getLinkModelNames();
	std_msgs::ColorRGBA color = colors_[WHITE];
	color.a = 0.1;
	ros::Duration dur(3600.0);

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
	const FullTrajectoryConstPtr& full_trajectory,
	const std::vector<std::vector<ContactVariables> >& contact_variables,
	bool is_best, bool keyframe_only)
{
	const double scale_line = 0.005;
	const double scale_keyframe = 0.03;
	geometry_msgs::Point point_from, point_to;

	visualization_msgs::Marker msg, msg2, msg3, msg4;
	msg.header.frame_id = reference_frame_;
	msg.header.stamp = ros::Time::now();
	msg.ns = is_best ? "itomp_best_cf" : "itomp_cf";
	msg.action = visualization_msgs::Marker::ADD;
	msg.pose.orientation.w = 1.0;

	msg.type = visualization_msgs::Marker::LINE_LIST;
	msg.scale.x = scale_line;
	msg.scale.y = scale_line;
	msg.scale.z = scale_line;
	msg.color = is_best ? colors_[YELLOW] : colors_[MAGENTA];

	msg.id = 0;
	msg.points.resize(0);

	msg2 = msg;
	msg2.ns = is_best ? "itomp_best_cp" : "itomp_cp";
	msg2.type = visualization_msgs::Marker::SPHERE_LIST;
	msg2.scale.x = scale_keyframe;
	msg2.scale.y = scale_keyframe;
	msg2.scale.z = scale_keyframe;

	// inactive
	msg3 = msg;
	msg3.id = 0;
	msg3.color = is_best ? colors_[RED] : colors_[MAGENTA];
	msg3.ns = is_best ? "itomp_best_cf_ia" : "itomp_cf_ia";
	msg4 = msg2;
	msg4.id = 0;
	msg4.color = is_best ? colors_[RED] : colors_[MAGENTA];
	msg4.ns = is_best ? "itomp_best_cp_ia" : "itomp_cp_ia";

	int begin, end, step;
	if (keyframe_only)
	{
		begin = full_trajectory->getKeyframeStartIndex();
		end = full_trajectory->getNumPoints();
		step = full_trajectory->getNumKeyframeIntervalPoints();
	}
	else
	{
		begin = 0;
		end = full_trajectory->getNumPoints();
		step = 1;
	}

	int num_contacts = contact_variables[0].size();
	for (int point = begin; point < end; point += step)
	{
		for (int i = 0; i < num_contacts; ++i)
		{
			double contact_v = contact_variables[point][i].getVariable();

			for (int c = 0; c < NUM_ENDEFFECTOR_CONTACT_POINTS; ++c)
			{
				Eigen::Vector3d point_position =
					contact_variables[point][i].projected_point_positions_[c];

				Eigen::Vector3d contact_force =
					contact_variables[point][i].getPointForce(c);
				contact_force *= contact_v;

				point_from.x = point_position(0);
				point_from.y = point_position(1);
				point_from.z = point_position(2);

				point_to.x = contact_force(0) * 0.001 + point_from.x;
				point_to.y = contact_force(1) * 0.001 + point_from.y;
				point_to.z = contact_force(2) * 0.001 + point_from.z;

				const double k1 = 0.01; //10.0;
				const double k2 = 3; //3.0;
				double contact_variable = contact_v;

				if (contact_variable > 0.0)
				{
					msg.points.push_back(point_from);
					msg.points.push_back(point_to);
					msg2.points.push_back(point_from);
				}
				else
				{
					msg3.points.push_back(point_from);
					msg3.points.push_back(point_to);
					msg4.points.push_back(point_from);
				}
			}

		}
	}
	vis_marker_publisher_.publish(msg);
	vis_marker_publisher_.publish(msg2);
	vis_marker_publisher_.publish(msg3);
	vis_marker_publisher_.publish(msg4);
}

}
