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
		robot_state::RobotStatePtr& robot_state, bool is_best)
{
	if (!is_best)
		return;

	geometry_msgs::Point point;

	visualization_msgs::MarkerArray ma;
	std::vector<std::string> link_names =
			robot_model_->getMoveitRobotModel()->getLinkModelNames();
	std_msgs::ColorRGBA color = colors_[WHITE];
	color.a = 0.1;
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
		const FullTrajectoryConstPtr& full_trajectory, bool is_best,
		bool keyframe_only)
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

	for (int point = begin; point < end; point += step)
	{
		const Eigen::VectorXd& r = full_trajectory->getComponentTrajectory(
				FullTrajectory::TRAJECTORY_COMPONENT_CONTACT_POSITION,
				Trajectory::TRAJECTORY_TYPE_POSITION).row(point);

		const Eigen::VectorXd& f = full_trajectory->getComponentTrajectory(
				FullTrajectory::TRAJECTORY_COMPONENT_CONTACT_FORCE,
				Trajectory::TRAJECTORY_TYPE_POSITION).row(point);

		int num_contacts = r.rows() / 3;

		for (int i = 0; i < num_contacts; ++i)
		{
			Eigen::Vector3d contact_position;
			Eigen::Vector3d contact_normal;
			GroundManager::getInstance()->getNearestGroundPosition(
					r.block(3 * i, 0, 3, 1), contact_position, contact_normal);
			Eigen::Vector3d contact_force = full_trajectory->getContactForce(
					point, i);

			// test
			int foot_index = i / 4 * 4;
			int ee_index = i % 4;
			contact_position = r.block(3 * foot_index, 0, 3, 1);

			double contact_v = contact_position(2);
			contact_v = 0.5 * tanh(4*contact_v-2) + 0.5;



			contact_position(2) = 0;
			switch (ee_index)
			{
			case 0:
				contact_position(0) -= 0.05;
				contact_position(1) -= 0.05;
				break;
			case 1:
				contact_position(0) += 0.05;
				contact_position(1) -= 0.05;
				break;
			case 2:
				contact_position(0) += 0.05;
				contact_position(1) += 0.2;
				break;
			case 3:
				contact_position(0) -= 0.05;
				contact_position(1) += 0.2;
				break;
			}

			point_from.x = contact_position(0);
			point_from.y = contact_position(1);
			point_from.z = contact_position(2);

			point_to.x = contact_force(0) * 0.00001 + point_from.x;
			point_to.y = contact_force(1) * 0.00001 + point_from.y;
			point_to.z = contact_force(2) * 0.00001 + point_from.z;

			const double k1 = 0.01;//10.0;
					const double k2 = 3;//3.0;
			const double force_normal = std::max(0.0,
					contact_normal.dot(contact_force));
			double contact_variable = contact_v;//0.5 * std::tanh(k1 * force_normal - k2)+ 0.5;

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
	vis_marker_publisher_.publish(msg);
	vis_marker_publisher_.publish(msg2);
	vis_marker_publisher_.publish(msg3);
	vis_marker_publisher_.publish(msg4);
}

}
