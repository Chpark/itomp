#include <itomp_cio_planner/visualization/new_viz_manager.h>
#include <itomp_cio_planner/util/planning_parameters.h>
#include <itomp_cio_planner/contact/ground_manager.h>
#include <itomp_cio_planner/contact/contact_util.h>
#include <ros/node_handle.h>
#include <moveit_msgs/DisplayTrajectory.h>
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
    vis_marker_array_publisher_path_ = node_handle.advertise<visualization_msgs::MarkerArray>("itomp_planner/animate_path", 10);
    vis_marker_array_publisher_contacts_ = node_handle.advertise<visualization_msgs::MarkerArray>("itomp_planner/animate_contacts", 10);
    trajectory_publisher_ = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/itomp_planner/display_planned_path", 10);

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

void NewVizManager::setPlanningGroup(const ItompPlanningGroupConstPtr& planning_group)
{
	planning_group_ = planning_group;

    const multimap<string, string>& endeffector_names =	PlanningParameters::getInstance()->getGroupEndeffectorNames();
    std::pair<multimap<string, string>::const_iterator,	multimap<string, string>::const_iterator> ret =
        endeffector_names.equal_range(planning_group_->name_);
	endeffector_rbdl_indices_.clear();
    for (multimap<string, string>::const_iterator it = ret.first; it != ret.second; ++it)
	{
		string endeffector_name = it->second;
        unsigned int rbdl_link_index = robot_model_->getRBDLRobotModel().GetBodyId(endeffector_name.c_str());
		endeffector_rbdl_indices_.push_back(rbdl_link_index);
	}
}

void NewVizManager::animateEndeffectors(const ItompTrajectoryConstPtr& full_trajectory,
                                        const std::vector<RigidBodyDynamics::Model>& models, bool is_best)
{
    /*
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
    */
}

void NewVizManager::animatePath(const ItompTrajectoryConstPtr& trajectory,
								const robot_state::RobotStatePtr& robot_state, bool is_best)
{
	if (!is_best)
		return;

	visualization_msgs::MarkerArray ma;
    std::vector<std::string> link_names = robot_model_->getMoveitRobotModel()->getLinkModelNames();
    std_msgs::ColorRGBA color = colors_[WHITE];
    color.a = 0.1;
	ros::Duration dur(3600.0);

    for (unsigned int point = 0; point < trajectory->getNumPoints(); ++point)
	{
		ma.markers.clear();
        const Eigen::MatrixXd mat = trajectory->getElementTrajectory(
                                        ItompTrajectory::COMPONENT_TYPE_POSITION,
                                        ItompTrajectory::SUB_COMPONENT_TYPE_JOINT)->getTrajectoryPoint(point);
		robot_state->setVariablePositions(mat.data());
        std::string ns = "frame_" + boost::lexical_cast<std::string>(point);
		robot_state->getRobotMarkers(ma, link_names, color, ns, dur);
        for (int i = 0; i < ma.markers.size(); ++i)
            ma.markers[i].mesh_use_embedded_materials = true;
        vis_marker_array_publisher_path_.publish(ma);
	}
}

void NewVizManager::animateContacts(const ItompTrajectoryConstPtr& trajectory,
                                    const std::vector<std::vector<ContactVariables> >& contact_variables,
                                    const std::vector<RigidBodyDynamics::Model>& models,
                                    bool is_best)
{
    if (!is_best)
        return;

    const double SCALE_FORCE_LINE = 0.005;
    const double SCALE_DISPLACEMENT_LINE = 0.0025;
    const double SCALE_SPHERE = 0.03;
    geometry_msgs::Point point_from, point_to, point_body;

    int num_contacts = contact_variables[0].size();

    visualization_msgs::MarkerArray ma;
    visualization_msgs::Marker marker_cf;
    visualization_msgs::Marker marker_cp;
    visualization_msgs::Marker marker_displacement;

    marker_cf.header.frame_id = reference_frame_;
    marker_cf.header.stamp = ros::Time::now();
    marker_cf.action = visualization_msgs::Marker::ADD;
    marker_cf.pose.orientation.w = 1.0;
    marker_cf.type = visualization_msgs::Marker::LINE_LIST;
    marker_cf.scale.x = SCALE_FORCE_LINE;
    marker_cf.scale.y = SCALE_FORCE_LINE;
    marker_cf.scale.z = SCALE_FORCE_LINE;
    marker_cf.color = colors_[GREEN];

    marker_cp = marker_cf;
    marker_cp.type = visualization_msgs::Marker::SPHERE_LIST;
    marker_cp.scale.x = SCALE_SPHERE;
    marker_cp.scale.y = SCALE_SPHERE;
    marker_cp.scale.z = SCALE_SPHERE;

    marker_displacement = marker_cf;
    marker_displacement.scale.x = SCALE_DISPLACEMENT_LINE;
    marker_displacement.scale.y = SCALE_DISPLACEMENT_LINE;
    marker_displacement.scale.z = SCALE_DISPLACEMENT_LINE;
    marker_displacement.color = colors_[YELLOW];

    for (int point = 0; point < trajectory->getNumPoints(); ++point)
    {
        marker_cf.ns = "contact_" + boost::lexical_cast<std::string>(point);
        marker_cp.ns = marker_cf.ns;
        marker_displacement.ns = marker_cf.ns;
        for (int i = 0; i < num_contacts; ++i)
        {
            for (int c = 0; c < NUM_ENDEFFECTOR_CONTACT_POINTS; ++c)
            {
                marker_cf.points.clear();
                marker_cp.points.clear();
                marker_displacement.points.clear();
                marker_cf.id = i * NUM_ENDEFFECTOR_CONTACT_POINTS + c;
                marker_cp.id = marker_cf.id + NUM_ENDEFFECTOR_CONTACT_POINTS * num_contacts;
                marker_displacement.id = marker_cp.id + NUM_ENDEFFECTOR_CONTACT_POINTS * num_contacts;
                marker_cf.color = colors_[i + 1];
                marker_cp.color = marker_cf.color;

                const Eigen::Vector3d& point_position = contact_variables[point][i].projected_point_positions_[c];
                const Eigen::Vector3d& contact_force = contact_variables[point][i].getPointForce(c);

                point_from.x = point_position(0);
                point_from.y = point_position(1);
                point_from.z = point_position(2);

                point_to.x = contact_force(0) * 0.001 + point_from.x;
                point_to.y = contact_force(1) * 0.001 + point_from.y;
                point_to.z = contact_force(2) * 0.001 + point_from.z;

                marker_cf.points.push_back(point_from);
                marker_cf.points.push_back(point_to);
                marker_cp.points.push_back(point_from);

                double contact_active_value = getContactActiveValue(i, c, contact_variables[point]);
                contact_active_value = std::min(1.0, contact_active_value);
                marker_cf.color.r *= contact_active_value;
                marker_cf.color.g *= contact_active_value;
                marker_cf.color.b *= contact_active_value;
                marker_cp.color.r *= contact_active_value;
                marker_cp.color.g *= contact_active_value;
                marker_cp.color.b *= contact_active_value;

                ma.markers.push_back(marker_cf);
                ma.markers.push_back(marker_cp);

                /*
                int rbdl_contact_point_id = planning_group_->contact_points_[i].getContactPointRBDLIds(c);
                const RigidBodyDynamics::Math::SpatialTransform& contact_point_transform = models[point].X_base[rbdl_contact_point_id];
                const Eigen::Vector3d& body_point_position = contact_point_transform.r;

                point_body.x = body_point_position(0);
                point_body.y = body_point_position(1);
                point_body.z = body_point_position(2);

                marker_displacement.points.push_back(point_from);
                marker_displacement.points.push_back(point_body);

                marker_displacement.color = colors_[WHITE];
                marker_displacement.color.r *= 0.5 * contact_active_value;
                marker_displacement.color.g *= 0.5 * contact_active_value;
                marker_displacement.color.b *= 0.6 * contact_active_value;
                ma.markers.push_back(marker_displacement);
                */
            }
        }
    }
    vis_marker_array_publisher_contacts_.publish(ma);
}

void NewVizManager::displayTrajectory(const ItompTrajectoryConstPtr& trajectory)
{

    moveit_msgs::DisplayTrajectory display_trajectory;

    const ElementTrajectoryConstPtr& joint_trajectory = trajectory->getElementTrajectory(
                ItompTrajectory::COMPONENT_TYPE_POSITION, ItompTrajectory::SUB_COMPONENT_TYPE_JOINT);
    const ElementTrajectoryConstPtr& joint_velocity_trajectory = trajectory->getElementTrajectory(
                ItompTrajectory::COMPONENT_TYPE_VELOCITY, ItompTrajectory::SUB_COMPONENT_TYPE_JOINT);
    const ElementTrajectoryConstPtr& joint_acceleration_trajectory = trajectory->getElementTrajectory(
                ItompTrajectory::COMPONENT_TYPE_ACCELERATION, ItompTrajectory::SUB_COMPONENT_TYPE_JOINT);
    const Eigen::MatrixXd& joint_data = joint_trajectory->getData();
    const Eigen::MatrixXd& joint_vel_data = joint_velocity_trajectory->getData();
    const Eigen::MatrixXd& joint_acc_data = joint_acceleration_trajectory->getData();

    unsigned int num_joints = joint_trajectory->getNumElements();

    moveit_msgs::RobotTrajectory moveit_trajectory;

    moveit_trajectory.joint_trajectory.header.frame_id = reference_frame_;
    moveit_trajectory.joint_trajectory.header.stamp = ros::Time::now();
    moveit_trajectory.joint_trajectory.joint_names = robot_model_->getMoveitRobotModel()->getVariableNames();

    double dt = trajectory->getDiscretization();
    trajectory_msgs::JointTrajectoryPoint trajectory_point;
    trajectory_point.positions.resize(num_joints);
    trajectory_point.velocities.resize(num_joints);
    trajectory_point.effort.resize(num_joints);
    for (std::size_t i = 0; i < trajectory->getNumPoints(); ++i)
    {
        trajectory_point.time_from_start = ros::Duration(dt * i);
        for (std::size_t j = 0; j < num_joints; ++j)
        {
            trajectory_point.positions[j] = joint_data(i, j);
            trajectory_point.velocities[j] = joint_vel_data(i, j);
            trajectory_point.effort[j] = joint_acc_data(i, j);
        }
        moveit_trajectory.joint_trajectory.points.push_back(trajectory_point);
    }

    display_trajectory.trajectory.push_back(moveit_trajectory);
    trajectory_publisher_.publish(display_trajectory);

}

}
