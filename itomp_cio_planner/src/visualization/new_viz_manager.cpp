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

    // marker array
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


    // MotionPlanning -> Planned Path -> trajectory
    moveit_msgs::DisplayTrajectory display_trajectory;

    int num_all_joints = robot_state->getVariableCount();

    ElementTrajectoryConstPtr joint_trajectory = trajectory->getElementTrajectory(ItompTrajectory::COMPONENT_TYPE_POSITION,
                ItompTrajectory::SUB_COMPONENT_TYPE_JOINT);
    robot_trajectory::RobotTrajectoryPtr response_trajectory = boost::make_shared<robot_trajectory::RobotTrajectory>(robot_model_->getMoveitRobotModel(), "");

    robot_state::RobotState ks = *robot_state;
    std::vector<double> positions(num_all_joints);
    double dt = trajectory->getDiscretization();
    // TODO:
    int num_return_points = joint_trajectory->getNumPoints();
    for (std::size_t i = 0; i < num_return_points; ++i)
    {
        for (std::size_t j = 0; j < num_all_joints; j++)
        {
            positions[j] = (*joint_trajectory)(i, j);
        }

        ks.setVariablePositions(&positions[0]);
        // TODO: copy vel/acc
        ks.update();

        response_trajectory->addSuffixWayPoint(ks, dt);
    }

    moveit_msgs::RobotTrajectory trajectory_msg;
    response_trajectory->getRobotTrajectoryMsg(trajectory_msg);

    display_trajectory.trajectory.push_back(trajectory_msg);

    ros::NodeHandle node_handle;
    static ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/itomp_planner/display_planned_path", 1, true);
    display_publisher.publish(display_trajectory);
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

    int num_contacts = contact_variables[0].size();

    visualization_msgs::MarkerArray ma;
    visualization_msgs::Marker marker_cf;
    visualization_msgs::Marker marker_cp;
    visualization_msgs::Marker marker_cp_line;
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

    marker_cp_line = marker_cf;

    marker_displacement = marker_cf;
    marker_displacement.scale.x = SCALE_DISPLACEMENT_LINE;
    marker_displacement.scale.y = SCALE_DISPLACEMENT_LINE;
    marker_displacement.scale.z = SCALE_DISPLACEMENT_LINE;
    marker_displacement.color = colors_[YELLOW];

    for (int point = 0; point < trajectory->getNumPoints(); ++point)
    {
        int marker_id = 0;
        marker_cf.ns = "contact_" + boost::lexical_cast<std::string>(point);
        marker_cp.ns = marker_cf.ns;
        marker_cp_line.ns = marker_cf.ns;
        marker_displacement.ns = marker_cf.ns;
        for (int i = 0; i < num_contacts; ++i)
        {
            double max_contact_active_value = 0.0;
            for (int c = 0; c < NUM_ENDEFFECTOR_CONTACT_POINTS; ++c)
            {
                const Eigen::Vector3d& point_position = contact_variables[point][i].projected_point_positions_[c];
                const Eigen::Vector3d& contact_force = contact_variables[point][i].getPointForce(c);

                Eigen::Vector3d point_to = point_position + contact_force * 0.001;

                double contact_active_value = getContactActiveValue(i, c, contact_variables[point]);
                contact_active_value = std::min(1.0, contact_active_value);
                if (contact_active_value > max_contact_active_value)
                    max_contact_active_value = contact_active_value;

                setLineMarker(marker_cf, marker_id++, point_position, point_to, colors_[i + 1], contact_active_value);
                setPointMarker(marker_cp, marker_id++, point_position, colors_[i + 1], contact_active_value);

                ma.markers.push_back(marker_cf);
                ma.markers.push_back(marker_cp);
            }
            const Eigen::Vector3d& endeffector_position = contact_variables[point][i].projected_position_;
            setPointMarker(marker_cp, marker_id++, endeffector_position, colors_[i + 1], 0);
            ma.markers.push_back(marker_cp);

            for (int c = 0; c < NUM_ENDEFFECTOR_CONTACT_POINTS; ++c)
            {
                const Eigen::Vector3d& point_position = contact_variables[point][i].projected_point_positions_[c];
                const Eigen::Vector3d& next_point_position = contact_variables[point][i].projected_point_positions_[(c + 1) % NUM_ENDEFFECTOR_CONTACT_POINTS];
                setLineMarker(marker_cf, marker_id++, endeffector_position, point_position, colors_[i + 1], 0);
                ma.markers.push_back(marker_cf);
                setLineMarker(marker_cf, marker_id++, point_position, next_point_position, colors_[i + 1], 0);
                ma.markers.push_back(marker_cf);
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

void NewVizManager::renderContactSurface()
{
    string contact_file = PlanningParameters::getInstance()->getContactModel();
    if (contact_file.empty())
        return;

    vector<double> environment_position = PlanningParameters::getInstance()->getContactModelPosition();
    double scale = PlanningParameters::getInstance()->getContactModelScale();
    environment_position.resize(3, 0);

    visualization_msgs::MarkerArray ma;
    visualization_msgs::Marker msg;
    msg.header.frame_id = reference_frame_;
    msg.header.stamp = ros::Time::now();
    msg.ns = "contact";
    msg.type = visualization_msgs::Marker::MESH_RESOURCE;
    msg.action = visualization_msgs::Marker::ADD;
    msg.scale.x = scale;
    msg.scale.y = scale;
    msg.scale.z = scale;
    msg.id = 0;
    msg.pose.position.x = environment_position[0];
    msg.pose.position.y = environment_position[1];
    msg.pose.position.z = environment_position[2];
    msg.pose.orientation.x = 0.0;
    msg.pose.orientation.y = 0.0;
    msg.pose.orientation.z = 0.0;
    msg.pose.orientation.w = 1.0;
    msg.color.a = 1.0;
    msg.color.r = 0.5;
    msg.color.g = 0.5;
    msg.color.b = 0.5;
    msg.mesh_resource = contact_file;
    ma.markers.push_back(msg);
    vis_marker_array_publisher_contacts_.publish(ma);
}

void NewVizManager::setPointMarker(visualization_msgs::Marker& marker, unsigned int id, const Eigen::Vector3d& pos, const visualization_msgs::Marker::_color_type& color, double color_scale)
{
     marker.id = id;

    marker.points.clear();
    geometry_msgs::Point position;
    position.x = pos(0);
    position.y = pos(1);
    position.z = pos(2);
    marker.points.push_back(position);

    marker.color = color;
    marker.color.r *= color_scale;
    marker.color.g *= color_scale;
    marker.color.b *= color_scale;
}

void NewVizManager::setLineMarker(visualization_msgs::Marker& marker, unsigned int id, const Eigen::Vector3d& pos1, const Eigen::Vector3d& pos2, const visualization_msgs::Marker::_color_type& color, double color_scale)
{
    marker.id = id;

    marker.points.clear();
    geometry_msgs::Point position;
    position.x = pos1(0);
    position.y = pos1(1);
    position.z = pos1(2);
    marker.points.push_back(position);
    position.x = pos2(0);
    position.y = pos2(1);
    position.z = pos2(2);
    marker.points.push_back(position);

    marker.color = color;
    marker.color.r *= color_scale;
    marker.color.g *= color_scale;
    marker.color.b *= color_scale;
}

}
