#ifndef NEW_VIZ_MANAGER_H_
#define NEW_VIZ_MANAGER_H_

#include <itomp_cio_planner/common.h>
#include <itomp_cio_planner/model/itomp_robot_model.h>
#include <itomp_cio_planner/model/itomp_planning_group.h>
#include <itomp_cio_planner/trajectory/itomp_trajectory.h>
#include <moveit/robot_state/robot_state.h>
#include <visualization_msgs/MarkerArray.h>
#include <ros/publisher.h>

namespace itomp_cio_planner
{
class NewVizManager: public Singleton<NewVizManager>
{
public:
	enum Colors
	{
		BLACK = 0, BLUE, GREEN, CYAN, RED, MAGENTA, YELLOW, WHITE,
	};
	NewVizManager();
	virtual ~NewVizManager();

	void initialize(const ItompRobotModelConstPtr& robot_model);
	void setPlanningGroup(const ItompPlanningGroupConstPtr& planning_group);

    void animateEndeffectors(const ItompTrajectoryConstPtr& trajectory,
							 const std::vector<RigidBodyDynamics::Model>& models, bool is_best);
    void animatePath(const ItompTrajectoryConstPtr& trajectory,
					 const robot_state::RobotStatePtr& robot_state, bool is_best);
    void animateContacts(const ItompTrajectoryConstPtr& trajectory,
                         const std::vector<std::vector<ContactVariables> >& contact_variables,
                         const std::vector<RigidBodyDynamics::Model>& models,
                         bool is_best);
    void animateInternalForces(const ItompTrajectoryConstPtr& trajectory, const std::vector<RigidBodyDynamics::Model>& models, bool forces, bool torques);

    void displayTrajectory(const ItompTrajectoryConstPtr& trajectory);
    void renderContactSurface();

	ros::Publisher& getVisualizationMarkerPublisher();
	ros::Publisher& getVisualizationMarkerArrayPublisher();

private:
    void setPointMarker(visualization_msgs::Marker& marker, unsigned int id, const Eigen::Vector3d& pos, const visualization_msgs::Marker::_color_type& color, double color_scale = 1.0);
    void setLineMarker(visualization_msgs::Marker& marker, unsigned int id, const Eigen::Vector3d& pos1, const Eigen::Vector3d& pos2, const visualization_msgs::Marker::_color_type& color, double color_scale = 1.0);

    ros::Publisher vis_marker_array_publisher_path_;
    ros::Publisher vis_marker_array_publisher_contacts_;
    ros::Publisher vis_marker_array_publisher_internal_forces_;
    ros::Publisher trajectory_publisher_;

	ItompRobotModelConstPtr robot_model_;
	ItompPlanningGroupConstPtr planning_group_;

	std::string reference_frame_;
	std::vector<unsigned int> endeffector_rbdl_indices_;
	std::vector<visualization_msgs::Marker::_color_type> colors_;
};

}

#endif /* NEW_VIZ_MANAGER_H_ */
