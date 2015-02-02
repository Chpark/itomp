#ifndef NEW_VIZ_MANAGER_H_
#define NEW_VIZ_MANAGER_H_

#include <itomp_cio_planner/common.h>
#include <itomp_cio_planner/model/itomp_robot_model.h>
#include <itomp_cio_planner/model/itomp_planning_group.h>
#include <itomp_cio_planner/trajectory/full_trajectory.h>
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

	void renderOneTime();

	void animateEndeffectors(const FullTrajectoryConstPtr& full_trajectory,
							 const std::vector<RigidBodyDynamics::Model>& models, bool is_best);
	void animatePath(const FullTrajectoryConstPtr& full_trajectory,
					 const robot_state::RobotStatePtr& robot_state, bool is_best);
	void animateContactForces(const FullTrajectoryConstPtr& full_trajectory,
							  const std::vector<std::vector<ContactVariables> >& contact_variables,
							  bool is_best, bool keyframe_only = false);

	ros::Publisher& getVisualizationMarkerPublisher();
	ros::Publisher& getVisualizationMarkerArrayPublisher();

private:
	void renderEnvironment();
	void renderGround();

	ros::Publisher vis_marker_array_publisher_;
	ros::Publisher vis_marker_publisher_;

	ItompRobotModelConstPtr robot_model_;
	ItompPlanningGroupConstPtr planning_group_;

	std::string reference_frame_;
	std::vector<unsigned int> endeffector_rbdl_indices_;
	std::vector<visualization_msgs::Marker::_color_type> colors_;
};

inline ros::Publisher& NewVizManager::getVisualizationMarkerPublisher()
{
	return vis_marker_publisher_;
}

inline ros::Publisher& NewVizManager::getVisualizationMarkerArrayPublisher()
{
	return vis_marker_array_publisher_;
}

}

#endif /* NEW_VIZ_MANAGER_H_ */
