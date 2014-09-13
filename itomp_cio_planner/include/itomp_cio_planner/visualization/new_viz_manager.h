#ifndef NEW_VIZ_MANAGER_H_
#define NEW_VIZ_MANAGER_H_

#include <itomp_cio_planner/common.h>
#include <itomp_cio_planner/model/itomp_robot_model.h>
#include <itomp_cio_planner/model/itomp_planning_group.h>
#include <itomp_cio_planner/trajectory/full_trajectory.h>
#include <ros/publisher.h>

namespace itomp_cio_planner
{
class NewVizManager: public Singleton<NewVizManager>
{
public:
	NewVizManager();
	virtual ~NewVizManager();

	void initialize(const ItompRobotModelConstPtr& robot_model);
	void setPlanningGroup(const ItompPlanningGroupConstPtr& planning_group);

	void renderOneTime();

	void animateEndeffectors(const FullTrajectoryConstPtr& full_trajectory,
			const std::vector<RigidBodyDynamics::Model>& models);

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
