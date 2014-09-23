/*
 * visualizationManager.h
 *
 *  Created on: Sep 11, 2013
 *      Author: cheonhyeonpark
 */

#ifndef VISUALIZATIONMANAGER_H_
#define VISUALIZATIONMANAGER_H_

#include <itomp_cio_planner/common.h>
#include <ros/publisher.h>
#include <kdl/frames.hpp>
#include <itomp_cio_planner/trajectory/itomp_cio_trajectory.h>
#include <itomp_cio_planner/util/singleton.h>

namespace itomp_cio_planner
{
class ItompRobotModel;

class VisualizationManager: public Singleton<VisualizationManager>
{
public:
	VisualizationManager();
	virtual ~VisualizationManager();

	void initialize(const ItompRobotModel& robot_model);
	void setPlanningGroup(const ItompRobotModel& robot_model,
			const std::string& groupName);

	void render();

	void animateEndeffector(int trajectory_index, int point_start,
			int point_end, const std::vector<std::vector<KDL::Frame> >& segmentFrames,
			bool best);

	void animateCoM(int numFreeVars, int freeVarStartIndex,
			const std::vector<KDL::Vector>& CoM, bool best);
	void animateRoot(int numFreeVars, int freeVarStartIndex,
			const std::vector<std::vector<KDL::Frame> >& segmentFrames,
			bool best);
	void animatePath(const ItompCIOTrajectory* traj);

	void clearCollisionPointMarkPositions()
	{
		collision_point_mark_positions_.clear();
	}
	void insertCollisionPointMarkPosition(const Eigen::Vector3d& pos)
	{
		collision_point_mark_positions_.push_back(pos);
	}
	void renderEnvironment();
	void renderGround();
	void clearAnimations()
	{
	}

	ros::Publisher& getVisualizationMarkerPublisher()
	{
		return vis_marker_publisher_;
	}
	ros::Publisher& getVisualizationMarkerArrayPublisher()
	{
		return vis_marker_array_publisher_;
	}

private:

	ros::Publisher vis_marker_array_publisher_;
	ros::Publisher vis_marker_publisher_;

	std::vector<int> animate_endeffector_segment_numbers_;
	int root_segment_number_;
	std::string reference_frame_;

	std::vector<Eigen::Vector3d> collision_point_mark_positions_;

	const itomp_cio_planner::ItompRobotModel* robot_model_;
	robot_state::RobotStatePtr robot_state_;
};
}
;

#endif /* VISUALIZATIONMANAGER_H_ */
