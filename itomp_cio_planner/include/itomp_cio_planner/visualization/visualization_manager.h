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
#include <itomp_cio_planner/util/singleton.h>

namespace itomp_cio_planner
{
ITOMP_FORWARD_DECL(ItompRobotModel);

class VisualizationManager: public Singleton<VisualizationManager>
{
public:
	VisualizationManager();
	virtual ~VisualizationManager();

	void initialize(const ItompRobotModelConstPtr& robot_model);
	void setPlanningGroup(const ItompRobotModelConstPtr& robot_model,
			const std::string& groupName);

	void render();

	void animateEndeffectors(const FullTrajectoryConstPtr& full_trajectory,
			const std::vector<RigidBodyDynamics::Model>& models);

	// TODO: replace below functions
	void animateEndeffector(int trajectory_index, int numFreeVars,
			int freeVarStartIndex,
			const std::vector<std::vector<KDL::Frame> >& segmentFrames,
			const std::vector<int>& stateValidity, bool best);
	void animateCoM(int numFreeVars, int freeVarStartIndex,
			const std::vector<KDL::Vector>& CoM, bool best);
	void animateRoot(int numFreeVars, int freeVarStartIndex,
			const std::vector<std::vector<KDL::Frame> >& segmentFrames,
			bool best);
	void animatePath(int free_vars_start, int free_vars_end);

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
	void visualizeState(int index);

	ros::Publisher vis_marker_array_publisher_;
	ros::Publisher vis_marker_publisher_;

	std::vector<int> animate_endeffector_segment_numbers_;
	int root_segment_number_;
	std::string reference_frame_;

	std::vector<Eigen::Vector3d> collision_point_mark_positions_;
};
}
;

#endif /* VISUALIZATIONMANAGER_H_ */
