/*
 * contactPoint.h
 *
 *  Created on: Sep 11, 2013
 *      Author: chpark
 */

#ifndef CONTACTPOINT_H_
#define CONTACTPOINT_H_

#include <itomp_cio_planner/common.h>
#include <itomp_cio_planner/util/vector_util.h>
#include <kdl/frames.hpp>
#include <moveit/planning_scene/planning_scene.h>

namespace itomp_cio_planner
{
class ItompRobotModel;

class ContactPoint
{
public:
	ContactPoint(const std::string& linkName, const ItompRobotModel* robot_model);
	virtual ~ContactPoint();

	void getPosition(int point, KDL::Vector& position, const std::vector<std::vector<KDL::Frame> >& segmentFrames) const;
	void getFrame(int point, KDL::Frame& frame, const std::vector<std::vector<KDL::Frame> >& segmentFrames) const;
	void updateContactViolationVector(int start, int end, double discretization,
			std::vector<Vector4d>& contactViolationVector,
			std::vector<KDL::Vector>& contactPointVelVector,
            const std::vector<std::vector<KDL::Frame> >& segmentFrames,
            const planning_scene::PlanningScenePtr& planning_scene) const;

    double getDistanceToGround(int point, const std::vector<std::vector<KDL::Frame> >& segmentFrames, const planning_scene::PlanningScenePtr& planning_scene) const;

	int getLinkSegmentNumber() const { return linkSegmentNumber_; }
	const std::string& getLinkName() const { return linkName_; }

private:
	std::string linkName_;
	int linkSegmentNumber_;
};

};


#endif /* CONTACTPOINT_H_ */
