/*
 * CollisionWorldFCLDerivatives.h
 *
 *  Created on: Dec 23, 2014
 *      Author: chonhyon
 */

#ifndef COLLISION_WORLD_FCL_DERIVATIVES_H_
#define COLLISION_WORLD_FCL_DERIVATIVES_H_

#include <itomp_cio_planner/common.h>
#include <moveit/collision_detection_fcl/collision_world_fcl.h>

namespace itomp_cio_planner
{

class CollisionWorldFCLDerivatives : public collision_detection::CollisionWorldFCL
{
public:
	CollisionWorldFCLDerivatives(const collision_detection::CollisionWorldFCL &other, const collision_detection::WorldPtr& world);
	virtual ~CollisionWorldFCLDerivatives();

	virtual void checkRobotCollision(const collision_detection::CollisionRequest &req, collision_detection::CollisionResult &res, const collision_detection::CollisionRobot &robot, const robot_state::RobotState &state) const;
	virtual void checkRobotCollision(const collision_detection::CollisionRequest &req, collision_detection::CollisionResult &res, const collision_detection::CollisionRobot &robot, const robot_state::RobotState &state, const collision_detection::AllowedCollisionMatrix &acm) const;
	virtual double distanceRobot(const collision_detection::CollisionRobot &robot, const robot_state::RobotState &state) const;
	virtual double distanceRobot(const collision_detection::CollisionRobot &robot, const robot_state::RobotState &state, const collision_detection::AllowedCollisionMatrix &acm) const;

	virtual void checkRobotCollision(const collision_detection::CollisionRequest &req, collision_detection::CollisionResult &res, const collision_detection::CollisionRobot &robot, const robot_state::RobotState &state1, const robot_state::RobotState &state2) const;
	virtual void checkRobotCollision(const collision_detection::CollisionRequest &req, collision_detection::CollisionResult &res, const collision_detection::CollisionRobot &robot, const robot_state::RobotState &state1, const robot_state::RobotState &state2, const collision_detection::AllowedCollisionMatrix &acm) const;
	virtual void checkWorldCollision(const collision_detection::CollisionRequest &req, collision_detection::CollisionResult &res, const collision_detection::CollisionWorld &other_world) const;
	virtual void checkWorldCollision(const collision_detection::CollisionRequest &req, collision_detection::CollisionResult &res, const collision_detection::CollisionWorld &other_world, const collision_detection::AllowedCollisionMatrix &acm) const;
	virtual double distanceWorld(const collision_detection::CollisionWorld &world) const;
	virtual double distanceWorld(const collision_detection::CollisionWorld &world, const collision_detection::AllowedCollisionMatrix &acm) const;

protected:
	void checkRobotCollisionDerivativesHelper(const collision_detection::CollisionRequest &req, collision_detection::CollisionResult &res, const collision_detection::CollisionRobot &robot, const robot_state::RobotState &state, const collision_detection::AllowedCollisionMatrix *acm) const;
	double distanceRobotDerivativesHelper(const collision_detection::CollisionRobot &robot, const robot_state::RobotState &state, const collision_detection::AllowedCollisionMatrix *acm) const;

	static bool collisionCallback(fcl::CollisionObject *o1, fcl::CollisionObject *o2, void *data);
	static bool distanceCallback(fcl::CollisionObject* o1, fcl::CollisionObject* o2, void *data, double& min_dist);
};
ITOMP_DEFINE_SHARED_POINTERS(CollisionWorldFCLDerivatives);

inline void CollisionWorldFCLDerivatives::checkRobotCollision(const collision_detection::CollisionRequest &req, collision_detection::CollisionResult &res, const collision_detection::CollisionRobot &robot, const robot_state::RobotState &state1, const robot_state::RobotState &state2) const
{
	logError("FCL continuous collision checking not yet implemented");
}

inline void CollisionWorldFCLDerivatives::checkRobotCollision(const collision_detection::CollisionRequest &req, collision_detection::CollisionResult &res, const collision_detection::CollisionRobot &robot, const robot_state::RobotState &state1, const robot_state::RobotState &state2, const collision_detection::AllowedCollisionMatrix &acm) const
{
	logError("FCL continuous collision checking not yet implemented");
}

inline void CollisionWorldFCLDerivatives::checkWorldCollision(const collision_detection::CollisionRequest &req, collision_detection::CollisionResult &res, const collision_detection::CollisionWorld &other_world) const
{
	logError("CollisionWorldFCLDerivatives::checkWorldCollision should not be called.");
}

inline void CollisionWorldFCLDerivatives::checkWorldCollision(const collision_detection::CollisionRequest &req, collision_detection::CollisionResult &res, const collision_detection::CollisionWorld &other_world, const collision_detection::AllowedCollisionMatrix &acm) const
{
	logError("CollisionWorldFCLDerivatives::checkWorldCollision should not be called.");
}

inline double CollisionWorldFCLDerivatives::distanceWorld(const collision_detection::CollisionWorld &world) const
{
	logError("CollisionWorldFCLDerivatives::distanceWorld should not be called.");
	return 0.0;
}

inline double CollisionWorldFCLDerivatives::distanceWorld(const collision_detection::CollisionWorld &world, const collision_detection::AllowedCollisionMatrix &acm) const
{
	logError("CollisionWorldFCLDerivatives::distanceWorld should not be called.");
	return 0.0;
}

}



#endif /* COLLISION_WORLD_FCL_DERIVATIVES_H_ */
