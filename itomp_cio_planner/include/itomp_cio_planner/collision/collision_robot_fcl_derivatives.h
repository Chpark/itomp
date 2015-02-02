/*
 * CollisionRobotFCLDerivatives.h
 *
 *  Created on: Dec 23, 2014
 *      Author: chonhyon
 */

#ifndef COLLISION_ROBOT_FCL_DERIVATIVES_H_
#define COLLISION_ROBOT_FCL_DERIVATIVES_H_

#include <itomp_cio_planner/common.h>
#include <moveit/collision_detection_fcl/collision_robot_fcl.h>

namespace itomp_cio_planner
{

class CollisionRobotFCLDerivatives : public collision_detection::CollisionRobotFCL
{
public:
	friend class CollisionWorldFCLDerivatives;

	CollisionRobotFCLDerivatives(const collision_detection::CollisionRobotFCL &other);
	void constructInternalFCLObject(const robot_state::RobotState &state);

	virtual void checkSelfCollision(const collision_detection::CollisionRequest &req, collision_detection::CollisionResult &res, const robot_state::RobotState &state) const;
	virtual void checkSelfCollision(const collision_detection::CollisionRequest &req, collision_detection::CollisionResult &res, const robot_state::RobotState &state, const collision_detection::AllowedCollisionMatrix &acm) const;
	virtual double distanceSelf(const robot_state::RobotState &state) const;
	virtual double distanceSelf(const robot_state::RobotState &state, const collision_detection::AllowedCollisionMatrix &acm) const;

	virtual void checkSelfCollision(const collision_detection::CollisionRequest &req, collision_detection::CollisionResult &res, const robot_state::RobotState &state1, const robot_state::RobotState &state2) const;
	virtual void checkSelfCollision(const collision_detection::CollisionRequest &req, collision_detection::CollisionResult &res, const robot_state::RobotState &state1, const robot_state::RobotState &state2, const collision_detection::AllowedCollisionMatrix &acm) const;
	virtual void checkOtherCollision(const collision_detection::CollisionRequest &req, collision_detection::CollisionResult &res, const robot_state::RobotState &state,
									 const collision_detection::CollisionRobot &other_robot, const robot_state::RobotState &other_state) const;
	virtual void checkOtherCollision(const collision_detection::CollisionRequest &req, collision_detection::CollisionResult &res, const robot_state::RobotState &state,
									 const collision_detection::CollisionRobot &other_robot, const robot_state::RobotState &other_state,
									 const collision_detection::AllowedCollisionMatrix &acm) const;
	virtual void checkOtherCollision(const collision_detection::CollisionRequest &req, collision_detection::CollisionResult &res, const robot_state::RobotState &state1, const robot_state::RobotState &state2,
									 const collision_detection::CollisionRobot &other_robot, const robot_state::RobotState &other_state1, const robot_state::RobotState &other_state2) const;
	virtual void checkOtherCollision(const collision_detection::CollisionRequest &req, collision_detection::CollisionResult &res, const robot_state::RobotState &state1, const robot_state::RobotState &state2,
									 const collision_detection::CollisionRobot &other_robot, const robot_state::RobotState &other_state1, const robot_state::RobotState &other_state2,
									 const collision_detection::AllowedCollisionMatrix &acm) const;
	virtual double distanceOther(const robot_state::RobotState &state,
								 const collision_detection::CollisionRobot &other_robot, const robot_state::RobotState &other_state) const;
	virtual double distanceOther(const robot_state::RobotState &state, const collision_detection::CollisionRobot &other_robot,
								 const robot_state::RobotState &other_state, const collision_detection::AllowedCollisionMatrix &acm) const;
protected:
	void checkSelfCollisionDerivativesHelper(const collision_detection::CollisionRequest &req, collision_detection::CollisionResult &res, const robot_state::RobotState &state, const collision_detection::AllowedCollisionMatrix *acm) const;
	double distanceSelfDerivativesHelper(const robot_state::RobotState &state, const collision_detection::AllowedCollisionMatrix *acm) const;

	static bool collisionCallback(fcl::CollisionObject *o1, fcl::CollisionObject *o2, void *data);
	static bool distanceCallback(fcl::CollisionObject* o1, fcl::CollisionObject* o2, void *data, double& min_dist);

	collision_detection::FCLObject fcl_obj_;
};
ITOMP_DEFINE_SHARED_POINTERS(CollisionRobotFCLDerivatives);

inline void CollisionRobotFCLDerivatives::checkSelfCollision(const collision_detection::CollisionRequest &req, collision_detection::CollisionResult &res, const robot_state::RobotState &state1, const robot_state::RobotState &state2) const
{
	logError("FCL continuous collision checking not yet implemented");
}
inline void CollisionRobotFCLDerivatives::checkSelfCollision(const collision_detection::CollisionRequest &req, collision_detection::CollisionResult &res, const robot_state::RobotState &state1, const robot_state::RobotState &state2, const collision_detection::AllowedCollisionMatrix &acm) const
{
	logError("FCL continuous collision checking not yet implemented");
}

inline void CollisionRobotFCLDerivatives::checkOtherCollision(const collision_detection::CollisionRequest &req, collision_detection::CollisionResult &res, const robot_state::RobotState &state,
		const collision_detection::CollisionRobot &other_robot, const robot_state::RobotState &other_state) const
{
	logError("CollisionRobotFCLDerivatives::checkOtherCollision should not be called.");
}
inline void CollisionRobotFCLDerivatives::checkOtherCollision(const collision_detection::CollisionRequest &req, collision_detection::CollisionResult &res, const robot_state::RobotState &state,
		const collision_detection::CollisionRobot &other_robot, const robot_state::RobotState &other_state,
		const collision_detection::AllowedCollisionMatrix &acm) const
{
	logError("CollisionRobotFCLDerivatives::checkOtherCollision should not be called.");
}
inline void CollisionRobotFCLDerivatives::checkOtherCollision(const collision_detection::CollisionRequest &req, collision_detection::CollisionResult &res, const robot_state::RobotState &state1, const robot_state::RobotState &state2,
		const collision_detection::CollisionRobot &other_robot, const robot_state::RobotState &other_state1, const robot_state::RobotState &other_state2) const
{
	logError("CollisionRobotFCLDerivatives::checkOtherCollision should not be called.");
}
inline void CollisionRobotFCLDerivatives::checkOtherCollision(const collision_detection::CollisionRequest &req, collision_detection::CollisionResult &res, const robot_state::RobotState &state1, const robot_state::RobotState &state2,
		const collision_detection::CollisionRobot &other_robot, const robot_state::RobotState &other_state1, const robot_state::RobotState &other_state2,
		const collision_detection::AllowedCollisionMatrix &acm) const
{
	logError("CollisionRobotFCLDerivatives::checkOtherCollision should not be called.");
}

inline double CollisionRobotFCLDerivatives::distanceOther(const robot_state::RobotState &state,
		const collision_detection::CollisionRobot &other_robot, const robot_state::RobotState &other_state) const
{
	logError("CollisionRobotFCLDerivatives::distanceOther should not be called.");
	return 0.0;
}
inline double CollisionRobotFCLDerivatives::distanceOther(const robot_state::RobotState &state, const collision_detection::CollisionRobot &other_robot,
		const robot_state::RobotState &other_state, const collision_detection::AllowedCollisionMatrix &acm) const
{
	logError("CollisionRobotFCLDerivatives::distanceOther should not be called.");
	return 0.0;
}

inline void CollisionRobotFCLDerivatives::constructInternalFCLObject(const robot_state::RobotState &state)
{
	fcl_obj_.clear();
	constructFCLObject(state, fcl_obj_);
}

}



#endif /* COLLISION_ROBOT_FCL_DERIVATIVES_H_ */
