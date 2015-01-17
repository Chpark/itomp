#ifndef TRAJECTORY_COST_H_
#define TRAJECTORY_COST_H_

#include <itomp_cio_planner/common.h>
#include <itomp_cio_planner/optimization/new_eval_manager.h>
#include <itomp_cio_planner/trajectory/full_trajectory.h>
#include <itomp_cio_planner/cost/trajectory_cost_helper.h>
#include <itomp_cio_planner/collision/collision_world_fcl_derivatives.h>
#include <itomp_cio_planner/collision/collision_robot_fcl_derivatives.h>

namespace itomp_cio_planner
{

class TrajectoryCost
{
public:
	TrajectoryCost(int index, std::string name, double weight);
	virtual ~TrajectoryCost();

	virtual void initialize(const NewEvalManager* evaluation_manager) {};

	virtual void preEvaluate(const NewEvalManager* evaluation_manager) {}
	virtual void postEvaluate(const NewEvalManager* evaluation_manager) {}
	virtual bool evaluate(const NewEvalManager* evaluation_manager, int point,
			double& cost) const = 0;
	virtual bool isInvariant(const NewEvalManager* evaluation_manager, int type, int element) const { return false; }

	int getIndex() const;
	const std::string& getName() const;
	double getWeight() const;

protected:
	int index_;
	std::string name_;
	double weight_;

};
ITOMP_DEFINE_SHARED_POINTERS(TrajectoryCost);

inline int TrajectoryCost::getIndex() const
{
	return index_;
}

inline const std::string& TrajectoryCost::getName() const
{
	return name_;
}

inline double TrajectoryCost::getWeight() const
{
	return weight_;
}

ITOMP_TRAJECTORY_COST_DECL(Smoothness)
//ITOMP_TRAJECTORY_COST_DECL(Obstacle)
ITOMP_TRAJECTORY_COST_DECL(Validity)
ITOMP_TRAJECTORY_COST_DECL(ContactInvariant)
ITOMP_TRAJECTORY_COST_DECL(PhysicsViolation)
ITOMP_TRAJECTORY_COST_DECL(GoalPose)
ITOMP_TRAJECTORY_COST_DECL(COM)
ITOMP_TRAJECTORY_COST_DECL(EndeffectorVelocity)
ITOMP_TRAJECTORY_COST_DECL(Torque)
ITOMP_TRAJECTORY_COST_DECL(RVO)
ITOMP_TRAJECTORY_COST_DECL(FTR)
ITOMP_TRAJECTORY_COST_DECL(ROM)
ITOMP_TRAJECTORY_COST_DECL(CartesianTrajectory)
ITOMP_TRAJECTORY_COST_DECL(Singularity)
ITOMP_TRAJECTORY_COST_DECL(FrictionCone)

class TrajectoryCostObstacle : public TrajectoryCost
{
	public:
		TrajectoryCostObstacle(int index, std::string name, double weight,
						  const NewEvalManager* evaluation_manager) : TrajectoryCost(index, name, weight)
		{
			initialize(evaluation_manager);
		}
		virtual ~TrajectoryCostObstacle() {}
		virtual void initialize(const NewEvalManager* evaluation_manager);
		virtual void preEvaluate(const NewEvalManager* evaluation_manager);
		virtual void postEvaluate(const NewEvalManager* evaluation_manager);
		virtual bool evaluate(const NewEvalManager* evaluation_manager,
								int point, double& cost) const;
		virtual bool isInvariant(const NewEvalManager* evaluation_manager, int type, int element) const;

	protected:
		std::vector<CollisionWorldFCLDerivativesPtr> collision_world_derivatives;
		std::vector<CollisionRobotFCLDerivativesPtr> collision_robot_derivatives;
};

}

#endif /* TRAJECTORY_COST_H_ */
