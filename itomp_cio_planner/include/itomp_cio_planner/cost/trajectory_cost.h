/*
 * trajectoryCost.h
 *
 *  Created on: Oct 23, 2013
 *      Author: cheonhyeonpark
 */

#ifndef TRAJECTORYCOST_H_
#define TRAJECTORYCOST_H_
#include <itomp_cio_planner/common.h>

namespace itomp_cio_planner
{
class EvaluationData;
class TrajectoryCost
{
public:
  enum COST_TYPE
  {
    COST_SMOOTHNESS = 0,
    COST_COLLISION,
    COST_VALIDITY,
    COST_CONTACT_INVARIANT,
    COST_PHYSICS_VIOLATION,
    COST_GOAL_POSE,
    COST_COM,
    COST_ENDEFFECTOR_VELOCITY,
    COST_TORQUE,
    COST_RVO,
    COST_FTR,
    COST_CARTESIAN_TRAJECTORY,
    COST_TYPES_NUM,
    COST_TYPE_INVALID = COST_TYPES_NUM,
  };

  TrajectoryCost(COST_TYPE type) :
      isHardConstraint_(false), type_(type)
  {
  }
  virtual ~TrajectoryCost()
  {
  }

  virtual void init(const EvaluationData* data);

  void compute(const EvaluationData* data, Eigen::VectorXd& costData, double& sum);
  virtual double getWaypointCost(int waypoint, const Eigen::VectorXd& costData) const
  {
    return costData(waypoint);
  }
  bool getIsHardConstraint() const
  {
    return isHardConstraint_;
  }
  COST_TYPE getType() const
  {
    return type_;
  }

  virtual double getWeight() const
  {
    return 0.0;
  }

  static boost::shared_ptr<TrajectoryCost> CreateTrajectoryCost(COST_TYPE type);

protected:
  virtual void doCompute(const EvaluationData* data, Eigen::VectorXd& costData) = 0;
  void computeCostSum(const EvaluationData* data, Eigen::VectorXd& costData, double& sum);

  bool isHardConstraint_;
  COST_TYPE type_;

};

typedef boost::shared_ptr<TrajectoryCost> TrajectoryCostPtr;

class TrajectorySmoothnessCost: public TrajectoryCost
{
public:
  TrajectorySmoothnessCost() :
      TrajectoryCost(COST_SMOOTHNESS)
  {
  }
  virtual ~TrajectorySmoothnessCost()
  {
  }

  virtual double getWaypointCost(int waypoint, const Eigen::VectorXd& costData) const
  {
    return 0.0;
  }
  virtual double getWeight() const;

protected:
  virtual void doCompute(const EvaluationData* data, Eigen::VectorXd& costData);
};

class TrajectoryCollisionCost: public TrajectoryCost
{
public:
  TrajectoryCollisionCost() :
      TrajectoryCost(COST_COLLISION)
  {
  }
  virtual ~TrajectoryCollisionCost()
  {
  }

  virtual double getWeight() const;

protected:
  virtual void doCompute(const EvaluationData* data, Eigen::VectorXd& costData);
};

class TrajectoryValidityCost: public TrajectoryCost
{
public:
  TrajectoryValidityCost() :
      TrajectoryCost(COST_VALIDITY)
  {
  }
  virtual ~TrajectoryValidityCost()
  {
  }

  virtual double getWeight() const;

protected:
  virtual void doCompute(const EvaluationData* data, Eigen::VectorXd& costData);
};

class TrajectoryContactInvariantCost: public TrajectoryCost
{
public:
  TrajectoryContactInvariantCost() :
      TrajectoryCost(COST_CONTACT_INVARIANT)
  {
  }
  virtual ~TrajectoryContactInvariantCost()
  {
  }

  virtual double getWeight() const;

protected:
  virtual void doCompute(const EvaluationData* data, Eigen::VectorXd& costData);
};

class TrajectoryPhysicsViolationCost: public TrajectoryCost
{
public:
  TrajectoryPhysicsViolationCost() :
      TrajectoryCost(COST_PHYSICS_VIOLATION)
  {
  }
  virtual ~TrajectoryPhysicsViolationCost()
  {
  }

  virtual double getWeight() const;

protected:
  virtual void doCompute(const EvaluationData* data, Eigen::VectorXd& costData);
};

class TrajectoryGoalPoseCost: public TrajectoryCost
{
public:
  TrajectoryGoalPoseCost() :
      TrajectoryCost(COST_GOAL_POSE)
  {
  }
  virtual ~TrajectoryGoalPoseCost()
  {
  }

  virtual double getWeight() const;

protected:
  virtual void doCompute(const EvaluationData* data, Eigen::VectorXd& costData);
};

class TrajectoryCoMCost: public TrajectoryCost
{
public:
  TrajectoryCoMCost() :
      TrajectoryCost(COST_COM)
  {
  }
  virtual ~TrajectoryCoMCost()
  {
  }

  virtual double getWeight() const;

protected:
  virtual void doCompute(const EvaluationData* data, Eigen::VectorXd& costData);
};

class TrajectoryFTRCost: public TrajectoryCost
{
public:
  TrajectoryFTRCost() :
      TrajectoryCost(COST_FTR)
  {
  }
  virtual ~TrajectoryFTRCost()
  {
  }

  virtual double getWeight() const;

protected:
  virtual void doCompute(const EvaluationData* data, Eigen::VectorXd& costData);
};

class TrajectoryCartesianCost: public TrajectoryCost
{
public:
  TrajectoryCartesianCost() :
      TrajectoryCost(COST_CARTESIAN_TRAJECTORY)
  {
  }
  virtual ~TrajectoryCartesianCost()
  {
  }

  virtual double getWeight() const;

protected:
  virtual void doCompute(const EvaluationData* data, Eigen::VectorXd& costData);
};

}
;

#endif /* TRAJECTORYCOST_H_ */
