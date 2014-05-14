#ifndef EVALUATION_DATA_H_
#define EVALUATION_DATA_H_

#include <itomp_cio_planner/common.h>
#include <itomp_cio_planner/model/itomp_robot_model.h>
#include <itomp_cio_planner/trajectory/itomp_cio_trajectory.h>
#include <itomp_cio_planner/cost/smoothness_cost.h>
#include <itomp_cio_planner/cost/trajectory_cost_accumulator.h>
#include <itomp_cio_planner/util/vector_util.h>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <Eigen/StdVector>
#include <moveit/planning_scene/planning_scene.h>
#include <itomp_cio_planner/contact/contact_force_solver.h>

namespace itomp_cio_planner
{
class EvaluationManager;
class ItompPlanningGroup;
class EvaluationData
{
public:
  EvaluationData();
  virtual ~EvaluationData();

  void initialize(ItompCIOTrajectory *full_trajectory, ItompCIOTrajectory *group_trajectory,
      ItompRobotModel *robot_model, const ItompPlanningGroup *planning_group,
      const EvaluationManager* evaluation_manager, int num_mass_segments);

  double getNumPoints() const;
  double getNumJoints() const;

  void setTrajectories(ItompCIOTrajectory* group_trajectory, ItompCIOTrajectory* full_trajectory);

  ItompCIOTrajectory* getGroupTrajectory();
  const ItompCIOTrajectory* getGroupTrajectory() const;
  ItompCIOTrajectory* getFullTrajectory();
  const ItompCIOTrajectory* getFullTrajectory() const;

  KDL::JntArray kdl_joint_array_;

  std::vector<itomp_cio_planner::SmoothnessCost> joint_costs_;

  std::vector<std::vector<KDL::Vector> > joint_axis_;
  std::vector<std::vector<KDL::Vector> > joint_pos_;
  std::vector<std::vector<KDL::Frame> > segment_frames_;

  std::vector<int> state_is_in_collision_;
  std::vector<int> state_validity_;

  Eigen::VectorXd dynamic_obstacle_cost_;

  // physics
  std::vector<KDL::Wrench> wrenchSum_;
  std::vector<std::vector<KDL::Vector> > linkPositions_;
  std::vector<std::vector<KDL::Vector> > linkVelocities_;
  std::vector<std::vector<KDL::Vector> > linkAngularVelocities_;
  std::vector<KDL::Vector> CoMPositions_;
  std::vector<KDL::Vector> CoMVelocities_;
  std::vector<KDL::Vector> CoMAccelerations_;
  std::vector<KDL::Vector> AngularMomentums_;
  std::vector<KDL::Vector> Torques_;
  std::vector<std::vector<Vector4d> > contactViolationVector_;
  std::vector<std::vector<KDL::Vector> > contactPointVelVector_;

  std::vector<double> stateContactInvariantCost_;
  std::vector<double> statePhysicsViolationCost_;
  std::vector<double> stateCollisionCost_;

  TrajectoryCostAccumulator costAccumulator_;

  KDL::TreeFkSolverJointPosAxisPartial fk_solver_;
  ContactForceSolver contact_force_solver_;

  planning_scene::PlanningScenePtr planning_scene_;
  robot_state::RobotStatePtr kinematic_state_;

  EvaluationData* clone() const;
  void deepCopy(const EvaluationData& data);

  void compare(const EvaluationData& ref) const;

protected:
  void initStaticEnvironment();

  ItompRobotModel *robot_model_;

  ItompCIOTrajectory* group_trajectory_;
  ItompCIOTrajectory* full_trajectory_;
};
typedef boost::shared_ptr<EvaluationData> EvaluationDataPtr;

inline ItompCIOTrajectory* EvaluationData::getGroupTrajectory()
{
  return group_trajectory_;
}

inline const ItompCIOTrajectory* EvaluationData::getGroupTrajectory() const
{
  return group_trajectory_;
}

inline ItompCIOTrajectory* EvaluationData::getFullTrajectory()
{
  return full_trajectory_;
}

inline const ItompCIOTrajectory* EvaluationData::getFullTrajectory() const
{
  return full_trajectory_;
}

inline double EvaluationData::getNumPoints() const
{
  return group_trajectory_->getNumPoints();
}
inline double EvaluationData::getNumJoints() const
{
  return group_trajectory_->getNumJoints();
}

inline void EvaluationData::setTrajectories(ItompCIOTrajectory* group_trajectory, ItompCIOTrajectory* full_trajectory)
{
  group_trajectory_ = group_trajectory;
  full_trajectory_ = full_trajectory;
}

}

#endif
