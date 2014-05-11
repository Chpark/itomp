#ifndef EVALUATION_MANAGER_H_
#define EVALUATION_MANAGER_H_

#include <itomp_cio_planner/common.h>
#include <itomp_cio_planner/optimization/evaluation_data.h>
#include <itomp_cio_planner/model/itomp_robot_model.h>
#include <itomp_cio_planner/trajectory/itomp_cio_trajectory.h>
#include <itomp_cio_planner/cost/smoothness_cost.h>
#include <itomp_cio_planner/cost/trajectory_cost_accumulator.h>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <ros/publisher.h>
#include <Eigen/StdVector>
#include <moveit/planning_scene/planning_scene.h>

namespace itomp_cio_planner
{
class ItompPlanningGroup;
class EvaluationManager
{
public:
  EvaluationManager(int* iteration);
  virtual ~EvaluationManager();

  void initialize(ItompCIOTrajectory *full_trajectory, ItompCIOTrajectory *group_trajectory,
      ItompRobotModel *robot_model, const ItompPlanningGroup *planning_group, double planning_start_time,
      double trajectory_start_time);

  double evaluate();

  double evaluate(const Eigen::MatrixXd& parameters, const Eigen::MatrixXd& vel_parameters,
      const Eigen::MatrixXd& contact_parameters, Eigen::VectorXd& costs);
  void evaluateDerivatives(const Eigen::MatrixXd& parameters, const Eigen::MatrixXd& vel_parameters,
      const Eigen::MatrixXd& contact_parameters, Eigen::VectorXd& derivatives);

  bool isLastTrajectoryFeasible() const;

  void handleJointLimits();
  void updateFullTrajectory();
  bool performForwardKinematics(); /**< Return true if collision free */
  void computeTrajectoryValidity();
  void render(int trajectory_index);

  const ItompCIOTrajectory* getGroupTrajectoryConst() const;
  const ItompPlanningGroup* getPlanningGroup() const;

  void postprocess_ik();

  double getTrajectoryCost(bool verbose = false);

private:
  void initStaticEnvironment();

  void computeMassAndGravityForce();
  void computeWrenchSum();
  void computeStabilityCosts();
  void updateCoM(int point);
  void computeCollisionCosts();

  int getIteration() const;

  const KDL::Vector& getSegmentPosition(int point, const std::string& segmentName) const;
  const KDL::Vector& getSegmentPosition(int point, int segmentIndex) const;

  ItompCIOTrajectory* getGroupTrajectory();
  ItompCIOTrajectory* getFullTrajectory();

  EvaluationData* data_;
  EvaluationData default_data_;

  planning_scene::PlanningScenePtr planning_scene_;

  double planning_start_time_;
  double trajectory_start_time_;

  const ItompRobotModel *robot_model_;
  const ItompPlanningGroup *planning_group_;
  std::string robot_name_;

  int* iteration_;

  int num_joints_;
  int num_contacts_;
  int num_points_;
  int num_contact_points_;

  std::vector<int> group_joint_to_kdl_joint_index_;

  bool is_collision_free_;
  bool last_trajectory_collision_free_;

  bool trajectory_validity_;

  // physics
  double totalMass_;
  std::vector<double> masses_;
  int numMassSegments_;
  KDL::Vector gravityForce_;

  ros::Publisher vis_marker_array_pub_;
  ros::Publisher vis_marker_pub_;

  // TODO: refactoring
  /*
  friend class TrajectoryCostAccumulator;
  friend class TrajectoryCost;
  friend class TrajectorySmoothnessCost;
  friend class TrajectoryCollisionCost;
  friend class TrajectoryValidityCost;
  friend class TrajectoryContactInvariantCost;
  friend class TrajectoryPhysicsViolationCost;
  friend class TrajectoryGoalPoseCost;
  friend class TrajectoryCoMCost;
  */

  // for debug
  std::vector<double> timings_;
};

inline bool EvaluationManager::isLastTrajectoryFeasible() const
{
  return last_trajectory_collision_free_;
}

inline const KDL::Vector& EvaluationManager::getSegmentPosition(int point, const std::string& segmentName) const
{
  int sn = robot_model_->getForwardKinematicsSolver()->segmentNameToIndex(segmentName);
  return getSegmentPosition(point, sn);
}

inline const KDL::Vector& EvaluationManager::getSegmentPosition(int point, int segmentIndex) const
{
  return data_->segment_frames_[point][segmentIndex].p;
}

inline int EvaluationManager::getIteration() const
{
  return *iteration_;
}

inline const ItompCIOTrajectory* EvaluationManager::getGroupTrajectoryConst() const
{
  return data_->getGroupTrajectory();
}

inline const ItompPlanningGroup* EvaluationManager::getPlanningGroup() const
{
  return planning_group_;
}

inline ItompCIOTrajectory* EvaluationManager::getGroupTrajectory()
{
  return data_->getGroupTrajectory();
}

inline ItompCIOTrajectory* EvaluationManager::getFullTrajectory()
{
  return data_->getFullTrajectory();
}

}

#endif
