#ifndef NEW_EVALUATION_MANAGER_H_
#define NEW_EVALUATION_MANAGER_H_

#include <itomp_cio_planner/common.h>
#include <itomp_cio_planner/optimization/evaluation_data.h>
#include <itomp_cio_planner/model/itomp_robot_model.h>
#include <itomp_cio_planner/trajectory/itomp_cio_trajectory.h>
#include <itomp_cio_planner/cost/smoothness_cost.h>
#include <itomp_cio_planner/cost/trajectory_cost_accumulator.h>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <ros/publisher.h>
#include <moveit/planning_scene/planning_scene.h>

namespace itomp_cio_planner
{
class ItompPlanningGroup;
class NewEvalManager
{
public:
  enum DERIVATIVE_VARIABLE_TYPE
  {
    DERIVATIVE_POSITION_VARIABLE = 0, DERIVATIVE_VELOCITY_VARIABLE = 1, DERIVATIVE_CONTACT_VARIABLE = 2,
  };

  NewEvalManager(int* iteration);
  virtual ~NewEvalManager();

  void initialize(ItompCIOTrajectory *full_trajectory, ItompCIOTrajectory *group_trajectory,
      ItompRobotModel *robot_model, const ItompPlanningGroup *planning_group, double planning_start_time,
      double trajectory_start_time, const moveit_msgs::Constraints& path_constraints);

  void setTrajectory(ItompCIOTrajectory * group_trajectory);

  double evaluate();
  double evaluateDerivatives();

  bool isLastTrajectoryFeasible() const;

  void handleJointLimits();
  void updateFullTrajectory();
  void render();

private:
  bool performForwardKinematics();
  bool performInverseDynamics();

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

  int num_vars_full_;
  int full_vars_start_;
  int full_vars_end_;

  std::vector<int> group_joint_to_kdl_joint_index_;

  bool is_collision_free_;
  bool last_trajectory_collision_free_;

  bool trajectory_validity_;

  // physics
  double total_mass_;
  std::vector<double> masses_;
  int num_mass_segments_;
  KDL::Vector gravity_force_;

  ros::Publisher vis_marker_array_pub_;
  ros::Publisher vis_marker_pub_;

  BackupData backup_data_;

  // TODO: refactoring
  int getSegmentIndex(int link, bool isLeft) const;
  void getJointIndex(int& groupIndex, int& kdlIndex, int joint, bool isLeft) const;
  void computeBaseFrames(KDL::JntArray& curJointArray, int point);
  void ComputeCollisionFreeLegUsingIK(int legIndex, const KDL::Vector& rootPos, const KDL::Frame& destPose,
      KDL::JntArray& curJointArray, int point, bool support = true, bool updatePhase = false);
  KDL::JntArray phaseJointArray_[3];

  // for debug
  std::vector<double> timings_;
  int count_;
public:
  bool print_debug_texts_;

};
typedef boost::shared_ptr<NewEvalManager> NewEvalManagerPtr;

inline bool NewEvalManager::isLastTrajectoryFeasible() const
{
  return last_trajectory_collision_free_;
}

inline const KDL::Vector& NewEvalManager::getSegmentPosition(int point, const std::string& segmentName) const
{
  int sn = robot_model_->getForwardKinematicsSolver()->segmentNameToIndex(segmentName);
  return getSegmentPosition(point, sn);
}

inline const KDL::Vector& NewEvalManager::getSegmentPosition(int point, int segmentIndex) const
{
  return data_->segment_frames_[point][segmentIndex].p;
}

inline int NewEvalManager::getIteration() const
{
  return *iteration_;
}

inline const ItompCIOTrajectory* NewEvalManager::getGroupTrajectoryConst() const
{
  return data_->getGroupTrajectory();
}

inline const ItompCIOTrajectory* NewEvalManager::getFullTrajectoryConst() const
{
  return data_->getFullTrajectory();
}

inline const ItompPlanningGroup* NewEvalManager::getPlanningGroup() const
{
  return planning_group_;
}

inline ItompCIOTrajectory* NewEvalManager::getGroupTrajectory()
{
  return data_->getGroupTrajectory();
}

inline ItompCIOTrajectory* NewEvalManager::getFullTrajectory()
{
  return data_->getFullTrajectory();
}

inline const EvaluationData& NewEvalManager::getDefaultData() const
{
  return default_data_;
}

inline void NewEvalManager::setData(EvaluationData* data)
{
  data_ = data;
}
inline void NewEvalManager::setDataToDefault()
{
  data_ = &default_data_;
}

inline void NewEvalManager::computeCollisionCosts()
{
  computeCollisionCosts(full_vars_start_ + 1, full_vars_end_ - 1);
}

inline void NewEvalManager::computeFTRs()
{
  computeFTRs(0, num_points_);
}

inline void NewEvalManager::computeStabilityCosts()
{
  computeStabilityCosts(full_vars_start_ + 1, full_vars_end_ - 1);
}

inline bool NewEvalManager::performForwardKinematics()
{
  return performForwardKinematics(0, num_points_);
}

inline void NewEvalManager::computeWrenchSum()
{
  computeWrenchSum(full_vars_start_, full_vars_end_);
}

inline void NewEvalManager::computeSingularityCosts()
{
  computeSingularityCosts(full_vars_start_ + 1, full_vars_end_ - 1);
}

}

#endif
