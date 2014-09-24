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
#include <moveit/planning_scene/planning_scene.h>

namespace itomp_cio_planner
{
class ItompPlanningGroup;
class EvaluationManager
{
  class BackupData
  {
  public:
    double trajectory_value_;

    std::vector<std::vector<KDL::Frame> > segment_frames_;

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

    std::vector<double> state_collision_cost_;
    std::vector<double> state_contact_invariant_cost_;
    std::vector<double> state_physics_violation_cost_;
    std::vector<double> state_ftr_cost_;
  };

public:
  enum DERIVATIVE_VARIABLE_TYPE
  {
    DERIVATIVE_POSITION_VARIABLE = 0, DERIVATIVE_VELOCITY_VARIABLE = 1, DERIVATIVE_CONTACT_VARIABLE = 2,
  };

  EvaluationManager(int* iteration);
  EvaluationManager() {};
  virtual ~EvaluationManager();

  void initialize(ItompCIOTrajectory *full_trajectory, ItompCIOTrajectory *group_trajectory,
      ItompRobotModel *robot_model, const ItompPlanningGroup *planning_group, double planning_start_time,
      double trajectory_start_time, const moveit_msgs::Constraints& path_constraints);

  void setTrajectory(const Eigen::MatrixXd& parameters, const Eigen::MatrixXd& vel_parameters,
      const Eigen::MatrixXd& contact_parameters);
  void setTrajectory(const std::vector<Eigen::VectorXd>& parameters,
      const std::vector<Eigen::VectorXd>& contact_parameters);

  double evaluate();
  double evaluate(Eigen::VectorXd& costs);
  double evaluateDerivatives(double value, DERIVATIVE_VARIABLE_TYPE variable_type, int free_point_index,
      int joint_index);

  bool isLastTrajectoryFeasible() const;

  void handleJointLimits();
  void updateFullTrajectory();
  void render(int trajectory_index, bool is_best);

  const ItompCIOTrajectory* getGroupTrajectoryConst() const;
  const ItompCIOTrajectory* getFullTrajectoryConst() const;
  const ItompPlanningGroup* getPlanningGroup() const;

  void postprocess_ik();

  double getTrajectoryCost(bool verbose = false);

  const EvaluationData& getDefaultData() const;
  void setData(EvaluationData* data);
  void setDataToDefault();

private:
  double evaluate(DERIVATIVE_VARIABLE_TYPE variable_type, int point_index, int joint_index);

  bool performForwardKinematics();
  void computeTrajectoryValidity();
  void computeMassAndGravityForce();
  void computeWrenchSum();
  void computeStabilityCosts();
  void updateCoM(int point);
  void computeCollisionCosts();
  void computeFTRs();
  void computeCartesianTrajectoryCosts();
  void handleTrajectoryConstraint();
  void computeSingularityCosts();

  void updateFullTrajectory(int point_index, int joint_index);
  bool performForwardKinematics(int begin, int end);
  void computeWrenchSum(int begin, int end);
  void computeStabilityCosts(int begin, int end);
  void computeCollisionCosts(int begin, int end);
  void computeFTRs(int begin, int end);
  void computeSingularityCosts(int begin, int end);

  void backupAndSetVariables(double new_value, DERIVATIVE_VARIABLE_TYPE variable_type, int free_point_index,
      int joint_index);
  void restoreVariable(DERIVATIVE_VARIABLE_TYPE variable_type, int free_point_index, int joint_index);

  int getIteration() const;

  const KDL::Vector& getSegmentPosition(int point, const std::string& segmentName) const;
  const KDL::Vector& getSegmentPosition(int point, int segmentIndex) const;

  ItompCIOTrajectory* getGroupTrajectory();
  ItompCIOTrajectory* getFullTrajectory();

  EvaluationData* data_;
  EvaluationData default_data_;

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
typedef boost::shared_ptr<EvaluationManager> EvaluationManagerPtr;

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

inline const ItompCIOTrajectory* EvaluationManager::getFullTrajectoryConst() const
{
  return data_->getFullTrajectory();
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

inline const EvaluationData& EvaluationManager::getDefaultData() const
{
  return default_data_;
}

inline void EvaluationManager::setData(EvaluationData* data)
{
  data_ = data;
}
inline void EvaluationManager::setDataToDefault()
{
  data_ = &default_data_;
}

inline void EvaluationManager::computeCollisionCosts()
{
  computeCollisionCosts(full_vars_start_ + 1, full_vars_end_ - 1);
}

inline void EvaluationManager::computeFTRs()
{
  computeFTRs(0, num_points_);
}

inline void EvaluationManager::computeStabilityCosts()
{
  computeStabilityCosts(full_vars_start_ + 1, full_vars_end_ - 1);
}

inline bool EvaluationManager::performForwardKinematics()
{
  return performForwardKinematics(0, num_points_);
}

inline void EvaluationManager::computeWrenchSum()
{
  computeWrenchSum(full_vars_start_, full_vars_end_);
}

inline void EvaluationManager::computeSingularityCosts()
{
  computeSingularityCosts(full_vars_start_ + 1, full_vars_end_ - 1);
}

}

#endif
