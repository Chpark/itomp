#include <ros/ros.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/PlanningScene.h>
#include <itomp_cio_planner/optimization/evaluation_manager.h>
#include <itomp_cio_planner/model/itomp_planning_group.h>
#include <itomp_cio_planner/contact/ground_manager.h>
#include <itomp_cio_planner/visualization/visualization_manager.h>
#include <itomp_cio_planner/contact/contact_force_solver.h>
#include <itomp_cio_planner/util/min_jerk_trajectory.h>
#include <itomp_cio_planner/util/planning_parameters.h>
#include <itomp_cio_planner/util/vector_util.h>
#include <itomp_cio_planner/util/multivariate_gaussian.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std;
using namespace Eigen;

namespace itomp_cio_planner
{

static bool STABILITY_COST_VERBOSE = false;

EvaluationManager::EvaluationManager(int* iteration) :
    iteration_(iteration), data_(&default_data_), count_(0)
{

}

EvaluationManager::~EvaluationManager()
{

}

void EvaluationManager::initialize(ItompCIOTrajectory *full_trajectory, ItompCIOTrajectory *group_trajectory,
    ItompRobotModel *robot_model, const ItompPlanningGroup *planning_group, double planning_start_time,
    double trajectory_start_time)
{
  planning_start_time_ = planning_start_time;
  trajectory_start_time_ = trajectory_start_time;

  robot_model_ = robot_model;
  planning_group_ = planning_group;
  robot_name_ = robot_model_->getRobotName();

  // init some variables:
  num_joints_ = group_trajectory->getNumJoints();
  num_contacts_ = group_trajectory->getNumContacts();
  num_points_ = group_trajectory->getNumPoints();
  num_contact_points_ = group_trajectory->getNumContactPhases() + 1;

  // set up joint index:
  group_joint_to_kdl_joint_index_.resize(num_joints_);
  for (int i = 0; i < num_joints_; ++i)
    group_joint_to_kdl_joint_index_[i] = planning_group_->group_joints_[i].kdl_joint_index_;

  is_collision_free_ = false;
  last_trajectory_collision_free_ = false;

  // Initialize visualizer
  VisualizationManager::getInstance()->setPlanningGroup(*robot_model_, planning_group_->name_);
  vis_marker_pub_ = VisualizationManager::getInstance()->getVisualizationMarkerPublisher();
  vis_marker_array_pub_ = VisualizationManager::getInstance()->getVisualizationMarkerArrayPublisher();

  computeMassAndGravityForce();

  GroundManager::getInstance().init();

  default_data_.initialize(full_trajectory, group_trajectory, robot_model, planning_group, this, num_mass_segments_);

  timings_.resize(100, 0);
  for (int i = 0; i < 100; ++i)
    timings_[i] = 0;
}

double EvaluationManager::evaluate()
{
  // do forward kinematics:
  last_trajectory_collision_free_ = performForwardKinematics();

  computeTrajectoryValidity();
  last_trajectory_collision_free_ &= trajectory_validity_;

  computeWrenchSum();

  computeStabilityCosts();

  computeCollisionCosts();

  data_->costAccumulator_.compute(data_);

  last_trajectory_collision_free_ &= data_->costAccumulator_.isFeasible();

  // TODO: if trajectory is changed in handle joint limits,
  // update parameters

  return data_->costAccumulator_.getTrajectoryCost();
}

double EvaluationManager::evaluate(DERIVATIVE_VARIABLE_TYPE variable_type, int free_point_index, int joint_index)
{
  int stride = getGroupTrajectory()->getContactPhaseStride();
  int begin = (free_point_index - 1) * stride;
  int end = (free_point_index + 1) * stride;

  INIT_TIME_MEASUREMENT(10)

  // do forward kinematics:
  if (variable_type != DERIVATIVE_CONTACT_VARIABLE)
    performForwardKinematics(begin, end);

  ADD_TIMER_POINT

  computeTrajectoryValidity();

  ADD_TIMER_POINT

  if (variable_type != DERIVATIVE_CONTACT_VARIABLE)
    computeWrenchSum(begin, end + 1);

  ADD_TIMER_POINT

  computeStabilityCosts(begin, end + 1);

  ADD_TIMER_POINT

  if (variable_type != DERIVATIVE_CONTACT_VARIABLE)
    computeCollisionCosts(begin, end);

  ADD_TIMER_POINT

  data_->costAccumulator_.compute(data_);

  UPDATE_TIME
  PRINT_TIME(evaluate, 10000)

  return data_->costAccumulator_.getTrajectoryCost();
}

void EvaluationManager::setTrajectory(const Eigen::MatrixXd& parameters, const Eigen::MatrixXd& vel_parameters,
    const Eigen::MatrixXd& contact_parameters)
{
  // copy the parameters into group_trajectory:
  int num_free_points = parameters.rows();
  ROS_ASSERT(getGroupTrajectory()->getFreePoints().rows() == num_free_points);

  getGroupTrajectory()->getFreePoints() = parameters;
  getGroupTrajectory()->getFreeVelPoints() = vel_parameters;
  getGroupTrajectory()->getContactTrajectory() = contact_parameters;

  getGroupTrajectory()->updateTrajectoryFromFreePoints();

  // respect joint limits:
  handleJointLimits();

  // copy to full traj:
  updateFullTrajectory();
}

void EvaluationManager::backupAndSetVariables(double new_value, DERIVATIVE_VARIABLE_TYPE variable_type,
    int free_point_index, int joint_index)
{
  // backup trajectory value
  Eigen::MatrixXd* target_trajectory;
  switch (variable_type)
  {
  case DERIVATIVE_POSITION_VARIABLE:
    target_trajectory = &getGroupTrajectory()->getFreePoints();
    break;

  case DERIVATIVE_VELOCITY_VARIABLE:
    target_trajectory = &getGroupTrajectory()->getFreeVelPoints();
    break;

  case DERIVATIVE_CONTACT_VARIABLE:
    target_trajectory = &getGroupTrajectory()->getContactTrajectory();
    break;
  }
  backup_data_.trajectory_value_ = (*target_trajectory)(free_point_index, joint_index);

  ROS_ASSERT(variable_type != DERIVATIVE_CONTACT_VARIABLE || new_value >= 0.0);

  // change trajectory variable
  (*target_trajectory)(free_point_index, joint_index) = new_value;

  // update trajectory
  if (variable_type != DERIVATIVE_CONTACT_VARIABLE)
  {
    getGroupTrajectory()->updateTrajectoryFromFreePoint(free_point_index, joint_index);
    handleJointLimits();

    // TODO: below function does not update free var trajectory
    updateFullTrajectory(free_point_index, joint_index);
  }

  int stride = getGroupTrajectory()->getContactPhaseStride();
  if (free_point_index == 0)
    free_point_index = 1;
  int begin = (free_point_index - 1) * stride;

  backup_data_.segment_frames_.resize(2 * stride);

  backup_data_.wrenchSum_.resize(2 * stride + 1);
  backup_data_.linkPositions_.resize(num_mass_segments_);
  backup_data_.linkVelocities_.resize(num_mass_segments_);
  backup_data_.linkAngularVelocities_.resize(num_mass_segments_);
  for (int i = 0; i < num_mass_segments_; ++i)
  {
    backup_data_.linkPositions_[i].resize(2 * stride + 1);
    backup_data_.linkVelocities_[i].resize(2 * stride + 1);
    backup_data_.linkAngularVelocities_[i].resize(2 * stride + 1);
  }
  backup_data_.CoMPositions_.resize(2 * stride + 1);
  backup_data_.CoMVelocities_.resize(2 * stride + 1);
  backup_data_.CoMAccelerations_.resize(2 * stride + 1);
  backup_data_.AngularMomentums_.resize(2 * stride + 1);
  backup_data_.Torques_.resize(2 * stride + 1);
  backup_data_.contactViolationVector_.resize(num_contacts_);
  backup_data_.contactPointVelVector_.resize(num_contacts_);
  for (int i = 0; i < num_contacts_; ++i)
  {
    backup_data_.contactViolationVector_[i].resize(2 * stride + 1);
    backup_data_.contactPointVelVector_[i].resize(2 * stride + 1);
  }

  backup_data_.state_contact_invariant_cost_.resize(2 * stride + 1);
  backup_data_.state_physics_violation_cost_.resize(2 * stride + 1);
  backup_data_.state_collision_cost_.resize(2 * stride);

  for (int i = 0; i < 2 * stride; ++i)
  {
    backup_data_.segment_frames_[i] = data_->segment_frames_[begin + i];
  }

  memcpy(&backup_data_.wrenchSum_[0], &data_->wrenchSum_[begin], sizeof(KDL::Wrench) * 2 * stride + 1);
  for (int i = 0; i < num_mass_segments_; ++i)
  {
    memcpy(&backup_data_.linkPositions_[i][0], &data_->linkPositions_[i][begin], sizeof(KDL::Vector) * 2 * stride + 1);
    memcpy(&backup_data_.linkVelocities_[i][0], &data_->linkVelocities_[i][begin],
        sizeof(KDL::Vector) * 2 * stride + 1);
    memcpy(&backup_data_.linkAngularVelocities_[i][0], &data_->linkAngularVelocities_[i][begin],
        sizeof(KDL::Vector) * 2 * stride + 1);
  }
  memcpy(&backup_data_.CoMPositions_[0], &data_->CoMPositions_[begin], sizeof(KDL::Vector) * 2 * stride + 1);
  memcpy(&backup_data_.CoMVelocities_[0], &data_->CoMVelocities_[begin], sizeof(KDL::Vector) * 2 * stride + 1);
  memcpy(&backup_data_.CoMAccelerations_[0], &data_->CoMAccelerations_[begin], sizeof(KDL::Vector) * 2 * stride + 1);
  memcpy(&backup_data_.AngularMomentums_[0], &data_->AngularMomentums_[begin], sizeof(KDL::Vector) * 2 * stride + 1);
  memcpy(&backup_data_.Torques_[0], &data_->Torques_[begin], sizeof(KDL::Vector) * 2 * stride + 1);
  for (int i = 0; i < num_contacts_; ++i)
  {
    memcpy(&backup_data_.contactViolationVector_[i][0], &data_->contactViolationVector_[i][begin],
        sizeof(Vector4d) * 2 * stride + 1);
    memcpy(&backup_data_.contactPointVelVector_[i][0], &data_->contactPointVelVector_[i][begin],
        sizeof(KDL::Vector) * 2 * stride + 1);
  }

  memcpy(&backup_data_.state_contact_invariant_cost_[0], &data_->stateContactInvariantCost_[begin],
      sizeof(double) * 2 * stride + 1);
  memcpy(&backup_data_.state_physics_violation_cost_[0], &data_->statePhysicsViolationCost_[begin],
      sizeof(double) * 2 * stride + 1);
  memcpy(&backup_data_.state_collision_cost_[0], &data_->stateCollisionCost_[begin], sizeof(double) * 2 * stride);
}

void EvaluationManager::restoreVariable(DERIVATIVE_VARIABLE_TYPE variable_type, int free_point_index, int joint_index)
{
  Eigen::MatrixXd* target_trajectory;
  switch (variable_type)
  {
  case DERIVATIVE_POSITION_VARIABLE:
    target_trajectory = &getGroupTrajectory()->getFreePoints();
    break;

  case DERIVATIVE_VELOCITY_VARIABLE:
    target_trajectory = &getGroupTrajectory()->getFreeVelPoints();
    break;

  case DERIVATIVE_CONTACT_VARIABLE:
    target_trajectory = &getGroupTrajectory()->getContactTrajectory();
    break;
  }

  // restore trajectory value
  (*target_trajectory)(free_point_index, joint_index) = backup_data_.trajectory_value_;
  if (variable_type != DERIVATIVE_CONTACT_VARIABLE)
  {
    getGroupTrajectory()->updateTrajectoryFromFreePoint(free_point_index, joint_index);
    updateFullTrajectory(free_point_index, joint_index);
  }
  // restore variables
  int stride = getGroupTrajectory()->getContactPhaseStride();
  if (free_point_index == 0)
    free_point_index = 1;
  int begin = (free_point_index - 1) * stride;

  for (int i = 0; i < 2 * stride; ++i)
    data_->segment_frames_[begin + i] = backup_data_.segment_frames_[i];

  memcpy(&data_->wrenchSum_[begin], &backup_data_.wrenchSum_[0], sizeof(KDL::Wrench) * 2 * stride + 1);
  for (int i = 0; i < num_mass_segments_; ++i)
  {
    memcpy(&data_->linkPositions_[i][begin], &backup_data_.linkPositions_[i][0], sizeof(KDL::Vector) * 2 * stride + 1);
    memcpy(&data_->linkVelocities_[i][begin], &backup_data_.linkVelocities_[i][0],
        sizeof(KDL::Vector) * 2 * stride + 1);
    memcpy(&data_->linkAngularVelocities_[i][begin], &backup_data_.linkAngularVelocities_[i][0],
        sizeof(KDL::Vector) * 2 * stride + 1);
  }
  memcpy(&data_->CoMPositions_[begin], &backup_data_.CoMPositions_[0], sizeof(KDL::Vector) * 2 * stride + 1);
  memcpy(&data_->CoMVelocities_[begin], &backup_data_.CoMVelocities_[0], sizeof(KDL::Vector) * 2 * stride + 1);
  memcpy(&data_->CoMAccelerations_[begin], &backup_data_.CoMAccelerations_[0], sizeof(KDL::Vector) * 2 * stride + 1);
  memcpy(&data_->AngularMomentums_[begin], &backup_data_.AngularMomentums_[0], sizeof(KDL::Vector) * 2 * stride + 1);
  memcpy(&data_->Torques_[begin], &backup_data_.Torques_[0], sizeof(KDL::Vector) * 2 * stride + 1);
  for (int i = 0; i < num_contacts_; ++i)
  {
    memcpy(&data_->contactViolationVector_[i][begin], &backup_data_.contactViolationVector_[i][0],
        sizeof(Vector4d) * 2 * stride + 1);
    memcpy(&data_->contactPointVelVector_[i][begin], &backup_data_.contactPointVelVector_[i][0],
        sizeof(KDL::Vector) * 2 * stride + 1);
  }

  memcpy(&data_->stateContactInvariantCost_[begin], &backup_data_.state_contact_invariant_cost_[0],
      sizeof(double) * 2 * stride + 1);
  memcpy(&data_->statePhysicsViolationCost_[begin], &backup_data_.state_physics_violation_cost_[0],
      sizeof(double) * 2 * stride + 1);
  memcpy(&data_->stateCollisionCost_[begin], &backup_data_.state_collision_cost_[0], sizeof(double) * 2 * stride);
}

double EvaluationManager::evaluateDerivatives(double value, DERIVATIVE_VARIABLE_TYPE variable_type,
    int free_point_index, int joint_index)
{
  // backup old values and update trajectory
  backupAndSetVariables(value, variable_type, free_point_index, joint_index);

  // evaluate
  double cost = evaluate(variable_type, free_point_index, joint_index);

  restoreVariable(variable_type, free_point_index, joint_index);

  return cost;
}

void EvaluationManager::render(int trajectory_index)
{
  if (PlanningParameters::getInstance()->getAnimateEndeffector())
  {
    VisualizationManager::getInstance()->animateEndeffector(trajectory_index, num_points_, 0, data_->segment_frames_,
        data_->state_validity_, false);
    VisualizationManager::getInstance()->animateCoM(num_points_, 0, data_->CoMPositions_, false);
  }
  if (PlanningParameters::getInstance()->getAnimatePath())
  {
    VisualizationManager::getInstance()->animatePath(0, num_points_ - 1);
  }
}

void EvaluationManager::computeMassAndGravityForce()
{
  total_mass_ = 0.0;
  const KDL::SegmentMap& segmentMap = robot_model_->getKDLTree()->getSegments();
  num_mass_segments_ = 0;
  for (KDL::SegmentMap::const_iterator it = segmentMap.begin(); it != segmentMap.end(); ++it)
  {
    const KDL::Segment& segment = it->second.segment;
    double mass = segment.getInertia().getMass();
    if (mass == 0)
      continue;

    total_mass_ += mass;
    masses_.push_back(mass);

    ++num_mass_segments_;
  }
  gravity_force_ = total_mass_ * KDL::Vector(0.0, 0.0, -9.8);

  // normalize gravity force to 1.0
  gravity_force_ = KDL::Vector(0.0, 0.0, -1.0);
}

void EvaluationManager::handleJointLimits()
{
  for (int joint = 0; joint < num_joints_; joint++)
  {
    if (!planning_group_->group_joints_[joint].has_joint_limits_)
      continue;

    double joint_max = planning_group_->group_joints_[joint].joint_limit_max_;
    double joint_min = planning_group_->group_joints_[joint].joint_limit_min_;

    int count = 0;

    for (int i = 1; i < num_points_ - 2; i++)
    {
      if ((*getGroupTrajectory())(i, joint) > joint_max)
      {
        (*getGroupTrajectory())(i, joint) = joint_max;
      }
      else if ((*getGroupTrajectory())(i, joint) < joint_min)
      {
        (*getGroupTrajectory())(i, joint) = joint_min;
      }
    }
  }
}

void EvaluationManager::updateFullTrajectory()
{
  getFullTrajectory()->updateFromGroupTrajectory(*getGroupTrajectory());
}

void EvaluationManager::updateFullTrajectory(int point_index, int joint_index)
{
  getFullTrajectory()->updateFromGroupTrajectory(*getGroupTrajectory(), point_index, joint_index);
}

bool EvaluationManager::performForwardKinematics(int begin, int end)
{
  double invTime = 1.0 / getGroupTrajectory()->getDiscretization();
  double invTimeSq = invTime * invTime;

  is_collision_free_ = true;

  int safe_begin = max(0, begin);
  int safe_end = min(num_points_, end);

  // for each point in the trajectory
  for (int i = safe_begin; i < safe_end; ++i)
  {
    getFullTrajectory()->getTrajectoryPointKDL(i, data_->kdl_joint_array_);
    // update kdl_joint_array with vel, acc
    if (i < 1)
    {
      for (int j = 0; j < planning_group_->num_joints_; j++)
      {
        int target_joint = planning_group_->group_joints_[j].kdl_joint_index_;
        data_->kdl_joint_array_(target_joint) = (*getGroupTrajectory())(i, j);
      }
    }

    if (i == safe_begin)
      data_->fk_solver_.JntToCartFull(data_->kdl_joint_array_, data_->joint_pos_[i], data_->joint_axis_[i],
          data_->segment_frames_[i]);
    else
      data_->fk_solver_.JntToCartPartial(data_->kdl_joint_array_, data_->joint_pos_[i], data_->joint_axis_[i],
          data_->segment_frames_[i]);

    data_->state_is_in_collision_[i] = false;

    if (data_->state_is_in_collision_[i])
    {
      is_collision_free_ = false;
    }
  }

  return is_collision_free_;
}

void EvaluationManager::computeTrajectoryValidity()
{
  trajectory_validity_ = true;
  const double clearance = 0.001;
  int collisionBV = 8001;
  visualization_msgs::Marker marker;
  visualization_msgs::Marker markerTemp;

  for (int i = 1; i < num_points_ - 1; i++)
  {
    bool valid = true;

    /*
     dynamic_obstacle_cost_(i) = 0;

     int full_traj_index = group_trajectory_->getFullTrajectoryIndex(i);
     full_trajectory_->getTrajectoryPointKDL(full_traj_index, kdl_joint_array_);
     for (int j = 0; j < full_trajectory_->getNumJoints(); ++j)
     {
     robot_state_.joint_state.position[j] = kdl_joint_array_(j);
     }

     planning_environment::setRobotStateAndComputeTransforms(robot_state_, *kinematic_state_);

     collision_space::EnvironmentModel
     * env_model =
     const_cast<collision_space::EnvironmentModel*> (collision_proximity_space_->getCollisionModelsInterface()->getOde());
     env_model->updateRobotModel(&(*kinematic_state_));

     // check collision points with dynamic obstacles
     double point_time = full_trajectory_->getDiscretization() * (i - 1);
     double cur_time = trajectory_start_time_ - planning_start_time_ + point_time;

     // TODO: dynamic obs
     double obstacleScale = 1.1 * (1 + cur_time * SENSOR_NOISE);
     obstacleScale = 1.0;

     bool inNextExecution = point_time < PlanningParameters::getInstance()->getPlanningStepSize();
     for (unsigned int j = 0; j < dynamic_obstacles_->size(); ++j)
     {
     const pomp_dynamic_obs_msgs::DynamicObstacle& dynamic_obstacle = dynamic_obstacles_->at(j);

     btVector3 origin(dynamic_obstacle.x, dynamic_obstacle.y, dynamic_obstacle.z);
     double obstacle_radius = dynamic_obstacle.lengthX * 0.5 * obstacleScale;
     }
     */

    data_->state_validity_[i] = valid;
    if (!valid)
      trajectory_validity_ = false;

  }

}

void EvaluationManager::updateCoM(int point)
{
  const KDL::SegmentMap& segmentMap = robot_model_->getKDLTree()->getSegments();
  // compute CoM, p_j
  int massSegmentIndex = 0;
  data_->CoMPositions_[point] = KDL::Vector::Zero();
  for (KDL::SegmentMap::const_iterator it = segmentMap.begin(); it != segmentMap.end(); ++it)
  {
    const KDL::Segment& segment = it->second.segment;
    double mass = segment.getInertia().getMass();
    int sn = robot_model_->getForwardKinematicsSolver()->segmentNameToIndex(segment.getName());
    const KDL::Vector& pos = data_->segment_frames_[point][sn] * segment.getInertia().getCOG();
    if (mass == 0.0)
      continue;

    data_->CoMPositions_[point] += pos * mass;
    data_->linkPositions_[massSegmentIndex][point] = pos;
    ++massSegmentIndex;
  }
  data_->CoMPositions_[point] = data_->CoMPositions_[point] / total_mass_;
}

void EvaluationManager::computeWrenchSum(int begin, int end)
{
  if (planning_group_->name_ != "lower_body" && planning_group_->name_ != "whole_body")
    return;

  int safe_begin = max(0, begin);
  int safe_end = min(num_points_, end);

  // compute CoM, p_j
  for (int point = safe_begin; point < safe_end; ++point)
  {
    updateCoM(point);
  }

  safe_begin = max(1, begin);
  safe_end = min(num_points_ - 1, end);

  // compute \dot{CoM} \ddot{CoM}
  itomp_cio_planner::getVectorVelocitiesAndAccelerations(safe_begin, safe_end - 1,
      getGroupTrajectory()->getDiscretization(), data_->CoMPositions_, data_->CoMVelocities_, data_->CoMAccelerations_,
      KDL::Vector::Zero());
  // compute \dot{p_j}
  for (int i = 0; i < num_mass_segments_; ++i)
  {
    itomp_cio_planner::getVectorVelocities(safe_begin, safe_end - 1, getGroupTrajectory()->getDiscretization(),
        data_->linkPositions_[i], data_->linkVelocities_[i], KDL::Vector::Zero());
  }

  // debug
  if (STABILITY_COST_VERBOSE)
  {
    printf("CoMPos CoMVel CoMAcc \n");
    for (int i = safe_begin; i < safe_end; ++i)
    {
      printf("%f %f %f %f %f %f %f %f %f\n", data_->CoMPositions_[i].x(), data_->CoMPositions_[i].y(),
          data_->CoMPositions_[i].z(), data_->CoMVelocities_[i].x(), data_->CoMVelocities_[i].y(),
          data_->CoMVelocities_[i].z(), data_->CoMAccelerations_[i].x(), data_->CoMAccelerations_[i].y(),
          data_->CoMAccelerations_[i].z());
    }
  }

  // TODO: compute angular velocities = (cur-prev)/time
  const KDL::SegmentMap& segmentMap = robot_model_->getKDLTree()->getSegments();
  const double invTime = 1.0 / getGroupTrajectory()->getDiscretization();
  for (int point = safe_begin; point < safe_end; ++point)
  {
    int massSegmentIndex = 0;
    for (KDL::SegmentMap::const_iterator it = segmentMap.begin(); it != segmentMap.end(); ++it)
    {
      const KDL::Segment& segment = it->second.segment;
      double mass = segment.getInertia().getMass();
      int sn = robot_model_->getForwardKinematicsSolver()->segmentNameToIndex(segment.getName());
      const KDL::Vector& pos = data_->segment_frames_[point][sn] * segment.getInertia().getCOG();
      if (mass == 0.0)
        continue;

      const KDL::Rotation& prevRotation = data_->segment_frames_[point - 1][sn].M;
      const KDL::Rotation& curRotation = data_->segment_frames_[point][sn].M;
      const KDL::Rotation& rotDiff = curRotation * prevRotation.Inverse();
      data_->linkAngularVelocities_[massSegmentIndex][point] = rotDiff.GetRot() * invTime;
      ++massSegmentIndex;
    }
  }

  // compute angular momentum
  for (int point = safe_begin; point < safe_end; ++point)
  {
    data_->AngularMomentums_[point] = KDL::Vector(0.0, 0.0, 0.0);

    int massSegmentIndex = 0;
    for (KDL::SegmentMap::const_iterator it = segmentMap.begin(); it != segmentMap.end(); ++it)
    {
      const KDL::Segment& segment = it->second.segment;
      double mass = segment.getInertia().getMass();
      if (mass == 0.0)
        continue;

      int sn = robot_model_->getForwardKinematicsSolver()->segmentNameToIndex(segment.getName());
      KDL::Vector angularVelTerm = (data_->segment_frames_[point][sn] * segment.getInertia()).getRotationalInertia()
          * data_->linkAngularVelocities_[massSegmentIndex][point];

      data_->AngularMomentums_[point] += masses_[massSegmentIndex]
          * (data_->linkPositions_[massSegmentIndex][point] - data_->CoMPositions_[point])
          * data_->linkVelocities_[massSegmentIndex][point] + angularVelTerm;
      ++massSegmentIndex;
    }
  }
  // compute torques
  itomp_cio_planner::getVectorVelocities(safe_begin, safe_end - 1, getGroupTrajectory()->getDiscretization(),
      data_->AngularMomentums_, data_->Torques_, KDL::Vector::Zero());

  // compute wrench sum (gravity wrench + inertia wrench)
  for (int point = safe_begin; point < safe_end; ++point)
  {
    data_->wrenchSum_[point].force = gravity_force_;
    data_->wrenchSum_[point].torque = data_->CoMPositions_[point] * gravity_force_;

    //wrenchSum_[point].force += -totalMass_ * CoMAccelerations_[point];
    //wrenchSum_[point].torque += -totalMass_ * CoMPositions_[point] * CoMAccelerations_[point] - Torques_[point];

    /*
     ROS_INFO("[%d] CoM pos:(%f %f %f)", point, CoMPositions_[point].x(), CoMPositions_[point].y(),
     CoMPositions_[point].z());
     ROS_INFO("[%d] CoM acc:(%f %f %f)", point, CoMAccelerations_[point].x(), CoMAccelerations_[point].y(),
     CoMAccelerations_[point].z());
     ROS_INFO("[%d] Ang mon:(%f %f %f)", point, AngularMomentums_[point].x(), AngularMomentums_[point].y(),
     AngularMomentums_[point].z());
     ROS_INFO("[%d] Com Tor:(%f %f %f)", point, Torques_[point].x(), Torques_[point].y(), Torques_[point].z());
     ROS_INFO("[%d] Wre For:(%f %f %f)", point, gravityForce_.x(), gravityForce_.y(), gravityForce_.z());
     ROS_INFO("[%d] Wre Tor:(%f %f %f)=(%f %f %f)x(%f %f %f)+%f(%f %f %f)x(%f %f %f)-(%f %f %f)", point,
     wrenchSum_[point].torque.x(), wrenchSum_[point].torque.y(), wrenchSum_[point].torque.z(),
     CoMPositions_[point].x(), CoMPositions_[point].y(), CoMPositions_[point].z(), gravityForce_.x(),
     gravityForce_.y(), gravityForce_.z(), totalMass_, CoMPositions_[point].x(), CoMPositions_[point].y(),
     CoMPositions_[point].z(), CoMAccelerations_[point].x(), CoMAccelerations_[point].y(),
     CoMAccelerations_[point].z(), Torques_[point].x(), Torques_[point].y(), Torques_[point].z());
     */

  }

  for (int i = 0; i < planning_group_->getNumContacts(); ++i)
  {
    planning_group_->contactPoints_[i].updateContactViolationVector(safe_begin, safe_end - 1,
        getGroupTrajectory()->getDiscretization(), data_->contactViolationVector_[i], data_->contactPointVelVector_[i],
        data_->segment_frames_);
  }
}

void EvaluationManager::computeStabilityCosts(int begin, int end)
{
  int safe_begin = max(1, begin);
  int safe_end = min(num_points_ - 1, end);
  for (int point = safe_begin; point < safe_end; point++)
  {
    INIT_TIME_MEASUREMENT(10)
    ADD_TIMER_POINT

    double state_contact_invariant_cost = 0.0;
    double state_physics_violation_cost = 0.0;
    if (planning_group_->name_ != "lower_body" && planning_group_->name_ != "whole_body")
    {
      data_->stateContactInvariantCost_[point] = state_contact_invariant_cost;
      data_->statePhysicsViolationCost_[point] = state_physics_violation_cost;
      continue;
    }

    int num_contacts = planning_group_->getNumContacts();
    std::vector<KDL::Vector> contact_forces(num_contacts);
    std::vector<KDL::Frame> contact_parent_frames(num_contacts);
    std::vector<double> contact_values(num_contacts);
    std::vector<KDL::Vector> contact_positions(num_contacts);
    for (int i = 0; i < num_contacts; ++i)
    {
      KDL::SegmentMap::const_iterator it_segment_link = robot_model_->getKDLTree()->getSegment(
          planning_group_->contactPoints_[i].getLinkName());
      it_segment_link = it_segment_link->second.parent;
      string parent_segment_name = it_segment_link->first;
      int segment_number = robot_model_->getForwardKinematicsSolver()->segmentNameToIndex(parent_segment_name);
      contact_parent_frames[i] = data_->segment_frames_[point][segment_number];

      planning_group_->contactPoints_[i].getPosition(point, contact_positions[i], data_->segment_frames_);
    }

    int phase = getGroupTrajectory()->getContactPhase(point);
    for (int i = 0; i < num_contacts; ++i)
      contact_values[i] = getGroupTrajectory()->getContactValue(phase, i);

    ADD_TIMER_POINT

    data_->contact_force_solver_(PlanningParameters::getInstance()->getFrictionCoefficient(), contact_forces,
        contact_positions, data_->wrenchSum_[point], contact_values, contact_parent_frames);

    ADD_TIMER_POINT

    for (int i = 0; i < num_contacts; ++i)
    {
      double cost = 0.0;
      for (int j = 0; j < 4; ++j)
        cost += data_->contactViolationVector_[i][point].data_[j] * data_->contactViolationVector_[i][point].data_[j];
      cost += 16.0 * KDL::dot(data_->contactPointVelVector_[i][point], data_->contactPointVelVector_[i][point]);
      state_contact_invariant_cost += contact_values[i] * cost;
    }

    KDL::Wrench contactWrench;
    for (int i = 0; i < num_contacts; ++i)
    {
      contactWrench.force += contact_forces[i];
      contactWrench.torque += contact_positions[i] * contact_forces[i];
    }

    if (STABILITY_COST_VERBOSE)
    {
      printf("\n");

      KDL::Vector root_pos = data_->segment_frames_[point][3].p;
      printf("%d Root : (%f %f %f) CoM : (%f %f %f)\n", point, root_pos.x(), root_pos.y(), root_pos.z(),
          data_->CoMPositions_[point].x(), data_->CoMPositions_[point].y(), data_->CoMPositions_[point].z());
      for (int i = 0; i < num_contacts; ++i)
      {
        KDL::Vector rel_pos = (contact_positions[i] - data_->CoMPositions_[point]);
        KDL::Vector contact_torque = rel_pos * contact_forces[i];
        printf("CP %d V:%f F:(%f %f %f) RT:(%f %f %f)xF=(%f %f %f) r:(%f %f %f) p:(%f %f %f)\n", i, contact_values[i],
            contact_forces[i].x(), contact_forces[0].y(), contact_forces[i].z(), rel_pos.x(), rel_pos.y(), rel_pos.z(),
            contact_torque.x(), contact_torque.y(), contact_torque.z(), contact_parent_frames[i].p.x(),
            contact_parent_frames[i].p.y(), contact_parent_frames[i].p.z(), contact_positions[i].x(),
            contact_positions[i].y(), contact_positions[i].z());
      }
    }

    KDL::Wrench violation = contactWrench + data_->wrenchSum_[point];
    state_physics_violation_cost = sqrt(
        violation.force.x() * violation.force.x() + violation.force.y() * violation.force.y()
            + violation.force.z() * violation.force.z() + violation.torque.x() * violation.torque.x()
            + violation.torque.y() * violation.torque.y() + violation.torque.z() * violation.torque.z());

    if (STABILITY_COST_VERBOSE)
    {
      printf("Gravity Force : (%f %f %f)\n", gravity_force_.x(), gravity_force_.y(), gravity_force_.z());
      printf("Inertia Force : (%f %f %f)\n", -total_mass_ * data_->CoMAccelerations_[point].x(),
          -total_mass_ * data_->CoMAccelerations_[point].y(), -total_mass_ * data_->CoMAccelerations_[point].z());

      printf("Wrench Torque : (%f %f %f)\n", data_->wrenchSum_[point].torque.x(), data_->wrenchSum_[point].torque.y(),
          data_->wrenchSum_[point].torque.z());

      printf("Violation : (%f %f %f) (%f %f %f)\n", violation.force.x(), violation.force.y(), violation.force.z(),
          violation.torque.x(), violation.torque.y(), violation.torque.z());

      printf("[%d] contactWrench (%f %f %f)(%f %f %f)\n", point, contactWrench.force.x(), contactWrench.force.y(),
          contactWrench.force.z(), contactWrench.torque.x(), contactWrench.torque.y(), contactWrench.torque.z());
      printf("[%d] violation (%f %f %f)(%f %f %f)\n", point, violation.force.x(), violation.force.y(),
          violation.force.z(), violation.torque.x(), violation.torque.y(), violation.torque.z());

      printf("[%d]CIcost:%f Pvcost:%f(%f,%f,%f,%f,%f,%f)\n", point, state_contact_invariant_cost,
          state_physics_violation_cost, violation.force.x(), violation.force.y(), violation.force.z(),
          violation.torque.x(), violation.torque.y(), violation.torque.z());
    }

    data_->stateContactInvariantCost_[point] = state_contact_invariant_cost;
    data_->statePhysicsViolationCost_[point] = state_physics_violation_cost;

    ADD_TIMER_POINT
    UPDATE_TIME
    PRINT_TIME(stability, 10000)
  }


}

void EvaluationManager::computeCollisionCosts(int begin, int end)
{
  const collision_detection::AllowedCollisionMatrix acm = data_->planning_scene_->getAllowedCollisionMatrix();

  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;
  collision_request.verbose = false;
  collision_request.contacts = true;
  collision_request.max_contacts = 1000;

  std::vector<double> positions;

  int num_all_joints = data_->kinematic_state_->getVariableCount();
  positions.resize(num_all_joints);

  int safe_begin = max(0, begin);
  int safe_end = min(num_points_, end);
  for (int i = safe_begin; i < safe_end; ++i)
  {
    double depthSum = 0.0;

    for (std::size_t k = 0; k < num_all_joints; k++)
    {
      positions[k] = (*getFullTrajectory())(i, k);
    }
    data_->kinematic_state_->setVariablePositions(&positions[0]);

    data_->kinematic_state_->updateCollisionBodyTransforms();

    data_->planning_scene_->getCollisionWorld()->checkRobotCollision(collision_request, collision_result,
        *data_->planning_scene_->getCollisionRobotUnpadded(), *data_->kinematic_state_, acm);

    const collision_detection::CollisionResult::ContactMap& contact_map = collision_result.contacts;
    for (collision_detection::CollisionResult::ContactMap::const_iterator it = contact_map.begin();
        it != contact_map.end(); ++it)
    {
      const collision_detection::Contact& contact = it->second[0];
      depthSum += contact.depth;
    }
    collision_result.clear();
    data_->stateCollisionCost_[i] = depthSum;
  }
}

void EvaluationManager::postprocess_ik()
{
  const double threshold = 0.1;
  const Eigen::MatrixXd& contactTrajectoryBlock = getGroupTrajectory()->getContactTrajectory();
  int num_contact_phases = getGroupTrajectory()->getNumContactPhases();
  std::vector<int> ik_ref_point(num_contact_phases);
  for (int j = 0; j < num_contacts_; ++j)
  {
    for (int i = 0; i < num_contact_phases; ++i)
    {
      ik_ref_point[i] = -1;
    }

    for (int i = num_contact_phases - 1; i >= 0; --i)
    {
      double contact_value = contactTrajectoryBlock(i, j);
      if (contact_value > threshold)
      {
        ik_ref_point[i] = num_contact_phases;
      }
      else
        break;
    }
    int ref_point = 0;
    for (int i = 1; i < num_contact_phases; ++i)
    {
      double contact_value = contactTrajectoryBlock(i - 1, j);
      if (contact_value > threshold && ik_ref_point[i] != num_contact_phases)
        ik_ref_point[i] = ref_point;
      else
        ref_point = i;
    }

    /*
     printf("Contact %d : ", j);
     for (int i = 0; i < num_contact_phases; ++i)
     {
     if (ik_ref_point[i] != -1)
     printf("%d -> %d ", i, ik_ref_point[i]);
     }
     printf("\n");
     */

    ////////////
    // ik
    // TODO:
    string ik_group_name;
    switch (j)
    {
    case 0:
      ik_group_name = "left_leg";
      break;
    case 1:
      ik_group_name = "right_leg";
      break;
    case 2:
      ik_group_name = "left_arm";
      break;
    case 3:
      ik_group_name = "right_arm";
      break;
    }
    for (int i = 1; i < num_contact_phases; ++i)
    {
      KDL::Frame contact_frame;
      if (ik_ref_point[i] != -1)
      {
        planning_group_->contactPoints_[j].getFrame(ik_ref_point[i] * getGroupTrajectory()->getContactPhaseStride(),
            contact_frame, data_->segment_frames_);

        // set kinematic_state to current joint values
        robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(robot_model_->getRobotModel()));
        int num_all_joints = kinematic_state->getVariableCount();
        std::vector<double> positions(num_all_joints);
        for (std::size_t k = 0; k < num_all_joints; k++)
        {
          positions[k] = (*getFullTrajectory())(i * getGroupTrajectory()->getContactPhaseStride(), k);
        }
        kinematic_state->setVariablePositions(&positions[0]);
        kinematic_state->update();

        // compute ik for ref_point end effector position
        const robot_state::JointModelGroup* joint_model_group = robot_model_->getRobotModel()->getJointModelGroup(
            ik_group_name);

        Eigen::Affine3d end_effector_state = Eigen::Affine3d::Identity();
        for (int r = 0; r < 3; ++r)
          for (int c = 0; c < 3; ++c)
            end_effector_state(r, c) = contact_frame.M(r, c);
        for (int r = 0; r < 3; ++r)
          end_effector_state(r, 3) = contact_frame.p(r);

        bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state, 10, 1.0);
        // Now, we can print out the IK solution (if found):
        if (found_ik)
        {
          std::vector<double> group_values;
          kinematic_state->copyJointGroupPositions(joint_model_group, group_values);
          kinematic_state->setVariablePositions(&positions[0]);
          kinematic_state->setJointGroupPositions(joint_model_group, group_values);

          double* state_pos = kinematic_state->getVariablePositions();
          for (std::size_t k = 0; k < num_all_joints; k++)
          {
            (*getFullTrajectory())(i * getGroupTrajectory()->getContactPhaseStride(), k) = state_pos[k];
          }
          ROS_INFO("[%d:%d] IK solution found", i, j);
        }
        else
        {
          ROS_INFO("[%d:%d] Did not find IK solution", i, j);
        }
      }
    }
  }
  getFullTrajectory()->updateFreePointsFromTrajectory();
  getGroupTrajectory()->copyFromFullTrajectory(*getFullTrajectory());
  getGroupTrajectory()->updateTrajectoryFromFreePoints();
}

double EvaluationManager::getTrajectoryCost(bool verbose)
{
  if (verbose)
    data_->costAccumulator_.print(*iteration_);
  return data_->costAccumulator_.getTrajectoryCost();
}

}
