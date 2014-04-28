#include <ros/ros.h>
#include <itomp_cio_planner/optimization/evaluation_manager.h>
#include <itomp_cio_planner/model/itomp_planning_group.h>
#include <itomp_cio_planner/cost/trajectory_cost_accumulator.h>
#include <itomp_cio_planner/contact/ground_manager.h>
#include <itomp_cio_planner/visualization/visualization_manager.h>
#include <itomp_cio_planner/contact/contact_force_solver.h>
#include <itomp_cio_planner/util/min_jerk_trajectory.h>
#include <itomp_cio_planner/util/planning_parameters.h>
#include <itomp_cio_planner/util/vector_util.h>
#include <visualization_msgs/MarkerArray.h>
#include "dlib/optimization.h"

using namespace std;
using namespace Eigen;

const static double SENSOR_NOISE = 0.18;

namespace itomp_cio_planner
{
static int LeftLegStart = 0;

EvaluationManager::EvaluationManager(int* iteration) :
    iteration_(iteration)
{

}

EvaluationManager::~EvaluationManager()
{

}

void EvaluationManager::initialize(ItompCIOTrajectory *full_trajectory, ItompCIOTrajectory *group_trajectory,
    const ItompRobotModel *robot_model, const ItompPlanningGroup *planning_group, double planning_start_time,
    double trajectory_start_time, TrajectoryCostAccumulator *costAccumulator)
{
  full_trajectory_ = full_trajectory;
  group_trajectory_ = group_trajectory;

  planning_start_time_ = planning_start_time;
  trajectory_start_time_ = trajectory_start_time;

  robot_model_ = robot_model;
  planning_group_ = planning_group;
  robot_name_ = robot_model_->getRobotName();

  costAccumulator_ = costAccumulator;

  kdl_joint_array_.resize(robot_model_->getKDLTree()->getNrOfJoints());
  kdl_vel_joint_array_.resize(robot_model_->getKDLTree()->getNrOfJoints());
  kdl_acc_joint_array_.resize(robot_model_->getKDLTree()->getNrOfJoints());

  kdl_group_joint_array_.resize(group_trajectory_->getNumJoints());
  kdl_group_vel_joint_array_.resize(group_trajectory_->getNumJoints());
  kdl_group_acc_joint_array_.resize(group_trajectory_->getNumJoints());
  kdl_group_torque_joint_array_.resize(group_trajectory_->getNumJoints());

  // init some variables:
  num_vars_free_ = group_trajectory_->getNumFreePoints();
  num_vars_all_ = group_trajectory_->getNumPoints();
  num_joints_ = group_trajectory_->getNumJoints();
  num_contacts_ = group_trajectory_->getNumContacts();
  free_vars_start_ = group_trajectory_->getStartIndex();
  free_vars_end_ = group_trajectory_->getEndIndex();

  ROS_INFO_STREAM("Setting free vars start to " << free_vars_start_ << " end " << free_vars_end_);

  // set up joint index:
  group_joint_to_kdl_joint_index_.resize(num_joints_);
  for (int i = 0; i < num_joints_; ++i)
    group_joint_to_kdl_joint_index_[i] = planning_group_->group_joints_[i].kdl_joint_index_;

  // set up the joint costs:
  joint_costs_.reserve(num_joints_);

  double max_cost_scale = 0.0;
  ros::NodeHandle nh("~");
  for (int i = 0; i < num_joints_; i++)
  {
    double joint_cost = 1.0;
    std::string joint_name = planning_group_->group_joints_[i].joint_name_;
    nh.param("joint_costs/" + joint_name, joint_cost, 1.0);
    std::vector<double> derivative_costs(NUM_DIFF_RULES);
    derivative_costs[DIFF_RULE_VELOCITY] = joint_cost * PlanningParameters::getInstance()->getSmoothnessCostVelocity();
    derivative_costs[DIFF_RULE_ACCELERATION] = joint_cost
        * PlanningParameters::getInstance()->getSmoothnessCostAcceleration();
    derivative_costs[DIFF_RULE_JERK] = joint_cost * PlanningParameters::getInstance()->getSmoothnessCostJerk();

    joint_costs_.push_back(
        SmoothnessCost(*group_trajectory_, i, derivative_costs, PlanningParameters::getInstance()->getRidgeFactor()));
    double cost_scale = joint_costs_[i].getMaxQuadCostInvValue();
    if (max_cost_scale < cost_scale)
      max_cost_scale = cost_scale;
  }

  // scale the smoothness costs
  for (int i = 0; i < num_joints_; i++)
  {
    joint_costs_[i].scale(max_cost_scale);
  }

  joint_axis_.resize(num_vars_all_, std::vector<KDL::Vector>(robot_model_->getKDLTree()->getNrOfJoints()));
  joint_pos_.resize(num_vars_all_, std::vector<KDL::Vector>(robot_model_->getKDLTree()->getNrOfJoints()));
  segment_frames_.resize(num_vars_all_, std::vector<KDL::Frame>(robot_model_->getKDLTree()->getNrOfSegments()));
  // create the eigen maps:
  itomp_cio_planner::kdlVecVecToEigenVecVec(joint_axis_, joint_axis_eigen_, 3, 1);
  itomp_cio_planner::kdlVecVecToEigenVecVec(joint_pos_, joint_pos_eigen_, 3, 1);

  is_collision_free_ = false;
  state_is_in_collision_.resize(num_vars_all_);

  // initialize exact collision checking stuff
  /*
   robot_state_.joint_state.name = robot_model_->getJointNames();
   robot_state_.joint_state.position.resize(robot_state_.joint_state.name.size());
   robot_state_.joint_state.velocity.resize(robot_state_.joint_state.name.size());
   robot_state_.joint_state.header.frame_id = robot_model_->getReferenceFrame();
   */

  state_validity_.resize(num_vars_all_);
  for (int i = 0; i < num_vars_all_; ++i)
    state_validity_[i] = true;

  // Initialize visualizer
  VisualizationManager::getInstance()->setPlanningGroup(*robot_model_, planning_group_->name_);
  vis_marker_pub_ = VisualizationManager::getInstance()->getVisualizationMarkerPublisher();
  vis_marker_array_pub_ = VisualizationManager::getInstance()->getVisualizationMarkerArrayPublisher();

  last_trajectory_collision_free_ = false;
  dynamic_obstacle_cost_ = Eigen::VectorXd::Zero(num_vars_all_);

  computeMassAndGravityForce();

  stateContactInvariantCost_.resize(num_vars_free_);
  statePhysicsViolationCost_.resize(num_vars_free_);

  GroundManager::getInstance().init();
}

double EvaluationManager::evaluate(vector<Eigen::VectorXd>& parameters, vector<Eigen::VectorXd>& contact_parameters,
    Eigen::VectorXd& costs)
{
  // copy the parameters into group_trajectory_:
  for (int d = 0; d < num_joints_; ++d)
  {
    group_trajectory_->getFreeJointTrajectoryBlock(d) = parameters[d];
  }
  for (int d = 0; d < num_contacts_; ++d)
  {
    group_trajectory_->getFreeContactTrajectoryBlock(d) = contact_parameters[d];
  }

  // respect joint limits:
  handleJointLimits();

  // copy to full traj:
  updateFullTrajectory();

  // do forward kinematics:
  last_trajectory_collision_free_ = performForwardKinematics();

  computeTrajectoryValidity();
  last_trajectory_collision_free_ &= trajectory_validity_;

  costAccumulator_->compute(this);

  last_trajectory_collision_free_ &= costAccumulator_->isFeasible();

  for (int i = 0; i < num_vars_free_; i++)
  {
    costs(i) = costAccumulator_->getWaypointCost(i);
  }

  // handle joint limits can modify trajectory
  for (int d = 0; d < num_joints_; ++d)
  {
    parameters[d] = group_trajectory_->getFreeJointTrajectoryBlock(d);
  }

  static int count = 0;
  if (++count % 1000 == 0)
  //if (group_trajectory_->getContactValue(1, 0) != 0 || group_trajectory_->getContactValue(1, 1) != 0 ||
  //  group_trajectory_->getContactValue(2, 0) != 0 || group_trajectory_->getContactValue(2, 1) != 0)
  {
    costAccumulator_->print(0);
    printf("Contact Values :\n");
    for (int i = 0; i < group_trajectory_->getNumFreeContactPhases(); ++i)
    {
      for (int j = 0; j < group_trajectory_->getNumContacts(); ++j)
        printf("%f ", group_trajectory_->getContactValue(i + 1, j));
      printf("\n");
    }
  }
  return costAccumulator_->getTrajectoryCost();

}

void EvaluationManager::render(int trajectory_index)
{
  if (PlanningParameters::getInstance()->getAnimateEndeffector())
  {
    VisualizationManager::getInstance()->animateEndeffector(trajectory_index, num_vars_free_, free_vars_start_,
        segment_frames_, state_validity_, false);
    VisualizationManager::getInstance()->animateCoM(num_vars_free_, free_vars_start_, CoMPositions_, false);
  }
  if (PlanningParameters::getInstance()->getAnimatePath())
  {
    VisualizationManager::getInstance()->animatePath(free_vars_start_, free_vars_end_);
  }
}

void EvaluationManager::computeMassAndGravityForce()
{
  totalMass_ = 0.0;
  const KDL::SegmentMap& segmentMap = robot_model_->getKDLTree()->getSegments();
  numMassSegments_ = 0;
  for (KDL::SegmentMap::const_iterator it = segmentMap.begin(); it != segmentMap.end(); ++it)
  {
    const KDL::Segment& segment = it->second.segment;
    double mass = segment.getInertia().getMass();
    if (mass == 0)
      continue;

    totalMass_ += mass;
    masses_.push_back(mass);

    ++numMassSegments_;
  }
  gravityForce_ = totalMass_ * KDL::Vector(0.0, 0.0, -9.8);

  linkPositions_.resize(numMassSegments_);
  linkVelocities_.resize(numMassSegments_);
  linkAngularVelocities_.resize(numMassSegments_);
  for (int i = 0; i < numMassSegments_; ++i)
  {
    linkPositions_[i].resize(num_vars_all_);
    linkVelocities_[i].resize(num_vars_all_);
    linkAngularVelocities_[i].resize(num_vars_all_);
  }
  CoMPositions_.resize(num_vars_all_);
  CoMVelocities_.resize(num_vars_all_);
  CoMAccelerations_.resize(num_vars_all_);
  CoMAccelerations_.resize(num_vars_all_);
  AngularMomentums_.resize(num_vars_all_);
  Torques_.resize(num_vars_all_);
  wrenchSum_.resize(num_vars_all_);

  int num_contacts = group_trajectory_->getNumContacts();
  tmpContactViolationVector_.resize(num_contacts);
  tmpContactPointVelVector_.resize(num_contacts);
  for (int i = 0; i < num_contacts; ++i)
  {
    tmpContactViolationVector_[i].resize(num_vars_all_);
    tmpContactPointVelVector_[i].resize(num_vars_all_);
  }
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

    bool violation = false;
    do
    {
      double max_abs_violation = 1e-6;
      double max_violation = 0.0;
      int max_violation_index = 0;
      violation = false;
      for (int i = free_vars_start_; i <= free_vars_end_; i++)
      {
        double amount = 0.0;
        double absolute_amount = 0.0;
        if ((*group_trajectory_)(i, joint) > joint_max)
        {
          amount = joint_max - (*group_trajectory_)(i, joint);
          absolute_amount = fabs(amount);
        }
        else if ((*group_trajectory_)(i, joint) < joint_min)
        {
          amount = joint_min - (*group_trajectory_)(i, joint);
          absolute_amount = fabs(amount);
        }
        if (absolute_amount > max_abs_violation)
        {
          max_abs_violation = absolute_amount;
          max_violation = amount;
          max_violation_index = i;
          violation = true;
        }
      }

      if (violation)
      {
        int free_var_index = max_violation_index - free_vars_start_;
        double multiplier = max_violation
            / joint_costs_[joint].getQuadraticCostInverse()(free_var_index, free_var_index);
        group_trajectory_->getFreeJointTrajectoryBlock(joint) += multiplier
            * joint_costs_[joint].getQuadraticCostInverse().col(free_var_index);
      }
      if (++count > 10)
        break;
    } while (violation);
  }
}

void EvaluationManager::updateFullTrajectory()
{
  full_trajectory_->updateFromGroupTrajectory(*group_trajectory_);
}

bool EvaluationManager::performForwardKinematics()
{
  double invTime = 1.0 / group_trajectory_->getDiscretization();
  double invTimeSq = invTime * invTime;

  is_collision_free_ = true;

  // calculate the forward kinematics for the fixed states only in the first iteration:
  int start = free_vars_start_;
  int end = free_vars_end_;
  if (getIteration() <= 0)
  {
    start = 0;
    end = num_vars_all_ - 1;

    // update segment_frames of the goal
    int full_traj_index = group_trajectory_->getFullTrajectoryIndex(end);
    full_trajectory_->getTrajectoryPointKDL(full_traj_index, kdl_joint_array_);
    planning_group_->fk_solver_->JntToCartFull(kdl_joint_array_, joint_pos_[end], joint_axis_[end],
        segment_frames_[end]);
  }

  // for each point in the trajectory
  for (int i = start; i <= end; ++i)
  {
    int full_traj_index = group_trajectory_->getFullTrajectoryIndex(i);
    full_trajectory_->getTrajectoryPointKDL(full_traj_index, kdl_joint_array_);
    // update kdl_joint_array with vel, acc
    if (i < free_vars_start_)
    {
      for (int j = 0; j < planning_group_->num_joints_; j++)
      {
        int target_joint = planning_group_->group_joints_[j].kdl_joint_index_;
        kdl_joint_array_(target_joint) = (*group_trajectory_)(i, j);
      }
    }

    //computeBaseFrames(kdl_joint_array_, i);

    if (i == 0)
      planning_group_->fk_solver_->JntToCartFull(kdl_joint_array_, joint_pos_[i], joint_axis_[i], segment_frames_[i]);
    else
      planning_group_->fk_solver_->JntToCartPartial(kdl_joint_array_, joint_pos_[i], joint_axis_[i],
          segment_frames_[i]);

    state_is_in_collision_[i] = false;

    if (state_is_in_collision_[i])
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

  for (int i = free_vars_start_; i <= free_vars_end_; i++)
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
     double point_time = full_trajectory_->getDiscretization() * (i - free_vars_start_);
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

    state_validity_[i] = valid;
    if (!valid)
      trajectory_validity_ = false;

  }

}

void EvaluationManager::updateCoM(int point)
{
  const KDL::SegmentMap& segmentMap = robot_model_->getKDLTree()->getSegments();
  // compute CoM, p_j
  int massSegmentIndex = 0;
  CoMPositions_[point] = KDL::Vector::Zero();
  for (KDL::SegmentMap::const_iterator it = segmentMap.begin(); it != segmentMap.end(); ++it)
  {
    const KDL::Segment& segment = it->second.segment;
    double mass = segment.getInertia().getMass();
    int sn = robot_model_->getForwardKinematicsSolver()->segmentNameToIndex(segment.getName());
    const KDL::Vector& pos = segment_frames_[point][sn] * segment.getInertia().getCOG();
    if (mass == 0.0)
      continue;

    CoMPositions_[point] += pos * mass;
    linkPositions_[massSegmentIndex][point] = pos;
    ++massSegmentIndex;
  }
  CoMPositions_[point] = CoMPositions_[point] / totalMass_;
}

static bool STABILITY_COST_VERBOSE = false;

void EvaluationManager::computeWrenchSum()
{
  if (planning_group_->name_ != "lower_body" && planning_group_->name_ != "whole_body")
    return;

  int start = 0;
  int end = num_vars_all_ - 1;
  if (getIteration() > 0)
  {
    start = free_vars_start_;
    end = free_vars_end_;
  }

  // compute CoM, p_j
  for (int point = start; point <= end; ++point)
  {
    updateCoM(point);
  }

  // compute \dot{CoM} \ddot{CoM}
  itomp_cio_planner::getVectorVelocitiesAndAccelerations(free_vars_start_, free_vars_end_,
      group_trajectory_->getDiscretization(), CoMPositions_, CoMVelocities_, CoMAccelerations_, KDL::Vector::Zero());
  // compute \dot{p_j}
  for (int i = 0; i < numMassSegments_; ++i)
  {
    itomp_cio_planner::getVectorVelocities(free_vars_start_, free_vars_end_, group_trajectory_->getDiscretization(),
        linkPositions_[i], linkVelocities_[i], KDL::Vector::Zero());
  }

  // debug
  if (STABILITY_COST_VERBOSE)
  {
    printf("CoMPos CoMVel CoMAcc \n");
    for (int i = free_vars_start_; i < free_vars_end_; ++i)
    {
      printf("%f %f %f %f %f %f %f %f %f\n", CoMPositions_[i].x(), CoMPositions_[i].y(), CoMPositions_[i].z(),
          CoMVelocities_[i].x(), CoMVelocities_[i].y(), CoMVelocities_[i].z(), CoMAccelerations_[i].x(),
          CoMAccelerations_[i].y(), CoMAccelerations_[i].z());
    }
  }

  // TODO: compute angular velocities = (cur-prev)/time
  const KDL::SegmentMap& segmentMap = robot_model_->getKDLTree()->getSegments();
  const double invTime = 1.0 / group_trajectory_->getDiscretization();
  for (int point = free_vars_start_; point <= free_vars_end_; ++point)
  {
    int massSegmentIndex = 0;
    for (KDL::SegmentMap::const_iterator it = segmentMap.begin(); it != segmentMap.end(); ++it)
    {
      const KDL::Segment& segment = it->second.segment;
      double mass = segment.getInertia().getMass();
      int sn = robot_model_->getForwardKinematicsSolver()->segmentNameToIndex(segment.getName());
      const KDL::Vector& pos = segment_frames_[point][sn] * segment.getInertia().getCOG();
      if (mass == 0.0)
        continue;

      const KDL::Rotation& prevRotation = segment_frames_[point - 1][sn].M;
      const KDL::Rotation& curRotation = segment_frames_[point][sn].M;
      const KDL::Rotation& rotDiff = curRotation * prevRotation.Inverse();
      linkAngularVelocities_[massSegmentIndex][point] = rotDiff.GetRot() * invTime;
      ++massSegmentIndex;
    }
  }

  // compute angular momentum
  for (int point = free_vars_start_; point <= free_vars_end_; ++point)
  {
    AngularMomentums_[point] = KDL::Vector(0.0, 0.0, 0.0);

    int massSegmentIndex = 0;
    for (KDL::SegmentMap::const_iterator it = segmentMap.begin(); it != segmentMap.end(); ++it)
    {
      const KDL::Segment& segment = it->second.segment;
      double mass = segment.getInertia().getMass();
      if (mass == 0.0)
        continue;

      int sn = robot_model_->getForwardKinematicsSolver()->segmentNameToIndex(segment.getName());
      KDL::Vector angularVelTerm = (segment_frames_[point][sn] * segment.getInertia()).getRotationalInertia()
          * linkAngularVelocities_[massSegmentIndex][point];

      AngularMomentums_[point] += masses_[massSegmentIndex]
          * (linkPositions_[massSegmentIndex][point] - CoMPositions_[point]) * linkVelocities_[massSegmentIndex][point]
          + angularVelTerm;
      ++massSegmentIndex;
    }
  }
  // compute torques
  itomp_cio_planner::getVectorVelocities(free_vars_start_, free_vars_end_, group_trajectory_->getDiscretization(),
      AngularMomentums_, Torques_, KDL::Vector::Zero());

  // compute wrench sum (gravity wrench + inertia wrench)
  for (int point = free_vars_start_; point <= free_vars_end_; ++point)
  {
    wrenchSum_[point].force = gravityForce_;
    wrenchSum_[point].torque = CoMPositions_[point] * gravityForce_;

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
    planning_group_->contactPoints_[i].updateContactViolationVector(free_vars_start_, free_vars_end_,
        group_trajectory_->getDiscretization(), tmpContactViolationVector_[i], tmpContactPointVelVector_[i],
        segment_frames_);
  }

}

void EvaluationManager::computeStabilityCosts()
{
  for (int point = free_vars_start_; point <= free_vars_end_; point++)
  {
    double state_contact_invariant_cost = 0.0;
    double state_physics_violation_cost = 0.0;
    if (planning_group_->name_ != "lower_body" && planning_group_->name_ != "whole_body")
    {
      stateContactInvariantCost_[point - free_vars_start_] = state_contact_invariant_cost;
      statePhysicsViolationCost_[point - free_vars_start_] = state_physics_violation_cost;
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
      contact_parent_frames[i] = segment_frames_[point][segment_number];

      planning_group_->contactPoints_[i].getPosition(point, contact_positions[i], segment_frames_);
    }

    /*
     if (point < free_vars_start_ || point > free_vars_end_)
     {
     contact_values[0] = 10.0;
     contact_values[1] = 10.0;
     }
     else
     {
     contact_values[0] = (group_trajectory_->getContactPhase(point) + LeftLegStart) % 2 == 0 ? 10.0 : 0.0;
     contact_values[1] = (group_trajectory_->getContactPhase(point) + LeftLegStart) % 2 == 0 ? 0.0 : 10.0;
     }
     contact_values[2] = contact_values[3] = 0.0;
     */

    int phase = group_trajectory_->getContactPhase(point);
    for (int i = 0; i < num_contacts; ++i)
      contact_values[i] = group_trajectory_->getContactValue(phase, i);

    /*
    switch (phase)
    //(phase-1)/2+1)
    {
    case 1:
      contact_values[0] = 0;
      contact_values[2] = contact_values[1] = contact_values[3] = 10.0;
      break;
    case 2:
      contact_values[1] = 0;
      contact_values[3] = contact_values[1] = contact_values[2] = 10.0;
      break;
    case 3:
      contact_values[2] = 0;
      contact_values[0] = contact_values[1] = contact_values[3] = 10.0;
      break;
    case 4:
      contact_values[3] = 0;
      contact_values[0] = contact_values[1] = contact_values[2] = 10.0;
      break;
    default:
      contact_values[2] = contact_values[3] = 0;
      contact_values[0] = contact_values[1] = 10.0;
      break;
    }
    */

    solveContactForces(PlanningParameters::getInstance()->getFrictionCoefficient(), contact_forces, contact_positions,
        wrenchSum_[point], contact_values, contact_parent_frames);

    for (int i = 0; i < num_contacts; ++i)
    {
      double cost = (tmpContactViolationVector_[i][point].transpose() * tmpContactViolationVector_[i][point]).value()
          + 16 * KDL::dot(tmpContactPointVelVector_[i][point], tmpContactPointVelVector_[i][point]);
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
      KDL::Vector root_pos = segment_frames_[point][3].p;
      ROS_INFO(
          "%d Root : (%f %f %f) CoM : (%f %f %f)", point, root_pos.x(), root_pos.y(), root_pos.z(), CoMPositions_[point].x(), CoMPositions_[point].y(), CoMPositions_[point].z());
      for (int i = 0; i < num_contacts; ++i)
      {
        ROS_INFO(
            "CP %d[%f,(%f %f %f)] : (%f %f %f) (%f %f %f)", i, contact_values[i], contact_forces[i].x(), contact_forces[0].y(), contact_forces[i].z(), contact_parent_frames[i].p.x(), contact_parent_frames[i].p.y(), contact_parent_frames[i].p.z(), contact_positions[i].x(), contact_positions[i].y(), contact_positions[i].z());
      }
    }

    KDL::Wrench violation = contactWrench + wrenchSum_[point];
    state_physics_violation_cost = sqrt(
        violation.force.x() * violation.force.x() + violation.force.y() * violation.force.y()
            + violation.force.z() * violation.force.z() + violation.torque.x() * violation.torque.x()
            + violation.torque.y() * violation.torque.y() + violation.torque.z() * violation.torque.z());

    if (STABILITY_COST_VERBOSE)
    {
      ROS_INFO(
          "Gravity Force : (%f %f %f), Inertia Force : (%f %f %f)", gravityForce_.x(), gravityForce_.y(), gravityForce_.z(), -totalMass_ * CoMAccelerations_[point].x(), -totalMass_ * CoMAccelerations_[point].y(), -totalMass_ * CoMAccelerations_[point].z());
      ROS_INFO(
          "Violation : (%f %f %f) (%f %f %f)", violation.force.x(), violation.force.y(), violation.force.z(), violation.torque.x(), violation.torque.y(), violation.torque.z());

      ROS_INFO(
          "[%d] contactWrench (%f %f %f)(%f %f %f)", point, contactWrench.force.x(), contactWrench.force.y(), contactWrench.force.z(), contactWrench.torque.x(), contactWrench.torque.y(), contactWrench.torque.z());
      ROS_INFO(
          "[%d] violation (%f %f %f)(%f %f %f)", point, violation.force.x(), violation.force.y(), violation.force.z(), violation.torque.x(), violation.torque.y(), violation.torque.z());

      ROS_INFO(
          "[%d]CIcost:%f Pvcost:%f(%f,%f,%f,%f,%f,%f)", point, state_contact_invariant_cost, state_physics_violation_cost, violation.force.x(), violation.force.y(), violation.force.z(), violation.torque.x(), violation.torque.y(), violation.torque.z());
    }

    stateContactInvariantCost_[point - free_vars_start_] = state_contact_invariant_cost;
    statePhysicsViolationCost_[point - free_vars_start_] = state_physics_violation_cost;
  }
}

typedef dlib::matrix<double, 0, 1> column_vector;

vector<Eigen::VectorXd> parameters_;
vector<Eigen::VectorXd> contact_parameters_;
Eigen::VectorXd costs_;
EvaluationManager* evaluation_manager_;

int num_dimensions_;
int num_time_steps_;
int num_contact_dimensions_;
int num_contact_time_steps_;

class test_function
{
public:

  test_function(EvaluationManager* evaluation_manager, int num_dimensions, int num_time_steps,
      int num_contact_dimensions, int num_contact_time_steps)
  {
    evaluation_manager_ = evaluation_manager;
    num_dimensions_ = num_dimensions;
    num_time_steps_ = num_time_steps;
    num_contact_dimensions_ = num_contact_dimensions;
    num_contact_time_steps_ = num_contact_time_steps;

    parameters_.resize(num_dimensions_, Eigen::VectorXd::Zero(num_time_steps_));
    contact_parameters_.resize(num_contact_dimensions_, Eigen::VectorXd::Zero(num_contact_time_steps_));
    costs_ = Eigen::VectorXd::Zero(num_time_steps_);
  }

  double operator()(const column_vector& variables) const
  {
    int readIndex = 0;
    // copy to group_trajectory_:
    for (int d = 0; d < num_dimensions_; ++d)
    {
      for (int i = 0; i < num_time_steps_; ++i)
      {
        parameters_[d](i) = variables(readIndex + i, 0);
      }
      readIndex += num_time_steps_;
    }
    for (int d = 0; d < num_contact_dimensions_; ++d)
    {
      for (int i = 0; i < num_contact_time_steps_; ++i)
      {
        contact_parameters_[d](i) = abs(variables(readIndex + i, 0));
      }
      readIndex += num_contact_time_steps_;
    }

    return evaluation_manager_->evaluate(parameters_, contact_parameters_, costs_);
  }

private:
};

#include <itomp_cio_planner/util/multivariate_gaussian.h>

void EvaluationManager::optimize_nlp()
{
  int num_contact_vars_free = getGroupTrajectory()->getNumContactPhases() - 2;
  int num_variables = num_joints_ * num_vars_free_ + num_contacts_ * num_contact_vars_free;

  column_vector variables(num_variables);

  int writeIndex = 0;
  // copy from group_trajectory_:
  for (int d = 0; d < num_joints_; ++d)
  {
    const Eigen::VectorXd& freeTrajectoryBlock = group_trajectory_->getFreeJointTrajectoryBlock(d);
    for (int i = 0; i < num_vars_free_; ++i)
    {
      variables(writeIndex + i, 0) = freeTrajectoryBlock(i);
    }
    writeIndex += num_vars_free_;
  }
  for (int d = 0; d < num_contacts_; ++d)
  {
    const Eigen::VectorXd& freeContactTrajectoryBlock = group_trajectory_->getFreeContactTrajectoryBlock(d);
    for (int i = 0; i < num_contact_vars_free; ++i)
    {
      variables(writeIndex + i, 0) = freeContactTrajectoryBlock(i);
    }
    writeIndex += num_contact_vars_free;
  }

  dlib::find_min_using_approximate_derivatives(dlib::bfgs_search_strategy(),
      dlib::objective_delta_stop_strategy(1e-7).be_verbose(),
      test_function(this, num_joints_, num_vars_free_, num_contacts_, num_contact_vars_free), variables, -1);

  STABILITY_COST_VERBOSE = true;
  costAccumulator_->compute(this);
  costAccumulator_->print(0);
  STABILITY_COST_VERBOSE = false;
}

}
