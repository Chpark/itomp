#include <ros/ros.h>
#include <itomp_cio_planner/optimization/evaluation_data.h>
#include <itomp_cio_planner/optimization/evaluation_manager.h>
#include <itomp_cio_planner/model/itomp_planning_group.h>
#include <itomp_cio_planner/util/planning_parameters.h>

using namespace std;
using namespace Eigen;

namespace itomp_cio_planner
{

EvaluationData::EvaluationData()
{

}

EvaluationData::~EvaluationData()
{

}

void EvaluationData::initialize(ItompCIOTrajectory *full_trajectory, ItompCIOTrajectory *group_trajectory,
    ItompRobotModel *robot_model, const ItompPlanningGroup *planning_group, const EvaluationManager* evaluation_manager, int num_mass_segments)
{
  full_trajectory_ = full_trajectory;
  group_trajectory_ = group_trajectory;

  kdl_joint_array_.resize(robot_model->getKDLTree()->getNrOfJoints());

  int num_joints = group_trajectory_->getNumJoints();
  int num_contacts = group_trajectory_->getNumContacts();
  int num_points = group_trajectory->getNumPoints();
  int num_contact_points = group_trajectory->getNumContactPhases() + 1;

  // set up the joint costs:
  joint_costs_.reserve(num_joints);

  double max_cost_scale = 0.0;
  ros::NodeHandle nh("~");
  for (int i = 0; i < num_joints; i++)
  {
    double joint_cost = 1.0;
    std::string joint_name = planning_group->group_joints_[i].joint_name_;
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
  for (int i = 0; i < num_joints; i++)
  {
    joint_costs_[i].scale(max_cost_scale);
  }

  joint_axis_.resize(num_points, std::vector<KDL::Vector>(robot_model->getKDLTree()->getNrOfJoints()));
  joint_pos_.resize(num_points, std::vector<KDL::Vector>(robot_model->getKDLTree()->getNrOfJoints()));
  segment_frames_.resize(num_points, std::vector<KDL::Frame>(robot_model->getKDLTree()->getNrOfSegments()));

  state_is_in_collision_.resize(num_points);

  state_validity_.resize(num_points);
  for (int i = 0; i < num_points; ++i)
    state_validity_[i] = true;
  dynamic_obstacle_cost_ = Eigen::VectorXd::Zero(num_points);

  stateContactInvariantCost_.resize(num_points);
  statePhysicsViolationCost_.resize(num_points);
  stateCollisionCost_.resize(num_points);

  linkPositions_.resize(num_mass_segments);
  linkVelocities_.resize(num_mass_segments);
  linkAngularVelocities_.resize(num_mass_segments);
  for (int i = 0; i < num_mass_segments; ++i)
  {
    linkPositions_[i].resize(num_points);
    linkVelocities_[i].resize(num_points);
    linkAngularVelocities_[i].resize(num_points);
  }
  CoMPositions_.resize(num_points);
  CoMVelocities_.resize(num_points);
  CoMAccelerations_.resize(num_points);
  CoMAccelerations_.resize(num_points);
  AngularMomentums_.resize(num_points);
  Torques_.resize(num_points);
  wrenchSum_.resize(num_points);

  contactViolationVector_.resize(num_contacts);
  contactPointVelVector_.resize(num_contacts);
  for (int i = 0; i < num_contacts; ++i)
  {
    contactViolationVector_[i].resize(num_points);
    contactPointVelVector_[i].resize(num_points);
  }

  costAccumulator_.addCost(TrajectoryCost::CreateTrajectoryCost(TrajectoryCost::COST_SMOOTHNESS));
  costAccumulator_.addCost(TrajectoryCost::CreateTrajectoryCost(TrajectoryCost::COST_COLLISION));
  costAccumulator_.addCost(TrajectoryCost::CreateTrajectoryCost(TrajectoryCost::COST_VALIDITY));
  costAccumulator_.addCost(TrajectoryCost::CreateTrajectoryCost(TrajectoryCost::COST_CONTACT_INVARIANT));
  costAccumulator_.addCost(TrajectoryCost::CreateTrajectoryCost(TrajectoryCost::COST_PHYSICS_VIOLATION));
  costAccumulator_.addCost(TrajectoryCost::CreateTrajectoryCost(TrajectoryCost::COST_GOAL_POSE));
  costAccumulator_.addCost(TrajectoryCost::CreateTrajectoryCost(TrajectoryCost::COST_COM));
  costAccumulator_.init(this);
}

}
