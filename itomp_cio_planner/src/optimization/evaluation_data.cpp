#include <ros/ros.h>
#include <itomp_cio_planner/optimization/evaluation_data.h>
#include <itomp_cio_planner/optimization/evaluation_manager.h>
#include <itomp_cio_planner/model/itomp_planning_group.h>
#include <itomp_cio_planner/util/planning_parameters.h>
#include <boost/variant/get.hpp>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shapes.h>

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
    ItompRobotModel *robot_model, const ItompPlanningGroup *planning_group, const EvaluationManager* evaluation_manager,
    int num_mass_segments)
{
  full_trajectory_ = full_trajectory;
  group_trajectory_ = group_trajectory;

  robot_model_ = robot_model;
  planning_scene_.reset(new planning_scene::PlanningScene(robot_model->getRobotModel()));
  kinematic_state_.reset(new robot_state::RobotState(robot_model->getRobotModel()));
  initStaticEnvironment();

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

  fk_solver_ = *planning_group->fk_solver_.get();
}

void EvaluationData::initStaticEnvironment()
{
  collision_detection::AllowedCollisionMatrix acm = planning_scene_->getAllowedCollisionMatrix();
  string environment_file = PlanningParameters::getInstance()->getEnvironmentModel();
  if (!environment_file.empty())
  {
    vector<double> environment_position = PlanningParameters::getInstance()->getEnvironmentModelPosition();
    double scale = PlanningParameters::getInstance()->getEnvironmentModelScale();
    environment_position.resize(3, 0);

    // Collision object
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = robot_model_->getRobotModel()->getModelFrame();
    collision_object.id = "environment";
    geometry_msgs::Pose pose;
    pose.position.x = environment_position[0];
    pose.position.y = environment_position[1];
    pose.position.z = environment_position[2];
    pose.orientation.x = sqrt(0.5);
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = sqrt(0.5);

    shapes::Mesh* shape = shapes::createMeshFromResource("package://move_itomp/meshes/" + environment_file);
    shapes::ShapeMsg mesh_msg;
    shapes::constructMsgFromShape(shape, mesh_msg);
    shape_msgs::Mesh mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

    collision_object.meshes.push_back(mesh);
    collision_object.mesh_poses.push_back(pose);

    collision_object.operation = collision_object.ADD;
    moveit_msgs::PlanningScene planning_scene_msg;
    planning_scene_msg.world.collision_objects.push_back(collision_object);
    planning_scene_msg.is_diff = true;
    planning_scene_->setPlanningSceneDiffMsg(planning_scene_msg);

    acm.setEntry(true);
  }
}

EvaluationData* EvaluationData::clone() const
{
  EvaluationData* new_data = new EvaluationData();

  *new_data = *this;

  new_data->group_trajectory_ = new ItompCIOTrajectory(*group_trajectory_);
  new_data->full_trajectory_ = new ItompCIOTrajectory(*full_trajectory_);

  new_data->fk_solver_.reset();

  new_data->planning_scene_.reset(new planning_scene::PlanningScene(robot_model_->getRobotModel()));
  new_data->initStaticEnvironment();
  new_data->kinematic_state_.reset(new robot_state::RobotState(robot_model_->getRobotModel()));

  return new_data;
}

void EvaluationData::deepCopy(const EvaluationData& data)
{
  // store pointers
  ItompCIOTrajectory* group_trajectory = group_trajectory_;
  ItompCIOTrajectory* full_trajectory = full_trajectory_;
  planning_scene::PlanningScenePtr planning_scene = planning_scene_;
  robot_state::RobotStatePtr kinematic_state = kinematic_state_;

  // copy
  *this = data;

  // deep copy trajectories
  *group_trajectory = *data.group_trajectory_;
  *full_trajectory = *data.full_trajectory_;

  // copy pointers again
  group_trajectory_ = group_trajectory;
  full_trajectory_ = full_trajectory;

  fk_solver_.reset();

  // do not copy planning scene
  planning_scene_ = planning_scene;
  kinematic_state_ = kinematic_state;
}

}
