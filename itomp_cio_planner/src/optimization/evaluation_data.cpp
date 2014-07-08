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
    int num_mass_segments, const moveit_msgs::Constraints& path_constraints)
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
  stateFTRCost_.resize(num_points);
  stateCartesianTrajectoryCost_.resize(num_points);

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
  contact_forces_.resize(num_points);
  for (int i = 0; i < num_points; ++i)
    contact_forces_[i].resize(num_contacts);

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
  //costAccumulator_.addCost(TrajectoryCost::CreateTrajectoryCost(TrajectoryCost::COST_GOAL_POSE));
  //costAccumulator_.addCost(TrajectoryCost::CreateTrajectoryCost(TrajectoryCost::COST_COM));
  //costAccumulator_.addCost(TrajectoryCost::CreateTrajectoryCost(TrajectoryCost::COST_FTR));
  costAccumulator_.addCost(TrajectoryCost::CreateTrajectoryCost(TrajectoryCost::COST_CARTESIAN_TRAJECTORY));
  costAccumulator_.init(this);

  fk_solver_ = *planning_group->fk_solver_.get();

  cartesian_waypoints_.resize(path_constraints.position_constraints.size());
  for (int i = 0; i < path_constraints.position_constraints.size(); ++i)
  {
    geometry_msgs::Vector3 position = path_constraints.position_constraints[i].target_point_offset;
    geometry_msgs::Quaternion orientation = path_constraints.orientation_constraints[i].orientation;
    cartesian_waypoints_[i].p = KDL::Vector(position.x, position.y, position.z);
    cartesian_waypoints_[i].M = KDL::Rotation::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w);
  }
}

void EvaluationData::initStaticEnvironment()
{
  // TODO:
  //return;

  string environment_file = PlanningParameters::getInstance()->getEnvironmentModel();
      //"package://kuka_description/cad/monitor_obs.dae";
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
    /*
     pose.orientation.x = sqrt(0.5);
     pose.orientation.y = 0.0;
     pose.orientation.z = 0.0;
     pose.orientation.w = sqrt(0.5);
     */
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;

    shapes::Mesh* shape = shapes::createMeshFromResource(environment_file);
    shapes::ShapeMsg mesh_msg;
    shapes::constructMsgFromShape(shape, mesh_msg);
    shape_msgs::Mesh mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

    collision_object.meshes.push_back(mesh);
    collision_object.mesh_poses.push_back(pose);
    /*
     // replace mesh with box
     shape_msgs::SolidPrimitive primitive;
     primitive.type = primitive.BOX;
     primitive.dimensions.resize(3);
     primitive.dimensions[0] = 2.0;
     primitive.dimensions[1] = 5.0;
     primitive.dimensions[2] = 5.0;
     collision_object.primitives.push_back(primitive);
     pose.position.x = 0.0;
     pose.position.y = 0.0;
     pose.position.z = -2.6;
     pose.orientation.x = 0.0;
     pose.orientation.y = 0.0;
     pose.orientation.z = 0.0;
     pose.orientation.w = 1.0;
     collision_object.primitive_poses.push_back(pose);
     */

    collision_object.operation = collision_object.ADD;
    moveit_msgs::PlanningScene planning_scene_msg;
    planning_scene_msg.world.collision_objects.push_back(collision_object);
    planning_scene_msg.is_diff = true;
    planning_scene_->setPlanningSceneDiffMsg(planning_scene_msg);

    const collision_detection::WorldPtr& world = planning_scene_->getWorldNonConst();
    std::vector<std::string> object_ids = world->getObjectIds();
    for (int i = 0; i < object_ids.size(); ++i)
    {
      collision_detection::World::ObjectPtr obj = world->getObject(object_ids[i]);
      for (int j = 0; j < obj->shapes_.size(); ++j)
      {
        shapes::Shape* shape = const_cast<shapes::Shape*>(obj->shapes_[j].get());
        shapes::Mesh* mesh = dynamic_cast<shapes::Mesh*>(shape);
        if (mesh == NULL)
          continue;
        mesh->computeTriangleNormals();
      }
    }
  }

  collision_detection::AllowedCollisionMatrix acm = planning_scene_->getAllowedCollisionMatrix();
  //acm.setEntry(true);
}

EvaluationData* EvaluationData::clone() const
{
  EvaluationData* new_data = new EvaluationData();

  *new_data = *this;

  new_data->group_trajectory_ = new ItompCIOTrajectory(*group_trajectory_);
  new_data->full_trajectory_ = new ItompCIOTrajectory(*full_trajectory_);

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

  // do not copy planning scene
  planning_scene_ = planning_scene;
  kinematic_state_ = kinematic_state;
}

void EvaluationData::compare(const EvaluationData& ref) const
{
  printf("Compare\n");
  // group trajectory
  {
    const MatrixXd& g = group_trajectory_->getTrajectory();
    const MatrixXd& rg = ref.group_trajectory_->getTrajectory();
    for (int r = 0; r < g.rows(); ++r)
    {
      for (int c = 0; c < g.cols(); ++c)
      {
        if (g(r, c) != rg(r, c))
        {
          double n1 = g(r, c);
          double n2 = rg(r, c);
          printf("GroupTraj(%d,%d): %.14f (%.14f - %.14f)\n", r, c, fabs(g(r, c) - rg(r, c)), g(r, c), rg(r, c));
        }
      }
    }
  }
  {
    const MatrixXd& g = group_trajectory_->getFreePoints();
    const MatrixXd& rg = ref.group_trajectory_->getFreePoints();
    for (int r = 0; r < g.rows(); ++r)
    {
      for (int c = 0; c < g.cols(); ++c)
      {
        if (g(r, c) != rg(r, c))
        {
          double n1 = g(r, c);
          double n2 = rg(r, c);
          printf("GroupFreeTraj(%d,%d): %.14f (%.14f - %.14f)\n", r, c, fabs(g(r, c) - rg(r, c)), g(r, c), rg(r, c));
        }
      }
    }
  }
  {
    const MatrixXd& g = group_trajectory_->getFreeVelPoints();
    const MatrixXd& rg = ref.group_trajectory_->getFreeVelPoints();
    for (int r = 0; r < g.rows(); ++r)
    {
      for (int c = 0; c < g.cols(); ++c)
      {
        if (g(r, c) != rg(r, c))
          printf("GroupFreeVelTraj(%d,%d): %.14f (%.14f - %.14f)\n", r, c, fabs(g(r, c) - rg(r, c)), g(r, c), rg(r, c));
      }
    }
  }
  // group trajectory
  {
    const MatrixXd& gc = group_trajectory_->getContactTrajectory();
    const MatrixXd& rgc = ref.group_trajectory_->getContactTrajectory();
    for (int r = 0; r < gc.rows(); ++r)
    {
      for (int c = 0; c < gc.cols(); ++c)
      {
        if (gc(r, c) != rgc(r, c))
          printf("ContactTraj(%d,%d): %.14f (%.14f - %.14f)\n", r, c, fabs(gc(r, c) - rgc(r, c)), gc(r, c), rgc(r, c));
      }
    }
  }
  // full trajectory
  {
    const MatrixXd& g = full_trajectory_->getTrajectory();
    const MatrixXd& rg = ref.full_trajectory_->getTrajectory();
    for (int r = 0; r < g.rows(); ++r)
    {
      for (int c = 0; c < g.cols(); ++c)
      {
        if (g(r, c) != rg(r, c))
          printf("FullTraj(%d,%d): %.14f (%.14f - %.14f)\n", r, c, fabs(g(r, c) - rg(r, c)), g(r, c), rg(r, c));
      }
    }
  }
  // segment frame
  {
    for (int i = 0; i < segment_frames_.size(); ++i)
    {
      for (int j = 0; j < segment_frames_[0].size(); ++j)
      {
        if (segment_frames_[i][j] != ref.segment_frames_[i][j])
        {
          printf("SegFrame [%d][%d] not equal\n", i, j);
        }
      }
    }
  }

  // wrench sum
  {
    for (int i = 0; i < wrenchSum_.size(); ++i)
    {
      if (wrenchSum_[i] != ref.wrenchSum_[i])
      {
        KDL::Wrench w1 = wrenchSum_[i];
        KDL::Wrench w2 = ref.wrenchSum_[i];
        printf("wrench sum %d not equal\n", i);
      }
    }
  }

  // com
  {
    for (int i = 0; i < CoMPositions_.size(); ++i)
    {
      if (CoMPositions_[i] != ref.CoMPositions_[i])
      {
        KDL::Vector w1 = CoMPositions_[i];
        KDL::Vector w2 = ref.CoMPositions_[i];
        printf("com pos %d not equal\n", i);
      }
    }
  }

  // contact violation vec
  {
    for (int i = 0; i < contactViolationVector_.size(); ++i)
    {
      for (int j = 0; j < contactViolationVector_[0].size(); ++j)
      {
        for (int k = 0; k < 4; ++k)
        {
          if (contactViolationVector_[i][j].data_[k] != ref.contactViolationVector_[i][j].data_[k])
          {
            Vector4d v1 = contactViolationVector_[i][j];
            Vector4d v2 = ref.contactViolationVector_[i][j];
            printf("contactViolationVector_ [%d][%d] not equal\n", i, j);
            printf("%.14f %.14f %.14f %.14f - %.14f %.14f %.14f %.14f\n", v1.data_[0], v1.data_[1], v1.data_[2],
                v1.data_[3], v2.data_[0], v2.data_[1], v2.data_[2], v2.data_[3]);
          }
        }
      }
    }
  }

  // contact violation vec
  {
    for (int i = 0; i < contactPointVelVector_.size(); ++i)
    {
      for (int j = 0; j < contactPointVelVector_[0].size(); ++j)
      {
        if (contactPointVelVector_[i][j] != ref.contactPointVelVector_[i][j])
        {
          KDL::Vector v1 = contactPointVelVector_[i][j];
          KDL::Vector v2 = ref.contactPointVelVector_[i][j];
          printf("contactPointVelVector_ [%d][%d] not equal\n", i, j);
          printf("%.14f %.14f %.14f  -  %.14f %.14f %.14f\n", v1.x(), v1.y(), v1.z(), v2.x(), v2.y(), v2.z());
        }

      }
    }
  }

  // contact invariant cost
  {
    for (int point = 0; point < stateContactInvariantCost_.size(); ++point)
    {
      printf("CICost %d : %.14f\n", point, stateContactInvariantCost_[point]);
      if (stateContactInvariantCost_[point] != ref.stateContactInvariantCost_[point])
      {
        double w1 = stateContactInvariantCost_[point];
        double w2 = ref.stateContactInvariantCost_[point];

        double calc = 0, calc2 = 0;

        {
          std::vector<double> contact_values(4);
          int phase = getGroupTrajectory()->getContactPhase(point);
          for (int i = 0; i < 4; ++i)
            contact_values[i] = getGroupTrajectory()->getContactValue(phase, i);

          for (int i = 0; i < 4; ++i)
          {
            double cost = 0.0;
            for (int j = 0; j < 4; ++j)
              cost += contactViolationVector_[i][point].data_[j] * contactViolationVector_[i][point].data_[j];
            cost += 16.0 * KDL::dot(contactPointVelVector_[i][point], contactPointVelVector_[i][point]);
            calc += contact_values[i] * cost;
            printf("CV %d : %.14f\n", i, contact_values[i]);
            printf("CVV %d : %.14f %.14f %.14f %.14f\n", i, contactViolationVector_[i][point].data_[0],
                contactViolationVector_[i][point].data_[1], contactViolationVector_[i][point].data_[2],
                contactViolationVector_[i][point].data_[3]);
            printf("CPV %d : %.14f : %.14f %.14f %.14f\n", i,
                16.0 * KDL::dot(contactPointVelVector_[i][point], contactPointVelVector_[i][point]),
                contactPointVelVector_[i][point].x(), contactPointVelVector_[i][point].y(),
                contactPointVelVector_[i][point].z());
          }
        }
        {
          std::vector<double> contact_values(4);
          int phase = ref.getGroupTrajectory()->getContactPhase(point);
          for (int i = 0; i < 4; ++i)
            contact_values[i] = ref.getGroupTrajectory()->getContactValue(phase, i);

          for (int i = 0; i < 4; ++i)
          {
            double cost = 0.0;
            for (int j = 0; j < 4; ++j)
              cost += ref.contactViolationVector_[i][point].data_[j] * ref.contactViolationVector_[i][point].data_[j];
            cost += 16.0 * KDL::dot(ref.contactPointVelVector_[i][point], ref.contactPointVelVector_[i][point]);
            calc2 += contact_values[i] * cost;
            printf("CV %d : %.14f\n", i, contact_values[i]);
            printf("CVV %d : %.14f %.14f %.14f %.14f\n", i, ref.contactViolationVector_[i][point].data_[0],
                ref.contactViolationVector_[i][point].data_[1], ref.contactViolationVector_[i][point].data_[2],
                ref.contactViolationVector_[i][point].data_[3]);
            printf("CPV %d : %.14f : %.14f %.14f %.14f\n", i,
                16.0 * KDL::dot(ref.contactPointVelVector_[i][point], ref.contactPointVelVector_[i][point]),
                ref.contactPointVelVector_[i][point].x(), ref.contactPointVelVector_[i][point].y(),
                ref.contactPointVelVector_[i][point].z());
          }
        }
        printf("ci cost %d not equal %.14f : %.14f\n", point, w1, w2);
      }
    }
  }

}

}
