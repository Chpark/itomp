#include <itomp_cio_planner/planner/itomp_planner_node.h>
#include <itomp_cio_planner/model/itomp_planning_group.h>
#include <itomp_cio_planner/util/planning_parameters.h>
#include <itomp_cio_planner/util/joint_state_util.h>
#include <itomp_cio_planner/visualization/visualization_manager.h>
#include <kdl/jntarray.hpp>
#include <angles/angles.h>
#include <visualization_msgs/MarkerArray.h>
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

using namespace std;

namespace itomp_cio_planner
{

ItompPlannerNode::ItompPlannerNode(const robot_model::RobotModelConstPtr& model) :
    robot_model_(model)
{

}

bool ItompPlannerNode::init()
{
  // load parameters
  PlanningParameters::getInstance()->initFromNodeHandle();

  // build itomp robot model
  itomp_robot_model_ = boost::make_shared<ItompRobotModel>();
  if (!itomp_robot_model_->init(robot_model_))
    return false;

  VisualizationManager::getInstance()->initialize(itomp_robot_model_);

  trajectory_ = boost::make_shared<ItompCIOTrajectory>(itomp_robot_model_,
      PlanningParameters::getInstance()->getTrajectoryDuration(),
      PlanningParameters::getInstance()->getTrajectoryDiscretization());

  ROS_INFO("Initialized ITOMP planning service...");

  return true;
}

bool ItompPlannerNode::planTrajectory(const planning_scene::PlanningSceneConstPtr& planning_scene,
    const planning_interface::MotionPlanRequest &req, planning_interface::MotionPlanResponse &res)
{
  // reload parameters
  PlanningParameters::getInstance()->initFromNodeHandle();

  if (!validateRequest(req))
    return false;

  double trajectory_start_time = req.start_state.joint_state.header.stamp.toSec();
  robot_state::RobotStatePtr initial_robot_state = planning_scene->getCurrentStateUpdated(req.start_state);

  // generate planning group list
  vector<string> planning_groups = getPlanningGroups(req.group_name);
  planning_info_manager_.reset(PlanningParameters::getInstance()->getNumTrials(), planning_groups.size());

  for (int c = 0; c < PlanningParameters::getInstance()->getNumTrials(); ++c)
  {
    double planning_start_time = ros::Time::now().toSec();

    ROS_INFO("Trial [%d]", c);

    // initialize trajectory with start state
    initTrajectory(req.start_state.joint_state);

    // for each planning group
    for (unsigned int i = 0; i != planning_groups.size(); ++i)
    {
      ros::WallTime create_time = ros::WallTime::now();

      /// optimize
      fillGroupJointTrajectory(planning_groups[i], getGoalStateFromGoalConstraints(itomp_robot_model_, req),
          req.path_constraints);
      optimizer_ = boost::make_shared<ItompOptimizer>(0, trajectory_, itomp_robot_model_,
          itomp_robot_model_->getPlanningGroup(planning_groups[i]), planning_start_time, trajectory_start_time,
          req.path_constraints);
      optimizer_->optimize();
      planning_info_manager_.write(c, i, optimizer_->getPlanningInfo());

      ROS_INFO(
          "Optimization of group %s took %f sec", planning_groups[i].c_str(), (ros::WallTime::now() - create_time).toSec());
    }
  }
  planning_info_manager_.printSummary();

  // return trajectory
  fillInResult(initial_robot_state, planning_groups, res);

  return true;
}

bool ItompPlannerNode::validateRequest(const planning_interface::MotionPlanRequest &req)
{
  ROS_INFO("Received planning request...");
  ROS_INFO("Trajectory Duration : %f", PlanningParameters::getInstance()->getTrajectoryDuration());

  // check goal constraint
  ROS_INFO("goal");
  sensor_msgs::JointState goal_joint_state = jointConstraintsToJointState(req.goal_constraints);
  if (goal_joint_state.name.size() != goal_joint_state.position.size())
  {
    ROS_ERROR("Invalid goal");
    return false;
  }
  for (unsigned int i = 0; i < goal_joint_state.name.size(); i++)
  {
    ROS_INFO("%s %f", goal_joint_state.name[i].c_str(), goal_joint_state.position[i]);
  }

  ROS_INFO_STREAM("Joint state has " << req.start_state.joint_state.name.size() << " joints");

  return true;
}

void ItompPlannerNode::initTrajectory(const sensor_msgs::JointState &joint_state)
{
  if (trajectory_->getDuration() != PlanningParameters::getInstance()->getTrajectoryDuration())
  {
    trajectory_ = boost::make_shared<ItompCIOTrajectory>(itomp_robot_model_, PlanningParameters::getInstance()->getTrajectoryDuration(),
        PlanningParameters::getInstance()->getTrajectoryDiscretization());
  }

  jointStateToArray(itomp_robot_model_, joint_state, trajectory_->getTrajectoryPoint(0),
      trajectory_->getTrajectoryPoint(0, ItompCIOTrajectory::TRAJECTORY_VELOCITY),
      trajectory_->getTrajectoryPoint(0, ItompCIOTrajectory::TRAJECTORY_ACCELERATION));

  for (int i = 1; i < trajectory_->getNumPoints(); ++i)
  {
    trajectory_->getTrajectoryPoint(i) = trajectory_->getTrajectoryPoint(0);
  }
}

std::vector<std::string> ItompPlannerNode::getPlanningGroups(const string& group_name) const
{
  std::vector<std::string> plannning_groups;

  if (group_name == "decomposed_body")
  {
    plannning_groups.push_back("lower_body");
    plannning_groups.push_back("torso");
    plannning_groups.push_back("head");
    plannning_groups.push_back("left_arm");
    plannning_groups.push_back("right_arm");
  }
  else
  {
    plannning_groups.push_back(group_name);
  }
  return plannning_groups;
}

void ItompPlannerNode::fillInResult(const robot_state::RobotStatePtr& robot_state,
    const std::vector<std::string>& planning_groups, planning_interface::MotionPlanResponse &res)
{
  int num_all_joints = robot_state->getVariableCount();

  res.trajectory_ = boost::make_shared<robot_trajectory::RobotTrajectory>(itomp_robot_model_->getMoveitRobotModel(), "");

  std::vector<double> velocity_limits(num_all_joints, std::numeric_limits<double>::max());

  robot_state::RobotState ks = *robot_state;
  std::vector<double> positions(num_all_joints);
  double duration = trajectory_->getDiscretization();
  for (std::size_t i = 0; i < trajectory_->getNumPoints(); ++i)
  {
    for (std::size_t j = 0; j < num_all_joints; j++)
    {
      positions[j] = (*trajectory_)(i, j);
    }

    ks.setVariablePositions(&positions[0]);
    // TODO: copy vel/acc
    ks.update();

    res.trajectory_->addSuffixWayPoint(ks, duration);
  }
  res.error_code_.val = moveit_msgs::MoveItErrorCodes::SUCCESS;

  // print results
  if (PlanningParameters::getInstance()->getPrintPlanningInfo())
  {
    const std::vector<std::string>& joint_names = res.trajectory_->getFirstWayPoint().getVariableNames();
    for (int j = 0; j < num_all_joints; j++)
      printf("%s ", joint_names[j].c_str());
    printf("\n");
    for (int i = 0; i < trajectory_->getNumPoints(); ++i)
    {
      for (int j = 0; j < num_all_joints; j++)
      {
        printf("%f ", res.trajectory_->getWayPoint(i).getVariablePosition(j));
      }
      printf("\n");
    }
  }
}

void ItompPlannerNode::fillGroupJointTrajectory(const string& group_name,
    const sensor_msgs::JointState& joint_goal_state, const moveit_msgs::Constraints& path_constraints)
{
  const ItompPlanningGroupConstPtr group = itomp_robot_model_->getPlanningGroup(group_name);

  // set trajectory goal point
  int goal_index = trajectory_->getNumPoints() - 1;
  Eigen::MatrixXd::RowXpr goalPoint = trajectory_->getTrajectoryPoint(goal_index);
  for (int i = 0; i < group->num_joints_; ++i)
  {
    string name = group->group_joints_[i].joint_name_;
    int kdl_number = itomp_robot_model_->urdfNameToKdlNumber(name);
    if (kdl_number >= 0)
    {
      goalPoint(kdl_number) = joint_goal_state.position[kdl_number];
    }
  }

  // interpolate trajectory
  std::set<int> group_joints_RBDL_joint_indices;
  for (int i = 0; i < group->num_joints_; ++i)
  {
    group_joints_RBDL_joint_indices.insert(group->group_joints_[i].rbdl_joint_index_);
  }
  if (path_constraints.position_constraints.size() == 0)
  {
    trajectory_->fillInMinJerk(group_joints_RBDL_joint_indices);
  }
  else
  {
    trajectory_->fillInMinJerkCartesianTrajectory(group_joints_RBDL_joint_indices, path_constraints, group_name);
  }
}

} // namespace
