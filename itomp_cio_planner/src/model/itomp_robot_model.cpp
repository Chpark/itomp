#include <itomp_cio_planner/model/itomp_robot_model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

using namespace std;

namespace itomp_cio_planner
{

ItompRobotModel::ItompRobotModel()
{
}

ItompRobotModel::~ItompRobotModel()
{
}

bool ItompRobotModel::init(const robot_model::RobotModelConstPtr& robot_model)
{
  moveit_robot_model_ = robot_model;
  reference_frame_ = moveit_robot_model_->getModelFrame();

  // get the urdf as a string:
  string urdf_string;
  ros::NodeHandle node_handle("~");
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  if (!node_handle.getParam(robot_model_loader.getRobotDescription(), urdf_string))
  {
    return false;
  }

  // TODO : Remove KDL-base code
  RigidBodyDynamics::Addons::read_urdf_model(
      "/home/chonhyon/hydro_workspace/itomp/human_description/robots/human_cio_rbdl.urdf", &rbdl_robot_model_);
  rbdl_robot_model_.gravity = Eigen::Vector3d(0, 0, -9.8);

  // rbdl_robot_model_.mJoints[0] is not used
  num_rbdl_joints_ = rbdl_robot_model_.mJoints.size() - 1;
  rbdl_number_to_joint_name_.resize(rbdl_robot_model_.mJoints.size());

  // TODO: handle root transform
  const string ROOT_TRANSFORM_LINK_NAMES[] =
  { "base_prismatic_dummy1", "base_prismatic_dummy2", "base_prismatic_dummy3", "base_revolute_dummy1",
      "base_revolute_dummy2", "pelvis_link" };
  const string ROOT_TRANSFORM_JOINT_NAMES[] =
  { "base_prismatic_joint_x", "base_prismatic_joint_x", "base_prismatic_joint_x", "base_revolute_joint_z",
      "base_revolute_joint_y", "base_revolute_joint_x" };

  // initialize the planning groups
  const std::vector<const robot_model::JointModelGroup*>& jointModelGroups = moveit_robot_model_->getJointModelGroups();
  for (std::vector<const robot_model::JointModelGroup*>::const_iterator it = jointModelGroups.begin();
      it != jointModelGroups.end(); ++it)
  {
    ItompPlanningGroupPtr group = boost::make_shared<ItompPlanningGroup>();
    group->name_ = (*it)->getName();
    ROS_INFO_STREAM("Planning group " << group->name_);

    const std::vector<std::string>& joint_model_names = (*it)->getJointModelNames();
    const std::vector<std::string>& link_model_names = (*it)->getLinkModelNames();

    group->num_joints_ = 0;
    std::vector<bool> active_joints;
    active_joints.resize(num_rbdl_joints_, false);
    for (int i = 0; i < link_model_names.size(); i++)
    {
      std::string link_name = link_model_names[i];
      const moveit::core::LinkModel* link_model = moveit_robot_model_->getLinkModel(link_name.c_str());
      if (link_model == NULL)
      {
        ROS_ERROR("Link name %s not exist.", link_name.c_str());
        return false;
      }
      const moveit::core::JointModel* joint_model = link_model->getParentJointModel();
      unsigned int joint_index = rbdl_robot_model_.GetBodyId(link_name.c_str());

      // TODO: handle root transform
      for (int j = 0; j < 6; ++j)
      {
        if (link_name == ROOT_TRANSFORM_LINK_NAMES[j])
        {
          joint_index = j + 1;
          break;
        }
      }

      // fixed joint
      if (joint_model == NULL || joint_index >= rbdl_robot_model_.fixed_body_discriminator)
        continue;
      std::string joint_name = joint_model->getName();

      const RigidBodyDynamics::Joint& rbdl_joint = rbdl_robot_model_.mJoints[joint_index];

      ItompRobotJoint joint;
      joint.group_joint_index_ = group->num_joints_;
      joint.rbdl_joint_index_ = rbdl_joint.q_index;
      joint.link_name_ = link_name;
      joint.joint_name_ = joint_name;

      switch (rbdl_joint.mJointType)
      {
      case RigidBodyDynamics::JointTypeRevolute:
        if (const robot_model::RevoluteJointModel* revolute_joint =
            dynamic_cast<const robot_model::RevoluteJointModel*>(joint_model))
        {
          joint.wrap_around_ = revolute_joint->isContinuous();
          joint.has_joint_limits_ = !(joint.wrap_around_);
          const robot_model::VariableBounds& bounds = revolute_joint->getVariableBounds(revolute_joint->getName());
          joint.joint_limit_min_ = bounds.min_position_;
          joint.joint_limit_max_ = bounds.max_position_;

          ROS_INFO_STREAM(
              "Setting bounds for joint[" << joint.group_joint_index_ << "][" << joint.rbdl_joint_index_ << "] " << revolute_joint->getName() << " to " << joint.joint_limit_min_ << " " << joint.joint_limit_max_);
        }
        break;
      case RigidBodyDynamics::JointTypePrismatic:
        if (const robot_model::PrismaticJointModel* prismatic_joint =
            dynamic_cast<const robot_model::PrismaticJointModel*>(joint_model))
        {
          joint.wrap_around_ = false;
          joint.has_joint_limits_ = true;
          const robot_model::VariableBounds& bounds = prismatic_joint->getVariableBounds(prismatic_joint->getName());
          joint.joint_limit_min_ = bounds.min_position_;
          joint.joint_limit_max_ = bounds.max_position_;
          ROS_INFO_STREAM(
              "Setting bounds for joint[" << joint.group_joint_index_ << "][" << joint.rbdl_joint_index_ << "] " << prismatic_joint->getName() << " to " << joint.joint_limit_min_ << " " << joint.joint_limit_max_);
        }
        break;
      default:
        ROS_ERROR("Unsupported joint type for joint %s", joint_name.c_str());
        return false;
      }

      rbdl_number_to_joint_name_[joint.rbdl_joint_index_] = joint_name;
      joint_name_to_rbdl_number_.insert(make_pair(joint_name, joint.rbdl_joint_index_));

      group->num_joints_++;
      group->group_joints_.push_back(joint);
      active_joints[joint.rbdl_joint_index_] = true;
    }

    for (int i = 0; i < group->num_joints_; i++)
    {
      group->rbdl_to_group_joint_[group->group_joints_[i].rbdl_joint_index_] = i;
    }

    planning_groups_.insert(make_pair(group->name_, group));
  }
  // TODO:
  planning_groups_.clear();

  // Construct the KDL tree
  if (!kdl_parser::treeFromString(urdf_string, kdl_tree_))
  {
    ROS_ERROR("Failed to construct KDL tree from URDF.");
    return false;
  }
  num_kdl_joints_ = kdl_tree_.getNrOfJoints();

  // create the joint_segment_mapping, which used to be created by the URDF -> KDL parser
  // but not any more, but the rest of the code depends on it, so we simply generate the mapping here:
  KDL::SegmentMap segment_map = kdl_tree_.getSegments();

  for (KDL::SegmentMap::const_iterator it = segment_map.begin(); it != segment_map.end(); ++it)
  {
    std::string joint_name = it->second.segment.getJoint().getName();
    std::string segment_name = it->first;
    joint_segment_mapping_.insert(make_pair(joint_name, segment_name));
  }

  // create the fk solver:
  fk_solver_ = new KDL::TreeFkSolverJointPosAxis(kdl_tree_, moveit_robot_model_->getRootLinkName());

  kdl_number_to_urdf_name_.resize(num_kdl_joints_);
  // Create the inverse mapping - KDL segment to joint name
  // (at the same time) Create a mapping from KDL numbers to URDF joint names and vice versa
  for (map<string, string>::iterator it = joint_segment_mapping_.begin(); it != joint_segment_mapping_.end(); ++it)
  {
    std::string joint_name = it->first;
    std::string segment_name = it->second;
    //  std::cout << joint_name << " -> " << segment_name << std::endl;
    segment_joint_mapping_.insert(make_pair(segment_name, joint_name));
    int kdl_number = kdl_tree_.getSegment(segment_name)->second.q_nr;
    if (kdl_tree_.getSegment(segment_name)->second.segment.getJoint().getType() != KDL::Joint::None)
    {
      //    std::cout << "Kdl number is " << kdl_number << std::endl;
      kdl_number_to_urdf_name_[kdl_number] = joint_name;
      urdf_name_to_kdl_number_.insert(make_pair(joint_name, kdl_number));
    }
  }

  // initialize the planning groups
  //const std::vector<robot_model::JointModelGroup*>& jointModelGroups = moveit_robot_model_->getJointModelGroups();
  for (std::vector<const robot_model::JointModelGroup*>::const_iterator it = jointModelGroups.begin();
      it != jointModelGroups.end(); ++it)
  {
    ItompPlanningGroupPtr group = boost::make_shared<ItompPlanningGroup>();
    group->name_ = (*it)->getName();
    ROS_INFO_STREAM("Planning group " << group->name_);

    const std::vector<std::string> joint_model_names = (*it)->getJointModelNames();

    group->num_joints_ = 0;
    std::vector<bool> active_joints;
    active_joints.resize(num_kdl_joints_, false);
    for (int i = 0; i < joint_model_names.size(); i++)
    {
      std::string joint_name = joint_model_names[i];
      map<string, string>::iterator link_name_it = joint_segment_mapping_.find(joint_name);
      if (link_name_it == joint_segment_mapping_.end())
      {
        ROS_ERROR("Joint name %s did not have containing KDL segment.", joint_name.c_str());
        return false;
      }
      std::string link_name = link_name_it->second;
      const KDL::Segment* segment = &(kdl_tree_.getSegment(link_name)->second.segment);
      KDL::Joint::JointType joint_type = segment->getJoint().getType();
      if (joint_type != KDL::Joint::None)
      {
        ItompRobotJoint joint;
        joint.group_joint_index_ = group->num_joints_;
        joint.kdl_joint_index_ = kdl_tree_.getSegment(link_name)->second.q_nr;
        joint.link_name_ = link_name;
        joint.joint_name_ = segment_joint_mapping_[link_name];

        const robot_model::JointModel * kin_model_joint = moveit_robot_model_->getJointModel(joint.joint_name_);
        if (const robot_model::RevoluteJointModel* revolute_joint =
            dynamic_cast<const robot_model::RevoluteJointModel*>(kin_model_joint))
        {
          joint.wrap_around_ = revolute_joint->isContinuous();
          joint.has_joint_limits_ = !(joint.wrap_around_);
          const robot_model::VariableBounds& bounds = revolute_joint->getVariableBounds(revolute_joint->getName());
          joint.joint_limit_min_ = bounds.min_position_;
          joint.joint_limit_max_ = bounds.max_position_;

          ROS_INFO_STREAM(
              "Setting bounds for joint[" << joint.group_joint_index_ << "][" << joint.kdl_joint_index_ << "] " << revolute_joint->getName() << " to " << joint.joint_limit_min_ << " " << joint.joint_limit_max_);
        }
        else if (const robot_model::PrismaticJointModel* prismatic_joint =
            dynamic_cast<const robot_model::PrismaticJointModel*>(kin_model_joint))
        {
          joint.wrap_around_ = false;
          joint.has_joint_limits_ = true;
          const robot_model::VariableBounds& bounds = prismatic_joint->getVariableBounds(prismatic_joint->getName());
          joint.joint_limit_min_ = bounds.min_position_;
          joint.joint_limit_max_ = bounds.max_position_;
          ROS_INFO_STREAM(
              "Setting bounds for joint[" << joint.group_joint_index_ << "][" << joint.kdl_joint_index_ << "] " << prismatic_joint->getName() << " to " << joint.joint_limit_min_ << " " << joint.joint_limit_max_);
        }
        else
        {
          ROS_WARN("Cannot handle floating or planar joints yet.");
        }

        group->num_joints_++;
        group->group_joints_.push_back(joint);
        active_joints[joint.kdl_joint_index_] = true;
      }

    }
    group->fk_solver_.reset(
        new KDL::TreeFkSolverJointPosAxisPartial(kdl_tree_, moveit_robot_model_->getRootLinkName(), active_joints));

    for (int i = 0; i < group->num_joints_; i++)
    {
      group->kdl_to_group_joint_[group->group_joints_[i].kdl_joint_index_] = i;
    }

    planning_groups_.insert(make_pair(group->name_, group));
  }

  // TODO: add contact points to lower body
  if (robot_model->hasLinkModel("left_foot_endeffector_link"))
  {
    boost::const_pointer_cast<ItompPlanningGroup>(planning_groups_["lower_body"])->contactPoints_.push_back(
        ContactPoint("left_foot_endeffector_link", this));
    boost::const_pointer_cast<ItompPlanningGroup>(planning_groups_["lower_body"])->contactPoints_.push_back(
        ContactPoint("right_foot_endeffector_link", this));

    boost::const_pointer_cast<ItompPlanningGroup>(planning_groups_["whole_body"])->contactPoints_.push_back(
        ContactPoint("left_foot_endeffector_link", this));
    boost::const_pointer_cast<ItompPlanningGroup>(planning_groups_["whole_body"])->contactPoints_.push_back(
        ContactPoint("right_foot_endeffector_link", this));
    boost::const_pointer_cast<ItompPlanningGroup>(planning_groups_["whole_body"])->contactPoints_.push_back(
        ContactPoint("left_hand_endeffector_link", this));
    boost::const_pointer_cast<ItompPlanningGroup>(planning_groups_["whole_body"])->contactPoints_.push_back(
        ContactPoint("right_hand_endeffector_link", this));
  }

  ROS_INFO("Initialized ITOMP robot model in %s reference frame.", reference_frame_.c_str());

  return true;
}

}
