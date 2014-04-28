#include <itomp_cio_planner/model/itomp_robot_model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
//#include <planning_environment/models/model_utils.h>

using namespace std;

namespace itomp_cio_planner
{

ItompRobotModel::ItompRobotModel()
{
}

ItompRobotModel::~ItompRobotModel()
{
}

bool ItompRobotModel::init(robot_model::RobotModelPtr& robot_model, const std::string& robot_description)
{
  robot_model_ = robot_model;
  reference_frame_ = robot_model_->getModelFrame();

  // get the urdf as a string:
  string urdf_string;
  ros::NodeHandle node_handle("~");
  if (!node_handle.getParam(robot_description, urdf_string))
  {
    return false;
  }

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
  fk_solver_ = new KDL::TreeFkSolverJointPosAxis(kdl_tree_, robot_model_->getRootLinkName());

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
  std::map<std::string, std::vector<std::string> > groups;
  const std::vector<robot_model::JointModelGroup*>& jointModelGroups = robot_model_->getJointModelGroups();
  for (std::vector<robot_model::JointModelGroup*>::const_iterator it = jointModelGroups.begin();
      it != jointModelGroups.end(); ++it)
  {
    groups.insert(std::pair<std::string, std::vector<std::string> >((*it)->getName(), (*it)->getJointModelNames()));
  }

  // add unified_body group
  std::vector<std::string> merged_joint_names;
  for (std::map<std::string, std::vector<std::string> >::iterator it = groups.begin(); it != groups.end(); ++it)
  {
    for (unsigned int i = 0; i != it->second.size(); ++i)
    {
      if (find(merged_joint_names.begin(), merged_joint_names.end(), it->second[i]) == merged_joint_names.end())
      {
        merged_joint_names.push_back(it->second[i]);
      }
    }
  }
  groups["unified_body"] = merged_joint_names;

  for (std::map<std::string, std::vector<std::string> >::iterator it = groups.begin(); it != groups.end(); ++it)
  {
    ItompPlanningGroup group;
    group.name_ = it->first;
    ROS_INFO_STREAM("Planning group " << group.name_);
    int num_links = it->second.size();
    group.num_joints_ = 0;
    group.link_names_.resize(num_links);
    std::vector<bool> active_joints;
    active_joints.resize(num_kdl_joints_, false);
    for (int i = 0; i < num_links; i++)
    {
      std::string joint_name = it->second[i];
      map<string, string>::iterator link_name_it = joint_segment_mapping_.find(joint_name);
      if (link_name_it == joint_segment_mapping_.end())
      {
        ROS_ERROR("Joint name %s did not have containing KDL segment.", joint_name.c_str());
        return false;
      }
      std::string link_name = link_name_it->second;
      group.link_names_[i] = link_name;
      const KDL::Segment* segment = &(kdl_tree_.getSegment(link_name)->second.segment);
      KDL::Joint::JointType joint_type = segment->getJoint().getType();
      if (joint_type != KDL::Joint::None)
      {
        ItompRobotJoint joint;
        joint.group_joint_index_ = group.num_joints_;
        joint.kdl_joint_index_ = kdl_tree_.getSegment(link_name)->second.q_nr;
        joint.kdl_joint_ = &(segment->getJoint());
        joint.link_name_ = link_name;
        joint.joint_name_ = segment_joint_mapping_[link_name];

        const robot_model::JointModel * kin_model_joint = robot_model_->getJointModel(joint.joint_name_);
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

        group.num_joints_++;
        group.group_joints_.push_back(joint);
        active_joints[joint.kdl_joint_index_] = true;
      }

    }
    group.fk_solver_.reset(
        new KDL::TreeFkSolverJointPosAxisPartial(kdl_tree_, robot_model_->getRootLinkName(), active_joints));

    for (int i = 0; i < group.num_joints_; i++)
    {
      group.kdl_to_group_joint_[group.group_joints_[i].kdl_joint_index_] = i;
    }

    planning_groups_.insert(make_pair(it->first, group));
  }

  // TODO: add contact points to lower body
  planning_groups_["lower_body"].contactPoints_.push_back(ContactPoint("left_foot_endeffector_link", this));
  planning_groups_["lower_body"].contactPoints_.push_back(ContactPoint("right_foot_endeffector_link", this));

  planning_groups_["whole_body"].contactPoints_.push_back(ContactPoint("left_foot_endeffector_link", this));
  planning_groups_["whole_body"].contactPoints_.push_back(ContactPoint("right_foot_endeffector_link", this));
  planning_groups_["whole_body"].contactPoints_.push_back(ContactPoint("left_hand_endeffector_link", this));
  planning_groups_["whole_body"].contactPoints_.push_back(ContactPoint("right_hand_endeffector_link", this));

  ROS_INFO("Initialized ITOMP robot model in %s reference frame.", reference_frame_.c_str());

  return true;
}

}
