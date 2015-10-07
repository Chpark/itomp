#include <itomp_cio_planner/model/itomp_robot_model.h>
#include <itomp_cio_planner/model/itomp_robot_model_ik.h>
#include <itomp_cio_planner/util/planning_parameters.h>
#include <itomp_cio_planner/model/rbdl_urdf_reader.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

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
    ItompRobotModelIKHelper::getInstance()->initialize(robot_model);
	reference_frame_ = moveit_robot_model_->getModelFrame();

    const std::vector<const robot_model::JointModel*>& urdf_joints = robot_model->getJointModels();

    if (PlanningParameters::getInstance()->getPrintPlanningInfo())
    {
        ROS_INFO("Initialize ItompRobotModel");
        for (int i = 0; i < urdf_joints.size(); ++i)
            ROS_INFO("[%d] %s", urdf_joints[i]->getFirstVariableIndex(), urdf_joints[i]->getName().c_str());
    }

	// get the urdf as a string:
	string urdf_string;
	ros::NodeHandle node_handle("~");
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    if (!node_handle.getParam(robot_model_loader.getRobotDescription(), urdf_string))
	{
		return false;
	}

	// RBDL
    std::vector<std::vector<unsigned int> > rbdl_affected_body_ids_vector(urdf_joints.size() + 1);
	////////////////////////////////////////////////////////////////////////////
	{
        ReadURDFModel(urdf_string.c_str(), &rbdl_robot_model_);

		// rbdl_robot_model_.mJoints[0] is not used
		num_rbdl_joints_ = rbdl_robot_model_.mJoints.size() - 1;
		rbdl_number_to_joint_name_.resize(rbdl_robot_model_.mJoints.size());

		// compute rbdl_affected_body_ids for partial FK
		for (unsigned int i = 1; i < rbdl_robot_model_.mJoints.size(); ++i)
		{
			unsigned int current = i;
			do
			{
				rbdl_affected_body_ids_vector[current].push_back(i);
				current = rbdl_robot_model_.lambda[current];
			}
			while (current != 0);
		}

		// initialize the planning groups
		const std::vector<const robot_model::JointModelGroup*>& jointModelGroups =
			moveit_robot_model_->getJointModelGroups();
		for (std::vector<const robot_model::JointModelGroup*>::const_iterator it =
					jointModelGroups.begin(); it != jointModelGroups.end(); ++it)
		{
            ItompPlanningGroupPtr group = boost::make_shared<ItompPlanningGroup>();
			group->name_ = (*it)->getName();

            if (PlanningParameters::getInstance()->getPrintPlanningInfo())
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
                unsigned int body_id = rbdl_robot_model_.GetBodyId(link_name.c_str());

				// fixed joint
                if (joint_model == NULL || body_id >= rbdl_robot_model_.fixed_body_discriminator)
					continue;
				std::string joint_name = joint_model->getName();

                const RigidBodyDynamics::Joint& rbdl_joint = rbdl_robot_model_.mJoints[body_id];

				ItompRobotJoint joint;
				joint.group_joint_index_ = group->num_joints_;
				joint.rbdl_joint_index_ = rbdl_joint.q_index;
				joint.link_name_ = link_name;
				joint.joint_name_ = joint_name;
                joint.rbdl_affected_body_ids_ = rbdl_affected_body_ids_vector[body_id];

				switch (rbdl_joint.mJointType)
				{
				case RigidBodyDynamics::JointTypeRevolute:
					if (const robot_model::RevoluteJointModel* revolute_joint =
								dynamic_cast<const robot_model::RevoluteJointModel*>(joint_model))
					{
						joint.wrap_around_ = revolute_joint->isContinuous();
						joint.has_joint_limits_ = !(joint.wrap_around_);
						const robot_model::VariableBounds& bounds =
                            revolute_joint->getVariableBounds(revolute_joint->getName());
						joint.joint_limit_min_ = bounds.min_position_;
						joint.joint_limit_max_ = bounds.max_position_;

                        if (PlanningParameters::getInstance()->getPrintPlanningInfo())
                            ROS_INFO_STREAM("Setting bounds for joint[" << joint.group_joint_index_ << "][" << joint.rbdl_joint_index_ << "] " << revolute_joint->getName() << " to " << joint.joint_limit_min_ << " " << joint.joint_limit_max_);
					}
					break;
				case RigidBodyDynamics::JointTypePrismatic:
					if (const robot_model::PrismaticJointModel* prismatic_joint =
								dynamic_cast<const robot_model::PrismaticJointModel*>(joint_model))
					{
						joint.wrap_around_ = false;
						joint.has_joint_limits_ = true;
                        const robot_model::VariableBounds& bounds =prismatic_joint->getVariableBounds(prismatic_joint->getName());
						joint.joint_limit_min_ = bounds.min_position_;
						joint.joint_limit_max_ = bounds.max_position_;

                        if (PlanningParameters::getInstance()->getPrintPlanningInfo())
                            ROS_INFO_STREAM("Setting bounds for joint[" << joint.group_joint_index_ << "][" << joint.rbdl_joint_index_ << "] " << prismatic_joint->getName() << " to " << joint.joint_limit_min_ << " " << joint.joint_limit_max_);
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

        if (PlanningParameters::getInstance()->getPrintPlanningInfo())
        {
            ROS_INFO("RBDL Model Initialized");
            ROS_INFO("Joints");
            for (int i = 0; i < rbdl_number_to_joint_name_.size(); ++i)
                ROS_INFO("[%d] %s", i, rbdl_number_to_joint_name_[i].c_str());
            ROS_INFO("Links");
            for (int i = 0; i < rbdl_robot_model_.mBodies.size(); ++i)
                ROS_INFO("[%d] %s", i, rbdl_robot_model_.GetBodyName(i).c_str());
        }
	}

	// add rbdl partial fk ids
    const std::map<std::string, std::vector<std::string> >& contact_points = PlanningParameters::getInstance()->getContactPoints();
    const std::vector<const robot_model::JointModelGroup*>& jmg = moveit_robot_model_->getJointModelGroups();
    for (std::vector<const robot_model::JointModelGroup*>::const_iterator it = jmg.begin(); it != jmg.end(); ++it)
	{
		std::string group_name = (*it)->getName();
        ItompPlanningGroupPtr planning_group = boost::const_pointer_cast<ItompPlanningGroup>(planning_groups_[group_name]);

        const multimap<string, string>& group_endeffector_names = PlanningParameters::getInstance()->getGroupEndeffectorNames();

        std::pair<multimap<string, string>::const_iterator, multimap<string, string>::const_iterator> ret = group_endeffector_names.equal_range(group_name);

        for (multimap<string, string>::const_iterator it = ret.first; it != ret.second; ++it)
		{
			string endeffector_name = it->second;
            unsigned int endeffector_rbdl_id = rbdl_robot_model_.GetBodyId(endeffector_name.c_str());
			while (rbdl_robot_model_.IsFixedBodyId(endeffector_rbdl_id))
			{
                endeffector_rbdl_id = rbdl_robot_model_.GetParentBodyId(endeffector_rbdl_id);
			}

            std::map<std::string, std::vector<std::string> >::const_iterator it2 = contact_points.find(endeffector_name);
            if (it2 != contact_points.end())
			{
                const std::vector<string>& endeffector_contact_point_names = it2->second;

				std::vector<unsigned int> contact_point_rbdl_ids;
				for (int i = 0; i < endeffector_contact_point_names.size(); ++i)
				{
					const std::string& name = endeffector_contact_point_names[i];
                    unsigned int point_rbdl_id = rbdl_robot_model_.GetBodyId(name.c_str());
					contact_point_rbdl_ids.push_back(point_rbdl_id);

				}
                planning_group->contact_points_.push_back(ContactPoint(endeffector_name, endeffector_rbdl_id, contact_point_rbdl_ids));

                if (PlanningParameters::getInstance()->getPrintPlanningInfo())
                    ROS_INFO("Endeffector %s for group %s is added", endeffector_name.c_str(), group_name.c_str());
			}
		}
	}

    ROS_INFO("Initialized ITOMP robot model in %s reference frame.", reference_frame_.c_str());

	return true;
}


}
