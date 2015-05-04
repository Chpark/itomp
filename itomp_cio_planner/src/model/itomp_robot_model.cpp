#include <itomp_cio_planner/model/itomp_robot_model.h>
#include <itomp_cio_planner/util/planning_parameters.h>
#include <itomp_cio_planner/model/rbdl_urdf_reader.h>
#include <kdl_parser/kdl_parser.hpp>
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
	reference_frame_ = moveit_robot_model_->getModelFrame();

	// print robot_model joints
	ROS_INFO("Initialize ItompRobotModel");
	const std::vector<const robot_model::JointModel*>& urdf_joints =
		robot_model->getJointModels();
	for (int i = 0; i < urdf_joints.size(); ++i)
		ROS_INFO(
			"[%d] %s", urdf_joints[i]->getFirstVariableIndex(), urdf_joints[i]->getName().c_str());

	// get the urdf as a string:
	string urdf_string;
	ros::NodeHandle node_handle("~");
	robot_model_loader::RobotModelLoader robot_model_loader(
		"robot_description");
	if (!node_handle.getParam(robot_model_loader.getRobotDescription(),
							  urdf_string))
	{
		return false;
	}

	// RBDL
	std::vector<std::vector<unsigned int> > rbdl_affected_body_ids_vector(
		urdf_joints.size() + 1);
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
			ItompPlanningGroupPtr group =
				boost::make_shared<ItompPlanningGroup>();
			group->name_ = (*it)->getName();
			ROS_INFO_STREAM("Planning group " << group->name_);

			const std::vector<std::string>& joint_model_names =
				(*it)->getJointModelNames();
			const std::vector<std::string>& link_model_names =
				(*it)->getLinkModelNames();

			group->num_joints_ = 0;
			std::vector<bool> active_joints;
			active_joints.resize(num_rbdl_joints_, false);
			for (int i = 0; i < link_model_names.size(); i++)
			{
				std::string link_name = link_model_names[i];
				const moveit::core::LinkModel* link_model =
					moveit_robot_model_->getLinkModel(link_name.c_str());
				if (link_model == NULL)
				{
					ROS_ERROR("Link name %s not exist.", link_name.c_str());
					return false;
				}
				const moveit::core::JointModel* joint_model =
					link_model->getParentJointModel();
				unsigned int body_id = rbdl_robot_model_.GetBodyId(
										   link_name.c_str());

				// fixed joint
				if (joint_model == NULL
						|| body_id
						>= rbdl_robot_model_.fixed_body_discriminator)
					continue;
				std::string joint_name = joint_model->getName();

				const RigidBodyDynamics::Joint& rbdl_joint =
					rbdl_robot_model_.mJoints[body_id];

				ItompRobotJoint joint;
				joint.group_joint_index_ = group->num_joints_;
				joint.rbdl_joint_index_ = rbdl_joint.q_index;
				joint.link_name_ = link_name;
				joint.joint_name_ = joint_name;
				joint.rbdl_affected_body_ids_ =
					rbdl_affected_body_ids_vector[body_id];

				switch (rbdl_joint.mJointType)
				{
				case RigidBodyDynamics::JointTypeRevolute:
					if (const robot_model::RevoluteJointModel* revolute_joint =
								dynamic_cast<const robot_model::RevoluteJointModel*>(joint_model))
					{
						joint.wrap_around_ = revolute_joint->isContinuous();
						joint.has_joint_limits_ = !(joint.wrap_around_);
						const robot_model::VariableBounds& bounds =
							revolute_joint->getVariableBounds(
								revolute_joint->getName());
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
						const robot_model::VariableBounds& bounds =
							prismatic_joint->getVariableBounds(
								prismatic_joint->getName());
						joint.joint_limit_min_ = bounds.min_position_;
						joint.joint_limit_max_ = bounds.max_position_;
						ROS_INFO_STREAM(
							"Setting bounds for joint[" << joint.group_joint_index_ << "][" << joint.rbdl_joint_index_ << "] " << prismatic_joint->getName() << " to " << joint.joint_limit_min_ << " " << joint.joint_limit_max_);
					}
					break;
				default:
					ROS_ERROR(
						"Unsupported joint type for joint %s", joint_name.c_str());
					return false;
				}

				rbdl_number_to_joint_name_[joint.rbdl_joint_index_] =
					joint_name;
				joint_name_to_rbdl_number_.insert(
					make_pair(joint_name, joint.rbdl_joint_index_));

				group->num_joints_++;
				group->group_joints_.push_back(joint);
				active_joints[joint.rbdl_joint_index_] = true;
			}

			for (int i = 0; i < group->num_joints_; i++)
			{
				group->rbdl_to_group_joint_[group->group_joints_[i].rbdl_joint_index_] =
					i;
			}

			planning_groups_.insert(make_pair(group->name_, group));
		}

		ROS_INFO("RBDL Model Initialized");
        ROS_INFO("Joints");
		for (int i = 0; i < rbdl_number_to_joint_name_.size(); ++i)
			ROS_INFO("[%d] %s", i, rbdl_number_to_joint_name_[i].c_str());
        ROS_INFO("Links");
        for (int i = 0; i < rbdl_robot_model_.mBodies.size(); ++i)
            ROS_INFO("[%d] %s", i, rbdl_robot_model_.GetBodyName(i).c_str());
	}
	////////////////////////////////////////////////////////////////////////////

	// TODO:
	planning_groups_.clear();
	////////////////////////////////////////////////////////////////////////////

	// TODO : Remove KDL-base code
	////////////////////////////////////////////////////////////////////////////
	{
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

		for (KDL::SegmentMap::const_iterator it = segment_map.begin();
				it != segment_map.end(); ++it)
		{
			std::string joint_name = it->second.segment.getJoint().getName();
			std::string segment_name = it->first;
			joint_segment_mapping_.insert(make_pair(joint_name, segment_name));
		}

		// create the fk solver:
		fk_solver_ = new KDL::TreeFkSolverJointPosAxis(kdl_tree_,
				moveit_robot_model_->getRootLinkName());

		kdl_number_to_urdf_name_.resize(num_kdl_joints_);
		// Create the inverse mapping - KDL segment to joint name
		// (at the same time) Create a mapping from KDL numbers to URDF joint names and vice versa
		for (map<string, string>::iterator it = joint_segment_mapping_.begin();
				it != joint_segment_mapping_.end(); ++it)
		{
			std::string joint_name = it->first;
			std::string segment_name = it->second;
			//  std::cout << joint_name << " -> " << segment_name << std::endl;
			segment_joint_mapping_.insert(make_pair(segment_name, joint_name));
			int kdl_number = kdl_tree_.getSegment(segment_name)->second.q_nr;
			if (kdl_tree_.getSegment(segment_name)->second.segment.getJoint().getType()
					!= KDL::Joint::None)
			{
				//    std::cout << "Kdl number is " << kdl_number << std::endl;
				kdl_number_to_urdf_name_[kdl_number] = joint_name;
				urdf_name_to_kdl_number_.insert(
					make_pair(joint_name, kdl_number));
			}
		}

		// initialize the planning groups
		const std::vector<const robot_model::JointModelGroup*>& jointModelGroups =
			moveit_robot_model_->getJointModelGroups();
		for (std::vector<const robot_model::JointModelGroup*>::const_iterator it =
					jointModelGroups.begin(); it != jointModelGroups.end(); ++it)
		{
			ItompPlanningGroupPtr group =
				boost::make_shared<ItompPlanningGroup>();
			group->name_ = (*it)->getName();
			ROS_INFO_STREAM("Planning group " << group->name_);

			const std::vector<std::string> joint_model_names =
				(*it)->getJointModelNames();

			group->num_joints_ = 0;
			std::vector<bool> active_joints;
			active_joints.resize(num_kdl_joints_, false);
			for (int i = 0; i < joint_model_names.size(); i++)
			{
				std::string joint_name = joint_model_names[i];
				map<string, string>::iterator link_name_it =
					joint_segment_mapping_.find(joint_name);
				if (link_name_it == joint_segment_mapping_.end())
				{
					ROS_ERROR(
						"Joint name %s did not have containing KDL segment.", joint_name.c_str());
					return false;
				}
				std::string link_name = link_name_it->second;
				const KDL::Segment* segment =
					&(kdl_tree_.getSegment(link_name)->second.segment);
				KDL::Joint::JointType joint_type =
					segment->getJoint().getType();
				if (joint_type != KDL::Joint::None)
				{
					ItompRobotJoint joint;
					joint.group_joint_index_ = group->num_joints_;
					joint.kdl_joint_index_ = joint.rbdl_joint_index_ =
												 kdl_tree_.getSegment(link_name)->second.q_nr;
					joint.link_name_ = link_name;
					joint.joint_name_ = segment_joint_mapping_[link_name];

					// TODO:
					joint.rbdl_affected_body_ids_ =
						rbdl_affected_body_ids_vector[joint.rbdl_joint_index_
													  + 1];

					const robot_model::JointModel * kin_model_joint =
						moveit_robot_model_->getJointModel(
							joint.joint_name_);
					if (const robot_model::RevoluteJointModel* revolute_joint =
								dynamic_cast<const robot_model::RevoluteJointModel*>(kin_model_joint))
					{
						joint.wrap_around_ = revolute_joint->isContinuous();
						joint.has_joint_limits_ = !(joint.wrap_around_);
						const robot_model::VariableBounds& bounds =
							revolute_joint->getVariableBounds(
								revolute_joint->getName());
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
						const robot_model::VariableBounds& bounds =
							prismatic_joint->getVariableBounds(
								prismatic_joint->getName());
						joint.joint_limit_min_ = bounds.min_position_;
						joint.joint_limit_max_ = bounds.max_position_;
						ROS_INFO_STREAM(
							"Setting bounds for joint[" << joint.group_joint_index_ << "][" << joint.kdl_joint_index_ << "] " << prismatic_joint->getName() << " to " << joint.joint_limit_min_ << " " << joint.joint_limit_max_);
					}
					else
					{
						ROS_WARN(
							"Cannot handle floating or planar joints yet.");
					}

					group->num_joints_++;
					group->group_joints_.push_back(joint);
					active_joints[joint.kdl_joint_index_] = true;
				}

			}
			group->fk_solver_.reset(
				new KDL::TreeFkSolverJointPosAxisPartial(kdl_tree_,
						moveit_robot_model_->getRootLinkName(),
						active_joints));

			for (int i = 0; i < group->num_joints_; i++)
			{
				group->kdl_to_group_joint_[group->group_joints_[i].kdl_joint_index_] =
					i;
			}

			planning_groups_.insert(make_pair(group->name_, group));
		}
	}
	////////////////////////////////////////////////////////////////////////////

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
                ROS_INFO("Endeffector %s for group %s is added", endeffector_name.c_str(), group_name.c_str());
			}
		}
	}

    ROS_INFO("Initialized ITOMP robot model in %s reference frame.", reference_frame_.c_str());

	return true;
}

bool fuzzyEquals(double a, double b)
{
    const double eps = 1E-15;
    return std::abs(a - b) < eps * std::max(std::abs(a), std::abs(b));
}

double solveASinXPlusBCosXIsC(double a, double b, double c)
{
    // solve a sin x + b cos x = c
    double r = sqrt(a * a + b * b);
    double alpha = atan2(a, b);

    // cos (x-alpha) = c / r
    // x = alpha +- acos (c/r)

    double t = c / r;
    if (fuzzyEquals(t, 1.0))
        t = 1.0;
    else if (fuzzyEquals(t, -1.0))
        t = -1.0;

    double v = std::min(std::max(t, -1.0), 1.0);
    double x1 = alpha + std::acos(v);
    double x2 = alpha - std::acos(v);

    while (x1 > M_PI)
        x1 -= 2 * M_PI;
    while (x1 <= -M_PI)
        x1 += 2 * M_PI;

    while (x2 > M_PI)
        x2 -= 2 * M_PI;
    while (x2 <= -M_PI)
        x2 += 2 * M_PI;

    if (std::abs(x1) < std::abs(x2))
        return x1;
    else
        return x2;
}

bool ItompRobotModel::computeInverseKinematics(const std::string& group_name, const Eigen::Affine3d& root_pose, const Eigen::Affine3d& dest_pose,
        std::vector<double>& joint_values) const
{
    // compute IK for dest_pose of endeffector

    if (group_name != "left_leg" && group_name != "right_leg")
        return false;

    const Eigen::Vector3d robot_model_right(1.0, 0.0, 0.0);
    const Eigen::Vector3d robot_model_dir(0.0, 1.0, 0.0);
    const Eigen::Vector3d robot_model_up(0.0, 0.0, 1.0);

    const std::vector<const robot_model::LinkModel*>& robot_link_models = moveit_robot_model_->getLinkModels();
    const std::vector<const robot_model::LinkModel*>& group_link_models = moveit_robot_model_->getJointModelGroup(group_name)->getLinkModels();
    std::string ee_group_name = moveit_robot_model_->getJointModelGroup(group_name)->getAttachedEndEffectorNames()[0];
    const robot_model::LinkModel* ee_link_model = moveit_robot_model_->getJointModelGroup(ee_group_name)->getLinkModels()[0];

    robot_state::RobotState zero_state(moveit_robot_model_);
    zero_state.setToDefaultValues();
    zero_state.updateLinkTransforms();

    const Eigen::Affine3d& root_transf = zero_state.getGlobalLinkTransform(robot_link_models[6]);
    const Eigen::Affine3d& ee_transf = zero_state.getGlobalLinkTransform(ee_link_model);

    //double initial_root_height = root_transf.translation()(2);
    //double max_lower_body_stretch = (root_transf.translation() - ee_transf.translation()).norm();

    const Eigen::Affine3d& hip_transf = zero_state.getGlobalLinkTransform(group_link_models[0]);
    const Eigen::Affine3d& knee_transf = zero_state.getGlobalLinkTransform(group_link_models[3]);
    const Eigen::Affine3d& ankle_transf = zero_state.getGlobalLinkTransform(group_link_models[4]);

    Eigen::Vector3d root_to_hip = hip_transf.translation() - root_transf.translation();
    Eigen::Vector3d hip_to_knee = knee_transf.translation() - hip_transf.translation();
    Eigen::Vector3d knee_to_ankle = ankle_transf.translation() - knee_transf.translation();
    Eigen::Vector3d ankle_to_ee = ee_transf.translation() - ankle_transf.translation();

    joint_values.resize(moveit_robot_model_->getJointModelGroup(group_name)->getVariableCount(), 0.0);

    // foot.p = ROOT.p + ROOT.M * (L0 + Rz0 * Ry1 * Rx2 * (L3 + Rx3 * (L4 + Rx4 * Ry5 * Rz6(I) * L7)))
    // foot.M = ROOT.M * Rz0 * Ry1 * Rx2 * Rx3 * Rx4 * Ry5 * Rz6(I)

    // set Rz6 (foot_yaw, foot) = I
    joint_values[6] = 0.0;

    // foot.p = ROOT.p + ROOT.M * (L0 + Rz0 * Ry1 * Rx2 * (L3 + Rx3 * (L4 + Rx4 * Ry5 * L7)))               (1)
    // foot.M = ROOT.M * Rz0 * Ry1 * Rx2 * Rx3 * Rx4 * Ry5                                                  (2)

    // from (1), ROOT.M^-1 * (foot.p - ROOT.p) - L0 = Rz0 * Ry1 * Rx2 * (L3 + Rx3 * (L4 + Rx4 * Ry5 * L7))  (3)
    // from (2), ROOT.M^-1 * foot.M * Ry5^-1 * Rx4^-1 * Rx3^-1 = Rz0 * Ry1 * Rx2                            (4)

    // from (3) and (4), (foot.p - ROOT.p) - ROOT.M * L0 = foot.M * Ry5^-1 * Rx4^-1 * Rx3^-1 * (L3 + Rx3 * (L4 + Rx4 * Ry5 * L7))
    //                                                   = foot.M * (Ry5^-1 * Rx4^-1 * (Rx3^-1 * L3 + L4) + L7)                                         (5)

    // from (5), foot.M^-1 * ((foot.p - ROOT.p) - ROOT.M * L0) - L7 = Ry5^-1 * Rx4^-1 * (Rx3^-1 * L3 + L4)                                              (6)

    // let foot.M^-1 * ((foot.p - ROOT.p) - ROOT.M * L0) - L7 = v
    // v = Ry5^-1 * Rx4^-1 * (Rx3^-1 * L3 + L4)                             (7)
    // Rx3 * Rx4 * Ry5 * v = (L3 + Rx3 * L4)                                (8)
    // length(v) = length(L3 + Rx3 * L4) : get theta3                       (9)

    // let Rx3^-1 * L3 + L4=k
    // from (8),
    //   cos5       0       sin5
    // ( sin4sin5   cos4    -sin4cos5 ) ( v ) = k                           (10)
    //  -cos4sin5   sin4    cos4cos5

    // solve v.x * cos5 + v.z * sin5 = k.x : get theta5                     (11)

    // solve (v.x * sin5 - v.z * cos5) * sin4 + v.y * cos4 = k.y : get theta4 (12)

    // let ROOT.M^-1 * foot.M * Ry5^-1 * Rx4^-1 * Rx3^-1 = M,
    // from (4), Rz0 * Ry1 * Rx2 = M                                        (13)
    //   c0c1       -s0c2+c0s1s2    s0s2+c0s1c2
    // ( s0c1       c0c2+s0s1s2     -c0s2+s0s1c2 ) = M                      (14)
    //   -s1        c1s2            c1c2

    // -s1  = M(2,0) : get theta1 (15)
    // s0c1 = M(1,0) : get theta0 (16)
    // c1s2 = M(2,1) : get theta2 (17)

    /////////////////////////////////////////////////////////////////////////////
    // let foot.M^-1 * ((foot.p - ROOT.p) - ROOT.M * L0) - L7 = v
    // v = Ry5^-1 * Rx4^-1 * (Rx3^-1 * L3 + L4)                             (7)
    // Rx3 * Rx4 * Ry5 * v = (L3 * Rx3 * L4)                                (8)
    // length(v) = length(L3 + Rx3 * L4) : get theta5                       (9)
    // v : hip to ankle
    // ignore x-axis diff
    // Use law of cosine for yz plane
    {
        const Eigen::Vector3d v = dest_pose.linear().inverse() *
                                  ((dest_pose.translation() - root_pose.translation()) - root_pose.linear() * root_to_hip) - ankle_to_ee;
        double yz_v_sq_dist = v.y() * v.y() + v.z() * v.z();

        double h1 = std::sqrt(hip_to_knee.y() * hip_to_knee.y() + hip_to_knee.z() * hip_to_knee.z());
        double h2 = std::sqrt(knee_to_ankle.y() * knee_to_ankle.y() + knee_to_ankle.z() * knee_to_ankle.z());
        double ph1 = std::atan2(-hip_to_knee.y(), -hip_to_knee.z());
        double ph2 = std::atan2(-knee_to_ankle.y(), -knee_to_ankle.z());

        double rho = std::acos((h1 * h1 + h2 * h2 - yz_v_sq_dist) / (2 * h1 * h2));
        joint_values[3] = -(M_PI + ph1 - ph2 - rho);
        //joint_values[3] = -(M_PI - rho) + ph2;

        // let Rx3^-1 * L3 + L4=k
        // solve v.x * cos7 + v.z * sin7 = k.x : get theta7 (11)
        // solve (v.x * sin7 - v.z * cos7) * sin6 + v.y * cos6 = k.y : get theta6 (12)
        const Eigen::Vector3d k = Eigen::AngleAxisd(-joint_values[3], Eigen::Vector3d::UnitX()) * hip_to_knee + knee_to_ankle;
        joint_values[5] = solveASinXPlusBCosXIsC(v.z(),v.x(), k.x());
        joint_values[4] = solveASinXPlusBCosXIsC(v.x() * std::sin(joint_values[5]) - v.z() * std::cos(joint_values[5]), v.y(), k.y());
    }

    // let ROOT.M^-1 * foot.M * Ry5^-1 * Rx4^-1 * Rx3^-1 = M,
    // -s3 = M(2,0) : get theta3 (15)
    // s2c3 = M(1,0) : get theta2 (16)
    // c3s4 = M(2,1) : get theta4 (17)
    {
        const Eigen::MatrixXd M = root_pose.linear().inverse() * dest_pose.linear()
                                  * Eigen::AngleAxisd(-joint_values[5], Eigen::Vector3d::UnitY())
                                  * Eigen::AngleAxisd(-joint_values[4], Eigen::Vector3d::UnitX())
                                  * Eigen::AngleAxisd(-joint_values[3], Eigen::Vector3d::UnitX());
        joint_values[1] = -std::asin(M(2, 0));
        joint_values[0] = std::asin(M(1, 0) / std::cos(joint_values[1]));
        joint_values[2] = std::asin(M(2, 1) / std::cos(joint_values[1]));
    }

    // convert Rx4 * Ry5 * Rz6 to Rz4 * Ry5 * Rx6
    {
        Eigen::Matrix3d ankle_rotation = Eigen::Matrix3d::Identity()
                                         * Eigen::AngleAxisd(joint_values[4], Eigen::Vector3d::UnitX())
                                         * Eigen::AngleAxisd(joint_values[5], Eigen::Vector3d::UnitY())
                                         * Eigen::AngleAxisd(joint_values[6], Eigen::Vector3d::UnitZ());
        Eigen::Vector3d euler_angles = ankle_rotation.eulerAngles(2, 1, 0);

        joint_values[4] = euler_angles(0);
        joint_values[5] = euler_angles(1);
        joint_values[6] = euler_angles(2);
    }

    // validate foot pos
    // foot.p = ROOT.p + ROOT.M * (L0 + Rz0 * Ry1 * Rx2 * (L3 + Rx3 * (L4 + Rx4 * Ry5 * Rz6(I) * L7)))
    // foot.M = ROOT.M * Rz0 * Ry1 * Rx2 * Rx3 * Rx4 * Ry5 * Rz6(I)
    Eigen::Vector3d evaluated_pos = root_pose.translation() + root_pose.linear()
                                    * (root_to_hip + Eigen::AngleAxisd(joint_values[0], Eigen::Vector3d::UnitZ())
                                       * Eigen::AngleAxisd(joint_values[1], Eigen::Vector3d::UnitY())
                                       * Eigen::AngleAxisd(joint_values[2], Eigen::Vector3d::UnitX())
                                       * (hip_to_knee + Eigen::AngleAxisd(joint_values[3], Eigen::Vector3d::UnitX())
                                          * (knee_to_ankle + Eigen::AngleAxisd(joint_values[4], Eigen::Vector3d::UnitZ())
                                                  * Eigen::AngleAxisd(joint_values[5], Eigen::Vector3d::UnitY())
                                                  * Eigen::AngleAxisd(joint_values[6], Eigen::Vector3d::UnitX()) * ankle_to_ee)));
    Eigen::MatrixXd evaluated_rotation = root_pose.linear()
                                         * Eigen::AngleAxisd(joint_values[0], Eigen::Vector3d::UnitZ())
                                         * Eigen::AngleAxisd(joint_values[1], Eigen::Vector3d::UnitY())
                                         * Eigen::AngleAxisd(joint_values[2], Eigen::Vector3d::UnitX())
                                         * Eigen::AngleAxisd(joint_values[3], Eigen::Vector3d::UnitX())
                                         * Eigen::AngleAxisd(joint_values[4], Eigen::Vector3d::UnitZ())
                                         * Eigen::AngleAxisd(joint_values[5], Eigen::Vector3d::UnitY())
                                         * Eigen::AngleAxisd(joint_values[6], Eigen::Vector3d::UnitX());

    return true;
}

bool ItompRobotModel::getGroupEndeffectorPos(const std::string& group_name, const robot_state::RobotState& robot_state, Eigen::Affine3d& ee_pose) const
{
    if (group_name != "left_leg" && group_name != "right_leg")
        return false;

    const robot_model::JointModelGroup* jmg = moveit_robot_model_->getJointModelGroup(group_name);
    if (jmg == NULL)
        return false;

    std::string ee_group_name = jmg->getAttachedEndEffectorNames()[0];
    const robot_model::LinkModel* ee_link_model = moveit_robot_model_->getJointModelGroup(ee_group_name)->getLinkModels()[0];

    ee_pose = robot_state.getGlobalLinkTransform(ee_link_model);

    return true;
}

bool ItompRobotModel::getGroupMaxStretch(const std::string& group_name, double& max_stretch, double& height_max_stretch_diff) const
{
    if (group_name != "left_leg" && group_name != "right_leg")
        return false;

    const robot_model::JointModelGroup* jmg = moveit_robot_model_->getJointModelGroup(group_name);
    if (jmg == NULL)
        return false;

    const std::vector<const robot_model::LinkModel*>& robot_link_models = moveit_robot_model_->getLinkModels();
    const std::vector<const robot_model::LinkModel*>& group_link_models = moveit_robot_model_->getJointModelGroup(group_name)->getLinkModels();

    robot_state::RobotState zero_state(moveit_robot_model_);
    zero_state.setToDefaultValues();
    zero_state.updateLinkTransforms();

    const Eigen::Affine3d& root_transf = zero_state.getGlobalLinkTransform(robot_link_models[6]);
    const Eigen::Affine3d& hip_transf = zero_state.getGlobalLinkTransform(group_link_models[0]);
    const Eigen::Affine3d& ankle_transf = zero_state.getGlobalLinkTransform(group_link_models[4]);

    double initial_root_height = root_transf.translation()(2);
    max_stretch = (hip_transf.translation() - ankle_transf.translation()).norm();
    height_max_stretch_diff = initial_root_height - max_stretch;

    return true;
}

bool ItompRobotModel::computeStandIKState(robot_state::RobotState& robot_state, Eigen::Affine3d& root_pose, const Eigen::Affine3d& left_foot_pose, const Eigen::Affine3d& right_foot_pose) const
{
    // adjust root_z for foot poses
    adjustRootZ("left_leg", root_pose, left_foot_pose);
    adjustRootZ("right_leg", root_pose, right_foot_pose);

    // set root transform from root_pose;
    Eigen::Vector3d euler_angles = root_pose.linear().eulerAngles(0, 1, 2);
    for (int i = 0; i < 3; ++i)
    {
        double default_value = (i == 2) ? 0.9619 : 0.0;
        robot_state.getVariablePositions()[i] = root_pose.translation()(i) - default_value;
        robot_state.getVariablePositions()[i + 3] = euler_angles(i);
    }

    std::vector<double> joint_values;
    computeInverseKinematics("left_leg", root_pose, left_foot_pose, joint_values);
    robot_state.setJointGroupPositions("left_leg", joint_values);
    computeInverseKinematics("right_leg", root_pose, right_foot_pose, joint_values);
    robot_state.setJointGroupPositions("right_leg", joint_values);

    /*
    Eigen::Vector3d evaluated_pos = root_pose.translation() + root_pose.linear()
                                    * (root_to_hip + Eigen::AngleAxisd(joint_values[0], Eigen::Vector3d::UnitZ())
                                       * Eigen::AngleAxisd(joint_values[1], Eigen::Vector3d::UnitY())
                                       * Eigen::AngleAxisd(joint_values[2], Eigen::Vector3d::UnitX())
                                       * (hip_to_knee + Eigen::AngleAxisd(joint_values[3], Eigen::Vector3d::UnitX())
                                          * (knee_to_ankle + Eigen::AngleAxisd(joint_values[4], Eigen::Vector3d::UnitX())
                                                  * Eigen::AngleAxisd(joint_values[5], Eigen::Vector3d::UnitY())
                                                  * Eigen::AngleAxisd(joint_values[6], Eigen::Vector3d::UnitZ()) * ankle_to_ee)));
                                                  */

    Eigen::MatrixXd evaluated_rotation[7];
    evaluated_rotation[0] = root_pose.linear()
                            * Eigen::AngleAxisd(joint_values[0], Eigen::Vector3d::UnitZ());
    evaluated_rotation[1] = evaluated_rotation[0]
                            * Eigen::AngleAxisd(joint_values[1], Eigen::Vector3d::UnitY());
    evaluated_rotation[2] = evaluated_rotation[1]
                            * Eigen::AngleAxisd(joint_values[2], Eigen::Vector3d::UnitX());
    evaluated_rotation[3] = evaluated_rotation[2]
                            * Eigen::AngleAxisd(joint_values[3], Eigen::Vector3d::UnitX());
    evaluated_rotation[4] = evaluated_rotation[3]
                            * Eigen::AngleAxisd(joint_values[4], Eigen::Vector3d::UnitZ());
    evaluated_rotation[5] = evaluated_rotation[4]
                            * Eigen::AngleAxisd(joint_values[5], Eigen::Vector3d::UnitY());
    evaluated_rotation[6] = evaluated_rotation[5]
                            * Eigen::AngleAxisd(joint_values[6], Eigen::Vector3d::UnitX());

    // test
    robot_state.update(true);
    Eigen::Affine3d root_transform = robot_state.getGlobalLinkTransform("pelvis_link");
    Eigen::Affine3d left_ee_transform = robot_state.getGlobalLinkTransform("left_foot_endeffector_link");
    Eigen::Affine3d right_ee_transform = robot_state.getGlobalLinkTransform("right_foot_endeffector_link");

    Eigen::Affine3d right_hip_transform = robot_state.getGlobalLinkTransform("upper_right_leg_x_link");
    Eigen::Affine3d right_knee_transform = robot_state.getGlobalLinkTransform("lower_right_leg_link");
    Eigen::Affine3d right_ankle_transform = robot_state.getGlobalLinkTransform("right_foot_x_link");

    return true;
}

bool ItompRobotModel::adjustRootZ(const std::string& group_name, Eigen::Affine3d& root_pose, const Eigen::Affine3d& dest_pose) const
{
    const std::vector<const robot_model::LinkModel*>& robot_link_models = moveit_robot_model_->getLinkModels();
    const std::vector<const robot_model::LinkModel*>& group_link_models = moveit_robot_model_->getJointModelGroup(group_name)->getLinkModels();
    std::string ee_group_name = moveit_robot_model_->getJointModelGroup(group_name)->getAttachedEndEffectorNames()[0];
    const robot_model::LinkModel* ee_link_model = moveit_robot_model_->getJointModelGroup(ee_group_name)->getLinkModels()[0];

    robot_state::RobotState robot_state(moveit_robot_model_);
    robot_state.setToDefaultValues();
    robot_state.update(true);

    Eigen::Affine3d hip_transf = robot_state.getGlobalLinkTransform(group_link_models[0]);
    Eigen::Affine3d knee_transf = robot_state.getGlobalLinkTransform(group_link_models[3]);
    Eigen::Affine3d ankle_transf = robot_state.getGlobalLinkTransform(group_link_models[4]);
    Eigen::Vector3d hip_to_knee = knee_transf.translation() - hip_transf.translation();
    Eigen::Vector3d knee_to_ankle = ankle_transf.translation() - knee_transf.translation();
    hip_to_knee(0) = knee_to_ankle(0) = 0.0;
    double max_stretch = hip_to_knee.norm() + knee_to_ankle.norm();

    Eigen::Vector3d euler_angles = root_pose.linear().eulerAngles(0, 1, 2);
    for (int i = 0; i < 3; ++i)
    {
        double default_value = (i == 2) ? 0.9619 : 0.0;
        robot_state.getVariablePositions()[i] = root_pose.translation()(i) - default_value;
        robot_state.getVariablePositions()[i + 3] = euler_angles(i);
    }
    robot_state.update(true);

    Eigen::Affine3d ee_transf = robot_state.getGlobalLinkTransform(ee_link_model);
    hip_transf = robot_state.getGlobalLinkTransform(group_link_models[0]);
    knee_transf = robot_state.getGlobalLinkTransform(group_link_models[3]);
    ankle_transf = robot_state.getGlobalLinkTransform(group_link_models[4]);
    Eigen::Vector3d ankle_to_ee = ee_transf.translation() - ankle_transf.translation();

    Eigen::Vector3d dest_ankle_pos = dest_pose.translation() - ankle_to_ee;
    Eigen::Vector3d dest_hip_to_ankle = dest_ankle_pos - hip_transf.translation();

    if (dest_hip_to_ankle.norm() <= max_stretch)
        return false; // no change
    else
    {
        double sq_max_stretch = max_stretch * max_stretch;
        double sq_dist_x_y = dest_hip_to_ankle.x() * dest_hip_to_ankle.x() + dest_hip_to_ankle.y() * dest_hip_to_ankle.y();
        if (sq_dist_x_y > sq_max_stretch)
            return false;
        double new_z = std::sqrt(sq_max_stretch - sq_dist_x_y);
        double dist_diff = new_z + dest_hip_to_ankle.z();
        root_pose.translation()(2) += dist_diff;
    }
    return true;
}

}
