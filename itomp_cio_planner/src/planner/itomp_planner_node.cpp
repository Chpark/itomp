#include <itomp_cio_planner/planner/itomp_planner_node.h>
#include <itomp_cio_planner/model/itomp_planning_group.h>
#include <itomp_cio_planner/trajectory/trajectory_factory.h>
#include <itomp_cio_planner/util/planning_parameters.h>
#include <itomp_cio_planner/util/joint_state_util.h>
#include <itomp_cio_planner/visualization/new_viz_manager.h>
#include <itomp_cio_planner/optimization/phase_manager.h>
#include <kdl/jntarray.hpp>
#include <angles/angles.h>
#include <visualization_msgs/MarkerArray.h>
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <ros/ros.h>

using namespace std;

namespace itomp_cio_planner
{

ItompPlannerNode::ItompPlannerNode(const robot_model::RobotModelConstPtr& model) :
	robot_model_(model)
{

}

ItompPlannerNode::~ItompPlannerNode()
{
    NewVizManager::getInstance()->destroy();
    TrajectoryFactory::getInstance()->destroy();
    PlanningParameters::getInstance()->destroy();

    optimizer_.reset();
    trajectory_.reset();
    itomp_trajectory_.reset();
    itomp_robot_model_.reset();
}

bool ItompPlannerNode::init()
{
	// load parameters
	PlanningParameters::getInstance()->initFromNodeHandle();

	// build itomp robot model
	itomp_robot_model_ = boost::make_shared<ItompRobotModel>();
	if (!itomp_robot_model_->init(robot_model_))
		return false;

	NewVizManager::getInstance()->initialize(itomp_robot_model_);

    TrajectoryFactory::getInstance()->initialize(TrajectoryFactory::TRAJECTORY_CIO);

    trajectory_.reset(TrajectoryFactory::getInstance()->CreateFullTrajectory(itomp_robot_model_,
                      PlanningParameters::getInstance()->getTrajectoryDuration(),
                      PlanningParameters::getInstance()->getTrajectoryDiscretization(),
                      PlanningParameters::getInstance()->getPhaseDuration()));
    itomp_trajectory_.reset(
        TrajectoryFactory::getInstance()->CreateItompTrajectory(itomp_robot_model_,
                PlanningParameters::getInstance()->getTrajectoryDuration(),
                PlanningParameters::getInstance()->getTrajectoryDiscretization(),
                PlanningParameters::getInstance()->getPhaseDuration()));

    deleteWaypointFiles();

	ROS_INFO("Initialized ITOMP planning service...");

	return true;
}

bool ItompPlannerNode::planTrajectory(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                      const planning_interface::MotionPlanRequest &req,
                                      planning_interface::MotionPlanResponse &res)
{
	// reload parameters
	PlanningParameters::getInstance()->initFromNodeHandle();

	if (!validateRequest(req))
		return false;

    double trajectory_start_time = req.start_state.joint_state.header.stamp.toSec();
    robot_state::RobotStatePtr initial_robot_state = planning_scene->getCurrentStateUpdated(req.start_state);

	// generate planning group list
	vector<string> planning_group_names = getPlanningGroups(req.group_name);
    planning_info_manager_.reset(PlanningParameters::getInstance()->getNumTrials(), planning_group_names.size());

	for (int c = 0; c < PlanningParameters::getInstance()->getNumTrials(); ++c)
	{
		double planning_start_time = ros::Time::now().toSec();

		ROS_INFO("Planning Trial [%d]", c);

		// initialize trajectory with start state
        trajectory_->setStartState(req.start_state.joint_state, itomp_robot_model_, true);
        itomp_trajectory_->setStartState(req.start_state.joint_state, itomp_robot_model_);

        // read start state
        readWaypoint(initial_robot_state);
        setSupportFoot(initial_robot_state);

		// for each planning group
		for (unsigned int i = 0; i != planning_group_names.size(); ++i)
		{
			ros::WallTime create_time = ros::WallTime::now();

            const ItompPlanningGroupConstPtr planning_group = itomp_robot_model_->getPlanningGroup(planning_group_names[i]);

			/// optimize
            trajectory_->setGroupGoalState(getGoalStateFromGoalConstraints(itomp_robot_model_, req), planning_group,
                                           itomp_robot_model_, req.trajectory_constraints, req.path_constraints,
                                           true);
            itomp_trajectory_->setGoalState(getGoalStateFromGoalConstraints(itomp_robot_model_, req), planning_group,
                                            itomp_robot_model_, req.trajectory_constraints);

            optimizer_ = boost::make_shared<ItompOptimizer>(0, trajectory_, itomp_trajectory_,
						 itomp_robot_model_, planning_scene, planning_group, planning_start_time,
						 trajectory_start_time, req.path_constraints);

			optimizer_->optimize();

			planning_info_manager_.write(c, i, optimizer_->getPlanningInfo());

            ROS_INFO("Optimization of group %s took %f sec", planning_group_names[i].c_str(), (ros::WallTime::now() - create_time).toSec());

            std::ofstream trajectory_file;
            trajectory_file.open("trajectory_out.txt");
            itomp_trajectory_->printTrajectory(trajectory_file);
            trajectory_file.close();
        }
	}
	planning_info_manager_.printSummary();

    // write goal state
    writeWaypoint();


	// return trajectory
    fillInResult(initial_robot_state, res);

	return true;
}

bool ItompPlannerNode::validateRequest(const planning_interface::MotionPlanRequest &req)
{
    ROS_INFO("Received planning request ... planning group : %s", req.group_name.c_str());
    ROS_INFO("Trajectory Duration : %f", PlanningParameters::getInstance()->getTrajectoryDuration());

	// check goal constraint
	ROS_INFO("Validate planning request goal state ...");
    sensor_msgs::JointState goal_joint_state = jointConstraintsToJointState(req.goal_constraints);
	if (goal_joint_state.name.size() != goal_joint_state.position.size())
	{
		ROS_ERROR("Invalid goal");
		return false;
	}
    ROS_INFO("Goal constraint has %d/%d joints", goal_joint_state.name.size(), req.start_state.joint_state.name.size());

	for (unsigned int i = 0; i < goal_joint_state.name.size(); i++)
	{
        ROS_INFO("%s : %f", goal_joint_state.name[i].c_str(), goal_joint_state.position[i]);
	}

	return true;
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
                                    planning_interface::MotionPlanResponse &res)
{
	int num_all_joints = robot_state->getVariableCount();

    const ElementTrajectoryPtr& joint_trajectory = itomp_trajectory_->getElementTrajectory(ItompTrajectory::COMPONENT_TYPE_POSITION,
            ItompTrajectory::SUB_COMPONENT_TYPE_JOINT);
    ROS_ASSERT(num_all_joints == joint_trajectory->getNumElements());

    res.trajectory_ = boost::make_shared<robot_trajectory::RobotTrajectory>(itomp_robot_model_->getMoveitRobotModel(), "");

	robot_state::RobotState ks = *robot_state;
	std::vector<double> positions(num_all_joints);
	double dt = trajectory_->getDiscretization();
    // TODO:
    int num_return_points = 41;
    for (std::size_t i = 0; i < num_return_points; ++i)
	{
		for (std::size_t j = 0; j < num_all_joints; j++)
		{
            positions[j] = (*joint_trajectory)(i, j);
		}

		ks.setVariablePositions(&positions[0]);
		// TODO: copy vel/acc
		ks.update();

		res.trajectory_->addSuffixWayPoint(ks, dt);
	}
	res.error_code_.val = moveit_msgs::MoveItErrorCodes::SUCCESS;

	// print results
	if (PlanningParameters::getInstance()->getPrintPlanningInfo())
	{
        const std::vector<std::string>& joint_names = res.trajectory_->getFirstWayPoint().getVariableNames();
		for (int j = 0; j < num_all_joints; j++)
			printf("%s ", joint_names[j].c_str());
		printf("\n");
        for (int i = 0; i < num_return_points; ++i)
		{
			for (int j = 0; j < num_all_joints; j++)
			{
                printf("%f ", res.trajectory_->getWayPoint(i).getVariablePosition(j));
			}
			printf("\n");
		}
	}
}

void ItompPlannerNode::readWaypoint(robot_state::RobotStatePtr& robot_state)
{
    double value;

    int agent_id = 0;
    ros::NodeHandle node_handle("itomp_planner");
    node_handle.getParam("agent_id", agent_id);

    std::ifstream trajectory_file;
    std::stringstream ss;
    ss << "agent_" << agent_id << "_waypoint.txt";
    trajectory_file.open(ss.str().c_str());
    if (trajectory_file.is_open())
    {
        ElementTrajectoryPtr& joint_pos_trajectory = itomp_trajectory_->getElementTrajectory(ItompTrajectory::COMPONENT_TYPE_POSITION,
                ItompTrajectory::SUB_COMPONENT_TYPE_JOINT);
        ElementTrajectoryPtr& joint_vel_trajectory = itomp_trajectory_->getElementTrajectory(ItompTrajectory::COMPONENT_TYPE_VELOCITY,
                ItompTrajectory::SUB_COMPONENT_TYPE_JOINT);
        ElementTrajectoryPtr& joint_acc_trajectory = itomp_trajectory_->getElementTrajectory(ItompTrajectory::COMPONENT_TYPE_ACCELERATION,
                ItompTrajectory::SUB_COMPONENT_TYPE_JOINT);
        int num_elements = joint_pos_trajectory->getNumElements();

        for (int i = 0; i < num_elements; ++i)
        {
            trajectory_file >> value;

            (*joint_pos_trajectory)(0, i) = value;
            robot_state->getVariablePositions()[i] = value;
        }


        for (int i = 0; i < num_elements; ++i)
        {
            trajectory_file >> value;

            (*joint_vel_trajectory)(0, i) = value;
            robot_state->getVariableVelocities()[i] = value;
        }

        for (int i = 0; i < num_elements; ++i)
        {
            trajectory_file >> value;

            (*joint_acc_trajectory)(0, i) = value;
            robot_state->getVariableAccelerations()[i] = value;
        }


        trajectory_file.close();

        robot_state->update(true);
    }
}

void ItompPlannerNode::writeWaypoint()
{
    int agent_id = 0;
    ros::NodeHandle node_handle("itomp_planner");
    node_handle.getParam("agent_id", agent_id);

    std::ofstream trajectory_file;
    std::stringstream ss;
    ss << "agent_" << agent_id << "_waypoint.txt";
    trajectory_file.open(ss.str().c_str());
    trajectory_file.precision(std::numeric_limits<double>::digits10);

    int last_point = 40;

    const ElementTrajectoryPtr& joint_pos_trajectory = itomp_trajectory_->getElementTrajectory(ItompTrajectory::COMPONENT_TYPE_POSITION,
            ItompTrajectory::SUB_COMPONENT_TYPE_JOINT);
    const ElementTrajectoryPtr& joint_vel_trajectory = itomp_trajectory_->getElementTrajectory(ItompTrajectory::COMPONENT_TYPE_VELOCITY,
            ItompTrajectory::SUB_COMPONENT_TYPE_JOINT);
    const ElementTrajectoryPtr& joint_acc_trajectory = itomp_trajectory_->getElementTrajectory(ItompTrajectory::COMPONENT_TYPE_ACCELERATION,
            ItompTrajectory::SUB_COMPONENT_TYPE_JOINT);
    int num_elements = joint_pos_trajectory->getNumElements();

    for (int i = 0; i < num_elements; ++i)
    {
        trajectory_file << (*joint_pos_trajectory)(last_point, i) << " ";
    }
    trajectory_file << std::endl;


    for (int i = 0; i < num_elements; ++i)
    {
        trajectory_file << (*joint_vel_trajectory)(last_point, i) << " ";
    }
    trajectory_file << std::endl;

    for (int i = 0; i < num_elements; ++i)
    {
        trajectory_file << (*joint_acc_trajectory)(last_point, i) << " ";
    }
    trajectory_file << std::endl;


    trajectory_file.close();
}

void ItompPlannerNode::deleteWaypointFiles()
{
    int agent_id = 0;
    while (true)
    {
        stringstream ss;
        ss << "agent_" << agent_id++ << "_waypoint.txt";
        if (remove(ss.str().c_str()) != 0)
            break;
    }
}

void ItompPlannerNode::setSupportFoot(robot_state::RobotStatePtr& robot_state)
{
    int support_foot = 0; // any
    double orientation = robot_state->getVariablePosition("base_revolute_joint_z");
    const Eigen::Vector3d root_pos = robot_state->getGlobalLinkTransform("pelvis_link").translation();
    const Eigen::Vector3d dir = Eigen::AngleAxisd(orientation, Eigen::Vector3d::UnitZ()) * Eigen::Vector3d::UnitY();
    const Eigen::Vector3d left_foot = robot_state->getGlobalLinkTransform("left_foot_endeffector_link").translation();
    const Eigen::Vector3d right_foot = robot_state->getGlobalLinkTransform("right_foot_endeffector_link").translation();
    double left_dot = dir.dot(left_foot - root_pos);
    double right_dot = dir.dot(right_foot - root_pos);
    if (std::abs(left_dot - right_dot) < 0.1)
    {
        support_foot = 0;
    }
    else if (left_dot > right_dot)
    {
        support_foot = 1; // left
    }
    else
    {
        support_foot = 2; // right
    }

    PhaseManager::getInstance()->support_foot_ = support_foot;
    ROS_INFO("Ori dir : %f %f %f", dir.x(), dir.y(), dir.z());
    ROS_INFO("left_foot : %f %f %f", left_foot.x(), left_foot.y(), left_foot.z());
    ROS_INFO("right_foot : %f %f %f", right_foot.x(), right_foot.y(), right_foot.z());
    ROS_INFO("Support foot : %d", support_foot);
    ROS_INFO("zz");
}

} // namespace
