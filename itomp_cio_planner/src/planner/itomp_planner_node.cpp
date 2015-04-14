#include <itomp_cio_planner/planner/itomp_planner_node.h>
#include <itomp_cio_planner/model/itomp_planning_group.h>
#include <itomp_cio_planner/trajectory/trajectory_factory.h>
#include <itomp_cio_planner/util/planning_parameters.h>
#include <itomp_cio_planner/util/joint_state_util.h>
#include <itomp_cio_planner/visualization/new_viz_manager.h>
#include <itomp_cio_planner/optimization/phase_manager.h>
#include <itomp_cio_planner/contact/ground_manager.h>
#include <kdl/jntarray.hpp>
#include <angles/angles.h>
#include <visualization_msgs/MarkerArray.h>
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
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
    {
        res.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
		return false;
    }

    GroundManager::getInstance()->initialize(planning_scene);

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

		// for each planning group
		for (unsigned int i = 0; i != planning_group_names.size(); ++i)
		{
			ros::WallTime create_time = ros::WallTime::now();

            const ItompPlanningGroupConstPtr planning_group = itomp_robot_model_->getPlanningGroup(planning_group_names[i]);

            sensor_msgs::JointState goal_joint_state = getGoalStateFromGoalConstraints(itomp_robot_model_, req);

			/// optimize
            trajectory_->setGroupGoalState(goal_joint_state, planning_group, itomp_robot_model_, req.trajectory_constraints, req.path_constraints, true);
            itomp_trajectory_->setGoalState(goal_joint_state, planning_group, itomp_robot_model_, req.trajectory_constraints);

            robot_state::RobotState goal_state(*initial_robot_state);
            //robot_state::jointStateToRobotState(goal_joint_state, goal_state);
            for (unsigned int i = 0; i < goal_joint_state.name.size(); ++i)
            {
                if (goal_joint_state.name[i] != "")
                    goal_state.setVariablePosition(goal_joint_state.name[i], goal_joint_state.position[i]);
            }
            goal_state.update(true);

            if (setSupportFoot(*initial_robot_state, goal_state) == false)
            {
                res.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
                return false;
            }

            adjustStartGoalPositions();

            optimizer_ = boost::make_shared<ItompOptimizer>(0, trajectory_, itomp_trajectory_,
						 itomp_robot_model_, planning_scene, planning_group, planning_start_time,
                         trajectory_start_time, req.trajectory_constraints.constraints);

			optimizer_->optimize();

            const PlanningInfo& planning_info = optimizer_->getPlanningInfo();

            planning_info_manager_.write(c, i, planning_info);

            ROS_INFO("Optimization of group %s took %f sec", planning_group_names[i].c_str(), (ros::WallTime::now() - create_time).toSec());

            const double FAILURE_THRESHOLD = 100000.0;
            if (planning_info.cost > FAILURE_THRESHOLD)
            {
                res.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
                return false;
            }
        }
	}
	planning_info_manager_.printSummary();

    if (itomp_trajectory_->avoidNeighbors(req.trajectory_constraints.constraints) == false)
    {
        res.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
        return false;
    }

    // write goal state
    writeWaypoint();
    writeTrajectory();


	// return trajectory
    fillInResult(initial_robot_state, res);

    GroundManager::getInstance()->destroy();

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
    int trajectory_index = 0;
    ros::NodeHandle node_handle("itomp_planner");
    node_handle.getParam("agent_id", agent_id);
    node_handle.getParam("agent_trajectory_index", trajectory_index);

    PhaseManager::getInstance()->agent_id_ = agent_id;

    std::ifstream trajectory_file;
    std::stringstream ss;
    ss << "agent_" << agent_id << "_" << trajectory_index - 1<< "_waypoint.txt";
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
    int trajectory_index = 0;
    ros::NodeHandle node_handle("itomp_planner");
    node_handle.getParam("agent_id", agent_id);
    node_handle.getParam("agent_trajectory_index", trajectory_index);

    std::ofstream trajectory_file;
    std::stringstream ss;
    ss << "agent_" << agent_id << "_" << trajectory_index << "_waypoint.txt";
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
        int trajectory_index = 0;
        while (true)
        {
            stringstream ss;
            ss << "agent_" << agent_id << "_" << trajectory_index << "_waypoint.txt";
            if (remove(ss.str().c_str()) != 0)
                break;
            ++trajectory_index;
        }

        trajectory_index = 0;
        while (true)
        {
            std::stringstream ss;
            ss << "trajectory_out_" << std::setfill('0') << std::setw(4) << agent_id << "_" << std::setfill('0') << std::setw(4) << trajectory_index << ".txt";
            if (remove(ss.str().c_str()) != 0)
                break;
            ++trajectory_index;
        }
        if (trajectory_index == 0)
            break;

        ++agent_id;
    }
}

bool ItompPlannerNode::setSupportFoot(const robot_state::RobotState& initial_state, const robot_state::RobotState& goal_state)
{
    int support_foot = 3; // {0 = none, 1 = left, 2 = right, 3 = any}
    int back_foot = 3; // any
    double start_orientation = initial_state.getVariablePosition("base_revolute_joint_z");
    const Eigen::Vector3d start_pos = initial_state.getGlobalLinkTransform("pelvis_link").translation();
    const Eigen::Vector3d start_dir = Eigen::AngleAxisd(start_orientation, Eigen::Vector3d::UnitZ()) * Eigen::Vector3d::UnitY();
    const Eigen::Vector3d start_left_foot = initial_state.getGlobalLinkTransform("left_foot_endeffector_link").translation();
    const Eigen::Vector3d start_right_foot = initial_state.getGlobalLinkTransform("right_foot_endeffector_link").translation();

    const Eigen::Vector3d goal_pos = goal_state.getGlobalLinkTransform("pelvis_link").translation();
    double goal_orientation = goal_state.getVariablePosition("base_revolute_joint_z");

    double left_dot = start_dir.dot(start_left_foot - start_pos);
    double right_dot = start_dir.dot(start_right_foot - start_pos);
    if (std::abs(left_dot - right_dot) < 0.1)
    {
        back_foot = 3;
    }
    else if (left_dot > right_dot)
    {
        back_foot = 1; // left
    }
    else
    {
        back_foot = 2; // right
    }

    // compute angle between start / goal
    Eigen::Vector3d move_pos = goal_pos - start_pos;
    move_pos.z() = 0.0;
    double move_orientation = 0.0;
    double move_orientation_diff = 0.0;
    if (move_pos.norm() > ITOMP_EPS)
    {
        move_orientation = std::atan2(move_pos.y(), move_pos.x()) - M_PI_2;
        move_orientation_diff = move_orientation - start_orientation;
        if (move_orientation_diff > M_PI)
            move_orientation_diff -= 2.0 * M_PI;
        else if (move_orientation_diff <= -M_PI)
            move_orientation_diff += 2.0 * M_PI;
    }

    if (std::abs(move_orientation_diff) < M_PI / 6.0)
    {
        support_foot = back_foot;
    }
    else if (std::abs(move_orientation_diff) > M_PI / 5.0 * 6.0)
    {
        switch (back_foot)
        {
        case 0:
        case 3:
            support_foot = back_foot;
            break;

        case 1:
            support_foot = 2;
            break;

        case 2:
            support_foot = 1;
            break;

        }
    }
    else if (move_orientation_diff > 0)
    {
        switch (back_foot)
        {
        case 0:
            support_foot = 0;
            break;

        case 1:
        case 2:
        case 3:
            support_foot = 2;
            break;
        }
    }
    else // diff_orientation < 0
    {
        switch (back_foot)
        {
        case 0:
            support_foot = 0;
            break;

        case 1:
        case 2:
        case 3:
            support_foot = 1;
            break;
        }
    }

    PhaseManager::getInstance()->support_foot_ = support_foot;
    ROS_INFO("Ori dir : %f %f %f", start_dir.x(), start_dir.y(), start_dir.z());
    ROS_INFO("left_foot : %f %f %f", start_left_foot.x(), start_left_foot.y(), start_left_foot.z());
    ROS_INFO("right_foot : %f %f %f", start_right_foot.x(), start_right_foot.y(), start_right_foot.z());
    ROS_INFO("init ori : %f move ori : %f goal ori : %f", start_orientation, move_orientation, goal_orientation);
    ROS_INFO("Support foot : %d", support_foot);

    double goal_orientation_diff = goal_orientation - start_orientation;
    if (goal_orientation_diff > M_PI)
        goal_orientation_diff -= 2.0 * M_PI;
    else if (goal_orientation_diff <= -M_PI)
        goal_orientation_diff += 2.0 * M_PI;
    if (std::abs(goal_orientation_diff) < M_PI / 6.0)
    {
        // always ok
    }
    else if (std::abs(move_orientation_diff) < M_PI / 5.0 * 6.0)
    {
        double goal_move_orientation_diff = goal_orientation - move_orientation;
        if (goal_move_orientation_diff > M_PI)
            goal_move_orientation_diff -= 2.0 * M_PI;
        else if (goal_move_orientation_diff <= -M_PI)
            goal_move_orientation_diff += 2.0 * M_PI;
        if (std::abs(goal_move_orientation_diff) > M_PI_4)
            return false;
    }
    else
    {
            return false;
    }

    return support_foot != 0;
}

void ItompPlannerNode::writeTrajectory()
{
    /*
    std::ofstream trajectory_file;
    trajectory_file.open("trajectory_out.txt");
    itomp_trajectory_->printTrajectory(trajectory_file);
    trajectory_file.close();
    */

    int agent_id = 0;
    int trajectory_index = 0;

    ros::NodeHandle node_handle("itomp_planner");
    node_handle.getParam("agent_id", agent_id);
    node_handle.getParam("agent_trajectory_index", trajectory_index);

    std::stringstream ss;
    ss << "trajectory_out_" << std::setfill('0') << std::setw(4) << agent_id << "_" << std::setfill('0') << std::setw(4) << trajectory_index << ".txt";
    std::ofstream trajectory_file;
    trajectory_file.open(ss.str().c_str());
    itomp_trajectory_->printTrajectory(trajectory_file, 0, 40);
    trajectory_file.close();
}

void ItompPlannerNode::adjustStartGoalPositions()
{
    unsigned int goal_index = itomp_trajectory_->getNumPoints() - 1;

    ElementTrajectoryPtr& joint_traj = itomp_trajectory_->getElementTrajectory(ItompTrajectory::COMPONENT_TYPE_POSITION, ItompTrajectory::SUB_COMPONENT_TYPE_JOINT);
    Eigen::MatrixXd::RowXpr traj_start_point = joint_traj->getTrajectoryPoint(0);
    Eigen::MatrixXd::RowXpr traj_goal_point = joint_traj->getTrajectoryPoint(goal_index);

    // root translation joints
    Eigen::Vector3d start_pos, goal_pos;
    for (int i = 0; i < 3; ++i)
    {
        start_pos(i) = traj_start_point(i);
        goal_pos(i) = traj_goal_point(i);
    }
    Eigen::Vector3d pos_out, normal_out, contact_normal_out;
    GroundManager::getInstance()->getNearestContactPosition(start_pos, Eigen::Vector3d::UnitZ(), pos_out, normal_out, contact_normal_out, false, true);
    start_pos = pos_out;
    GroundManager::getInstance()->getNearestContactPosition(goal_pos, Eigen::Vector3d::UnitZ(), pos_out, normal_out, contact_normal_out, false, true);
    goal_pos = pos_out;
    traj_start_point(2) = start_pos(2);
    traj_goal_point(2) = goal_pos(2);
    std::vector<unsigned int> translation_joints;
    translation_joints.push_back(2);
    itomp_trajectory_->interpolate(0, 60, ItompTrajectory::SUB_COMPONENT_TYPE_JOINT, &translation_joints);
    PhaseManager::getInstance()->initial_goal_pos = goal_pos;

    // root rotation joints
    std::vector<unsigned int> rotation_joints;
    for (int i = 3; i < 6; ++i)
    {
        double start_angle = traj_start_point(i);
        double goal_angle = traj_goal_point(i);
        if (start_angle > M_PI)
            start_angle -= 2.0 * M_PI;
        else if (start_angle <= -M_PI)
            start_angle += 2.0 * M_PI;
        if (goal_angle - start_angle > M_PI)
            goal_angle -= 2.0 * M_PI;
        else if (goal_angle - start_angle <= -M_PI)
            goal_angle += 2.0 * M_PI;
        traj_start_point(i) = start_angle;
        traj_goal_point(i) = goal_angle;

        rotation_joints.push_back(i);
    }


    // knee
    int left_hip = robot_model_->getVariableIndex("upper_left_leg_x_joint");
    int right_hip = robot_model_->getVariableIndex("upper_right_leg_x_joint");
    int left_knee = robot_model_->getVariableIndex("lower_left_leg_joint");
    int right_knee = robot_model_->getVariableIndex("lower_right_leg_joint");
    int left_foot = robot_model_->getVariableIndex("left_foot_x_joint");
    int right_foot = robot_model_->getVariableIndex("right_foot_x_joint");
    std::vector<unsigned int> support_leg_joints;
    std::vector<unsigned int> free_leg_joints;

    if (PhaseManager::getInstance()->support_foot_ & 0x1)
    {
        support_leg_joints.push_back(left_knee);
        support_leg_joints.push_back(left_hip);
        support_leg_joints.push_back(left_foot);
        free_leg_joints.push_back(right_knee);
        free_leg_joints.push_back(right_hip);
        free_leg_joints.push_back(right_foot);
    }
    else
    {
        free_leg_joints.push_back(left_knee);
        free_leg_joints.push_back(left_hip);
        free_leg_joints.push_back(left_foot);
        support_leg_joints.push_back(right_knee);
        support_leg_joints.push_back(right_hip);
        support_leg_joints.push_back(right_foot);
    }

    const double default_angle = M_PI_4;

    itomp_trajectory_->interpolate(20, 40, ItompTrajectory::SUB_COMPONENT_TYPE_JOINT, &support_leg_joints);

    for (int j = 8; j <= 12; ++j)
    {
        (*joint_traj)(j, free_leg_joints[0]) = -default_angle;
        (*joint_traj)(j, free_leg_joints[1]) = default_angle * 0.5;
        (*joint_traj)(j, free_leg_joints[2]) = default_angle * 0.5;
    }
    for (int j = 28; j <= 32; ++j)
    {
        (*joint_traj)(j, support_leg_joints[0]) = -default_angle;
        (*joint_traj)(j, support_leg_joints[1]) = default_angle * 0.5;
        (*joint_traj)(j, support_leg_joints[2]) = default_angle * 0.5;
    }
    for (int j = 48; j <= 52; ++j)
    {
        (*joint_traj)(j, free_leg_joints[0]) = -default_angle;
        (*joint_traj)(j, free_leg_joints[1]) = default_angle * 0.5;
        (*joint_traj)(j, free_leg_joints[2]) = default_angle * 0.5;
    }

    itomp_trajectory_->interpolate(0, 10, ItompTrajectory::SUB_COMPONENT_TYPE_JOINT, &free_leg_joints);
    itomp_trajectory_->interpolate(10, 20, ItompTrajectory::SUB_COMPONENT_TYPE_JOINT, &free_leg_joints);
    itomp_trajectory_->interpolate(20, 30, ItompTrajectory::SUB_COMPONENT_TYPE_JOINT, &support_leg_joints);
    itomp_trajectory_->interpolate(30, 40, ItompTrajectory::SUB_COMPONENT_TYPE_JOINT, &support_leg_joints);
    itomp_trajectory_->interpolate(40, 50, ItompTrajectory::SUB_COMPONENT_TYPE_JOINT, &free_leg_joints);
    itomp_trajectory_->interpolate(50, 60, ItompTrajectory::SUB_COMPONENT_TYPE_JOINT, &free_leg_joints);



    itomp_trajectory_->copy(60, 20, ItompTrajectory::SUB_COMPONENT_TYPE_JOINT, &rotation_joints);
    itomp_trajectory_->interpolate(0, 20, ItompTrajectory::SUB_COMPONENT_TYPE_JOINT, &rotation_joints);
    itomp_trajectory_->interpolate(20, 60, ItompTrajectory::SUB_COMPONENT_TYPE_JOINT, &rotation_joints);
    //itomp_trajectory_->interpolateStartEnd(ItompTrajectory::SUB_COMPONENT_TYPE_JOINT, &rotation_joints);

    /*
    std::vector<unsigned int> z_joints;
    z_joints.push_back(0);
    z_joints.push_back(1);
    z_joints.push_back(2);
    itomp_trajectory_->interpolate(0, 60, ItompTrajectory::SUB_COMPONENT_TYPE_JOINT, &z_joints);
    itomp_trajectory_->copy(0, 20, ItompTrajectory::SUB_COMPONENT_TYPE_JOINT, &z_joints);
    itomp_trajectory_->copy(30, 40, ItompTrajectory::SUB_COMPONENT_TYPE_JOINT, &z_joints);
    itomp_trajectory_->interpolate(0, 20, ItompTrajectory::SUB_COMPONENT_TYPE_JOINT, &z_joints);
    itomp_trajectory_->interpolate(20, 40, ItompTrajectory::SUB_COMPONENT_TYPE_JOINT, &z_joints);
    itomp_trajectory_->interpolate(40, 60, ItompTrajectory::SUB_COMPONENT_TYPE_JOINT, &z_joints);
    itomp_trajectory_->copy(0, 59, ItompTrajectory::SUB_COMPONENT_TYPE_JOINT, &z_joints);
    itomp_trajectory_->interpolate(0, 59, ItompTrajectory::SUB_COMPONENT_TYPE_JOINT, &z_joints);
    */
}

} // namespace
