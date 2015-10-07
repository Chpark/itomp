#include <itomp_cio_planner/planner/itomp_planner_node.h>
#include <itomp_cio_planner/model/itomp_planning_group.h>
#include <itomp_cio_planner/model/itomp_robot_model_ik.h>
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
        ROS_INFO("Planning failure - invalid planning request");
        res.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
		return false;
    }

    // set trajectory to zero
    itomp_trajectory_->reset();

    GroundManager::getInstance()->initialize(planning_scene);

    double trajectory_start_time = req.start_state.joint_state.header.stamp.toSec();
    robot_state::RobotStatePtr initial_robot_state = planning_scene->getCurrentStateUpdated(req.start_state);

	// generate planning group list
	vector<string> planning_group_names = getPlanningGroups(req.group_name);
    planning_info_manager_.reset(PlanningParameters::getInstance()->getNumTrials(), planning_group_names.size());

	for (int c = 0; c < PlanningParameters::getInstance()->getNumTrials(); ++c)
	{
		double planning_start_time = ros::Time::now().toSec();

        //ROS_INFO("Planning Trial [%d]", c);

		// initialize trajectory with start state
        itomp_trajectory_->setStartState(req.start_state.joint_state, itomp_robot_model_);

        // read start state
        //bool read_start_state_from_previous_step = readWaypoint(initial_robot_state);

		// for each planning group
		for (unsigned int i = 0; i != planning_group_names.size(); ++i)
		{
			ros::WallTime create_time = ros::WallTime::now();

            const ItompPlanningGroupConstPtr planning_group = itomp_robot_model_->getPlanningGroup(planning_group_names[i]);

            sensor_msgs::JointState goal_joint_state = getGoalStateFromGoalConstraints(itomp_robot_model_, req);

			/// optimize
            itomp_trajectory_->setGoalState(goal_joint_state, planning_group, itomp_robot_model_, req.trajectory_constraints);

            robot_state::RobotState goal_state(*initial_robot_state);
            //robot_state::jointStateToRobotState(goal_joint_state, goal_state);
            for (unsigned int j = 0; j < goal_joint_state.name.size(); ++j)
            {
                if (goal_joint_state.name[j] != "")
                    goal_state.setVariablePosition(goal_joint_state.name[j], goal_joint_state.position[j]);
            }
            goal_state.update(true);

            //if (!adjustStartGoalPositions(*initial_robot_state, goal_state, read_start_state_from_previous_step))
              //  res.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;

            optimizer_ = boost::make_shared<ItompOptimizer>(0, itomp_trajectory_,
						 itomp_robot_model_, planning_scene, planning_group, planning_start_time,
                         trajectory_start_time, req.trajectory_constraints.constraints);

            optimizer_->optimize();

            const PlanningInfo& planning_info = optimizer_->getPlanningInfo();

            planning_info_manager_.write(c, i, planning_info);

            ROS_INFO("Optimization of group %s took %f sec", planning_group_names[i].c_str(), (ros::WallTime::now() - create_time).toSec());

            if (planning_info.cost > PlanningParameters::getInstance()->getFailureCost())
            {
                //res.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
                ROS_INFO("Planning failure - cost : %f", planning_info.cost);
                //return false;
            }
        }
	}
    if (PlanningParameters::getInstance()->getPrintPlanningInfo())
        planning_info_manager_.printSummary();

    /*
    if (itomp_trajectory_->avoidNeighbors(req.trajectory_constraints.constraints) == false)
    {
        ROS_INFO("Planning failure - collision with neighbor");
        res.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
        return false;
    }
    */

    // write goal state
    //writeWaypoint();
    //writeTrajectory();


	// return trajectory
    fillInResult(initial_robot_state, res);

    GroundManager::getInstance()->destroy();

	return true;
}

bool ItompPlannerNode::validateRequest(const planning_interface::MotionPlanRequest &req)
{
    ROS_INFO("Received planning request ... planning group : %s", req.group_name.c_str());
    if (PlanningParameters::getInstance()->getPrintPlanningInfo())
        ROS_INFO("Trajectory Duration : %f", PlanningParameters::getInstance()->getTrajectoryDuration());

	// check goal constraint
    if (PlanningParameters::getInstance()->getPrintPlanningInfo())
        ROS_INFO("Validate planning request goal state ...");
    sensor_msgs::JointState goal_joint_state = jointConstraintsToJointState(req.goal_constraints);
	if (goal_joint_state.name.size() != goal_joint_state.position.size())
	{
		ROS_ERROR("Invalid goal");
		return false;
	}

    if (PlanningParameters::getInstance()->getPrintPlanningInfo())
	{
        ROS_INFO("Goal constraint has %u/%u joints", goal_joint_state.name.size(), req.start_state.joint_state.name.size());

        for (unsigned int i = 0; i < goal_joint_state.name.size(); i++)
        {
            ROS_INFO("%s : %f", goal_joint_state.name[i].c_str(), goal_joint_state.position[i]);
        }
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
    double dt = itomp_trajectory_->getDiscretization();
    // TODO:
    int num_return_points = joint_trajectory->getNumPoints();
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

bool ItompPlannerNode::readWaypoint(robot_state::RobotStatePtr& robot_state)
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
    ss << "agent_" << agent_id << "_" << trajectory_index - 1 << "_waypoint.txt";
    trajectory_file.open(ss.str().c_str());
    ROS_INFO("read waypoint file : %s", ss.str().c_str());

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

        return true;
    }
    return false;
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

bool ItompPlannerNode::adjustStartGoalPositions(robot_state::RobotState& initial_state, robot_state::RobotState& goal_state, bool read_start_state_from_previous_step)
{
    const double MIN_ANKLE_Z_ROTATION_ANGLE = M_PI / 60.0;
    const double GOAL_MOVE_ORIENTATION_BOUND = M_PI / 6.0;

    unsigned int goal_index = itomp_trajectory_->getNumPoints() - 1;
    unsigned int mid_index = goal_index / 2;
    ElementTrajectoryPtr& joint_traj = itomp_trajectory_->getElementTrajectory(ItompTrajectory::COMPONENT_TYPE_POSITION,
                                       ItompTrajectory::SUB_COMPONENT_TYPE_JOINT);
    Eigen::MatrixXd::RowXpr traj_start_point = joint_traj->getTrajectoryPoint(0);
    Eigen::MatrixXd::RowXpr traj_mid_point = joint_traj->getTrajectoryPoint(mid_index);
    Eigen::MatrixXd::RowXpr traj_goal_point = joint_traj->getTrajectoryPoint(goal_index);

    bool side_stepping = applySideStepping(initial_state, goal_state);
    if (side_stepping)
    {
        goal_state.setVariablePosition("base_revolute_joint_z", initial_state.getVariablePosition("base_revolute_joint_z"));
        goal_state.update(true);
    }

    //ROS_INFO("Use Side stepping : %s", side_stepping ? "true" : "false");

    eFOOT_INDEX initial_support_foot;
    eFOOT_INDEX initial_front_foot = getFrontFoot(initial_state, initial_support_foot);

    //ROS_INFO("initial_front_foot : %s", (initial_front_foot == 1) ? "left" : "right");
    //ROS_INFO("initial_support_foot : %s", (initial_support_foot == 1) ? "left" : "right");

    // root translation joints
    Eigen::Vector3d start_pos;
    Eigen::Vector3d goal_pos;
    for (int i = 0; i < 3; ++i)
    {
        start_pos(i) = initial_state.getVariablePosition(i);
        goal_pos(i) = goal_state.getVariablePosition(i);
    }
    Eigen::Vector3d mid_pos = 0.5 * (start_pos + goal_pos);
    Eigen::Vector3d move_pos = goal_pos - start_pos;
    move_pos(2) = 0.0;
    double move_dist = move_pos.norm();
    Eigen::Vector3d move_dir = (move_dist > ITOMP_EPS) ? Eigen::Vector3d(move_pos / move_dist) : Eigen::Vector3d::UnitY();

    double start_orientation = normalizeAngle(initial_state.getVariablePosition("base_revolute_joint_z"));
    double move_orientation = std::atan2(move_pos.y(), move_pos.x()) - M_PI_2;
    double move_orientation_diff = normalizeAngle(move_orientation - start_orientation);
    move_orientation = start_orientation + move_orientation_diff;
    double goal_orientation = goal_state.getVariablePosition("base_revolute_joint_z");
    if (side_stepping)
    {
        goal_orientation = start_orientation;
    }
    else
    {
        double goal_move_orientation_diff = normalizeAngle(goal_orientation - move_orientation);
        if (goal_move_orientation_diff > GOAL_MOVE_ORIENTATION_BOUND)
            goal_move_orientation_diff = GOAL_MOVE_ORIENTATION_BOUND;
        else if (goal_move_orientation_diff < -GOAL_MOVE_ORIENTATION_BOUND)
            goal_move_orientation_diff = -GOAL_MOVE_ORIENTATION_BOUND;
        goal_orientation = move_orientation + goal_move_orientation_diff;
    }
    bool mid_orientation_use_goal_orientation = true;

    //ROS_INFO("Start ori : %f Move ori : %f Goal ori : %f", start_orientation, move_orientation, goal_orientation);
    //ROS_INFO("Move ori diff: %f", move_orientation_diff);

    Eigen::Vector3d velocities[3];
    ros::NodeHandle node_handle("itomp_planner");
    node_handle.getParam("/itomp_planner/agent_vel_x_0", velocities[0](0));
    node_handle.getParam("/itomp_planner/agent_vel_y_0", velocities[0](1));
    node_handle.getParam("/itomp_planner/agent_vel_x_10", velocities[1](0));
    node_handle.getParam("/itomp_planner/agent_vel_y_10", velocities[1](1));
    node_handle.getParam("/itomp_planner/agent_vel_x_20", velocities[2](0));
    node_handle.getParam("/itomp_planner/agent_vel_y_20", velocities[2](1));
    velocities[0](2) = velocities[1](2) = velocities[2](2) = 0.0;

    Eigen::Affine3d mid_transform;

    if (!read_start_state_from_previous_step)
        velocities[0] = Eigen::Vector3d::Zero();

    if (side_stepping)
    {
        velocities[2] = Eigen::Vector3d::Zero();
        if (initial_support_foot == ANY_FOOT)
        {
            if (move_orientation_diff > 0)
                initial_support_foot = RIGHT_FOOT;
            else
                initial_support_foot = LEFT_FOOT;
        }
        else if (initial_support_foot == LEFT_FOOT)
        {
            if (move_orientation_diff > MIN_ANKLE_Z_ROTATION_ANGLE &&
                    move_orientation_diff < M_PI - MIN_ANKLE_Z_ROTATION_ANGLE)
                initial_support_foot = RIGHT_FOOT;
        }
        else if (initial_support_foot == RIGHT_FOOT)
        {
            if (move_orientation_diff < -MIN_ANKLE_Z_ROTATION_ANGLE &&
                    move_orientation_diff > -M_PI + MIN_ANKLE_Z_ROTATION_ANGLE)
                initial_support_foot = LEFT_FOOT;
        }
    }
    else
    {
        if (move_orientation_diff > MIN_ANKLE_Z_ROTATION_ANGLE)
        {
            if (initial_support_foot == LEFT_FOOT)
            {
                velocities[1] = Eigen::Vector3d::Zero();
                mid_orientation_use_goal_orientation = false;
                Eigen::Affine3d ee_pose;
                if (!ItompRobotModelIKHelper::getInstance()->getGroupEndeffectorPos("left_leg", initial_state, ee_pose))
                    return false;
                if (!ItompRobotModelIKHelper::getInstance()->getRootPose("left_leg", ee_pose, mid_transform))
                    return false;
                //ROS_INFO("Turn left at 2 step");
            }
            else if (initial_support_foot == ANY_FOOT)
                initial_support_foot = RIGHT_FOOT;
        }
        if (move_orientation_diff < -MIN_ANKLE_Z_ROTATION_ANGLE)
        {
            if (initial_support_foot == RIGHT_FOOT)
            {
                velocities[1] = Eigen::Vector3d::Zero();
                mid_orientation_use_goal_orientation = false;
                Eigen::Affine3d ee_pose;
                if (!ItompRobotModelIKHelper::getInstance()->getGroupEndeffectorPos("right_leg", initial_state, ee_pose))
                    return false;
                if (!ItompRobotModelIKHelper::getInstance()->getRootPose("right_leg", ee_pose, mid_transform))
                    return false;
                //ROS_INFO("Turn right at 2 step");
            }
            else if (initial_support_foot == ANY_FOOT)
                initial_support_foot = LEFT_FOOT;
        }
    }
    //ROS_INFO("initial_support_foot : %s", (initial_support_foot == 1) ? "left" : "right");

    ROS_INFO("Speeds : %f %f %f", velocities[0].norm(), velocities[1].norm(), velocities[2].norm());

    if (mid_orientation_use_goal_orientation)
        mid_pos = start_pos + std::min(0.5 * (velocities[0] + velocities[1]).norm(), move_dist * 0.5) * move_dir;
    else
    {
        mid_pos = mid_transform.translation();
    }
    goal_pos = mid_pos + std::min(0.5 * (velocities[1] + velocities[2]).norm(), move_dist * 0.5) * move_dir;

    double dist_start_mid = (mid_pos - start_pos).norm();
    double dist_mid_goal = (goal_pos - mid_pos).norm();

    //ROS_INFO("Start pos : %f %f %f", start_pos(0), start_pos(1), start_pos(2));
    //ROS_INFO("Move pos : %f %f %f", move_pos(0), move_pos(1), move_pos(2));
    //ROS_INFO("goal pos : %f %f %f", goal_pos(0), goal_pos(1), goal_pos(2));

    double foot_pos_1 = 0.5 * std::min(velocities[1].norm(), move_dist * 0.5);
    double foot_pos_2 = 0.5 * std::min(velocities[2].norm(), move_dist * 0.5);

    //ROS_INFO("Foot_pos : %f %f", foot_pos_1, foot_pos_2);

    Eigen::Vector3d pos_out, normal_out;
    if (!read_start_state_from_previous_step)
    {
        GroundManager::getInstance()->getNearestZPosition(start_pos, pos_out, normal_out);
        start_pos(2) = pos_out(2);
    }
    //GroundManager::getInstance()->getNearestZPosition(mid_pos, pos_out, normal_out);
    Eigen::Vector3d ori = Eigen::Vector3d(1, 0, 0);
    GroundManager::getInstance()->getNearestContactPosition(mid_pos, ori, pos_out, ori, normal_out);
    //mid_pos = pos_out;
    mid_pos(2) = pos_out(2);
    ///GroundManager::getInstance()->getNearestZPosition(goal_pos, pos_out, normal_out);
    GroundManager::getInstance()->getNearestContactPosition(goal_pos, ori, pos_out, ori, normal_out);
    //goal_pos = pos_out;
    goal_pos(2) = pos_out(2);

    robot_state::RobotState mid_state(initial_state);
    for (int k = 0; k < initial_state.getVariableCount(); ++k)
    {
        mid_state.getVariablePositions()[k] = traj_mid_point(k);
    }
    for (int i = 0; i < 3; ++i)
    {
        traj_start_point(i) = start_pos(i);
        traj_mid_point(i) = mid_pos(i);
        traj_goal_point(i) = goal_pos(i);

        initial_state.setVariablePosition(i, start_pos(i));
        mid_state.setVariablePosition(i, mid_pos(i));
        goal_state.setVariablePosition(i, goal_pos(i));
    }
    goal_state.setVariablePosition(3, 0.0);
    goal_state.setVariablePosition(4, 0.0);
    traj_goal_point(3) = traj_goal_point(4) = 0.0;
    traj_start_point(5) = start_orientation;
    traj_goal_point(5) = goal_orientation;
    initial_state.setVariablePosition("base_revolute_joint_z", start_orientation);
    goal_state.setVariablePosition("base_revolute_joint_z", goal_orientation);

    if (mid_orientation_use_goal_orientation)
    {
        for (int i = 3; i < 6; ++i)
        {
            double value = goal_state.getVariablePosition(i);
            mid_state.setVariablePosition(i, value);
            traj_mid_point(i) = value;
        }
    }
    else
    {
        Eigen::Vector3d euler_angles = mid_transform.linear().eulerAngles(0, 1, 2);
        for (int i = 3; i < 6; ++i)
        {
            double value = euler_angles(i - 3);
            mid_state.setVariablePosition(i, value);
            traj_mid_point(i) = value;
        }
    }

    initial_state.update(true);
    mid_state.update(true);
    goal_state.update(true);

    if (initial_support_foot == ANY_FOOT)
        initial_support_foot = initial_front_foot;

    ROS_INFO("initial_support_foot : %s", (initial_support_foot == 1) ? "left" : "right");
    PhaseManager::getInstance()->support_foot_ = initial_support_foot;
    eFOOT_INDEX initial_back_foot = (initial_support_foot == LEFT_FOOT) ? RIGHT_FOOT : LEFT_FOOT;
    std::map<eFOOT_INDEX, std::string> group_name_map;
    group_name_map[LEFT_FOOT] = "left_leg";
    group_name_map[RIGHT_FOOT] = "right_leg";

    std::map<eFOOT_INDEX, Eigen::Affine3d> foot_pose_0;
    std::map<eFOOT_INDEX, Eigen::Affine3d> foot_pose_1;

    if (!ItompRobotModelIKHelper::getInstance()->getGroupEndeffectorPos(group_name_map[initial_support_foot], initial_state, foot_pose_0[initial_support_foot]))
        return false;
    if (!ItompRobotModelIKHelper::getInstance()->getGroupEndeffectorPos(group_name_map[initial_back_foot], initial_state, foot_pose_0[initial_back_foot]))
        return false;

    if (!ItompRobotModelIKHelper::getInstance()->getGroupEndeffectorPos(group_name_map[initial_back_foot], mid_state, foot_pose_1[initial_back_foot]))
        return false;
    foot_pose_1[initial_back_foot].translation() += foot_pos_1 * move_dir;

    GroundManager::getInstance()->getNearestZPosition(foot_pose_1[initial_back_foot].translation(), pos_out, normal_out);
    foot_pose_1[initial_back_foot].translation()(2) = pos_out(2);
    /*
    Eigen::Vector3d orientation_out;
    GroundManager::getInstance()->getNearestContactPosition(foot_pose_1[initial_back_foot].translation(), exponential_map::RotationToExponentialMap(foot_pose_1[initial_back_foot].linear()),
                                                            pos_out, orientation_out, normal_out);
    foot_pose_1[initial_back_foot].translation() = pos_out;
    foot_pose_1[initial_back_foot].linear() = exponential_map::ExponentialMapToRotation(orientation_out);
    */

    if (!ItompRobotModelIKHelper::getInstance()->getGroupEndeffectorPos(group_name_map[initial_support_foot], goal_state, foot_pose_1[initial_support_foot]))
        return false;
    foot_pose_1[initial_support_foot].translation() += foot_pos_2 * move_dir;

    GroundManager::getInstance()->getNearestZPosition(foot_pose_1[initial_support_foot].translation(), pos_out, normal_out);
    foot_pose_1[initial_support_foot].translation()(2) = pos_out(2);
    /*
    GroundManager::getInstance()->getNearestContactPosition(foot_pose_1[initial_support_foot].translation(), exponential_map::RotationToExponentialMap(foot_pose_1[initial_support_foot].linear()),
                                                            pos_out, orientation_out, normal_out);
    foot_pose_1[initial_support_foot].translation() = pos_out;
    foot_pose_1[initial_support_foot].linear() = exponential_map::ExponentialMapToRotation(orientation_out);
    */

    std::vector<unsigned int> root_transform_indices;
    for (unsigned int i = 0; i < 6; ++i)
        root_transform_indices.push_back(i);
    itomp_trajectory_->interpolate(0, 20, ItompTrajectory::SUB_COMPONENT_TYPE_JOINT, &root_transform_indices);
    itomp_trajectory_->interpolate(20, 40, ItompTrajectory::SUB_COMPONENT_TYPE_JOINT, &root_transform_indices);

    if (side_stepping == false)
    {
        // add mocap data
        Eigen::MatrixXd mocap_trajectory;
        mocap_trajectory.setZero(itomp_trajectory_->getNumPoints(), joint_traj->getNumElements());

        readMocapData((initial_support_foot == LEFT_FOOT) ? "../motions/walking_itomp_left.txt" : "../motions/walking_itomp_right.txt", mocap_trajectory);
        {

            Eigen::Vector3d mid_translation = Eigen::Vector3d::Zero();
            for (int i = 5; i <= 40; i += 5)
            {
                Eigen::Vector3d root_translation;
                root_translation(0) = std::cos(move_orientation) * mocap_trajectory(i, 0) - std::sin(move_orientation) * mocap_trajectory(i, 1);
                root_translation(1) = std::sin(move_orientation) * mocap_trajectory(i, 0) + std::cos(move_orientation) * mocap_trajectory(i, 1);
                root_translation(2) = mocap_trajectory(i, 2);

                Eigen::MatrixXd::RowXpr traj_point = joint_traj->getTrajectoryPoint(i);

                if (i <= 20)
                {
                    traj_point(0) = traj_start_point(0) + root_translation(0) * dist_start_mid * 2.0;
                    traj_point(1) = traj_start_point(1) + root_translation(1) * dist_start_mid * 2.0;
                    traj_point(2) = traj_point(2) + root_translation(2) * dist_start_mid * 2.0;
                }
                else
                {
                    traj_point(0) = traj_mid_point(0) + (root_translation(0) - mid_translation(0)) * dist_mid_goal * 2.0;
                    traj_point(1) = traj_mid_point(1) + (root_translation(1) - mid_translation(1)) * dist_mid_goal * 2.0;
                    traj_point(2) = traj_point(2) + (root_translation(2) - mid_translation(2)) * dist_mid_goal * 2.0;
                }

                if (i == 20)
                {
                    mid_translation(0) = root_translation(0);
                    mid_translation(1) = root_translation(1);
                    mid_translation(2) = root_translation(2);
                }

                int num_joints = joint_traj->getNumElements();
                for (int j = 3; j < 6; ++j)
                {
                    traj_point(j) += mocap_trajectory(i, j);
                }
                for (int j = 6; j < num_joints; ++j)
                {
                    traj_point(j) = mocap_trajectory(i, j);
                    if (j == 8)
                        traj_point(j) -= 0.25;
                }

                itomp_trajectory_->interpolate(i - 5, i, ItompTrajectory::SUB_COMPONENT_TYPE_JOINT);
            }
        }
    }

    const std::vector<const robot_model::LinkModel*>& robot_link_models = itomp_robot_model_->getMoveitRobotModel()->getLinkModels();
    {
        Eigen::Affine3d root_pose;
        root_pose = initial_state.getGlobalLinkTransform(robot_link_models[6]);

        if (PlanningParameters::getInstance()->getPrintPlanningInfo())
            ROS_INFO("Point %d : (%f %f %f) (%f %f %f) (%f %f %f)", 0,
                     foot_pose_0[LEFT_FOOT].translation()(0), foot_pose_0[LEFT_FOOT].translation()(1), foot_pose_0[LEFT_FOOT].translation()(2),
                     foot_pose_0[RIGHT_FOOT].translation()(0), foot_pose_0[RIGHT_FOOT].translation()(1), foot_pose_0[RIGHT_FOOT].translation()(2),
                     root_pose.translation()(0), root_pose.translation()(1), root_pose.translation()(2));
    }

    for (int i = 5; i <= 40; i += 5)
    {
        robot_state::RobotState robot_state(initial_state);

        Eigen::Affine3d root_pose;
        Eigen::Affine3d left_foot_pose;
        Eigen::Affine3d right_foot_pose;

        for (unsigned int j = 0; j < robot_state.getVariableCount(); ++j)
            robot_state.getVariablePositions()[j] = joint_traj->getTrajectoryPoint(i)(j);
        robot_state.update(true);
        root_pose = robot_state.getGlobalLinkTransform(robot_link_models[6]);

        int left_foot_change, right_foot_change;
        if (initial_support_foot == LEFT_FOOT)
        {
            left_foot_change = 30;
            right_foot_change = 10;
        }
        else
        {
            left_foot_change = 10;
            right_foot_change = 30;
        }

        if (i < left_foot_change)
        {
            left_foot_pose = foot_pose_0[LEFT_FOOT];
        }
        else if (i == left_foot_change)
        {

            left_foot_pose.translation() = 0.5 * (foot_pose_0[LEFT_FOOT].translation() + foot_pose_1[LEFT_FOOT].translation());
            left_foot_pose.linear() = foot_pose_1[LEFT_FOOT].linear();



            /*
            Eigen::Vector3d trans = 0.5 * (foot_pose_0[LEFT_FOOT].translation() + foot_pose_1[LEFT_FOOT].translation());
            Eigen::Vector3d prev_rot = exponential_map::RotationToExponentialMap(foot_pose_0[LEFT_FOOT].linear());
            Eigen::Vector3d rot = exponential_map::RotationToExponentialMap(foot_pose_1[LEFT_FOOT].linear(), &prev_rot);
            Eigen::Vector3d position_out, orientation_out, normal;
            GroundManager::getInstance()->getNearestZPosition(trans, rot, position_out, orientation_out, normal);
            left_foot_pose.translation() = position_out;
            left_foot_pose.linear() = exponential_map::ExponentialMapToRotation(orientation_out);
            */


        }
        else // i > left_foot_chage
        {
            left_foot_pose = foot_pose_1[LEFT_FOOT];
        }

        if (i < right_foot_change)
        {
            right_foot_pose = foot_pose_0[RIGHT_FOOT];
        }
        else if (i == right_foot_change)
        {

            right_foot_pose.translation() = 0.5 * (foot_pose_0[RIGHT_FOOT].translation() + foot_pose_1[RIGHT_FOOT].translation());
            right_foot_pose.linear() = foot_pose_1[RIGHT_FOOT].linear();



            /*
            Eigen::Vector3d trans = 0.5 * (foot_pose_0[RIGHT_FOOT].translation() + foot_pose_1[RIGHT_FOOT].translation());
            Eigen::Vector3d prev_rot = exponential_map::RotationToExponentialMap(foot_pose_0[RIGHT_FOOT].linear());
            Eigen::Vector3d rot = exponential_map::RotationToExponentialMap(foot_pose_1[RIGHT_FOOT].linear(), &prev_rot);
            Eigen::Vector3d position_out, orientation_out, normal;
            GroundManager::getInstance()->getNearestZPosition(trans, rot, position_out, orientation_out, normal);
            right_foot_pose.translation() = position_out;
            right_foot_pose.linear() = exponential_map::ExponentialMapToRotation(orientation_out);
            */


        }
        else // i > right_foot_change
        {
            right_foot_pose = foot_pose_1[RIGHT_FOOT];
        }

        if (!ItompRobotModelIKHelper::getInstance()->computeStandIKState(robot_state, root_pose, left_foot_pose, right_foot_pose))
            return false;

        if (PlanningParameters::getInstance()->getPrintPlanningInfo())
            ROS_INFO("Point %d : (%f %f %f) (%f %f %f) (%f %f %f)", i,
                     left_foot_pose.translation()(0), left_foot_pose.translation()(1), left_foot_pose.translation()(2),
                     right_foot_pose.translation()(0), right_foot_pose.translation()(1), right_foot_pose.translation()(2),
                     root_pose.translation()(0), root_pose.translation()(1), root_pose.translation()(2));

        Eigen::MatrixXd::RowXpr traj_point = joint_traj->getTrajectoryPoint(i);
        for (int k = 0; k < robot_state.getVariableCount(); ++k)
            traj_point(k) = robot_state.getVariablePosition(k);

        double prev_angle = joint_traj->getTrajectoryPoint(i - 5)(5);
        double current_angle = joint_traj->getTrajectoryPoint(i)(5);
        if (current_angle - prev_angle > M_PI)
            current_angle -= 2.0 * M_PI;
        else if (current_angle - prev_angle <= -M_PI)
            current_angle += 2.0 * M_PI;
        joint_traj->getTrajectoryPoint(i)(5) = current_angle;

        itomp_trajectory_->interpolate(i - 5, i, ItompTrajectory::SUB_COMPONENT_TYPE_JOINT);
    }

    PhaseManager::getInstance()->initial_goal_pos(0) = goal_state.getVariablePosition(0);
    PhaseManager::getInstance()->initial_goal_pos(1) = goal_state.getVariablePosition(1);
    PhaseManager::getInstance()->initial_goal_pos(2) = goal_state.getVariablePosition(5);

    return true;
}

bool ItompPlannerNode::applySideStepping(const robot_state::RobotState& initial_state, robot_state::RobotState& goal_state)
{
    const double FORWARD_WALKING_ANGLE = M_PI / 3.0;

    Eigen::Vector3d pref_velocity;

    ros::NodeHandle node_handle("itomp_planner");
    node_handle.getParam("/itomp_planner/agent_pref_vel_x_20", pref_velocity(0));
    node_handle.getParam("/itomp_planner/agent_pref_vel_y_20", pref_velocity(1));
    double pref_vel_orientation = 0.0;
    if (pref_velocity.norm() < ITOMP_EPS)
        return true;

    pref_vel_orientation = std::atan2(pref_velocity.y(), pref_velocity.x()) - M_PI_2;

    const Eigen::Vector3d start_pos = initial_state.getGlobalLinkTransform("pelvis_link").translation();
    const Eigen::Vector3d goal_pos = goal_state.getGlobalLinkTransform("pelvis_link").translation();
    Eigen::Vector3d move_pos = goal_pos - start_pos;
    move_pos.z() = 0.0;
    double move_orientation = 0.0;
    if (move_pos.norm() < ITOMP_EPS)
        return true;

    move_orientation = std::atan2(move_pos.y(), move_pos.x()) - M_PI_2;

    double orientation_diff = normalizeAngle(move_orientation - pref_vel_orientation);

    if (std::abs(orientation_diff) <= FORWARD_WALKING_ANGLE)
    {
        // forward walking
        return false;
    }

    return true;
}

eFOOT_INDEX ItompPlannerNode::getFrontFoot(const robot_state::RobotState& robot_state, eFOOT_INDEX& support_foot)
{
    eFOOT_INDEX front_foot = ANY_FOOT; // any

    double start_orientation = robot_state.getVariablePosition("base_revolute_joint_z");

    Eigen::Vector3d start_pos = robot_state.getGlobalLinkTransform("pelvis_link").translation();
    Eigen::Vector3d start_dir = Eigen::AngleAxisd(start_orientation, Eigen::Vector3d::UnitZ()) * Eigen::Vector3d::UnitY();
    Eigen::Vector3d start_left_foot = robot_state.getGlobalLinkTransform("left_foot_endeffector_link").translation();
    Eigen::Vector3d start_right_foot = robot_state.getGlobalLinkTransform("right_foot_endeffector_link").translation();

    start_pos(2) = start_left_foot(2) = start_right_foot(2) = 0.0;

    double left_dot = start_dir.dot(start_left_foot - start_pos);
    double right_dot = start_dir.dot(start_right_foot - start_pos);

    if (left_dot > right_dot)
        front_foot = support_foot = LEFT_FOOT; // left
    else
        front_foot = support_foot = RIGHT_FOOT; // right

    if (std::abs(left_dot - right_dot) < 0.1)
        support_foot = ANY_FOOT;

    return front_foot;
}


bool ItompPlannerNode::readMocapData(const std::string& file_name, Eigen::MatrixXd& mocap_trajectory)
{
    std::ifstream trajectory_file;
    trajectory_file.open(file_name.c_str());
    std::string temp;
    if (trajectory_file.is_open())
    {
        std::getline(trajectory_file, temp);
        std::getline(trajectory_file, temp);

        std::map<int,int> index_map;
        index_map[	0	]=	0	;
        index_map[	1	]=	1	;
        index_map[	2	]=	2	;
        index_map[	3	]=	3	;
        index_map[	4	]=	4	;
        index_map[	5	]=	5	;
        index_map[	6	]=	8	;
        index_map[	7	]=	7	;
        index_map[	8	]=	6	;
        index_map[	9	]=	-1	;
        index_map[	10	]=	-1	;
        index_map[	11	]=	12	;
        index_map[	12	]=	13	;
        index_map[	13	]=	14	;
        index_map[	14	]=	15	;
        index_map[	15	]=	-1	;
        index_map[	16	]=	16	;
        index_map[	17	]=	-1	;
        index_map[	18	]=	19	;
        index_map[	19	]=	18	;
        index_map[	20	]=	17	;
        index_map[	21	]=	-1	;
        index_map[	22	]=	-1	;
        index_map[	23	]=	25	;
        index_map[	24	]=	26	;
        index_map[	25	]=	27	;
        index_map[	26	]=	28	;
        index_map[	27	]=	-1	;
        index_map[	28	]=	29	;
        index_map[	29	]=	-1	;
        index_map[	30	]=	19	;
        index_map[	31	]=	18	;
        index_map[	32	]=	17	;
        index_map[	33	]=	11	;
        index_map[	34	]=	10	;
        index_map[	35	]=	9	;
        index_map[	36	]=	40	;
        index_map[	37	]=	39	;
        index_map[	38	]=	38	;
        index_map[	39	]=	41	;
        index_map[	40	]=	-1	;
        index_map[	41	]=	-1	;
        index_map[	42	]=	44	;
        index_map[	43	]=	43	;
        index_map[	44	]=	42	;
        index_map[	45	]=	52	;
        index_map[	46	]=	51	;
        index_map[	47	]=	50	;
        index_map[	48	]=	53	;
        index_map[	49	]=	-1	;
        index_map[	50	]=	-1	;
        index_map[	51	]=	56	;
        index_map[	52	]=	55	;
        index_map[	53	]=	54	;


        double v;

        ElementTrajectoryPtr t = itomp_trajectory_->getElementTrajectory(ItompTrajectory::COMPONENT_TYPE_POSITION,
                                 ItompTrajectory::SUB_COMPONENT_TYPE_JOINT);

        std::vector<double> old_value(54);
        for (int c = 0; c < 54; ++c)
            old_value[c] = t->at(0, c);
        old_value[14] = 0.0;
        old_value[27] = 0.0;

        Eigen::MatrixXd mat_old(32, 54);

        // 0
        // 9  -> 0
        // 16
        // 25 -> 16

        Eigen::MatrixXd mat(32, 54);


        for (int r = 0; r < 32; ++r)
        {
            for (int c = 0; c < 54; ++c)
            {
                trajectory_file >> v;

                if (c < 3)
                    v *= 0.01;
                else
                    v *= M_PI / 180.0;

                if (c == 2)
                    v /= 1.64215;

                mat_old(r, c) = v;
                mat((r-9+32)%32, c) = v;
            }
        }
        double s = mat(0, 2);
        for (int r = 0; r < 32; ++r)
        {
            mat(r, 2) -= s;
            if (mat(r, 2) < 0.0)
                   mat(r, 2) += 1.0;
        }

        trajectory_file.close();

        for (int rr = 0; rr <= 40; ++rr)
        {
            int r = rr * 31 / 40;

            //t->at(rr, 0) = old_value[0] + -mat(r, 2);
            //t->at(rr, 1) = old_value[1] + -mat(r, 0);
            //t->at(rr, 2) = old_value[2] + mat(r, 1) - 0.9619;

            mocap_trajectory(rr, 0) = -mat(r, 0);
            mocap_trajectory(rr, 1) = mat(r, 2);
            mocap_trajectory(rr, 2) = mat(r, 1) - 0.9619;

            Eigen::Vector3d dir[3];
            dir[0] = -Eigen::Vector3d::UnitX();
            dir[1] = Eigen::Vector3d::UnitZ();
            dir[2] = Eigen::Vector3d::UnitY();

            for (int c = 3; c < 54; c += 3)
            {
                int ori_set[3] = {0, 1, 2};
                if ((9 <= c && c < 15) || (21 <= c && c < 27))
                {
                    ori_set[0] = 2;
                    ori_set[1] = 0;
                    ori_set[2] = 1;
                }
                else if ((18 <= c && c < 21) || (30 <= c && c < 33))
                {
                    ori_set[0] = 2;
                    ori_set[1] = 1;
                    ori_set[2] = 0;
                }

                Eigen::Matrix3d m;
                m = Eigen::AngleAxisd(mat(r, c), dir[ori_set[0]]) *
                    Eigen::AngleAxisd(mat(r, c + 1), dir[ori_set[1]]) *
                    Eigen::AngleAxisd(mat(r, c + 2), dir[ori_set[2]]);

                Eigen::Vector3d euler_angles = m.eulerAngles(0, 1, 2);

                for (int j = 0; j < 3; ++j)
                {
                    int k = index_map[c + j];
                    if (k != -1)
                        //t->at(rr, k) = old_value[k] + euler_angles(j);
                         mocap_trajectory(rr, k) = euler_angles(j);
                }
            }
        }
    }
    else
    {
        ROS_ERROR("Failed to open %s", file_name.c_str());
        return false;
    }
    return true;
}

} // namespace
