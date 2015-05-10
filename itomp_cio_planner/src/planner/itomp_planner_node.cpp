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
        trajectory_->setStartState(req.start_state.joint_state, itomp_robot_model_, true);
        itomp_trajectory_->setStartState(req.start_state.joint_state, itomp_robot_model_);

        // read start state
        bool read_start_state = readWaypoint(initial_robot_state);

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
            for (unsigned int j = 0; j < goal_joint_state.name.size(); ++j)
            {
                if (goal_joint_state.name[j] != "")
                    goal_state.setVariablePosition(goal_joint_state.name[j], goal_joint_state.position[j]);
            }
            goal_state.update(true);

            if (!adjustStartGoalPositions(*initial_robot_state, goal_state, !read_start_state))
                res.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;

            optimizer_ = boost::make_shared<ItompOptimizer>(0, trajectory_, itomp_trajectory_,
						 itomp_robot_model_, planning_scene, planning_group, planning_start_time,
                         trajectory_start_time, req.trajectory_constraints.constraints);

			optimizer_->optimize();

            const PlanningInfo& planning_info = optimizer_->getPlanningInfo();

            planning_info_manager_.write(c, i, planning_info);

            ROS_INFO("Optimization of group %s took %f sec", planning_group_names[i].c_str(), (ros::WallTime::now() - create_time).toSec());

            if (planning_info.cost > PlanningParameters::getInstance()->getFailureCost())
            {
                res.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
                ROS_INFO("Planning failure - cost : %f", planning_info.cost);
                return false;
            }
        }
	}
    if (PlanningParameters::getInstance()->getPrintPlanningInfo())
        planning_info_manager_.printSummary();

    if (itomp_trajectory_->avoidNeighbors(req.trajectory_constraints.constraints) == false)
    {
        ROS_INFO("Planning failure - collision with neighbor");
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
        ROS_INFO("Goal constraint has %d/%d joints", goal_joint_state.name.size(), req.start_state.joint_state.name.size());

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

bool ItompPlannerNode::adjustStartGoalPositions(robot_state::RobotState& initial_state, robot_state::RobotState& goal_state, bool initial_state_fixed)
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

    eFOOT_INDEX initial_support_foot;
    eFOOT_INDEX initial_front_foot = getFrontFoot(initial_state, initial_support_foot);

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

    Eigen::Vector3d velocities[3];
    ros::NodeHandle node_handle("itomp_planner");
    node_handle.getParam("/itomp_planner/agent_vel_x_0", velocities[0](0));
    node_handle.getParam("/itomp_planner/agent_vel_y_0", velocities[0](1));
    node_handle.getParam("/itomp_planner/agent_vel_x_10", velocities[1](0));
    node_handle.getParam("/itomp_planner/agent_vel_y_10", velocities[1](1));
    node_handle.getParam("/itomp_planner/agent_vel_x_20", velocities[2](0));
    node_handle.getParam("/itomp_planner/agent_vel_y_20", velocities[2](1));
    velocities[0](2) = velocities[1](2) = velocities[2](2) = 0.0;

    if (initial_state_fixed)
        velocities[0] = Eigen::Vector3d::Zero();

    if (side_stepping)
        velocities[2] = Eigen::Vector3d::Zero();
    else
    {
        if (initial_support_foot == LEFT_FOOT && move_orientation_diff > MIN_ANKLE_Z_ROTATION_ANGLE)
        {
            velocities[1] = Eigen::Vector3d::Zero();
            mid_orientation_use_goal_orientation = false;
        }
        if (initial_support_foot == RIGHT_FOOT && move_orientation_diff < -MIN_ANKLE_Z_ROTATION_ANGLE)
        {
            velocities[1] = Eigen::Vector3d::Zero();
            mid_orientation_use_goal_orientation = false;
        }
    }

    mid_pos = start_pos + std::min(0.5 * (velocities[0] + velocities[1]).norm(), move_dist * 0.5) * move_dir;
    goal_pos = mid_pos + std::min(0.5 * (velocities[1] + velocities[2]).norm(), move_dist * 0.5) * move_dir;

    double foot_pos_1 = 0.5 * std::min(velocities[1].norm(), move_dist * 0.5);
    double foot_pos_2 = 0.5 * std::min(velocities[2].norm(), move_dist * 0.5);

    Eigen::Vector3d pos_out, normal_out;
    if (!initial_state_fixed)
    {
        GroundManager::getInstance()->getNearestZPosition(start_pos, pos_out, normal_out);
        start_pos(2) = pos_out(2);
    }
    GroundManager::getInstance()->getNearestZPosition(mid_pos, pos_out, normal_out);
    mid_pos(2) = pos_out(2);
    GroundManager::getInstance()->getNearestZPosition(goal_pos, pos_out, normal_out);
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
    traj_start_point(5) = start_orientation;
    traj_goal_point(5) = goal_orientation;
    initial_state.setVariablePosition("base_revolute_joint_z", start_orientation);
    goal_state.setVariablePosition("base_revolute_joint_z", goal_orientation);
    for (int i = 3; i < 6; ++i)
    {
        double value = mid_orientation_use_goal_orientation ? goal_state.getVariablePosition(i) : initial_state.getVariablePosition(i);
        mid_state.setVariablePosition(i, value);
        traj_mid_point(i) = value;
    }

    initial_state.update(true);
    mid_state.update(true);
    goal_state.update(true);

    if (initial_support_foot == ANY_FOOT)
        initial_support_foot = initial_front_foot;
    PhaseManager::getInstance()->support_foot_ = initial_support_foot;
    eFOOT_INDEX initial_back_foot = (initial_support_foot == LEFT_FOOT) ? RIGHT_FOOT : LEFT_FOOT;
    std::map<eFOOT_INDEX, std::string> group_name_map;
    group_name_map[LEFT_FOOT] = "left_leg";
    group_name_map[RIGHT_FOOT] = "right_leg";

    std::map<eFOOT_INDEX, Eigen::Affine3d> foot_pose_0;
    std::map<eFOOT_INDEX, Eigen::Affine3d> foot_pose_1;

    if (!itomp_robot_model_->getGroupEndeffectorPos(group_name_map[initial_support_foot], initial_state, foot_pose_0[initial_support_foot]))
        return false;
    if (!itomp_robot_model_->getGroupEndeffectorPos(group_name_map[initial_back_foot], initial_state, foot_pose_0[initial_back_foot]))
        return false;

    if (!itomp_robot_model_->getGroupEndeffectorPos(group_name_map[initial_back_foot], mid_state, foot_pose_1[initial_back_foot]))
        return false;
    foot_pose_1[initial_back_foot].translation() += foot_pos_1 * move_dir;
    GroundManager::getInstance()->getNearestZPosition(foot_pose_1[initial_back_foot].translation(), pos_out, normal_out);
    foot_pose_1[initial_back_foot].translation()(2) = pos_out(2);

    if (!itomp_robot_model_->getGroupEndeffectorPos(group_name_map[initial_support_foot], goal_state, foot_pose_1[initial_support_foot]))
        return false;
    foot_pose_1[initial_support_foot].translation() += foot_pos_2 * move_dir;
    GroundManager::getInstance()->getNearestZPosition(foot_pose_1[initial_support_foot].translation(), pos_out, normal_out);
    foot_pose_1[initial_support_foot].translation()(2) = pos_out(2);

    std::vector<unsigned int> root_transform_indices;
    for (unsigned int i = 0; i < 6; ++i)
        root_transform_indices.push_back(i);
    itomp_trajectory_->interpolate(0, 20, ItompTrajectory::SUB_COMPONENT_TYPE_JOINT, &root_transform_indices);
    itomp_trajectory_->interpolate(20, 40, ItompTrajectory::SUB_COMPONENT_TYPE_JOINT, &root_transform_indices);

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
        }
        else // i > right_foot_change
        {
            right_foot_pose = foot_pose_1[RIGHT_FOOT];
        }

        if (!itomp_robot_model_->computeStandIKState(robot_state, root_pose, left_foot_pose, right_foot_pose))
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
    node_handle.getParam("/itomp_planner/agent_pref_vel_x_0", pref_velocity(0));
    node_handle.getParam("/itomp_planner/agent_pref_vel_y_0", pref_velocity(1));
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

} // namespace
