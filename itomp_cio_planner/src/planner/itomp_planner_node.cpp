#include <itomp_cio_planner/planner/itomp_planner_node.h>
#include <itomp_cio_planner/model/itomp_planning_group.h>
#include <itomp_cio_planner/util/planning_parameters.h>
#include <itomp_cio_planner/visualization/visualization_manager.h>
#include <kdl/jntarray.hpp>
#include <angles/angles.h>
#include <visualization_msgs/MarkerArray.h>
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

using namespace std;

namespace itomp_cio_planner
{

ItompPlannerNode::ItompPlannerNode(const robot_model::RobotModelConstPtr& model) :
		last_planning_time_(0), last_min_cost_trajectory_(0)
{
	complete_initial_robot_state_.reset(new robot_state::RobotState(model));
}

bool ItompPlannerNode::init()
{
	//Eigen::initParallel();

	PlanningParameters::getInstance()->initFromNodeHandle();

	robot_model_loader::RobotModelLoader robot_model_loader(
			"robot_description");
	robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();

	// build the robot model
	string reference_frame = kinematic_model->getModelFrame();
	if (!robot_model_.init(kinematic_model,
			robot_model_loader.getRobotDescription()))
		return false;

	VisualizationManager::getInstance()->initialize(robot_model_);

	double trajectory_duration =
			PlanningParameters::getInstance()->getTrajectoryDuration();
	double trajectory_discretization =
			PlanningParameters::getInstance()->getTrajectoryDiscretization();
	int num_contacts = PlanningParameters::getInstance()->getNumContacts();
	int num_trajectories =
			PlanningParameters::getInstance()->getNumTrajectories();
	trajectory_.reset(
			new ItompCIOTrajectory(&robot_model_, trajectory_duration,
					trajectory_discretization, num_contacts,
					PlanningParameters::getInstance()->getPhaseDuration()));

	ROS_INFO("Initialized ITOMP planning service...");

	return true;
}

ItompPlannerNode::~ItompPlannerNode()
{
}

int ItompPlannerNode::run()
{
	return 0;
}

bool ItompPlannerNode::planKinematicPath(
		const planning_scene::PlanningSceneConstPtr& planning_scene,
		const planning_interface::MotionPlanRequest &req,
		planning_interface::MotionPlanResponse &res)
{
	// reload parameters
	PlanningParameters::getInstance()->initFromNodeHandle();

	ros::WallTime start_time = ros::WallTime::now();

	//ros::spinOnce();

	if (!preprocessRequest(req))
		return false;

	// generate planning group list
	vector<string> planningGroups;
	getPlanningGroups(planningGroups, req.group_name);

	int num_trials = PlanningParameters::getInstance()->getNumTrials();
	resetPlanningInfo(num_trials, planningGroups.size());
	for (int c = 0; c < num_trials; ++c)
	{
		printf("Trial [%d]\n", c);

		// initialize trajectory with start state
		initTrajectory(req.start_state.joint_state);
		complete_initial_robot_state_ = planning_scene->getCurrentStateUpdated(
				req.start_state);

		sensor_msgs::JointState jointGoalState;
		getGoalState(req, jointGoalState);

		planning_start_time_ = ros::Time::now().toSec();

		// for each planning group
		for (unsigned int i = 0; i != planningGroups.size(); ++i)
		{
			const string& groupName = planningGroups[i];

			VisualizationManager::getInstance()->setPlanningGroup(robot_model_,
					groupName);

			// optimize
			trajectoryOptimization(groupName, jointGoalState,
					req.path_constraints, req.trajectory_constraints,
					planning_scene);

			writePlanningInfo(c, i);
		}
	}
	printPlanningInfoSummary();

	// return trajectory
	fillInResult(planningGroups, res);

	return true;
}

bool ItompPlannerNode::preprocessRequest(
		const planning_interface::MotionPlanRequest &req)
{
	ROS_INFO("Received planning request...");

	ROS_INFO(
			"Trajectory Duration : %f", PlanningParameters::getInstance()->getTrajectoryDuration());

	trajectory_start_time_ = req.start_state.joint_state.header.stamp.toSec();

	// check goal constraint
	ROS_INFO("goal");
	sensor_msgs::JointState goal_joint_state = jointConstraintsToJointState(
			req.goal_constraints);
	if (goal_joint_state.name.size() != goal_joint_state.position.size())
	{
		ROS_ERROR("Invalid goal");
		return false;
	}
	for (unsigned int i = 0; i < goal_joint_state.name.size(); i++)
	{
		ROS_INFO(
				"%s %f", goal_joint_state.name[i].c_str(), goal_joint_state.position[i]);
	}

	ROS_INFO_STREAM(
			"Joint state has " << req.start_state.joint_state.name.size() << " joints");

	return true;
}

void ItompPlannerNode::initTrajectory(
		const sensor_msgs::JointState &joint_state)
{
	int num_trajectories =
			PlanningParameters::getInstance()->getNumTrajectories();
	double trajectory_duration =
			PlanningParameters::getInstance()->getTrajectoryDuration();
	if (trajectory_->getDuration() != trajectory_duration)
	{
		double trajectory_discretization =
				PlanningParameters::getInstance()->getTrajectoryDiscretization();

		trajectory_.reset(
				new ItompCIOTrajectory(&robot_model_, trajectory_duration,
						trajectory_discretization,
						PlanningParameters::getInstance()->getNumContacts(),
						PlanningParameters::getInstance()->getPhaseDuration()));
	}

	// set the trajectory to initial state value
	start_point_velocities_ = Eigen::MatrixXd(1,
			robot_model_.getNumKDLJoints());
	start_point_accelerations_ = Eigen::MatrixXd(1,
			robot_model_.getNumKDLJoints());

	robot_model_.jointStateToArray(joint_state,
			trajectory_->getTrajectoryPoint(0), start_point_velocities_.row(0),
			start_point_accelerations_.row(0));

	for (int i = 1; i < trajectory_->getNumPoints(); ++i)
	{
		trajectory_->getTrajectoryPoint(i) = trajectory_->getTrajectoryPoint(0);
	}

	// set the contact trajectory initial values
	Eigen::MatrixXd::RowXpr initContacts =
			trajectory_->getContactTrajectoryPoint(0);
	Eigen::MatrixXd::RowXpr goalContacts =
			trajectory_->getContactTrajectoryPoint(
					trajectory_->getNumContactPhases());
	for (int i = 0; i < trajectory_->getNumContacts(); ++i)
	{
		initContacts(i) =
				PlanningParameters::getInstance()->getContactVariableInitialValues()[i];
		goalContacts(i) =
				PlanningParameters::getInstance()->getContactVariableGoalValues()[i];
	}
	for (int i = 1; i < trajectory_->getNumContactPhases(); ++i)
	{
		trajectory_->getContactTrajectoryPoint(i) = initContacts;
	}
}

void ItompPlannerNode::getGoalState(
		const planning_interface::MotionPlanRequest &req,
		sensor_msgs::JointState& goalState)
{
	sensor_msgs::JointState goal_joint_state = jointConstraintsToJointState(
			req.goal_constraints);
	goalState.name.resize(req.start_state.joint_state.name.size());
	goalState.position.resize(req.start_state.joint_state.position.size());
	for (unsigned int i = 0; i < goal_joint_state.name.size(); ++i)
	{
		string name = goal_joint_state.name[i];
		int kdl_number = robot_model_.urdfNameToKdlNumber(name);
		if (kdl_number >= 0)
		{
			goalState.name[kdl_number] = name;
			goalState.position[kdl_number] = goal_joint_state.position[i];
		}
	}
}

void ItompPlannerNode::getPlanningGroups(
		std::vector<std::string>& plannningGroups, const string& groupName)
{
	plannningGroups.clear();
	if (groupName == "decomposed_body")
	{
		plannningGroups.push_back("lower_body");
		plannningGroups.push_back("torso");
		//plannningGroups.push_back("head");
		plannningGroups.push_back("left_arm");
		plannningGroups.push_back("right_arm");
	}
	else
	{
		plannningGroups.push_back(groupName);
	}
}

void optimization_thread_function(ItompOptimizerPtr& optimizer)
{
	omp_set_num_threads(getNumParallelThreads());
	optimizer->optimize();
}

void ItompPlannerNode::trajectoryOptimization(const string& groupName,
		const sensor_msgs::JointState& jointGoalState,
		const moveit_msgs::Constraints& path_constraints,
		const moveit_msgs::TrajectoryConstraints& trajectory_constraints,
		const planning_scene::PlanningSceneConstPtr& planning_scene)
{
	ros::WallTime create_time = ros::WallTime::now();

	fillGroupJointTrajectory(groupName, jointGoalState, path_constraints,
			trajectory_constraints);

	int num_trajectories =
			PlanningParameters::getInstance()->getNumTrajectories();
	const ItompPlanningGroup* group = robot_model_.getPlanningGroup(groupName);

	best_cost_manager_.reset();

	optimizers_.resize(num_trajectories);
	for (int i = 0; i < num_trajectories; ++i)
		optimizers_[i].reset(
				new ItompOptimizer(i, trajectories_[i].get(), &robot_model_,
						group, planning_start_time_, trajectory_start_time_,
						path_constraints, &best_cost_manager_, planning_scene));

	std::vector<boost::shared_ptr<boost::thread> > optimization_threads(
			num_trajectories);
	for (int i = 0; i < num_trajectories; ++i)
		optimization_threads[i].reset(
				new boost::thread(optimization_thread_function,
						optimizers_[i]));

	for (int i = 0; i < num_trajectories; ++i)
		optimization_threads[i]->join();

	last_planning_time_ = (ros::WallTime::now() - create_time).toSec();
	ROS_INFO(
			"Optimization of group %s took %f sec", groupName.c_str(), last_planning_time_);
}

void ItompPlannerNode::fillInResult(
		const std::vector<std::string>& planningGroups,
		planning_interface::MotionPlanResponse &res)
{
	int best_trajectory_index = best_cost_manager_.getBestCostTrajectoryIndex();

	const std::map<std::string, double>& joint_velocity_limits =
			PlanningParameters::getInstance()->getJointVelocityLimits();

	int num_all_joints = complete_initial_robot_state_->getVariableCount();

	res.trajectory_.reset(
			new robot_trajectory::RobotTrajectory(robot_model_.getRobotModel(),
					""));

	std::vector<double> velocity_limits(num_all_joints,
			std::numeric_limits<double>::max());

	robot_state::RobotState ks = *complete_initial_robot_state_;
	std::vector<double> positions(num_all_joints);
	double duration = trajectories_[best_trajectory_index]->getDiscretization();
	for (std::size_t i = 0;
			i < trajectories_[best_trajectory_index]->getNumPoints(); ++i)
	{
		for (std::size_t j = 0; j < num_all_joints; j++)
		{
			positions[j] = (*trajectories_[best_trajectory_index])(i, j);
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
		const std::vector<std::string>& joint_names =
				res.trajectory_->getFirstWayPoint().getVariableNames();
		for (int j = 0; j < num_all_joints; j++)
			printf("%s ", joint_names[j].c_str());
		printf("\n");
		for (int i = 0;
				i < trajectories_[best_trajectory_index]->getNumPoints(); ++i)
		{
			for (int j = 0; j < num_all_joints; j++)
			{
				printf("%f ",
						res.trajectory_->getWayPoint(i).getVariablePosition(j));
			}
			printf("\n");
		}
	}
}

void ItompPlannerNode::fillGroupJointTrajectory(const string& groupName,
		const sensor_msgs::JointState& jointGoalState,
		const moveit_msgs::Constraints& path_constraints,
		const moveit_msgs::TrajectoryConstraints& trajectory_constraints)
{
	int num_trajectories =
			PlanningParameters::getInstance()->getNumTrajectories();

	const ItompPlanningGroup* group = robot_model_.getPlanningGroup(groupName);
	int goal_index = trajectory_->getNumPoints() - 1;
	Eigen::MatrixXd::RowXpr goalPoint = trajectory_->getTrajectoryPoint(
			goal_index);
	for (int i = 0; i < group->num_joints_; ++i)
	{
		string name = group->group_joints_[i].joint_name_;
		int kdl_number = robot_model_.urdfNameToKdlNumber(name);
		if (kdl_number >= 0)
		{
			goalPoint(kdl_number) = jointGoalState.position[kdl_number];
		}
	}

	std::set<int> groupJointsKDLIndices;
	for (int i = 0; i < group->num_joints_; ++i)
	{
		groupJointsKDLIndices.insert(group->group_joints_[i].kdl_joint_index_);
	}

	trajectories_.resize(num_trajectories);
	for (int i = 0; i < num_trajectories; ++i)
	{
		trajectories_[i].reset(
				new ItompCIOTrajectory(&robot_model_,
						trajectory_->getDuration(),
						trajectory_->getDiscretization(),
						PlanningParameters::getInstance()->getNumContacts(),
						PlanningParameters::getInstance()->getPhaseDuration()));

		*(trajectories_[i].get()) = *(trajectory_.get());

		if (i != 0 && trajectory_constraints.constraints.size() != 0)
		{
			trajectories_[i]->fillInMinJerk(i, groupJointsKDLIndices, group,
					trajectory_constraints);
		}
		if (true)//path_constraints.position_constraints.size() == 0)
		{
			trajectories_[i]->fillInMinJerk(groupJointsKDLIndices,
					start_point_velocities_.row(0),
					start_point_accelerations_.row(0));
		}
		else
		{
			trajectories_[i]->fillInMinJerkCartesianTrajectory(
					groupJointsKDLIndices, start_point_velocities_.row(0),
					start_point_accelerations_.row(0), path_constraints,
					groupName);
		}
	}
}

void ItompPlannerNode::resetPlanningInfo(int trials, int component)
{
	planning_info_.clear();
	planning_info_.resize(trials, std::vector<PlanningInfo>(component));
}

void ItompPlannerNode::writePlanningInfo(int trials, int component)
{
	int best_trajectory_index = best_cost_manager_.getBestCostTrajectoryIndex();

	PlanningInfo& info = planning_info_[trials][component];
	info.time = last_planning_time_;
	info.iterations = optimizers_[best_trajectory_index]->getLastIteration()
			+ 1;
	info.cost = optimizers_[best_trajectory_index]->getBestCost();
	info.success = (optimizers_[best_trajectory_index]->isSucceed() ? 1 : 0);
}

void ItompPlannerNode::printPlanningInfoSummary()
{
	int numPlannings = planning_info_.size();
	int numComponents = planning_info_[0].size();

	vector<PlanningInfo> summary(numComponents);
	PlanningInfo sumOfSum;
	for (int j = 0; j < numComponents; ++j)
	{
		for (int i = 0; i < numPlannings; ++i)
		{
			summary[j] += planning_info_[i][j];
		}
		sumOfSum += summary[j];
	}

	// compute success rate
	// if a component fails, that trail fails.
	int numSuccess = 0;
	for (int i = 0; i < numPlannings; ++i)
	{
		bool failed = false;
		for (int j = 0; j < numComponents; ++j)
		{
			if (planning_info_[i][j].success == 0)
			{
				failed = true;
				break;
			}
		}
		if (!failed)
		{
			++numSuccess;
		}
	}

	printf("%d Trials, %d components\n", numPlannings, numComponents);
	printf("Component Iterations Time Smoothness SuccessRate\n");
	for (int j = 0; j < numComponents; ++j)
	{
		printf("%d %f %f %f %f\n", j,
				((double) summary[j].iterations) / numPlannings,
				((double) summary[j].time) / numPlannings,
				((double) summary[j].cost) / numPlannings,
				((double) summary[j].success) / numPlannings);
	}
	printf("Sum %f %f %f %f\n", ((double) sumOfSum.iterations) / numPlannings,
			((double) sumOfSum.time) / numPlannings,
			((double) sumOfSum.cost) / numPlannings,
			((double) numSuccess) / numPlannings);
	printf("\n");

	printf("plannings info\n");
	printf("Component Iterations Time Smoothness SuccessRate\n");
	for (int i = 0; i < numPlannings; ++i)
	{
		double iterationsSum = 0, timeSum = 0, costSum = 0;
		for (int j = 0; j < numComponents; ++j)
		{
			iterationsSum += planning_info_[i][j].iterations;
			timeSum += planning_info_[i][j].time;
			costSum += planning_info_[i][j].cost;
		}
		printf("[%d] %f %f %f \n", i, iterationsSum, timeSum, costSum);

	}
}

} // namespace
