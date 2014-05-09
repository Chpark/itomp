#ifndef EVALUATION_MANAGER_H_
#define EVALUATION_MANAGER_H_

#include <itomp_cio_planner/common.h>
#include <itomp_cio_planner/model/itomp_robot_model.h>
#include <itomp_cio_planner/trajectory/itomp_cio_trajectory.h>
#include <itomp_cio_planner/cost/smoothness_cost.h>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <ros/publisher.h>
#include <Eigen/StdVector>
#include <moveit/planning_scene/planning_scene.h>

namespace itomp_cio_planner
{
class TrajectoryCostAccumulator;
class ItompPlanningGroup;
class EvaluationManager
{
public:
	EvaluationManager(int* iteration);
	virtual ~EvaluationManager();

	void initialize(ItompCIOTrajectory *full_trajectory, ItompCIOTrajectory *group_trajectory,
			ItompRobotModel *robot_model, const ItompPlanningGroup *planning_group, double planning_start_time,
			double trajectory_start_time, TrajectoryCostAccumulator *costAccumulator);

	double evaluate(Eigen::MatrixXd& parameters, Eigen::MatrixXd& vel_parameters,
	    Eigen::MatrixXd& contact_parameters, Eigen::VectorXd& costs);

	bool isLastTrajectoryFeasible() const;

	void handleJointLimits();
	void updateFullTrajectory();
	bool performForwardKinematics(); /**< Return true if collision free */
	void computeTrajectoryValidity();
	void render(int trajectory_index);

	const ItompCIOTrajectory* getGroupTrajectory() const;
	const ItompPlanningGroup* getPlanningGroup() const;

	void optimize_nlp(bool add_noise);
	void postprocess_ik();

	friend class test_function;

private:
	void computeMassAndGravityForce();
	void computeWrenchSum();
	void computeStabilityCosts();
	void updateCoM(int point);
	void computeCollisionCosts();

	int getIteration() const;

	const KDL::Vector& getSegmentPosition(int point, const std::string& segmentName) const;
	const KDL::Vector& getSegmentPosition(int point, int segmentIndex) const;

	ItompCIOTrajectory *full_trajectory_;
	ItompCIOTrajectory *group_trajectory_;

	TrajectoryCostAccumulator *costAccumulator_;

	planning_scene::PlanningScenePtr planning_scene_;

	double planning_start_time_;
	double trajectory_start_time_;

	const ItompRobotModel *robot_model_;
	const ItompPlanningGroup *planning_group_;
	std::string robot_name_;

	KDL::JntArray kdl_joint_array_;

	int* iteration_;

	int num_joints_;
	int num_contacts_;
	int num_points_;
	int num_contact_points_;

	std::vector<itomp_cio_planner::SmoothnessCost> joint_costs_;
	std::vector<int> group_joint_to_kdl_joint_index_;

	std::vector<std::vector<KDL::Vector> > joint_axis_;
	std::vector<std::vector<KDL::Vector> > joint_pos_;
	std::vector<std::vector<KDL::Frame> > segment_frames_;
	std::vector<std::vector<Eigen::Map<Eigen::Vector3d> > > joint_axis_eigen_;
	std::vector<std::vector<Eigen::Map<Eigen::Vector3d> > > joint_pos_eigen_;

	std::vector<int> state_is_in_collision_; /**< Array containing a boolean about collision info for each point in the trajectory */
	bool is_collision_free_;
	bool last_trajectory_collision_free_;

	//arm_navigation_msgs::RobotState robot_state_;
	std::vector<int> state_validity_;
	bool trajectory_validity_;

	Eigen::VectorXd dynamic_obstacle_cost_;

	// physics
	double totalMass_;
	std::vector<double> masses_;
	int numMassSegments_;
	KDL::Vector gravityForce_;
	std::vector<KDL::Wrench> wrenchSum_;
	std::vector<std::vector<KDL::Vector> > linkPositions_;
	std::vector<std::vector<KDL::Vector> > linkVelocities_;
	std::vector<std::vector<KDL::Vector> > linkAngularVelocities_;
	std::vector<KDL::Vector> CoMPositions_;
	std::vector<KDL::Vector> CoMVelocities_;
	std::vector<KDL::Vector> CoMAccelerations_;
	std::vector<KDL::Vector> AngularMomentums_;
	std::vector<KDL::Vector> Torques_;
	std::vector<std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > > tmpContactViolationVector_;
	std::vector<std::vector<KDL::Vector> > tmpContactPointVelVector_;

	std::vector<double> stateContactInvariantCost_;
	std::vector<double> statePhysicsViolationCost_;
	std::vector<double> stateCollisionCost_;

	ros::Publisher vis_marker_array_pub_;
	ros::Publisher vis_marker_pub_;

	// TODO: refactoring
	friend class TrajectoryCostAccumulator;
	friend class TrajectoryCost;
	friend class TrajectorySmoothnessCost;
	friend class TrajectoryCollisionCost;
	friend class TrajectoryValidityCost;
	friend class TrajectoryContactInvariantCost;
	friend class TrajectoryPhysicsViolationCost;
	friend class TrajectoryGoalPoseCost;
	friend class TrajectoryCoMCost;
};

inline bool EvaluationManager::isLastTrajectoryFeasible() const
{
	return last_trajectory_collision_free_;
}

inline const KDL::Vector& EvaluationManager::getSegmentPosition(int point, const std::string& segmentName) const
{
	int sn = robot_model_->getForwardKinematicsSolver()->segmentNameToIndex(segmentName);
	return getSegmentPosition(point, sn);
}

inline const KDL::Vector& EvaluationManager::getSegmentPosition(int point, int segmentIndex) const
{
	return segment_frames_[point][segmentIndex].p;
}

inline int EvaluationManager::getIteration() const
{
	return *iteration_;
}

inline const ItompCIOTrajectory* EvaluationManager::getGroupTrajectory() const
{
	return group_trajectory_;
}

inline const ItompPlanningGroup* EvaluationManager::getPlanningGroup() const
{
	return planning_group_;
}

}

#endif
