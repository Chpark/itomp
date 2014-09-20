/*
 * planningParameters.h
 *
 *  Created on: Oct 23, 2013
 *      Author: cheonhyeonpark
 */

#ifndef PLANNINGPARAMETERS_H_
#define PLANNINGPARAMETERS_H_

#include <itomp_cio_planner/common.h>

namespace itomp_cio_planner
{

class PlanningParameters: public Singleton<PlanningParameters>
{
public:
	PlanningParameters();
	virtual ~PlanningParameters();

	void initFromNodeHandle();
	int getUpdateIndex() const;

	void setTrajectoryDuration(double trajectory_duration);
	double getTrajectoryDuration() const;
	double getTrajectoryDiscretization() const;
	double getPlanningTimeLimit() const;
	void setPlanningTimeLimit(double planning_time_limit);
	int getMaxIterations() const;
	int getMaxIterationsAfterCollisionFree() const;
	double getSmoothnessCostWeight() const;
	double getObstacleCostWeight() const;
	double getValidityCostWeight() const;
	double getEndeffectorVelocityCostWeight() const;
	double getTorqueCostWeight() const;
	double getContactInvariantCostWeight() const;
	double getPhysicsViolationCostWeight() const;
	double getGoalPoseCostWeight() const;
	double getCOMCostWeight() const;
	double getRVOCostWeight() const;
	double getFTRCostWeight() const;
	double getCartesianTrajectoryCostWeight() const;
	double getSingularityCostWeight() const;
	double getFrictionConeCostWeight() const;

	bool getAnimatePath() const;
	double getSmoothnessCostVelocity() const;
	double getSmoothnessCostAcceleration() const;
	double getSmoothnessCostJerk() const;
	std::vector<double> getSmoothnessCosts() const;
	double getRidgeFactor() const;
	bool getAnimateEndeffector() const;
	const std::multimap<std::string, std::string>& getGroupEndeffectorNames() const;
	int getNumTrajectories() const;
	int getNumTrials() const;
	int getNumRollouts() const;
	int getNumReusedRollouts() const;
	double getNoiseStddev() const;
	double getNoiseDecay() const;
	bool getUseCumulativeCosts() const;
	bool getUseSmoothNoises() const;
	int getNumContacts() const;
	const std::vector<double>& getContactVariableInitialValues() const;
	const std::vector<double>& getContactVariableGoalValues() const;
	bool getPrintPlanningInfo() const
	{
		return print_planning_info_;
	}

	double getPhaseDuration() const
	{
		return keyframe_duration_;
	}
	double getFrictionCoefficient() const
	{
		return friction_coefficient_;
	}
	std::string getLowerBodyRoot() const
	{
		return lower_body_root_;
	}
	double getTemporaryVariable(int i) const
	{
		return temporary_variables_[i];
	}
	double getPlanningStepSize() const
	{
		return planning_step_size_;
	}

	int getNumTimeSteps() const
	{
		return (int) (trajectory_duration_ / trajectory_discretization_) - 1;
	}

	const std::map<std::string, double>& getJointVelocityLimits() const;

	std::string getEnvironmentModel() const;
	const std::vector<double>& getEnvironmentModelPosition() const;
	double getEnvironmentModelScale() const;
	bool getHasRoot6d() const;

	const std::map<std::string, std::vector<std::string> >& getContactPoints() const;

private:
	int updateIndex;
	double trajectory_duration_;
	double trajectory_discretization_;
	double planning_time_limit_;
	int max_iterations_;
	int max_iterations_after_collision_free_;
	double smoothness_cost_weight_;
	double obstacle_cost_weight_;
	double state_validity_cost_weight_;
	double torque_cost_weight_;
	double endeffector_velocity_cost_weight_;
	double contact_invariant_cost_weight_;
	double physics_violation_cost_weight_;
	double goal_pose_cost_weight_;
	double com_cost_weight_;
	double ftr_cost_weight_;
	double rvo_cost_weight_;
	double cartesian_trajectory_cost_weight_;
	double singularity_cost_weight_;
	double friction_cone_cost_weight_;
	bool animate_path_;
	double smoothness_cost_velocity_;
	double smoothness_cost_acceleration_;
	double smoothness_cost_jerk_;
	double ridge_factor_;
	bool animate_endeffector_;
	std::multimap<std::string, std::string> group_endeffector_names_;
	std::map<std::string, std::vector<std::string> > contact_points_;
	int num_trajectories_;
	double planning_step_size_;
	int num_time_steps_;

	std::map<std::string, double> joint_velocity_limits_;

	double keyframe_duration_;
	double friction_coefficient_;
	std::string lower_body_root_;

	bool print_planning_info_;

	int num_contacts_;
	std::vector<double> contact_variable_initial_values_;
	std::vector<double> contact_variable_goal_values_;

	int num_trials_;

	int num_rollouts_;
	int num_reused_rollouts_;
	double noise_stddev_;
	double noise_decay_;
	bool use_cumulative_costs_;
	bool use_smooth_noises_;

	std::vector<double> temporary_variables_;

	std::string environment_model_;
	std::vector<double> environment_model_position_;
	double environment_model_scale_;

	bool has_root_6d_;

	friend class Singleton<PlanningParameters> ;
};

/////////////////////// inline functions follow ////////////////////////
inline int PlanningParameters::getUpdateIndex() const
{
	return updateIndex;
}

inline void PlanningParameters::setTrajectoryDuration(
		double trajectory_duration)
{
	trajectory_duration_ = trajectory_duration;
}

inline double PlanningParameters::getTrajectoryDuration() const
{
	return trajectory_duration_;
}

inline double PlanningParameters::getTrajectoryDiscretization() const
{
	return trajectory_discretization_;
}

inline double PlanningParameters::getPlanningTimeLimit() const
{
	return planning_time_limit_;
}

inline void PlanningParameters::setPlanningTimeLimit(double planning_time_limit)
{
	planning_time_limit_ = planning_time_limit;
}

inline int PlanningParameters::getMaxIterations() const
{
	return max_iterations_;
}

inline int PlanningParameters::getMaxIterationsAfterCollisionFree() const
{
	return max_iterations_after_collision_free_;
}

inline double PlanningParameters::getSmoothnessCostWeight() const
{
	return smoothness_cost_weight_;
}

inline double PlanningParameters::getObstacleCostWeight() const
{
	return obstacle_cost_weight_;
}

inline double PlanningParameters::getValidityCostWeight() const
{
	return state_validity_cost_weight_;
}

inline double PlanningParameters::getEndeffectorVelocityCostWeight() const
{
	return endeffector_velocity_cost_weight_;
}

inline double PlanningParameters::getTorqueCostWeight() const
{
	return torque_cost_weight_;
}

inline double PlanningParameters::getContactInvariantCostWeight() const
{
	return contact_invariant_cost_weight_;
}

inline double PlanningParameters::getPhysicsViolationCostWeight() const
{
	return physics_violation_cost_weight_;
}

inline double PlanningParameters::getGoalPoseCostWeight() const
{
	return goal_pose_cost_weight_;
}

inline double PlanningParameters::getCartesianTrajectoryCostWeight() const
{
	return cartesian_trajectory_cost_weight_;
}

inline double PlanningParameters::getSingularityCostWeight() const
{
	return singularity_cost_weight_;
}

inline bool PlanningParameters::getAnimatePath() const
{
	return animate_path_;
}

inline double PlanningParameters::getSmoothnessCostVelocity() const
{
	return smoothness_cost_velocity_;
}

inline double PlanningParameters::getSmoothnessCostAcceleration() const
{
	return smoothness_cost_acceleration_;
}

inline double PlanningParameters::getSmoothnessCostJerk() const
{
	return smoothness_cost_jerk_;
}

inline double PlanningParameters::getRidgeFactor() const
{
	return ridge_factor_;
}

inline bool PlanningParameters::getAnimateEndeffector() const
{
	return animate_endeffector_;
}

inline const std::multimap<std::string, std::string>& PlanningParameters::getGroupEndeffectorNames() const
{
	return group_endeffector_names_;
}

inline std::vector<double> PlanningParameters::getSmoothnessCosts() const
{
	std::vector<double> ret(3);
	ret[0] = smoothness_cost_velocity_;
	ret[1] = smoothness_cost_acceleration_;
	ret[2] = smoothness_cost_jerk_;
	return ret;
}

inline double PlanningParameters::getCOMCostWeight() const
{
	return com_cost_weight_;
}

inline double PlanningParameters::getFTRCostWeight() const
{
	return ftr_cost_weight_;
}

inline double PlanningParameters::getRVOCostWeight() const
{
	return rvo_cost_weight_;
}

inline int PlanningParameters::getNumTrajectories() const
{
	return num_trajectories_;
}

inline const std::map<std::string, double>& PlanningParameters::getJointVelocityLimits() const
{
	return joint_velocity_limits_;
}

inline int PlanningParameters::getNumTrials() const
{
	return num_trials_;
}

inline int PlanningParameters::getNumContacts() const
{
	return num_contacts_;
}

inline const std::vector<double>& PlanningParameters::getContactVariableInitialValues() const
{
	return contact_variable_initial_values_;
}

inline const std::vector<double>& PlanningParameters::getContactVariableGoalValues() const
{
	return contact_variable_goal_values_;
}

inline int PlanningParameters::getNumRollouts() const
{
	return num_rollouts_;
}
inline int PlanningParameters::getNumReusedRollouts() const
{
	return num_reused_rollouts_;
}
inline double PlanningParameters::getNoiseStddev() const
{
	return noise_stddev_;
}
inline double PlanningParameters::getNoiseDecay() const
{
	return noise_decay_;
}
inline bool PlanningParameters::getUseCumulativeCosts() const
{
	return use_cumulative_costs_;
}
inline bool PlanningParameters::getUseSmoothNoises() const
{
	return use_smooth_noises_;
}

inline std::string PlanningParameters::getEnvironmentModel() const
{
	return environment_model_;
}

inline const std::vector<double>& PlanningParameters::getEnvironmentModelPosition() const
{
	return environment_model_position_;
}

inline double PlanningParameters::getEnvironmentModelScale() const
{
	return environment_model_scale_;
}

inline bool PlanningParameters::getHasRoot6d() const
{
	return has_root_6d_;
}

inline double PlanningParameters::getFrictionConeCostWeight() const
{
	return friction_cone_cost_weight_;
}

inline const std::map<std::string, std::vector<std::string> >& PlanningParameters::getContactPoints() const
{
	return contact_points_;
}

}
#endif /* PLANNINGPARAMETERS_H_ */
