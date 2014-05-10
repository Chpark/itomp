#include <itomp_cio_planner/optimization/improvement_manager_chomp.h>
#include <itomp_cio_planner/util/planning_parameters.h>
#include <itomp_cio_planner/util/differentiation_rules.h>
#include <itomp_cio_planner/model/itomp_robot_joint.h>
#include <Eigen/LU>

using namespace Eigen;

namespace itomp_cio_planner
{

ImprovementManagerChomp::ImprovementManagerChomp()
{

}

ImprovementManagerChomp::~ImprovementManagerChomp()
{

}

bool ImprovementManagerChomp::updatePlanningParameters()
{
  if (!ImprovementManager::updateParameters())
    return false;

	num_time_steps_ = PlanningParameters::getInstance()->getNumTimeSteps();
	num_contact_time_steps_ = evaluation_manager_->getGroupTrajectory()->getNumContactPhases() - 2;
	num_dimensions_ = evaluation_manager_->getGroupTrajectory()->getNumJoints();
	num_contact_dimensions_ = evaluation_manager_->getGroupTrajectory()->getNumContacts();
	noise_decay_ = PlanningParameters::getInstance()->getNoiseDecay();
	double noise_stddev_param = PlanningParameters::getInstance()->getNoiseStddev();
	noise_stddev_.resize(num_dimensions_);

	use_cumulative_costs_ = PlanningParameters::getInstance()->getUseCumulativeCosts();
	use_smooth_noises_ = PlanningParameters::getInstance()->getUseSmoothNoises();

	const std::vector<ItompRobotJoint>& group_joints = evaluation_manager_->getPlanningGroup()->group_joints_;
	for (int i = 0; i < group_joints.size(); ++i)
	{
		double limit_min = group_joints[i].joint_limit_min_;
		double limit_max = group_joints[i].joint_limit_max_;
		if (limit_min == limit_max)
			noise_stddev_[i] = 0.0;
		else
			noise_stddev_[i] = noise_stddev_param;
	}
	noise_stddev_contacts_ = noise_stddev_param;

	num_vars_free_ = num_time_steps_;
	num_vars_all_ = num_vars_free_ + 2 * (DIFF_RULE_LENGTH - 1);
	free_vars_start_index_ = DIFF_RULE_LENGTH - 1;
	free_vars_end_index_ = free_vars_start_index_ + num_vars_free_ - 1;
	parameters_all_.resize(num_dimensions_, Eigen::VectorXd::Zero(num_vars_all_));
	parameters_.resize(num_dimensions_, Eigen::VectorXd::Zero(num_time_steps_));
	num_contact_vars_free_ = num_contact_time_steps_;
	num_contact_vars_all_ = num_contact_vars_free_ + 2;
	free_contact_vars_start_index_ = 1;
	free_contact_vars_end_index_ = free_contact_vars_start_index_ + num_contact_vars_free_ - 1;
	contact_parameters_all_.resize(num_contact_dimensions_, Eigen::VectorXd::Zero(num_contact_vars_all_));
	contact_parameters_.resize(num_contact_dimensions_, Eigen::VectorXd::Zero(num_contact_time_steps_));

	copyGroupTrajectory();

	initializeCosts();
	initializeNoiseGenerators();
	initializeRollouts();

	preAllocateTempVariables();

	return true;
}

void ImprovementManagerChomp::initializeRollouts()
{
	num_rollouts_ = PlanningParameters::getInstance()->getNumRollouts();
	num_rollouts_reused_ = PlanningParameters::getInstance()->getNumReusedRollouts();
	num_rollouts_extra_ = 1;
	num_rollouts_gen_ = 0;
	if (num_rollouts_reused_ >= num_rollouts_)
	{
		ROS_ERROR("Number of reused rollouts must be strictly less than number of rollouts.");
		return;
	}

	// preallocate memory for a single rollout:
	Rollout rollout;

	rollout.parameters_.clear();
	rollout.noise_.clear();
	rollout.noise_projected_.clear();
	rollout.parameters_noise_projected_.clear();
	rollout.control_costs_.clear();
	rollout.total_costs_.clear();
	rollout.cumulative_costs_.clear();
	rollout.probabilities_.clear();
	for (int d = 0; d < num_dimensions_; ++d)
	{
		rollout.parameters_.push_back(VectorXd::Zero(num_time_steps_));
		rollout.noise_.push_back(VectorXd::Zero(num_time_steps_));
		rollout.noise_projected_.push_back(VectorXd::Zero(num_time_steps_));
		rollout.control_costs_.push_back(VectorXd::Zero(num_time_steps_));
		rollout.total_costs_.push_back(VectorXd::Zero(num_time_steps_));
		rollout.cumulative_costs_.push_back(VectorXd::Zero(num_time_steps_));
		rollout.probabilities_.push_back(VectorXd::Zero(num_time_steps_));
	}
	rollout.state_costs_ = VectorXd::Zero(num_time_steps_);

	rollout.contact_parameters_.clear();
	rollout.contact_noise_.clear();
	for (int d = 0; d < num_contact_dimensions_; ++d)
	{
		rollout.contact_parameters_.push_back(VectorXd::Zero(num_contact_time_steps_));
		rollout.contact_noise_.push_back(VectorXd::Zero(num_contact_time_steps_));
	}
	rollout.contact_probabilities_ = VectorXd::Zero(num_contact_time_steps_);

	// duplicate this rollout:
	for (int r = 0; r < num_rollouts_; ++r)
		rollouts_.push_back(rollout);

	for (int r = 0; r < num_rollouts_reused_; ++r)
		reused_rollouts_.push_back(rollout);

	for (int r = 0; r < num_rollouts_extra_; ++r)
		extra_rollouts_.push_back(rollout);

	rollouts_reused_ = false;
	rollouts_reused_next_ = false;
	extra_rollouts_added_ = false;
	rollout_cost_sorter_.reserve(num_rollouts_);

	rollout_costs_ = Eigen::MatrixXd::Zero(num_rollouts_, num_time_steps_);
	tmp_rollout_cost_ = Eigen::VectorXd::Zero(num_time_steps_);
}

void ImprovementManagerChomp::initializeCosts()
{
	control_cost_weight_ = PlanningParameters::getInstance()->getSmoothnessCostWeight();

	double multiplier = 1.0;
	differentiation_matrices_.clear();
	differentiation_matrices_.resize(NUM_DIFF_RULES, MatrixXd::Zero(num_vars_all_, num_vars_all_));
	for (int d = 0; d < NUM_DIFF_RULES; ++d)
	{
		multiplier /= PlanningParameters::getInstance()->getTrajectoryDiscretization();
		for (int i = 0; i < num_vars_all_; i++)
		{
			for (int j = -DIFF_RULE_LENGTH / 2; j <= DIFF_RULE_LENGTH / 2; j++)
			{
				int index = i + j;
				if (index < 0)
					continue;
				if (index >= num_vars_all_)
					continue;
				differentiation_matrices_[d](i, index) = multiplier * DIFF_RULES[d][j + DIFF_RULE_LENGTH / 2];
			}
		}
		//ROS_INFO_STREAM(differentiation_matrices_[d]);
	}

	control_costs_all_.clear();
	control_costs_.clear();
	inv_control_costs_.clear();
	for (int d = 0; d < num_dimensions_; ++d)
	{
		// construct the quadratic cost matrices (for all variables)
		MatrixXd cost_all = MatrixXd::Identity(num_vars_all_, num_vars_all_)
				* PlanningParameters::getInstance()->getRidgeFactor();
		for (int i = 0; i < NUM_DIFF_RULES; ++i)
		{
			cost_all += PlanningParameters::getInstance()->getSmoothnessCosts()[i]
					* (differentiation_matrices_[i].transpose() * differentiation_matrices_[i]);
		}
		control_costs_all_.push_back(cost_all);

		// extract the quadratic cost just for the free variables:
		MatrixXd cost_free = cost_all.block(DIFF_RULE_LENGTH - 1, DIFF_RULE_LENGTH - 1, num_vars_free_, num_vars_free_);
		control_costs_.push_back(cost_free);

		inv_control_costs_.push_back(cost_free.fullPivLu().inverse());
	}

	ROS_INFO("Precomputing projection matrices..");
	projection_matrix_.resize(num_dimensions_);
	for (int d = 0; d < num_dimensions_; ++d)
	{
		if (use_smooth_noises_)
		{
			projection_matrix_[d] = inv_control_costs_[d];
			for (int p = 0; p < num_time_steps_; ++p)
			{
				double column_max = inv_control_costs_[d](0, p);
				for (int p2 = 1; p2 < num_time_steps_; ++p2)
				{
					if (inv_control_costs_[d](p2, p) > column_max)
						column_max = inv_control_costs_[d](p2, p);
				}
				projection_matrix_[d].col(p) *= (1.0 / (num_time_steps_ * column_max));
			}
		}
		else
		{
			projection_matrix_[d].setIdentity(inv_control_costs_[d].rows(), inv_control_costs_[d].cols());
		}
	}
	ROS_INFO("Done precomputing projection matrices.");
}

void ImprovementManagerChomp::initializeNoiseGenerators()
{
	// invert the control costs, initialize noise generators:
	for (int d = 0; d < num_dimensions_; ++d)
	{
		MultivariateGaussian mvg(VectorXd::Zero(num_time_steps_), inv_control_costs_[d]);
		noise_generators_.push_back(mvg);
	}
	contact_noise_generators_.clear();
	for (int d = 0; d < num_contact_dimensions_; ++d)
	{
		MultivariateGaussian mvg(VectorXd::Zero(num_contact_time_steps_), MatrixXd::Identity(num_contact_time_steps_,
				num_contact_time_steps_));
		contact_noise_generators_.push_back(mvg);
	}
}

bool ImprovementManagerChomp::preAllocateTempVariables()
{
	tmp_noise_.clear();
	tmp_contact_noise_.clear();
	tmp_parameters_.clear();
	parameter_updates_.clear();
	contact_parameter_updates_.clear();
	for (int d = 0; d < num_dimensions_; ++d)
	{
		tmp_noise_.push_back(VectorXd::Zero(num_time_steps_));
		tmp_parameters_.push_back(VectorXd::Zero(num_time_steps_));
		parameter_updates_.push_back(MatrixXd::Zero(num_time_steps_, num_time_steps_));
		time_step_weights_.push_back(VectorXd::Zero(num_time_steps_));
	}
	for (int d = 0; d < num_contact_dimensions_; ++d)
	{
		tmp_contact_noise_.push_back(VectorXd::Zero(num_contact_time_steps_));
		contact_parameter_updates_.push_back(VectorXd::Zero(num_contact_time_steps_));
	}
	tmp_max_cost_ = VectorXd::Zero(num_time_steps_);
	tmp_min_cost_ = VectorXd::Zero(num_time_steps_);
	tmp_sum_rollout_probabilities_ = VectorXd::Zero(num_time_steps_);

	return true;
}

void ImprovementManagerChomp::runSingleIteration(int iteration)
{
	// compute appropriate noise values
	std::vector<double> noise;
	noise.resize(num_dimensions_);
	for (int i = 0; i < num_dimensions_; ++i)
	{
		noise[i] = noise_stddev_[i] * pow(noise_decay_, iteration);
	}
	std::vector<double> contact_noise;
	contact_noise.resize(num_contact_dimensions_);
	for (int i = 0; i < num_contact_dimensions_; ++i)
	{
		contact_noise[i] = noise_stddev_contacts_ * pow(noise_decay_, iteration);
	}

	getParameters();

	// get rollouts and execute them
	generateRollouts(noise, contact_noise);

	for (unsigned int r = 0; r < rollouts_.size(); ++r)
	{
		//evaluation_manager_->evaluate(rollouts_[r].parameters_, rollouts_[r].contact_parameters_, tmp_rollout_cost_);
		rollout_costs_.row(r) = tmp_rollout_cost_.transpose();
	}

	setRolloutCosts();

	// improve the policy
	computeUpdates();
	updateParameters();

	// get a noise-less rollout to check the cost
	getParameters();
	//evaluation_manager_->evaluate(parameters_, contact_parameters_, tmp_rollout_cost_);

	// add the noiseless rollout into policy_improvement:
	addExtraRollout(parameters_, contact_parameters_, tmp_rollout_cost_);
}

bool ImprovementManagerChomp::generateRollouts(const std::vector<double>& noise_stddev,
		const std::vector<double>& contact_noise_stddev)
{
	// we assume here that rollout_parameters_ and rollout_noise_ have already been allocated
	num_rollouts_gen_ = num_rollouts_ - num_rollouts_reused_;
	if (!rollouts_reused_next_)
	{
		num_rollouts_gen_ = num_rollouts_;
		if (num_rollouts_reused_ > 0)
		{
			rollouts_reused_next_ = true;
		}
	}
	else
	{
		// figure out which rollouts to reuse
		rollout_cost_sorter_.clear();
		for (int r = 0; r < num_rollouts_; ++r)
		{
			double cost = rollouts_[r].getCost();
			rollout_cost_sorter_.push_back(std::make_pair(cost, r));
		}
		if (extra_rollouts_added_)
		{
			for (int r = 0; r < num_rollouts_extra_; ++r)
			{
				double cost = extra_rollouts_[r].getCost();
				rollout_cost_sorter_.push_back(std::make_pair(cost, -r - 1));
				// index is -ve if rollout is taken from extra_rollouts
			}
			extra_rollouts_added_ = false;
		}
		std::sort(rollout_cost_sorter_.begin(), rollout_cost_sorter_.end());

		// use the best ones: (copy them into reused_rollouts)
		for (int r = 0; r < num_rollouts_reused_; ++r)
		{
			double reuse_index = rollout_cost_sorter_[r].second;

			//ROS_INFO("Reuse %d, cost = %lf", r, reuse_cost);

			if (reuse_index >= 0)
				reused_rollouts_[r] = rollouts_[reuse_index];
			else
			{
				//ROS_INFO("Reused noise-less rollout of cost %f", reuse_cost);
				reused_rollouts_[r] = extra_rollouts_[-reuse_index - 1];
			}
		}
		// copy them back from reused_rollouts_ into rollouts_
		for (int r = 0; r < num_rollouts_reused_; ++r)
		{
			rollouts_[num_rollouts_gen_ + r] = reused_rollouts_[r];

			// update the noise based on the new parameters:
			for (int d = 0; d < num_dimensions_; ++d)
			{
				rollouts_[num_rollouts_gen_ + r].noise_[d] = rollouts_[num_rollouts_gen_ + r].parameters_[d]
						- parameters_[d];
			}
			for (int d = 0; d < num_contact_dimensions_; ++d)
			{
				rollouts_[num_rollouts_gen_ + r].contact_noise_[d]
						= rollouts_[num_rollouts_gen_ + r].contact_parameters_[d] - contact_parameters_[d];
			}
		}
		rollouts_reused_ = true;
	}

	// generate new rollouts
	for (int d = 0; d < num_dimensions_; ++d)
	{
		for (int r = 0; r < num_rollouts_gen_; ++r)
		{
			noise_generators_[d].sample(tmp_noise_[d]);
			rollouts_[r].noise_[d] = noise_stddev[d] * tmp_noise_[d];
			rollouts_[r].parameters_[d] = parameters_[d] + rollouts_[r].noise_[d];
		}
	}
	// contact
	const double maxContactValue = PlanningParameters::getInstance()->getContactVariableInitialValues()[0];
	for (int d = 0; d < num_contact_dimensions_; ++d)
	{
		for (int r = 0; r < num_rollouts_gen_; ++r)
		{
			contact_noise_generators_[d].sample(tmp_contact_noise_[d]);
			rollouts_[r].contact_noise_[d] = contact_noise_stddev[d] * tmp_contact_noise_[d];
			for (int i = 0; i < tmp_contact_noise_[d].rows(); ++i)
			{
				if (rollouts_[r].contact_noise_[d](i) + contact_parameters_[d](i) > maxContactValue)
					rollouts_[r].contact_noise_[d](i) = maxContactValue - contact_parameters_[d](i);
				else if (rollouts_[r].contact_noise_[d](i) + contact_parameters_[d](i) < 0)
					rollouts_[r].contact_noise_[d](i) = -contact_parameters_[d](i);
			}
			rollouts_[r].contact_parameters_[d] = contact_parameters_[d] + rollouts_[r].contact_noise_[d];
		}
	}

	return true;
}

void ImprovementManagerChomp::copyGroupTrajectory()
{
  /*
	for (int d = 0; d < num_dimensions_; ++d)
	{
		parameters_all_[d].segment(free_vars_start_index_, num_vars_free_)
				= evaluation_manager_->getGroupTrajectory()->getFreeJointTrajectoryBlock(d);
	}
	for (int d = 0; d < num_contact_dimensions_; ++d)
	{
		contact_parameters_all_[d].segment(free_contact_vars_start_index_, num_contact_vars_free_)
				= evaluation_manager_->getGroupTrajectory()->getFreeContactTrajectoryBlock(d);
	}
	*/
}

void ImprovementManagerChomp::computeRolloutControlCost(Rollout& rollout)
{
	// this measures the accelerations and squares them
	for (int d = 0; d < num_dimensions_; ++d)
	{
		VectorXd params_all = parameters_all_[d];
		VectorXd costs_all = VectorXd::Zero(num_vars_all_);

		params_all.segment(free_vars_start_index_, num_vars_free_) = rollout.parameters_[d]
				+ rollout.noise_projected_[d];
		VectorXd acc_all = VectorXd::Zero(num_vars_all_);
		for (int i = 0; i < NUM_DIFF_RULES; ++i)
		{
			acc_all = differentiation_matrices_[i] * params_all;
			costs_all += control_cost_weight_ * PlanningParameters::getInstance()->getSmoothnessCosts()[i]
					* (acc_all.cwiseProduct(acc_all));
		}

		rollout.control_costs_[d] = costs_all.segment(free_vars_start_index_, num_vars_free_);
		for (int i = 0; i < free_vars_start_index_; ++i)
		{
			rollout.control_costs_[d](0) += costs_all(i);
			rollout.control_costs_[d](num_vars_free_ - 1) += costs_all(num_vars_all_ - (i + 1));
		}
	}

}

bool ImprovementManagerChomp::setRolloutCosts()
{
	for (int r = 0; r < num_rollouts_; ++r)
	{
		computeRolloutControlCost(rollouts_[r]);
	}

	for (int r = 0; r < num_rollouts_gen_; ++r)
	{
		rollouts_[r].state_costs_ = rollout_costs_.row(r).transpose();
	}

	return true;
}

void ImprovementManagerChomp::computeUpdates()
{
	computeRolloutCumulativeCosts();
	computeRolloutProbabilities();
	computeParameterUpdates();
}

bool ImprovementManagerChomp::computeRolloutCumulativeCosts()
{
	// compute cumulative costs at each timestep
	for (int r = 0; r < num_rollouts_; ++r)
	{
		for (int d = 0; d < num_dimensions_; ++d)
		{
			rollouts_[r].total_costs_[d] = rollouts_[r].state_costs_ + rollouts_[r].control_costs_[d];
			rollouts_[r].cumulative_costs_[d] = rollouts_[r].total_costs_[d];
			if (use_cumulative_costs_)
			{
				for (int t = num_time_steps_ - 2; t >= 0; --t)
				{
					rollouts_[r].cumulative_costs_[d](t) += rollouts_[r].cumulative_costs_[d](t + 1);
				}
			}
		}
	}
	return true;
}

bool ImprovementManagerChomp::computeRolloutProbabilities()
{
	for (int d = 0; d < num_dimensions_; ++d)
	{
		for (int t = 0; t < num_time_steps_; t++)
		{

			// find min and max cost over all rollouts:
			double min_cost = rollouts_[0].cumulative_costs_[d](t);
			double max_cost = min_cost;
			for (int r = 1; r < num_rollouts_; ++r)
			{
				double c = rollouts_[r].cumulative_costs_[d](t);
				if (c < min_cost)
					min_cost = c;
				if (c > max_cost)
					max_cost = c;
			}

			double denom = max_cost - min_cost;

			time_step_weights_[d][t] = denom;

			// prevent divide by zero:
			if (denom < 1e-8)
				denom = 1e-8;

			double p_sum = 0.0;
			for (int r = 0; r < num_rollouts_; ++r)
			{
				// the -10.0 here is taken from the paper:
				rollouts_[r].probabilities_[d](t) = exp(-10.0 * (rollouts_[r].cumulative_costs_[d](t) - min_cost)
						/ denom);
				p_sum += rollouts_[r].probabilities_[d](t);
			}
			for (int r = 0; r < num_rollouts_; ++r)
			{
				rollouts_[r].probabilities_[d](t) /= p_sum;
			}

		}
	}

	for (int t = 0; t < num_contact_time_steps_; t++)
	{
		for (int r = 0; r < num_rollouts_; ++r)
		{
			rollouts_[r].contact_probabilities_(t) = 0.0;
			for (int d = 0; d < num_dimensions_; ++d)
			{
				rollouts_[r].contact_probabilities_(t) += rollouts_[r].probabilities_[d](t);
			}
			rollouts_[r].contact_probabilities_(t) /= num_dimensions_;
		}
	}

	return true;
}

bool ImprovementManagerChomp::computeParameterUpdates()
{
	for (int d = 0; d < num_dimensions_; ++d)
	{
		parameter_updates_[d] = MatrixXd::Zero(num_time_steps_, num_time_steps_);

		for (int r = 0; r < num_rollouts_; ++r)
		{
			parameter_updates_[d].row(0).transpose() += rollouts_[r].noise_[d].cwiseProduct(
					rollouts_[r].probabilities_[d]);
		}

		// reweighting the updates per time-step
		double weight = 0.0;
		double weight_sum = 0.0;
		for (int t = 0; t < num_time_steps_; ++t)
		{
			weight = time_step_weights_[d][t];
			weight_sum += weight;
			parameter_updates_[d](0, t) *= weight;
		}
		if (weight_sum < 1e-6)
			weight_sum = 1e-6;
		parameter_updates_[d].row(0) *= num_time_steps_ / weight_sum;

		parameter_updates_[d].row(0).transpose() = projection_matrix_[d] * parameter_updates_[d].row(0).transpose();
	}

	for (int d = 0; d < num_contact_dimensions_; ++d)
	{
		contact_parameter_updates_[d] = VectorXd::Zero(num_contact_time_steps_);

		for (int r = 0; r < num_rollouts_; ++r)
		{
			contact_parameter_updates_[d] += rollouts_[r].contact_noise_[d].cwiseProduct(
					rollouts_[r].contact_probabilities_);
		}
	}
	return true;
}

bool ImprovementManagerChomp::updateParameters()
{
	double divisor = 1.0;
	for (int d = 0; d < num_dimensions_; ++d)
	{
		parameters_all_[d].segment(free_vars_start_index_, num_vars_free_).transpose() += divisor
				* parameter_updates_[d].row(0);
	}
	for (int d = 0; d < num_contact_dimensions_; ++d)
	{
		contact_parameters_all_[d].segment(free_contact_vars_start_index_, num_contact_vars_free_) += divisor
				* contact_parameter_updates_[d];
	}

	return true;
}

void ImprovementManagerChomp::getParameters()
{
	// save the latest policy parameters:
	for (int d = 0; d < num_dimensions_; ++d)
	{
		parameters_[d] = parameters_all_[d].segment(free_vars_start_index_, num_vars_free_);
	}
	for (int d = 0; d < num_contact_dimensions_; ++d)
	{
		contact_parameters_[d] = contact_parameters_all_[d].segment(free_contact_vars_start_index_,
				num_contact_vars_free_);
	}
}

void ImprovementManagerChomp::addExtraRollout(std::vector<Eigen::VectorXd>& parameters,
		std::vector<Eigen::VectorXd>& contact_parameters, Eigen::VectorXd& costs)
{
	for (int r = 0; r < num_rollouts_extra_; ++r)
	{
		extra_rollouts_[r].parameters_ = parameters;
		extra_rollouts_[r].contact_parameters_ = contact_parameters;
		extra_rollouts_[r].state_costs_ = costs;
		computeNoise(extra_rollouts_[r]);
		computeProjectedNoise(extra_rollouts_[r]);
		computeRolloutControlCost(extra_rollouts_[r]);
	}

	extra_rollouts_added_ = true;
}

bool ImprovementManagerChomp::computeNoise(Rollout& rollout)
{
	for (int d = 0; d < num_dimensions_; ++d)
	{
		rollout.noise_[d] = rollout.parameters_[d] - parameters_[d];
	}
	for (int d = 0; d < num_contact_dimensions_; ++d)
	{
		rollout.contact_noise_[d] = rollout.contact_parameters_[d] - contact_parameters_[d];
	}
	return true;
}

bool ImprovementManagerChomp::computeProjectedNoise(Rollout& rollout)
{
	for (int d = 0; d < num_dimensions_; ++d)
	{
		rollout.noise_projected_[d] = projection_matrix_[d] * rollout.noise_[d];
		//rollout.parameters_noise_projected_[d] = rollout.parameters_[d] + rollout.noise_projected_[d];
	}

	return true;
}

}
