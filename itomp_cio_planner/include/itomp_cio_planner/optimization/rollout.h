#ifndef ROLLOUT_H_
#define ROLLOUT_H_

#include <itomp_cio_planner/common.h>

namespace itomp_cio_planner
{

class Rollout
{
public:
	std::vector<Eigen::VectorXd> parameters_; /**< [num_dimensions] num_parameters */
	std::vector<Eigen::VectorXd> noise_; /**< [num_dimensions] num_parameters */
	std::vector<Eigen::VectorXd> noise_projected_; /**< [num_dimensions][num_time_steps] num_parameters */
	std::vector<Eigen::VectorXd> parameters_noise_projected_; /**< [num_dimensions][num_time_steps] num_parameters */
	Eigen::VectorXd state_costs_; /**< num_time_steps */
	std::vector<Eigen::VectorXd> control_costs_; /**< [num_dimensions] num_time_steps */
	std::vector<Eigen::VectorXd> total_costs_; /**< [num_dimensions] num_time_steps */
	std::vector<Eigen::VectorXd> cumulative_costs_; /**< [num_dimensions] num_time_steps */
	std::vector<Eigen::VectorXd> probabilities_; /**< [num_dimensions] num_time_steps */

	std::vector<Eigen::VectorXd> contact_parameters_; /**< [num_dimensions] num_parameters */
	std::vector<Eigen::VectorXd> contact_noise_; /**< [num_dimensions] num_parameters */
	Eigen::VectorXd contact_probabilities_; /**< [num_dimensions] num_time_steps */

	double getCost(); /**< Gets the rollout cost = state cost + control costs per dimension */
};
}

#endif
