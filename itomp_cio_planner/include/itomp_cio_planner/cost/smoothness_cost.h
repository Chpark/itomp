#ifndef SMOOTHNESS_COST_H_
#define SMOOTHNESS_COST_H_

#include <itomp_cio_planner/common.h>
#include <itomp_cio_planner/trajectory/itomp_cio_trajectory.h>

namespace itomp_cio_planner
{

class SmoothnessCost
{
public:
	SmoothnessCost(const ItompCIOTrajectory& trajectory, int joint_number, const std::vector<double>& derivative_costs,
			double ridge_factor = 0.0);
	virtual ~SmoothnessCost();

	const Eigen::MatrixXd& getQuadraticCostInverse() const;
	const Eigen::MatrixXd& getQuadraticCost() const;

	double getCost(Eigen::MatrixXd::ColXpr joint_trajectory) const;
	double getCost(Eigen::MatrixXd::ConstColXpr joint_trajectory) const;

	double getMaxQuadCostInvValue() const;

	void scale(double scale);

private:
	Eigen::MatrixXd quad_cost_full_;
	Eigen::MatrixXd quad_cost_;
	Eigen::MatrixXd quad_cost_inv_;

	Eigen::MatrixXd getDiffMatrix(int size, const double* diff_rule) const;

};

inline const Eigen::MatrixXd& SmoothnessCost::getQuadraticCostInverse() const
{
	return quad_cost_inv_;
}

inline const Eigen::MatrixXd& SmoothnessCost::getQuadraticCost() const
{
	return quad_cost_;
}

inline double SmoothnessCost::getCost(Eigen::MatrixXd::ColXpr joint_trajectory) const
{
	return joint_trajectory.dot(quad_cost_full_ * joint_trajectory);
}

inline double SmoothnessCost::getCost(Eigen::MatrixXd::ConstColXpr joint_trajectory) const
{
	return joint_trajectory.dot(quad_cost_full_ * joint_trajectory);
}

}

#endif
