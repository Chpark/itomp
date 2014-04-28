#include <itomp_cio_planner/optimization/rollout.h>

namespace itomp_cio_planner
{
double Rollout::getCost()
{
	double cost = state_costs_.sum();
	int num_dim = control_costs_.size();
	for (int d = 0; d < num_dim; ++d)
		cost += control_costs_[d].sum();
	return cost;
}
}
