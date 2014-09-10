#ifndef TRAJECTORY_COST_HELPER_H_
#define TRAJECTORY_COST_HELPER_H_

#define ITOMP_TRAJECTORY_COST_DECL(C) \
class TrajectoryCost##C : public TrajectoryCost \
{\
	public:\
		TrajectoryCost##C(int index, std::string name, double weight) : TrajectoryCost(index, name, weight) {} \
		virtual ~TrajectoryCost##C() {} \
		virtual bool evaluate(const NewEvalManager* evaluation_manager, \
								const FullTrajectoryConstPtr& trajectory, int point, double& cost) const;\
};

#define ITOMP_TRAJECTORY_COST_ADD(C) \
if (PlanningParameters::getInstance()->get##C##CostWeight() > 0.0) \
		cost_function_vector_.push_back( \
				boost::make_shared<TrajectoryCost##C >(index++, #C, \
						PlanningParameters::getInstance()->get##C##CostWeight())); \


#endif /* TRAJECTORY_COST_HELPER_H_ */
