#ifndef TRAJECTORY_COST_HELPER_H_
#define TRAJECTORY_COST_HELPER_H_

#define ITOMP_TRAJECTORY_COST_DECL(C) \
class TrajectoryCost##C : public TrajectoryCost \
{\
	public:\
		TrajectoryCost##C(int index, std::string name, double weight,\
						  const NewEvalManager* evaluation_manager) : TrajectoryCost(index, name, weight)\
		{ \
			initialize(evaluation_manager); \
		} \
		virtual ~TrajectoryCost##C() {} \
		virtual void initialize(const NewEvalManager* evaluation_manager);\
		virtual bool evaluate(const NewEvalManager* evaluation_manager, \
								int point, double& cost) const;\
};

#define ITOMP_TRAJECTORY_COST_ADD(C) \
if (PlanningParameters::getInstance()->get##C##CostWeight() > 0.0) \
{ \
		cost_function_vector_.push_back( \
				boost::make_shared<TrajectoryCost##C >(index++, #C, \
						PlanningParameters::getInstance()->get##C##CostWeight(), \
						evaluation_manager)); \
		TIME_PROFILER_ADD_ENTRY(C) \
}

#define ITOMP_TRAJECTORY_COST_EMPTY_INIT_FUNC(C) \
void TrajectoryCost##C::initialize(const NewEvalManager* evaluation_manager) {}


#endif /* TRAJECTORY_COST_HELPER_H_ */
