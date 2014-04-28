#include <itomp_cio_planner/itomp_planning_interface.h>

namespace itomp_cio_planner
{

ItompPlanningContext::ItompPlanningContext(const std::string &name, const std::string &group)
: planning_interface::PlanningContext(name, group)
{

}

ItompPlanningContext::~ItompPlanningContext()
{

}

bool ItompPlanningContext::initialize(const robot_model::RobotModelConstPtr& model)
{
	itomp_planner_node_.reset(new ItompPlannerNode(model));
	return itomp_planner_node_->init();
}

bool ItompPlanningContext::solve(planning_interface::MotionPlanResponse &res)
{
	return itomp_planner_node_->planKinematicPath(req_, res);
}
bool ItompPlanningContext::solve(planning_interface::MotionPlanDetailedResponse &res)
{
	// TODO:
	return true;
}

void ItompPlanningContext::clear()
{

}
bool ItompPlanningContext::terminate()
{
	// TODO:
	return true;
}

void ItompPlanningContext::setPlanRequest(const planning_interface::MotionPlanRequest& req)
{
	req_ = req;
	group_ = req_.group_name;
}

}
