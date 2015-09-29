#include <itomp_cio_planner/itomp_planning_interface.h>

namespace itomp_cio_planner
{

ItompPlanningContext::ItompPlanningContext(const std::string &name, const std::string &group) :
	planning_interface::PlanningContext(name, group)
{

}

ItompPlanningContext::~ItompPlanningContext()
{

}

bool ItompPlanningContext::initialize(const robot_model::RobotModelConstPtr& model)
{
	itomp_planner_node_ = boost::make_shared<ItompPlannerNode>(model);
	return itomp_planner_node_->init();
}

bool ItompPlanningContext::solve(planning_interface::MotionPlanResponse &res)
{
	group_ = request_.group_name;
	return itomp_planner_node_->planTrajectory(planning_scene_, request_, res);
}
bool ItompPlanningContext::solve(planning_interface::MotionPlanDetailedResponse &res)
{
	// TODO:
	ROS_ERROR("Unsupported function");
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

}
