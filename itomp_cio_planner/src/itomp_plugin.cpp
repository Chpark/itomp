#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/profiler/profiler.h>
#include <class_loader/class_loader.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <itomp_cio_planner/itomp_planning_interface.h>

namespace itomp_cio_planner
{

class ITOMPPlannerManager: public planning_interface::PlannerManager
{
public:

	ITOMPPlannerManager() :
			planning_interface::PlannerManager()
	{
	}

	virtual bool initialize(const robot_model::RobotModelConstPtr& model, const std::string &ns)
	{
		context_.reset(new ItompPlanningContext("ITOMP", "right_arm"));
		return context_->initialize(model);
	}

	virtual bool canServiceRequest(const moveit_msgs::MotionPlanRequest &req) const
	{
		return true;
	}

	virtual std::string getDescription() const
	{
		return "ITOMP";
	}

	virtual void getPlanningAlgorithms(std::vector<std::string> &algs) const
	{
		algs.push_back("ITOMP");
	}

	virtual void setPlannerConfigurations(const planning_interface::PlannerConfigurationMap &pconfig)
	{
		// this call can add a few more configs than we pass in (adds defaults)
		//ompl_interface_->setPlannerConfigurations(pconfig);
		// so we read the configs instead of just setting pconfig
		//PlannerManager::setPlannerConfigurations(ompl_interface_->getPlannerConfigurations());
		PlannerManager::setPlannerConfigurations(pconfig);
	}

	virtual planning_interface::PlanningContextPtr getPlanningContext(
			const planning_scene::PlanningSceneConstPtr& planning_scene,
			const planning_interface::MotionPlanRequest &req, moveit_msgs::MoveItErrorCodes &error_code) const
	{
		//return ompl_interface_->getPlanningContext(planning_scene, req, error_code);
		context_->setPlanRequest(req);

		return context_;
	}

private:
	boost::shared_ptr<ItompPlanningContext> context_;
};

}

CLASS_LOADER_REGISTER_CLASS(itomp_cio_planner::ITOMPPlannerManager, planning_interface::PlannerManager);
