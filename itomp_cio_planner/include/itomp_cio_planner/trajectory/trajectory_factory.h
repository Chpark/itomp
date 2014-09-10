#ifndef TRAJECTORY_FACTORY_H_
#define TRAJECTORY_FACTORY_H_

#include <itomp_cio_planner/common.h>
#include <itomp_cio_planner/util/singleton.h>
#include <itomp_cio_planner/trajectory/trajectory.h>
#include <itomp_cio_planner/trajectory/full_trajectory.h>
#include <itomp_cio_planner/trajectory/parameter_trajectory.h>

namespace itomp_cio_planner
{

class TrajectoryFactory: public Singleton<TrajectoryFactory>
{
public:
	enum TRAJECTORY_FACTORY_TYPE
	{
		TRAJECTORY_CIO = 0,
		TRAJECTORY_STOMP,
		TRAJECTORY_NUM,
		TRAJECTORY_UNDEFINED = TRAJECTORY_NUM
	};
	TrajectoryFactory();
	virtual ~TrajectoryFactory();

	void initialize(TRAJECTORY_FACTORY_TYPE type);

	FullTrajectoryPtr CreateFullTrajectory(
			const ItompRobotModelConstPtr& robot_model, double duration,
			double discretization, double keyframe_interval);
	ParameterTrajectoryPtr CreateParameterTrajectory(
			const FullTrajectoryConstPtr& full_trajectory,
			const ItompPlanningGroupConstPtr& planning_group);

protected:
	TRAJECTORY_FACTORY_TYPE type_;
};
}

#endif /* TRAJECTORY_FACTORY_H_ */
