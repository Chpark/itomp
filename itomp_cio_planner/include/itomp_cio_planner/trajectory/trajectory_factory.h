#ifndef TRAJECTORY_FACTORY_H_
#define TRAJECTORY_FACTORY_H_

#include <itomp_cio_planner/common.h>
#include <itomp_cio_planner/util/singleton.h>
#include <itomp_cio_planner/trajectory/trajectory.h>
#include <itomp_cio_planner/trajectory/full_trajectory.h>
#include <itomp_cio_planner/trajectory/parameter_trajectory.h>
#include <itomp_cio_planner/trajectory/itomp_trajectory.h>

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

	FullTrajectory* CreateFullTrajectory(
		const ItompRobotModelConstPtr& robot_model, double duration,
		double discretization, double keyframe_interval);
	ParameterTrajectory* CreateParameterTrajectory(
		const FullTrajectoryConstPtr& full_trajectory,
		const ItompPlanningGroupConstPtr& planning_group);

    ItompTrajectory* CreateItompTrajectory(const ItompRobotModelConstPtr& robot_model, double duration,
                                           double discretization, double keyframe_interval);

protected:
	TRAJECTORY_FACTORY_TYPE type_;

    void computeTrajectoryParameters(unsigned int& num_points, unsigned int& num_keyframes, unsigned int& num_keyframe_interval_points,
                                     double& duration, double& keyframe_interval, double discretization) const;
};
}

#endif /* TRAJECTORY_FACTORY_H_ */
