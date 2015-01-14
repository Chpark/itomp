#ifndef PARAMETER_TRAJECTORY_H_
#define PARAMETER_TRAJECTORY_H_

#include <itomp_cio_planner/common.h>
#include <itomp_cio_planner/trajectory/trajectory.h>

namespace itomp_cio_planner
{
ITOMP_FORWARD_DECL(FullTrajectory);

class ParameterTrajectory: public Trajectory
{
public:
	// Construct a trajectory for a planning group
	ParameterTrajectory(const FullTrajectoryConstPtr& full_trajectory,
			const ItompPlanningGroupConstPtr& planning_group);
	virtual ~ParameterTrajectory();

	int getNumJoints() const;
	const std::vector<int>& getGroupToFullJointIndices() const;

protected:
	int num_joints_;
	std::vector<int> group_to_full_joint_indices;

	friend class TrajectoryFactory;
	friend class FullTrajectory;
};
ITOMP_DEFINE_SHARED_POINTERS(ParameterTrajectory);

///////////////////////// inline functions follow //////////////////////

inline int ParameterTrajectory::getNumJoints() const
{
	return num_joints_;
}

inline const std::vector<int>& ParameterTrajectory::getGroupToFullJointIndices() const
{
	return group_to_full_joint_indices;
}

}

#endif /* PARAMETER_TRAJECTORY_H_ */
