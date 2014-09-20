#ifndef GROUNDMANAGER_H_
#define GROUNDMANAGER_H_

#include <itomp_cio_planner/common.h>
#include <kdl/frames.hpp>
#include <moveit/planning_scene/planning_scene.h>

namespace itomp_cio_planner
{
class GroundManager: public Singleton<GroundManager>
{
public:
	GroundManager();
	virtual ~GroundManager();
	void initialize();

	void getNearestGroundPosition(const Eigen::Vector3d& position_in,
			const Eigen::Vector3d& orientation_in,
			Eigen::Vector3d& position_out, Eigen::Vector3d& orientation_out,
			Eigen::Vector3d& normal) const;

	void getNearestGroundPosition(const KDL::Vector& in, KDL::Vector& out,
			KDL::Vector& normal,
			const planning_scene::PlanningScenePtr& planning_scene) const;
	void getSafeGroundPosition(const KDL::Vector& in, KDL::Vector& out) const;

private:

};

}
;

#endif /* GROUNDMANAGER_H_ */
