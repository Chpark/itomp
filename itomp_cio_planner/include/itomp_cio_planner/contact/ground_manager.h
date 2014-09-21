#ifndef GROUNDMANAGER_H_
#define GROUNDMANAGER_H_

#include <itomp_cio_planner/common.h>
#include <kdl/frames.hpp>
#include <moveit/planning_scene/planning_scene.h>

namespace itomp_cio_planner
{
class Triangle
{
public:
	Eigen::Vector3d points_[3];
	Eigen::Vector3d normal_;
};

class GroundManager: public Singleton<GroundManager>
{
public:
	GroundManager();
	virtual ~GroundManager();
	void initialize(
			const planning_scene::PlanningSceneConstPtr& planning_scene);

	void getNearestGroundPosition(const Eigen::Vector3d& position_in,
			const Eigen::Vector3d& orientation_in,
			Eigen::Vector3d& position_out, Eigen::Vector3d& orientation_out,
			Eigen::Vector3d& normal) const;

	bool getNearestMeshPosition(const Eigen::Vector3d& position_in,
			Eigen::Vector3d& position_out, const Eigen::Vector3d& normal_in,
			Eigen::Vector3d& normal, double current_min_distance) const;

private:
	void initializeStaticScene();
	planning_scene::PlanningSceneConstPtr planning_scene_;
	std::vector<Triangle> triangles_;

};

}
;

#endif /* GROUNDMANAGER_H_ */
