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
    int plane_index_;
};
class Plane
{
public:
    Plane(const Triangle& triangle);

    bool isTriangleIn(const Triangle& triangle) const;
    bool projectionZ(const Eigen::Vector3d& position_in, Eigen::Vector3d& position_out) const;

    std::set<int> triangle_indices_;
    Eigen::Vector3d normal_;
    double d_;
};

class GroundManager: public Singleton<GroundManager>
{
public:
	GroundManager();
	virtual ~GroundManager();
	void initialize(
		const planning_scene::PlanningSceneConstPtr& planning_scene);

	void getNearestContactPosition(const Eigen::Vector3d& position_in,
								  const Eigen::Vector3d& orientation_in,
								  Eigen::Vector3d& position_out, Eigen::Vector3d& orientation_out,
                                  Eigen::Vector3d& normal, bool include_ground = true, bool ignore_Z = false) const;

    void getNearestZPosition(const Eigen::Vector3d& position_in, Eigen::Vector3d& position_out, Eigen::Vector3d& normal) const;

private:
	void initializeContactSurfaces();

    bool getNearestMeshPosition(const Eigen::Vector3d& position_in,
                                Eigen::Vector3d& position_out, const Eigen::Vector3d& normal_in,
                                Eigen::Vector3d& normal, double current_min_distance, bool ignore_Z = false) const;
    void getNearestMeshZPosition(const Eigen::Vector3d& position_in, Eigen::Vector3d& position_out, Eigen::Vector3d& normal, double current_min_distance) const;

	planning_scene::PlanningSceneConstPtr planning_scene_;
	std::vector<Triangle> triangles_;
    std::vector<Plane> planes_;
};

}
;

#endif /* GROUNDMANAGER_H_ */
