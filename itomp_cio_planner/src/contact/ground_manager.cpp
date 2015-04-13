/*
 * groundManager.cpp
 *
 *  Created on: Sep 11, 2013
 *      Author: chpark
 */
#include <itomp_cio_planner/contact/ground_manager.h>
#include <itomp_cio_planner/util/planning_parameters.h>
#include <itomp_cio_planner/util/point_to_triangle_projection.h>
#include <itomp_cio_planner/util/exponential_map.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shapes.h>
#include <limits>

namespace itomp_cio_planner
{

GroundManager::GroundManager()
{
}

GroundManager::~GroundManager()
{
}

void GroundManager::initialize(
	const planning_scene::PlanningSceneConstPtr& planning_scene)
{
	planning_scene_ = planning_scene;
    initializeContactSurfaces();
}

void GroundManager::getNearestContactPosition(const Eigen::Vector3d& position_in,
		const Eigen::Vector3d& orientation_in, Eigen::Vector3d& position_out,
        Eigen::Vector3d& orientation_out, Eigen::Vector3d& normal, bool include_ground, bool ignore_Z) const
{
    include_ground = PlanningParameters::getInstance()->getUseDefaultContactGround();

    double min_dist = include_ground ? (position_in(2) - 0) : std::numeric_limits<double>::max();

    Eigen::Matrix3d orientation_in_mat = exponential_map::ExponentialMapToRotation(orientation_in);
    Eigen::Vector3d x_axis = orientation_in_mat.col(0);
    Eigen::Vector3d y_axis = orientation_in_mat.col(1);
    Eigen::Vector3d normal_in = orientation_in_mat.col(2);

    Eigen::Vector3d temp_position_out;

    if (include_ground)
    {
        // ground

        temp_position_out = Eigen::Vector3d(position_in(0), position_in(1), 0.0);
        normal = Eigen::Vector3d(0, 0, 1);
        temp_position_out(2) = 0.0;
    }

    getNearestMeshPosition(position_in, temp_position_out, normal_in, normal, min_dist, ignore_Z);

	Eigen::Vector3d proj_x_axis = x_axis - x_axis.dot(normal) * normal;
	Eigen::Vector3d proj_y_axis = y_axis - y_axis.dot(normal) * normal;
	if (proj_x_axis.norm() > proj_y_axis.norm())
	{
		proj_x_axis.normalize();
		proj_y_axis = normal.cross(proj_x_axis);
	}
	else
	{
		proj_y_axis.normalize();
		proj_x_axis = proj_y_axis.cross(normal);
	}
	Eigen::Matrix3d orientation_out_mat;
	orientation_out_mat.col(0) = proj_x_axis;
	orientation_out_mat.col(1) = proj_y_axis;
	orientation_out_mat.col(2) = normal;
    orientation_out = exponential_map::RotationToExponentialMap(orientation_out_mat);
    //orientation_out.normalize();

    position_out = temp_position_out;
}

bool GroundManager::getNearestMeshPosition(const Eigen::Vector3d& position_in,
		Eigen::Vector3d& position_out, const Eigen::Vector3d& normal_in, Eigen::Vector3d& normal,
        double current_min_distance, bool ignore_Z) const
{
	bool updated = false;

	for (int i = 0; i < triangles_.size(); ++i)
	{
		const Triangle& triangle = triangles_[i];

		Eigen::Vector3d projection = ProjPoint2Triangle(triangle.points_[0], triangle.points_[1],
									 triangle.points_[2], position_in);       

        double distance = (position_in - projection).norm();
        if (ignore_Z)
        {
            Eigen::Vector3d diff = position_in - projection;
            diff(2) = 0.0;
            distance = diff.norm();
        }

		if (distance < current_min_distance)
		{
			current_min_distance = distance;
			normal = triangle.normal_;
			position_out = projection;

			updated = true;
		}
	}

	return updated;
}

void GroundManager::initializeContactSurfaces()
{
	triangles_.clear();

    std::string contact_model = PlanningParameters::getInstance()->getContactModel();
    if (contact_model == "")
        return;

    const std::vector<double>& contact_model_position = PlanningParameters::getInstance()->getContactModelPosition();
    double contact_model_scale = PlanningParameters::getInstance()->getContactModelScale();
    Eigen::Vector3d scale(contact_model_scale, contact_model_scale, contact_model_scale);
    Eigen::Vector3d translation(contact_model_position[0], contact_model_position[1], contact_model_position[2]);

    shapes::Mesh* mesh = shapes::createMeshFromResource(contact_model, scale);
    if (mesh == NULL)
        return;

    for (int k = 0; k < mesh->triangle_count; ++k)
    {
        int triangle_vertex1 = mesh->triangles[3 * k];
        int triangle_vertex2 = mesh->triangles[3 * k + 1];
        int triangle_vertex3 = mesh->triangles[3 * k + 2];
        Eigen::Vector3d position1(mesh->vertices[3 * triangle_vertex1], mesh->vertices[3 * triangle_vertex1 + 1], mesh->vertices[3 * triangle_vertex1 + 2]);
        Eigen::Vector3d position2(mesh->vertices[3 * triangle_vertex2], mesh->vertices[3 * triangle_vertex2 + 1], mesh->vertices[3 * triangle_vertex2 + 2]);
        Eigen::Vector3d position3(mesh->vertices[3 * triangle_vertex3], mesh->vertices[3 * triangle_vertex3 + 1], mesh->vertices[3 * triangle_vertex3 + 2]);

        Eigen::Vector3d p0 = (position2 - position1);
        Eigen::Vector3d p1 = (position3 - position1);
        p0.normalize();
        p1.normalize();
        Eigen::Vector3d normal = p0.cross(p1);
        if (normal.norm() < ITOMP_EPS)
            continue;
        normal.normalize();

        Triangle tri;
        tri.points_[0] = position1 + translation;
        tri.points_[1] = position2 + translation;
        tri.points_[2] = position3 + translation;
        tri.normal_ = normal;

        triangles_.push_back(tri);
    }
}

}

