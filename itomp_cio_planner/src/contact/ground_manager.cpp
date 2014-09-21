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
}

void GroundManager::getNearestGroundPosition(const Eigen::Vector3d& position_in,
		const Eigen::Vector3d& orientation_in, Eigen::Vector3d& position_out,
		Eigen::Vector3d& orientation_out, Eigen::Vector3d& normal) const
{
	double min_dist = position_in(2) - (-0.227936);

	// ground
	position_out = Eigen::Vector3d(position_in(0), position_in(1), -0.227936);
	normal = Eigen::Vector3d(0, 0, 1);

	getNearestMeshPosition(position_in, position_out, normal, min_dist);

	Eigen::Matrix3d orientation_in_mat = exponential_map::ExponentialMapToRotation(orientation_in);
	Eigen::Vector3d x_axis = orientation_in_mat.col(0);
	Eigen::Vector3d y_axis = orientation_in_mat.col(1);

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
}

double interpolateSqrt(double x, double x1, double x2, double y1, double y2)
{
	//double y = (y2 - y1) * sqrt((x - x1) / (x2 - x1)) + y1;
	double y = (y2 - y1) * (x - x1) / (x2 - x1) + y1;
	return y;
}

bool GroundManager::getNearestMeshPosition(const Eigen::Vector3d& position_in,
		Eigen::Vector3d& position_out, Eigen::Vector3d& normal, double current_min_distance) const
{
	bool updated = false;

	const collision_detection::WorldConstPtr& world =
			planning_scene_->getWorld();
	std::vector<std::string> object_ids = world->getObjectIds();
	for (int i = 0; i < object_ids.size(); ++i)
	{
		collision_detection::World::ObjectConstPtr obj = world->getObject(
				object_ids[i]);
		for (int j = 0; j < obj->shapes_.size(); ++j)
		{
			shapes::ShapeConstPtr shape = obj->shapes_[j];
			const shapes::Mesh* mesh =
					dynamic_cast<const shapes::Mesh*>(shape.get());
			if (mesh == NULL)
				continue;
			Eigen::Affine3d& transform = obj->shape_poses_[j];
			for (int k = 0; k < mesh->triangle_count; ++k)
			{
				int triangle_vertex1 = mesh->triangles[3 * k];
				int triangle_vertex2 = mesh->triangles[3 * k + 1];
				int triangle_vertex3 = mesh->triangles[3 * k + 2];
				Eigen::Vector3d position1(mesh->vertices[3 * triangle_vertex1],
						mesh->vertices[3 * triangle_vertex1 + 1],
						mesh->vertices[3 * triangle_vertex1 + 2]);
				Eigen::Vector3d position2(mesh->vertices[3 * triangle_vertex2],
						mesh->vertices[3 * triangle_vertex2 + 1],
						mesh->vertices[3 * triangle_vertex2 + 2]);
				Eigen::Vector3d position3(mesh->vertices[3 * triangle_vertex3],
						mesh->vertices[3 * triangle_vertex3 + 1],
						mesh->vertices[3 * triangle_vertex3 + 2]);
				position1 = transform * position1;
				position2 = transform * position2;
				position3 = transform * position3;

				Eigen::Vector3d projection = ProjPoint2Triangle(position1,
						position2, position3, position_in);
				double distance = (position_in - projection).norm();
				if (distance < current_min_distance)
				{
					current_min_distance = distance;
					normal = Eigen::Vector3d(mesh->triangle_normals[3 * k],
							mesh->triangle_normals[3 * k + 1],
							mesh->triangle_normals[3 * k + 2]);
					normal = transform.rotation() * normal;
					position_out = projection;

					updated = true;
				}
			}
		}
	}

	return updated;
}

void GroundManager::getSafeGroundPosition(const KDL::Vector& in,
		KDL::Vector& out) const
{
	const double FOOT_FRONT = 0.2;
	const double FOOT_REAR = 0.05;
	const double MARGIN = 0.1;

	double safeX = in.x();
	double height = 0.0;
	if (PlanningParameters::getInstance()->getTemporaryVariable(2) == 1.0)
	{
		const int NUM_TERRAINS = 7;

		double x[] =
		{ -50.0, -1.0, -0.6, -0.2, 0.2, 0.6, 1.0, 50.0 };
		double z[] =
		{ 0.0, 0.15, 0.3, 0.45, 0.3, 0.15, 0.0, 0.0 };

		for (int i = 0; i < NUM_TERRAINS; ++i)
		{
			if (in.x() >= x[i] && in.x() <= x[i + 1])
			{
				height = z[i];

				if (z[i] < z[i + 1] && x[i + 1] - FOOT_FRONT - MARGIN < in.x())
					safeX = x[i + 1] - FOOT_FRONT - MARGIN;
				if (i != 0 && z[i - 1] < z[i] && in.x() < x[i] + FOOT_REAR)
				{
					safeX = x[i] - FOOT_FRONT - MARGIN;
					height = z[i - 1];
				}

				if (z[i] > z[i + 1] && x[i + 1] - FOOT_FRONT < in.x())
					safeX = x[i + 1] - FOOT_FRONT;
				if (i != 0 && z[i - 1] > z[i]
						&& in.x() < x[i] + FOOT_REAR + MARGIN)
				{
					safeX = x[i] - FOOT_FRONT;
					height = z[i - 1];
				}
			}
		}
	}
	else if (PlanningParameters::getInstance()->getTemporaryVariable(2) == 2.0)
	{
		if (-1.60 - FOOT_FRONT - MARGIN < in.x()
				&& in.x() < -1.50 + FOOT_REAR + MARGIN)
		{
			safeX = -1.60 - FOOT_FRONT - MARGIN;
		}
	}
	else if (PlanningParameters::getInstance()->getTemporaryVariable(2) == 3.0)
	{
		height = 0.0;
	}

	out = KDL::Vector(safeX, in.y(), height);
}

}

