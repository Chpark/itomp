/*
 * groundManager.cpp
 *
 *  Created on: Sep 11, 2013
 *      Author: chpark
 */
#include <itomp_cio_planner/contact/ground_manager.h>
#include <itomp_cio_planner/util/planning_parameters.h>
#include <itomp_cio_planner/util/point_to_triangle_projection.h>
#include <limits>

namespace itomp_cio_planner
{

GroundManager::GroundManager()
{
}

GroundManager::~GroundManager()
{
}

void GroundManager::initialize()
{
}

void GroundManager::getNearestGroundPosition(const Eigen::Vector3d& in, Eigen::Vector3d& out, Eigen::Vector3d& normal) const
{
	out = Eigen::Vector3d(in(0), in(1), 0);

	if (in(1) > 5.5 && in(1) < 6.5)
		out(2) = 0.05;

	normal = Eigen::Vector3d(0, 0, 1);
}

double interpolateSqrt(double x, double x1, double x2, double y1, double y2)
{
  //double y = (y2 - y1) * sqrt((x - x1) / (x2 - x1)) + y1;
  double y = (y2 - y1) * (x - x1) / (x2 - x1) + y1;
  return y;
}

void GroundManager::getNearestGroundPosition(const KDL::Vector& in, KDL::Vector& out, KDL::Vector& normal,
    const planning_scene::PlanningScenePtr& planning_scene) const
{
  const double FOOT_FRONT = 0.2;
  const double FOOT_REAR = 0.2; //0.05;
  const double MARGIN = 0.1;

  normal = KDL::Vector(0, 0, 1);

  double height = 0.0;

  const int NUM_TERRAINS = 5;

  double x[] =
  { -10000, -1.9, -1.5, -1.1, -0.7, 10000 };
  double z[] =
  { -0.6, -0.45, -0.3, -0.15, 0 };

  for (int i = 0; i < NUM_TERRAINS; ++i)
  {
    if (in.y() >= x[i] && in.y() <= x[i + 1])
    {
      height = z[i];

//      if (exact)
//        break;

      if (i != 0 && z[i - 1] > z[i])
      {
        if (in.y() - x[i] <= FOOT_REAR)
          height = z[i - 1];
        else if (in.y() - x[i] <= FOOT_REAR + MARGIN)
        {
          height = interpolateSqrt(in.y(), x[i] + FOOT_REAR, x[i] + FOOT_REAR + MARGIN, z[i - 1], z[i]);
        }
      }
      else if (z[i + 1] > z[i])
      {
        if (x[i + 1] - in.y() <= FOOT_FRONT)
          height = z[i + 1];
        else if (x[i + 1] - in.y() <= FOOT_FRONT + MARGIN)
        {
          height = interpolateSqrt(in.y(), x[i + 1] - FOOT_FRONT - MARGIN, x[i + 1] - FOOT_FRONT, z[i], z[i + 1]);
        }
      }
    }
  }

  out = KDL::Vector(in.x(), in.y(), height);

  /*
   double current_min_distance = std::numeric_limits<double>::max();
   Eigen::Vector3d in_eigen(in.x(), in.y(), in.z());
   const collision_detection::WorldConstPtr& world = planning_scene->getWorld();
   std::vector<std::string> object_ids = world->getObjectIds();
   for (int i = 0; i < object_ids.size(); ++i)
   {
   collision_detection::World::ObjectConstPtr obj = world->getObject(object_ids[i]);
   for (int j = 0; j < obj->shapes_.size(); ++j)
   {
   shapes::ShapeConstPtr shape = obj->shapes_[j];
   const shapes::Mesh* mesh = dynamic_cast<const shapes::Mesh*>(shape.get());
   if (mesh == NULL)
   continue;
   Eigen::Affine3d& transform = obj->shape_poses_[j];
   for(int k=0; k<mesh->triangle_count; ++k)
   {
   int triangle_vertex1 = mesh->triangles[3*k];
   int triangle_vertex2 = mesh->triangles[3*k+1];
   int triangle_vertex3 = mesh->triangles[3*k+2];
   Eigen::Vector3d position1(mesh->vertices[3*triangle_vertex1],mesh->vertices[3*triangle_vertex1+1],mesh->vertices[3*triangle_vertex1+2]);
   Eigen::Vector3d position2(mesh->vertices[3*triangle_vertex2],mesh->vertices[3*triangle_vertex2+1],mesh->vertices[3*triangle_vertex2+2]);
   Eigen::Vector3d position3(mesh->vertices[3*triangle_vertex3],mesh->vertices[3*triangle_vertex3+1],mesh->vertices[3*triangle_vertex3+2]);
   position1 = transform * position1;
   position2 = transform * position2;
   position3 = transform * position3;

   Eigen::Vector3d projection = ProjPoint2Triangle(position1, position2, position3, in_eigen);
   double distance = (in_eigen - projection).norm();
   if(distance < current_min_distance)
   {
   current_min_distance = distance;
   Eigen::Vector3d normal_eigen(mesh->triangle_normals[3*k], mesh->triangle_normals[3*k+1], mesh->triangle_normals[3*k+2]);
   normal_eigen = transform.rotation() * normal_eigen;
   normal.x(normal_eigen.x());
   normal.y(normal_eigen.y());
   normal.z(normal_eigen.z());
   out.x(projection.x());
   out.y(projection.y());
   out.z(projection.z());
   }
   }
   }

   }
   */
}

void GroundManager::getSafeGroundPosition(const KDL::Vector& in, KDL::Vector& out) const
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
        if (i != 0 && z[i - 1] > z[i] && in.x() < x[i] + FOOT_REAR + MARGIN)
        {
          safeX = x[i] - FOOT_FRONT;
          height = z[i - 1];
        }
      }
    }
  }
  else if (PlanningParameters::getInstance()->getTemporaryVariable(2) == 2.0)
  {
    if (-1.60 - FOOT_FRONT - MARGIN < in.x() && in.x() < -1.50 + FOOT_REAR + MARGIN)
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

