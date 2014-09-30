/*
 * contactPoint.cpp
 *
 *  Created on: Sep 11, 2013
 *      Author: chpark
 */

#include <itomp_cio_planner/model/itomp_robot_model.h>
#include <itomp_cio_planner/util/vector_util.h>
#include <itomp_cio_planner/contact/contact_point.h>
#include <itomp_cio_planner/contact/ground_manager.h>

using namespace std;

namespace itomp_cio_planner
{

ContactPoint::ContactPoint(const string& linkName, const ItompRobotModel* robot_model)
{
  linkName_ = linkName;
  linkSegmentNumber_ = robot_model->getForwardKinematicsSolver()->segmentNameToIndex(linkName);
}

ContactPoint::~ContactPoint()
{

}

void ContactPoint::getPosition(int point, KDL::Vector& position,
    const std::vector<std::vector<KDL::Frame> >& segmentFrames) const
{
  position = segmentFrames[point][linkSegmentNumber_].p;
}

void ContactPoint::getFrame(int point, KDL::Frame& frame,
    const std::vector<std::vector<KDL::Frame> >& segmentFrames) const
{
  frame = segmentFrames[point][linkSegmentNumber_];
}

void ContactPoint::updateContactViolationVector(int start, int end, double discretization,
    vector<Vector4d>& contactViolationVector, vector<KDL::Vector>& contactPointVelVector,
    const vector<vector<KDL::Frame> >& segmentFrames, const planning_scene::PlanningSceneConstPtr& planning_scene) const
{
  vector<KDL::Vector> contactPointPosVector(contactViolationVector.size());
  for (int i = start; i <= end; ++i)
  {
    KDL::Vector position = segmentFrames[i][linkSegmentNumber_].p;
    KDL::Vector normal = segmentFrames[i][linkSegmentNumber_].M * KDL::Vector(0.0, 0.0, 1.0);
    normal.Normalize();

    KDL::Vector groundPosition;
    KDL::Vector groundNormal;
    GroundManager::getInstance().getNearestGroundPosition(position, groundPosition, groundNormal, planning_scene);

    KDL::Vector diff = position - groundPosition;
    double angle = acos(KDL::dot(normal, groundNormal));

    contactViolationVector[i] = Vector4d(diff.x(), diff.y(), diff.z(), angle);
    contactPointPosVector[i] = position;
  }
  //for (int i = 0; i < start; ++i)
//  if (start == 1)
  //  contactPointPosVector[0] = segmentFrames[1][linkSegmentNumber_].p;
  //else
    contactPointPosVector[start - 1] = segmentFrames[start - 1][linkSegmentNumber_].p;
  //for (int i = end + 1; i < contactPointPosVector.size(); ++i)
//  if (end == contactPointPosVector.size() - 2)
  //  contactPointPosVector[end + 1] = segmentFrames[end][linkSegmentNumber_].p;
  //else
    contactPointPosVector[end + 1] = segmentFrames[end + 1][linkSegmentNumber_].p;

  itomp_cio_planner::getVectorVelocities(start, end, discretization, contactPointPosVector, contactPointVelVector,
      KDL::Vector::Zero());
}

double ContactPoint::getDistanceToGround(int point, const std::vector<std::vector<KDL::Frame> >& segmentFrames, const planning_scene::PlanningSceneConstPtr& planning_scene) const
{
  KDL::Vector position;
  getPosition(point, position, segmentFrames);

  KDL::Vector groundPosition;
  KDL::Vector groundNormal;
  GroundManager::getInstance().getNearestGroundPosition(position, groundPosition, groundNormal, planning_scene);

  KDL::Vector diff = position - groundPosition;
  if (diff.z() < 0.0)
    diff.z(0.0);
  return diff.Norm();
}
}

