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
    vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> >& contactViolationVector, vector<KDL::Vector>& contactPointVelVector,
    const vector<vector<KDL::Frame> >& segmentFrames) const
{
  vector<KDL::Vector> contactPointPosVector(contactViolationVector.size());
  for (int i = start; i <= end; ++i)
  {
    KDL::Vector position = segmentFrames[i][linkSegmentNumber_].p;
    KDL::Vector normal = segmentFrames[i][linkSegmentNumber_].M * KDL::Vector(0.0, 0.0, 1.0);
    normal.Normalize();

    KDL::Vector groundPosition;
    KDL::Vector groundNormal;
    GroundManager::getInstance().getNearestGroundPosition(position, groundPosition, groundNormal, true);

    KDL::Vector diff = position - groundPosition;
    double angle = acos(KDL::dot(normal, groundNormal));

    contactViolationVector[i] = Eigen::Vector4d(diff.x(), diff.y(), diff.z(), 10 * angle);
    contactPointPosVector[i] = position;
  }
  for (int i = 0; i < start; ++i)
    contactPointPosVector[i] = segmentFrames[i][linkSegmentNumber_].p;
  for (int i = end + 1; i < contactPointPosVector.size(); ++i)
    contactPointPosVector[i] = segmentFrames[i][linkSegmentNumber_].p;

  itomp_cio_planner::getVectorVelocities(start, end, discretization, contactPointPosVector, contactPointVelVector,
      KDL::Vector::Zero());
}

double ContactPoint::getDistanceToGround(int point, const std::vector<std::vector<KDL::Frame> >& segmentFrames) const
{
  KDL::Vector position;
  getPosition(point, position, segmentFrames);

  KDL::Vector groundPosition;
  KDL::Vector groundNormal;
  GroundManager::getInstance().getNearestGroundPosition(position, groundPosition, groundNormal);

  KDL::Vector diff = position - groundPosition;
  if (diff.z() < 0.0)
    diff.z(0.0);
  return diff.Norm();
}
}

