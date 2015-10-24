#ifndef BVH_WRITER_H_
#define BVH_WRITER_H_

#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/robot_model/robot_model.h>

namespace bvh_writer
{

void writeWalkingTrajectoryBVHFile(const robot_model::RobotModelPtr robot_model, const moveit_msgs::DisplayTrajectory& display_trajectory, const std::string& filename);

}

#endif

