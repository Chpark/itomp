#ifndef BVH_WRITER_H_
#define BVH_WRITER_H_

#include <moveit_msgs/DisplayTrajectory.h>

namespace bvh_writer
{

void writeRocketboxTrajectoryBVHFile(moveit_msgs::DisplayTrajectory& display_trajectory, const std::string& filename);

}

#endif

