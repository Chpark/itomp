#ifndef EXPONENTIAL_MAP_H_
#define EXPONENTIAL_MAP_H_

#include <itomp_cio_planner/common.h>

namespace itomp_cio_planner
{
namespace exponential_map
{
Eigen::Vector3d RotationToExponentialMap(const Eigen::Matrix3d& matrix);
Eigen::Matrix3d ExponentialMapToRotation(
	const Eigen::Vector3d& exponential_rotation);

Eigen::Vector3d QuaternionToExponentialMap(const Eigen::Quaterniond& quaternion);
Eigen::Quaterniond ExponentialMapToQuaternion(
	const Eigen::Vector3d& exponential_rotation);
}
}

#endif /* EXPONENTIAL_MAP_H_ */
