#include <itomp_cio_planner/util/exponential_map.h>
#include <boost/math/special_functions/sinc.hpp>

namespace itomp_cio_planner
{
namespace exponential_map
{

Eigen::Vector3d RotationToExponentialMap(const Eigen::Matrix3d& matrix)
{
	return AngleAxisToExponentialMap(Eigen::AngleAxisd(matrix));
}

Eigen::Matrix3d ExponentialMapToRotation(
		const Eigen::Vector3d& exponential_rotation)
{
	return ExponentialMapToAngleAxis(exponential_rotation).toRotationMatrix();
}

Eigen::Vector3d AngleAxisToExponentialMap(const Eigen::AngleAxisd& angle_axis)
{
	return angle_axis.axis() * 2.0 * angle_axis.angle();
}

Eigen::AngleAxisd ExponentialMapToAngleAxis(
		const Eigen::Vector3d& exponential_rotation)
{
	double angle = 0.5 * exponential_rotation.norm();
	Eigen::Vector3d axis = exponential_rotation.normalized();

	return Eigen::AngleAxisd(angle, axis);
}

}
}
