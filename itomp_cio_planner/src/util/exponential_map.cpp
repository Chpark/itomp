#include <itomp_cio_planner/util/exponential_map.h>
#include <boost/math/special_functions/sinc.hpp>

namespace itomp_cio_planner
{
namespace exponential_map
{

Eigen::Vector3d RotationToExponentialMap(const Eigen::Matrix3d& matrix)
{
	return QuaternionToExponentialMap(Eigen::Quaterniond(matrix));
}

Eigen::Matrix3d ExponentialMapToRotation(
		const Eigen::Vector3d& exponential_rotation)
{
	return ExponentialMapToQuaternion(exponential_rotation).toRotationMatrix();
}

Eigen::Vector3d QuaternionToExponentialMap(
		const Eigen::Quaterniond& quaternion)
{
	Eigen::Vector3d vec = quaternion.vec();
	if (vec.norm() < 1e-7)
		return Eigen::Vector3d::Zero();

	double theta = 2.0 * std::acos(quaternion.w());
	vec.normalize();
	return theta * vec;
}

Eigen::Quaterniond ExponentialMapToQuaternion(
		const Eigen::Vector3d& exponential_rotation)
{
	double angle = 0.5 * exponential_rotation.norm();
	Eigen::Quaterniond quaternion;
	quaternion.w() = std::cos(angle);
	quaternion.vec() = 0.5 * boost::math::sinc_pi(angle) * exponential_rotation;
	return quaternion;
}

}
}
