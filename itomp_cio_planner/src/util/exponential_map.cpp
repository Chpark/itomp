#include <itomp_cio_planner/util/exponential_map.h>
#include <boost/math/special_functions/sinc.hpp>

namespace itomp_cio_planner
{
namespace exponential_map
{

Eigen::Vector3d RotationToExponentialMap(const Eigen::Matrix3d& matrix, const Eigen::Vector3d* close_to)
{
    if (close_to == NULL)
        return QuaternionToExponentialMap(Eigen::Quaterniond(matrix));
    else
    {
        Eigen::Quaterniond q0 = Eigen::Quaterniond(matrix);
        Eigen::Vector3d exp_map0 = QuaternionToExponentialMap(q0);

        Eigen::Quaterniond q1 = q0;
        q1.coeffs() = -q0.coeffs();
        Eigen::Vector3d exp_map1 = QuaternionToExponentialMap(q1);

        if ((exp_map0 - *close_to).norm() <= (exp_map1 - *close_to).norm())
            return exp_map0;
        else
            return exp_map1;
    }
}

Eigen::Matrix3d ExponentialMapToRotation(const Eigen::Vector3d& exponential_rotation)
{
	return ExponentialMapToQuaternion(exponential_rotation).toRotationMatrix();
}

Eigen::Vector3d QuaternionToExponentialMap(const Eigen::Quaterniond& quaternion)
{
	Eigen::Vector3d vec = quaternion.vec();
    if (vec.norm() < ITOMP_EPS)
		return Eigen::Vector3d::Zero();

	double theta = 2.0 * std::acos(quaternion.w());
	vec.normalize();
	return theta * vec;
}

Eigen::Quaterniond ExponentialMapToQuaternion(const Eigen::Vector3d& exponential_rotation)
{
	double angle = 0.5 * exponential_rotation.norm();
	Eigen::Quaterniond quaternion;
	quaternion.w() = std::cos(angle);
	quaternion.vec() = 0.5 * boost::math::sinc_pi(angle) * exponential_rotation;
	return quaternion;
}

}
}
