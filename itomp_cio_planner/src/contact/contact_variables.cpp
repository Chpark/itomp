#include <itomp_cio_planner/contact/contact_variables.h>

using namespace std;

namespace itomp_cio_planner
{

void ContactVariables::setPointForce(int point_index, const Eigen::Vector3d& point_force)
{
    double scale = 60 * 9.8 * 10;
    //serialized_forces_.block(3 * point_index, 0, 3, 1) = point_force / scale;

    double magnitude = point_force.norm() / scale;
    Eigen::Quaterniond q;
    q.setFromTwoVectors(Eigen::Vector3d(0, 0, 1), point_force);
    serialized_forces_.block(4 * point_index, 0, 3, 1) = exponential_map::QuaternionToExponentialMap(q);
    serialized_forces_(4 * point_index + 3) = magnitude;
}
Eigen::Vector3d ContactVariables::getPointForce(int point_index) const
{
    //Eigen::Vector3d force = serialized_forces_.block(3 * point_index, 0, 3, 1);
    double scale = 60 * 9.8 * 10;
    //force *= scale;

    double magnitude = serialized_forces_(4 * point_index + 3);
    Eigen::Vector3d orientation = serialized_forces_.block(4 * point_index, 0, 3, 1);
    Eigen::Vector3d force = exponential_map::ExponentialMapToRotation(orientation) * Eigen::Vector3d(0, 0, 1);
    force *= magnitude * scale;

    return force;
}

}
