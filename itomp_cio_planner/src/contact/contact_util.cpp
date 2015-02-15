#include <itomp_cio_planner/contact/contact_util.h>

using namespace std;

namespace itomp_cio_planner
{

double getContactActiveValue(unsigned int contact, unsigned int contact_point,
                             const std::vector<ContactVariables>& contact_variables)
{
    /*
    const double K1 = 10.0;
    const double K2 = 3.0;
    static double MIN_CI_COST = 0.5 * std::tanh(-K2) + 0.5;

    const Eigen::Matrix3d& orientation = exponential_map::ExponentialMapToRotation(contact_variables[contact].projected_orientation_);
    Eigen::Vector3d contact_normal = orientation.block(0, 2, 3, 1);

    Eigen::Vector3d point_contact_force = contact_variables[contact].getPointForce(contact_point);

    double f_norm = point_contact_force.dot(contact_normal);
    f_norm = std::abs(f_norm);

    double c = 0.5 * std::tanh(K1 * f_norm - K2) + 0.5 - MIN_CI_COST;
    */

    double c = contact_variables[contact].getPointForce(contact_point).squaredNorm();

    return c;
}

}
