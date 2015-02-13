#include <itomp_cio_planner/contact/contact_util.h>

using namespace std;

namespace itomp_cio_planner
{

double getContactActiveValue(unsigned int contact, unsigned int contact_point,
                             const std::vector<ContactVariables>& contact_variables,
                             const ItompPlanningGroupConstPtr& planning_group,
                             const RigidBodyDynamics::Model& model)
{
    const double K1 = 10.0;
    const double K2 = 3.0;
    static double MIN_CI_COST = 0.5 * std::tanh(-K2) + 0.5;

    int rbdl_body_id = planning_group->contact_points_[contact].getRBDLBodyId();
    const RigidBodyDynamics::Math::SpatialTransform& contact_body_transform = model.X_base[rbdl_body_id];
    Eigen::Vector3d z_dir = contact_body_transform.E.col(2);

    // TODO: change to const ref
    Eigen::Vector3d point_contact_force = contact_variables[contact].getPointForce(contact_point);
    if (contact >= 2)
        point_contact_force = Eigen::Vector3d::Zero();

    double f_norm = point_contact_force.dot(z_dir);

    // TODO: cone constraint
    if (f_norm < 0.0)
        f_norm = 0.0;

    double c = 0.5 * std::tanh(K1 * f_norm - K2) + 0.5 - MIN_CI_COST;

    return c;
}

}
