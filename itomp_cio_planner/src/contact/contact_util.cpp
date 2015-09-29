#include <itomp_cio_planner/contact/contact_util.h>

using namespace std;

namespace itomp_cio_planner
{

double getContactActiveValue(unsigned int contact, unsigned int contact_point,
                             const std::vector<ContactVariables>& contact_variables)
{
    double c = contact_variables[contact].getPointForce(contact_point).squaredNorm();

    return c;
}

}
