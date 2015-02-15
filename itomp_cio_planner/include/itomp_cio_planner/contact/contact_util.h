#ifndef CONTACT_UTIL_H_
#define CONTACT_UTIL_H_

#include <itomp_cio_planner/common.h>
#include <itomp_cio_planner/contact/contact_variables.h>
#include <itomp_cio_planner/model/itomp_planning_group.h>
#include <rbdl/Model.h>

namespace itomp_cio_planner
{

double getContactActiveValue(unsigned int contact, unsigned int contact_point,
                             const std::vector<ContactVariables>& contact_variables);


};

#endif /* CONTACT_UTIL_H_ */
