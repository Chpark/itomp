#ifndef RBDL_URDF_READER_H_
#define RBDL_URDF_READER_H_

#include <itomp_cio_planner/common.h>
#include <rbdl/rbdl.h>

namespace itomp_cio_planner
{
bool ReadURDFModel (const std::string& xml_string, RigidBodyDynamics::Model* model, bool verbose = false);
}

#endif /* RBDL_URDF_READER_H_ */
