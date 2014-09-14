#ifndef RBDL_MODEL_UTIL_H_
#define RBDL_MODEL_UTIL_H_

#include <itomp_cio_planner/common.h>
#include <rbdl/rbdl.h>

namespace itomp_cio_planner
{
void UpdatePartialKinematics(RigidBodyDynamics::Model & model,
		const RigidBodyDynamics::Math::VectorNd& Q,
		const RigidBodyDynamics::Math::VectorNd& QDot,
		const RigidBodyDynamics::Math::VectorNd& QDDot,
		const std::vector<unsigned int>& body_ids);
}

#endif /* RBDL_MODEL_UTIL_H_ */
