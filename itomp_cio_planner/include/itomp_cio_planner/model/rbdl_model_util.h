#ifndef RBDL_MODEL_UTIL_H_
#define RBDL_MODEL_UTIL_H_

#include <itomp_cio_planner/common.h>
#include <rbdl/rbdl.h>

namespace itomp_cio_planner
{
void updateFullKinematicsAndDynamics(RigidBodyDynamics::Model &model,
									 const RigidBodyDynamics::Math::VectorNd &Q,
									 const RigidBodyDynamics::Math::VectorNd &QDot,
									 const RigidBodyDynamics::Math::VectorNd &QDDot,
									 RigidBodyDynamics::Math::VectorNd &Tau,
                                     const std::vector<RigidBodyDynamics::Math::SpatialVector> *f_ext,
                                     const std::vector<double> *joint_forces);

void updatePartialKinematicsAndDynamics(RigidBodyDynamics::Model &model,
										const RigidBodyDynamics::Math::VectorNd &Q,
										const RigidBodyDynamics::Math::VectorNd &QDot,
										const RigidBodyDynamics::Math::VectorNd &QDDot,
										RigidBodyDynamics::Math::VectorNd &Tau,
                                        const std::vector<RigidBodyDynamics::Math::SpatialVector> *f_ext,
                                        const std::vector<double> *joint_forces,
										const std::vector<unsigned int>& body_ids);

void updatePartialDynamics(RigidBodyDynamics::Model &model,
						   const RigidBodyDynamics::Math::VectorNd &Q,
						   const RigidBodyDynamics::Math::VectorNd &QDot,
						   const RigidBodyDynamics::Math::VectorNd &QDDot,
						   RigidBodyDynamics::Math::VectorNd &Tau,
                           const std::vector<RigidBodyDynamics::Math::SpatialVector> *f_ext,
                           const std::vector<double> *joint_forces);

void UpdatePartialKinematics(RigidBodyDynamics::Model & model,
							 const RigidBodyDynamics::Math::VectorNd& Q,
							 const RigidBodyDynamics::Math::VectorNd& QDot,
							 const RigidBodyDynamics::Math::VectorNd& QDDot,
							 const std::vector<unsigned int>& body_ids);

void CalcFullJacobian(RigidBodyDynamics::Model & model,
					  const RigidBodyDynamics::Math::VectorNd & Q, unsigned int body_id,
					  const RigidBodyDynamics::Math::Vector3d & point_position,
					  RigidBodyDynamics::Math::MatrixNd & G, bool update_kinematics = true);
}

#endif /* RBDL_MODEL_UTIL_H_ */
