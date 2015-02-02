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
									 std::vector<RigidBodyDynamics::Math::SpatialVector> *f_ext);

void updatePartialKinematicsAndDynamics(RigidBodyDynamics::Model &model,
										const RigidBodyDynamics::Math::VectorNd &Q,
										const RigidBodyDynamics::Math::VectorNd &QDot,
										const RigidBodyDynamics::Math::VectorNd &QDDot,
										RigidBodyDynamics::Math::VectorNd &Tau,
										std::vector<RigidBodyDynamics::Math::SpatialVector> *f_ext,
										const std::vector<unsigned int>& body_ids);

void updatePartialDynamics(RigidBodyDynamics::Model &model,
						   const RigidBodyDynamics::Math::VectorNd &Q,
						   const RigidBodyDynamics::Math::VectorNd &QDot,
						   const RigidBodyDynamics::Math::VectorNd &QDDot,
						   RigidBodyDynamics::Math::VectorNd &Tau,
						   std::vector<RigidBodyDynamics::Math::SpatialVector> *f_ext);

void UpdatePartialKinematics(RigidBodyDynamics::Model & model,
							 const RigidBodyDynamics::Math::VectorNd& Q,
							 const RigidBodyDynamics::Math::VectorNd& QDot,
							 const RigidBodyDynamics::Math::VectorNd& QDDot,
							 const std::vector<unsigned int>& body_ids);

void CalcFullJacobian(RigidBodyDynamics::Model & model,
					  const RigidBodyDynamics::Math::VectorNd & Q, unsigned int body_id,
					  const RigidBodyDynamics::Math::Vector3d & point_position,
					  RigidBodyDynamics::Math::MatrixNd & G, bool update_kinematics = true);

void CalcFullJacobianBasePosition(RigidBodyDynamics::Model & model,
								  const RigidBodyDynamics::Math::VectorNd & Q, unsigned int body_id,
								  const RigidBodyDynamics::Math::Vector3d & point_base_pos,
								  RigidBodyDynamics::Math::MatrixNd & G, bool update_kinematics = true);

void InverseDynamics2(RigidBodyDynamics::Model &model,
					  const RigidBodyDynamics::Math::VectorNd &Q,
					  const RigidBodyDynamics::Math::VectorNd &QDot,
					  const RigidBodyDynamics::Math::VectorNd &QDDot,
					  RigidBodyDynamics::Math::VectorNd &Tau,
					  std::vector<RigidBodyDynamics::Math::SpatialVector> *f_ext);

}

#endif /* RBDL_MODEL_UTIL_H_ */
