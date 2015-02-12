#include <itomp_cio_planner/model/rbdl_model_util.h>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;
namespace itomp_cio_planner
{

void updateFullKinematicsAndDynamics(RigidBodyDynamics::Model &model,
									 const RigidBodyDynamics::Math::VectorNd &Q,
									 const RigidBodyDynamics::Math::VectorNd &QDot,
									 const RigidBodyDynamics::Math::VectorNd &QDDot,
									 RigidBodyDynamics::Math::VectorNd &Tau,
                                     const std::vector<RigidBodyDynamics::Math::SpatialVector> *f_ext)
{
    SpatialVector spatial_gravity(0., 0., 0., model.gravity[0], model.gravity[1], model.gravity[2]);

	unsigned int i;

	// Reset the velocity of the root body
	model.v[0].setZero();
	model.a[0] = spatial_gravity * -1.;

	for (i = 1; i < model.mBodies.size(); i++)
	{
		unsigned int q_index = model.mJoints[i].q_index;
		SpatialTransform X_J;
		SpatialVector v_J;
		SpatialVector c_J;
		unsigned int lambda = model.lambda[i];

		jcalc(model, i, X_J, v_J, c_J, Q, QDot);

		model.X_lambda[i] = X_J * model.X_T[i];

		if (lambda == 0)
		{
			model.X_base[i] = model.X_lambda[i];
			model.v[i] = v_J;
			model.a[i] = model.X_base[i].apply(spatial_gravity * -1.);

			if (model.mJoints[i].mDoFCount == 3)
			{
				model.a[i] = model.a[i]
							 + model.multdof3_S[i]
							 * Vector3d(QDDot[q_index], QDDot[q_index + 1],
										QDDot[q_index + 2]);
			}
			else
			{
				model.a[i] = model.a[i] + model.S[i] * QDDot[q_index];
			}

		}
		else
		{
			model.X_base[i] = model.X_lambda[i] * model.X_base[lambda];
			model.v[i] = model.X_lambda[i].apply(model.v[lambda]) + v_J;
			model.c[i] = c_J + crossm(model.v[i], v_J);
			model.a[i] = model.X_lambda[i].apply(model.a[lambda]) + model.c[i];

			if (model.mJoints[i].mDoFCount == 3)
			{
				Vector3d omegadot_temp(QDDot[q_index], QDDot[q_index + 1],
									   QDDot[q_index + 2]);
				model.a[i] = model.a[i] + model.multdof3_S[i] * omegadot_temp;
			}
			else
			{
				model.a[i] = model.a[i] + model.S[i] * QDDot[q_index];
			}
		}

		model.f[i] = model.mBodies[i].mSpatialInertia * model.a[i]
					 + crossf(model.v[i],
							  model.mBodies[i].mSpatialInertia * model.v[i]);
		if (f_ext != NULL && (*f_ext)[i] != SpatialVectorZero)
			model.f[i] -= model.X_base[i].toMatrixAdjoint() * (*f_ext)[i];
	}

    // debug
    for (i = 0; i < model.mBodies.size(); ++i)
    {
        std::cout << "Joint Force " << i << " : " << model.f[i].transpose() << std::endl;
    }

	for (i = model.mBodies.size() - 1; i > 0; i--)
	{
		unsigned int q_index = model.mJoints[i].q_index;
		unsigned int lambda = model.lambda[i];

		if (model.mJoints[i].mDoFCount == 3)
		{
			Vector3d tau_temp = model.multdof3_S[i].transpose() * model.f[i];
			Tau[q_index] = tau_temp[0];
			Tau[q_index + 1] = tau_temp[1];
			Tau[q_index + 2] = tau_temp[2];
		}
		else
		{
			Tau[q_index] = model.S[i].dot(model.f[i]);
		}

		if (lambda != 0)
		{
			model.f[lambda] = model.f[lambda]
							  + model.X_lambda[i].applyTranspose(model.f[i]);
		}
	}

    // debug
    for (i = 1; i < model.mBodies.size(); ++i)
    {
        std::cout << "Joint Force " << i << " : " << model.f[i].transpose() << std::endl;
    }
    // debug
    for (i = 1; i < model.mBodies.size(); ++i)
    {
        unsigned int q_index = model.mJoints[i].q_index;
        std::cout << "Tau " << q_index << " from " << i << " : " << Tau[q_index] << std::endl;
    }
}

void updatePartialKinematicsAndDynamics(RigidBodyDynamics::Model &model,
										const RigidBodyDynamics::Math::VectorNd &Q,
										const RigidBodyDynamics::Math::VectorNd &QDot,
										const RigidBodyDynamics::Math::VectorNd &QDDot,
										RigidBodyDynamics::Math::VectorNd &Tau,
                                        const std::vector<RigidBodyDynamics::Math::SpatialVector> *f_ext,
										const std::vector<unsigned int>& body_ids)
{
	SpatialVector spatial_gravity(0., 0., 0., model.gravity[0],
								  model.gravity[1], model.gravity[2]);

	unsigned int i;

	// subtract the force of body_ids[0] from parents
	i = body_ids[0];
	unsigned int lambda = model.lambda[i];
	RigidBodyDynamics::Math::SpatialVector propagated_force = model.f[i];
	while (lambda != 0)
	{
		propagated_force = model.X_lambda[i].applyTranspose(propagated_force);
		model.f[lambda] -= propagated_force;

		i = lambda;
		lambda = model.lambda[i];
	}

	for (unsigned int id = 0; id < body_ids.size(); ++id)
	{
		i = body_ids[id];

		unsigned int q_index = model.mJoints[i].q_index;
		SpatialTransform X_J;
		SpatialVector v_J;
		SpatialVector c_J;
		unsigned int lambda = model.lambda[i];

		jcalc(model, i, X_J, v_J, c_J, Q, QDot);

		model.X_lambda[i] = X_J * model.X_T[i];

		if (lambda == 0)
		{
			model.X_base[i] = model.X_lambda[i];
			model.v[i] = v_J;
			model.a[i] = model.X_base[i].apply(spatial_gravity * -1.);

			if (model.mJoints[i].mDoFCount == 3)
			{
				model.a[i] = model.a[i]
							 + model.multdof3_S[i]
							 * Vector3d(QDDot[q_index], QDDot[q_index + 1],
										QDDot[q_index + 2]);
			}
			else
			{
				model.a[i] = model.a[i] + model.S[i] * QDDot[q_index];
			}

		}
		else
		{
			model.X_base[i] = model.X_lambda[i] * model.X_base[lambda];
			model.v[i] = model.X_lambda[i].apply(model.v[lambda]) + v_J;
			model.c[i] = c_J + crossm(model.v[i], v_J);
			model.a[i] = model.X_lambda[i].apply(model.a[lambda]) + model.c[i];

			if (model.mJoints[i].mDoFCount == 3)
			{
				Vector3d omegadot_temp(QDDot[q_index], QDDot[q_index + 1],
									   QDDot[q_index + 2]);
				model.a[i] = model.a[i] + model.multdof3_S[i] * omegadot_temp;
			}
			else
			{
				model.a[i] = model.a[i] + model.S[i] * QDDot[q_index];
			}
		}

		model.f[i] = model.mBodies[i].mSpatialInertia * model.a[i]
					 + crossf(model.v[i],
							  model.mBodies[i].mSpatialInertia * model.v[i]);
		if (f_ext != NULL && (*f_ext)[i] != SpatialVectorZero)
			model.f[i] -= model.X_base[i].toMatrixAdjoint() * (*f_ext)[i];
	}

	for (int id = body_ids.size() - 1; id > 0; --id)
	{
		i = body_ids[id];

		unsigned int q_index = model.mJoints[i].q_index;
		unsigned int lambda = model.lambda[i];

		if (model.mJoints[i].mDoFCount == 3)
		{
			Vector3d tau_temp = model.multdof3_S[i].transpose() * model.f[i];
			Tau[q_index] = tau_temp[0];
			Tau[q_index + 1] = tau_temp[1];
			Tau[q_index + 2] = tau_temp[2];
		}
		else
		{
			Tau[q_index] = model.S[i].dot(model.f[i]);
		}

		model.f[lambda] = model.f[lambda]
						  + model.X_lambda[i].applyTranspose(model.f[i]);
	}

	i = body_ids[0];
	lambda = model.lambda[i];
	propagated_force = model.f[i];

	while (i != 0)
	{
		unsigned int q_index = model.mJoints[i].q_index;

		if (model.mJoints[i].mDoFCount == 3)
		{
			Vector3d tau_temp = model.multdof3_S[i].transpose() * model.f[i];
			Tau[q_index] = tau_temp[0];
			Tau[q_index + 1] = tau_temp[1];
			Tau[q_index + 2] = tau_temp[2];
		}
		else
		{
			Tau[q_index] = model.S[i].dot(model.f[i]);
		}

		if (lambda != 0)
		{
			propagated_force = model.X_lambda[i].applyTranspose(
								   propagated_force);
			model.f[lambda] += propagated_force;
		}

		i = lambda;
		lambda = model.lambda[i];
	}
}

void updatePartialDynamics(RigidBodyDynamics::Model &model,
						   const RigidBodyDynamics::Math::VectorNd &Q,
						   const RigidBodyDynamics::Math::VectorNd &QDot,
						   const RigidBodyDynamics::Math::VectorNd &QDDot,
						   RigidBodyDynamics::Math::VectorNd &Tau,
                           const std::vector<RigidBodyDynamics::Math::SpatialVector> *f_ext)
{
	unsigned int i;

	for (i = 1; i < model.mBodies.size(); i++)
	{
		model.f[i] = model.mBodies[i].mSpatialInertia * model.a[i]
					 + crossf(model.v[i],
							  model.mBodies[i].mSpatialInertia * model.v[i]);
		if (f_ext != NULL && (*f_ext)[i] != SpatialVectorZero)
			model.f[i] -= model.X_base[i].toMatrixAdjoint() * (*f_ext)[i];
	}

	for (i = model.mBodies.size() - 1; i > 0; i--)
	{
		unsigned int q_index = model.mJoints[i].q_index;
		unsigned int lambda = model.lambda[i];

		if (model.mJoints[i].mDoFCount == 3)
		{
			Vector3d tau_temp = model.multdof3_S[i].transpose() * model.f[i];
			Tau[q_index] = tau_temp[0];
			Tau[q_index + 1] = tau_temp[1];
			Tau[q_index + 2] = tau_temp[2];
		}
		else
		{
			Tau[q_index] = model.S[i].dot(model.f[i]);
		}

		if (lambda != 0)
		{
			model.f[lambda] = model.f[lambda]
							  + model.X_lambda[i].applyTranspose(model.f[i]);
		}
	}
}

///////////////////////////////////////////////////////////////////////

void UpdatePartialKinematics(RigidBodyDynamics::Model & model,
							 const RigidBodyDynamics::Math::VectorNd& Q,
							 const RigidBodyDynamics::Math::VectorNd& QDot,
							 const RigidBodyDynamics::Math::VectorNd& QDDot,
							 const std::vector<unsigned int>& body_ids)
{
	unsigned int i;

	SpatialVector spatial_gravity(0., 0., 0., model.gravity[0],
								  model.gravity[1], model.gravity[2]);

	model.a[0].setZero();
	//model.a[0] = spatial_gravity;

	//for (i = 1; i < model.mBodies.size(); i++)
	for (unsigned int id = 0; id < body_ids.size(); ++id)
	{
		i = body_ids[id];

		unsigned int q_index = model.mJoints[i].q_index;

		SpatialTransform X_J;
		SpatialVector v_J;
		SpatialVector c_J;
		Joint joint = model.mJoints[i];
		unsigned int lambda = model.lambda[i];

		jcalc(model, i, X_J, v_J, c_J, Q, QDot);

		model.X_lambda[i] = X_J * model.X_T[i];

		if (lambda != 0)
		{
			model.X_base[i] = model.X_lambda[i] * model.X_base[lambda];
			model.v[i] = model.X_lambda[i].apply(model.v[lambda]) + v_J;
			model.c[i] = c_J + crossm(model.v[i], v_J);
		}
		else
		{
			model.X_base[i] = model.X_lambda[i];
			model.v[i] = v_J;
			model.c[i].setZero();
		}

		model.a[i] = model.X_lambda[i].apply(model.a[lambda]) + model.c[i];

		if (model.mJoints[i].mDoFCount == 3)
		{
			Vector3d omegadot_temp(QDDot[q_index], QDDot[q_index + 1],
								   QDDot[q_index + 2]);
			model.a[i] = model.a[i] + model.multdof3_S[i] * omegadot_temp;
		}
		else
		{
			model.a[i] = model.a[i] + model.S[i] * QDDot[q_index];
		}
	}
}

void CalcFullJacobian(RigidBodyDynamics::Model & model,
					  const RigidBodyDynamics::Math::VectorNd & Q, unsigned int body_id,
					  const RigidBodyDynamics::Math::Vector3d & point_position,
					  RigidBodyDynamics::Math::MatrixNd & G, bool update_kinematics)
{
	// update the Kinematics if necessary
	if (update_kinematics)
	{
		UpdateKinematicsCustom(model, &Q, NULL, NULL);
	}

	Vector3d point_base_pos = CalcBodyToBaseCoordinates(model, Q, body_id,
							  point_position, false);
	SpatialMatrix point_trans = Xtrans_mat(point_base_pos);

	assert(G.rows() == 6 && G.cols() == model.qdot_size);

	G.setZero();

	// we have to make sure that only the joints that contribute to the
	// bodies motion also get non-zero columns in the jacobian.
	// VectorNd e = VectorNd::Zero(Q.size() + 1);
	char *e = new char[Q.size() + 1];
	if (e == NULL)
	{
		std::cerr << "Error: allocating memory." << std::endl;
		abort();
	}
	memset(&e[0], 0, Q.size() + 1);

	unsigned int reference_body_id = body_id;

	if (model.IsFixedBodyId(body_id))
	{
		unsigned int fbody_id = body_id - model.fixed_body_discriminator;
		reference_body_id = model.mFixedBodies[fbody_id].mMovableParent;
	}

	unsigned int j = reference_body_id;

	// e[j] is set to 1 if joint j contributes to the jacobian that we are
	// computing. For all other joints the column will be zero.
	while (j != 0)
	{
		e[j] = 1;
		j = model.lambda[j];
	}

	for (j = 1; j < model.mBodies.size(); j++)
	{
		if (e[j] == 1)
		{
			unsigned int q_index = model.mJoints[j].q_index;

			if (model.mJoints[j].mDoFCount == 3)
			{
				Matrix63 S_base = point_trans
								  * spatial_inverse(model.X_base[j].toMatrix())
								  * model.multdof3_S[j];

				// TODO
				G(0, q_index) = S_base(0, 0);
				G(1, q_index) = S_base(1, 0);
				G(2, q_index) = S_base(2, 0);

				G(0, q_index + 1) = S_base(0, 1);
				G(1, q_index + 1) = S_base(1, 1);
				G(2, q_index + 1) = S_base(2, 1);

				G(0, q_index + 2) = S_base(0, 2);
				G(1, q_index + 2) = S_base(1, 2);
				G(2, q_index + 2) = S_base(2, 2);

				G(3, q_index) = S_base(3, 0);
				G(4, q_index) = S_base(4, 0);
				G(5, q_index) = S_base(5, 0);

				G(3, q_index + 1) = S_base(3, 1);
				G(4, q_index + 1) = S_base(4, 1);
				G(5, q_index + 1) = S_base(5, 1);

				G(3, q_index + 2) = S_base(3, 2);
				G(4, q_index + 2) = S_base(4, 2);
				G(5, q_index + 2) = S_base(5, 2);
			}
			else
			{
				SpatialVector S_base;
				S_base = point_trans
						 * spatial_inverse(model.X_base[j].toMatrix())
						 * model.S[j];
				// TODO
				G(0, q_index) = S_base[0];
				G(1, q_index) = S_base[1];
				G(2, q_index) = S_base[2];
				G(3, q_index) = S_base[3];
				G(4, q_index) = S_base[4];
				G(5, q_index) = S_base[5];
			}
		}
	}

	delete[] e;
}

void CalcFullJacobianBasePosition(RigidBodyDynamics::Model & model,
								  const RigidBodyDynamics::Math::VectorNd & Q, unsigned int body_id,
								  const RigidBodyDynamics::Math::Vector3d & point_base_pos,
								  RigidBodyDynamics::Math::MatrixNd & G, bool update_kinematics)
{
	// update the Kinematics if necessary
	if (update_kinematics)
	{
		UpdateKinematicsCustom(model, &Q, NULL, NULL);
	}
	SpatialMatrix point_trans = Xtrans_mat(point_base_pos);

	assert(G.rows() == 6 && G.cols() == model.qdot_size);

	G.setZero();

	// we have to make sure that only the joints that contribute to the
	// bodies motion also get non-zero columns in the jacobian.
	// VectorNd e = VectorNd::Zero(Q.size() + 1);
	char *e = new char[Q.size() + 1];
	if (e == NULL)
	{
		std::cerr << "Error: allocating memory." << std::endl;
		abort();
	}
	memset(&e[0], 0, Q.size() + 1);

	unsigned int reference_body_id = body_id;

	if (model.IsFixedBodyId(body_id))
	{
		unsigned int fbody_id = body_id - model.fixed_body_discriminator;
		reference_body_id = model.mFixedBodies[fbody_id].mMovableParent;
	}

	unsigned int j = reference_body_id;

	// e[j] is set to 1 if joint j contributes to the jacobian that we are
	// computing. For all other joints the column will be zero.
	while (j != 0)
	{
		e[j] = 1;
		j = model.lambda[j];
	}

	for (j = 1; j < model.mBodies.size(); j++)
	{
		if (e[j] == 1)
		{
			unsigned int q_index = model.mJoints[j].q_index;

			if (model.mJoints[j].mDoFCount == 3)
			{
				Matrix63 S_base = point_trans
								  * spatial_inverse(model.X_base[j].toMatrix())
								  * model.multdof3_S[j];

				// TODO
				G(0, q_index) = S_base(0, 0);
				G(1, q_index) = S_base(1, 0);
				G(2, q_index) = S_base(2, 0);

				G(0, q_index + 1) = S_base(0, 1);
				G(1, q_index + 1) = S_base(1, 1);
				G(2, q_index + 1) = S_base(2, 1);

				G(0, q_index + 2) = S_base(0, 2);
				G(1, q_index + 2) = S_base(1, 2);
				G(2, q_index + 2) = S_base(2, 2);

				G(3, q_index) = S_base(3, 0);
				G(4, q_index) = S_base(4, 0);
				G(5, q_index) = S_base(5, 0);

				G(3, q_index + 1) = S_base(3, 1);
				G(4, q_index + 1) = S_base(4, 1);
				G(5, q_index + 1) = S_base(5, 1);

				G(3, q_index + 2) = S_base(3, 2);
				G(4, q_index + 2) = S_base(4, 2);
				G(5, q_index + 2) = S_base(5, 2);
			}
			else
			{
				SpatialVector S_base;
				S_base = point_trans
						 * spatial_inverse(model.X_base[j].toMatrix())
						 * model.S[j];
				// TODO
				G(0, q_index) = S_base[0];
				G(1, q_index) = S_base[1];
				G(2, q_index) = S_base[2];
				G(3, q_index) = S_base[3];
				G(4, q_index) = S_base[4];
				G(5, q_index) = S_base[5];
			}
		}
	}

	delete[] e;
}

void InverseDynamics2(Model &model, const VectorNd &Q, const VectorNd &QDot,
					  const VectorNd &QDDot, VectorNd &Tau, std::vector<SpatialVector> *f_ext)
{
	SpatialVector spatial_gravity(0., 0., 0., model.gravity[0],
								  model.gravity[1], model.gravity[2]);

	unsigned int i;

	// Reset the velocity of the root body
	model.v[0].setZero();
	model.a[0] = spatial_gravity * -1.;

	for (i = 1; i < model.mBodies.size(); i++)
	{
		unsigned int q_index = model.mJoints[i].q_index;
		SpatialTransform X_J;
		SpatialVector v_J;
		SpatialVector c_J;
		unsigned int lambda = model.lambda[i];

		jcalc(model, i, X_J, v_J, c_J, Q, QDot);

		model.X_lambda[i] = X_J * model.X_T[i];

		if (lambda == 0)
		{
			model.X_base[i] = model.X_lambda[i];
			model.v[i] = v_J;
			model.a[i] = model.X_base[i].apply(spatial_gravity * -1.);

			if (model.mJoints[i].mDoFCount == 3)
			{
				model.a[i] = model.a[i]
							 + model.multdof3_S[i]
							 * Vector3d(QDDot[q_index], QDDot[q_index + 1],
										QDDot[q_index + 2]);
			}
			else
			{
				model.a[i] = model.a[i] + model.S[i] * QDDot[q_index];
			}

		}
		else
		{
			model.X_base[i] = model.X_lambda[i] * model.X_base[lambda];
			model.v[i] = model.X_lambda[i].apply(model.v[lambda]) + v_J;
			model.c[i] = c_J + crossm(model.v[i], v_J);
			model.a[i] = model.X_lambda[i].apply(model.a[lambda]) + model.c[i];

			if (model.mJoints[i].mDoFCount == 3)
			{
				Vector3d omegadot_temp(QDDot[q_index], QDDot[q_index + 1],
									   QDDot[q_index + 2]);
				model.a[i] = model.a[i] + model.multdof3_S[i] * omegadot_temp;
			}
			else
			{
				model.a[i] = model.a[i] + model.S[i] * QDDot[q_index];
			}
		}

		model.f[i] = model.mBodies[i].mSpatialInertia * model.a[i]
					 + crossf(model.v[i],
							  model.mBodies[i].mSpatialInertia * model.v[i]);
		if (f_ext != NULL && (*f_ext)[i] != SpatialVectorZero)
			model.f[i] -= model.X_base[i].toMatrixAdjoint() * (*f_ext)[i];
	}

	for (i = model.mBodies.size() - 1; i > 0; i--)
	{
		unsigned int q_index = model.mJoints[i].q_index;
		unsigned int lambda = model.lambda[i];

		if (model.mJoints[i].mDoFCount == 3)
		{
			Vector3d tau_temp = model.multdof3_S[i].transpose() * model.f[i];
			Tau[q_index] = tau_temp[0];
			Tau[q_index + 1] = tau_temp[1];
			Tau[q_index + 2] = tau_temp[2];
		}
		else
		{
			Tau[q_index] = model.S[i].dot(model.f[i]);
		}

		if (lambda != 0)
		{
			model.f[lambda] = model.f[lambda]
							  + model.X_lambda[i].applyTranspose(model.f[i]);
		}
	}
}

}

