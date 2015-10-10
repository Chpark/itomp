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
                                     const std::vector<RigidBodyDynamics::Math::SpatialVector> *f_ext,
                                     const std::vector<double> *joint_forces)
{
    SpatialVector spatial_gravity(0., 0., 0., model.gravity[0], model.gravity[1], model.gravity[2]);

	unsigned int i;

	// Reset the velocity of the root body
	model.v[0].setZero();
    model.a[0] = spatial_gravity;

	for (i = 1; i < model.mBodies.size(); i++)
	{
		unsigned int q_index = model.mJoints[i].q_index;
		unsigned int lambda = model.lambda[i];

        jcalc(model, i, Q, QDot);

        // forward kinematics
        model.X_lambda[i] = model.X_J[i] * model.X_T[i];

        if (lambda != 0)
        {
            model.X_base[i] = model.X_lambda[i] * model.X_base[lambda];
            model.v[i] = model.X_lambda[i].apply(model.v[lambda]) + model.v_J[i];
        }
        else
        {
            model.X_base[i] = model.X_lambda[i];
            model.v[i] = model.v_J[i];
        }

        model.c[i] = model.c_J[i] + crossm(model.v[i], model.v_J[i]);
        model.a[i] = model.X_lambda[i].apply(model.a[lambda]) + model.c[i];

        if (model.mJoints[i].mDoFCount == 3)
        {
            Vector3d omegadot_temp (QDDot[q_index], QDDot[q_index + 1], QDDot[q_index + 2]);
            model.a[i] = model.a[i] + model.multdof3_S[i] * omegadot_temp;
        }
        else
        {
            model.a[i] = model.a[i] + model.S[i] * QDDot[q_index];
        }

        // inverse dynamics
        if (!model.mBodies[i].mIsVirtual)
        {
            model.f[i] = model.I[i] * model.a[i] + crossf(model.v[i],model.I[i] * model.v[i]);
        }
        else
        {
            model.f[i].setZero();
        }

        if (joint_forces != NULL && (*joint_forces)[i] != 0.0)
            model.f[i] -= model.S[i] * (*joint_forces)[i];

        if (f_ext != NULL && (*f_ext)[i] != SpatialVectorZero)
            model.f[i] += model.X_base[i].toMatrixAdjoint() * (*f_ext)[i];
	}

	for (i = model.mBodies.size() - 1; i > 0; i--)
	{
        if (model.mJoints[i].mDoFCount == 3)
        {
            Tau.block<3,1>(model.mJoints[i].q_index, 0) = model.multdof3_S[i].transpose() * model.f[i];
        }
        else
        {
            Tau[model.mJoints[i].q_index] = model.S[i].dot(model.f[i]);
        }

        if (model.lambda[i] != 0)
        {
            model.f[model.lambda[i]] = model.f[model.lambda[i]] + model.X_lambda[i].applyTranspose(model.f[i]);
        }
	}
}

void updatePartialKinematicsAndDynamics(RigidBodyDynamics::Model &model,
										const RigidBodyDynamics::Math::VectorNd &Q,
										const RigidBodyDynamics::Math::VectorNd &QDot,
										const RigidBodyDynamics::Math::VectorNd &QDDot,
										RigidBodyDynamics::Math::VectorNd &Tau,
                                        const std::vector<RigidBodyDynamics::Math::SpatialVector> *f_ext,
                                        const std::vector<double> *joint_forces,
										const std::vector<unsigned int>& body_ids)
{
    SpatialVector spatial_gravity(0., 0., 0., model.gravity[0], model.gravity[1], model.gravity[2]);

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
		unsigned int lambda = model.lambda[i];

        jcalc(model, i, Q, QDot);

        // forward kinematics
        model.X_lambda[i] = model.X_J[i] * model.X_T[i];

        if (lambda != 0)
        {
            model.X_base[i] = model.X_lambda[i] * model.X_base[lambda];
            model.v[i] = model.X_lambda[i].apply(model.v[lambda]) + model.v_J[i];
        }
        else
        {
            model.X_base[i] = model.X_lambda[i];
            model.v[i] = model.v_J[i];
        }

        model.c[i] = model.c_J[i] + crossm(model.v[i], model.v_J[i]);
        model.a[i] = model.X_lambda[i].apply(model.a[lambda]) + model.c[i];

        if (model.mJoints[i].mDoFCount == 3)
        {
            Vector3d omegadot_temp (QDDot[q_index], QDDot[q_index + 1], QDDot[q_index + 2]);
            model.a[i] = model.a[i] + model.multdof3_S[i] * omegadot_temp;
        }
        else
        {
            model.a[i] = model.a[i] + model.S[i] * QDDot[q_index];
        }

        // inverse dynamics
        if (!model.mBodies[i].mIsVirtual)
        {
            model.f[i] = model.I[i] * model.a[i] + crossf(model.v[i],model.I[i] * model.v[i]);
        }
        else
        {
            model.f[i].setZero();
        }

        if (joint_forces != NULL && (*joint_forces)[i] != 0.0)
            model.f[i] -= model.S[i] * (*joint_forces)[i];

        if (f_ext != NULL && (*f_ext)[i] != SpatialVectorZero)
            model.f[i] += model.X_base[i].toMatrixAdjoint() * (*f_ext)[i];
	}

	for (int id = body_ids.size() - 1; id > 0; --id)
	{
		i = body_ids[id];

        if (model.mJoints[i].mDoFCount == 3)
        {
            Tau.block<3,1>(model.mJoints[i].q_index, 0) = model.multdof3_S[i].transpose() * model.f[i];
        }
        else
        {
            Tau[model.mJoints[i].q_index] = model.S[i].dot(model.f[i]);
        }

        if (model.lambda[i] != 0)
        {
            model.f[model.lambda[i]] = model.f[model.lambda[i]] + model.X_lambda[i].applyTranspose(model.f[i]);
        }
	}

	i = body_ids[0];
	lambda = model.lambda[i];
	propagated_force = model.f[i];

	while (i != 0)
	{
		if (model.mJoints[i].mDoFCount == 3)
		{
            Tau.block<3,1>(model.mJoints[i].q_index, 0) = model.multdof3_S[i].transpose() * model.f[i];
		}
		else
		{
            Tau[model.mJoints[i].q_index] = model.S[i].dot(model.f[i]);
		}

		if (lambda != 0)
		{
            propagated_force = model.X_lambda[i].applyTranspose(propagated_force);
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
                           const std::vector<RigidBodyDynamics::Math::SpatialVector> *f_ext,
                           const std::vector<double> *joint_forces)
{
	unsigned int i;

	for (i = 1; i < model.mBodies.size(); i++)
	{
        // inverse dynamics
        if (!model.mBodies[i].mIsVirtual)
        {
            model.f[i] = model.I[i] * model.a[i] + crossf(model.v[i],model.I[i] * model.v[i]);
        }
        else
        {
            model.f[i].setZero();
        }

        if (joint_forces != NULL && (*joint_forces)[i] != 0.0)
            model.f[i] -= model.S[i] * (*joint_forces)[i];

        if (f_ext != NULL && (*f_ext)[i] != SpatialVectorZero)
            model.f[i] += model.X_base[i].toMatrixAdjoint() * (*f_ext)[i];
	}

	for (i = model.mBodies.size() - 1; i > 0; i--)
	{
        if (model.mJoints[i].mDoFCount == 3)
        {
            Tau.block<3,1>(model.mJoints[i].q_index, 0) = model.multdof3_S[i].transpose() * model.f[i];
        }
        else
        {
            Tau[model.mJoints[i].q_index] = model.S[i].dot(model.f[i]);
        }

        if (model.lambda[i] != 0)
        {
            model.f[model.lambda[i]] = model.f[model.lambda[i]] + model.X_lambda[i].applyTranspose(model.f[i]);
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

	for (unsigned int id = 0; id < body_ids.size(); ++id)
	{
		i = body_ids[id];

		unsigned int q_index = model.mJoints[i].q_index;
		unsigned int lambda = model.lambda[i];

        jcalc(model, i, Q, QDot);

        model.X_lambda[i] = model.X_J[i] * model.X_T[i];

        if (lambda != 0)
        {
            model.X_base[i] = model.X_lambda[i] * model.X_base[lambda];
            model.v[i] = model.X_lambda[i].apply(model.v[lambda]) + model.v_J[i];
        }
        else
        {
            model.X_base[i] = model.X_lambda[i];
            model.v[i] = model.v_J[i];
        }

        model.c[i] = model.c_J[i] + crossm(model.v[i], model.v_J[i]);
        model.a[i] = model.X_lambda[i].apply(model.a[lambda]) + model.c[i];

        if (model.mJoints[i].mDoFCount == 3)
        {
            Vector3d omegadot_temp (QDDot[q_index], QDDot[q_index + 1], QDDot[q_index + 2]);
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
    /*
	// update the Kinematics if necessary
	if (update_kinematics)
	{
		UpdateKinematicsCustom(model, &Q, NULL, NULL);
	}

    Vector3d point_base_pos = CalcBodyToBaseCoordinates(model, Q, body_id, point_position, false);
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
    */
}

}

