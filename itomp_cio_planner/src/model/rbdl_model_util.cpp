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
            model.f[i] += model.S[i] * (*joint_forces)[i];

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
            model.f[i] += model.S[i] * (*joint_forces)[i];

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
            model.f[i] += model.S[i] * (*joint_forces)[i];

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

bool InverseKinematics6D (
        Model &model,
        const VectorNd &Qinit,
        const std::vector<unsigned int>& body_id,
        const std::vector<Vector3d>& target_pos,
        const std::vector<Matrix3d>& target_ori,
        VectorNd &Qres,
        double step_tol,
        double lambda,
        unsigned int max_iter
        )
{

    assert (Qinit.size() == model.q_size);
    assert (body_id.size() == target_pos.size());

    MatrixNd J = MatrixNd::Zero(6 * body_id.size(), model.qdot_size);
    VectorNd e = VectorNd::Zero(6 * body_id.size());

    Qres = Qinit;

    for (unsigned int ik_iter = 0; ik_iter < max_iter; ik_iter++) {
        UpdateKinematicsCustom (model, &Qres, NULL, NULL);
        for (unsigned int k = 0; k < body_id.size(); k++) {
            MatrixNd G (MatrixNd::Zero(6, model.qdot_size));
            CalcPointJacobian6D(model, Qres, body_id[k], Vector3d::Zero(), G, false);
            Vector3d point_base = CalcBodyToBaseCoordinates (model, Qres, body_id[k], Vector3d::Zero(), false);
            LOG << "current_pos = " << point_base.transpose() << std::endl;

            Matrix3d body_world_ori = CalcBodyWorldOrientation(model, Qres, body_id[k], false);
            Eigen::Quaterniond quat_from(body_world_ori);
            Eigen::Quaterniond quat_target(target_ori[k]);
            Matrix3d skew = Matrix3d::Zero();
            skew(0, 1) = -quat_target.vec()[2];
            skew(1, 0) = quat_target.vec()[2];
            skew(0, 2) = quat_target.vec()[1];
            skew(2, 0) = -quat_target.vec()[1];
            skew(1, 2) = -quat_target.vec()[0];
            skew(2, 1) = quat_target.vec()[0];
            Vector3d ori_diff = quat_from.w() * quat_target.vec() - quat_target.w() * quat_from.vec() - skew * quat_from.vec();

            for (unsigned int i = 0; i < 6; i++) {
                for (unsigned int j = 0; j < model.qdot_size; j++) {
                    unsigned int row = k * 6 + i;
                    LOG << "i = " << i << " j = " << j << " k = " << k << " row = " << row << " col = " << j << std::endl;
                    J(row, j) = G (i,j);
                }

                if (i < 3)
                {
                    e[k * 6 + i] = -ori_diff[i];
                }
                else
                {
                    e[k * 6 + i] = target_pos[k][i - 3] - point_base[i - 3];
                }
            }

            LOG << J << std::endl;

            // abort if we are getting "close"
            if (e.norm() < step_tol) {
                LOG << "Reached target close enough after " << ik_iter << " steps" << std::endl;
                return true;
            }
        }

        LOG << "J = " << J << std::endl;
        LOG << "e = " << e.transpose() << std::endl;

        //std::cout << "iteration " << ik_iter << " " << e.norm() << std::endl;

        MatrixNd JJTe_lambda2_I = J * J.transpose() + lambda*lambda * MatrixNd::Identity(e.size(), e.size());

        VectorNd z (body_id.size() * 6);
#ifndef RBDL_USE_SIMPLE_MATH
        z = JJTe_lambda2_I.colPivHouseholderQr().solve (e);
#else
        bool solve_successful = LinSolveGaussElimPivot (JJTe_lambda2_I, e, z);
        assert (solve_successful);
#endif

        LOG << "z = " << z << std::endl;

        VectorNd delta_theta = J.transpose() * z;
        LOG << "change = " << delta_theta << std::endl;

        Qres = Qres + delta_theta;
        LOG << "Qres = " << Qres.transpose() << std::endl;

        if (delta_theta.norm() < step_tol) {
            LOG << "reached convergence after " << ik_iter << " steps" << std::endl;
            return true;
        }

        VectorNd test_1 (z.size());
        VectorNd test_res (z.size());

        test_1.setZero();

        for (unsigned int i = 0; i < z.size(); i++) {
            test_1[i] = 1.;

            VectorNd test_delta = J.transpose() * test_1;

            test_res[i] = test_delta.squaredNorm();

            test_1[i] = 0.;
        }

        LOG << "test_res = " << test_res.transpose() << std::endl;
    }

    return false;
}

void CalcPointJacobian6D (
        Model &model,
        const VectorNd &Q,
        unsigned int body_id,
        const Vector3d &point_position,
        MatrixNd &G,
        bool update_kinematics
    )
{
    LOG << "-------- " << __func__ << " --------" << std::endl;

    // update the Kinematics if necessary
    if (update_kinematics) {
        UpdateKinematicsCustom (model, &Q, NULL, NULL);
    }

    SpatialTransform point_trans = SpatialTransform (Matrix3d::Identity(), CalcBodyToBaseCoordinates (model, Q, body_id, point_position, false));

    assert (G.rows() == 6 && G.cols() == model.qdot_size );

    unsigned int reference_body_id = body_id;

    if (model.IsFixedBodyId(body_id)) {
        unsigned int fbody_id = body_id - model.fixed_body_discriminator;
        reference_body_id = model.mFixedBodies[fbody_id].mMovableParent;
    }

    unsigned int j = reference_body_id;

    Matrix3d point_rot = model.X_base[j].E;

    // e[j] is set to 1 if joint j contributes to the jacobian that we are
    // computing. For all other joints the column will be zero.
    while (j != 0) {
        unsigned int q_index = model.mJoints[j].q_index;

        if (model.mJoints[j].mDoFCount == 3) {
            //G.block(0, q_index, 3, 3) = ((point_rot * model.X_base[j].inverse()).toMatrix() * model.multdof3_S[j]).block(0, 0, 3, 3);
            G.block(3, q_index, 3, 3) = ((point_trans * model.X_base[j].inverse()).toMatrix() * model.multdof3_S[j]).block(3, 0, 3, 3);
        } else {
            G.block(3, q_index, 3, 1) = point_trans.apply(model.X_base[j].inverse().apply(model.S[j])).block(3, 0, 3, 1);

            Eigen::AngleAxisd aa(1.0, point_rot * model.X_base[j].E.transpose() * model.S[j].block(0, 0, 3, 1));
            Matrix3d mat = aa.toRotationMatrix();
            Eigen::Quaterniond quat_from = Eigen::Quaterniond::Identity();
            Eigen::Quaterniond quat_target(mat);
            Matrix3d skew = Matrix3d::Zero();
            skew(0, 1) = -quat_target.vec()[2];
            skew(1, 0) = quat_target.vec()[2];
            skew(0, 2) = quat_target.vec()[1];
            skew(2, 0) = -quat_target.vec()[1];
            skew(1, 2) = -quat_target.vec()[0];
            skew(2, 1) = quat_target.vec()[0];
            Vector3d ori_diff = quat_from.w() * quat_target.vec() - quat_target.w() * quat_from.vec() - skew * quat_from.vec();
            G.block(0, q_index, 3, 1) = ori_diff;
        }

        j = model.lambda[j];
    }
}

}

