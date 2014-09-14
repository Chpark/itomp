#include <itomp_cio_planner/model/rbdl_model_util.h>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;
namespace itomp_cio_planner
{

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

}

