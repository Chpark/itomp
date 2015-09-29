
#ifndef _CLASS_JACOBIAN
#define _CLASS_JACOBIAN

#include <Eigen/Dense>
#include <itomp_cio_planner/optimization/new_eval_manager.h>
/*
namespace itomp_cio_planner
{
class NewEvalManager;
}
*/
namespace planner
{
class Node;
}

Eigen::MatrixXd PseudoInverseDLS(const Eigen::MatrixXd& J, double eps);
void PseudoInverseSVDDLS(const Eigen::MatrixXd& J, const Eigen::JacobiSVD<Eigen::MatrixXd>& svdOfJ, Eigen::MatrixXd& Jinv);

class Jacobian
{

public:
	Jacobian();
	~Jacobian();

private:

public:
	void  SetJacobian(const Eigen::MatrixXd& jacobian);
	void  ComputeAll(); // recomputes everything but the jacobian
	void  ComputeJacobian();
	const Eigen::MatrixXd& GetNullspace();
	const Eigen::MatrixXd& GetJacobian();
	const Eigen::MatrixXd& GetJacobianInverse();
	const Eigen::MatrixXd& GetJacobianProduct();
	const Eigen::MatrixXd& GetJacobianProductInverse();
	void GetEllipsoidAxes(Eigen::Vector3d& /*u1*/, Eigen::Vector3d& /*u2*/, Eigen::Vector3d& /*u3*/);
	void GetEllipsoidAxes(Eigen::Vector3d& /*u1*/, Eigen::Vector3d& /*u2*/, Eigen::Vector3d& /*u3*/, double& /*sig1*/, double& /*sig2*/, double& /*sig3*/);

	void  GetNullspace(const Eigen::MatrixXd /*pseudoId*/, Eigen::MatrixXd& /*result*/);

	// temporary
	static void GetProjection(int point, const Eigen::VectorXd& q, Eigen::VectorXd& a);
	template <typename T>
	static void projectToNullSpace(T& x, T& s)
	{
        assert(false);

        /*
		int parameter_length = x.size();

		const itomp_cio_planner::ParameterTrajectoryConstPtr parameter_trajectory =
			evaluation_manager_->getParameterTrajectory();
		const itomp_cio_planner::FullTrajectoryConstPtr full_trajectory =
			evaluation_manager_->getFullTrajectory();

		int num_parameter_types = parameter_trajectory->hasVelocity() ? 2 : 1;
		int num_parameter_points = parameter_trajectory->getNumPoints();
		int num_parameter_elements = parameter_trajectory->getNumElements();
		int num_variables = num_parameter_elements * num_parameter_points * num_parameter_types;

		if (parameter_length != num_variables)
			return;

		std::vector<Eigen::MatrixXd> parameters(itomp_cio_planner::Trajectory::TRAJECTORY_TYPE_NUM,
												Eigen::MatrixXd(num_parameter_points, num_parameter_elements));
		evaluation_manager_->setParameters(parameters);
		evaluation_manager_->updateFromParameterTrajectory();

		int num_full_joints = full_trajectory->getComponentSize(
								  itomp_cio_planner::FullTrajectory::TRAJECTORY_COMPONENT_JOINT);

		const std::vector<int>& group_to_full_joint_indices = parameter_trajectory->getGroupToFullJointIndices();
		for (int i = 0; i < num_parameter_points; ++i)
		{
			int parameter_traj_index = full_trajectory->getKeyframeStartIndex()
									   + i * full_trajectory->getNumKeyframeIntervalPoints();

			Eigen::VectorXd q = full_trajectory->getTrajectory(
									itomp_cio_planner::Trajectory::TRAJECTORY_TYPE_POSITION).block(
									parameter_traj_index, 0, 1, num_full_joints).transpose();
			Eigen::VectorXd a(num_full_joints);
			for (int j = 0; j < parameter_trajectory->getNumJoints(); ++j)
			{
				// copy from s to a
				a(group_to_full_joint_indices[j]) = s(i * num_parameter_elements + j);
			}
			GetProjection(i, q, a);

			for (int j = 0; j < parameter_trajectory->getNumJoints(); ++j)
			{
				// copy from a to s
				s(i * num_parameter_elements + j) = a(group_to_full_joint_indices[j]);
			}
		}
        */
	}

private:
	void ComputeSVD();

private:
	bool computeInverse_;
	bool computeProduct_;
	bool computeProductInverse_;
	bool computeJacSVD_;
	bool computeNullSpace_;
	void Invalidate();

private:
	Eigen::MatrixXd jacobianProductInverse_;
	Eigen::MatrixXd jacobianProduct_;
	Eigen::MatrixXd jacobian_;
	Eigen::MatrixXd jacobianInverse_;
	Eigen::MatrixXd jacobianInverseNoDls_;
	Eigen::MatrixXd Identitymin_;
	Eigen::JacobiSVD<Eigen::MatrixXd> svd_;
	Eigen::JacobiSVD<Eigen::MatrixXd> svdProduct_;

public:
	static itomp_cio_planner::NewEvalManager* evaluation_manager_;
};

inline Eigen::MatrixXd PseudoInverseDLS(const Eigen::MatrixXd& J, double eps)
{
	Eigen::JacobiSVD<Eigen::MatrixXd> svdOfJ(J, Eigen::ComputeThinU | Eigen::ComputeThinV);

	Eigen::MatrixXd Jinv;
	PseudoInverseSVDDLS(J, svdOfJ, Jinv);

	return Jinv;
}

inline void PseudoInverseSVDDLS(const Eigen::MatrixXd& m, const Eigen::JacobiSVD<Eigen::MatrixXd>& svdOfJ, Eigen::MatrixXd& Jinv)
{
	const Eigen::MatrixXd U = svdOfJ.matrixU();
	const Eigen::MatrixXd V = svdOfJ.matrixV();
	const Eigen::VectorXd S = svdOfJ.singularValues();
	Eigen::VectorXd Sinv = S;
	static const double pinvtoler = std::numeric_limits<float>::epsilon();
	double maxsv = 0.0 ;
	for (std::size_t i = 0; i < static_cast<std::size_t>(S.rows()); ++i)
		if (fabs(S(i)) > maxsv) maxsv = fabs(S(i));
	for (std::size_t i = 0; i < static_cast<std::size_t>(S.rows()); ++i)
	{
		//Those singular values smaller than a percentage of the maximum singular value are removed
		if (fabs(S(i)) > maxsv * pinvtoler)
			Sinv(i) = 1.0 / S(i);
		else Sinv(i) = 0.0;
	}
	Jinv = (V * Sinv.asDiagonal() * U.transpose());
}

#endif //_CLASS_JACOBIAN
