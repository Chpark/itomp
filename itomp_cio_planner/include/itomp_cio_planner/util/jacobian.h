
#ifndef _CLASS_JACOBIAN
#define _CLASS_JACOBIAN

#include <Eigen/Dense>
#include <itomp_cio_planner/optimization/new_eval_manager.h>
#include <itomp_cio_planner/trajectory/itomp_trajectory.h>

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
    static void projectToNullSpace(dlib::matrix<double, 0, 1>& x, dlib::matrix<double, 0, 1>& s);

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
