
#ifndef _CLASS_JACOBIAN
#define _CLASS_JACOBIAN

#include <Eigen/Dense>
#include <itomp_cio_planner/optimization/new_eval_manager.h>
#include <itomp_cio_planner/trajectory/itomp_trajectory.h>
#include "dlib/assert.h"
#include "dlib/matrix.h"

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
    static void projectToNullSpace(const dlib::matrix<double, 0, 1>& x, dlib::matrix<double, 0, 1>& s);

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

/*
template <typename funct, typename T>
class line_search_funct2
{
public:
    line_search_funct2(const funct& f_, const T& start_, const T& direction_)
        : f(f_),start(start_), direction(direction_), matrix_r(0), scalar_r(0)
    {}

    line_search_funct2(const funct& f_, const T& start_, const T& direction_, T& r)
        : f(f_),start(start_), direction(direction_), matrix_r(&r), scalar_r(0)
    {}

    line_search_funct2(const funct& f_, const T& start_, const T& direction_, double& r)
        : f(f_),start(start_), direction(direction_), matrix_r(0), scalar_r(&r)
    {}

    double operator()(const double& x) const
    {
        return get_value(f(x));
    }

private:

    double get_value (const double& r) const
    {
        // save a copy of this value for later
        if (scalar_r)
            *scalar_r = r;

        return r;
    }

    const funct& f;
    const T& start;
    const T& direction;
    T* matrix_r;
    double* scalar_r;
};

template <typename funct, typename T>
const line_search_funct2<funct,T> make_line_search_function2(const funct& f, const T& start, const T& direction, double& f_out)
{
    COMPILE_TIME_ASSERT(dlib::is_matrix<T>::value);
    DLIB_ASSERT (
        is_col_vector(start) && is_col_vector(direction) && start.size() == direction.size(),
        "\tline_search_funct make_line_search_function(f,start,direction)"
        << "\n\tYou have to supply column vectors to this function"
        << "\n\tstart.nc():     " << start.nc()
        << "\n\tdirection.nc(): " << direction.nc()
        << "\n\tstart.nr():     " << start.nr()
        << "\n\tdirection.nr(): " << direction.nr()
    );
    return line_search_funct2<funct,T>(f,start,direction,f_out);
}

template <typename funct, typename EXP1, typename EXP2, typename T>
struct clamped_function_object2
{
    clamped_function_object2(
        const funct& f_,
        const dlib::matrix_exp<EXP1>& x_lower_,
        const dlib::matrix_exp<EXP2>& x_upper_,
        const T& start_,
        const T& direction_
    ) : f(f_), x_lower(x_lower_), x_upper(x_upper_), start(start_), direction(direction_)
    {
    }

    double operator() (
        const double& x
    ) const
    {
        T diff = clamp(start + x * direction, x_lower, x_upper) - start;
        Jacobian::projectToNullSpace(start, diff);
        return f(start + diff);
    }

    const funct& f;
    const dlib::matrix_exp<EXP1>& x_lower;
    const dlib::matrix_exp<EXP2>& x_upper;
    const T& start;
    const T& direction;
};

template <typename funct, typename EXP1, typename EXP2, typename T>
clamped_function_object2<funct,EXP1,EXP2,T> clamp_function2(
    const funct& f,
    const dlib::matrix_exp<EXP1>& x_lower,
    const dlib::matrix_exp<EXP2>& x_upper,
    const T& start,
    const T& direction
) { return clamped_function_object2<funct,EXP1,EXP2,T>(f,x_lower,x_upper,start,direction); }
*/

#endif //_CLASS_JACOBIAN
