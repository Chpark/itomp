#ifndef VECTOR_UTIL_H_
#define VECTOR_UTIL_H_

#include <itomp_cio_planner/common.h>
#include <itomp_cio_planner/util/differentiation_rules.h>

namespace itomp_cio_planner
{

template<typename KDLType, typename EigenType>
void kdlVecToEigenVec(std::vector<KDLType>& kdl_v, std::vector<Eigen::Map<EigenType> >& eigen_v, int rows, int cols)
{
	int size = kdl_v.size();
	eigen_v.clear();
	for (int i = 0; i < size; i++)
	{
		eigen_v.push_back(Eigen::Map<EigenType>(kdl_v[i].data, rows, cols));
	}
}

template<typename KDLType, typename EigenType>
void kdlVecVecToEigenVecVec(std::vector<std::vector<KDLType> >& kdl_vv,
		std::vector<std::vector<Eigen::Map<EigenType> > > & eigen_vv, int rows, int cols)
{
	int size = kdl_vv.size();
	eigen_vv.resize(size);
	for (int i = 0; i < size; i++)
	{
		kdlVecToEigenVec(kdl_vv[i], eigen_vv[i], rows, cols);
	}
}

template<typename VecType>
void getVectorVelocities(int start, int end, double discretization, const std::vector<VecType>& pos, std::vector<
		VecType>& vel, const VecType& zeroVec)
{
	const double invTime = 1.0 / discretization;

	for (int point = start; point <= end; ++point)
	{
		vel[point] = zeroVec;

		for (int k = -DIFF_RULE_LENGTH / 2; k <= DIFF_RULE_LENGTH / 2; k++)
		{
			vel[point] += (invTime * DIFF_RULES[DIFF_RULE_VELOCITY][k + DIFF_RULE_LENGTH / 2]) * pos[point + k];
		}
	}
}

template<typename VecType>
void getVectorVelocitiesAndAccelerations(int start, int end, double discretization, const std::vector<VecType>& pos,
		std::vector<VecType>& vel, std::vector<VecType>& acc, const VecType& zeroVec)
{
	const double invTime = 1.0 / discretization;

	for (int point = start; point <= end; ++point)
	{
		acc[point] = vel[point] = zeroVec;

		for (int k = -DIFF_RULE_LENGTH / 2; k <= DIFF_RULE_LENGTH / 2; k++)
		{
			vel[point] += (invTime * DIFF_RULES[DIFF_RULE_VELOCITY][k + DIFF_RULE_LENGTH / 2]) * pos[point + k];
			acc[point] += (invTime * invTime * DIFF_RULES[DIFF_RULE_ACCELERATION][k + DIFF_RULE_LENGTH / 2]) * pos[point + k];
		}
	}
}

}
#endif
