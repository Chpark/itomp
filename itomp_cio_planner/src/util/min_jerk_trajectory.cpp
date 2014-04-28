/*
 * minJerkTrajectory.cpp
 *
 *  Created on: Sep 19, 2013
 *      Author: cheonhyeonpark
 */

#include <itomp_cio_planner/util/min_jerk_trajectory.h>

MinJerkTrajectory::MinJerkTrajectory(double x0, double v0, double a0, double x1, double v1, double a1) :
	x0_(x0), x1_(x1), v0_(v0), v1_(v1), a0_(a0), a1_(a1)
{
	// TODO: for now, x1 = v1 = a1 = 0
	coeff[0] = x0_;
	coeff[1] = v0_;
	coeff[2] = 0.5 * a0_;
	coeff[3] = -1.5 * a0_ - 6.0 * v0_ - 10.0 * x0_ + 0.5 * a1_ - 4.0 * v1_ + 10.0 * x1_;
	coeff[4] = 1.5 * a0_ + 8.0 * v0_ + 15.0 * x0_ - 1.0 * a1_ + 7.0 * v1_ -15.0 * x1_;
	coeff[5] = -0.5 * a0_ - 3.0 * v0_ - 6.0 * x0_ + 0.5 * a1_ - 3.0 * v1_ + 6.0 * x1_;
}

MinJerkTrajectory::~MinJerkTrajectory()
{

}

double MinJerkTrajectory::operator()(double t)
{
	double ret = coeff[0];
	double mul = t;
	for (int i = 1; i < 6; ++i)
	{
		ret += mul * coeff[i];
		mul *= t;
	}
	return ret;
}
