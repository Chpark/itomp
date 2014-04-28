/*
 * minJerkTrajectory.h
 *
 *  Created on: Sep 19, 2013
 *      Author: cheonhyeonpark
 */

#ifndef MINJERKTRAJECTORY_H_
#define MINJERKTRAJECTORY_H_

class MinJerkTrajectory
{
public:
	MinJerkTrajectory(double x0, double v0, double a0, double x1, double v1, double a1);
	virtual ~MinJerkTrajectory();

	double operator()(double t);

private:
	double x0_, x1_;
	double v0_, v1_;
	double a0_, a1_;
	double coeff[6];
};

#endif /* MINJERKTRAJECTORY_H_ */
