/*
 * planning_info.h
 *
 *  Created on: Mar 21, 2014
 *      Author: cheonhyeonpark
 */

#ifndef PLANNING_INFO_H_
#define PLANNING_INFO_H_

namespace itomp_cio_planner
{

class PlanningInfo
{
public:
	PlanningInfo() :
		time(0), iterations(0), cost(0), success(0)
	{
	}

	PlanningInfo& operator+=(const PlanningInfo &rhs)
	{
		time += rhs.time;
		iterations += rhs.iterations;
		cost += rhs.cost;
		success += rhs.success;
		return *this;
	}
	double time;
	int iterations;
	double cost;
	int success;
};

}
#endif /* PLANNING_INFO_H_ */
