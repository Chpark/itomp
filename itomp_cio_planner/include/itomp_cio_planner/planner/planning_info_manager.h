/*
 * planning_info_manager.h
 *
 *  Created on: Mar 21, 2014
 *      Author: cheonhyeonpark
 */

#ifndef PLANNING_INFO_MANAGER_H_
#define PLANNING_INFO_MANAGER_H_

#include <itomp_cio_planner/common.h>

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

class PlanningInfoManager
{
public:
  PlanningInfoManager() {}

  void reset(int trials, int component);
  void write(int trials, int component, const PlanningInfo& info);
  void printSummary() const;

protected:
  std::vector<std::vector<PlanningInfo> > planning_info_;
};

}
#endif /* PLANNING_INFO_H_ */
