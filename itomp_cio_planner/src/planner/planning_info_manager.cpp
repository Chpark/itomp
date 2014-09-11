#include <itomp_cio_planner/planner/planning_info_manager.h>
#include <ros/ros.h>

namespace itomp_cio_planner
{

void PlanningInfoManager::reset(int trials, int component)
{
  planning_info_.clear();
  planning_info_.resize(trials, std::vector<PlanningInfo>(component));
}

void PlanningInfoManager::write(int trials, int component, const PlanningInfo& info)
{
  planning_info_[trials][component] = info;
}

void PlanningInfoManager::printSummary() const
{
  int num_plannings = planning_info_.size();
  int num_components = planning_info_[0].size();

  std::vector<PlanningInfo> summary(num_components);
  PlanningInfo sum_of_sum;
  for (int j = 0; j < num_components; ++j)
  {
    for (int i = 0; i < num_plannings; ++i)
    {
      summary[j] += planning_info_[i][j];
    }
    sum_of_sum += summary[j];
  }

  // compute success rate
  // if a component fails, that trail fails.
  int num_success = 0;
  for (int i = 0; i < num_plannings; ++i)
  {
    bool failed = false;
    for (int j = 0; j < num_components; ++j)
    {
      if (planning_info_[i][j].success == 0)
      {
        failed = true;
        break;
      }
    }
    if (!failed)
    {
      ++num_success;
    }
  }

  ROS_INFO("\nPlannings info");

  ROS_INFO("%d Trials, %d components", num_plannings, num_components);
  ROS_INFO("Component #Iter Time Cost S-Rate");
  for (int j = 0; j < num_components; ++j)
  {
    ROS_INFO("%d %f %f %f %f", j, ((double) summary[j].iterations) / num_plannings,
        ((double) summary[j].time) / num_plannings, ((double) summary[j].cost) / num_plannings,
        ((double) summary[j].success) / num_plannings);
  }
  ROS_INFO("Sum %f %f %f %f", ((double) sum_of_sum.iterations) / num_plannings, ((double) sum_of_sum.time) / num_plannings,
      ((double) sum_of_sum.cost) / num_plannings, ((double) num_success) / num_plannings);

  ROS_INFO("Component #Iter Time Cost");
  for (int i = 0; i < num_plannings; ++i)
  {
    double iterations_sum = 0, time_sum = 0, cost_sum = 0;
    for (int j = 0; j < num_components; ++j)
    {
      iterations_sum += planning_info_[i][j].iterations;
      time_sum += planning_info_[i][j].time;
      cost_sum += planning_info_[i][j].cost;
    }
    ROS_INFO("[%d] %f %f %f ", i, iterations_sum, time_sum, cost_sum);
  }
}

}
