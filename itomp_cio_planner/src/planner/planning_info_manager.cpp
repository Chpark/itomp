#include <itomp_cio_planner/planner/planning_info_manager.h>

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

  printf("%d Trials, %d components\n", num_plannings, num_components);
  printf("Component Iterations Time Smoothness SuccessRate\n");
  for (int j = 0; j < num_components; ++j)
  {
    printf("%d %f %f %f %f\n", j, ((double) summary[j].iterations) / num_plannings,
        ((double) summary[j].time) / num_plannings, ((double) summary[j].cost) / num_plannings,
        ((double) summary[j].success) / num_plannings);
  }
  printf("Sum %f %f %f %f\n", ((double) sum_of_sum.iterations) / num_plannings, ((double) sum_of_sum.time) / num_plannings,
      ((double) sum_of_sum.cost) / num_plannings, ((double) num_success) / num_plannings);
  printf("\n");

  printf("plannings info\n");
  printf("Component Iterations Time Smoothness SuccessRate\n");
  for (int i = 0; i < num_plannings; ++i)
  {
    double iterationsSum = 0, timeSum = 0, costSum = 0;
    for (int j = 0; j < num_components; ++j)
    {
      iterationsSum += planning_info_[i][j].iterations;
      timeSum += planning_info_[i][j].time;
      costSum += planning_info_[i][j].cost;
    }
    printf("[%d] %f %f %f \n", i, iterationsSum, timeSum, costSum);
  }
}

}
