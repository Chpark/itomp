#ifndef ITOMP_DEBUG_H_
#define ITOMP_DEBUG_H_

#include <kdl/jntarray.hpp>

namespace itomp_cio_planner
{
inline void debugJointArray(KDL::JntArray& joint_array)
{
  for (unsigned int i = 0; i < joint_array.rows(); i++)
  {
    std::cout << joint_array(i) << "\t";
  }
  std::cout << std::endl;
}

//#define MEASURE_TIME

#ifdef MEASURE_TIME
#define INIT_TIME_MEASUREMENT(N) std::vector<ros::Time> times;static int count=0;static double elapsed[N];for(int i=0;i<N;++i) elapsed[i]=0.0;
#define ADD_TIMER_POINT times.push_back(ros::Time::now());
#define UPDATE_TIME for(int i=0; i < times.size()-1; ++i) elapsed[i]=(times[i+1] - times[i]).toSec();
#define PRINT_TIME(name, c) if (++count % c == 0) for(int i=0; i < times.size()-1; ++i) printf("%s Timing %d : %f %f\n", #name, i, elapsed[i], elapsed[i] / count);
#else
#define INIT_TIME_MEASUREMENT(N)
#define ADD_TIMER_POINT
#define UPDATE_TIME
#define PRINT_TIME(name, c)
#endif

}
#endif
