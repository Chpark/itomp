#ifndef ITOMP_DEBUG_H_
#define ITOMP_DEBUG_H_

#include <kdl/jntarray.hpp>
#include <itomp_cio_planner/util/performance_profiler.h>

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

//#define USE_TIME_PROFILER
#ifdef USE_TIME_PROFILER
#define TIME_PROFILER_INIT(get_time_func, num_threads) PerformanceProfiler::getInstance()->initialize(get_time_func, num_threads);
#define TIME_PROFILER_ADD_ENTRY(name) PerformanceProfiler::getInstance()->addEntry(#name);
#define TIME_PROFILER_START_ITERATION PerformanceProfiler::getInstance()->startIteration();
#define TIME_PROFILER_START_TIMER(name) PerformanceProfiler::getInstance()->startTimer(#name);
#define TIME_PROFILER_END_TIMER(name) PerformanceProfiler::getInstance()->endTimer(#name);
#define TIME_PROFILER_PRINT_TOTAL_TIME(show_percentage) PerformanceProfiler::getInstance()->printTotalTime(show_percentage);
#define TIME_PROFILER_PRINT_ITERATION_TIME(show_percentage) PerformanceProfiler::getInstance()->printIterationTime(show_percentage);
#else
#define TIME_PROFILER_INIT(get_time_func, num_threads)
#define TIME_PROFILER_ADD_ENTRY(name)
#define TIME_PROFILER_START_ITERATION
#define TIME_PROFILER_START_TIMER(name)
#define TIME_PROFILER_END_TIMER(name)
#define TIME_PROFILER_PRINT_TOTAL_TIME(show_percentage)
#define TIME_PROFILER_PRINT_ITERATION_TIME(show_percentage)
#endif

}
#endif
