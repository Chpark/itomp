#ifndef PERFORMANCE_PROFILER_H_
#define PERFORMANCE_PROFILER_H_

#include <omp.h>

namespace itomp_cio_planner
{
class PerformanceProfiler: public Singleton<PerformanceProfiler>
{
public:
	PerformanceProfiler() :
			num_threads_(1), get_time_func_(NULL)
	{
	}
	virtual ~PerformanceProfiler()
	{
	}

	// non thread-safe functions. should be called after omp_set_num_threads()
	void initialize(double (*get_time_func)());
	void addEntry(const char* entry_name);

	// clear last iteration
	void startIteration();

	void printIterationTime(bool show_percentage = false);
	void printTotalTime(bool show_percentage = false);

	// thread-safe functions (in openMP)
	void startTimer(const char* entry_name);
	void endTimer(const char* entry_name);

protected:
	class Entry
	{
	public:
		Entry(int num_threads);
		void initialize(int num_threads);

		void startIteration();

		void startTimer(double (*get_time_func)());
		void endTimer(double (*get_time_func)());

		double getTotalElapsed() const;
		double getIterationElapsed() const;

		std::vector<double> timer_start_time_;

		std::vector<double> total_elapsed_;
		std::vector<double> iteration_elpased_;
	};
	//double getROSWallTime();

	std::map<std::string, Entry> entries_;
	int num_threads_;
	double (*get_time_func_)();
};

inline void PerformanceProfiler::initialize(double (*get_time_func)())
{
	get_time_func_ = get_time_func;

	num_threads_ = omp_get_num_threads();
	for (std::map<std::string, Entry>::iterator it = entries_.begin();
			it != entries_.end(); ++it)
		it->second.initialize(num_threads_);
}

inline void PerformanceProfiler::addEntry(const char* entry_name)
{
	entries_.insert(std::make_pair(entry_name, Entry(num_threads_)));
}

inline void PerformanceProfiler::startIteration()
{
	for (std::map<std::string, Entry>::iterator it = entries_.begin();
			it != entries_.end(); ++it)
	{
		it->second.startIteration();
	}
}

inline void PerformanceProfiler::printIterationTime(bool show_percentage)
{
	for (std::map<std::string, Entry>::iterator it = entries_.begin();
			it != entries_.end(); ++it)
	{
		printf("%s ", it->first.c_str());
	}
	printf("\n");

	double sum = 0.0;
	for (std::map<std::string, Entry>::iterator it = entries_.begin();
			it != entries_.end(); ++it)
	{
		double elpased = it->second.getIterationElapsed();
		printf("%f ", elpased);
		sum += elpased;
	}
	printf("\n");

	if (show_percentage)
	{
		for (std::map<std::string, Entry>::iterator it = entries_.begin();
				it != entries_.end(); ++it)
		{
			double percentage = it->second.getIterationElapsed() / sum * 100.0;
			printf("%.2f ", percentage);
		}
		printf("\n");
	}
}

inline void PerformanceProfiler::printTotalTime(bool show_percentage)
{
	for (std::map<std::string, Entry>::iterator it = entries_.begin();
			it != entries_.end(); ++it)
	{
		printf("%s ", it->first.c_str());
	}
	printf("\n");

	double sum = 0.0;
	for (std::map<std::string, Entry>::iterator it = entries_.begin();
			it != entries_.end(); ++it)
	{
		double elpased = it->second.getTotalElapsed();
		printf("%f ", elpased);
		sum += elpased;
	}
	printf("\n");

	if (show_percentage)
	{
		for (std::map<std::string, Entry>::iterator it = entries_.begin();
				it != entries_.end(); ++it)
		{
			double percentage = it->second.getTotalElapsed() / sum * 100.0;
			printf("%.2f ", percentage);
		}
		printf("\n");
	}
}

// thread-safe
inline void PerformanceProfiler::startTimer(const char* entry_name)
{
	entries_.find(entry_name)->second.startTimer(get_time_func_);
}

inline void PerformanceProfiler::endTimer(const char* entry_name)
{
	entries_.find(entry_name)->second.endTimer(get_time_func_);
}

/*
inline double PerformanceProfiler::getROSWallTime()
{
	return ros::WallTime::now().toSec();
}
*/

////////////////////////////////////////////////////////////////////////////////
inline PerformanceProfiler::Entry::Entry(int num_threads)
{
	initialize(num_threads);
}

inline void PerformanceProfiler::Entry::initialize(int num_threads)
{
	timer_start_time_.resize(num_threads, 0.0);
	total_elapsed_.resize(num_threads, 0.0);
	iteration_elpased_.resize(num_threads, 0.0);
}

inline void PerformanceProfiler::Entry::startIteration()
{
	for (int i = 0; i < iteration_elpased_.size(); ++i)
		iteration_elpased_[i] = 0.0;
}

inline void PerformanceProfiler::Entry::startTimer(double (*get_time_func)())
{
	int thread_index = omp_get_thread_num();
	timer_start_time_[thread_index] = (*get_time_func)();
}

inline void PerformanceProfiler::Entry::endTimer(double (*get_time_func)())
{
	int thread_index = omp_get_thread_num();
	double elapsed = (*get_time_func)() - timer_start_time_[thread_index];
	iteration_elpased_[thread_index] += elapsed;
	total_elapsed_[thread_index] += elapsed;
}

inline double PerformanceProfiler::Entry::getTotalElapsed() const
{
	double sum = 0.0;
	for (int i = 0; i < total_elapsed_.size(); ++i)
		sum += total_elapsed_[i];
	return sum;
}

inline double PerformanceProfiler::Entry::getIterationElapsed() const
{
	double sum = 0.0;
	for (int i = 0; i < iteration_elpased_.size(); ++i)
		sum += iteration_elpased_[i];
	return sum;
}

}

#endif /* PERFORMANCE_PROFILER_H_ */
