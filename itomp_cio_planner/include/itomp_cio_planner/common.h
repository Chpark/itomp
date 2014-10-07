#ifndef COMMON_H_
#define COMMON_H_

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <map>
#include <string>
#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <sstream>
#include <stddef.h>
#include <math.h>
#include <omp.h>

#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/thread.hpp>

#include <Eigen/Core>

#include <itomp_cio_planner/util/itomp_debug.h>

inline int getNumParallelThreads()
{
	//return omp_get_max_threads();
	return 1;
}

#endif
