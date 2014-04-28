#ifndef ITOMP_DEBUG_H_
#define ITOMP_DEBUG_H_

#include <itomp_cio_planner/common.h>
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

}
#endif
