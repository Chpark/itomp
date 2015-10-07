#ifndef ITOMP_ROBOT_JOINT_H_
#define ITOMP_ROBOT_JOINT_H_

//#include <ros/ros.h>
#include <itomp_cio_planner/common.h>
#include <kdl/tree.hpp>

namespace itomp_cio_planner
{
class ItompRobotJoint
{
public:
	int group_joint_index_; /**< Joint index in a planning group*/
	std::string joint_name_; /**< Name of the joint */
	std::string link_name_; /**< Name of the corresponding link (from planning.yaml) */
	bool wrap_around_; /**< Does this joint wrap-around? */
	bool has_joint_limits_; /**< Are there joint limits? */
	double joint_limit_min_; /**< Minimum joint angle value */
	double joint_limit_max_; /**< Maximum joint angle value */

	unsigned int rbdl_joint_index_; // q
	std::vector<unsigned int> rbdl_affected_body_ids_; // used in partial FK
};
}
#endif
