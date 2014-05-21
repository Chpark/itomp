/*
 * point_to_triangle_projection.h
 *
 *  Created on: May 21, 2014
 *      Author: cheonhyeonpark
 */

#ifndef POINT_TO_TRIANGLE_PROJECTION_H_
#define POINT_TO_TRIANGLE_PROJECTION_H_

#include <itomp_cio_planner/common.h>
namespace itomp_cio_planner
{
Eigen::Vector3d ProjPoint2Triangle(const Eigen::Vector3d &p0, const
Eigen::Vector3d &p1, const Eigen::Vector3d &p2, const Eigen::Vector3d &sourcePosition);
} // namespace itomp_cio_planner
#endif /* POINT_TO_TRIANGLE_PROJECTION_H_ */
