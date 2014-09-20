/*
 * contactPoint.cpp
 *
 *  Created on: Sep 11, 2013
 *      Author: chpark
 */

#include <itomp_cio_planner/model/itomp_robot_model.h>
#include <itomp_cio_planner/util/vector_util.h>
#include <itomp_cio_planner/contact/contact_point.h>
#include <itomp_cio_planner/contact/ground_manager.h>

using namespace std;

namespace itomp_cio_planner
{

ContactPoint::ContactPoint(const string& link_name, unsigned rbdl_body_id,
		const std::vector<unsigned int>& contact_point_rbdl_ids) :
		link_name_(link_name), rbdl_body_id_(rbdl_body_id), contact_point_rbdl_ids_(contact_point_rbdl_ids)
{

}

ContactPoint::~ContactPoint()
{

}

}
