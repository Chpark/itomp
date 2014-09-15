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

ContactPoint::ContactPoint(const string& link_name, unsigned rbdl_body_id) :
		link_name_(link_name), rbdl_body_id_(rbdl_body_id)
{
	link_name_ = link_name;
}

ContactPoint::~ContactPoint()
{

}

}
