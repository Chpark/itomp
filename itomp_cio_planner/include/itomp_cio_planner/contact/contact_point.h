/*
 * contactPoint.h
 *
 *  Created on: Sep 11, 2013
 *      Author: chpark
 */

#ifndef CONTACTPOINT_H_
#define CONTACTPOINT_H_

#include <itomp_cio_planner/common.h>

namespace itomp_cio_planner
{
class ItompRobotModel;

class ContactPoint
{
public:
	ContactPoint(const std::string& link_name, unsigned int rbdl_body_id);
	virtual ~ContactPoint();

	const std::string& getLinkName() const;
	unsigned getRBDLBodyId() const;

private:
	std::string link_name_;
	unsigned rbdl_body_id_;
};

inline const std::string& ContactPoint::getLinkName() const
{
	return link_name_;
}

inline unsigned ContactPoint::getRBDLBodyId() const
{
	return rbdl_body_id_;
}


};

#endif /* CONTACTPOINT_H_ */
