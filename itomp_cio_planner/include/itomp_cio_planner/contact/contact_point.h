/*
 * contactPoint.h
 *
 *  Created on: Sep 11, 2013
 *      Author: chpark
 */

#ifndef CONTACT_POINT_H_
#define CONTACT_POINT_H_

#include <itomp_cio_planner/common.h>

namespace itomp_cio_planner
{
class ItompRobotModel;

class ContactPoint
{
public:
	ContactPoint(const std::string& link_name, unsigned int rbdl_body_id, const std::vector<unsigned int>& contact_point_rbdl_ids);
	virtual ~ContactPoint();

	const std::string& getLinkName() const;
	unsigned getRBDLBodyId() const;
	unsigned int getContactPointRBDLIds(int point_index) const;

private:
	std::string link_name_;
	unsigned rbdl_body_id_;
	std::vector<unsigned int> contact_point_rbdl_ids_;
};

inline const std::string& ContactPoint::getLinkName() const
{
	return link_name_;
}

inline unsigned ContactPoint::getRBDLBodyId() const
{
	return rbdl_body_id_;
}

inline unsigned int ContactPoint::getContactPointRBDLIds(int point_index) const
{
	return contact_point_rbdl_ids_[point_index];
}


};

#endif /* CONTACT_POINT_H_ */
