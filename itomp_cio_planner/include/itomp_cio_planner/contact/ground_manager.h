/*
 * groundManager.h
 *
 *  Created on: Sep 11, 2013
 *      Author: chpark
 */

#ifndef GROUNDMANAGER_H_
#define GROUNDMANAGER_H_

#include <kdl/frames.hpp>

namespace itomp_cio_planner
{
class GroundManager
{
public:
	virtual ~GroundManager();
	void init();

	static GroundManager& getInstance() { return instance_; }
	void getNearestGroundPosition(const KDL::Vector& in, KDL::Vector& out, KDL::Vector& normal, bool exact = false) const;
	void getSafeGroundPosition(const KDL::Vector& in, KDL::Vector& out) const;

private:
	GroundManager();

	static GroundManager instance_;
};

};

#endif /* GROUNDMANAGER_H_ */
