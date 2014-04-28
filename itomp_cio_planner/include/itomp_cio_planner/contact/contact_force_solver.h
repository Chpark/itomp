/*
 * contactForceSolver.h
 *
 *  Created on: Sep 12, 2013
 *      Author: cheonhyeonpark
 */

#ifndef CONTACTFORCESOLVER_H_
#define CONTACTFORCESOLVER_H_

#include <kdl/frames.hpp>

void solveContactForces(double friction_coeff, std::vector<KDL::Vector>& contact_forces, std::vector<KDL::Vector>& contact_positions,
    const KDL::Wrench& wrench, const std::vector<double>& contact_values, const std::vector<KDL::Frame> contact_parent_frames);

void solveContactForces2(double friction_coeff, std::vector<KDL::Vector>& contact_forces, std::vector<KDL::Vector>& contact_positions,
    const KDL::Wrench& wrench, const std::vector<double>& contact_values, const std::vector<KDL::Frame> contact_frames);

#endif /* CONTACTFORCESOLVER_H_ */
