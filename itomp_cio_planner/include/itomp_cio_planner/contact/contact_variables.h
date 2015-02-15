#ifndef CONTACT_VARIABLES_H_
#define CONTACT_VARIABLES_H_

#include <itomp_cio_planner/common.h>
#include <itomp_cio_planner/contact/contact_point.h>
#include <itomp_cio_planner/util/exponential_map.h>
#include <rbdl/Model.h>

namespace itomp_cio_planner
{

class ContactVariables
{
public:
	ContactVariables();
	virtual ~ContactVariables()
	{
	}

	// directly from optimization parameters
	void setVariable(double value);
	double getVariable() const;
	double getRawVariable() const;

	void setPosition(const Eigen::Vector3d& position);
	Eigen::Vector3d getPosition() const;

	void setOrientation(const Eigen::Vector3d& orientation);
	Eigen::Vector3d getOrientation() const;

	void setPointForce(int point_index, const Eigen::Vector3d& point_force);
	Eigen::Vector3d getPointForce(int point_index) const;

	Eigen::VectorXd serialized_position_;
	Eigen::VectorXd serialized_forces_;

	/////////////////////

	void ComputeProjectedPointPositions(
		const Eigen::Vector3d& projected_position,
		const Eigen::Vector3d& projected_orientation,
		const RigidBodyDynamics::Model& model,
		const ContactPoint& contact_point);

	// from FK
	Eigen::Vector3d projected_position_;
	Eigen::Vector3d projected_orientation_;
	std::vector<Eigen::Vector3d> projected_point_positions_;

};

/////////////////////////////

inline ContactVariables::ContactVariables()
{
	serialized_position_.resize(7);
	serialized_forces_.resize(NUM_ENDEFFECTOR_CONTACT_POINTS * 3);
	projected_point_positions_.resize(NUM_ENDEFFECTOR_CONTACT_POINTS);
}

inline void ContactVariables::setVariable(double value)
{
	serialized_position_(0) = value;
}
inline double ContactVariables::getVariable() const
{
    double variable = serialized_position_(0);
    //variable = std::abs(variable);
    //variable = 0.5 * tanh(4 * variable - 2) + 0.5;
	return variable;
}
inline double ContactVariables::getRawVariable() const
{
	return serialized_position_(0);
}

inline void ContactVariables::setPosition(const Eigen::Vector3d& position)
{
	serialized_position_.block(1, 0, 3, 1) = position;
}
inline Eigen::Vector3d ContactVariables::getPosition() const
{
	return serialized_position_.block(1, 0, 3, 1);
}

inline void ContactVariables::setOrientation(const Eigen::Vector3d& orientation)
{
	serialized_position_.block(4, 0, 3, 1) = orientation;
}
inline Eigen::Vector3d ContactVariables::getOrientation() const
{
	return serialized_position_.block(4, 0, 3, 1);
}

inline void ContactVariables::setPointForce(int point_index,
		const Eigen::Vector3d& point_force)
{
    serialized_forces_.block(3 * point_index, 0, 3, 1) = point_force;
}
inline Eigen::Vector3d ContactVariables::getPointForce(int point_index) const
{
    Eigen::Vector3d force =	serialized_forces_.block(3 * point_index, 0, 3, 1);

    // TODO: scale = mass * gravity

    double scale = 60 * 9.8;
	force *= scale;

	return force;
}

inline void ContactVariables::ComputeProjectedPointPositions(
	const Eigen::Vector3d& projected_position,
	const Eigen::Vector3d& projected_orientation,
	const RigidBodyDynamics::Model& model,
	const ContactPoint& contact_point)
{
	projected_position_ = projected_position;
	projected_orientation_ = projected_orientation;
	for (int i = 0; i < NUM_ENDEFFECTOR_CONTACT_POINTS; ++i)
	{
		RigidBodyDynamics::Math::SpatialTransform x_base_lambda(
			exponential_map::ExponentialMapToRotation(
				projected_orientation), projected_position);
		RigidBodyDynamics::Math::SpatialTransform x_base =
			model.X_lambda[contact_point.getContactPointRBDLIds(i)]
			* x_base_lambda;

		projected_point_positions_[i] = x_base.r;

	}
}

}

#endif
