#ifndef ELEMENT_TRAJECTORY_H_
#define ELEMENT_TRAJECTORY_H_

#include <itomp_cio_planner/common.h>
#include <itomp_cio_planner/trajectory/new_trajectory.h>

namespace itomp_cio_planner
{
ITOMP_FORWARD_DECL(ElementTrajectory)

class ElementTrajectory : public NewTrajectory
{
public:
    // Construct a trajectory
    ElementTrajectory(const std::string& name, unsigned int num_points, unsigned int num_elements);

    virtual ~ElementTrajectory();

    Eigen::MatrixXd::RowXpr getTrajectoryPoint(int point);
    Eigen::MatrixXd::ConstRowXpr getTrajectoryPoint(int point) const;

    double& operator()(unsigned int point, unsigned int element);
    double operator()(unsigned int point, unsigned int element) const;

    double& at(unsigned int point, unsigned int element);
    double at(unsigned int point, unsigned int element) const;

    Eigen::MatrixXd& getData();
    const Eigen::MatrixXd& getData() const;

    virtual void printTrajectory() const;

protected:
    void allocate(); /**< \brief Allocates memory for the trajectory */

    Eigen::MatrixXd trajectory_data_; /**< Storage for the actual trajectory */

};
ITOMP_DEFINE_SHARED_POINTERS(ElementTrajectory)

///////////////////////// inline functions follow //////////////////////

inline Eigen::MatrixXd::RowXpr ElementTrajectory::getTrajectoryPoint(int point)
{
    return trajectory_data_.row(point);
}

inline Eigen::MatrixXd::ConstRowXpr ElementTrajectory::getTrajectoryPoint(int point) const
{
    return trajectory_data_.row(point);
}

inline double& ElementTrajectory::operator()(unsigned int point, unsigned int element)
{
    return trajectory_data_(point, element);
}

inline double ElementTrajectory::operator()(unsigned int point, unsigned int element) const
{
    return trajectory_data_(point, element);
}

inline double& ElementTrajectory::at(unsigned int point, unsigned int element)
{
    return trajectory_data_(point, element);
}

inline double ElementTrajectory::at(unsigned int point, unsigned int element) const
{
    return trajectory_data_(point, element);
}

inline Eigen::MatrixXd& ElementTrajectory::getData()
{
    return trajectory_data_;
}

inline const Eigen::MatrixXd& ElementTrajectory::getData() const
{
    return trajectory_data_;
}

}
#endif
