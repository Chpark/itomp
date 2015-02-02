#ifndef NEW_TRAJECTORY_H_
#define NEW_TRAJECTORY_H_

#include <itomp_cio_planner/common.h>

namespace itomp_cio_planner
{
ITOMP_FORWARD_DECL(NewTrajectory)

class NewTrajectory
{
public:
    // Construct a trajectory
    NewTrajectory(const std::string& name, unsigned int num_points);

    virtual ~NewTrajectory();

    unsigned int getNumPoints() const;
    unsigned int getNumElements() const;
    const std::string& getName() const;

    virtual void printTrajectory() const = 0;

protected:
    unsigned int num_elements_; /**< Number of elements in each trajectory point */
    unsigned int num_points_; /**< Number of points in the trajectory */

    std::string name_;
};
ITOMP_DEFINE_SHARED_POINTERS(NewTrajectory)

///////////////////////// inline functions follow //////////////////////

inline unsigned int NewTrajectory::getNumPoints() const
{
    return num_points_;
}

inline unsigned int NewTrajectory::getNumElements() const
{
    return num_elements_;
}

inline const std::string& NewTrajectory::getName() const
{
    return name_;
}

}
#endif
