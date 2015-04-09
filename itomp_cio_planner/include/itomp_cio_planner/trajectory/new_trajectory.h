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
    NewTrajectory(const NewTrajectory& trajectory);

    virtual ~NewTrajectory();
    virtual NewTrajectory* clone() const = 0;

    unsigned int getNumPoints() const;
    unsigned int getNumElements() const;
    const std::string& getName() const;

    virtual void printTrajectory(std::ostream& out_stream, int point_start = 0, int point_end = -1) const = 0;

protected:
    std::string name_;
    unsigned int num_points_; /**< Number of points in the trajectory */
    unsigned int num_elements_; /**< Number of elements in each trajectory point */
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
