#ifndef ITOMP_TRAJECTORY_H_
#define ITOMP_TRAJECTORY_H_

#include <itomp_cio_planner/common.h>
#include <itomp_cio_planner/trajectory/composite_trajectory.h>
#include <itomp_cio_planner/trajectory/element_trajectory.h>
#include <itomp_cio_planner/model/itomp_robot_model.h>
#include <sensor_msgs/JointState.h>
#include <moveit_msgs/TrajectoryConstraints.h>
#include "dlib/matrix.h"

namespace itomp_cio_planner
{
ITOMP_FORWARD_DECL(ItompTrajectory)

struct ItompTrajectoryIndex
{
    unsigned int indices[4]; // [pos/vel/acc, joint/contact pos/contact force, point, element]
};

class ItompTrajectory : public CompositeTrajectory
{
public:
    enum COMPONENT_TYPE
    {
        COMPONENT_TYPE_POSITION = 0,
        COMPONENT_TYPE_VELOCITY,
        COMPONENT_TYPE_ACCELERATION,
        COMPONENT_TYPE_NUM,
    };
    enum SUB_COMPONENT_TYPE
    {
        SUB_COMPONENT_TYPE_JOINT = 0,
        SUB_COMPONENT_TYPE_CONTACT_POSITION,
        SUB_COMPONENT_TYPE_CONTACT_FORCE,
        SUB_COMPONENT_TYPE_NUM,
    };

    typedef dlib::matrix<double, 0, 1> ParameterVector;
    typedef std::vector<ItompTrajectoryIndex> ParameterMap;

    virtual ~ItompTrajectory();

    void computeParameterToTrajectoryIndexMap(const ItompRobotModelConstPtr& robot_model,
                                              const ItompPlanningGroupConstPtr& planning_group);

    void setFromParameter(ParameterVector& parameter);
    void directChangeForDerivativeComputation(unsigned int parameter_index, double value,
            unsigned int& trajectory_point_begin, unsigned int& trajectory_point_end,
            bool backup = true);

    void setStartState(const sensor_msgs::JointState& joint_state,
                       const ItompRobotModelConstPtr& robot_model);
    void setGoalState(const sensor_msgs::JointState& joint_goal_state,
                      const ItompPlanningGroupConstPtr& planning_group,
                      const ItompRobotModelConstPtr& robot_model,
                      const moveit_msgs::TrajectoryConstraints& trajectory_constraints);

    ElementTrajectoryPtr& getElementTrajectory(unsigned int component, unsigned int sub_component);
    ElementTrajectoryConstPtr getElementTrajectory(unsigned int component, unsigned int sub_component) const;

    void backupTrajectory(const ItompTrajectoryIndex& index);
    void restoreTrajectory();

protected:
    ItompTrajectory(const std::string& name, unsigned int num_points, const std::vector<NewTrajectoryPtr>& components,
                    unsigned int num_keyframes, unsigned int keyframe_interval, double duration, double discretization);
    void interpolateKeyframes();
    void interpolateTrajectory(unsigned int trajectory_point_begin, unsigned int trajectory_point_end,
                               const ItompTrajectoryIndex& index);

    void interpolateStartEnd(const std::vector<unsigned int>& group_rbdl_indices);
    void interpolateInputTrajectory(const std::vector<unsigned int>& group_rbdl_indices,
                                    const ItompPlanningGroupConstPtr& planning_group,
                                    const moveit_msgs::TrajectoryConstraints& trajectory_constraints);

    unsigned int num_keyframes_;
    unsigned int keyframe_interval_;
    double duration_;
    double discretization_;

    ParameterMap parameter_to_index_map_;

    ElementTrajectoryPtr element_trajectories_[COMPONENT_TYPE_NUM][SUB_COMPONENT_TYPE_NUM];

    Eigen::MatrixXd backup_trajectory_[COMPONENT_TYPE_NUM];
    ItompTrajectoryIndex backup_index_;

    friend class TrajectoryFactory;
};
ITOMP_DEFINE_SHARED_POINTERS(ItompTrajectory)

///////////////////////// inline functions follow //////////////////////

inline ElementTrajectoryPtr& ItompTrajectory::getElementTrajectory(unsigned int component, unsigned int sub_component)
{
    return element_trajectories_[component][sub_component];
}

inline ElementTrajectoryConstPtr ItompTrajectory::getElementTrajectory(unsigned int component, unsigned int sub_component) const
{
    return boost::const_pointer_cast<const ElementTrajectory>(element_trajectories_[component][sub_component]);
}

}
#endif
