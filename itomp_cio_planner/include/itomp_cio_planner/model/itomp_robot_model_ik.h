#ifndef ITOMP_ROBOT_MODEL_IK_H_
#define ITOMP_ROBOT_MODEL_IK_H_

#include <itomp_cio_planner/common.h>
#include <itomp_cio_planner/model/itomp_planning_group.h>
#include <itomp_cio_planner/model/itomp_robot_joint.h>
#include <moveit/robot_model/robot_model.h>
#include <sensor_msgs/JointState.h>

namespace itomp_cio_planner
{

struct ItompRobotModelIKData
{
    Eigen::Vector3d root_to_hip;
    Eigen::Vector3d hip_to_knee;
    Eigen::Vector3d knee_to_ankle;
    Eigen::Vector3d ankle_to_ee;

    double h1;
    double h2;
    double ph1;
    double ph2;

    double max_stretch;

    Eigen::Affine3d ee_to_root;
};

class ItompRobotModelIKHelper : public Singleton<ItompRobotModelIKHelper>
{
public:

    ItompRobotModelIKHelper();
    virtual ~ItompRobotModelIKHelper();

    void initialize(const robot_model::RobotModelConstPtr& moveit_robot_model);

    bool getGroupEndeffectorPos(const std::string& group_name, const robot_state::RobotState& robot_state, Eigen::Affine3d& ee_pose) const;
    bool computeStandIKState(robot_state::RobotState& robot_state, Eigen::Affine3d& root_pose, const Eigen::Affine3d& left_foot_pose, const Eigen::Affine3d& right_foot_pose) const;
    bool getRootPose(const std::string& group_name, const Eigen::Affine3d& ee_pose, Eigen::Affine3d& root_pose) const;

private:
    void initializeIKData(const std::string& group_name) const;
    bool computeInverseKinematics(const std::string& group_name, const Eigen::Affine3d& root_pose, const Eigen::Affine3d& dest_pose,
                                  std::vector<double>& joint_values) const;
    bool adjustRootZ(const std::string& group_name, Eigen::Affine3d& root_pose, const Eigen::Affine3d& dest_pose) const;

    mutable std::map<std::string, ItompRobotModelIKData> ik_data_map_;
    robot_model::RobotModelConstPtr moveit_robot_model_;
};
ITOMP_DEFINE_SHARED_POINTERS(ItompRobotModelIKHelper)



}
#endif
