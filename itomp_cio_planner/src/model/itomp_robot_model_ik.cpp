#include <itomp_cio_planner/model/itomp_robot_model_ik.h>
#include <itomp_cio_planner/util/planning_parameters.h>
#include <itomp_cio_planner/model/rbdl_urdf_reader.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

using namespace std;

namespace itomp_cio_planner
{

ItompRobotModelIKHelper::ItompRobotModelIKHelper()
{
}

ItompRobotModelIKHelper::~ItompRobotModelIKHelper()
{
}

void ItompRobotModelIKHelper::initialize(const robot_model::RobotModelConstPtr& moveit_robot_model)
{
    moveit_robot_model_ = moveit_robot_model;
}

bool fuzzyEquals(double a, double b)
{
    const double eps = 1E-15;
    return std::abs(a - b) < eps * std::max(std::abs(a), std::abs(b));
}

double solveASinXPlusBCosXIsC(double a, double b, double c)
{
    // solve a sin x + b cos x = c
    double r = sqrt(a * a + b * b);
    double alpha = atan2(a, b);

    // cos (x-alpha) = c / r
    // x = alpha +- acos (c/r)

    double t = c / r;
    if (fuzzyEquals(t, 1.0))
        t = 1.0;
    else if (fuzzyEquals(t, -1.0))
        t = -1.0;

    double v = std::min(std::max(t, -1.0), 1.0);
    double x1 = alpha + std::acos(v);
    double x2 = alpha - std::acos(v);

    while (x1 > M_PI)
        x1 -= 2 * M_PI;
    while (x1 <= -M_PI)
        x1 += 2 * M_PI;

    while (x2 > M_PI)
        x2 -= 2 * M_PI;
    while (x2 <= -M_PI)
        x2 += 2 * M_PI;

    if (std::abs(x1) < std::abs(x2))
        return x1;
    else
        return x2;
}

void ItompRobotModelIKHelper::initializeIKData(const string &group_name) const
{
    ItompRobotModelIKData& ik_data = ik_data_map_[group_name];

    const std::vector<const robot_model::LinkModel*>& robot_link_models = moveit_robot_model_->getLinkModels();
    const std::vector<const robot_model::LinkModel*>& group_link_models = moveit_robot_model_->getJointModelGroup(group_name)->getLinkModels();
    std::string ee_group_name = moveit_robot_model_->getJointModelGroup(group_name)->getAttachedEndEffectorNames()[0];
    const robot_model::LinkModel* ee_link_model = moveit_robot_model_->getJointModelGroup(ee_group_name)->getLinkModels()[0];

    robot_state::RobotState zero_state(moveit_robot_model_);
    zero_state.setToDefaultValues();
    zero_state.updateLinkTransforms();

    const Eigen::Affine3d& root_transf = zero_state.getGlobalLinkTransform(robot_link_models[6]);
    const Eigen::Affine3d ee_transf = zero_state.getGlobalLinkTransform(ee_link_model);

    const Eigen::Affine3d& hip_transf = zero_state.getGlobalLinkTransform(group_link_models[0]);
    const Eigen::Affine3d& knee_transf = zero_state.getGlobalLinkTransform(group_link_models[3]);
    const Eigen::Affine3d& ankle_transf = zero_state.getGlobalLinkTransform(group_link_models[4]);

    ik_data.root_to_hip = hip_transf.translation() - root_transf.translation();
    ik_data.hip_to_knee = knee_transf.translation() - hip_transf.translation();
    ik_data.knee_to_ankle = ankle_transf.translation() - knee_transf.translation();
    ik_data.ankle_to_ee = ee_transf.translation() - ankle_transf.translation();

    ik_data.h1 = std::sqrt(ik_data.hip_to_knee.y() * ik_data.hip_to_knee.y() + ik_data.hip_to_knee.z() * ik_data.hip_to_knee.z());
    ik_data.h2 = std::sqrt(ik_data.knee_to_ankle.y() * ik_data.knee_to_ankle.y() + ik_data.knee_to_ankle.z() * ik_data.knee_to_ankle.z());
    ik_data.ph1 = std::atan2(-ik_data.hip_to_knee.y(), -ik_data.hip_to_knee.z());
    ik_data.ph2 = std::atan2(-ik_data.knee_to_ankle.y(), -ik_data.knee_to_ankle.z());

    ik_data.max_stretch = std::sqrt(ik_data.hip_to_knee(1) * ik_data.hip_to_knee(1) + ik_data.hip_to_knee(2) * ik_data.hip_to_knee(2)) +
                          std::sqrt(ik_data.knee_to_ankle(1) * ik_data.knee_to_ankle(1) + ik_data.knee_to_ankle(2) * ik_data.knee_to_ankle(2));

    ik_data.ee_to_root = ee_transf.inverse();
}

bool ItompRobotModelIKHelper::computeInverseKinematics(const std::string& group_name, const Eigen::Affine3d& root_pose, const Eigen::Affine3d& dest_pose,
        std::vector<double>& joint_values) const
{
    // compute IK for dest_pose of endeffector

    if (group_name != "left_leg" && group_name != "right_leg")
        return false;

    if (ik_data_map_.find(group_name) == ik_data_map_.end())
        initializeIKData(group_name);

    const ItompRobotModelIKData& ik_data = ik_data_map_[group_name];

    joint_values.resize(moveit_robot_model_->getJointModelGroup(group_name)->getVariableCount(), 0.0);

    // foot.p = ROOT.p + ROOT.M * (L0 + Rz0 * Ry1 * Rx2 * (L3 + Rx3 * (L4 + Rx4 * Ry5 * Rz6(I) * L7)))
    // foot.M = ROOT.M * Rz0 * Ry1 * Rx2 * Rx3 * Rx4 * Ry5 * Rz6(I)

    // set Rz6 (foot_yaw, foot) = I
    joint_values[6] = 0.0;

    // foot.p = ROOT.p + ROOT.M * (L0 + Rz0 * Ry1 * Rx2 * (L3 + Rx3 * (L4 + Rx4 * Ry5 * L7)))               (1)
    // foot.M = ROOT.M * Rz0 * Ry1 * Rx2 * Rx3 * Rx4 * Ry5                                                  (2)

    // from (1), ROOT.M^-1 * (foot.p - ROOT.p) - L0 = Rz0 * Ry1 * Rx2 * (L3 + Rx3 * (L4 + Rx4 * Ry5 * L7))  (3)
    // from (2), ROOT.M^-1 * foot.M * Ry5^-1 * Rx4^-1 * Rx3^-1 = Rz0 * Ry1 * Rx2                            (4)

    // from (3) and (4), (foot.p - ROOT.p) - ROOT.M * L0 = foot.M * Ry5^-1 * Rx4^-1 * Rx3^-1 * (L3 + Rx3 * (L4 + Rx4 * Ry5 * L7))
    //                                                   = foot.M * (Ry5^-1 * Rx4^-1 * (Rx3^-1 * L3 + L4) + L7)                                         (5)

    // from (5), foot.M^-1 * ((foot.p - ROOT.p) - ROOT.M * L0) - L7 = Ry5^-1 * Rx4^-1 * (Rx3^-1 * L3 + L4)                                              (6)

    // let foot.M^-1 * ((foot.p - ROOT.p) - ROOT.M * L0) - L7 = v
    // v = Ry5^-1 * Rx4^-1 * (Rx3^-1 * L3 + L4)                             (7)
    // Rx3 * Rx4 * Ry5 * v = (L3 + Rx3 * L4)                                (8)
    // length(v) = length(L3 + Rx3 * L4) : get theta3                       (9)

    // let Rx3^-1 * L3 + L4=k
    // from (8),
    //   cos5       0       sin5
    // ( sin4sin5   cos4    -sin4cos5 ) ( v ) = k                           (10)
    //  -cos4sin5   sin4    cos4cos5

    // solve v.x * cos5 + v.z * sin5 = k.x : get theta5                     (11)

    // solve (v.x * sin5 - v.z * cos5) * sin4 + v.y * cos4 = k.y : get theta4 (12)

    // let ROOT.M^-1 * foot.M * Ry5^-1 * Rx4^-1 * Rx3^-1 = M,
    // from (4), Rz0 * Ry1 * Rx2 = M                                        (13)
    //   c0c1       -s0c2+c0s1s2    s0s2+c0s1c2
    // ( s0c1       c0c2+s0s1s2     -c0s2+s0s1c2 ) = M                      (14)
    //   -s1        c1s2            c1c2

    // -s1  = M(2,0) : get theta1 (15)
    // s0c1 = M(1,0) : get theta0 (16)
    // c1s2 = M(2,1) : get theta2 (17)

    /////////////////////////////////////////////////////////////////////////////
    // let foot.M^-1 * ((foot.p - ROOT.p) - ROOT.M * L0) - L7 = v
    // v = Ry5^-1 * Rx4^-1 * (Rx3^-1 * L3 + L4)                             (7)
    // Rx3 * Rx4 * Ry5 * v = (L3 * Rx3 * L4)                                (8)
    // length(v) = length(L3 + Rx3 * L4) : get theta5                       (9)
    // v : hip to ankle
    // ignore x-axis diff
    // Use law of cosine for yz plane
    {
        const Eigen::Vector3d v = dest_pose.linear().inverse() *
                                  ((dest_pose.translation() - root_pose.translation()) - root_pose.linear() * ik_data.root_to_hip) - ik_data.ankle_to_ee;
        double yz_v_sq_dist = v.y() * v.y() + v.z() * v.z();

        double value = (ik_data.h1 * ik_data.h1 + ik_data.h2 * ik_data.h2 - yz_v_sq_dist) / (2 * ik_data.h1 * ik_data.h2);
        if (value < -1.0)
            value = -1.0;
        if (value > 1.0)
            value = 1.0;
        double rho = std::acos(value);

        joint_values[3] = -(M_PI + ik_data.ph1 - ik_data.ph2 - rho);

        // let Rx3^-1 * L3 + L4=k
        // solve v.x * cos7 + v.z * sin7 = k.x : get theta7 (11)
        // solve (v.x * sin7 - v.z * cos7) * sin6 + v.y * cos6 = k.y : get theta6 (12)
        const Eigen::Vector3d k = Eigen::AngleAxisd(-joint_values[3], Eigen::Vector3d::UnitX()) * ik_data.hip_to_knee + ik_data.knee_to_ankle;
        joint_values[5] = solveASinXPlusBCosXIsC(v.z(),v.x(), k.x());
        joint_values[4] = solveASinXPlusBCosXIsC(v.x() * std::sin(joint_values[5]) - v.z() * std::cos(joint_values[5]), v.y(), k.y());

        if (!isfinite(joint_values[3]) || !isfinite(joint_values[4]) || !isfinite(joint_values[5]))
        {
            ROS_INFO("v.y : %f v.z : %f yz_v_sq_dist : %f, rho : %f", v.y(), v.z(), yz_v_sq_dist, rho);
            ROS_INFO("Error:finite value!");
        }
    }

    // let ROOT.M^-1 * foot.M * Ry5^-1 * Rx4^-1 * Rx3^-1 = M,
    // -s3 = M(2,0) : get theta3 (15)
    // s2c3 = M(1,0) : get theta2 (16)
    // c3s4 = M(2,1) : get theta4 (17)
    {
        const Eigen::MatrixXd M = root_pose.linear().inverse() * dest_pose.linear()
                                  * Eigen::AngleAxisd(-joint_values[5], Eigen::Vector3d::UnitY())
                                  * Eigen::AngleAxisd(-joint_values[4], Eigen::Vector3d::UnitX())
                                  * Eigen::AngleAxisd(-joint_values[3], Eigen::Vector3d::UnitX());
        joint_values[1] = -std::asin(M(2, 0));
        joint_values[0] = std::asin(M(1, 0) / std::cos(joint_values[1]));
        joint_values[2] = std::asin(M(2, 1) / std::cos(joint_values[1]));

        if (!isfinite(joint_values[0]) || !isfinite(joint_values[1]) || !isfinite(joint_values[2]))
        {
            ROS_INFO("Error:finite value!");
        }
    }

    // convert Rx4 * Ry5 * Rz6 to Rz4 * Ry5 * Rx6
    {
        Eigen::Matrix3d ankle_rotation = Eigen::Matrix3d::Identity()
                                         * Eigen::AngleAxisd(joint_values[4], Eigen::Vector3d::UnitX())
                                         * Eigen::AngleAxisd(joint_values[5], Eigen::Vector3d::UnitY())
                                         * Eigen::AngleAxisd(joint_values[6], Eigen::Vector3d::UnitZ());
        Eigen::Vector3d euler_angles = ankle_rotation.eulerAngles(2, 1, 0);

        joint_values[4] = euler_angles(0);
        joint_values[5] = euler_angles(1);
        joint_values[6] = euler_angles(2);

        if (!isfinite(joint_values[6]) || !isfinite(joint_values[4]) || !isfinite(joint_values[5]))
        {
            ROS_INFO("Error:finite value!");
        }
    }

    // validate foot pos
    // foot.p = ROOT.p + ROOT.M * (L0 + Rz0 * Ry1 * Rx2 * (L3 + Rx3 * (L4 + Rx4 * Ry5 * Rz6(I) * L7)))
    // foot.M = ROOT.M * Rz0 * Ry1 * Rx2 * Rx3 * Rx4 * Ry5 * Rz6(I)
    Eigen::Vector3d evaluated_pos = root_pose.translation() + root_pose.linear()
                                    * (ik_data.root_to_hip + Eigen::AngleAxisd(joint_values[0], Eigen::Vector3d::UnitZ())
                                       * Eigen::AngleAxisd(joint_values[1], Eigen::Vector3d::UnitY())
                                       * Eigen::AngleAxisd(joint_values[2], Eigen::Vector3d::UnitX())
                                       * (ik_data.hip_to_knee + Eigen::AngleAxisd(joint_values[3], Eigen::Vector3d::UnitX())
                                          * (ik_data.knee_to_ankle + Eigen::AngleAxisd(joint_values[4], Eigen::Vector3d::UnitZ())
                                                  * Eigen::AngleAxisd(joint_values[5], Eigen::Vector3d::UnitY())
                                                  * Eigen::AngleAxisd(joint_values[6], Eigen::Vector3d::UnitX()) * ik_data.ankle_to_ee)));
    Eigen::MatrixXd evaluated_rotation = root_pose.linear()
                                         * Eigen::AngleAxisd(joint_values[0], Eigen::Vector3d::UnitZ())
                                         * Eigen::AngleAxisd(joint_values[1], Eigen::Vector3d::UnitY())
                                         * Eigen::AngleAxisd(joint_values[2], Eigen::Vector3d::UnitX())
                                         * Eigen::AngleAxisd(joint_values[3], Eigen::Vector3d::UnitX())
                                         * Eigen::AngleAxisd(joint_values[4], Eigen::Vector3d::UnitZ())
                                         * Eigen::AngleAxisd(joint_values[5], Eigen::Vector3d::UnitY())
                                         * Eigen::AngleAxisd(joint_values[6], Eigen::Vector3d::UnitX());

    return true;
}

bool ItompRobotModelIKHelper::getGroupEndeffectorPos(const std::string& group_name, const robot_state::RobotState& robot_state, Eigen::Affine3d& ee_pose) const
{
    if (group_name != "left_leg" && group_name != "right_leg")
        return false;

    const robot_model::JointModelGroup* jmg = moveit_robot_model_->getJointModelGroup(group_name);
    if (jmg == NULL)
        return false;

    std::string ee_group_name = jmg->getAttachedEndEffectorNames()[0];
    const robot_model::LinkModel* ee_link_model = moveit_robot_model_->getJointModelGroup(ee_group_name)->getLinkModels()[0];

    ee_pose = robot_state.getGlobalLinkTransform(ee_link_model);

    return true;
}

bool ItompRobotModelIKHelper::computeStandIKState(robot_state::RobotState& robot_state, Eigen::Affine3d& root_pose, const Eigen::Affine3d& left_foot_pose, const Eigen::Affine3d& right_foot_pose) const
{
    // adjust root_z for foot poses
    bool is_feasible = true;
    is_feasible &= adjustRootZ("left_leg", root_pose, left_foot_pose);
    is_feasible &= adjustRootZ("right_leg", root_pose, right_foot_pose);

    // set root transform from root_pose;
    Eigen::Vector3d euler_angles = root_pose.linear().eulerAngles(0, 1, 2);
    for (int i = 0; i < 3; ++i)
    {
        double default_value = (i == 2) ? 0.9619 : 0.0;
        robot_state.getVariablePositions()[i] = root_pose.translation()(i) - default_value;
        robot_state.getVariablePositions()[i + 3] = euler_angles(i);
    }

    if (is_feasible)
    {
        std::vector<double> joint_values;
        computeInverseKinematics("left_leg", root_pose, left_foot_pose, joint_values);
        robot_state.setJointGroupPositions("left_leg", joint_values);
        computeInverseKinematics("right_leg", root_pose, right_foot_pose, joint_values);
        robot_state.setJointGroupPositions("right_leg", joint_values);
    }

    /*
    Eigen::Vector3d evaluated_pos = root_pose.translation() + root_pose.linear()
                                    * (root_to_hip + Eigen::AngleAxisd(joint_values[0], Eigen::Vector3d::UnitZ())
                                       * Eigen::AngleAxisd(joint_values[1], Eigen::Vector3d::UnitY())
                                       * Eigen::AngleAxisd(joint_values[2], Eigen::Vector3d::UnitX())
                                       * (hip_to_knee + Eigen::AngleAxisd(joint_values[3], Eigen::Vector3d::UnitX())
                                          * (knee_to_ankle + Eigen::AngleAxisd(joint_values[4], Eigen::Vector3d::UnitX())
                                                  * Eigen::AngleAxisd(joint_values[5], Eigen::Vector3d::UnitY())
                                                  * Eigen::AngleAxisd(joint_values[6], Eigen::Vector3d::UnitZ()) * ankle_to_ee)));


    Eigen::MatrixXd evaluated_rotation[7];
    evaluated_rotation[0] = root_pose.linear()
                            * Eigen::AngleAxisd(joint_values[0], Eigen::Vector3d::UnitZ());
    evaluated_rotation[1] = evaluated_rotation[0]
                            * Eigen::AngleAxisd(joint_values[1], Eigen::Vector3d::UnitY());
    evaluated_rotation[2] = evaluated_rotation[1]
                            * Eigen::AngleAxisd(joint_values[2], Eigen::Vector3d::UnitX());
    evaluated_rotation[3] = evaluated_rotation[2]
                            * Eigen::AngleAxisd(joint_values[3], Eigen::Vector3d::UnitX());
    evaluated_rotation[4] = evaluated_rotation[3]
                            * Eigen::AngleAxisd(joint_values[4], Eigen::Vector3d::UnitZ());
    evaluated_rotation[5] = evaluated_rotation[4]
                            * Eigen::AngleAxisd(joint_values[5], Eigen::Vector3d::UnitY());
    evaluated_rotation[6] = evaluated_rotation[5]
                            * Eigen::AngleAxisd(joint_values[6], Eigen::Vector3d::UnitX());

    // test
    robot_state.update(true);

    Eigen::Affine3d root_transform = robot_state.getGlobalLinkTransform("pelvis_link");
    Eigen::Affine3d left_ee_transform = robot_state.getGlobalLinkTransform("left_foot_endeffector_link");
    Eigen::Affine3d right_ee_transform = robot_state.getGlobalLinkTransform("right_foot_endeffector_link");

    Eigen::Affine3d right_hip_transform = robot_state.getGlobalLinkTransform("upper_right_leg_x_link");
    Eigen::Affine3d right_knee_transform = robot_state.getGlobalLinkTransform("lower_right_leg_link");
    Eigen::Affine3d right_ankle_transform = robot_state.getGlobalLinkTransform("right_foot_x_link");
    */

    return true;
}

bool ItompRobotModelIKHelper::adjustRootZ(const std::string& group_name, Eigen::Affine3d& root_pose, const Eigen::Affine3d& dest_pose) const
{
    if (group_name != "left_leg" && group_name != "right_leg")
        return false;

    if (ik_data_map_.find(group_name) == ik_data_map_.end())
        initializeIKData(group_name);

    const ItompRobotModelIKData& ik_data = ik_data_map_[group_name];

    const std::vector<const robot_model::LinkModel*>& group_link_models = moveit_robot_model_->getJointModelGroup(group_name)->getLinkModels();
    std::string ee_group_name = moveit_robot_model_->getJointModelGroup(group_name)->getAttachedEndEffectorNames()[0];
    const robot_model::LinkModel* ee_link_model = moveit_robot_model_->getJointModelGroup(ee_group_name)->getLinkModels()[0];

    robot_state::RobotState robot_state(moveit_robot_model_);
    robot_state.setToDefaultValues();
    robot_state.update(true);

    Eigen::Affine3d hip_transf = robot_state.getGlobalLinkTransform(group_link_models[0]);
    Eigen::Affine3d knee_transf = robot_state.getGlobalLinkTransform(group_link_models[3]);
    Eigen::Affine3d ankle_transf = robot_state.getGlobalLinkTransform(group_link_models[4]);
    Eigen::Vector3d hip_to_knee = knee_transf.translation() - hip_transf.translation();
    Eigen::Vector3d knee_to_ankle = ankle_transf.translation() - knee_transf.translation();

    Eigen::Vector3d euler_angles = root_pose.linear().eulerAngles(0, 1, 2);
    for (int i = 0; i < 3; ++i)
    {
        double default_value = (i == 2) ? 0.9619 : 0.0;
        robot_state.getVariablePositions()[i] = root_pose.translation()(i) - default_value;
        robot_state.getVariablePositions()[i + 3] = euler_angles(i);
    }
    robot_state.update(true);

    Eigen::Affine3d ee_transf = robot_state.getGlobalLinkTransform(ee_link_model);
    hip_transf = robot_state.getGlobalLinkTransform(group_link_models[0]);
    knee_transf = robot_state.getGlobalLinkTransform(group_link_models[3]);
    ankle_transf = robot_state.getGlobalLinkTransform(group_link_models[4]);
    Eigen::Vector3d ankle_to_ee = ee_transf.translation() - ankle_transf.translation();

    Eigen::Vector3d dest_ankle_pos = dest_pose.translation() - ankle_to_ee;
    Eigen::Vector3d dest_hip_to_ankle = dest_ankle_pos - hip_transf.translation();

    if (dest_hip_to_ankle.norm() <= ik_data.max_stretch)
        return true; // no change
    else
    {
        double sq_max_stretch = ik_data.max_stretch * ik_data.max_stretch;
        double sq_dist_x_y = dest_hip_to_ankle.x() * dest_hip_to_ankle.x() + dest_hip_to_ankle.y() * dest_hip_to_ankle.y();
        if (sq_dist_x_y > sq_max_stretch)
        {
            ROS_INFO("Failed to set initial IK pose");
            return false;
        }
        double new_z = std::sqrt(sq_max_stretch - sq_dist_x_y);
        double dist_diff = new_z + dest_hip_to_ankle.z();

        //ROS_INFO("Z adjusted : %f -> %f", root_pose.translation()(2), root_pose.translation()(2) + dist_diff);

        root_pose.translation()(2) += dist_diff;

        /*
        // for debug
        double old_stretch = dest_hip_to_ankle.norm();
        Eigen::Vector3d euler_angles = root_pose.linear().eulerAngles(0, 1, 2);
        for (int i = 0; i < 3; ++i)
        {
            double default_value = (i == 2) ? 0.9619 : 0.0;
            robot_state.getVariablePositions()[i] = root_pose.translation()(i) - default_value;
            robot_state.getVariablePositions()[i + 3] = euler_angles(i);
        }
        robot_state.update(true);
        hip_transf = robot_state.getGlobalLinkTransform(group_link_models[0]);
        dest_hip_to_ankle = dest_ankle_pos - hip_transf.translation();
        ROS_INFO("New dist(max:%f) : %f -> %f(%f, %f)", ik_data.max_stretch, old_stretch, dest_hip_to_ankle.norm(),
                 std::sqrt(dest_hip_to_ankle.x() * dest_hip_to_ankle.x() + dest_hip_to_ankle.y() * dest_hip_to_ankle.y()),
                 dest_hip_to_ankle.z());
                 */
    }
    return true;
}

bool ItompRobotModelIKHelper::getRootPose(const std::string& group_name, const Eigen::Affine3d& ee_pose, Eigen::Affine3d& root_pose) const
{
    if (group_name != "left_leg" && group_name != "right_leg")
        return false;

    if (ik_data_map_.find(group_name) == ik_data_map_.end())
        initializeIKData(group_name);

    const ItompRobotModelIKData& ik_data = ik_data_map_[group_name];

    root_pose.linear() = ee_pose.linear();
    root_pose.translation() = ee_pose.translation() + ee_pose.linear() * ik_data.ee_to_root.translation();

    return true;
}

}
