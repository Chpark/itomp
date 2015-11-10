#include <itomp_cio_planner/trajectory/itomp_trajectory.h>
#include <itomp_cio_planner/util/joint_state_util.h>
#include <itomp_cio_planner/optimization/phase_manager.h>
#include <itomp_cio_planner/util/planning_parameters.h>
#include <ros/assert.h>
#include <ecl/geometry/polynomial.hpp>
#include <ecl/geometry.hpp>

using namespace std;

namespace itomp_cio_planner
{

ItompTrajectory::ItompTrajectory(const std::string& name, unsigned int num_points, const std::vector<NewTrajectoryPtr>& components,
                                 unsigned int num_keyframes, unsigned int keyframe_interval, double duration, double discretization)
    : CompositeTrajectory(name, num_points, components), num_keyframes_(num_keyframes), keyframe_interval_(keyframe_interval),
      duration_(duration), discretization_(discretization)
{
    for (int i = 0; i < COMPONENT_TYPE_NUM; ++i)
    {
        CompositeTrajectoryPtr component = boost::static_pointer_cast<CompositeTrajectory>(getComponent(i));
        for (unsigned int s = 0; s < SUB_COMPONENT_TYPE_NUM; ++s)
        {
            element_trajectories_[i][s] = boost::static_pointer_cast<ElementTrajectory>(component->getComponent(s));
        }
    }

    for (int i = 0; i < COMPONENT_TYPE_NUM; ++i)
    {
        backup_trajectory_[i] = Eigen::MatrixXd(num_points_, 1);
    }
}

ItompTrajectory::ItompTrajectory(const ItompTrajectory& trajectory)
    : CompositeTrajectory(trajectory),
      num_keyframes_(trajectory.num_keyframes_),
      keyframe_interval_(trajectory.keyframe_interval_),
      duration_(trajectory.duration_),
      discretization_(trajectory.discretization_),
      parameter_to_index_map_(trajectory.parameter_to_index_map_),
      full_to_parameter_joint_index_map_(trajectory.full_to_parameter_joint_index_map_)
{
    for (int i = 0; i < COMPONENT_TYPE_NUM; ++i)
    {
        CompositeTrajectoryPtr component = boost::static_pointer_cast<CompositeTrajectory>(getComponent(i));
        for (unsigned int s = 0; s < SUB_COMPONENT_TYPE_NUM; ++s)
        {
            element_trajectories_[i][s] = boost::static_pointer_cast<ElementTrajectory>(component->getComponent(s));
        }
    }

    for (int i = 0; i < COMPONENT_TYPE_NUM; ++i)
    {
        backup_trajectory_[i] = trajectory.backup_trajectory_[i];
    }
}

ItompTrajectory::~ItompTrajectory()
{

}

ItompTrajectory* ItompTrajectory::clone() const
{
    return new ItompTrajectory(*this);
}

void ItompTrajectory::setStartState(const sensor_msgs::JointState &joint_state,
                                    const ItompRobotModelConstPtr& robot_model)
{
    if (PlanningParameters::getInstance()->getPrintPlanningInfo())
        ROS_INFO("Set the trajectory start state");

    Eigen::MatrixXd::RowXpr traj_start_point[] =
    {
        getElementTrajectory(COMPONENT_TYPE_POSITION, SUB_COMPONENT_TYPE_JOINT)->getTrajectoryPoint(0),
        getElementTrajectory(COMPONENT_TYPE_VELOCITY, SUB_COMPONENT_TYPE_JOINT)->getTrajectoryPoint(0),
        getElementTrajectory(COMPONENT_TYPE_ACCELERATION, SUB_COMPONENT_TYPE_JOINT)->getTrajectoryPoint(0)
    };

    jointStateToArray(robot_model, joint_state,
                      traj_start_point[COMPONENT_TYPE_POSITION],
                      traj_start_point[COMPONENT_TYPE_VELOCITY],
                      traj_start_point[COMPONENT_TYPE_ACCELERATION]);

    for (unsigned int i = 0; i < joint_state.name.size(); i++)
    {
        std::string name = joint_state.name[i];
        int rbdl_number = robot_model->jointNameToRbdlNumber(name);
        if (rbdl_number >= 0)
        {
            traj_start_point[COMPONENT_TYPE_POSITION](rbdl_number) = joint_state.position[i];
            traj_start_point[COMPONENT_TYPE_VELOCITY](rbdl_number) = joint_state.velocity.size() ? joint_state.velocity[i] : 0.0;
            traj_start_point[COMPONENT_TYPE_ACCELERATION](rbdl_number) = joint_state.effort.size() ? joint_state.effort[i] : 0.0;
            if (PlanningParameters::getInstance()->getPrintPlanningInfo())
                ROS_INFO("[%d] %s : %f %f %f", rbdl_number, name.c_str(),
                         traj_start_point[COMPONENT_TYPE_POSITION](rbdl_number),
                         traj_start_point[COMPONENT_TYPE_VELOCITY](rbdl_number),
                         traj_start_point[COMPONENT_TYPE_ACCELERATION](rbdl_number));
        }
    }

    // fill_trajectory
    for (int i = 1; i < getNumPoints(); ++i)
    {
        getElementTrajectory(COMPONENT_TYPE_POSITION, SUB_COMPONENT_TYPE_JOINT)->getTrajectoryPoint(i)
            = traj_start_point[COMPONENT_TYPE_POSITION];
    }

}

void ItompTrajectory::setGoalState(const sensor_msgs::JointState& joint_goal_state,
                                   const ItompPlanningGroupConstPtr& planning_group,
                                   const ItompRobotModelConstPtr& robot_model,
                                   const moveit_msgs::TrajectoryConstraints& trajectory_constraints)
{
    if (PlanningParameters::getInstance()->getPrintPlanningInfo())
        ROS_INFO("Set the trajectory goal state for planning group : %s", planning_group->name_.c_str());

    ROS_ASSERT(getNumPoints() > 0);

    // set trajectory goal point
    unsigned int goal_index = getNumPoints() - 1;
    Eigen::MatrixXd::RowXpr traj_start_point = getElementTrajectory(COMPONENT_TYPE_POSITION, SUB_COMPONENT_TYPE_JOINT)->getTrajectoryPoint(0);
    Eigen::MatrixXd::RowXpr traj_goal_point = getElementTrajectory(COMPONENT_TYPE_POSITION, SUB_COMPONENT_TYPE_JOINT)->getTrajectoryPoint(goal_index);

    std::vector<unsigned int> group_rbdl_indices;
    for (unsigned int i = 0; i < planning_group->num_joints_; ++i)
    {
        unsigned int rbdl_index = planning_group->group_joints_[i].rbdl_joint_index_;
        group_rbdl_indices.push_back(rbdl_index);

        double pos = joint_goal_state.position[rbdl_index];

        // wrap around
        if (planning_group->group_joints_[i].wrap_around_)
        {
            double start_pos = traj_start_point(rbdl_index);
            while (pos - start_pos > M_PI)
                pos -= 2 * M_PI;
            while (pos - start_pos < -M_PI)
                pos += 2 * M_PI;
        }

        traj_goal_point(rbdl_index) = pos;
    }

    // interpolate trajectory

    if (trajectory_constraints.constraints.size() != 0)
    {
        interpolateInputJointTrajectory(group_rbdl_indices, planning_group, trajectory_constraints);
    }
    else

    {
        //interpolateStartEnd(SUB_COMPONENT_TYPE_JOINT, &group_rbdl_indices);
    }
}

void ItompTrajectory::interpolateInputJointTrajectory(const std::vector<unsigned int>& group_rbdl_indices,
        const ItompPlanningGroupConstPtr& planning_group,
        const moveit_msgs::TrajectoryConstraints& trajectory_constraints)
{
    Eigen::MatrixXd::RowXpr traj_start_point[] =
    {
        getElementTrajectory(COMPONENT_TYPE_POSITION, SUB_COMPONENT_TYPE_JOINT)->getTrajectoryPoint(0),
        getElementTrajectory(COMPONENT_TYPE_VELOCITY, SUB_COMPONENT_TYPE_JOINT)->getTrajectoryPoint(0),
        getElementTrajectory(COMPONENT_TYPE_ACCELERATION, SUB_COMPONENT_TYPE_JOINT)->getTrajectoryPoint(0)
    };
    unsigned int goal_index = getNumPoints() - 1;
    Eigen::MatrixXd::RowXpr traj_goal_point = getElementTrajectory(COMPONENT_TYPE_POSITION, SUB_COMPONENT_TYPE_JOINT)->getTrajectoryPoint(goal_index);

    int num_points = getNumPoints();
    int num_input_waypoints = trajectory_constraints.constraints.size();
    double waypoint_interval = (double) (num_points - 1) / (num_input_waypoints - 1);

    for (unsigned int j = 0; j < group_rbdl_indices.size(); ++j)
    {
        unsigned int rbdl_index = group_rbdl_indices[j];

        bool has_constraints = false;
        int constraint_index = -1;
        for (int k = 0; k < trajectory_constraints.constraints[0].joint_constraints.size(); ++k)
        {
            if (trajectory_constraints.constraints[0].joint_constraints[k].joint_name
                    == planning_group->group_joints_[j].joint_name_)
            {

                has_constraints = true;
                constraint_index = k;
                break;
            }
        }

        if (!has_constraints)
        {
            double x0 = traj_start_point[COMPONENT_TYPE_POSITION](rbdl_index);
            double v0 = traj_start_point[COMPONENT_TYPE_VELOCITY](rbdl_index);
            double a0 = traj_start_point[COMPONENT_TYPE_ACCELERATION](rbdl_index);

            double x1 = traj_goal_point(rbdl_index);
            double v1 = 0.0;
            double a1 = 0.0;

            ecl::QuinticPolynomial poly;
            poly = ecl::QuinticPolynomial::Interpolation(0, x0, v0, a0, duration_,
                    x1, v1, a1);
            for (unsigned int i = 1; i < getNumPoints() - 1; ++i)
            {
                Eigen::MatrixXd::RowXpr traj_point[] =
                {
                    getElementTrajectory(COMPONENT_TYPE_POSITION, SUB_COMPONENT_TYPE_JOINT)->getTrajectoryPoint(i),
                    getElementTrajectory(COMPONENT_TYPE_VELOCITY, SUB_COMPONENT_TYPE_JOINT)->getTrajectoryPoint(i),
                    getElementTrajectory(COMPONENT_TYPE_ACCELERATION, SUB_COMPONENT_TYPE_JOINT)->getTrajectoryPoint(i)
                };
                traj_point[COMPONENT_TYPE_POSITION](rbdl_index) = poly(i * discretization_);
                traj_point[COMPONENT_TYPE_VELOCITY](rbdl_index) = poly.derivative(i * discretization_);
                traj_point[COMPONENT_TYPE_ACCELERATION](rbdl_index) = poly.dderivative(i * discretization_);
            }
        }
        else
        {
            // interpolate between waypoints
            for (int k = 0; k < num_input_waypoints - 1; ++k)
            {
                double x0, v0, a0;
                double x1, v1, a1;

                x0 = trajectory_constraints.constraints[k].joint_constraints[constraint_index].position;
                v0 = 0.0;
                a0 = 0.0;
                x1 = trajectory_constraints.constraints[k + 1].joint_constraints[constraint_index].position;
                v1 = 0.0;
                a1 = 0.0;

                int from = safeDoubleToInt(k * waypoint_interval);
                int to = std::min(safeDoubleToInt((k + 1) * waypoint_interval), num_points - 1);

                ecl::QuinticPolynomial poly;
                poly = ecl::QuinticPolynomial::Interpolation(from * discretization_, x0, v0, a0,
                        to * discretization_, x1, v1, a1);
                for (int i = from; i <= to; ++i)
                {
                    Eigen::MatrixXd::RowXpr traj_point[] =
                    {
                        getElementTrajectory(COMPONENT_TYPE_POSITION, SUB_COMPONENT_TYPE_JOINT)->getTrajectoryPoint(i),
                        getElementTrajectory(COMPONENT_TYPE_VELOCITY, SUB_COMPONENT_TYPE_JOINT)->getTrajectoryPoint(i),
                        getElementTrajectory(COMPONENT_TYPE_ACCELERATION, SUB_COMPONENT_TYPE_JOINT)->getTrajectoryPoint(i)
                    };
                    traj_point[COMPONENT_TYPE_POSITION](rbdl_index) = poly(i * discretization_);
                    traj_point[COMPONENT_TYPE_VELOCITY](rbdl_index) = 0.0;//poly.derivative(i * discretization_);
                    traj_point[COMPONENT_TYPE_ACCELERATION](rbdl_index) = 0.0;//poly.dderivative(i * discretization_);
                }
            }

            traj_start_point[COMPONENT_TYPE_POSITION](rbdl_index) = trajectory_constraints.constraints[0].joint_constraints[constraint_index].position;
            traj_goal_point(rbdl_index) = trajectory_constraints.constraints[num_input_waypoints - 1].joint_constraints[constraint_index].position;
        }
    }
}

void ItompTrajectory::interpolateKeyframes(const ItompPlanningGroupConstPtr& planning_group)
{
    // cubic interpolation of pos, vel, acc
    // update trajectory between (k, k+1]
    // acc is discontinuous at each keyframe
    for (unsigned int s = 0; s < SUB_COMPONENT_TYPE_NUM; ++s)
    {
        unsigned int num_sub_component_elements = getElementTrajectory(0, s)->getNumElements();
        for (unsigned int j = 0; j < num_sub_component_elements; ++j)
        {
            for (unsigned int k = 0; k < num_keyframes_ - 1; ++k)
            {
                ecl::CubicPolynomial poly;
                unsigned int cur_keyframe_index = k * keyframe_interval_;
                unsigned int next_keyframe_index = cur_keyframe_index + keyframe_interval_;

                double cur_pos = getElementTrajectory(COMPONENT_TYPE_POSITION, s)->at(cur_keyframe_index, j);
                double cur_vel = getElementTrajectory(COMPONENT_TYPE_VELOCITY, s)->at(cur_keyframe_index, j);
                double next_pos = getElementTrajectory(COMPONENT_TYPE_POSITION, s)->at(next_keyframe_index, j);
                double next_vel = getElementTrajectory(COMPONENT_TYPE_VELOCITY, s)->at(next_keyframe_index, j);

                // handle joint limits / wrapping
                bool next_pos_changed = false;
                if (s == SUB_COMPONENT_TYPE_JOINT && full_to_parameter_joint_index_map_[j] != -1)
                {
                    double old_next_pos = next_pos;

                    const ItompRobotJoint& joint = planning_group->group_joints_[full_to_parameter_joint_index_map_[j]];
                    if (joint.has_joint_limits_)
                    {
                        next_pos = std::max(next_pos, joint.joint_limit_min_);
                        next_pos = std::min(next_pos, joint.joint_limit_max_);
                    }
                    if (joint.wrap_around_)
                    {
                        while (next_pos - cur_pos > M_PI)
                            next_pos -= 2 * M_PI;
                        while (next_pos - cur_pos < -M_PI)
                            next_pos += 2 * M_PI;
                    }

                    if (next_pos != old_next_pos)
                        next_pos_changed = true;
                }

                poly = ecl::CubicPolynomial::DerivativeInterpolation(
                           (double)cur_keyframe_index * discretization_, cur_pos, cur_vel,
                           (double)next_keyframe_index * discretization_, next_pos, next_vel);

                unsigned int interpolation_end = next_pos_changed ? next_keyframe_index : next_keyframe_index - 1;
                for (unsigned int i = cur_keyframe_index + 1; i <= interpolation_end; ++i)
                {
                    double t = i * discretization_;
                    getElementTrajectory(COMPONENT_TYPE_POSITION, s)->at(i, j) = poly(t);
                    getElementTrajectory(COMPONENT_TYPE_VELOCITY, s)->at(i, j) = poly.derivative(t);
                    if (i != next_keyframe_index)
                        getElementTrajectory(COMPONENT_TYPE_ACCELERATION, s)->at(i, j) = poly.dderivative(t);
                }
                if (false)//next_keyframe_index != num_points_ - 1)
                    getElementTrajectory(COMPONENT_TYPE_ACCELERATION, s)->at(next_keyframe_index, j) = poly.dderivative((double)next_keyframe_index * discretization_);
            }
        }
    }
}

void ItompTrajectory::interpolateKeyframes()
{
    if (keyframe_interval_ <= 1)
        return;

    /*
    std::set<unsigned int> DOF3;
    DOF3.insert(3);
    DOF3.insert(6);
    DOF3.insert(9);
    DOF3.insert(12);
    DOF3.insert(15);
    DOF3.insert(18);
    DOF3.insert(21);
    DOF3.insert(29);
    DOF3.insert(32);
    DOF3.insert(35);
    DOF3.insert(38);
    DOF3.insert(46);
    DOF3.insert(49);
    DOF3.insert(52);
    DOF3.insert(60);
    DOF3.insert(63);
    DOF3.insert(66);
    */

    // cubic interpolation of pos, vel, acc
    // update trajectory between (k, k+1]
    // acc is discontinuous at each keyframe
    for (unsigned int s = 0; s < SUB_COMPONENT_TYPE_NUM; ++s)
    {
        unsigned int num_sub_component_elements = getElementTrajectory(0, s)->getNumElements();
        for (unsigned int j = 0; j < num_sub_component_elements; ++j)
        {
            //if (s != SUB_COMPONENT_TYPE_JOINT || j < 3)
            {
                for (unsigned int k = 0; k < num_keyframes_ - 1; ++k)
                {
                    ecl::CubicPolynomial poly;
                    unsigned int cur_keyframe_index = k * keyframe_interval_;
                    unsigned int next_keyframe_index = cur_keyframe_index + keyframe_interval_;

                    double cur_pos = getElementTrajectory(COMPONENT_TYPE_POSITION, s)->at(cur_keyframe_index, j);
                    double cur_vel = getElementTrajectory(COMPONENT_TYPE_VELOCITY, s)->at(cur_keyframe_index, j);
                    double next_pos = getElementTrajectory(COMPONENT_TYPE_POSITION, s)->at(next_keyframe_index, j);
                    double next_vel = getElementTrajectory(COMPONENT_TYPE_VELOCITY, s)->at(next_keyframe_index, j);

                    poly = ecl::CubicPolynomial::DerivativeInterpolation(
                               (double)cur_keyframe_index * discretization_, cur_pos, cur_vel,
                               (double)next_keyframe_index * discretization_, next_pos, next_vel);

                    for (unsigned int i = cur_keyframe_index + 1; i < next_keyframe_index; ++i)
                    {
                        double t = i * discretization_;
                        getElementTrajectory(COMPONENT_TYPE_POSITION, s)->at(i, j) = poly(t);
                        getElementTrajectory(COMPONENT_TYPE_VELOCITY, s)->at(i, j) = poly.derivative(t);
                        getElementTrajectory(COMPONENT_TYPE_ACCELERATION, s)->at(i, j) = poly.dderivative(t);
                    }
                    if (false)//next_keyframe_index != num_points_ - 1)
                        getElementTrajectory(COMPONENT_TYPE_ACCELERATION, s)->at(next_keyframe_index, j) = poly.dderivative((double)next_keyframe_index * discretization_);
                    //std::cout << "Acc " << cur_keyframe_index << " : " << poly.dderivative( (double)cur_keyframe_index * discretization_ ) << std::endl;
                    //std::cout << "Acc " << next_keyframe_index << " : " << poly.dderivative( (double)next_keyframe_index * discretization_ ) << std::endl;
                }
            }
            /*
            else if (DOF3.find(j) != DOF3.end())
            {
                for (unsigned int k = 0; k < num_keyframes_ - 1; ++k)
                {
                    ecl::CubicPolynomial poly;
                    unsigned int cur_keyframe_index = k * keyframe_interval_;
                    unsigned int next_keyframe_index = cur_keyframe_index + keyframe_interval_;

                    Eigen::Vector3d cur_euler;
                    for (unsigned int i = 0; i < 3; ++i)
                        cur_euler(i) = getElementTrajectory(COMPONENT_TYPE_POSITION, s)->at(cur_keyframe_index, j + i);
                    Eigen::Quaterniond cur_rot = Eigen::AngleAxisd(cur_euler(0), Eigen::Vector3d::UnitZ())
                            * Eigen::AngleAxisd(cur_euler(1), Eigen::Vector3d::UnitY())
                            * Eigen::AngleAxisd(cur_euler(2), Eigen::Vector3d::UnitX());

                    double cur_vel = getElementTrajectory(COMPONENT_TYPE_VELOCITY, s)->at(cur_keyframe_index, j);

                    Eigen::Vector3d next_euler;
                    for (unsigned int i = 0; i < 3; ++i)
                        next_euler(i) = getElementTrajectory(COMPONENT_TYPE_POSITION, s)->at(next_keyframe_index, j + i);
                    Eigen::Quaterniond next_rot = Eigen::AngleAxisd(next_euler(0), Eigen::Vector3d::UnitZ())
                            * Eigen::AngleAxisd(next_euler(1), Eigen::Vector3d::UnitY())
                            * Eigen::AngleAxisd(next_euler(2), Eigen::Vector3d::UnitX());

                    double next_vel = getElementTrajectory(COMPONENT_TYPE_VELOCITY, s)->at(next_keyframe_index, j);

                    poly = ecl::CubicPolynomial::DerivativeInterpolation(
                               (double)cur_keyframe_index * discretization_, 0, cur_vel,
                               (double)next_keyframe_index * discretization_, 1, next_vel);

                    for (unsigned int i = cur_keyframe_index + 1; i < next_keyframe_index; ++i)
                    {
                        double t = i * discretization_;
                        Eigen::Vector3d interpolated_euler = cur_rot.slerp(poly(t), next_rot).
                                toRotationMatrix().eulerAngles(2, 1, 0);
                        getElementTrajectory(COMPONENT_TYPE_POSITION, s)->at(i, j) = interpolated_euler(0);
                        getElementTrajectory(COMPONENT_TYPE_POSITION, s)->at(i, j + 1) = interpolated_euler(1);
                        getElementTrajectory(COMPONENT_TYPE_POSITION, s)->at(i, j + 2) = interpolated_euler(2);
                        getElementTrajectory(COMPONENT_TYPE_VELOCITY, s)->at(i, j) = poly.derivative(t);
                        getElementTrajectory(COMPONENT_TYPE_ACCELERATION, s)->at(i, j) = poly.dderivative(t);
                    }
                    //if (false)//next_keyframe_index != num_points_ - 1)
                      //  getElementTrajectory(COMPONENT_TYPE_ACCELERATION, s)->at(next_keyframe_index, j) = poly.dderivative((double)next_keyframe_index * discretization_);
                    //std::cout << "Acc " << cur_keyframe_index << " : " << poly.dderivative( (double)cur_keyframe_index * discretization_ ) << std::endl;
                    //std::cout << "Acc " << next_keyframe_index << " : " << poly.dderivative( (double)next_keyframe_index * discretization_ ) << std::endl;
                }
            }
            */
        }
    }
}

void ItompTrajectory::interpolateTrajectory(unsigned int trajectory_point_begin, unsigned int trajectory_point_end,
        const ItompTrajectoryIndex& index)
{
    if (keyframe_interval_ <= 1)
        return;

    unsigned int sub_component_index = index.sub_component;
    unsigned int element = index.element;

    // skip the initial position
    ecl::CubicPolynomial poly;

    for (unsigned int cur_keyframe_index = trajectory_point_begin,
            next_keyframe_index = cur_keyframe_index + keyframe_interval_;
            next_keyframe_index <= trajectory_point_end;
            cur_keyframe_index += keyframe_interval_, next_keyframe_index += keyframe_interval_)
    {
        poly = ecl::CubicPolynomial::DerivativeInterpolation(
                   (double)cur_keyframe_index * discretization_,
                   getElementTrajectory(COMPONENT_TYPE_POSITION, sub_component_index)->at(cur_keyframe_index, element),
                   getElementTrajectory(COMPONENT_TYPE_VELOCITY, sub_component_index)->at(cur_keyframe_index, element),
                   (double)next_keyframe_index * discretization_,
                   getElementTrajectory(COMPONENT_TYPE_POSITION, sub_component_index)->at(next_keyframe_index, element),
                   getElementTrajectory(COMPONENT_TYPE_VELOCITY, sub_component_index)->at(next_keyframe_index, element));

        for (unsigned int i = cur_keyframe_index + 1; i < next_keyframe_index; ++i)
        {
            double t = i * discretization_;
            getElementTrajectory(COMPONENT_TYPE_POSITION, sub_component_index)->at(i, element) = poly(t);
            getElementTrajectory(COMPONENT_TYPE_VELOCITY, sub_component_index)->at(i, element) = poly.derivative(t);
            getElementTrajectory(COMPONENT_TYPE_ACCELERATION, sub_component_index)->at(i, element) = poly.dderivative(t);
        }
        if (false)//next_keyframe_index != num_points_ - 1)
            getElementTrajectory(COMPONENT_TYPE_ACCELERATION, sub_component_index)->at(next_keyframe_index, element) =
                poly.dderivative((double)next_keyframe_index * discretization_);
    }
}

void ItompTrajectory::setParameters(const ParameterVector& parameters, const ItompPlanningGroupConstPtr& planning_group)
{
    unsigned int num_parameters = getNumParameters();

    ROS_ASSERT(num_parameters > 0);
    ROS_ASSERT(num_parameters == parameters.size());

    for (unsigned int i = 0; i < num_parameters; ++i)
    {
        ItompTrajectoryIndex index = parameter_to_index_map_[i];

        if (PhaseManager::getInstance()->updateParameter(index) == false)
            continue;

        ElementTrajectoryPtr& et = getElementTrajectory(index.component, index.sub_component);
        Eigen::MatrixXd::RowXpr row = et->getTrajectoryPoint(index.point);
        row(index.element) = parameters(i, 0);
        if (index.component == COMPONENT_TYPE_VELOCITY)
            row(index.element) *= 10.0;
    }
    interpolateKeyframes();
}

void ItompTrajectory::getParameters(ParameterVector& parameters) const
{
    unsigned int num_parameters = parameter_to_index_map_.size();

    ROS_ASSERT(num_parameters > 0);
    ROS_ASSERT(num_parameters == parameters.size());

    for (unsigned int i = 0; i < num_parameters; ++i)
    {
        ItompTrajectoryIndex index = parameter_to_index_map_[i];

        ElementTrajectoryConstPtr et = getElementTrajectory(index.component, index.sub_component);
        Eigen::MatrixXd::ConstRowXpr row = et->getTrajectoryPoint(index.point);
        parameters(i, 0) = row(index.element);
        if (index.component == COMPONENT_TYPE_VELOCITY)
            parameters(i, 0) *= 0.1;
    }
}

void ItompTrajectory::directChangeForDerivativeComputation(unsigned int parameter_index, double value,
        unsigned int& trajectory_point_begin, unsigned int& trajectory_point_end,
        bool backup)
{
    ROS_ASSERT(parameter_to_index_map_.size() > 0);

    ItompTrajectoryIndex index = parameter_to_index_map_[parameter_index];

    int point = index.point;
    int element = index.element;

    trajectory_point_begin = std::max(0, point - (int)keyframe_interval_);
    trajectory_point_end = std::min(num_points_ - 1, point + keyframe_interval_);

    if (backup)
        backupTrajectory(index);

    // Do not update joint values of start/goal points
    if (PhaseManager::getInstance()->updateParameter(index) == false)
        return;

    // set value
    if (index.component == COMPONENT_TYPE_VELOCITY)
        value *= 10.0;
    getElementTrajectory(index.component, index.sub_component)->at(point, element) = value;

    interpolateTrajectory(trajectory_point_begin, trajectory_point_end, index);
}

void ItompTrajectory::backupTrajectory(const ItompTrajectoryIndex& index)
{
    int point = index.point;
    int element = index.element;
    int backup_point_begin = std::max(0, point - (int)keyframe_interval_);
    int backup_point_end = std::min(num_points_ - 1, point + keyframe_interval_);
    //if (point == num_points_ - 1)
    ++backup_point_end;
    int backup_length = backup_point_end - backup_point_begin;

    for (unsigned int i = 0; i < COMPONENT_TYPE_NUM; ++i)
    {
        backup_trajectory_[i].block(0, 0, backup_length, 1) =
            getElementTrajectory(i, index.sub_component)->getData().block(
                backup_point_begin, element, backup_length, 1);
    }
    backup_index_ = index;
}

void ItompTrajectory::restoreTrajectory()
{
    int point = backup_index_.point;
    int element = backup_index_.element;
    int backup_point_begin = std::max(0, point - (int)keyframe_interval_);
    int backup_point_end = std::min(num_points_ - 1, point + keyframe_interval_);
    //if (point == num_points_ - 1)
    ++backup_point_end;
    int backup_length = backup_point_end - backup_point_begin;

    for (unsigned int i = 0; i < COMPONENT_TYPE_NUM; ++i)
    {
        getElementTrajectory(i, backup_index_.sub_component)->getData().block(
            backup_point_begin, element, backup_length, 1) =
                backup_trajectory_[i].block(0, 0, backup_length, 1);
    }
}

void ItompTrajectory::computeParameterToTrajectoryIndexMap(const ItompRobotModelConstPtr& robot_model,
        const ItompPlanningGroupConstPtr& planning_group)
{
    int num_parameter_joints = planning_group->num_joints_;
    unsigned int num_full_joints = robot_model->getNumJoints();

    std::vector<unsigned int> parameter_to_full_joint_indices(num_parameter_joints);
    full_to_parameter_joint_index_map_.resize(num_full_joints, -1);

    for (unsigned int i = 0; i < num_parameter_joints; ++i)
    {
        parameter_to_full_joint_indices[i] = planning_group->group_joints_[i].rbdl_joint_index_;
        full_to_parameter_joint_index_map_[planning_group->group_joints_[i].rbdl_joint_index_] = i;
    }

    unsigned int num_contact_position_params = planning_group->getNumContacts() * 7; // var + pos(3) + ori(3)
    unsigned int num_contact_force_params = planning_group->getNumContacts() * NUM_ENDEFFECTOR_CONTACT_POINTS * 4; // n * force(3)

    unsigned int parameter_size = num_keyframes_ * 2 * (num_parameter_joints + num_contact_position_params + num_contact_force_params);
    parameter_to_index_map_.resize(parameter_size);

    unsigned int parameter_pos = 0;
    // pos, vel
    for (unsigned int j = 0; j < 2; ++j)
    {
        for (unsigned int i = 0; i < num_keyframes_; ++i)
        {
            unsigned int keyframe_pos = i * keyframe_interval_;

            // indices for joints
            for (unsigned int k = 0; k < num_parameter_joints; ++k)
            {
                ItompTrajectoryIndex& index = parameter_to_index_map_[parameter_pos++];

                index.component = j;
                index.sub_component = 0;
                index.point = keyframe_pos;
                index.element = parameter_to_full_joint_indices[k];
            }

            // indices for contact pos
            for (unsigned int k = 0; k < num_contact_position_params; ++k)
            {
                ItompTrajectoryIndex& index = parameter_to_index_map_[parameter_pos++];

                index.component = j;
                index.sub_component = 1;
                index.point = keyframe_pos;
                index.element = k;
            }

            // indices for contact forces
            for (unsigned int k = 0; k < num_contact_force_params; ++k)
            {
                ItompTrajectoryIndex& index = parameter_to_index_map_[parameter_pos++];

                index.component = j;
                index.sub_component = 2;
                index.point = keyframe_pos;
                index.element = k;
            }
        }
    }
}

void ItompTrajectory::setContactVariables(int point, const std::vector<ContactVariables>& contact_variables)
{
    Eigen::MatrixXd::RowXpr point_contact_positions =
        getElementTrajectory(COMPONENT_TYPE_POSITION, SUB_COMPONENT_TYPE_CONTACT_POSITION)->getTrajectoryPoint(point);
    Eigen::MatrixXd::RowXpr point_contact_forces =
        getElementTrajectory(COMPONENT_TYPE_POSITION, SUB_COMPONENT_TYPE_CONTACT_FORCE)->getTrajectoryPoint(point);

    int num_contacts = contact_variables.size();
    for (int i = 0; i < num_contacts; ++i)
    {
        point_contact_positions.block(0, i * 7, 1, 7) = contact_variables[i].serialized_position_.transpose();
        point_contact_forces.block(0, i * 4 * NUM_ENDEFFECTOR_CONTACT_POINTS, 1, 4 * NUM_ENDEFFECTOR_CONTACT_POINTS) =
            contact_variables[i].serialized_forces_.transpose();
    }
}

void ItompTrajectory::getContactVariables(int point, std::vector<ContactVariables>& contact_variables)
{
    // footstep
    int contact_point_ref_point = point;// - (point % 20);
    Eigen::MatrixXd::RowXpr point_contact_positions =
        getElementTrajectory(COMPONENT_TYPE_POSITION, SUB_COMPONENT_TYPE_CONTACT_POSITION)->getTrajectoryPoint(contact_point_ref_point);
    Eigen::MatrixXd::RowXpr point_contact_forces =
        getElementTrajectory(COMPONENT_TYPE_POSITION, SUB_COMPONENT_TYPE_CONTACT_FORCE)->getTrajectoryPoint(point);

    int num_contacts = contact_variables.size();
    for (int i = 0; i < num_contacts; ++i)
    {
        contact_variables[i].serialized_position_.transpose() = point_contact_positions.block(0, i * 7, 1, 7);
        contact_variables[i].serialized_forces_.transpose() =
            point_contact_forces.block(0, i * 4 * NUM_ENDEFFECTOR_CONTACT_POINTS, 1, 4 * NUM_ENDEFFECTOR_CONTACT_POINTS);
    }
}

void ItompTrajectory::interpolate(int point_start, int point_end, SUB_COMPONENT_TYPE sub_component_type,
                                  const std::vector<unsigned int>* element_indices)
{
    if (sub_component_type == SUB_COMPONENT_TYPE_ALL)
    {
        interpolate(point_start, point_end, SUB_COMPONENT_TYPE_JOINT, element_indices);
        interpolate(point_start, point_end, SUB_COMPONENT_TYPE_CONTACT_POSITION, element_indices);
        interpolate(point_start, point_end, SUB_COMPONENT_TYPE_CONTACT_FORCE, element_indices);
        return;
    }

    /*
    std::set<unsigned int> DOF3;
    DOF3.insert(3);
    DOF3.insert(6);
    DOF3.insert(9);
    DOF3.insert(12);
    DOF3.insert(15);
    DOF3.insert(18);
    DOF3.insert(21);
    DOF3.insert(29);
    DOF3.insert(32);
    DOF3.insert(35);
    DOF3.insert(38);
    DOF3.insert(46);
    DOF3.insert(49);
    DOF3.insert(52);
    DOF3.insert(60);
    DOF3.insert(63);
    DOF3.insert(66);
    */

    Eigen::MatrixXd::RowXpr traj_start_point[] =
    {
        getElementTrajectory(COMPONENT_TYPE_POSITION, sub_component_type)->getTrajectoryPoint(point_start),
        getElementTrajectory(COMPONENT_TYPE_VELOCITY, sub_component_type)->getTrajectoryPoint(point_start),
        getElementTrajectory(COMPONENT_TYPE_ACCELERATION, sub_component_type)->getTrajectoryPoint(point_start)
    };

    Eigen::MatrixXd::RowXpr traj_goal_point[] =
    {
        getElementTrajectory(COMPONENT_TYPE_POSITION, sub_component_type)->getTrajectoryPoint(point_end),
        getElementTrajectory(COMPONENT_TYPE_VELOCITY, sub_component_type)->getTrajectoryPoint(point_end),
        getElementTrajectory(COMPONENT_TYPE_ACCELERATION, sub_component_type)->getTrajectoryPoint(point_end)
    };

    unsigned int elements = element_indices ? element_indices->size() : traj_start_point[0].cols();
    for (unsigned int j = 0; j < elements; ++j)
    {
        unsigned int index = element_indices ? (*element_indices)[j] : j;
        //if (sub_component_type != SUB_COMPONENT_TYPE_JOINT || index < 3)
        {
            double x0 = traj_start_point[COMPONENT_TYPE_POSITION](index);
            double v0 = traj_start_point[COMPONENT_TYPE_VELOCITY](index);
            double a0 = traj_start_point[COMPONENT_TYPE_ACCELERATION](index);

            double x1 = traj_goal_point[COMPONENT_TYPE_POSITION](index);
            double v1 = traj_goal_point[COMPONENT_TYPE_VELOCITY](index);
            double a1 = traj_goal_point[COMPONENT_TYPE_ACCELERATION](index);

            double duration = (point_end - point_start) * discretization_;

            ecl::QuinticPolynomial poly;
            poly = ecl::QuinticPolynomial::Interpolation(0, x0, v0, a0, duration, x1, v1, a1);
            for (unsigned int i = point_start + 1; i < point_end; ++i)
            {
                Eigen::MatrixXd::RowXpr traj_point[] =
                {
                    getElementTrajectory(COMPONENT_TYPE_POSITION, sub_component_type)->getTrajectoryPoint(i),
                    getElementTrajectory(COMPONENT_TYPE_VELOCITY, sub_component_type)->getTrajectoryPoint(i),
                    getElementTrajectory(COMPONENT_TYPE_ACCELERATION, sub_component_type)->getTrajectoryPoint(i)
                };
                traj_point[COMPONENT_TYPE_POSITION](index) = poly((i - point_start) * discretization_);
                traj_point[COMPONENT_TYPE_VELOCITY](index) = poly.derivative((i - point_start) * discretization_);
                traj_point[COMPONENT_TYPE_ACCELERATION](index) = poly.dderivative((i - point_start) * discretization_);
            }
        }
        /*
        else if (DOF3.find(index) != DOF3.end())
        {
            Eigen::Vector3d start_euler;
            for (unsigned int k = 0; k < 3; ++k)
            {
                start_euler(k) = traj_start_point[COMPONENT_TYPE_POSITION](index + k);
            }
            Eigen::Quaterniond start_rot = Eigen::AngleAxisd(start_euler(0), Eigen::Vector3d::UnitZ())
                    * Eigen::AngleAxisd(start_euler(1), Eigen::Vector3d::UnitY())
                    * Eigen::AngleAxisd(start_euler(2), Eigen::Vector3d::UnitX());
            double v0 = traj_start_point[COMPONENT_TYPE_VELOCITY](index);
            double a0 = traj_start_point[COMPONENT_TYPE_ACCELERATION](index);

            Eigen::Vector3d goal_euler;
            for (unsigned int k = 0; k < 3; ++k)
            {
                goal_euler(k) = traj_goal_point[COMPONENT_TYPE_POSITION](index + k);
            }
            Eigen::Quaterniond goal_rot = Eigen::AngleAxisd(goal_euler(0), Eigen::Vector3d::UnitZ())
                    * Eigen::AngleAxisd(goal_euler(1), Eigen::Vector3d::UnitY())
                    * Eigen::AngleAxisd(goal_euler(2), Eigen::Vector3d::UnitX());
            double v1 = traj_goal_point[COMPONENT_TYPE_VELOCITY](index);
            double a1 = traj_goal_point[COMPONENT_TYPE_ACCELERATION](index);

            double duration = (point_end - point_start) * discretization_;

            ecl::QuinticPolynomial poly;
            poly = ecl::QuinticPolynomial::Interpolation(0, 0, v0, a0, duration, 1, v1, a1);
            for (unsigned int i = point_start + 1; i < point_end; ++i)
            {
                Eigen::MatrixXd::RowXpr traj_point[] =
                {
                    getElementTrajectory(COMPONENT_TYPE_POSITION, sub_component_type)->getTrajectoryPoint(i),
                    getElementTrajectory(COMPONENT_TYPE_VELOCITY, sub_component_type)->getTrajectoryPoint(i),
                    getElementTrajectory(COMPONENT_TYPE_ACCELERATION, sub_component_type)->getTrajectoryPoint(i)
                };
                traj_point[COMPONENT_TYPE_VELOCITY](index) = poly.derivative((i - point_start) * discretization_);
                traj_point[COMPONENT_TYPE_ACCELERATION](index) = poly.dderivative((i - point_start) * discretization_);
                Eigen::Vector3d interpolated_euler = start_rot.slerp(poly((i - point_start) * discretization_), goal_rot).
                        toRotationMatrix().eulerAngles(2, 1, 0);
                traj_point[COMPONENT_TYPE_POSITION](index) = interpolated_euler(0);
                traj_point[COMPONENT_TYPE_POSITION](index + 1) = interpolated_euler(1);
                traj_point[COMPONENT_TYPE_POSITION](index + 2) = interpolated_euler(2);
            }
        }
        */

    }
}

void ItompTrajectory::copy(int point_src, int point_dest, SUB_COMPONENT_TYPE sub_component_type,
                           const std::vector<unsigned int>* element_indices)
{
    Eigen::MatrixXd::RowXpr copy_src_point[] =
    {
        getElementTrajectory(COMPONENT_TYPE_POSITION, sub_component_type)->getTrajectoryPoint(point_src),
        getElementTrajectory(COMPONENT_TYPE_VELOCITY, sub_component_type)->getTrajectoryPoint(point_src),
        getElementTrajectory(COMPONENT_TYPE_ACCELERATION, sub_component_type)->getTrajectoryPoint(point_src)
    };

    Eigen::MatrixXd::RowXpr copy_dest_point[] =
    {
        getElementTrajectory(COMPONENT_TYPE_POSITION, sub_component_type)->getTrajectoryPoint(point_dest),
        getElementTrajectory(COMPONENT_TYPE_VELOCITY, sub_component_type)->getTrajectoryPoint(point_dest),
        getElementTrajectory(COMPONENT_TYPE_ACCELERATION, sub_component_type)->getTrajectoryPoint(point_dest)
    };

    unsigned int elements = element_indices ? element_indices->size() :
                            copy_src_point[0].cols();

    for (unsigned int j = 0; j < elements; ++j)
    {
        unsigned int index = element_indices ? (*element_indices)[j] : j;

        copy_dest_point[COMPONENT_TYPE_POSITION](index) = copy_src_point[COMPONENT_TYPE_POSITION](index);
        copy_dest_point[COMPONENT_TYPE_VELOCITY](index) = copy_src_point[COMPONENT_TYPE_VELOCITY](index);
        copy_dest_point[COMPONENT_TYPE_ACCELERATION](index) = copy_src_point[COMPONENT_TYPE_ACCELERATION](index);
    }
}

bool ItompTrajectory::avoidNeighbors(const std::vector<moveit_msgs::Constraints>& neighbors)
{
    return true;

    if (neighbors.size() == 0)
        return true;

    ElementTrajectoryPtr joint = getElementTrajectory(COMPONENT_TYPE_POSITION, SUB_COMPONENT_TYPE_JOINT);

    double last_dist_to_move = std::numeric_limits<double>::max();
    double dist_to_move = 0.0;
    while (true)
    {
        dist_to_move = 0.0;

        for (int i = 0; i < neighbors[0].position_constraints.size(); ++i)
        {
            // 0 is the robot itself
            double my_radius = neighbors[0].position_constraints[i].target_point_offset.z;
            Eigen::Vector3d my_pos;
            my_pos(0) = (*joint)(i, 0);
            my_pos(1) = (*joint)(i, 1);

            for (int j = 1; j < neighbors.size(); ++j)
            {
                if (neighbors[j].position_constraints[i].weight < 0)
                    continue;

                double neighbor_radius = neighbors[j].position_constraints[i].target_point_offset.z;

                double sq_radius_sum = (my_radius + neighbor_radius) * (my_radius + neighbor_radius);

                Eigen::Vector3d neighbor_pos;
                neighbor_pos(0) = neighbors[j].position_constraints[i].target_point_offset.x;
                neighbor_pos(1) = neighbors[j].position_constraints[i].target_point_offset.y;

                double sq_dist = (my_pos - neighbor_pos).squaredNorm();

                if (sq_dist < sq_radius_sum)
                {
                    double dist = std::sqrt(sq_radius_sum - sq_dist);
                    dist_to_move += dist;

                    Eigen::Vector3d dir = (my_pos - neighbor_pos);
                    dir.normalize();

                    //(*joint)(i, 0) += dist * dir(0);
                    //(*joint)(i, 1) += dist * dir(1);
                }
            }
            if (/*i == 0 && */dist_to_move > 0)
            {
                ROS_INFO("Collide with neighbor %d", i);
                return false;
            }
        }

        if (dist_to_move == 0)
            break;

        if (dist_to_move >= last_dist_to_move)
            return false;
        else
            last_dist_to_move = dist_to_move;
    }

    return true;
}

bool ItompTrajectory::setJointPositions(Eigen::VectorXd& trajectory_data, const ParameterVector& parameters, int point) const
{
    bool updated = false;
    trajectory_data = getElementTrajectory(COMPONENT_TYPE_POSITION, SUB_COMPONENT_TYPE_JOINT)->getTrajectoryPoint(point);
    for (unsigned int i = 0; i < parameters.size(); ++i)
    {
        const ItompTrajectoryIndex& index = getTrajectoryIndex(i);
        if (index.sub_component == SUB_COMPONENT_TYPE_JOINT && index.component == COMPONENT_TYPE_POSITION && index.point == point)
        {
            trajectory_data(index.element) = parameters(i, 0);
            updated = true;
        }
    }
    return updated;
}
void ItompTrajectory::getJointPositions(ParameterVector& parameters, const Eigen::VectorXd& trajectory_data, int point) const
{
    for (unsigned int i = 0; i < parameters.size(); ++i)
    {
        const ItompTrajectoryIndex& index = getTrajectoryIndex(i);
        if (index.sub_component == SUB_COMPONENT_TYPE_JOINT && index.component == COMPONENT_TYPE_POSITION && index.point == point)
            parameters(i, 0) = trajectory_data(index.element);
    }
}

}

