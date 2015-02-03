#include <itomp_cio_planner/trajectory/itomp_trajectory.h>
#include <itomp_cio_planner/util/joint_state_util.h>
#include <ros/assert.h>
#include <ecl/geometry/polynomial.hpp>

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

ItompTrajectory::~ItompTrajectory()
{

}

void ItompTrajectory::setStartState(const sensor_msgs::JointState &joint_state,
                                    const ItompRobotModelConstPtr& robot_model)
{
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
            traj_start_point[COMPONENT_TYPE_VELOCITY](rbdl_number) = joint_state.velocity[i];
            traj_start_point[COMPONENT_TYPE_ACCELERATION](rbdl_number) = joint_state.effort[i];
            ROS_INFO("[%d] %s : %f %f %f", rbdl_number, name.c_str(), joint_state.position[i], joint_state.velocity[i], joint_state.effort[i]);
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
        interpolateInputTrajectory(group_rbdl_indices, planning_group,
                                   trajectory_constraints);
    }
    else
    {
        interpolateStartEnd(group_rbdl_indices);
    }
}

void ItompTrajectory::interpolateStartEnd(const std::vector<unsigned int>& group_rbdl_indices)
{
    Eigen::MatrixXd::RowXpr traj_start_point[] =
    {
        getElementTrajectory(COMPONENT_TYPE_POSITION, SUB_COMPONENT_TYPE_JOINT)->getTrajectoryPoint(0),
        getElementTrajectory(COMPONENT_TYPE_VELOCITY, SUB_COMPONENT_TYPE_JOINT)->getTrajectoryPoint(0),
        getElementTrajectory(COMPONENT_TYPE_ACCELERATION, SUB_COMPONENT_TYPE_JOINT)->getTrajectoryPoint(0)
    };
    unsigned int goal_index = getNumPoints() - 1;
    Eigen::MatrixXd::RowXpr traj_goal_point = getElementTrajectory(COMPONENT_TYPE_POSITION, SUB_COMPONENT_TYPE_JOINT)->getTrajectoryPoint(goal_index);

    for (unsigned int j = 0; j < group_rbdl_indices.size(); ++j)
    {
        unsigned int rbdl_index = group_rbdl_indices[j];

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
            traj_point[COMPONENT_TYPE_ACCELERATION](rbdl_index) =	poly.dderivative(i * discretization_);
        }
    }
}

void ItompTrajectory::interpolateInputTrajectory(const std::vector<unsigned int>& group_rbdl_indices,
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
                traj_point[COMPONENT_TYPE_ACCELERATION](rbdl_index) =	poly.dderivative(i * discretization_);
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
                poly = ecl::QuinticPolynomial::Interpolation(
                           from, x0, v0, a0,
                           to, x1, v1, a1);
                for (int i = from; i <= to; ++i)
                {
                    Eigen::MatrixXd::RowXpr traj_point[] =
                    {
                        getElementTrajectory(COMPONENT_TYPE_POSITION, SUB_COMPONENT_TYPE_JOINT)->getTrajectoryPoint(i),
                        getElementTrajectory(COMPONENT_TYPE_VELOCITY, SUB_COMPONENT_TYPE_JOINT)->getTrajectoryPoint(i),
                        getElementTrajectory(COMPONENT_TYPE_ACCELERATION, SUB_COMPONENT_TYPE_JOINT)->getTrajectoryPoint(i)
                    };
                    traj_point[COMPONENT_TYPE_POSITION](rbdl_index) = poly(i * discretization_);
                    traj_point[COMPONENT_TYPE_VELOCITY](rbdl_index) = poly.derivative(i * discretization_);
                    traj_point[COMPONENT_TYPE_ACCELERATION](rbdl_index) =	poly.dderivative(i * discretization_);
                }

            }
        }
    }
}

void ItompTrajectory::interpolateKeyframes()
{
    if (keyframe_interval_ <= 1)
        return;

    // cubic interpolation of pos, vel, acc
    // update trajectory between (k, k+1]
    // acc is discontinuous at each keyframe
    for (unsigned int s = 0; s < SUB_COMPONENT_TYPE_NUM; ++s)
    {
        unsigned int num_sub_component_elements = getElementTrajectory(0, s)->getNumElements();
        for (unsigned int j = 0; j < num_sub_component_elements; ++j)
        {
            // skip the initial position
            unsigned int current_point = 1;
            for (unsigned int k = 0; k < num_keyframes_ - 1; ++k)
            {
                ecl::CubicPolynomial poly;
                unsigned int cur_keyframe_index = k * keyframe_interval_;
                unsigned int next_keyframe_index = cur_keyframe_index + keyframe_interval_;

                poly = ecl::CubicPolynomial::DerivativeInterpolation(
                           0.0,
                           getElementTrajectory(COMPONENT_TYPE_POSITION, s)->at(cur_keyframe_index, j),
                           getElementTrajectory(COMPONENT_TYPE_VELOCITY, s)->at(cur_keyframe_index, j),
                           (double)keyframe_interval_,
                           getElementTrajectory(COMPONENT_TYPE_POSITION, s)->at(next_keyframe_index, j),
                           getElementTrajectory(COMPONENT_TYPE_VELOCITY, s)->at(next_keyframe_index, j));

                for (unsigned int i = 1; i <= keyframe_interval_; ++i)
                {
                    if (current_point != getNumPoints() - 1)
                    {
                        double t = i * discretization_;
                        getElementTrajectory(COMPONENT_TYPE_POSITION, s)->at(current_point, j) = poly(t);
                        getElementTrajectory(COMPONENT_TYPE_VELOCITY, s)->at(current_point, j) = poly.derivative(t);
                        getElementTrajectory(COMPONENT_TYPE_ACCELERATION, s)->at(current_point, j) = poly.dderivative(t);
                    }
                    ++current_point;
                }
            }
        }
    }
}

void ItompTrajectory::interpolateTrajectory(unsigned int trajectory_point_begin, unsigned int trajectory_point_end,
        const ItompTrajectoryIndex& index)
{
    if (keyframe_interval_ <= 1)
        return;

    unsigned int sub_component_index = index.indices[1];
    unsigned int element = index.indices[3];

    // skip the initial position
    ecl::CubicPolynomial poly;
    unsigned int cur_keyframe_index = trajectory_point_begin;
    unsigned int next_keyframe_index = trajectory_point_end;

    poly = ecl::CubicPolynomial::DerivativeInterpolation(
               (double)cur_keyframe_index,
               getElementTrajectory(COMPONENT_TYPE_POSITION, sub_component_index)->at(cur_keyframe_index, element),
               getElementTrajectory(COMPONENT_TYPE_VELOCITY, sub_component_index)->at(cur_keyframe_index, element),
               (double)next_keyframe_index,
               getElementTrajectory(COMPONENT_TYPE_POSITION, sub_component_index)->at(next_keyframe_index, element),
               getElementTrajectory(COMPONENT_TYPE_VELOCITY, sub_component_index)->at(next_keyframe_index, element));

    for (unsigned int i = cur_keyframe_index; i < next_keyframe_index; ++i)
    {
        double t = i * discretization_;
        getElementTrajectory(COMPONENT_TYPE_POSITION, sub_component_index)->at(i, element) = poly(t);
        getElementTrajectory(COMPONENT_TYPE_VELOCITY, sub_component_index)->at(i, element) = poly.derivative(t);
        getElementTrajectory(COMPONENT_TYPE_ACCELERATION, sub_component_index)->at(i, element) = poly.dderivative(t);
    }
}

void ItompTrajectory::setFromParameter(ParameterVector& parameter)
{
    unsigned int num_parameters = parameter_to_index_map_.size();
    unsigned int goal_index = getNumPoints() - 1;

    ROS_ASSERT(num_parameters > 0);

    for (unsigned int i = 0; i < num_parameters; ++i)
    {
        ItompTrajectoryIndex index = parameter_to_index_map_[i];

        // Do not update joint values of start/goal points
        if (index.indices[1] == SUB_COMPONENT_TYPE_JOINT &&
                (index.indices[2] == 0 || index.indices[2] == getNumPoints() - 1))
            continue;

        ElementTrajectoryPtr& et = getElementTrajectory(index.indices[0], index.indices[1]);
        Eigen::MatrixXd::RowXpr row = et->getTrajectoryPoint(index.indices[2]);
        row(index.indices[3]) = parameter(i, 0);
    }
    interpolateKeyframes();
}

void ItompTrajectory::directChangeForDerivativeComputation(unsigned int parameter_index, double value,
        unsigned int& trajectory_point_begin, unsigned int& trajectory_point_end,
        bool backup)
{
    ROS_ASSERT(parameter_to_index_map_.size() > 0);

    ItompTrajectoryIndex index = parameter_to_index_map_[parameter_index];

    int point = index.indices[2];
    int element = index.indices[3];

    trajectory_point_begin = std::max(0, point - (int)keyframe_interval_);
    trajectory_point_end = std::min(num_points_ - 1, point + keyframe_interval_);

    if (backup)
        backupTrajectory(index);

    // Do not update joint values of start/goal points
    if (index.indices[1] == SUB_COMPONENT_TYPE_JOINT &&
            (index.indices[2] == 0 || index.indices[2] == getNumPoints() - 1))
        return;

    // set value
    getElementTrajectory(index.indices[0], index.indices[1])->at(point, element) = value;

    interpolateTrajectory(trajectory_point_begin, trajectory_point_end, index);
}

void ItompTrajectory::backupTrajectory(const ItompTrajectoryIndex& index)
{
    int point = index.indices[2];
    int element = index.indices[3];
    int backup_point_begin = std::max(0, point - (int)keyframe_interval_);
    int backup_point_end = std::min(num_points_ - 1, point + keyframe_interval_);
    int backup_length = backup_point_end - backup_point_begin;

    for (unsigned int i = 0; i < COMPONENT_TYPE_NUM; ++i)
    {
        backup_trajectory_[i].block(0, 0, backup_length, 1) =
            getElementTrajectory(i, index.indices[1])->getData().block(
                backup_point_begin, element, backup_length, 1);
    }
    backup_index_ = index;
}

void ItompTrajectory::restoreTrajectory()
{
    int point = backup_index_.indices[2];
    int element = backup_index_.indices[3];
    int backup_point_begin = std::max(0, point - (int)keyframe_interval_);
    int backup_point_end = std::min(num_points_ - 1, point + keyframe_interval_);
    int backup_length = backup_point_end - backup_point_begin;

    for (unsigned int i = 0; i < COMPONENT_TYPE_NUM; ++i)
    {
        getElementTrajectory(i, backup_index_.indices[1])->getData().block(
            backup_point_begin, element, backup_length, 1) =
                backup_trajectory_[i].block(0, 0, backup_length, 1);
    }
}

void ItompTrajectory::computeParameterToTrajectoryIndexMap(const ItompRobotModelConstPtr& robot_model,
        const ItompPlanningGroupConstPtr& planning_group)
{
    int num_parameter_joints = planning_group->num_joints_;
    std::vector<unsigned int> parameter_to_full_joint_indices(num_parameter_joints);
    for (unsigned int i = 0; i < num_parameter_joints; ++i)
        parameter_to_full_joint_indices[i] = planning_group->group_joints_[i].rbdl_joint_index_;

    unsigned int num_full_joints = robot_model->getNumJoints();
    unsigned int num_contact_positions = planning_group->getNumContacts() * 7; // var + pos(3) + ori(3)
    unsigned int num_contact_forces = planning_group->getNumContacts() * NUM_ENDEFFECTOR_CONTACT_POINTS * 3; // n * force(3)

    unsigned int parameter_size = num_keyframes_ * 2 * (num_parameter_joints + num_contact_positions + num_contact_forces);
    parameter_to_index_map_.resize(parameter_size);

    unsigned int parameter_pos = 0;
    for (unsigned int i = 0; i < num_keyframes_; ++i)
    {
        unsigned int keyframe_pos = i * keyframe_interval_;

        // pos, vel
        for (unsigned int j = 0; j < 2; ++j)
        {
            // indices for joints
            for (unsigned int k = 0; k < num_parameter_joints; ++k)
            {
                ItompTrajectoryIndex& index = parameter_to_index_map_[parameter_pos++];

                index.indices[0] = j;
                index.indices[1] = 0;
                index.indices[2] = keyframe_pos;
                index.indices[3] = parameter_to_full_joint_indices[k];
            }

            // indices for contact pos
            for (unsigned int k = 0; k < num_contact_positions; ++k)
            {
                ItompTrajectoryIndex& index = parameter_to_index_map_[parameter_pos++];

                index.indices[0] = j;
                index.indices[1] = 1;
                index.indices[2] = keyframe_pos;
                index.indices[3] = num_full_joints + k;
            }

            // indices for contact forces
            for (unsigned int k = 0; k < num_contact_positions; ++k)
            {
                ItompTrajectoryIndex& index = parameter_to_index_map_[parameter_pos++];

                index.indices[0] = j;
                index.indices[1] = 2;
                index.indices[2] = keyframe_pos;
                index.indices[3] = num_full_joints + num_contact_positions + k;
            }
        }
    }
}

}

