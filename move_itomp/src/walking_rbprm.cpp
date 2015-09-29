#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <move_itomp/move_itomp_util.h>
#include <move_itomp/rbprm_reader.h>

const double INV_SQRT_2 = 1.0 / std::sqrt((long double) 2.0);

using namespace move_itomp_util;
using namespace rbprm_reader;

int main(int argc, char **argv)
{
    int motion = 0;
    std::string initialpath = "";
    if (argc >= 2)
    {
        motion = atoi(argv[1]);
        if(argc >=3)
        {
            initialpath = argv[2];
        }
    }
    printf("%d Motion %d\n", argc, motion);

    ros::init(argc, argv, "move_itomp");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle node_handle("~");

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();

    planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));

    ros::Publisher planning_scene_diff_publisher;
    planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("/planning_scene", 1);
    while (planning_scene_diff_publisher.getNumSubscribers() < 1)
    {
        ros::WallDuration sleep_t(0.5);
        sleep_t.sleep();
        ROS_INFO("Waiting planning_scene subscribers");
    }

    boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager> > planner_plugin_loader;
    planning_interface::PlannerManagerPtr planner_instance;
    initializePlanner(planner_plugin_loader, planner_instance, node_handle, robot_model);

    loadStaticScene(node_handle, planning_scene, robot_model, planning_scene_diff_publisher);

    /* Sleep a little to allow time to startup rviz, etc. */
    ros::WallDuration sleep_time(1.0);
    sleep_time.sleep();

    std::vector<robot_state::RobotState> robot_states;

    robot_states.push_back(planning_scene->getCurrentStateNonConst());
    robot_states.push_back(robot_states.back());

    // set trajectory constraints
    std::vector<Eigen::VectorXd> waypoints;
    std::vector<Eigen::MatrixXd> contactPoints;
    std::vector<std::string> hierarchy;

    if(initialpath.empty())
    {
        ROS_ERROR("Initial path is empty");
    }

    hierarchy = InitTrajectoryFromFile(waypoints, contactPoints, initialpath);
    robot_state::RobotState rs(planning_scene->getCurrentStateNonConst());

    while (true)
        displayInitialWaypoints(rs, node_handle, robot_model, hierarchy, waypoints);

    //for (unsigned int i = 0; i < waypoints.size() - 1; ++i)
    for (unsigned int i = 0; i < 1; ++i)
    {
        planning_interface::MotionPlanRequest req;
        planning_interface::MotionPlanResponse res;

        for (unsigned int j = i; j <= i + 1; ++j)
        {
            //moveit_msgs::Constraints constraint = setRootJointAndContactPointConstraints(hierarchy, waypoints[j], contactPoints[j]);
            //req.trajectory_constraints.constraints.push_back(constraint);
        }
        setRobotStateFrom(robot_states[0], hierarchy, waypoints, i);
        setRobotStateFrom(robot_states[1], hierarchy, waypoints, i + 1);
        displayStates(robot_states[0], robot_states[1], node_handle, robot_model);
        //doPlan("whole_body", req, res, robot_states[0], robot_states[1], planning_scene, planner_instance);
        //visualizeResult(res, node_handle, 0, 1.0);
    }

    ROS_INFO("Done");

    return 0;
}

