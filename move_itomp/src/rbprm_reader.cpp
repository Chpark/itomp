#include <move_itomp/rbprm_reader.h>
#include <string>
#include <sstream>
#include <fstream>

namespace rbprm_reader
{
//load initial path
// file has following form (like a bvh)
// HIERARCHY
// Ordered list of all joints for which there are constraints
// MOTION
// Frames:	16
// Frame Time: 1
// "X Y Z Rx Ry Rz J1 J2... Jn" one line per state
// "CONTACT EFFECTORS N"  N is the number of effectors
// "hasContact X Y Z qx qy qz qw" hasContact 0 or 1, times number effectors one line per state
std::vector<std::string> InitTrajectoryFromFile(std::vector<Eigen::VectorXd>& waypoints, std::vector<Eigen::MatrixXd>& contactPoints, const std::string& filepath)
{
    std::vector<std::string> res;
    bool hierarchy = false;
    bool motion = false;
    bool contacts = false;
    double offset [3] = {0,0,0};
    int nbEffectors = 0;
    std::ifstream myfile (filepath.c_str());
    if (myfile.is_open())
    {
        std::string line;
        while (myfile.good())
        {
            getline(myfile, line);
            if(line.find("HIERARCHY") != std::string::npos)
            {
                hierarchy = true;
            }
            else if(line.find("OFFSET ") != std::string::npos)
            {
                hierarchy = false;
                line = line.substr(7);
                char *endptr;
                int h = 0;
                offset[h++] = strtod(line.c_str(), &endptr);
                for(; h< 3; ++h)
                {
                    double tg = strtod(endptr, &endptr);
                    offset[h] = tg; // strtod(endptr, &endptr);
                }
            }
            else if(line.find("MOTION") != std::string::npos)
            {
                hierarchy = false;
            }
            else if(line.find("Frame Time") != std::string::npos)
            {
                motion = true;
            }
            else if(line.find("CONTACT EFFECTORS ") != std::string::npos)
            {
                motion = false;
                contacts = true;
                line = line.substr(18);
                char *endptr;
                nbEffectors = (int)strtod(line.c_str(), &endptr);
            }
            else if(!line.empty())
            {
                if(hierarchy)
                {
                    res.push_back(line);
                }
                else if(motion)
                {
                    Eigen::VectorXd waypoint(res.size());
                    char *endptr;
                    unsigned int h = 0;
                    waypoint[h++] = strtod(line.c_str(), &endptr);
                    for(; h< res.size(); ++h)
                    {
                        waypoint[h] = strtod(endptr, &endptr);
                    }
                    for(int i =0; i<3; ++i)
                    {
                        waypoint[i] += offset[i];
                    }
                    waypoint[2] -= 1;
                    waypoints.push_back(waypoint);
                }
                else if(contacts)
                {
                    Eigen::MatrixXd contactPoint(nbEffectors, 8);
                    char *endptr;
                    for(int i=0; i< nbEffectors; ++i)
                    {
                        int h = 0;
                        if (i == 0)
                            contactPoint(i,h++) = strtod(line.c_str(), &endptr);
                        else
                            contactPoint(i,h++) = strtod(endptr, &endptr);

                        for(; h< 8; ++h)
                        {
                            contactPoint(i,h) = strtod(endptr, &endptr);
                        }
                        for(int k =1; k<4; ++k)
                        {
                            contactPoint(i,k) += offset[k];
                        }
                        contactPoint(i,3) -= 1;
                    }
                    contactPoints.push_back(contactPoint);
                }
            }
        }
        myfile.close();
    }
    else
    {
        std::cout << "can not find initial path file" << filepath << std::endl;
    }
    if(!(contactPoints.empty() || waypoints.size() == contactPoints.size()))
    {
        std::cout << "Error in initial path file: not same number of contacts and frames" << filepath << std::endl;
    }
    return res;
}

void displayInitialWaypoints(robot_state::RobotState& state,
                             ros::NodeHandle& node_handle,
                             robot_model::RobotModelPtr& robot_model,
                             const std::vector<std::string>& hierarchy,
                             const std::vector<Eigen::VectorXd>& waypoints)
{
    static ros::Publisher vis_marker_array_publisher = node_handle.advertise<visualization_msgs::MarkerArray>("/move_itomp/visualization_marker_array", 10);

    visualization_msgs::MarkerArray ma;
    std::vector<std::string> link_names = robot_model->getLinkModelNames();
    std_msgs::ColorRGBA color;
    color.a = 0.5;
    color.r = 1.0;
    color.g = 1.0;
    color.b = 0.0;

    ros::Duration dur(3600.0);
    //ros::Duration dur(0.25);

    for (unsigned int point = 0; point < waypoints.size(); ++point)
    {
        ma.markers.clear();

        setRobotStateFrom(state, hierarchy, waypoints, point);


        double time = 0.05;
        ros::WallDuration timer(time);
        timer.sleep();


        std::string ns = "init_" + boost::lexical_cast<std::string>(point);
        state.getRobotMarkers(ma, link_names, color, ns, dur);
        vis_marker_array_publisher.publish(ma);
    }
}



moveit_msgs::Constraints setRootJointConstraint(moveit_msgs::Constraints& c,
                                                const std::vector<std::string>& hierarchy,
                                                const Eigen::VectorXd& transform)
{
    moveit_msgs::JointConstraint jc;
    moveit_msgs::OrientationConstraint oc;

    int id = 0;
    for(std::vector<std::string>::const_iterator cit= hierarchy.begin();
            cit != hierarchy.end(); ++cit, ++id)
    {
        jc.joint_name = *cit;
        jc.position = transform(id);
        c.joint_constraints.push_back(jc);
    }
    return c;
}

moveit_msgs::Constraints setContactPointConstraint(moveit_msgs::Constraints& c,
                                                   const std::vector<std::string>& hierarchy,
                                                   const Eigen::MatrixXd& transform)
{
    for(int i=0; i< transform.rows(); ++i) // rows is number of contacts
    {
        moveit_msgs::JointConstraint activeContact; // TODO Should I use something else ?
        moveit_msgs::PositionConstraint pc;
        moveit_msgs::OrientationConstraint oc;
        std::stringstream ss;
        ss << "contact_" << i;
        std::string str = ss.str();
        activeContact.joint_name = str;
        activeContact.position = transform(i,0);
        pc.link_name = str;
        pc.target_point_offset.x = transform(i,1);
        pc.target_point_offset.y = transform(i,2);
        pc.target_point_offset.z = transform(i,3);
        oc.link_name = str;
        oc.orientation.x = transform(i,4);
        oc.orientation.y = transform(i,5);
        oc.orientation.z = transform(i,6);
        oc.orientation.w = transform(i,7);
        c.position_constraints.push_back(pc);
        c.orientation_constraints.push_back(oc);
        c.joint_constraints.push_back(activeContact);
    }
    return c;
}

moveit_msgs::Constraints setRootJointAndContactPointConstraints(const std::vector<std::string>& hierarchy,
                                                                const Eigen::VectorXd& joint_transform,
                                                                const Eigen::MatrixXd& contacts)
{
    moveit_msgs::Constraints c;

    setRootJointConstraint(c, hierarchy, joint_transform);
    setContactPointConstraint(c, hierarchy, contacts);
    return c;
}

void setRobotStateFrom(robot_state::RobotState& state,
                       const std::vector<std::string>& hierarchy,
                       const std::vector<Eigen::VectorXd>& waypoints,
                       int index)
{
    std::map<std::string, double> values;
    double jointValue = 0.0;
    const robot_state::JointModelGroup* joint_model_group = state.getJointModelGroup("whole_body");

    joint_model_group->getVariableDefaultPositions("standup", values);
    state.setVariablePositions(values);

    int id = 0;
    for(std::vector<std::string>::const_iterator cit = hierarchy.begin(); cit != hierarchy.end(); ++cit, ++id)
    {
        jointValue = waypoints[index](id);
        state.setJointPositions(*cit, &jointValue);
    }
}

}
