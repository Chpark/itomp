#include <move_itomp/bvh_writer.h>

#include <string>
#include <fstream>


namespace bvh_writer
{

void writeWalkingTrajectoryBVHFile(const robot_model::RobotModelPtr robot_model, const moveit_msgs::DisplayTrajectory& display_trajectory, const std::string& filename)
{
    std::ofstream out (filename.c_str());

    const std::vector<std::string>& model_joint_names = robot_model->getJointModelNames();
    const std::vector<std::string>& joint_names = display_trajectory.trajectory[0].joint_trajectory.joint_names;


    out << "MOTION" << std::endl;

    int num_frames = 0;
    for (int i=0; i < display_trajectory.trajectory.size(); ++i)
        num_frames += display_trajectory.trajectory[i].joint_trajectory.points.size();
    out << "Frames: " << num_frames << std::endl;

    out << "Frame Time: 0.0333333" << std::endl;

    for (int i=0; i < display_trajectory.trajectory.size(); ++i)
    {
        for (int j=0; j < display_trajectory.trajectory[i].joint_trajectory.points.size(); ++j)
        {
            for (int k=1; k < model_joint_names.size(); ++k)
            {
                if (model_joint_names[k].find("_endeffector_") != std::string::npos ||
                    model_joint_names[k].find("_cp_") != std::string::npos ||
                    model_joint_names[k].find("_beam_") != std::string::npos)
                    continue;

                double value = 0.0;

                for (int l=0; l < joint_names.size(); ++l)
                {
                    if (model_joint_names[k] == joint_names[l])
                    {
                        value = display_trajectory.trajectory[i].joint_trajectory.points[j].positions[l];

                        // radian to degree for angles (k=0: virtual joint, k=1~3: base prismatic joints)
                        if (k >= 4)
                            value *= 180.0 / M_PI;

                        break;
                    }
                }

                out << value << ' ';
            }

            out << std::endl;
        }
    }

    out.close();
}

}
