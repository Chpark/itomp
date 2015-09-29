#include <move_itomp/bvh_writer.h>

#include <string>
#include <fstream>


namespace bvh_writer
{

void writeRocketboxTrajectoryBVHFile(moveit_msgs::DisplayTrajectory& display_trajectory, const std::string& filename)
{
    std::ofstream out (filename.c_str());

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
            for (int k=0; k < display_trajectory.trajectory[i].joint_trajectory.points[j].positions.size(); ++k)
            {
                double value = display_trajectory.trajectory[i].joint_trajectory.points[j].positions[k];

                // radian to degree for angles (k>=3)
                if (k >= 3)
                    value *= 180.0 / M_PI;

                // duplicate rotation channels for endeffector and contact point
                if (joint_names[k].find("_endeffector_") != std::string::npos ||
                    joint_names[k].find("_cp_") != std::string::npos)
                {
                    out << value << ' '
                        << value << ' '
                        << value << ' ';
                }

                else
                {
                    out << value << ' ';
                }
            }

            out << std::endl;
        }
    }

    out.close();
}

}
