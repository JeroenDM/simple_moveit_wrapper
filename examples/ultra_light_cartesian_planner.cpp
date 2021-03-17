#include <ros/ros.h>

#include <memory>

#include <simple_moveit_wrapper/industrial_robot.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include "range.h"

const std::string LOGNAME{ "ultra_light_planner" };

namespace smw = simple_moveit_wrapper;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ocpl_moveit_demo");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // setup MoveIt visual tools
    auto visual_tools = std::make_shared<moveit_visual_tools::MoveItVisualTools>("base_link", "/visualization_marker_"
                                                                                              "array");
    visual_tools->loadMarkerPub(true);
    visual_tools->loadRobotStatePub("/display_robot_state", true);
    visual_tools->deleteAllMarkers();
    ros::Duration(0.2).sleep();  // delay to make sure al the messages got where they had to be

    // Create an instance of a wrapper around MoveIt stuff
    smw::IndustrialRobot robot("manipulator", "tool0");

    // note that smw::JointPositions is just an std::vector<double>
    smw::JointPositions q_start{ 0.0, 0.5, 0.0, 0.0, -0.5, 0.0 };

    smw::Transform current_frame = robot.fk(q_start);
    std::vector<smw::JointPositions> path;
    double y_reference = current_frame.translation().y();

    for (auto y_offset : range(/* min */ -0.5, /* max */ 0.5, /* num points */ 20))
    {
        current_frame.translation().y() = y_reference + y_offset;

        // visualize the path
        visual_tools->publishAxis(current_frame);
        visual_tools->trigger();

        // find robot positions that can reach this path points
        std::vector<smw::JointPositions> ik_solutions = robot.ik(current_frame);
        if (ik_solutions.empty())
        {
            ROS_ERROR_STREAM_NAMED(LOGNAME, "No IK solution found for y_offset: " << y_offset);
            ROS_ERROR_NAMED(LOGNAME, "It will not be added to the path.");
        }
        else
        {
            // quickly show all the robot positions that can reach this path point
            for (auto q_sol : ik_solutions)
            {
                robot.plot(visual_tools, q_sol);
                ros::Duration(0.05).sleep();
            }

            // extremely simple "planner", just take the first ik solution that does not collide
            for (auto possible_solution : ik_solutions)
            {
                if (!robot.isColliding(possible_solution))
                {
                    path.push_back(possible_solution);
                    break;
                }
            }
        }
    }

    robot.animatePath(visual_tools, path);

    ros::shutdown();

    return 0;
}
