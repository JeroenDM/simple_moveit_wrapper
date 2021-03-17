#include <ros/ros.h>

#include <memory>

#include <simple_moveit_wrapper/robot.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

namespace smw = simple_moveit_wrapper;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ocpl_moveit_demo");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // setup MoveIt visual tools
    auto visual_tools = std::make_shared<moveit_visual_tools::MoveItVisualTools>("base_link", "/visualization_marker_array");
    visual_tools->loadMarkerPub(true);
    visual_tools->loadRobotStatePub("/display_robot_state", true);
    ros::Duration(0.2).sleep();  // delay to make sure al the messages got where they had to be

    // Create an instance of a wrapper around MoveIt stuff
    smw::Robot robot("manipulator", "tool0");

    // Visualize robot joint positions in Rviz
    std::vector<double> q1 {0.6, -0.5, 0.4, -0.3, 0.2, -0.1};
    robot.plot(visual_tools, q1);

    // Calculate the forward kinematics and visualize the end-effector frame in Rviz
    auto ee_frame = robot.fk(q1);
    visual_tools->publishAxis(ee_frame);
    visual_tools->trigger();
    ros::Duration(0.05).sleep(); // these visual tools take their time

    ros::shutdown();

    return 0;
}
