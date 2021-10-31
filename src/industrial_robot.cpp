#include <simple_moveit_wrapper/industrial_robot.h>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/link_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <opw_kinematics/opw_parameters.h>
#include <opw_kinematics/opw_kinematics.h>
#include <opw_kinematics/opw_utilities.h>
#include <opw_kinematics/opw_io.h>

namespace simple_moveit_wrapper
{
IndustrialRobot::IndustrialRobot(const std::string& planning_group, const std::string& tcp_frame)
  : Robot(planning_group, tcp_frame)
{
    group_name_ = joint_model_group_->getName();
    if (num_dof_ > num_base_joints_)  // dof > 6 for 3D robots
    {
        is_redundant_ = true;
        num_red_joints_ = num_dof_ - num_base_joints_;
        ROS_INFO_STREAM("Robot is redundant with " << num_red_joints_ << " redundant joints.");
    }
    setOPWParameters();
    messyHardCodedStuff();
    num_red_dof_ = num_red_joints_;
}

std::vector<JointPositions> IndustrialRobot::ik(const Transform& tf) const
{
    if (is_redundant_)
    {
        std::vector<double> zeros(num_dof_ - num_base_joints_, 0.0);
        return ik(tf, zeros);
    }
    else
    {
        return ik(tf, {});
    }
}

std::vector<JointPositions> IndustrialRobot::ik(const Transform& pose, const std::vector<double>& q_redundant) const
{
    std::vector<JointPositions> solutions;

    auto tf_tool0 = pose * tool0_to_tcp_inverse_;

    if (is_redundant_)
    {
        assert(q_redundant.size() == num_red_joints_);
        // get base frame for 6dof robot ik
        std::vector<double> q_dummy(num_dof_, 0.0);
        for (std::size_t i{}; i < num_red_joints_; ++i)
        {
            q_dummy[i] = q_redundant[i];
        }
        auto base_link_pose = fk(q_dummy, "base_link");

        // tranfrom pose to reference frame of 6dof robot
        tf_tool0 = base_link_pose.inverse() * tf_tool0;
    }

    std::array<std::array<double, 6>, 8> sols = opw_kinematics::inverse(opw_parameters_, tf_tool0);

    // Check the output
    for (auto& q : sols)
    {
        if (opw_kinematics::isValid(q))
        {
            opw_kinematics::harmonizeTowardZero(q);
            solutions.emplace_back(std::vector<double>{ q.begin(), q.end() });
        }
    }

    if (is_redundant_)
    {
        // add fixed joint values to all solutions
        // TODO is this inserting slow?
        for (auto& q_sol : solutions)
        {
            q_sol.insert(q_sol.begin(), q_redundant.begin(), q_redundant.end());
        }
    }

    return solutions;
}

void IndustrialRobot::messyHardCodedStuff()
{
    ROS_DEBUG_STREAM("tcp frame: " << tcp_frame_);
    if (tcp_frame_ == "tool0")
    {
        // no tool seems to be mounted on the robot
        tool0_to_tcp_ = Transform::Identity();
        tool0_to_tcp_inverse_ = Transform::Identity();
    }
    else
    {
        // Find the transform between the end-effector tip link
        // and the tool0 reference for the analytical inverse kinematics solver
        auto names = joint_model_group_->getLinkModelNames();
        std::vector<std::string> offset_chain;
        bool tool0_found{ false };
        for (const std::string& name : names)
        {
            ROS_DEBUG_STREAM(name);
            if (tool0_found)
                offset_chain.push_back(name);

            if (name == "tool0")
                tool0_found = true;
        }

        ROS_DEBUG_STREAM("Found offset chain with length: " << offset_chain.size());
        for (auto s : offset_chain)
            std::cout << s << ", ";
        std::cout << std::endl;

        tool0_to_tcp_ = Transform::Identity();
        for (std::string name : offset_chain)
        {
            tool0_to_tcp_ = tool0_to_tcp_ * getLinkFixedRelativeTransform(name);
        }
        tool0_to_tcp_inverse_ = tool0_to_tcp_.inverse();
        ROS_DEBUG_STREAM("Offset transform: " << tool0_to_tcp_.translation().transpose());
    }
}

bool IndustrialRobot::setOPWParameters()
{
    ROS_INFO_STREAM("Getting kinematic parameters from parameter server.");

    ros::NodeHandle nh;

    std::map<std::string, double> geometric_parameters;
    if (!lookupParam("opw_kinematics_geometric_parameters", geometric_parameters, {}))
    {
        ROS_ERROR_STREAM("Failed to load geometric parameters for ik solver.");
        return false;
    }

    std::vector<double> joint_offsets;
    if (!lookupParam("opw_kinematics_joint_offsets", joint_offsets, {}))
    {
        ROS_ERROR_STREAM("Failed to load joint offsets for ik solver.");
        return false;
    }

    std::vector<int> joint_sign_corrections;
    if (!lookupParam("opw_kinematics_joint_sign_corrections", joint_sign_corrections, {}))
    {
        ROS_ERROR_STREAM("Failed to load joint sign corrections for ik solver.");
        return false;
    }

    opw_parameters_.a1 = geometric_parameters["a1"];
    opw_parameters_.a2 = geometric_parameters["a2"];
    opw_parameters_.b = geometric_parameters["b"];
    opw_parameters_.c1 = geometric_parameters["c1"];
    opw_parameters_.c2 = geometric_parameters["c2"];
    opw_parameters_.c3 = geometric_parameters["c3"];
    opw_parameters_.c4 = geometric_parameters["c4"];

    if (joint_offsets.size() != 6)
    {
        ROS_ERROR_STREAM("Expected joint_offsets to contain 6 elements, but it has " << joint_offsets.size() << ".");
        return false;
    }

    if (joint_sign_corrections.size() != 6)
    {
        ROS_ERROR_STREAM("Expected joint_sign_corrections to contain 6 elements, but it has "
                         << joint_sign_corrections.size() << ".");
        return false;
    }

    for (std::size_t i = 0; i < joint_offsets.size(); ++i)
    {
        opw_parameters_.offsets[i] = joint_offsets[i];
        opw_parameters_.sign_corrections[i] = static_cast<signed char>(joint_sign_corrections[i]);
    }

    ROS_INFO_STREAM("Loaded parameters for ik solver:\n" << opw_parameters_);

    return true;
}
}  // namespace simple_moveit_wrapper
