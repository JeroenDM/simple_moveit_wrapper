#include <simple_moveit_wrapper/planar_robot.h>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/link_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <simple_moveit_wrapper/inverse_kinematics/planar_3r_ik.h>

namespace simple_moveit_wrapper
{
PlanarRobot::PlanarRobot(const std::string& planning_group, const std::string& tcp_frame) : Robot(planning_group, tcp_frame)
{
    assert(num_dof_ > 3);  // this class is for planar redundant robots
    messyHardCodedStuff();
}

void PlanarRobot::messyHardCodedStuff()
{
    // guess the base link of the last three joints alternative
    std::size_t link_index = num_dof_ - num_base_joints_;
    analytical_ik_base_link_ =
        joint_model_group_->getActiveJointModels().at(link_index)->getChildLinkModel()->getName();

    // get the link lengths of the last three links
    auto joint_models = joint_model_group_->getActiveJointModels();

    // iterate over the last two active links of the robot
    for (int i{ 2 }; i > 0; --i)
    {
        std::string name = joint_models[num_dof_ - i]->getChildLinkModel()->getName();
        double length = getLinkFixedRelativeTransform(name).translation().norm();
        analytical_ik_link_length_.push_back(length);
    }
    // the last link length is measured towards the flange
    analytical_ik_link_length_.push_back(getLinkFixedRelativeTransform("flange").translation().norm());
}

std::vector<JointPositions> PlanarRobot::ik(const Transform& tf)
{
    std::vector<double> zeros(num_dof_ - num_base_joints_, 0.0);
    return ik(tf, zeros);
}

std::vector<JointPositions> PlanarRobot::ik(const Transform& tf, const std::vector<double>& q_fixed)
{
    // set ik based given the fixed joint values q_fixed
    std::vector<double> q_temp(num_dof_, 0.0);
    for (std::size_t i{ 0 }; i < (num_dof_ - num_base_joints_); ++i)
        q_temp[i] = q_fixed[i];

    auto tf_ik_base = fk(q_temp, analytical_ik_base_link_);

    // account ik_base_frame and fixed transform of tcp
    auto flange_to_tool0 = getLinkFixedRelativeTransform("tool") * getLinkFixedRelativeTransform("tool0");
    auto tf_local = tf_ik_base.inverse() * tf * flange_to_tool0.inverse();

    // find the rotation of the last link around the z-axis in the tf_ik_base frame
    Eigen::Vector3d pos = tf_local.translation();
    Eigen::Vector3d rot = tf_local.rotation().eulerAngles(0, 1, 2);

    auto solution = ik::planar_3r_ik(pos[0], pos[1], rot[2], analytical_ik_link_length_);

    // add fixed joint values to all solutions
    // TODO is this inserting slow?
    for (auto& q_sol : solution)
    {
        q_sol.insert(q_sol.begin(), q_fixed.begin(), q_fixed.end());
    }
    return solution;
}
}  // namespace simple_moveit_wrapper
