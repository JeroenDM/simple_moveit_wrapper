#include <simple_moveit_wrapper/robot.h>

#include <string>
#include <vector>

#include <Eigen/Dense>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/link_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <simple_moveit_wrapper/math.h>

namespace simple_moveit_wrapper
{
Robot::Robot(const std::string& planning_group, const std::string& tcp_frame)
  : planning_group_(planning_group), tcp_frame_(tcp_frame), num_red_dof_{ 0 }
{
    // load robot model
    robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>("robot_description");
    // robot_model_loader::RobotModelLoader
    // robot_model_loader("robot_description");
    robot_model_ = robot_model_loader_->getModel();
    ROS_INFO("Model frame: %s", robot_model_->getModelFrame().c_str());

    state_storage_.init(robot_model_);

    joint_model_group_ = robot_model_->getJointModelGroup(planning_group_);

    // create planning scene to for collision checking
    planning_scene_.reset(new planning_scene::PlanningScene(robot_model_));
    planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
    updatePlanningScene();

    num_dof_ = joint_model_group_->getActiveJointModelNames().size();

    for (auto jm : joint_model_group_->getActiveJointModels())
    {
        joint_position_limits_.push_back(
            { jm->getVariableBounds().at(0).min_position_, jm->getVariableBounds().at(0).max_position_ });
        joint_velocity_limits_.push_back(
            { jm->getVariableBounds().at(0).min_velocity_, jm->getVariableBounds().at(0).max_velocity_ });
    }

    ROS_DEBUG_STREAM("Number of DOFS: " << num_dof_);
    auto joint_names = joint_model_group_->getActiveJointModelNames();
    for (const std::string& name : joint_names)
    {
        ROS_DEBUG_STREAM("Joint name: " << name);
    }
}

const Transform& Robot::getLinkFixedRelativeTransform(const std::string& frame) const
{
    auto robot_state = state_storage_.getAState();
    return robot_state->getLinkModel(frame)->getJointOriginTransform();
}

Transform Robot::fk(const std::vector<double>& q) const
{
    auto robot_state = state_storage_.getAState();
    robot_state->setJointGroupPositions(joint_model_group_, q);
    return robot_state->getGlobalLinkTransform(tcp_frame_);
}

Transform Robot::fk(const Eigen::Ref<const Eigen::VectorXd>& q) const
{
    auto robot_state = state_storage_.getAState();
    robot_state->setJointGroupPositions(joint_model_group_, q);
    return robot_state->getGlobalLinkTransform(tcp_frame_);
}

Transform Robot::fk(const std::vector<double>& q, const std::string& frame) const
{
    auto robot_state = state_storage_.getAState();
    robot_state->setJointGroupPositions(joint_model_group_, q);
    return robot_state->getGlobalLinkTransform(frame);
}

Transform Robot::fk(const Eigen::Ref<const Eigen::VectorXd>& q, const std::string& frame) const
{
    auto robot_state = state_storage_.getAState();
    robot_state->setJointGroupPositions(joint_model_group_, q);
    return robot_state->getGlobalLinkTransform(frame);
}

std::vector<JointPositions> Robot::ik(const Transform& tf) const
{
    auto robot_state = state_storage_.getAState();
    double timeout = 0.1;
    robot_state->setToDefaultValues();  // use a deterministic state to initialize
                                        // IK solver
    bool found_ik = robot_state->setFromIK(joint_model_group_, tf, timeout);
    std::vector<JointPositions> sol;
    if (found_ik)
    {
        std::vector<double> joint_values;
        robot_state->copyJointGroupPositions(joint_model_group_, joint_values);
        sol.push_back(joint_values);
    }
    else
    {
        ROS_INFO_STREAM("Failed to find ik solution.");
    }
    return sol;
}

std::vector<JointPositions> Robot::ik(const Transform& tf, const std::vector<double>& q_redundant) const
{
    double timeout = 0.1;
    auto robot_state = state_storage_.getAState();
    robot_state->setToDefaultValues();  // use a deterministic state to initialize
                                        // IK solver

    // fill out the redundant joint values that where provided as an extra
    // argument
    static const std::vector<std::string> joint_names = joint_model_group_->getActiveJointModelNames();
    for (std::size_t i{ 0 }; i < q_redundant.size(); ++i)
    {
        robot_state->setJointPositions(joint_names[i], { q_redundant[i] });
    }

    bool found_ik = robot_state->setFromIK(joint_model_group_, tf, timeout);
    std::vector<JointPositions> sol;
    if (found_ik)
    {
        std::vector<double> joint_values;
        robot_state->copyJointGroupPositions(joint_model_group_, joint_values);
        sol.push_back(joint_values);
    }
    else
    {
        ROS_INFO_STREAM("Failed to find ik solution.");
    }
    return sol;
}

Eigen::MatrixXd Robot::jacobian(const std::vector<double>& q) const
{
    auto robot_state = state_storage_.getAState();
    Eigen::Vector3d reference_point(0.0, 0.0, 0.0);
    robot_state->setJointGroupPositions(joint_model_group_, q);
    return robot_state->getJacobian(joint_model_group_);
}

Eigen::MatrixXd Robot::jacobian(const Eigen::Ref<const Eigen::VectorXd>& q) const
{
    auto robot_state = state_storage_.getAState();
    Eigen::Vector3d reference_point(0.0, 0.0, 0.0);
    robot_state->setJointGroupPositions(joint_model_group_, q);
    return robot_state->getJacobian(joint_model_group_);
}

void Robot::updatePlanningScene()
{
    // I'm not sure yet how this works
    planning_scene_monitor_->requestPlanningSceneState();
    planning_scene_monitor::LockedPlanningSceneRW ps(planning_scene_monitor_);
    ps->getCurrentStateNonConst().update();
    planning_scene_ = ps->diff();
    planning_scene_->decoupleParent();
}

bool Robot::isColliding(const std::vector<double>& joint_pose) const
{
    auto robot_state = state_storage_.getAState();
    robot_state->setJointGroupPositions(joint_model_group_, joint_pose);
    return planning_scene_->isStateColliding(*robot_state);
}

bool Robot::isPathColliding(const JointPositions& q_from, const JointPositions& q_to, int steps) const
{
    for (int step = 0; step < steps; ++step)
    {
        auto q_step = interpolate(q_from, q_to, static_cast<double>(step) / (steps - 1));
        if (isColliding(q_step))
        {
            return true;
        }
    }
    return false;
}

const std::vector<std::string> Robot::getJointNames() const
{
    return joint_model_group_->getActiveJointModelNames();
}

const moveit::core::RobotStatePtr Robot::getMoveItRobotState() const
{
    return state_storage_.getAState();
}

std::vector<double> Robot::randomJointPositions() const
{
    auto robot_state = state_storage_.getAState();
    robot_state->setToRandomPositions();
    std::vector<double> joint_values;
    robot_state->copyJointGroupPositions(joint_model_group_, joint_values);
    return joint_values;
}

void Robot::randomJointPositions(Eigen::VectorXd& out)  // Eigen::Ref is not available to copy to in MoveIt
{
    auto robot_state = state_storage_.getAState();
    robot_state->setToRandomPositions();
    robot_state->copyJointGroupPositions(joint_model_group_, out);
}

void Robot::plot(moveit_visual_tools::MoveItVisualToolsPtr mvt, std::vector<double>& joint_pose,
                 const rviz_visual_tools::colors& color)
{
    auto robot_state = state_storage_.getAState();
    robot_state->setJointGroupPositions(joint_model_group_, joint_pose);
    mvt->publishRobotState(robot_state, color);
    mvt->trigger();
}

/** Simultaniously plot the pose of multiple planning groups.
 *
 * This function can be called on any robot, but uses the robot_state in the particular robot it is called on the
 * visualize the poses in rviz.
 *  **/
void Robot::plotMultipleGroups(moveit_visual_tools::MoveItVisualToolsPtr mvt,
                               const std::map<std::string, JointPositions>& joint_values_map,
                               const rviz_visual_tools::colors& color)
{
    auto robot_state = state_storage_.getAState();
    for (auto group_values : joint_values_map)
    {
        robot_state->setJointGroupPositions(group_values.first, group_values.second);
    }
    mvt->publishRobotState(robot_state, color);
    mvt->trigger();
}

void Robot::animatePath(moveit_visual_tools::MoveItVisualToolsPtr mvt, const std::vector<JointPositions>& path)
{
    for (JointPositions q : path)
    {
        if (isColliding(q))
            plot(mvt, q, rviz_visual_tools::RED);
        else
            plot(mvt, q, rviz_visual_tools::DEFAULT);
        ros::Duration(0.1).sleep();
    }
}

}  // namespace simple_moveit_wrapper
