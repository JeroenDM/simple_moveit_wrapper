#pragma once

#include <string>
#include <vector>

#include <Eigen/Dense>

#include <moveit/macros/class_forward.h>

#include <simple_moveit_wrapper/threadsafe_state_storage.h>

// A bunch of forward declarations that should decrease compile time for users
namespace moveit
{
namespace core
{
MOVEIT_CLASS_FORWARD(RobotModel);
MOVEIT_CLASS_FORWARD(RobotState);
MOVEIT_CLASS_FORWARD(JointModelGroup);
}  // namespace core
}  // namespace moveit

namespace robot_model_loader
{
MOVEIT_CLASS_FORWARD(RobotModelLoader);
}

namespace planning_scene
{
MOVEIT_CLASS_FORWARD(PlanningScene);
}

namespace planning_scene_monitor
{
MOVEIT_CLASS_FORWARD(PlanningSceneMonitor);
}

namespace moveit_visual_tools
{
MOVEIT_CLASS_FORWARD(MoveItVisualTools);
}

// Not sure how to forward declare the default enum value `rviz_visual_tools::DEFAULT`
#include <rviz_visual_tools/rviz_visual_tools.h>

// Start of the useful code
namespace simple_moveit_wrapper
{
typedef Eigen::Isometry3d Transform;
typedef std::vector<double> JointPositions;

/** A simplified representation of joint limits, compared to MoveIt's variable bounds. **/
struct Bounds
{
    double lower;
    double upper;
    double range() const
    {
        return upper - lower;
    }
};

/** \brief Wrapper around a MoveIt robot model end planning scene to simplify some basic operations.**/
class Robot
{
  protected:
    moveit::core::RobotModelPtr robot_model_;
    const moveit::core::JointModelGroup* joint_model_group_;

    robot_model_loader::RobotModelLoaderPtr robot_model_loader_;

    planning_scene::PlanningScenePtr planning_scene_;
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

    bool check_collisions_ = true;

    const std::string tcp_frame_;
    const std::string planning_group_;
    std::size_t num_dof_;
    std::size_t num_red_dof_;

    std::vector<Bounds> joint_position_limits_;
    std::vector<Bounds> joint_velocity_limits_;

    mutable TSStateStorage state_storage_;

  public:
    Robot(const std::string& planning_group = "manipulator", const std::string& tcp_frame = "tool0");
    virtual ~Robot() = default;

    Transform fk(const JointPositions& q) const;
    Transform fk(const Eigen::Ref<const Eigen::VectorXd>& q) const;

    Transform fk(const JointPositions& q, const std::string& frame) const;
    Transform fk(const Eigen::Ref<const Eigen::VectorXd>& q, const std::string& frame) const;

    /** \brief Inverse kinematics
     *
     * This function should be overriden with a robot specific implementation of analytical inverse kinematics.
     * The default class uses MoveIts default inverse kinematics plugin (which could have analytical inverse kinematics.
     *
     * (TODO) Return multiple solutions from MoveIts IK plugin.
     *
     * **/
    virtual std::vector<JointPositions> ik(const Transform& tf) const;

    /** \brief Redundant inverse kinematics
     *
     * The robot's joints are divided in two groups. The first k joints are called "redundant joints",
     * and the last 3 / 6 joints are called "base joints".
     * Given as input positions for the redundant joints, an analytical inverse kinematics solver can calculate
     * the base joints for the given end-effector pose.
     *
     * **/
    virtual std::vector<JointPositions> ik(const Transform& tf, const std::vector<double>& q_redundant) const;

    Eigen::MatrixXd jacobian(const JointPositions& q) const;

    const Transform& getLinkFixedRelativeTransform(const std::string& name) const;

    void updatePlanningScene();

    bool isColliding(const JointPositions& joint_pose) const;

    bool isPathColliding(const JointPositions& q_from, const JointPositions& q_to, int steps) const;

    std::size_t getNumDof()
    {
        return num_dof_;
    }

    std::size_t getNumRedDof()
    {
        return num_red_dof_;
    }

    const std::vector<Bounds>& getJointPositionLimits()
    {
        return joint_position_limits_;
    }

    const std::vector<Bounds>& getJointVelocityLimits()
    {
        return joint_velocity_limits_;
    }

    void plot(moveit_visual_tools::MoveItVisualToolsPtr mvt, JointPositions& joint_pose,
              const rviz_visual_tools::colors& color = rviz_visual_tools::DEFAULT);

    /** Show the different robot poses along the path with a short delay in between.
     *
     * When a configuration is colliding, it is shown red. Otherwise it is shown green.
     * **/
    void animatePath(moveit_visual_tools::MoveItVisualToolsPtr mvt, const std::vector<JointPositions>& path);
};

}  // namespace simple_moveit_wrapper
