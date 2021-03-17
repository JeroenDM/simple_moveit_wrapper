#pragma once

#include <simple_moveit_wrapper/robot.h>

#include <opw_kinematics/opw_parameters.h>

namespace simple_moveit_wrapper
{
class IndustrialRobot : public Robot
{
    opw_kinematics::Parameters<double> opw_parameters_;
    std::string group_name_;
    Transform tool0_to_tcp_;
    Transform tool0_to_tcp_inverse_;
    const std::size_t num_base_joints_{ 6 };
    std::size_t num_red_joints_{ 0 };
    bool is_redundant_{ false };

    void messyHardCodedStuff();

    /** Copied from MoveIt source code, I should really make all this more generic maybe...
     * http://docs.ros.org/en/melodic/api/moveit_core/html/kinematics__base_8h_source.html#l00618
     * **/
    template <typename T>
    inline bool lookupParam(const std::string& param, T& val, const T& default_val) const
    {
        ros::NodeHandle pnh("~");
        if (pnh.hasParam(group_name_ + "/" + param))
        {
            val = pnh.param(group_name_ + "/" + param, default_val);
            return true;
        }

        if (pnh.hasParam(param))
        {
            val = pnh.param(param, default_val);
            return true;
        }

        ros::NodeHandle nh;
        if (nh.hasParam("robot_description_kinematics/" + group_name_ + "/" + param))
        {
            val = nh.param("robot_description_kinematics/" + group_name_ + "/" + param, default_val);
            return true;
        }

        if (nh.hasParam("robot_description_kinematics/" + param))
        {
            val = nh.param("robot_description_kinematics/" + param, default_val);
            return true;
        }

        val = default_val;

        return false;
    }

  public:
    IndustrialRobot(const std::string& planning_group = "manipulator", const std::string& tcp_frame = "tool0");
    std::vector<JointPositions> ik(const Transform& tf) const override;
    std::vector<JointPositions> ik(const Transform& tf, const std::vector<double>& q_redundant) const override;
    bool setOPWParameters();
};

}  // namespace simple_moveit_wrapper
