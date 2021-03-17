#include <gtest/gtest.h>
#include <ros/ros.h>

#include <iostream>

#include <simple_moveit_wrapper/industrial_robot.h>

namespace testing
{
const double TOLERANCE = 1e-6;  // absolute tolerance for EXPECT_NEAR checks

template <typename T>
using Transform = Eigen::Transform<T, 3, Eigen::Isometry>;

/** \brief Compare every element of two transforms.
 */
template <typename T>
void comparePoses(const Transform<T>& Ta, const Transform<T>& Tb)
{
    using Matrix = Eigen::Matrix<T, 3, 3>;
    using Vector = Eigen::Matrix<T, 3, 1>;

    Matrix Ra = Ta.rotation(), Rb = Tb.rotation();
    for (int i = 0; i < Ra.rows(); ++i)
    {
        for (int j = 0; j < Ra.cols(); ++j)
        {
            EXPECT_NEAR(Ra(i, j), Rb(i, j), TOLERANCE);
        }
    }

    Vector pa = Ta.translation(), pb = Tb.translation();
    EXPECT_NEAR(pa[0], pb[0], TOLERANCE);
    EXPECT_NEAR(pa[1], pb[1], TOLERANCE);
    EXPECT_NEAR(pa[2], pb[2], TOLERANCE);
}
}  // namespace testing

TEST(Fanuc, Initialize)
{
    // Read the planning group name and tip link for the robot being tested
    ros::NodeHandle nh;
    std::string group_name, tip_link;
    ASSERT_TRUE(nh.getParam("group", group_name));
    ASSERT_TRUE(nh.getParam("tip_link", tip_link));

    simple_moveit_wrapper::IndustrialRobot robot(group_name, tip_link);

    ASSERT_TRUE(true);
}

TEST(Fanuc, ForwardAndInverseKinematics)
{
    // Read the planning group name and tip link for the robot being tested
    ros::NodeHandle nh;
    std::string group_name, tip_link;
    ASSERT_TRUE(nh.getParam("group", group_name));
    ASSERT_TRUE(nh.getParam("tip_link", tip_link));

    simple_moveit_wrapper::IndustrialRobot robot(group_name, tip_link);

    simple_moveit_wrapper::JointPositions home{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    auto ee_transform = robot.fk(home);
    auto ik_solutions = robot.ik(ee_transform);


    EXPECT_EQ(ik_solutions.size(), 8);
    for (auto& q : ik_solutions)
    {
        testing::comparePoses(ee_transform, robot.fk(q));
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "moveit_opw_kinematics_test_fanuc");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
