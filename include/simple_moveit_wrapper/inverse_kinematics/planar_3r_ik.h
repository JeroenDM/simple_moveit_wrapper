#pragma once

#include <cmath>
#include <vector>

namespace simple_moveit_wrapper
{
namespace ik
{
template <typename T>
bool isInLimits(std::vector<T> v, T lower, T upper)
{
    for (auto value : v)
    {
        if (value < lower || value > upper)
            return false;
    }
    return true;
}

/** Analytical inverse kinematics for a planar robot with 3 revolute joints.
 *
 * I hard coded the joint limits as default parameters for now.
 *
 * **/
template <typename T>
std::vector<std::vector<T>> planar_3r_ik(T x, T y, T angle, std::vector<T> l, T lower_limit = -1.5, T upper_limit = 1.5)
{
    std::vector<std::vector<T>> sol;

    const T TOL{ 1e-12 };

    if ((l[0] + l[1] + l[2]) > std::sqrt(x * x + y * y))
    {
        // coordinates of end point second link
        T pwx = x - l[2] * std::cos(angle);
        T pwy = y - l[2] * std::sin(angle);
        T rws = pwx * pwx + pwy * pwy;  // squared distance to second joint

        std::vector<T> q_up{ 0.0, 0.0, 0.0 };    // elbow up
        std::vector<T> q_down{ 0.0, 0.0, 0.0 };  // elbow down

        if ((l[0] + l[1]) > sqrt(rws))
        {
            T c2 = (rws - l[0] * l[0] - l[1] * l[1]) / (2 * l[0] * l[1]);

            if (c2 < -1 || c2 > 1)
            {
                // point unreachable close to robot base
                // second condition does normally not occur at this point.
            }
            else if (std::abs(c2 - 1) < TOL)
            {
                // first two links are aligned
                q_up[0] = std::atan2(pwy, pwx);
                q_up[1] = 0.0;
                q_up[2] = angle - q_up[0];
                if (isInLimits(q_up, lower_limit, upper_limit))
                    sol.push_back(q_up);
                return sol;
            }
            else if (abs(-c2 - 1) < TOL)
            {
                // first two links aligned and folded
                q_up[0] = std::atan2(pwy, pwx);
                q_up[1] = M_PI;
                q_up[2] = angle - q_up[0] - q_up[1];
                if (isInLimits(q_up, lower_limit, upper_limit))
                    sol.push_back(q_up);
                q_down[0] = std::atan2(pwy, pwx);
                q_down[1] = -M_PI;
                q_down[2] = angle - q_down[0] - q_down[1];
                if (isInLimits(q_down, lower_limit, upper_limit))
                    sol.push_back(q_down);
                return sol;
            }
            else
            {
                // general reachable case
                T s2 = sqrt(1 - c2 * c2);
                q_up[1] = atan2(s2, c2);
                q_down[1] = atan2(-s2, c2);

                T temp = l[0] + l[1] * c2;
                T s1_up = (temp * pwy - l[1] * s2 * pwx) / rws;
                T c1_up = (temp * pwx + l[1] * s2 * pwy) / rws;
                T s1_do = (temp * pwy + l[1] * s2 * pwx) / rws;
                T c1_do = (temp * pwx - l[1] * s2 * pwy) / rws;
                q_up[0] = std::atan2(s1_up, c1_up);
                q_down[0] = std::atan2(s1_do, c1_do);

                q_up[2] = angle - q_up[0] - q_up[1];
                q_down[2] = angle - q_down[0] - q_down[1];

                if (isInLimits(q_up, lower_limit, upper_limit))
                    sol.push_back(q_up);
                if (isInLimits(q_down, lower_limit, upper_limit))
                    sol.push_back(q_down);
                return sol;
            }
        }
    }
    return sol;
}
}  // namespace ik

}  // namespace simple_moveit_wrapper
