#include <vector>

namespace simple_moveit_wrapper {

/** \brief Simple interpolation between two vectors, given a fraction s. **/
template <typename T>
inline std::vector<T> interpolate(std::vector<T> q_from, std::vector<T> q_to,
                                  T s) {
  std::vector<T> q(q_from.size());
  for (std::size_t i = 0; i < q_from.size(); ++i) {
    q[i] = (1 - s) * q_from[i] + s * q_to[i];
  }
  return q;
}

}  // namespace simple_moveit_wrapper
