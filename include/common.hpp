#ifndef COMMON_HPP
#define COMMON_HPP

#include <cmath>
#include <limits>
#include <nav2_msgs/msg/costmap.hpp>
#include "cell_2d.hpp"

const int INF = std::numeric_limits<int>::max();

enum class GridConnectivity
{
  FOUR_CONNECTED,
  EIGHT_CONNECTED
};

inline char int_to_alpha(const int num)
{
  if ((num >= 0) && (num < 26)) {
    return (char)('A' + num);
  }
  return '?';
}

inline double wrap_to_2pi(double angle)
{
  angle = std::fmod(angle, 2.0 * M_PI);
  if (angle < 0.0) angle += 2.0 * M_PI;
  return angle;
}

inline uint8_t get_costmap_value(const nav2_msgs::msg::Costmap & costmap, const Cell & cell)
{
  int index = cell.row * costmap.metadata.size_x + cell.col;
  if (index < 0 || index >= static_cast<int>(costmap.data.size())) {
    return 255;  // Treat out-of-bounds as occupied
  }
  return static_cast<uint8_t>(costmap.data[index]);
}

#endif  // COMMON_HPP