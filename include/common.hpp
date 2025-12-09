#ifndef COMMON_HPP
#define COMMON_HPP

#include <limits>

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

#endif // COMMON_HPP