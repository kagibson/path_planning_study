#ifndef A_STAR_HEURISTICS_HPP
#define A_STAR_HEURISTICS_HPP

#include <cmath>
#include <limits>
#include <functional>

#include "cell_2d.hpp"

using HeuristicFcn = std::function<double(const Cell &, const Cell &)>;

inline double chebychev_distance(const Cell & p1, const Cell & p2)
{
  return std::max(llabs(p2.col - p1.col), llabs(p2.row - p1.row));
}

inline double manhattan_distance(const Cell & p1, const Cell & p2)
{
  return llabs(p2.col - p1.col) + llabs(p2.row - p1.row);
}

inline double euclidean_distance(const Cell & p1, const Cell & p2)
{
  return std::sqrt(pow((p2.col - p1.col), 2) + pow((p2.row - p1.row), 2));
}

inline double octile(const Cell & p1, const Cell & p2)
{
  double dx = std::abs(p2.col - p1.col);
  double dy = std::abs(p2.row - p1.row);

  double F = std::sqrt(2.0) - 1.0;
  return (dx < dy) ? F * dx + dy : F * dy + dx;
}

inline double zero_distance(const Cell & p1, const Cell & p2)
{
  (void)p1;
  (void)p2;
  return 0.0;
}

#endif  // A_STAR_HEURISTICS_HPP