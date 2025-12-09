#ifndef A_STAR_2D_GRID_PLANNER_HPP
#define A_STAR_2D_GRID_PLANNER_HPP

#include "a_star_heuristics.hpp"
#include "cell_2d.hpp"
#include "common.hpp"
#include "costmap_2d.hpp"

struct NodeRecord
{
  double f;     // g(n) + h(n)
  long long g;  // Cost from start to cell
  Cell cell;

  bool operator>(const NodeRecord & other) const { return f > other.f; }

  bool operator<(const NodeRecord & other) const { return f < other.f; }
};

class AStar2DPlanner
{
public:
  AStar2DPlanner(
    const std::shared_ptr<Costmap2D> & costmap_ptr, HeuristicFcn heuristic,
    GridConnectivity grid_connectivity = GridConnectivity::FOUR_CONNECTED);

  std::vector<Cell> shortest_path(Cell src, Cell dest);

  void set_heuristic(HeuristicFcn heuristic);

  void set_grid_connectivity(GridConnectivity grid_connectivity);

private:
  std::shared_ptr<Costmap2D> costmap_ptr_;
  HeuristicFcn heuristic_;
  GridConnectivity grid_connectivity_;
};

#endif  // A_STAR_2D_GRID_PLANNER_HPP