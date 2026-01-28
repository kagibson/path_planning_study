#ifndef A_STAR_2D_GRID_PLANNER_HPP
#define A_STAR_2D_GRID_PLANNER_HPP

#include "a_star_heuristics.hpp"
#include "cell_2d.hpp"
#include "common.hpp"
#include "costmap_2d.hpp"

struct NodeRecord
{
  double f;  // g(n) + h(n)
  double g;  // Cost from start to cell
  Cell cell;

  bool operator>(const NodeRecord & other) const { return f > other.f; }

  bool operator<(const NodeRecord & other) const { return f < other.f; }
};

class AStar2DPlanner
{
public:
/**
 * @brief Construct a new AStar2DPlanner object
 * 
 * @param costmap_ptr 
 * @param heuristic 
 * @param grid_connectivity 
 */
  AStar2DPlanner(
    const std::shared_ptr<Costmap2D> & costmap_ptr, HeuristicFcn heuristic,
    GridConnectivity grid_connectivity = GridConnectivity::FOUR_CONNECTED, bool apply_traversal_cost = false);

  /**
   * @brief Construct a new AStar2DPlanner object
   * 
   * @param costmap_ptr 
   * @param heuristic 
   * @param grid_connectivity 
   */
  AStar2DPlanner(
    const std::shared_ptr<nav2_msgs::msg::Costmap> & nav2_costmap_ptr, HeuristicFcn heuristic,
    GridConnectivity grid_connectivity = GridConnectivity::FOUR_CONNECTED, bool apply_traversal_cost = false);

  std::pair<std::vector<Cell>, double> shortest_path(Cell src, Cell dest);

  void set_heuristic(HeuristicFcn heuristic);

  void set_grid_connectivity(GridConnectivity grid_connectivity);

  int get_rows() const;

  int get_cols() const;

  uint8_t get_cost(const Cell & coords) const;

  double get_cell_resolution() const;

private:
  std::shared_ptr<nav2_msgs::msg::Costmap> nav2_costmap_ptr_;
  std::shared_ptr<Costmap2D> costmap_ptr_;
  HeuristicFcn heuristic_;
  GridConnectivity grid_connectivity_;
  bool apply_traversal_cost_;
};

#endif  // A_STAR_2D_GRID_PLANNER_HPP