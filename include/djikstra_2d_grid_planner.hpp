#ifndef DJIKSTRA_2D_GRID_PLANNER_HPP
#define DJIKSTRA_2D_GRID_PLANNER_HPP

#include <memory>

#include "cell_2d.hpp"
#include "common.hpp"
#include "costmap_2d.hpp"
#include "nav2_msgs/msg/costmap.hpp"
/**
 * @brief 2D path planner based on Djikstra's shortest path algorithm.
 * 
 */
class Djikstra2DGridPlanner
{
public:
  /**
 * @brief Construct a new Djikstra 2 D Grid Planner object
 * 
 * @param costmap_ptr 
 * @param grid_connectivity 
 */
  Djikstra2DGridPlanner(
    const std::shared_ptr<nav2_msgs::msg::Costmap> & costmap_ptr,
    GridConnectivity grid_connectivity = GridConnectivity::FOUR_CONNECTED);

  /**
   * @brief Returns a grid of minimum costs from source cell to every other reachable cell
   * in the costmap.
   * @param src Cell in which path should start.
   * @return std::vector<std::vector<double>> 2D grid of minimum costs from source cell.
   */
  std::vector<std::vector<double>> get_minimum_distance_grid(const Cell& src);

  /**
   * @brief Returns the optimal (lowest cost) path between source costmap cell
   * and destination costmap cell.
   * 
   * @param src Cell in which path should start.
   * @param dest Cell in which path should terminate.
   * @return std::vector<Cell> Path from source cell to destination cell.
   */
  std::vector<Cell> shortest_path(Cell src, Cell dest);

private:
  std::shared_ptr<nav2_msgs::msg::Costmap> costmap_ptr_;
  GridConnectivity grid_connectivity_;
};

#endif  // DJIKSTRA_2D_GRID_PLANNER_HPP