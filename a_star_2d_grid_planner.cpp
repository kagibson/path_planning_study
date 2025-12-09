#include "a_star_2d_grid_planner.hpp"

#include <chrono>
#include <cmath>
#include <ctime>
#include <functional>
#include <iostream>
#include <limits>
#include <list>
#include <memory>
#include <queue>
#include <stdexcept>
#include <tuple>

AStar2DPlanner::AStar2DPlanner(
  const std::shared_ptr<Costmap2D> & costmap_ptr, HeuristicFcn heuristic,
  GridConnectivity grid_connectivity)
{
  costmap_ptr_ = costmap_ptr;
  heuristic_ = heuristic;
  grid_connectivity_ = grid_connectivity;
}

std::vector<Cell> AStar2DPlanner::shortest_path(Cell src, Cell dest)
{
  std::vector<Cell> path;

  if (costmap_ptr_ == nullptr) {
    std::cout << "Costmap not provided" << std::endl;
    return path;
  }

  // Initialize g_costs
  std::vector<std::vector<long long>> g_costs;
  for (int i = 0; i < costmap_ptr_->get_rows(); i++) {
    g_costs.emplace_back(costmap_ptr_->get_cols(), INF);
  }
  g_costs[src.row][src.col] = 0;

  std::vector<std::vector<Cell>> parents;
  for (int i = 0; i < costmap_ptr_->get_rows(); i++) {
    parents.emplace_back(costmap_ptr_->get_cols(), Cell(-1, -1));
  }  // TODO: I think that this and g_costs could be a different data structure as we won't be visiting every cell
  parents[src.row][src.col] = src;

  std::priority_queue<NodeRecord, std::vector<NodeRecord>, std::greater<NodeRecord>> open_set;
  open_set.emplace(NodeRecord{0, 0, Cell(src.row, src.col)});

  int rows = costmap_ptr_->get_rows();
  int cols = costmap_ptr_->get_cols();

  while (!open_set.empty()) {
    auto curr = open_set.top();
    open_set.pop();
    Cell cell = curr.cell;

    long long g_cost = g_costs.at(cell.row).at(cell.col);

    if (cell == dest) {
      std::cout << "Found goal!" << std::endl;
      break;
    }

    if (g_cost != curr.g) {
      /* Stale entry */
      continue;
      ;
    }

    /* Update all neighbor costs */
    std::vector<Cell> neighbor_cells;
    if (grid_connectivity_ == GridConnectivity::FOUR_CONNECTED) {
      neighbor_cells = {
        {cell.row - 1, cell.col},
        {cell.row, cell.col - 1},
        {cell.row, cell.col + 1},
        {cell.row + 1, cell.col}};
    } else {
      neighbor_cells = {{cell.row - 1, cell.col - 1}, {cell.row - 1, cell.col},
                        {cell.row - 1, cell.col + 1}, {cell.row, cell.col - 1},
                        {cell.row, cell.col + 1},     {cell.row + 1, cell.col - 1},
                        {cell.row + 1, cell.col},     {cell.row + 1, cell.col + 1}};
    }

    for (const auto & neighbor_cell : neighbor_cells) {
      if (
        neighbor_cell.col < 0 || neighbor_cell.col >= cols || neighbor_cell.row < 0 ||
        neighbor_cell.row >= rows) {
        continue;
      }

      long long new_g = g_cost + costmap_ptr_->get_cost(neighbor_cell);
      if (new_g < g_costs.at(neighbor_cell.row).at(neighbor_cell.col)) {
        g_costs.at(neighbor_cell.row).at(neighbor_cell.col) = new_g;
        parents.at(neighbor_cell.row).at(neighbor_cell.col) = cell;
        double new_f = new_g + heuristic_(neighbor_cell, dest);
        NodeRecord nr{new_f, new_g, neighbor_cell};
        open_set.emplace(nr);
      }
    }
  }
  auto curr = dest;
  path.push_back(dest);
  while (curr != src) {
    auto parent = parents.at(curr.row).at(curr.col);
    path.push_back(parent);
    curr = parent;
  }
  std::reverse(path.begin(), path.end());
  return path;
}

void AStar2DPlanner::set_heuristic(HeuristicFcn heuristic) { heuristic_ = heuristic; }

void AStar2DPlanner::set_grid_connectivity(GridConnectivity grid_connectivity)
{
  grid_connectivity_ = grid_connectivity;
}

