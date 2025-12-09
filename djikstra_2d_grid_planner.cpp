#include "djikstra_2d_grid_planner.hpp"

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

Djikstra2DGridPlanner::Djikstra2DGridPlanner(
  const std::shared_ptr<Costmap2D> & costmap_ptr,
  GridConnectivity grid_connectivity)
{
  costmap_ptr_ = costmap_ptr;
  grid_connectivity_ = grid_connectivity;
}

std::vector<Cell> Djikstra2DGridPlanner::shortest_path(Cell src, Cell dest)
{
  std::vector<Cell> path;

  if (costmap_ptr_ == nullptr) {
    std::cout << "Costmap not provided" << std::endl;
    return path;
  }

  // Initialize distance grid
  std::vector<std::vector<std::pair<long long, Cell>>> dists;
  for (int i = 0; i < costmap_ptr_->get_rows(); i++) {
    dists.emplace_back(
      costmap_ptr_->get_cols(), std::make_pair<long long, Cell>(INF, Cell(-1, -1)));
  }

  dists[src.row][src.col].first = 0;
  dists[src.row][src.col].second = src;

  std::priority_queue<
    std::pair<long long, Cell>, std::vector<std::pair<long long, Cell>>,
    std::greater<std::pair<long long, Cell>>>
    pq;
  pq.emplace(0, Cell(src.row, src.col));

  int rows = costmap_ptr_->get_rows();
  int cols = costmap_ptr_->get_cols();

  while (!pq.empty()) {
    auto next = pq.top();
    pq.pop();
    Cell cell = next.second;

    int min_cost = next.first;

    if (cell == dest) {
      break;
    }

    if (dists.at(cell.row).at(cell.col).first < min_cost) {
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
      // Check bounds
      if (
        neighbor_cell.col < 0 || neighbor_cell.col >= cols || neighbor_cell.row < 0 ||
        neighbor_cell.row >= rows) {
        continue;
      }

      long long new_dist = min_cost + costmap_ptr_->get_cost(neighbor_cell);
      if (new_dist < dists.at(neighbor_cell.row).at(neighbor_cell.col).first) {
        dists.at(neighbor_cell.row).at(neighbor_cell.col).first = new_dist;
        dists.at(neighbor_cell.row).at(neighbor_cell.col).second = cell;
        pq.emplace(new_dist, neighbor_cell);
      }
    }
  }

  auto curr = dest;
  path.push_back(dest);
  while (curr != src) {
    auto parent = dists.at(curr.row).at(curr.col).second;
    path.push_back(parent);
    curr = parent;
  }
  std::reverse(path.begin(), path.end());
  return path;
}
