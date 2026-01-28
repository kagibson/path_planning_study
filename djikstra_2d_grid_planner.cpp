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
  const std::shared_ptr<nav2_msgs::msg::Costmap> & costmap_ptr, GridConnectivity grid_connectivity)
{
  costmap_ptr_ = costmap_ptr;
  grid_connectivity_ = grid_connectivity;
}

std::vector<std::vector<double>> Djikstra2DGridPlanner::get_minimum_distance_grid(const Cell & src)
{
  std::vector<std::vector<double>> dists;
  if (costmap_ptr_ == nullptr) {
    std::cout << "Costmap not provided" << std::endl;
    return dists;
  }

  // Initialize distance grid
  for (int i = 0; i < static_cast<int>(costmap_ptr_->metadata.size_y); i++) {
    dists.emplace_back(
      static_cast<double>(costmap_ptr_->metadata.size_x), std::numeric_limits<double>::infinity());
  }

  dists[src.row][src.col] = 0.0;

  std::priority_queue<
    std::pair<double, Cell>, std::vector<std::pair<double, Cell>>,
    std::greater<std::pair<double, Cell>>>
    pq;
  pq.emplace(0.0, Cell(src.row, src.col));

  int rows = static_cast<int>(costmap_ptr_->metadata.size_y);
  int cols = static_cast<int>(costmap_ptr_->metadata.size_x);

  while (!pq.empty()) {
    auto next = pq.top();
    pq.pop();
    Cell cell = next.second;

    double min_cost = next.first;

    if (dists.at(cell.row).at(cell.col) < min_cost) {
      /* Stale entry */
      continue;
    }

    /* Update all neighbor costs */
    std::vector<Cell> neighbor_cells;
    if (grid_connectivity_ == GridConnectivity::FOUR_CONNECTED) {
      neighbor_cells = {
        {cell.row - 1, cell.col},   // Up
        {cell.row, cell.col - 1},   // Left
        {cell.row, cell.col + 1},   // Right
        {cell.row + 1, cell.col}};  // Down
    } else {
      neighbor_cells = {
        {cell.row - 1, cell.col},       // Up
        {cell.row, cell.col - 1},       // Left
        {cell.row + 1, cell.col},       // Down
        {cell.row, cell.col + 1},       // Right
        {cell.row - 1, cell.col - 1},   // Up-Left
        {cell.row + 1, cell.col - 1},   // Down-Left
        {cell.row + 1, cell.col + 1},   // Down-Right
        {cell.row - 1, cell.col + 1}};  // Up-Right
    }

    for (int i = 0; i < static_cast<int>(neighbor_cells.size()); i++) {
      Cell neighbor_cell = neighbor_cells[i];
      // Check bounds
      if (
        neighbor_cell.col < 0 || neighbor_cell.col >= cols || neighbor_cell.row < 0 ||
        neighbor_cell.row >= rows) {
        continue;
      } else if (get_costmap_value(*costmap_ptr_, neighbor_cell) >= kInscribedInflatedObstacle) {
        // Untraversable
        continue;
      }

      double movement;
      if ((i < 4) || (grid_connectivity_ == GridConnectivity::FOUR_CONNECTED)) {
        // Cardinal direction
        movement = costmap_ptr_->metadata.resolution;
      } else {
        // Diagonal direction
        movement = costmap_ptr_->metadata.resolution * std::sqrt(2.0);
      }

      double traversal_cost_multiplier =
        1.0 +
        static_cast<double>(get_costmap_value(*costmap_ptr_, neighbor_cell)) /
          static_cast<double>(kInscribedInflatedObstacle - 1);
      double new_dist =
        min_cost + movement * traversal_cost_multiplier;
      if (new_dist < dists.at(neighbor_cell.row).at(neighbor_cell.col)) {
        dists.at(neighbor_cell.row).at(neighbor_cell.col) = new_dist;
        pq.emplace(new_dist, neighbor_cell);
      }
    }
  }
  return dists;
}

std::vector<Cell> Djikstra2DGridPlanner::shortest_path(Cell src, Cell dest)
{
  std::vector<Cell> path;

  if (costmap_ptr_ == nullptr) {
    std::cout << "Costmap not provided" << std::endl;
    return path;
  }

  // Initialize distance grid
  std::vector<std::vector<std::pair<double, Cell>>> dists;
  for (int i = 0; i < static_cast<int>(costmap_ptr_->metadata.size_y); i++) {
    dists.emplace_back(
      static_cast<int>(costmap_ptr_->metadata.size_x),
      std::make_pair<double, Cell>(INF, Cell(-1, -1)));
  }

  dists[src.row][src.col].first = 0.0;
  dists[src.row][src.col].second = src;

  std::priority_queue<
    std::pair<double, Cell>, std::vector<std::pair<double, Cell>>,
    std::greater<std::pair<double, Cell>>>
    pq;
  pq.emplace(0.0, Cell(src.row, src.col));

  int rows = static_cast<int>(costmap_ptr_->metadata.size_y);
  int cols = static_cast<int>(costmap_ptr_->metadata.size_x);

  while (!pq.empty()) {
    auto next = pq.top();
    pq.pop();
    Cell cell = next.second;

    double min_cost = next.first;

    if (dists.at(cell.row).at(cell.col).first < min_cost) {
      /* Stale entry */
      continue;
    }

    /* Update all neighbor costs */
    std::vector<Cell> neighbor_cells;
    if (grid_connectivity_ == GridConnectivity::FOUR_CONNECTED) {
      neighbor_cells = {
        {cell.row - 1, cell.col},   // Up
        {cell.row, cell.col - 1},   // Left
        {cell.row, cell.col + 1},   // Right
        {cell.row + 1, cell.col}};  // Down
    } else {
      neighbor_cells = {
        {cell.row - 1, cell.col},       // Up
        {cell.row, cell.col - 1},       // Left
        {cell.row + 1, cell.col},       // Down
        {cell.row, cell.col + 1},       // Right
        {cell.row - 1, cell.col - 1},   // Up-Left
        {cell.row + 1, cell.col - 1},   // Down-Left
        {cell.row + 1, cell.col + 1},   // Down-Right
        {cell.row - 1, cell.col + 1}};  // Up-Right
    }

    for (int i = 0; i < static_cast<int>(neighbor_cells.size()); i++) {
      Cell neighbor_cell = neighbor_cells[i];
      // Check bounds
      if (
        neighbor_cell.col < 0 || neighbor_cell.col >= cols || neighbor_cell.row < 0 ||
        neighbor_cell.row >= rows) {
        continue;
      } else if (get_costmap_value(*costmap_ptr_, neighbor_cell) >= kInscribedInflatedObstacle) {
        // Untraversable
        continue;
      }

      double movement;
      if ((i < 4) || (grid_connectivity_ == GridConnectivity::FOUR_CONNECTED)) {
        // Cardinal direction
        movement = costmap_ptr_->metadata.resolution;
      } else {
        // Diagonal direction
        movement = costmap_ptr_->metadata.resolution * std::sqrt(2.0);
      }

      double traversal_cost_multiplier =
        1.0 +
        static_cast<double>(get_costmap_value(*costmap_ptr_, neighbor_cell)) /
          static_cast<double>(kInscribedInflatedObstacle - 1);
      double new_dist =
        min_cost + movement * traversal_cost_multiplier;
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
