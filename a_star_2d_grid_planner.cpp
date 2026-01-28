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
  GridConnectivity grid_connectivity, bool apply_traversal_cost)
: costmap_ptr_(costmap_ptr),
  heuristic_(heuristic),
  grid_connectivity_(grid_connectivity),
  apply_traversal_cost_(apply_traversal_cost)
{
}

AStar2DPlanner::AStar2DPlanner(
  const std::shared_ptr<nav2_msgs::msg::Costmap> & nav2_costmap_ptr, HeuristicFcn heuristic,
  GridConnectivity grid_connectivity, bool apply_traversal_cost) :
  nav2_costmap_ptr_(nav2_costmap_ptr),
  heuristic_(heuristic),
  grid_connectivity_(grid_connectivity),
  apply_traversal_cost_(apply_traversal_cost)
{
}


uint8_t AStar2DPlanner::get_cost(const Cell & coords) const
{
  if (costmap_ptr_ != nullptr) {
    return costmap_ptr_->get_cost(coords);
  } else if (nav2_costmap_ptr_ != nullptr) {
    int index = coords.row * nav2_costmap_ptr_->metadata.size_x + coords.col;
    return nav2_costmap_ptr_->data[index];
  } else {
    throw std::runtime_error("Costmap not provided");
  }
}

int AStar2DPlanner::get_rows() const
{
  if (costmap_ptr_ != nullptr) {
    return costmap_ptr_->get_rows();
  } else if (nav2_costmap_ptr_ != nullptr) {
    return nav2_costmap_ptr_->metadata.size_y;
  } else {
    throw std::runtime_error("Costmap not provided");
  }
}

int AStar2DPlanner::get_cols() const
{
  if (costmap_ptr_ != nullptr) {
    return costmap_ptr_->get_cols();
  } else if (nav2_costmap_ptr_ != nullptr) {
    return nav2_costmap_ptr_->metadata.size_x;
  } else {
    throw std::runtime_error("Costmap not provided");
  }
}

double AStar2DPlanner::get_cell_resolution() const
{
  if (costmap_ptr_ != nullptr) {
    return costmap_ptr_->get_cell_resolution();
  } else if (nav2_costmap_ptr_ != nullptr) {
    return nav2_costmap_ptr_->metadata.resolution;
  } else {
    throw std::runtime_error("Costmap not provided");
  }
}

std::pair<std::vector<Cell>, double> AStar2DPlanner::shortest_path(Cell src, Cell dest)
{
  std::vector<Cell> path;
  double path_cost = 0.0;

  if ((costmap_ptr_ == nullptr) && (nav2_costmap_ptr_ == nullptr)) {
    std::cout << "Costmap not provided" << std::endl;
    return {path, path_cost};
  }

  // Initialize g_costs
  std::vector<std::vector<double>> g_costs;
  for (int i = 0; i < get_rows(); i++) {
    g_costs.emplace_back(get_cols(), INF);
  }
  g_costs[src.row][src.col] = 0;

  std::vector<std::vector<Cell>> parents;
  for (int i = 0; i < get_rows(); i++) {
    parents.emplace_back(get_cols(), Cell(-1, -1));
  }
  parents[src.row][src.col] = src;

  std::priority_queue<NodeRecord, std::vector<NodeRecord>, std::greater<NodeRecord>> open_set;
  open_set.emplace(NodeRecord{0, 0, Cell(src.row, src.col)});

  int rows = get_rows();
  int cols = get_cols();

  while (!open_set.empty()) {
    auto curr = open_set.top();
    open_set.pop();
    Cell cell = curr.cell;

    double g_cost = g_costs.at(cell.row).at(cell.col);

    if (cell == dest) {
      path_cost = g_cost;
      break;
    }

    if (g_cost != curr.g) {
      /* Stale entry */
      continue;
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
      uint8_t neighbor_cell_cost = get_cost(neighbor_cell);
      if (neighbor_cell_cost >= kInscribedInflatedObstacle) {
        continue;
      }
      double costmap_resolution = get_cell_resolution();
      double movement = cell.is_diagonal_to(neighbor_cell)
                          ? M_SQRT2 * costmap_resolution
                          : 1.0 * costmap_resolution;
      if (apply_traversal_cost_) {
        movement *= 1.0 + static_cast<double>(neighbor_cell_cost) / 255.0;
      }
      double new_g = g_cost + movement;

      if (new_g < g_costs.at(neighbor_cell.row).at(neighbor_cell.col)) {
        g_costs.at(neighbor_cell.row).at(neighbor_cell.col) = new_g;
        parents.at(neighbor_cell.row).at(neighbor_cell.col) = cell;
        double new_f = new_g + heuristic_(neighbor_cell, dest) * costmap_resolution;
        NodeRecord nr{new_f, new_g, neighbor_cell};
        open_set.emplace(nr);
      }
    }
  }
  auto curr = dest;
  path.push_back(dest);
  while (curr != src) {
    auto parent = parents.at(curr.row).at(curr.col);
    if (parent == Cell(-1, -1)) {
      path = std::vector<Cell>();
      break;
    }
    path.push_back(parent);
    curr = parent;
  }
  std::reverse(path.begin(), path.end());
  return {path, path_cost};
}

void AStar2DPlanner::set_heuristic(HeuristicFcn heuristic) { heuristic_ = heuristic; }

void AStar2DPlanner::set_grid_connectivity(GridConnectivity grid_connectivity)
{
  grid_connectivity_ = grid_connectivity;
}
