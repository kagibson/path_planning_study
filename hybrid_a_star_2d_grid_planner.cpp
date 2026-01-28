#include "hybrid_a_star_2d_grid_planner.hpp"

#include <cmath>
#include <functional>
#include <optional>
#include <queue>
#include <unordered_map>
#include <unordered_set>

extern "C" {
#include "dubins.h"
}
#include "common.hpp"

/**
 * @brief Greater than function for comparing HybridNode structs.
 * 
 */
struct HybridNodeGreaterFcn
{
  bool operator()(const HybridNode & first, const HybridNode & second)
  {
    return first.f > second.f;
  };
};

/**
 * @brief Computes the Eucliden distance between two 2D poses.
 * 
 * @param pose_1 - first pose.
 * @param pose_2 - second pose.
 * @return double - Euclidean distance between poses.
 */
double euclidean_distance(const Pose2D & pose_1, const Pose2D & pose_2)
{
  return hypot(pose_2.y - pose_1.y, pose_2.x - pose_1.x);
}

/**
 * @brief Computes the distance of the Dubin's path that connects two poses.
 * Finds the distance for each Dubin's type and returns the minimum.
 * 
 * @param pose_1 
 * @param pose_2 
 * @return double 
 */
double HybridAStar2DGridPlanner::dubins_distance(const Pose2D & pose_1, const Pose2D & pose_2)
{
  double dist;
  DubinsPath dubins_between_poses;
  double q0[3] = {pose_1.x, pose_1.y, pose_1.theta};
  double q1[3] = {pose_2.x, pose_2.y, pose_2.theta};
  int result = dubins_shortest_path(&dubins_between_poses, q0, q1, robot_turning_radius_);

  if (result == 0) {
    dist = dubins_path_length(&dubins_between_poses);
  } else {
    dist = euclidean_distance(pose_1, pose_2);
  }

  return dist;
}

HybridAStar2DGridPlanner::HybridAStar2DGridPlanner(
  std::shared_ptr<nav2_msgs::msg::Costmap> costmap_msg_ptr, double robot_turning_radius,
  double robot_wheelbase, int num_theta_bins, double robot_circular_footprint_radius)
: costmap_msg_ptr_{costmap_msg_ptr},
  num_theta_bins_{num_theta_bins},
  robot_turning_radius_{robot_turning_radius},
  robot_wheelbase_{robot_wheelbase},
  sigma_{std::atan(robot_wheelbase / robot_turning_radius)},
  ds_{5.0 * costmap_msg_ptr_->metadata.resolution},
  motion_primitives_{{{-sigma_, ds_}, {0.0, ds_}, {sigma_, ds_}}},
  robot_circular_footprint_radius_{robot_circular_footprint_radius},
  djikstra_2d_grid_planner_{costmap_msg_ptr, GridConnectivity::EIGHT_CONNECTED}
{
  std::cout << "Number of theta bins: " << num_theta_bins_ << std::endl;
  std::cout << "Sigma: " << sigma_ << std::endl;
}

int HybridAStar2DGridPlanner::theta_to_bin(double theta_angle)
{
  return static_cast<int>(wrap_to_2pi(theta_angle) / (2 * M_PI) * num_theta_bins_) %
         num_theta_bins_;
}

Cell HybridAStar2DGridPlanner::world_pose_to_map_cell(const Pose2D & world_pose)
{
  int row = std::floor(
    (world_pose.y - costmap_msg_ptr_->metadata.origin.position.y) /
    costmap_msg_ptr_->metadata.resolution);
  int col = std::floor(
    (world_pose.x - costmap_msg_ptr_->metadata.origin.position.x) /
    costmap_msg_ptr_->metadata.resolution);
  return Cell({row, col});
}

inline double shortest_angle(double a, double b)
{
  double d = a - b;
  d = std::fmod(d + M_PI, 2.0 * M_PI);
  if (d < 0) d += 2.0 * M_PI;
  return d - M_PI;
}
bool HybridAStar2DGridPlanner::goal_check(const Pose2D & p, const Pose2D & goal)
{
  double position_tolerance = 2.0 * costmap_msg_ptr_->metadata.resolution;
  double heading_tolerance = 4 * (2 * M_PI / num_theta_bins_);

  return hypot(p.x - goal.x, p.y - goal.y) < position_tolerance &&
         abs(shortest_angle(p.theta, goal.theta)) < heading_tolerance;
}

bool HybridAStar2DGridPlanner::check_circular_footprint(const Pose2D & pose)
{
  Cell center_cell = world_pose_to_map_cell(pose);
  int radius_cells =
    std::ceil(robot_circular_footprint_radius_ / costmap_msg_ptr_->metadata.resolution);

  for (int dr = -radius_cells; dr <= radius_cells; ++dr) {
    for (int dc = -radius_cells; dc <= radius_cells; ++dc) {
      // Skip cells outside circular footprint
      if (dr * dr + dc * dc > radius_cells * radius_cells) {
        continue;
      }

      Cell check_cell(center_cell.row + dr, center_cell.col + dc);

      // Bounds check
      if (
        check_cell.row >= static_cast<int>(costmap_msg_ptr_->metadata.size_y) ||
        check_cell.col >= static_cast<int>(costmap_msg_ptr_->metadata.size_x) ||
        check_cell.row < 0 || check_cell.col < 0) {
        return false;  // Collision with boundary
      }

      // Collision check
      if (get_costmap_value(*costmap_msg_ptr_, check_cell) > kInscribedInflatedObstacle) {
        return false;  // Collision
      }
    }
  }

  return true;  // No collision
}

double HybridAStar2DGridPlanner::get_obstacle_aware_heuristic_cost(const Cell & curr)
{
  double h = 0.0;
  auto djikstra_cost = obstacle_aware_heuristic_cache_[curr.row][curr.col];
  if (djikstra_cost != INF)
  {
    h = static_cast<double>(djikstra_cost);
  }
  return h;
}

double HybridAStar2DGridPlanner::get_nonholonomic_heuristic_cost(
  const HybridNode & curr, const Pose2D & goal)
{
  double h = 0.0;
  auto it = nonholonomic_heuristic_cache_.find(curr.discrete);
  if (it != nonholonomic_heuristic_cache_.end()) {
    h = it->second;
  } else {
    h = dubins_distance(curr.continuous, goal);
    nonholonomic_heuristic_cache_[curr.discrete] = h;
  }
  return h;
}

std::vector<Pose2D> HybridAStar2DGridPlanner::get_shortest_path(
  const Pose2D & source, const Pose2D & goal, cv::Mat & display)
{
  std::vector<Pose2D> path;
  int theta_bin = theta_to_bin(source.theta);
  Cell source_cell = world_pose_to_map_cell(source);

  std::unordered_map<HybridKey, double, HybridKeyHash> g_dists = {{{source_cell, theta_bin}, 0.0}};
  std::unordered_map<HybridKey, HybridKey, HybridKeyHash> parents;
  std::unordered_map<HybridKey, Pose2D, HybridKeyHash> parent_poses;
  HybridKey key{source_cell, theta_bin};

  std::priority_queue<HybridNode, std::vector<HybridNode>, HybridNodeGreaterFcn> open_set;
  std::unordered_set<HybridKey, HybridKeyHash> closed_set;

  HybridNode source_node;
  source_node.discrete.cell = source_cell;
  source_node.discrete.theta_bin = theta_bin;
  source_node.continuous = source;
  source_node.f = 0;
  source_node.g = 0;
  open_set.emplace(source_node);

  Cell goal_cell = world_pose_to_map_cell(goal);

  obstacle_aware_heuristic_cache_.clear();
  obstacle_aware_heuristic_cache_ = djikstra_2d_grid_planner_.get_minimum_distance_grid(goal_cell);

  HybridNode curr;
  int iterations = 0;
  while (!open_set.empty()) {
    curr = open_set.top();
    open_set.pop();

    iterations++;

    if (closed_set.count(curr.discrete)) {
      continue;
    }
    closed_set.insert(curr.discrete);

    if ((curr.discrete != source_node.discrete) && goal_check(curr.continuous, goal)) {
      std::cout << "Found goal!" << std::endl;
      break;
    }

    for (const auto & motion : motion_primitives_) {
      double kappa = tan(motion.sigma) / robot_wheelbase_;
      double dtheta = kappa * motion.ds;

      // Sample points along the arc
      bool collision_free = true;
      double accumulated_terrain_cost = 0.0;
      int num_checks = 10;

      for (int i = 0; i <= num_checks; ++i) {
        double s = (i / (double)num_checks) * motion.ds;

        // Compute pose at arc distance s
        Pose2D check_pose;
        if (abs(kappa) < 1e-6) {
          // Straight line
          check_pose.x = curr.continuous.x + s * cos(curr.continuous.theta);
          check_pose.y = curr.continuous.y + s * sin(curr.continuous.theta);
          check_pose.theta = curr.continuous.theta;
        } else {
          // Circular arc
          double radius = 1.0 / abs(kappa);
          double theta_traveled = s / radius * (kappa > 0 ? 1 : -1);
          double sign = (kappa > 0 ? 1 : -1);

          double cx = curr.continuous.x - radius * sin(curr.continuous.theta) * sign;
          double cy = curr.continuous.y + radius * cos(curr.continuous.theta) * sign;

          check_pose.theta = curr.continuous.theta + theta_traveled;
          check_pose.x = cx + radius * sin(check_pose.theta) * sign;
          check_pose.y = cy - radius * cos(check_pose.theta) * sign;
        }

        Cell check_cell = world_pose_to_map_cell(check_pose);

        // Bounds check
        if (
          check_cell.row >= static_cast<int>(costmap_msg_ptr_->metadata.size_y) ||
          check_cell.col >= static_cast<int>(costmap_msg_ptr_->metadata.size_x) ||
          check_cell.row < 0 || check_cell.col < 0) {
          collision_free = false;
          break;
        }

        uint8_t cell_cost = get_costmap_value(*costmap_msg_ptr_, check_cell);

        // Collision check
        if (!check_circular_footprint(check_pose)) {
          collision_free = false;
          break;
        }

        // Accumulate terrain cost (average over all sampled points)
        accumulated_terrain_cost += static_cast<double>(cell_cost) / 255.0;
      }

      if (!collision_free) {
        continue;
      }

      // Compute next state
      HybridNode next;
      double mid_theta = curr.continuous.theta + dtheta / 2.0;
      next.continuous.x = curr.continuous.x + motion.ds * cos(mid_theta);
      next.continuous.y = curr.continuous.y + motion.ds * sin(mid_theta);
      next.continuous.theta = wrap_to_2pi(curr.continuous.theta + dtheta);
      next.discrete.cell = world_pose_to_map_cell(next.continuous);
      next.discrete.theta_bin = theta_to_bin(next.continuous.theta);

      // Skip if in closed set
      if (closed_set.count(next.discrete)) {
        continue;
      }

      // Cost calculation using accumulated terrain cost
      double base_cost = motion.ds;
      double steering_penalty = (motion.sigma != 0.0) ? 1.05 : 1.0;

      // Average terrain cost over all sampled points
      double avg_terrain_cost = accumulated_terrain_cost / (num_checks + 1);
      double terrain_penalty = 1.0 + avg_terrain_cost;

      double delta_g = base_cost * steering_penalty * terrain_penalty;
      next.g = curr.g + delta_g;
      double obstacle_aware_h = get_obstacle_aware_heuristic_cost(next.discrete.cell);
      double nonholonomic_h = get_nonholonomic_heuristic_cost(next, goal);
      //std::cout << "Obstacle-aware h: " << obstacle_aware_h << ", Non-holonomic h: " << nonholonomic_h << std::endl;
      double h = std::max(obstacle_aware_h, nonholonomic_h);

      /*if (iterations % 100 == 0) {
        std::cout << "Obstacle h: " << obstacle_aware_h << ", Dubins h: " << nonholonomic_h
                  << ", max: " << std::max(obstacle_aware_h, nonholonomic_h) << ", g: " << next.g
                  << std::endl;
      }*/

      next.f = next.g + h;

      auto g_dists_it = g_dists.find(next.discrete);
      if ((g_dists_it == g_dists.end()) || (next.g < g_dists_it->second)) {
        g_dists[next.discrete] = next.g;
        open_set.emplace(next);
        parents[next.discrete] = curr.discrete;
        parent_poses[next.discrete] = curr.continuous;
      }
    }
  }

  std::cout << "Planned in " << iterations << " iterations." << std::endl;
  std::cout << "Final node is (" << curr.discrete.cell.row << "," << curr.discrete.cell.col << ","
            << curr.discrete.theta_bin << ") with cost "
            << static_cast<unsigned int>(get_costmap_value(*costmap_msg_ptr_, curr.discrete.cell))
            << std::endl;
  if (!parents.count(curr.discrete)) {
    std::cerr << "Goal node has no parent!" << std::endl;
  }

  HybridKey parent = curr.discrete;
  while (parent != source_node.discrete) {
    auto parent_pose = parent_poses.at(parent);
    path.push_back(parent_pose);
    parent = parents.at(parent);
  }
  path.push_back(source);

  std::reverse(path.begin(), path.end());

  return path;
}