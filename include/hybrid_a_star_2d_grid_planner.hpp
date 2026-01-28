#ifndef HYBRID_A_STAR_2D_GRID_PLANNER_HPP
#define HYBRID_A_STAR_2D_GRID_PLANNER_HPP

#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "nav2_msgs/msg/costmap.hpp"
#include "common.hpp"
#include "pose_2d.hpp"
#include "costmap_2d.hpp"
#include "djikstra_2d_grid_planner.hpp"

struct Motion
{
  double sigma;
  double ds;
};

/**
 * @brief Key for unordered map of g values.
 * 
 */
struct HybridKey
{
  Cell cell;
  int theta_bin;

  bool operator==(const HybridKey & other) const
  {
    return (other.cell.col == cell.col) && (other.cell.row == cell.row) &&
           (other.theta_bin == theta_bin);
  }

  bool operator!=(const HybridKey & other) const { return !(other == *this); }
};

/**
 * @brief Hash function for hashing HybridKey, not ideal but
 * will do for now.
 * 
 */
struct HybridKeyHash
{
  std::size_t operator()(const HybridKey & key) const
  {
    size_t h1 = std::hash<int>{}(key.cell.col);
    size_t h2 = std::hash<int>{}(key.cell.row);
    size_t h3 = std::hash<int>{}(key.theta_bin);
    return h1 ^ (h2 << 1) ^ (h3 << 2);
  }
};

enum direction
{
  LEFT = 0,
  STRAIGHT,
  RIGHT
};

/**
 * @brief Node used for open set priority key.
 * 
 */
struct HybridNode
{
  double f;
  double g;
  HybridKey discrete;
  Pose2D continuous;
  direction dir;
};

class HybridAStar2DGridPlanner
{
public:
  explicit HybridAStar2DGridPlanner(
    std::shared_ptr<nav2_msgs::msg::Costmap> costmap_msg_ptr, double robot_turning_radius,
    double robot_wheelbase, int num_theta_bins, double robot_circular_footprint_radius);

  std::vector<Pose2D> get_shortest_path(
    const Pose2D & source, const Pose2D & goal, cv::Mat & display);

private:
  int theta_to_bin(double theta_angle);

  Cell world_pose_to_map_cell(const Pose2D & world_pose);

  bool goal_check(const Pose2D & pose_to_check, const Pose2D & goal_pose);

  double dubins_distance(const Pose2D & pose_1, const Pose2D & pose_2);

  bool check_circular_footprint(const Pose2D & pose);

  double get_obstacle_aware_heuristic_cost(const Cell & curr);

  double get_nonholonomic_heuristic_cost(const HybridNode & curr, const Pose2D & goal);

  std::shared_ptr<nav2_msgs::msg::Costmap> costmap_msg_ptr_;
  double robot_turning_radius_;
  double robot_wheelbase_;
  double sigma_;
  double ds_;  // arc length to use for calculating motion primitives
  int num_theta_bins_;
  std::array<Motion, 3> motion_primitives_;
  double robot_circular_footprint_radius_;
  std::vector<std::vector<double>> obstacle_aware_heuristic_cache_;
  std::unordered_map<HybridKey, double, HybridKeyHash> nonholonomic_heuristic_cache_;
  Djikstra2DGridPlanner djikstra_2d_grid_planner_;
};

#endif  // HYBRID_A_STAR_2D_GRID_PLANNER_HPP