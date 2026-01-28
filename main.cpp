
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "a_star_2d_grid_planner.hpp"
#include "a_star_heuristics.hpp"
#include "djikstra_2d_grid_planner.hpp"
#include "hybrid_a_star_2d_grid_planner.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

cv::Mat g_base;
cv::Mat g_display;
cv::Point g_src(-1, -1);
double g_src_theta = 0.0;
cv::Point g_dest(-1, -1);
double g_dest_theta = 0.0;
bool g_have_src = false;
bool g_have_src_heading = false;
bool g_have_dest = false;
bool g_have_dest_heading = false;
bool g_need_replan = false;

void mouseCallback(int event, int x, int y, int flags, void * userdata)
{
  if (!g_have_src && (event == cv::EVENT_LBUTTONDOWN)) {
    g_src = cv::Point(x, y);
    g_have_src = true;
    g_need_replan = false;  // not ready yet
    std::cout << "Source selected: (" << g_src.y << ", " << g_src.x << ")\n";
  } else if (!g_have_src_heading && (event == cv::EVENT_LBUTTONUP)) {
    std::cout << "(" << y << "," << x << ")" << std::endl;
    g_src_theta = wrap_to_2pi(std::atan2(g_src.y - y, x - g_src.x));
    std::cout << "Source heading is " << g_src_theta << std::endl;

    cv::arrowedLine(
      g_display, cv::Point(g_src.x, g_src.y),
      cv::Point(g_src.x + cos(g_src_theta) * 25, g_src.y - sin(g_src_theta) * 25),
      cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
    g_have_src_heading = true;

  } else if (!g_have_dest && (event == cv::EVENT_LBUTTONDOWN)) {
    g_dest = cv::Point(x, y);
    g_have_dest = true;

    std::cout << "Destination selected: (" << g_dest.y << ", " << g_dest.x << ")\n";
  } else if (!g_have_dest_heading && (event == cv::EVENT_LBUTTONUP)) {
    std::cout << "(" << y << "," << x << ")" << std::endl;
    g_dest_theta = wrap_to_2pi(std::atan2(g_dest.y - y, x - g_dest.x));
    std::cout << "Dest heading is " << g_dest_theta << std::endl;
    cv::arrowedLine(
      g_display, cv::Point(g_dest.x, g_dest.y),
      cv::Point(g_dest.x + cos(g_dest_theta) * 25, g_dest.y - sin(g_dest_theta) * 25),
      cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
    g_have_dest_heading = true;
    g_need_replan = true;
  }
  if (event == cv::EVENT_RBUTTONDOWN) {
    g_have_src = false;
    g_have_dest = false;
    g_have_src_heading = false;
    g_have_dest_heading = false;
    g_need_replan = false;
    g_display = g_base.clone();
  }

  cv::imshow("Costmap", g_display);
}

long long get_total_path_cost(
  const std::shared_ptr<Costmap2D> & costmap, const std::vector<Cell> & path)
{
  long long total_cost = 0;

  // Skip the first cell (src), because entering it costs nothing
  for (size_t i = 1; i < path.size(); i++) {
    total_cost += costmap->get_cost(path[i]);
  }

  return total_cost;
}

long long get_nav2_total_path_cost(
  const nav2_msgs::msg::Costmap & costmap_msg, const std::vector<Cell> & path)
{
  long long total_cost = 0;
  int width = costmap_msg.metadata.size_x;
  // Skip the first cell (src), because entering it costs nothing
  for (size_t i = 1; i < path.size(); i++) {
    int index = path[i].col + (costmap_msg.metadata.size_y - path[i].row - 1) * width;
    total_cost += costmap_msg.data[index];
  }

  return total_cost;
}

/**
 * @brief Sends a goal to the ComputePathToPose action server and waits for the result.
 * Note: This function is blocking.
 * 
 * @param node 
 * @param path_planning_client
 * @param start 
 * @param goal 
 * @return nav_msgs::msg::Path 
 */
nav_msgs::msg::Path call_compute_path_to_pose_action(
  rclcpp::Node::SharedPtr node,
  std::shared_ptr<rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>> path_planning_client,
  const Pose2D & start, const Pose2D & goal)
{
  if (!path_planning_client->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(node->get_logger(), "Action server not available after waiting");
    return nav_msgs::msg::Path();
  }

  nav2_msgs::action::ComputePathToPose::Goal goal_msg;
  goal_msg.start.header.frame_id = "map";
  goal_msg.start.header.stamp = node->get_clock()->now();
  goal_msg.start.pose.position.x = start.x;
  goal_msg.start.pose.position.y = start.y;
  goal_msg.start.pose.orientation.z = sin(start.theta / 2.0);
  goal_msg.start.pose.orientation.w = cos(start.theta / 2.0);
  goal_msg.goal.header.frame_id = "map";
  goal_msg.goal.header.stamp = node->get_clock()->now();
  goal_msg.goal.pose.position.x = goal.x;
  goal_msg.goal.pose.position.y = goal.y;
  goal_msg.goal.pose.orientation.z = sin(goal.theta / 2.0);
  goal_msg.goal.pose.orientation.w = cos(goal.theta / 2.0);
  goal_msg.use_start = true;

  std::shared_future<
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::ComputePathToPose>::SharedPtr>
    future_goal_handle = path_planning_client->async_send_goal(goal_msg);

  if (
    rclcpp::spin_until_future_complete(node, future_goal_handle) !=
    rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Failed to send goal");
    return nav_msgs::msg::Path();
  }

  auto future_result = path_planning_client->async_get_result(future_goal_handle.get());
  if (
    rclcpp::spin_until_future_complete(node, future_result) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Failed to get result");
    return nav_msgs::msg::Path();
  }
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::ComputePathToPose>::WrappedResult result =
    future_result.get();
  if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_ERROR(node->get_logger(), "Goal was not achieved");
    return nav_msgs::msg::Path();
  }

  return result.result->path;
}

cv::Mat occupancy_grid_to_cv(const nav_msgs::msg::OccupancyGrid & msg)
{
  int w = msg.info.width;
  int h = msg.info.height;

  cv::Mat img(h, w, CV_8UC3);  // Create BGR image directly

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      int i = x + (h - y - 1) * w;  // flip Y
      int8_t v = msg.data[i];

      if (v == 0) {
        // Free space - Black
        img.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 0);
      } else if (v >= 1 && v <= 98) {
        // Gradient from blue (value 1) to red (value 98)
        float t = (v - 1) / 97.0f;  // Normalize to [0, 1]
        uint8_t blue = static_cast<uint8_t>(255 * (1.0f - t));
        uint8_t red = static_cast<uint8_t>(255 * t);
        img.at<cv::Vec3b>(y, x) = cv::Vec3b(blue, 0, red);
      } else if (v == 99) {
        // Obstacle - Cyan (B=255, G=255, R=0)
        img.at<cv::Vec3b>(y, x) = cv::Vec3b(255, 255, 0);
      } else if (v == 100) {
        // Lethal obstacle - Purple (B=255, G=0, R=255)
        img.at<cv::Vec3b>(y, x) = cv::Vec3b(255, 0, 255);
      } else {
        // Unknown or out of range - Gray
        img.at<cv::Vec3b>(y, x) = cv::Vec3b(128, 128, 128);
      }
    }
  }

  return img;
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("path_planning_example");
  auto path_planning_client = rclcpp_action::create_client<nav2_msgs::action::ComputePathToPose>(
    node, "compute_path_to_pose");

  std::shared_ptr<nav2_msgs::msg::Costmap> costmap_msg_ptr;
  std::shared_ptr<rclcpp::Subscription<nav2_msgs::msg::Costmap>> global_costmap_sub =
    node->create_subscription<nav2_msgs::msg::Costmap>(
      "/global_costmap/costmap_raw", rclcpp::QoS(10),
      [&costmap_msg_ptr](const nav2_msgs::msg::Costmap::SharedPtr msg) {
        if (costmap_msg_ptr == nullptr) {
          RCLCPP_INFO(
            rclcpp::get_logger("rclcpp"), "Received global costmap_raw of size %d x %d",
            msg->metadata.size_y, msg->metadata.size_x);
          costmap_msg_ptr = msg;
        }
      });

  std::shared_ptr<nav_msgs::msg::OccupancyGrid> occupancy_grid_msg_ptr;
  std::shared_ptr<rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>> occupancy_grid_sub =
    node->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/global_costmap/costmap", rclcpp::QoS(10),
      [&occupancy_grid_msg_ptr](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        if (occupancy_grid_msg_ptr == nullptr) {
          RCLCPP_INFO(
            rclcpp::get_logger("rclcpp"), "Received occupancy grid of size %d x %d",
            msg->info.height, msg->info.width);
          occupancy_grid_msg_ptr = msg;
        }
      });

  while (rclcpp::ok() && (!costmap_msg_ptr || !occupancy_grid_msg_ptr)) {
    rclcpp::spin_some(node);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    if (!costmap_msg_ptr) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting for global costmap_raw...");
    }
    if (!occupancy_grid_msg_ptr) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting for occupancy grid...");
    }
  }
  if (!rclcpp::ok()) {
    return 0;
  }

  double grid_resolution = costmap_msg_ptr->metadata.resolution;  // Grid size is 0.2m by 0.2m
  int costmap_rows = costmap_msg_ptr->metadata.size_y;
  int costmap_cols = costmap_msg_ptr->metadata.size_x;

  // Use OccupancyGrid for visualization
  cv::Mat costmap_img = occupancy_grid_to_cv(*occupancy_grid_msg_ptr);

  // occupancy_grid_to_cv returns BGR directly, no conversion needed
  g_base = costmap_img.clone();
  g_display = g_base.clone();
  cv::namedWindow("Costmap", cv::WINDOW_NORMAL);
  cv::setMouseCallback("Costmap", mouseCallback);

  //AStar2DPlanner a_star_planner(costmap, zero_distance);
  //Djikstra2DGridPlanner djikstra_planner(costmap_msg_ptr, GridConnectivity::EIGHT_CONNECTED);
  //AStar2DPlanner a_star_planner(costmap_msg_ptr, octile, GridConnectivity::EIGHT_CONNECTED);

  //std::cout << "Costmap value is " << static_cast<int>(get_costmap_value(*costmap_msg_ptr, Cell(699,698))) << std::endl;  
  HybridAStar2DGridPlanner hybrid_astar_planner(costmap_msg_ptr, 4.5, 2.7, 16, 1.5);

  // TODO:  demonstrate different heuristics, weighting of heuristic vs cost, diagonal move costs
  while (rclcpp::ok()) {
    int key = cv::waitKey(10);
    if (key == 27) break;  // ESC also exits nicely

    if (g_need_replan && g_have_src && g_have_dest) {
      Cell g_src_cell(costmap_img.rows - g_src.y, g_src.x);
      Cell g_dest_cell(costmap_img.rows - g_dest.y, g_dest.x);

      // Run planner
      /*
      auto djikstra_start = std::chrono::high_resolution_clock::now();
      //auto path_djikstra = djikstra_planner.shortest_path(g_src_cell, g_dest_cell);
      std::vector<std::vector<double>> distance_grid = djikstra_planner.get_minimum_distance_grid(g_dest_cell);  // Just compute distance grid
      auto djikstra_stop = std::chrono::high_resolution_clock::now();
      auto djikstra_duration =
        std::chrono::duration_cast<std::chrono::microseconds>(djikstra_stop - djikstra_start);
      std::cout << "Djikstra took " << djikstra_duration.count() << " microseconds to plan path."
                << std::endl;
      for (int r = 0; r < costmap_rows; r++) {
        for (int c = 0; c < costmap_cols; c++) {
          if (distance_grid[r][c] < std::numeric_limits<double>::infinity()) {
            float normalized_cost = static_cast<float>(distance_grid[r][c]) / 1000.0f;  // scale for visualization
            normalized_cost = std::min(std::max(normalized_cost, 0.0f), 1.0f);
            uint8_t blue = static_cast<uint8_t>(255 * (1.0f - normalized_cost));
            uint8_t red = static_cast<uint8_t>(255 * normalized_cost);
            g_display.at<cv::Vec3b>(costmap_img.rows - r - 1, c) = cv::Vec3b(blue, 0, red);
          } else {
            // Unreachable
            g_display.at<cv::Vec3b>(costmap_img.rows - r - 1, c) = cv::Vec3b(128, 128, 128);
          }
        }
      }
      a_star_planner.set_grid_connectivity(GridConnectivity::EIGHT_CONNECTED);
      a_star_planner.set_heuristic(zero_distance);
      auto astar_start = std::chrono::high_resolution_clock::now();
      auto path_astar_zero = a_star_planner.shortest_path(g_src_cell, g_dest_cell);
      auto astar_stop = std::chrono::high_resolution_clock::now();
      auto astar_duration =
        std::chrono::duration_cast<std::chrono::microseconds>(astar_stop - astar_start);
      std::cout << "A* with h(n) = 0 took " << astar_duration.count()
                << " microseconds to plan path." << std::endl;
      std::cout << "Total cost is " << get_nav2_total_path_cost(*costmap_msg_ptr, path_astar_zero) << std::endl;

      
      a_star_planner.set_grid_connectivity(GridConnectivity::FOUR_CONNECTED);
      a_star_planner.set_heuristic(manhattan_distance);
      astar_start = std::chrono::high_resolution_clock::now();
      auto path_astar_manhattan = a_star_planner.shortest_path(g_src_cell, g_dest_cell);
      astar_stop = std::chrono::high_resolution_clock::now();
      astar_duration =
        std::chrono::duration_cast<std::chrono::microseconds>(astar_stop - astar_start);
      std::cout << "A* with manhattan heuristic took " << astar_duration.count()
                << " microseconds to plan path." << std::endl;
      std::cout << "Total cost is " << get_nav2_total_path_cost(*costmap_msg_ptr, path_astar_manhattan)
                << std::endl;

      a_star_planner.set_grid_connectivity(GridConnectivity::EIGHT_CONNECTED);
      a_star_planner.set_heuristic(chebychev_distance);
      astar_start = std::chrono::high_resolution_clock::now();
      auto path_astar_chebyshev = a_star_planner.shortest_path(g_src_cell, g_dest_cell);
      astar_stop = std::chrono::high_resolution_clock::now();
      astar_duration =
        std::chrono::duration_cast<std::chrono::microseconds>(astar_stop - astar_start);
      std::cout << "A* with chebyshev heuristic took " << astar_duration.count()
                << " microseconds to plan path." << std::endl;
      std::cout << "Total cost is " << get_nav2_total_path_cost(*costmap_msg_ptr, path_astar_chebyshev)
                << std::endl;
      */
      /*a_star_planner.set_heuristic(euclidean_distance);
      auto astar_start = std::chrono::high_resolution_clock::now();
      auto path_astar_euclidean = a_star_planner.shortest_path(g_src_cell, g_dest_cell);
      auto astar_stop = std::chrono::high_resolution_clock::now();
      auto astar_duration =
        std::chrono::duration_cast<std::chrono::microseconds>(astar_stop - astar_start);
      std::cout << "A* with euclidean heuristic took " << astar_duration.count()
                << " microseconds to plan path." << std::endl;
      std::cout << "Total cost is " << get_nav2_total_path_cost(*costmap_msg_ptr, path_astar_euclidean)
                << std::endl;
      
      a_star_planner.set_heuristic(octile);
      auto astar_start = std::chrono::high_resolution_clock::now();
      auto result = a_star_planner.shortest_path(g_src_cell, g_dest_cell);
      auto path_astar_octile = result.first;
      if (path_astar_octile.empty()) {
        std::cout << "Couldn't find path from " << g_src_cell << " to " << g_dest_cell << std::endl;
      }
      auto astar_stop = std::chrono::high_resolution_clock::now();
      auto astar_duration =
        std::chrono::duration_cast<std::chrono::microseconds>(astar_stop - astar_start);
      std::cout << "A* with octile heuristic took " << astar_duration.count()
                << " microseconds to plan path." << std::endl;
      std::cout << "Path is size: " << path_astar_octile.size() << std::endl;
      std::cout << "Total cost is " << get_nav2_total_path_cost(*costmap_msg_ptr, path_astar_octile) << std::endl;
      
      // Draw path
      
      for (const auto & p : path_djikstra) {
        g_display.at<cv::Vec3b>(costmap_img.rows - p.row, p.col) = {255, 255, 0};
      }
      
      for (const auto & p : path_astar_zero) {
        g_display.at<cv::Vec3b>(costmap_img.rows - p.row, p.col) = {255, 0, 0};
      }
      
      for (const auto & p : path_astar_chebyshev) {
        g_display.at<cv::Vec3b>(costmap_img.rows - p.row, p.col) = {0, 255, 0};
      }
      for (const auto & p : path_astar_manhattan) {
        g_display.at<cv::Vec3b>(costmap_img.rows - p.row, p.col) = {255, 0, 0};
      }
      for (const auto & p : path_astar_euclidean) {
        g_display.at<cv::Vec3b>(costmap_img.rows - p.row, p.col) = {255, 0, 255};
      }
      
      for (const auto & p : path_astar_octile) {
        g_display.at<cv::Vec3b>(costmap_img.rows - p.row, p.col) = {0, 0, 255};
      }
      */
      Pose2D src_pose;
      src_pose.x = (g_src_cell.col * grid_resolution) + costmap_msg_ptr->metadata.origin.position.x;
      src_pose.y = (g_src_cell.row * grid_resolution) + costmap_msg_ptr->metadata.origin.position.y;
      src_pose.theta = g_src_theta;
      std::cout << "Source pose: (" << src_pose.x << ", " << src_pose.y << ", " << src_pose.theta
                << ")\n";

      Pose2D dest_pose;
      dest_pose.x = (g_dest_cell.col * grid_resolution) + costmap_msg_ptr->metadata.origin.position.x;
      dest_pose.y = (g_dest_cell.row * grid_resolution) + costmap_msg_ptr->metadata.origin.position.y;
      dest_pose.theta = g_dest_theta;
      std::cout << "Dest pose: (" << dest_pose.x << ", " << dest_pose.y << ", " << dest_pose.theta
                << ")\n";

      auto start_time = node->get_clock()->now();
      nav_msgs::msg::Path path_from_smac_hybrid =
        call_compute_path_to_pose_action(node, path_planning_client, src_pose, dest_pose);
      auto end_time = node->get_clock()->now();
      auto duration = end_time - start_time;
      RCLCPP_INFO_STREAM(
        node->get_logger(),
        "SMAC Hybrid A* took " << duration.nanoseconds() / 1e6 << " milliseconds to plan path.");
      if (path_from_smac_hybrid.poses.empty()) {
        std::cout << "SMAC Hybrid A* could not find a path from source to destination."
                  << std::endl;
      } else {
        std::cout << "SMAC Hybrid A* found a path of size " << path_from_smac_hybrid.poses.size()
                  << std::endl;
      }
      for (const auto & pose_stamped : path_from_smac_hybrid.poses) {
        int row = static_cast<int>(costmap_rows - std::floor((pose_stamped.pose.position.y - costmap_msg_ptr->metadata.origin.position.y) / grid_resolution));
        int col = static_cast<int>(
          std::floor((pose_stamped.pose.position.x - costmap_msg_ptr->metadata.origin.position.x) / grid_resolution));
        g_display.at<cv::Vec3b>(row, col) = {255, 255, 0};
      }
      cv::imshow("Costmap", g_display);

      start_time = node->get_clock()->now();
      auto path_hybrid_astar =
        hybrid_astar_planner.get_shortest_path(src_pose, dest_pose, g_display);
      end_time = node->get_clock()->now();
      duration = end_time - start_time;
      RCLCPP_INFO_STREAM(
        node->get_logger(),
        "Custom hybrid A* took " << duration.nanoseconds() / 1e6 << " milliseconds to plan path.");
      std::cout << "Path is size: " << path_hybrid_astar.size() << std::endl;
      for (const auto & p : path_hybrid_astar) {
        int row = static_cast<int>(costmap_rows - std::floor((p.y - costmap_msg_ptr->metadata.origin.position.y) / grid_resolution));
        int col = static_cast<int>(std::floor((p.x - costmap_msg_ptr->metadata.origin.position.x) / grid_resolution));
        g_display.at<cv::Vec3b>(row, col) = {255, 0, 255};
      }

      cv::imshow("Costmap", g_display);

      g_need_replan = false;  // Done planning
    }
  }

  rclcpp::shutdown();
}