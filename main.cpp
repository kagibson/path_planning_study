
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "a_star_2d_grid_planner.hpp"
#include "djikstra_2d_grid_planner.hpp"
#include "a_star_heuristics.hpp"

cv::Mat g_base;
cv::Mat g_display;
Cell g_src(-1, -1);
Cell g_dest(-1, -1);
bool g_have_src = false;
bool g_have_dest = false;
bool g_need_replan = false;

void mouseCallback(int event, int x, int y, int flags, void * userdata)
{
  if (!g_have_src && (event == cv::EVENT_LBUTTONDOWN)) {
    g_src = Cell(y, x);
    g_have_src = true;
    g_need_replan = false;  // not ready yet
    std::cout << "Source selected: (" << g_src.row << ", " << g_src.col << ")\n";

    // Draw a small green circle at src
    cv::circle(g_display, cv::Point(x, y), 3, cv::Scalar(0, 255, 0), -1);
  } else if (!g_have_dest & (event == cv::EVENT_LBUTTONDOWN)) {
    g_dest = Cell(y, x);
    g_have_dest = true;
    g_need_replan = true;  // not ready yet

    std::cout << "Destination selected: (" << g_dest.row << ", " << g_dest.col << ")\n";

    // Draw a small red circle at dest
    cv::circle(g_display, cv::Point(x, y), 3, cv::Scalar(0, 0, 255), -1);
  } else if (event == cv::EVENT_RBUTTONDOWN) {
    g_have_src = false;
    g_have_dest = false;
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

int main(int argc, char ** argv)
{
  if (argc < 2) {
    std::cout << "./djikstra_2d_grid_planner <image_path>" << std::endl;
    return -1;
  }

  std::string image_path = argv[1];
  cv::Mat image = cv::imread(image_path, cv::IMREAD_GRAYSCALE);

  if (image.empty()) {
    std::cerr << "Error: Could not open or find the image at " << image_path << std::endl;
    return -1;
  }

  std::vector<uint8_t> data(image.rows * image.cols);

  for (int y = 0; y < image.rows; y++) {
    for (int x = 0; x < image.cols; x++) {
      data[y * image.cols + x] = image.at<uint8_t>(y, x);
    }
  }

  auto costmap = std::make_shared<Costmap2D>(image.rows, image.cols, data);
  int rows = costmap->get_rows();
  int cols = costmap->get_cols();

  cv::Mat costmap_img(rows, cols, CV_8UC1);
  for (int y = 0; y < rows; y++) {
    for (int x = 0; x < cols; x++) {
      uint8_t cost = costmap->get_cost({y, x});
      costmap_img.at<uint8_t>(y, x) = 255 - cost;
    }
  }

  cv::Mat costmap_color;
  cv::cvtColor(costmap_img, costmap_color, cv::COLOR_GRAY2BGR);
  g_base = costmap_color.clone();
  g_display = g_base.clone();
  cv::namedWindow("Costmap", cv::WINDOW_NORMAL);
  cv::setMouseCallback("Costmap", mouseCallback);

  AStar2DPlanner a_star_planner(costmap, zero_distance);
  Djikstra2DGridPlanner djikstra_planner(costmap, GridConnectivity::EIGHT_CONNECTED);

  // TODO:  demonstrate different heuristics, weighting of heuristic vs cost, diagonal move costs
  while (true) {
    int key = cv::waitKey(10);
    if (key == 27) break;  // ESC also exits nicely

    if (g_need_replan && g_have_src && g_have_dest) {
      // Run planner

      auto djikstra_start = std::chrono::high_resolution_clock::now();
      auto path_djikstra = djikstra_planner.shortest_path(g_src, g_dest);
      auto djikstra_stop = std::chrono::high_resolution_clock::now();
      auto djikstra_duration =
        std::chrono::duration_cast<std::chrono::microseconds>(djikstra_stop - djikstra_start);
      std::cout << "Djikstra took " << djikstra_duration.count() << " microseconds to plan path."
                << std::endl;
      std::cout << "Total cost is " << get_total_path_cost(costmap, path_djikstra) << std::endl;
      
      a_star_planner.set_grid_connectivity(GridConnectivity::EIGHT_CONNECTED);
      a_star_planner.set_heuristic(zero_distance);
      auto astar_start = std::chrono::high_resolution_clock::now();
      auto path_astar_zero = a_star_planner.shortest_path(g_src, g_dest);
      auto astar_stop = std::chrono::high_resolution_clock::now();
      auto astar_duration =
        std::chrono::duration_cast<std::chrono::microseconds>(astar_stop - astar_start);
      std::cout << "A* with h(n) = 0 took " << astar_duration.count() << " microseconds to plan path."
                << std::endl;
      std::cout << "Total cost is " << get_total_path_cost(costmap, path_astar_zero) << std::endl;

      /*
      a_star_planner.set_grid_connectivity(GridConnectivity::FOUR_CONNECTED);
      a_star_planner.set_heuristic(manhattan_distance);
      astar_start = std::chrono::high_resolution_clock::now();
      auto path_astar_manhattan = a_star_planner.shortest_path(g_src, g_dest);
      astar_stop = std::chrono::high_resolution_clock::now();
      astar_duration =
        std::chrono::duration_cast<std::chrono::microseconds>(astar_stop - astar_start);
      std::cout << "A* with manhattan heuristic took " << astar_duration.count()
                << " microseconds to plan path." << std::endl;
      std::cout << "Total cost is " << get_total_path_cost(costmap, path_astar_manhattan)
                << std::endl;

      a_star_planner.set_grid_connectivity(GridConnectivity::EIGHT_CONNECTED);
      a_star_planner.set_heuristic(chebychev_distance);
      astar_start = std::chrono::high_resolution_clock::now();
      auto path_astar_chebyshev = a_star_planner.shortest_path(g_src, g_dest);
      astar_stop = std::chrono::high_resolution_clock::now();
      astar_duration =
        std::chrono::duration_cast<std::chrono::microseconds>(astar_stop - astar_start);
      std::cout << "A* with chebyshev heuristic took " << astar_duration.count()
                << " microseconds to plan path." << std::endl;
      std::cout << "Total cost is " << get_total_path_cost(costmap, path_astar_chebyshev)
                << std::endl;

      a_star_planner.set_heuristic(euclidean_distance);
      astar_start = std::chrono::high_resolution_clock::now();
      auto path_astar_euclidean = a_star_planner.shortest_path(g_src, g_dest);
      astar_stop = std::chrono::high_resolution_clock::now();
      astar_duration =
        std::chrono::duration_cast<std::chrono::microseconds>(astar_stop - astar_start);
      std::cout << "A* with euclidean heuristic took " << astar_duration.count()
                << " microseconds to plan path." << std::endl;
      std::cout << "Total cost is " << get_total_path_cost(costmap, path_astar_euclidean)
                << std::endl;

      a_star_planner.set_heuristic(octile);
      astar_start = std::chrono::high_resolution_clock::now();
      auto path_astar_octile = a_star_planner.shortest_path(g_src, g_dest);
      astar_stop = std::chrono::high_resolution_clock::now();
      astar_duration =
        std::chrono::duration_cast<std::chrono::microseconds>(astar_stop - astar_start);
      std::cout << "A* with octile heuristic took " << astar_duration.count()
                << " microseconds to plan path." << std::endl;
      std::cout << "Total cost is " << get_total_path_cost(costmap, path_astar_octile) << std::endl;
      */

      // Draw path
      for (const auto & p : path_djikstra) {
        g_display.at<cv::Vec3b>(p.row, p.col) = {255, 255, 0};
      }
      for (const auto & p : path_astar_zero) {
        g_display.at<cv::Vec3b>(p.row, p.col) = {255, 0, 0};
      }
      /*
      for (const auto & p : path_astar_chebyshev) {
        g_display.at<cv::Vec3b>(p.row, p.col) = {0, 255, 0};
      }
      for (const auto & p : path_astar_manhattan) {
        g_display.at<cv::Vec3b>(p.row, p.col) = {255, 0, 0};
      }
      for (const auto & p : path_astar_euclidean) {
        g_display.at<cv::Vec3b>(p.row, p.col) = {255, 0, 255};
      }
      for (const auto & p : path_astar_octile) {
        g_display.at<cv::Vec3b>(p.row, p.col) = {52, 171, 235};
      }*/

      cv::imshow("Costmap", g_display);

      g_need_replan = false;  // Done planning
    }
  }
}