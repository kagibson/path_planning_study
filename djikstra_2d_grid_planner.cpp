#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <list>
#include <queue>
#include <limits>
#include <functional>
#include <memory>
#include <tuple>
#include <stdexcept>
#include <chrono>
#include <ctime>
#include <cmath>

const int INF = std::numeric_limits<int>::max();

char int_to_alpha(const int num)
{
  if ((num >= 0) && (num < 26)) {
    return (char)('A'+num);
  }
  return '?'; 
}

struct Cell
{
  Cell(int row, int col)
  {
    this->col = col;
    this->row = row;
  }

  bool operator==(Cell other) {
    return (other.col == col) && (other.row == row);
  }

  bool operator!=(Cell other) {
    return !(other==*this);
  }

  bool operator<(const Cell & other) const
  {
    return (col < other.col) || ((col == other.col) && (row < other.row));
  }

  friend std::ostream& operator<<(std::ostream& os, const Cell& obj)
  {
    os << "(" << obj.row << "," << obj.col << ")";
    return os;
  }

  int col;
  int row;
};

long long chebychev_distance(const Cell &p1, const Cell &p2)
{
  return std::max(llabs(p2.col-p1.col), llabs(p2.row-p2.row));
}

long long manhattan_distance(const Cell &p1, const Cell &p2)
{
  return llabs(p2.col-p1.col) + llabs(p2.row-p1.row);
}

long long euclidean_distance(const Cell &p1, const Cell &p2)
{
  return std::sqrt((p2.col - p1.col)^2 + (p2.row - p1.row)^2);
}

class Costmap2D
{
public:
  Costmap2D(int rows, int cols, uint8_t free_space_cost) : 
    rows_{rows}, cols_{cols}
  {
    data_ = std::make_shared<std::vector<uint8_t>>(rows_*cols_, free_space_cost);
  }

  Costmap2D(int height, int width, const std::vector<uint8_t> &data) : 
    rows_{height}, cols_{width}, data_{std::make_shared<std::vector<uint8_t>>(data)} {}

  uint8_t get_cost(const Cell &coords) const {
    if (coords.col >= cols_ || coords.row >= rows_ || coords.col < 0 || coords.row < 0) {
      throw std::out_of_range("Coordinates of of map range.");
    } else {
      return data_->at(coords.row*cols_ + coords.col);
    }
  }

  int get_rows() {return rows_;};

  int get_cols() {return cols_;};

private:
  int rows_;
  int cols_;
  std::shared_ptr<std::vector<uint8_t>> data_;

};

class Djikstra2DPlanner
{
public:

  Djikstra2DPlanner(const std::shared_ptr<Costmap2D> &costmap_ptr)
  {
    costmap_ptr_ = costmap_ptr;
  }

  std::vector<Cell> shortest_path(Cell src, Cell dest)
  {
    
    std::vector<Cell> path;

    if (costmap_ptr_ ==  nullptr) {
      std::cout << "Costmap not provided" << std::endl;
      return path;
    }

    // Initialize distance grid
    std::vector<std::vector<std::pair<long long, Cell>>> dists;
    for (int i=0; i<costmap_ptr_->get_rows(); i++) {
      dists.emplace_back(costmap_ptr_->get_cols(), std::make_pair<long long, Cell>(INF, Cell(-1, -1)));
    }

    dists[src.row][src.col].first = 0;
    dists[src.row][src.col].second = src;

    std::priority_queue<std::pair<long long, Cell>, std::vector<std::pair<long long, Cell>>, std::greater<std::pair<long long, Cell>>> pq;
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
        continue;;
      }

      /* Update all neighbor costs */
      std::vector<Cell> neighbor_cells =
      {
        {cell.row-1, cell.col-1},
        {cell.row-1, cell.col},
        {cell.row-1, cell.col+1},
        {cell.row, cell.col-1},
        {cell.row, cell.col+1},
        {cell.row+1, cell.col-1},
        {cell.row+1, cell.col},
        {cell.row+1, cell.col+1}
      };

      for (const auto &neighbor_cell : neighbor_cells) {
        // Check bounds
        if (neighbor_cell.col < 0 || neighbor_cell.col >= cols || 
        neighbor_cell.row < 0 || neighbor_cell.row >= rows) {
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

private:
  std::shared_ptr<Costmap2D> costmap_ptr_;
};

struct NodeRecord {
  long long f; // g(n) + h(n)
  long long g; // Cost from start to cell
  Cell cell;

  bool operator>(const NodeRecord &other) const {
    return f > other.f;
  }

  bool operator<(const NodeRecord &other) const {
    return f < other.f;
  }
};

using HeuristicFcn = std::function<long long(const Cell&, const Cell&)>;

class AStar2DPlanner
{
public:

  AStar2DPlanner(const std::shared_ptr<Costmap2D> &costmap_ptr, HeuristicFcn heuristic)
  {
    costmap_ptr_ = costmap_ptr;
    heuristic_ = heuristic;
  }

  std::vector<Cell> shortest_path(Cell src, Cell dest)
  {
    
    std::vector<Cell> path;

    if (costmap_ptr_ ==  nullptr) {
      std::cout << "Costmap not provided" << std::endl;
      return path;
    }

    // Initialize g_costs
    std::vector<std::vector<long long>> g_costs;
    for (int i=0; i<costmap_ptr_->get_rows(); i++) {
      g_costs.emplace_back(costmap_ptr_->get_cols(), INF);
    }
    g_costs[src.row][src.col] = 0;

    std::vector<std::vector<Cell>> parents;
    for (int i=0; i<costmap_ptr_->get_rows(); i++) {
      parents.emplace_back(costmap_ptr_->get_cols(), Cell(-1,-1));
    }// TODO: I think that this and g_costs could be a different data structure as we won't be visiting every cell
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
        continue;;
      }

      /* Update all neighbor costs */
      std::vector<Cell> neighbor_cells =
      {
        {cell.row-1, cell.col-1},
        {cell.row-1, cell.col},
        {cell.row-1, cell.col+1},
        {cell.row, cell.col-1},
        {cell.row, cell.col+1},
        {cell.row+1, cell.col-1},
        {cell.row+1, cell.col},
        {cell.row+1, cell.col+1}
      };

      for (const auto &neighbor_cell : neighbor_cells) {
        if (neighbor_cell.col < 0 || neighbor_cell.col >= cols || 
        neighbor_cell.row < 0 || neighbor_cell.row >= rows) {
          continue;
        }
        
        long long new_g = g_cost + costmap_ptr_->get_cost(neighbor_cell);
        if (new_g < g_costs.at(neighbor_cell.row).at(neighbor_cell.col)) {
          g_costs.at(neighbor_cell.row).at(neighbor_cell.col) = new_g;
          parents.at(neighbor_cell.row).at(neighbor_cell.col) = cell;
          long long new_f = new_g + heuristic_(neighbor_cell, dest);
          NodeRecord nr {new_f, new_g, neighbor_cell};
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

  void set_heuristic(HeuristicFcn heuristic) {
    heuristic_ = heuristic;
  }

private:
  std::shared_ptr<Costmap2D> costmap_ptr_;
  HeuristicFcn heuristic_;
};


cv::Mat g_base;
cv::Mat g_display;
Cell g_src(-1, -1);
Cell g_dest(-1, -1);
bool g_have_src = false;
bool g_have_dest = false;
bool g_need_replan = false;

void mouseCallback(int event, int x, int y, int flags, void* userdata)
{
    if (!g_have_src && (event == cv::EVENT_LBUTTONDOWN)) {
        g_src = Cell(y, x);
        g_have_src = true;
        g_need_replan = false;   // not ready yet
        std::cout << "Source selected: (" << g_src.row << ", " << g_src.col << ")\n";

        // Draw a small green circle at src
        cv::circle(g_display, cv::Point(x,y), 3, cv::Scalar(0,255,0), -1);
    }
    else if (!g_have_dest & (event == cv::EVENT_LBUTTONDOWN)) {
        g_dest = Cell(y, x);
        g_have_dest = true;
        g_need_replan = true;   // not ready yet

        std::cout << "Destination selected: (" << g_dest.row << ", " << g_dest.col << ")\n";

        // Draw a small red circle at dest
        cv::circle(g_display, cv::Point(x,y), 3, cv::Scalar(0,0,255), -1);
    } else if (event == cv::EVENT_RBUTTONDOWN) {
        g_have_src = false;
        g_have_dest = false;
        g_need_replan = false;
        g_display = g_base.clone();
    }

    cv::imshow("Costmap", g_display);
}

long long get_total_path_cost(const std::shared_ptr<Costmap2D> &costmap, const std::vector<Cell> &path)
{
  long long total_cost = 0;

  // Skip the first cell (src), because entering it costs nothing
  for (size_t i = 1; i < path.size(); i++) {
      total_cost += costmap->get_cost(path[i]);
  }

  return total_cost;
}


int main(int argc, char **argv)
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

  Djikstra2DPlanner planner_dijkstra(costmap);
  AStar2DPlanner planner_astar(costmap, chebychev_distance);

  // TODO:  demonstrate different heuristics, weighting of heuristic vs cost, diagonal move costs
  while (true)
  {
      int key = cv::waitKey(10);
      if (key == 27) break; // ESC also exits nicely

      if (g_need_replan && g_have_src && g_have_dest)
      {
          // Run planner
          auto djikstra_start = std::chrono::high_resolution_clock::now();
          auto path_djikstra = planner_dijkstra.shortest_path(g_src, g_dest);
          auto djikstra_stop = std::chrono::high_resolution_clock::now();
          auto djikstra_duration = std::chrono::duration_cast<std::chrono::microseconds>(djikstra_stop - djikstra_start);
          std::cout << "Djikstra took " << djikstra_duration.count() << " microseconds to plan path." << std::endl;
          std::cout << "Total cost is " << get_total_path_cost(costmap, path_djikstra) << std::endl;
          
          planner_astar.set_heuristic(chebychev_distance);
          auto astar_start = std::chrono::high_resolution_clock::now();
          auto path_astar = planner_astar.shortest_path(g_src, g_dest);
          auto astar_stop = std::chrono::high_resolution_clock::now();
          auto astar_duration = std::chrono::duration_cast<std::chrono::microseconds>(astar_stop - astar_start);
          std::cout << "A* with chebyshev heuristic took " << astar_duration.count() << " microseconds to plan path." << std::endl;
          std::cout << "Total cost is " << get_total_path_cost(costmap, path_astar) << std::endl;

          planner_astar.set_heuristic(manhattan_distance);
          astar_start = std::chrono::high_resolution_clock::now();
          auto path_astar_manhattan = planner_astar.shortest_path(g_src, g_dest);
          astar_stop = std::chrono::high_resolution_clock::now();
          astar_duration = std::chrono::duration_cast<std::chrono::microseconds>(astar_stop - astar_start);
          std::cout << "A* with manhattan heuristic took " << astar_duration.count() << " microseconds to plan path." << std::endl;
          std::cout << "Total cost is " << get_total_path_cost(costmap, path_astar_manhattan) << std::endl;

          planner_astar.set_heuristic(euclidean_distance);
          astar_start = std::chrono::high_resolution_clock::now();
          auto path_astar_euclidean = planner_astar.shortest_path(g_src, g_dest);
          astar_stop = std::chrono::high_resolution_clock::now();
          astar_duration = std::chrono::duration_cast<std::chrono::microseconds>(astar_stop - astar_start);
          std::cout << "A* with euclidean heuristic took " << astar_duration.count() << " microseconds to plan path." << std::endl;
          std::cout << "Total cost is " << get_total_path_cost(costmap, path_astar_euclidean) << std::endl;\

          // Draw path
          for (const auto& p : path_djikstra) {
              g_display.at<cv::Vec3b>(p.row, p.col) = {0, 0, 255};
          }
          for (const auto& p : path_astar) {
              g_display.at<cv::Vec3b>(p.row, p.col) = {0, 255, 0};
          }
          for (const auto& p : path_astar_manhattan) {
              g_display.at<cv::Vec3b>(p.row, p.col) = {255, 0, 0};
          }
          for (const auto& p : path_astar_euclidean) {
              g_display.at<cv::Vec3b>(p.row, p.col) = {255, 0, 255};
          }

          cv::imshow("Costmap", g_display);

          g_need_replan = false;   // Done planning
      }
  }

}
