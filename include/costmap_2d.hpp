#ifndef COSTMAP_2D_HPP
#define COSTMAP_2D_HPP

#include <memory>
#include <vector>

#include "cell_2d.hpp"

const uint8_t kFreeSpaceCost = 0;
const uint8_t kInscribedInflatedObstacle = 253;
const uint8_t kLethalObstacleCost = 254;
const uint8_t kNoInformationCost = 255;

class Costmap2D
{
public:
  explicit Costmap2D(
    int height, int width, const std::vector<uint8_t> & data, double cell_resolution,
    double inflation_radius = 0.0);

  uint8_t get_cost(const Cell & coords) const;

  int get_rows();

  int get_cols();

  double get_cell_resolution();

private:
  void inflate_obstacles();

  int rows_;
  int cols_;
  double cell_resolution_;
  double inflation_radius_;
  std::shared_ptr<std::vector<uint8_t>> data_;
  std::vector<uint8_t> inflated_boundary_data_;
};

#endif  // COSTMAP_2D_HPP