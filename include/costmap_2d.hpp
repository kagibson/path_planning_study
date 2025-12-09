#ifndef COSTMAP_2D_HPP
#define COSTMAP_2D_HPP

#include <vector>
#include <memory>
#include "cell_2d.hpp"


class Costmap2D
{
public:
  Costmap2D(int rows, int cols, uint8_t free_space_cost);

  Costmap2D(int height, int width, const std::vector<uint8_t> & data);

  uint8_t get_cost(const Cell & coords) const;

  int get_rows();

  int get_cols();

private:
  int rows_;
  int cols_;
  std::shared_ptr<std::vector<uint8_t>> data_;
};

#endif // COSTMAP_2D_HPP