#include "costmap_2d.hpp"

#include <memory>

Costmap2D::Costmap2D(int rows, int cols, uint8_t free_space_cost) : rows_{rows}, cols_{cols}
{
  data_ = std::make_shared<std::vector<uint8_t>>(rows_ * cols_, free_space_cost);
}

Costmap2D::Costmap2D(int height, int width, const std::vector<uint8_t> & data)
: rows_{height}, cols_{width}, data_{std::make_shared<std::vector<uint8_t>>(data)}
{
}

uint8_t Costmap2D::get_cost(const Cell & coords) const
{
  if (coords.col >= cols_ || coords.row >= rows_ || coords.col < 0 || coords.row < 0) {
    throw std::out_of_range("Coordinates of of map range.");
  } else {
    return data_->at(coords.row * cols_ + coords.col);
  }
}

int Costmap2D::get_rows() { return rows_; };

int Costmap2D::get_cols() { return cols_; };
