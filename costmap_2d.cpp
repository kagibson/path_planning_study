#include "costmap_2d.hpp"

#include <cmath>
#include <memory>

Costmap2D::Costmap2D(
  int rows, int cols, const std::vector<uint8_t> & data, double cell_resolution,
  double inflation_radius)
: rows_{rows},
  cols_{cols},
  data_{std::make_shared<std::vector<uint8_t>>(data)},
  cell_resolution_{cell_resolution},
  inflation_radius_{inflation_radius}
{
  if (inflation_radius > 0.0) {
    inflate_obstacles();
  }
}

uint8_t Costmap2D::get_cost(const Cell & coords) const
{
  if (coords.col >= cols_ || coords.row >= rows_ || coords.col < 0 || coords.row < 0) {
    throw std::out_of_range("Coordinates of of map range.");
  } else if (inflated_boundary_data_.empty()) {
    return data_->at(coords.row * cols_ + coords.col);
  } else {
    return inflated_boundary_data_.at(coords.row * cols_ + coords.col);
  }
}

int Costmap2D::get_rows() { return rows_; };

int Costmap2D::get_cols() { return cols_; };

double Costmap2D::get_cell_resolution() {return cell_resolution_; };

void Costmap2D::inflate_obstacles()
{
  int inflation_radius_cells = std::ceil(inflation_radius_ / cell_resolution_);
  std::cout << "Inflation radius in cells " << inflation_radius_cells << std::endl;
  inflated_boundary_data_ = *data_;
  for (int r = 0; r < rows_; r++) {
    for (int c = 0; c < cols_; c++) {
      uint8_t cell_cost = data_->at(r * cols_ + c);
      if (cell_cost >= kInscribedInflatedObstacle) {
        for (int dr = -inflation_radius_cells; dr <= inflation_radius_cells; dr++) {
          for (int dc = -inflation_radius_cells; dc <= inflation_radius_cells; dc++) {
            int nr = r + dr;
            int nc = c + dc;

            if ((nr < 0) || (nr >= rows_) || (nc < 0) || (nc >= cols_)) {
              continue;
            }

            double dist = std::sqrt(dr * dr + dc * dc);

            if (dist > inflation_radius_cells) {
              continue;
            }

            uint8_t inflated_cost =
              uint8_t(std::max(0.0, 255.0 - (255.0 * dist / inflation_radius_cells)));
            inflated_boundary_data_.at(nr * cols_ + nc) =
              std::max(inflated_boundary_data_.at(nr * cols_ + nc), inflated_cost);
          }
        }
      }
    }
  }
}
