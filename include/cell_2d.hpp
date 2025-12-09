#ifndef CELL_2D_HPP
#define CELL_2D_HPP

#include <iostream>

struct Cell
{
  Cell(int row, int col)
  {
    this->col = col;
    this->row = row;
  }

  bool operator==(Cell other) { return (other.col == col) && (other.row == row); }

  bool operator!=(Cell other) { return !(other == *this); }

  bool operator<(const Cell & other) const
  {
    return (col < other.col) || ((col == other.col) && (row < other.row));
  }

  friend std::ostream & operator<<(std::ostream & os, const Cell & obj)
  {
    os << "(" << obj.row << "," << obj.col << ")";
    return os;
  }

  int col;
  int row;
};

#endif // CELL_2D_HPP