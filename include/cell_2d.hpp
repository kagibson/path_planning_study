#ifndef CELL_2D_HPP
#define CELL_2D_HPP

#include <iostream>
#include <unordered_map>

struct Cell
{
  Cell()
  {
    this->col = 0;
    this->row = 0;
  }

  Cell(int row, int col)
  {
    this->col = col;
    this->row = row;
  }

  bool operator==(const Cell & other) const { return (other.col == col) && (other.row == row); }

  bool operator!=(const Cell & other) const { return !(other == *this); }

  bool operator<(const Cell & other) const
  {
    return (col < other.col) || ((col == other.col) && (row < other.row));
  }

  bool is_diagonal_to(const Cell & other) const
  {
    return ((col != other.col) && (row != other.row));
  }

  friend std::ostream & operator<<(std::ostream & os, const Cell & obj)
  {
    os << "(" << obj.row << "," << obj.col << ")";
    return os;
  }

  int col;
  int row;
};

/**
 * @brief Hash function for hashing Cell structs, not ideal but
 * will do for now.
 * 
 */
struct CellHash
{
  std::size_t operator()(const Cell & cell) const
  {
    size_t h1 = std::hash<int>{}(cell.col);
    size_t h2 = std::hash<int>{}(cell.row);
    return h1 ^ (h2 << 1);
  }
};

#endif  // CELL_2D_HPP