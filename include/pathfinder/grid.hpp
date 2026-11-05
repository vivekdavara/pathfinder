#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace pathfinder {

// A cell's coordinates: x is the column (0 at the left), y the row (0 at the top), as in the MovingAI formats.
struct Point {
  std::int32_t x = 0;
  std::int32_t y = 0;
  friend bool operator==(Point, Point) = default;
};

// A cell's index in a grid, y * width + x. Searches and plans use these; Point is for people and files.
using CellId = std::uint32_t;

// The largest width or height a grid may have. 10,000 x 10,000 cells still fit a CellId; the largest map in the
// MovingAI MAPF set is orz900d, 1,491 x 656.
inline constexpr std::int32_t kMaxDimension = 10'000;

// A 4-connected grid of passable and blocked cells. Moving to a passable neighbour (north, east, south or west) or
// waiting in place costs one time step; cells outside the grid are blocked.
class Grid {
 public:
  // A grid with no cells (what a default-constructed member holds until it's assigned).
  Grid() = default;
  // A width x height grid with every cell passable. Throws std::invalid_argument unless both are 1..kMaxDimension.
  Grid(std::int32_t width, std::int32_t height);

  std::int32_t width() const { return width_; }
  std::int32_t height() const { return height_; }
  std::size_t cell_count() const { return passable_.size(); }
  std::size_t passable_count() const;

  bool in_bounds(Point p) const { return p.x >= 0 && p.y >= 0 && p.x < width_ && p.y < height_; }
  // False outside the grid.
  bool passable(Point p) const { return in_bounds(p) && passable_[index(p)] != 0; }
  // Precondition: c < cell_count().
  bool passable(CellId c) const { return passable_[c] != 0; }
  // Throws std::out_of_range outside the grid.
  void set_passable(Point p, bool passable);

  // Precondition: in_bounds(p).
  CellId id(Point p) const { return static_cast<CellId>(index(p)); }
  // Precondition: c < cell_count().
  Point point(CellId c) const {
    return Point{static_cast<std::int32_t>(c % static_cast<CellId>(width_)),
                 static_cast<std::int32_t>(c / static_cast<CellId>(width_))};
  }

  // The passable 4-neighbours of cell c, in the fixed order north, east, south, west (searches break ties by it, so
  // runs are the same everywhere). Writes them to the front of `out` and returns how many there are.
  // Precondition: c < cell_count().
  int neighbors(CellId c, std::array<CellId, 4>& out) const;

  // True when a and b are the same cell or 4-neighbours, i.e. one time step can take an agent from a to b (both
  // must also be passable for that step to be legal). Precondition: both < cell_count().
  bool adjacent_or_same(CellId a, CellId b) const;

  friend bool operator==(const Grid&, const Grid&) = default;

 private:
  std::size_t index(Point p) const {
    return static_cast<std::size_t>(p.y) * static_cast<std::size_t>(width_) + static_cast<std::size_t>(p.x);
  }

  std::int32_t width_ = 0;
  std::int32_t height_ = 0;
  std::vector<std::uint8_t> passable_;
};

}  // namespace pathfinder
