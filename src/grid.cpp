#include "pathfinder/grid.hpp"

#include <algorithm>
#include <stdexcept>
#include <string>

namespace pathfinder {

Grid::Grid(std::int32_t width, std::int32_t height) {
  if (width < 1 || width > kMaxDimension || height < 1 || height > kMaxDimension) {
    throw std::invalid_argument("grid size " + std::to_string(width) + "x" + std::to_string(height) +
                                " is outside 1.." + std::to_string(kMaxDimension));
  }
  width_ = width;
  height_ = height;
  passable_.assign(static_cast<std::size_t>(width) * static_cast<std::size_t>(height), 1);
}

std::size_t Grid::passable_count() const {
  return static_cast<std::size_t>(std::count(passable_.begin(), passable_.end(), std::uint8_t{1}));
}

void Grid::set_passable(Point p, bool passable) {
  if (!in_bounds(p)) {
    throw std::out_of_range("cell (" + std::to_string(p.x) + ", " + std::to_string(p.y) + ") is outside a " +
                            std::to_string(width_) + "x" + std::to_string(height_) + " grid");
  }
  passable_[index(p)] = passable ? 1 : 0;
}

int Grid::neighbors(CellId c, std::array<CellId, 4>& out) const {
  const auto w = static_cast<CellId>(width_);
  const auto x = c % w;
  const auto y = c / w;
  int n = 0;
  if (y > 0 && passable_[c - w] != 0) {
    out[static_cast<std::size_t>(n++)] = c - w;  // north
  }
  if (x + 1 < w && passable_[c + 1] != 0) {
    out[static_cast<std::size_t>(n++)] = c + 1;  // east
  }
  if (y + 1 < static_cast<CellId>(height_) && passable_[c + w] != 0) {
    out[static_cast<std::size_t>(n++)] = c + w;  // south
  }
  if (x > 0 && passable_[c - 1] != 0) {
    out[static_cast<std::size_t>(n++)] = c - 1;  // west
  }
  return n;
}

bool Grid::adjacent_or_same(CellId a, CellId b) const {
  const Point p = point(a);
  const Point q = point(b);
  const auto dx = p.x > q.x ? p.x - q.x : q.x - p.x;
  const auto dy = p.y > q.y ? p.y - q.y : q.y - p.y;
  return dx + dy <= 1;
}

}  // namespace pathfinder
