#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

#include "pathfinder/grid.hpp"

namespace pathfinder {

// The distance BFS gives a cell it never reaches (blocked, or in another connected piece).
inline constexpr std::uint32_t kUnreachable = std::numeric_limits<std::uint32_t>::max();

// The number of moves from `source` to every cell, by breadth-first search over passable 4-neighbours, ignoring
// other agents; kUnreachable where there is no way. Moves are reversible, so this is also every cell's distance *to*
// `source`: the planners' heuristic for an agent is distances(grid, goal). A blocked source reaches nothing, not even
// itself. Precondition: source < grid.cell_count().
std::vector<std::uint32_t> distances(const Grid& grid, CellId source);

// The grid's connected pieces of passable cells (4-connected).
struct Components {
  static constexpr std::uint32_t kNone = std::numeric_limits<std::uint32_t>::max();
  // label[c] is cell c's piece (0, 1, ...; numbered in the order of each piece's lowest cell), or kNone if blocked.
  std::vector<std::uint32_t> label;
  // sizes[i] is the number of cells in piece i.
  std::vector<std::size_t> sizes;
  // The largest piece (the lowest-numbered of equal ones), or kNone if no cell is passable.
  std::uint32_t largest = kNone;
};

Components components(const Grid& grid);

}  // namespace pathfinder
