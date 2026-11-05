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

// The octile distance from `source` to every cell: the shortest path with 8-connected moves, straight moves costing 1
// and diagonal moves sqrt(2), where a diagonal move needs both cells it passes between to be passable (no cutting
// corners); +infinity where there is none. This is the convention of the MovingAI scenario files' "optimal length"
// column, MAPF sets included, and it is used only to check those files: the agents here move 4-connected, and the
// planners use distances() above. Precondition: source < grid.cell_count().
std::vector<double> octile_distances(const Grid& grid, CellId source);

// octile_distances(grid, source)[target], computed by the same search stopped once `target` is settled: the same
// operations up to that point, so the same value to the last bit, usually in a fraction of the time. Preconditions:
// source and target < grid.cell_count().
double octile_distance(const Grid& grid, CellId source, CellId target);

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
