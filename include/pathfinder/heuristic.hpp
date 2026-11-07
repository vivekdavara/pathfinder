#pragma once

// The planners' heuristic tables (DESIGN.md "The planners", the heuristic budget). An agent's heuristic is the BFS
// distance from every cell to its goal. distances() (distance.hpp) stores it for every cell of the grid, blocked or
// not; a DistanceTable stores it only for the passable cells, through a PassableIndex the tables of one grid share.
// On orz900d, where only 96,603 of 978,096 cells are passable, that is a tenth of the memory per agent.

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include "pathfinder/distance.hpp"
#include "pathfinder/grid.hpp"

namespace pathfinder {

// Numbers a grid's passable cells 0, 1, ..., passable_count() - 1 in row-major order (the order of their CellIds).
class PassableIndex {
 public:
  static constexpr std::uint32_t kBlocked = kUnreachable;

  explicit PassableIndex(const Grid& grid);

  // The number of passable cells.
  std::size_t size() const { return cells_.size(); }
  // The cell's number, or kBlocked for a blocked cell. Precondition: c < the grid's cell_count().
  std::uint32_t operator[](CellId c) const { return index_[c]; }
  // The cell with number i. Precondition: i < size().
  CellId cell(std::uint32_t i) const { return cells_[i]; }
  // The numbers of cell i's passable 4-neighbours, in the grid's order (north, east, south, west), then kBlocked.
  // Precondition: i < size().
  const std::array<std::uint32_t, 4>& neighbors(std::uint32_t i) const { return neighbors_[i]; }
  // The memory the index holds (4 bytes per cell, and 20 per passable cell).
  std::size_t bytes() const;
  // Whether this is g's index: the same width and cell count, and exactly g's passable cells (the same number of
  // cells, all of them passable in g, is the same set). O(cells) to count g's passable cells (a scan of a byte per
  // cell), then O(passable cells).
  bool fits(const Grid& g) const;

 private:
  std::int32_t width_ = 0;
  std::vector<std::uint32_t> index_;                     // by CellId
  std::vector<CellId> cells_;                            // by number
  std::vector<std::array<std::uint32_t, 4>> neighbors_;  // by number
};

// One goal's heuristic: the BFS distance from every passable cell to `goal`, ignoring other agents, held by passable
// number. The same values as distances(grid, goal), only stored more compactly.
class DistanceTable {
 public:
  // `index` must be the grid's (PassableIndex::fits; std::invalid_argument otherwise). A blocked goal reaches
  // nothing. Precondition: goal < grid.cell_count().
  DistanceTable(const Grid& grid, std::shared_ptr<const PassableIndex> index, CellId goal);

  CellId goal() const { return goal_; }
  // The distance from c to the goal: kUnreachable if c is blocked or can't reach it.
  // Precondition: c < the grid's cell_count().
  std::uint32_t operator()(CellId c) const {
    const std::uint32_t i = (*index_)[c];
    return i == PassableIndex::kBlocked ? kUnreachable : dist_[i];
  }
  // The memory this table holds on its own, not counting the shared index (4 bytes per passable cell).
  std::size_t bytes() const { return dist_.size() * sizeof(std::uint32_t); }

 private:
  std::shared_ptr<const PassableIndex> index_;
  CellId goal_ = 0;
  std::vector<std::uint32_t> dist_;
};

}  // namespace pathfinder
