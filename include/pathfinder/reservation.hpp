#pragma once

// The reservation table: what a space-time search must avoid (DESIGN.md "The planners"). It is written from the
// searching agent's point of view: every entry forbids that agent something. Prioritized planning turns the paths of
// the agents planned before into entries (reserve_path); CBS (day 3) will add its constraints directly.

#include <cstddef>
#include <cstdint>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "pathfinder/grid.hpp"
#include "pathfinder/plan.hpp"

namespace pathfinder {

class ReservationTable {
 public:
  // The latest time an entry may name. It keeps every time the search computes from one (up to horizon() + 1 plus
  // the grid's cell count) inside 32 bits.
  static constexpr std::uint32_t kMaxTime = 1U << 30;

  // A table for searches on `grid` (it keeps the grid's size, to tell moves apart, not the grid).
  explicit ReservationTable(const Grid& grid);

  // The agent may not be in `cell` at time t.
  void forbid_vertex(CellId cell, std::uint32_t t);
  // The agent may not move from `from` to `to` between t and t + 1. `to` must be a 4-neighbour of `from` (a wait
  // is forbidden with forbid_vertex).
  void forbid_move(CellId from, CellId to, std::uint32_t t);
  // The agent may not be in `cell` at time t or at any time after it: another agent rests there for ever.
  // Forbidding a cell from two times keeps the earlier.
  void forbid_from(CellId cell, std::uint32_t t);
  // Reserves another agent's path (DESIGN.md "The model"): its cell at each time; the reverse of each of its moves,
  // so the two can't swap; and its last cell from its last time on, where it stays after its path ends. Throws, and
  // changes nothing, for an empty path, one longer than kMaxTime, a cell outside the grid (std::out_of_range) or a
  // step that is neither a wait nor a move to a 4-neighbour (std::invalid_argument). (Whether the cells are passable
  // is the grid's business; the table doesn't keep it.)
  void reserve_path(const Path& path);

  // Whether the agent may be in `cell` at time t. Precondition: cell < cell_count().
  bool vertex_free(CellId cell, std::uint32_t t) const {
    return (marks_[cell] & kCellMark) == 0 || cell_free(cell, t);
  }
  // Whether the agent may move from `from` to its neighbour `to` between t and t + 1 (it says nothing about `to` at
  // t + 1; ask vertex_free for that). Preconditions: both < cell_count(), and 4-neighbours.
  bool move_free(CellId from, CellId to, std::uint32_t t) const {
    return (marks_[from] & kMoveMark) == 0 || move_allowed(from, to, t);
  }
  // The latest t of a forbid_vertex on `cell`, if any: a path can end at `cell` only after it.
  std::optional<std::uint32_t> last_vertex_time(CellId cell) const;
  // The time from which `cell` is forbidden for ever, if it is.
  std::optional<std::uint32_t> forbidden_from(CellId cell) const;
  // The latest time any entry concerns: t for a vertex or a forbid_from, t + 1 for a move; 0 for an empty table.
  // From horizon() + 1 on nothing changes with time any more, which is what bounds the search.
  std::uint32_t horizon() const { return horizon_; }
  bool empty() const { return vertices_.empty() && moves_.empty() && from_.empty(); }
  // The number of cells of the grid it was made for.
  std::size_t cell_count() const { return cells_; }
  // Whether this table was made for a grid of g's size (searches need the table and the grid to agree).
  bool fits(const Grid& g) const { return g.width() == width_ && g.cell_count() == cells_; }
  // The number of entries of each kind (a forbid_from counts once per cell).
  std::size_t vertex_count() const { return vertices_.size(); }
  std::size_t move_count() const;
  std::size_t from_count() const { return from_.size(); }

 private:
  // A 64-bit key with its bits mixed (SplitMix64's finaliser), so the hash tables' buckets use all of them.
  struct Mix {
    std::size_t operator()(std::uint64_t k) const {
      k ^= k >> 30;
      k *= 0xBF58476D1CE4E5B9ULL;
      k ^= k >> 27;
      k *= 0x94D049BB133111EBULL;
      k ^= k >> 31;
      return static_cast<std::size_t>(k);
    }
  };
  static std::uint64_t key(CellId cell, std::uint32_t t) { return (std::uint64_t{t} << 32) | cell; }
  // The direction of the move from -> to (0 north, 1 east, 2 south, 3 west); throws unless they are 4-neighbours.
  unsigned direction(CellId from, CellId to) const;
  void check(CellId cell, std::uint32_t t) const;
  bool cell_free(CellId cell, std::uint32_t t) const;
  bool move_allowed(CellId from, CellId to, std::uint32_t t) const;

  // A byte per cell saying whether any entry names it, so the search's lookups of the many cells nothing names skip
  // the hash tables (with 50 planned paths in the table, they doubled the cost of an expansion).
  static constexpr std::uint8_t kCellMark = 1;  // a vertex or forbid_from entry on this cell
  static constexpr std::uint8_t kMoveMark = 2;  // a move entry from this cell

  std::int32_t width_;
  std::size_t cells_;
  std::vector<std::uint8_t> marks_;  // by cell
  std::unordered_set<std::uint64_t, Mix> vertices_;               // key(cell, t)
  std::unordered_map<std::uint64_t, std::uint8_t, Mix> moves_;     // key(from, t) -> a bit per direction
  std::unordered_map<CellId, std::uint32_t> from_;                // cell -> forbidden from this time on
  std::unordered_map<CellId, std::uint32_t> last_vertex_;         // cell -> latest forbid_vertex time
  std::uint32_t horizon_ = 0;
};

}  // namespace pathfinder
