#pragma once

// The reservation table: what a space-time search must avoid (DESIGN.md "The planners"). It is written from the
// searching agent's point of view: every entry forbids that agent something. Prioritized planning turns the paths of
// the agents planned before into entries (reserve_path); CBS (day 3) will add its constraints directly.

#include <cstddef>
#include <cstdint>
#include <optional>
#include <unordered_map>
#include <unordered_set>

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
  // so the two can't swap; and its last cell from its last time on, where it stays after its path ends.
  // Precondition: a non-empty path of legal steps.
  void reserve_path(const Path& path);

  // Whether the agent may be in `cell` at time t.
  bool vertex_free(CellId cell, std::uint32_t t) const;
  // Whether the agent may move from `from` to its neighbour `to` between t and t + 1 (it says nothing about `to` at
  // t + 1; ask vertex_free for that).
  bool move_free(CellId from, CellId to, std::uint32_t t) const;
  // The latest t of a forbid_vertex on `cell`, if any: a path can end at `cell` only after it.
  std::optional<std::uint32_t> last_vertex_time(CellId cell) const;
  // The time from which `cell` is forbidden for ever, if it is.
  std::optional<std::uint32_t> forbidden_from(CellId cell) const;
  // The latest time any entry concerns: t for a vertex or a forbid_from, t + 1 for a move; 0 for an empty table.
  // From horizon() + 1 on nothing changes with time any more, which is what bounds the search.
  std::uint32_t horizon() const { return horizon_; }
  bool empty() const { return vertices_.empty() && moves_.empty() && from_.empty(); }
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

  std::int32_t width_;
  std::size_t cells_;
  std::unordered_set<std::uint64_t, Mix> vertices_;               // key(cell, t)
  std::unordered_map<std::uint64_t, std::uint8_t, Mix> moves_;     // key(from, t) -> a bit per direction
  std::unordered_map<CellId, std::uint32_t> from_;                // cell -> forbidden from this time on
  std::unordered_map<CellId, std::uint32_t> last_vertex_;         // cell -> latest forbid_vertex time
  std::uint32_t horizon_ = 0;
};

}  // namespace pathfinder
