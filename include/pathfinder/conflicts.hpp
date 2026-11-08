#pragma once

// Finding a plan's conflicts quickly, for CBS (DESIGN.md "The planners"). CBS looks at every node's plan: it needs
// the number of conflicts (its tie-break) and the first one (the one it splits on). These are exactly the vertex and
// edge conflicts validate() reports, each agent staying at its path's last cell after the path ends; the scanner
// finds them with a few arrays sized once for the grid instead of a hash table rebuilt at every time step.

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

#include "pathfinder/plan.hpp"

namespace pathfinder {

struct Conflict {
  // false: agents a and b are both in `cell` at `time`. true: they swap, a going `cell` -> `to` while b goes
  // `to` -> `cell`, between `time` and `time` + 1.
  bool edge = false;
  std::size_t a = 0;  // a < b
  std::size_t b = 0;
  std::uint32_t time = 0;
  CellId cell = 0;
  CellId to = 0;  // edge conflicts only (0 for a vertex conflict)
  friend bool operator==(const Conflict&, const Conflict&) = default;
};

// Whether x comes before y in time: a vertex conflict at t happens before a swap between t and t + 1, which happens
// before anything at t + 1; then by (a, b).
bool earlier(const Conflict& x, const Conflict& y);

struct ConflictScan {
  // Every vertex conflict (one per pair of agents in a cell at a time, as validate() counts them: three agents in one
  // cell are three) and every swap.
  std::uint64_t count = 0;
  // The earliest of them by earlier(); none when the count is 0.
  std::optional<Conflict> first;
};

class ConflictScanner {
 public:
  // A scanner for plans on a grid of `cell_count` cells; it can scan any number of plans.
  explicit ConflictScanner(std::size_t cell_count);

  // The plan's conflicts. Preconditions: every path non-empty, every cell < cell_count. A null pointer is not a path.
  ConflictScan scan(std::span<const Path* const> plan);
  ConflictScan scan(const Plan& plan);

 private:
  static constexpr std::uint32_t kNone = 0xFFFF'FFFFU;
  // By cell: the epoch of the time step whose list `head_` holds (a cell with an older epoch is empty now). A new
  // epoch per time step and per scan, so nothing is ever cleared but on the counter's wrap.
  std::vector<std::uint32_t> epoch_of_;
  std::vector<std::uint32_t> head_;  // by cell: the last agent listed there at this time
  std::vector<std::uint32_t> next_;  // by agent: the agent listed in the same cell before it
  std::uint32_t epoch_ = 0;
};

}  // namespace pathfinder
