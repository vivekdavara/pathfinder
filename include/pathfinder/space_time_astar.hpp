#pragma once

// Space-time A* (DESIGN.md "The planners"): the shortest path for one agent that avoids everything a reservation
// table forbids, ending at its goal at a time after which nothing forbids the goal.

#include <chrono>
#include <cstdint>
#include <optional>

#include "pathfinder/grid.hpp"
#include "pathfinder/heuristic.hpp"
#include "pathfinder/plan.hpp"
#include "pathfinder/reservation.hpp"

namespace pathfinder {

struct SearchLimits {
  // Give up after expanding this many states; nullopt: no limit. 0 allows none, so only a search whose start passes
  // the goal test succeeds (prioritized planning hands an agent whatever budget the agents before it left, and an
  // agent that starts on its goal needs no expansion: the day-2 review).
  std::optional<std::uint64_t> max_expansions;
  // Give up once the clock passes this (checked every 1,024 expansions, and before the first).
  std::optional<std::chrono::steady_clock::time_point> deadline;
};

enum class SearchStatus {
  kFound,   // `path` is a shortest path
  kNoPath,  // there is none: the search ran out of states (it always does, in finite time)
  kLimit,   // a limit stopped the search first
};

struct SearchResult {
  SearchStatus status = SearchStatus::kNoPath;
  // When found: the agent's cell at t = 0, 1, ..., arrival; it starts at the start and ends at the goal, every step
  // is a wait or a move to a passable 4-neighbour, the table allows every cell and move on it, and nothing forbids
  // the goal after the arrival. Its length is the smallest such arrival + 1. Empty otherwise.
  Path path;
  std::uint64_t expansions = 0;  // states taken off the open list and expanded
  std::uint64_t generated = 0;   // states pushed onto it
};

// The shortest path from `start` to h.goal() on `grid` that `table` allows. `h` must be h.goal()'s table on `grid`;
// `table` must be for `grid`. States are (cell, time); from table.horizon() + 1 on nothing changes with time, so
// every later time of a cell is one state, which keeps the state space finite: (horizon + 2) x passable cells at
// most. Ties on f = t + h are broken towards the larger t, then the earlier generated state, so the same inputs give
// the same path everywhere. Throws std::invalid_argument if the table was made for a grid of another size.
// Precondition: start < grid.cell_count().
SearchResult space_time_astar(const Grid& grid, CellId start, const DistanceTable& h, const ReservationTable& table,
                              const SearchLimits& limits = {});

// The same, building goal's distance table first (for one-off searches; planners keep their tables).
// Preconditions: start and goal < grid.cell_count().
SearchResult space_time_astar(const Grid& grid, CellId start, CellId goal, const ReservationTable& table,
                              const SearchLimits& limits = {});

}  // namespace pathfinder
