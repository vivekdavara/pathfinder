#pragma once

// Test helpers for the space-time search: what "a path the table allows" means, checked from the definitions
// (DESIGN.md "The model" and "The planners") rather than from the search's own code.

#include <cstdint>
#include <string>

#include "pathfinder/grid.hpp"
#include "pathfinder/plan.hpp"
#include "pathfinder/reservation.hpp"

namespace pathfinder::testing {

// Empty when `path` goes from start to goal by legal steps, the table allows every cell and move on it, and the
// agent may stay at the goal from the path's end for ever; otherwise the first thing wrong.
inline std::string allowed(const Grid& g, const ReservationTable& table, const Path& path, CellId start, CellId goal) {
  if (path.empty()) {
    return "empty path";
  }
  if (path.front() != start) {
    return "doesn't start at the start";
  }
  if (path.back() != goal) {
    return "doesn't end at the goal";
  }
  for (std::size_t t = 0; t < path.size(); ++t) {
    const auto time = static_cast<std::uint32_t>(t);
    if (path[t] >= g.cell_count() || !g.passable(path[t])) {
      return "off the map or blocked at t = " + std::to_string(t);
    }
    if (!table.vertex_free(path[t], time)) {
      return "on a forbidden cell at t = " + std::to_string(t);
    }
    if (t + 1 < path.size()) {
      if (!g.adjacent_or_same(path[t], path[t + 1])) {
        return "jumps at t = " + std::to_string(t);
      }
      if (path[t] != path[t + 1] && !table.move_free(path[t], path[t + 1], time)) {
        return "makes a forbidden move at t = " + std::to_string(t);
      }
    }
  }
  const auto end = static_cast<std::uint32_t>(path.size() - 1);
  if (table.forbidden_from(goal) || (table.last_vertex_time(goal) && *table.last_vertex_time(goal) > end)) {
    return "can't stay at the goal after t = " + std::to_string(end);
  }
  return "";
}

}  // namespace pathfinder::testing
