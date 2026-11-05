#pragma once

// Plans and the plan checker (DESIGN.md "The model" and "Checking a plan").

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "pathfinder/grid.hpp"
#include "pathfinder/instance.hpp"

namespace pathfinder {

// An agent's cells at t = 0, 1, ..., size() - 1. After the last, it stays where the path ends.
using Path = std::vector<CellId>;
// One path per agent, in the instance's agent order.
using Plan = std::vector<Path>;

// An agent's cost: the smallest t such that the path is at `goal` at t and at every later step (waits at the goal
// after its final arrival are free). Precondition: the path is non-empty and ends at `goal`.
std::uint32_t path_cost(const Path& path, CellId goal);

enum class ProblemKind {
  kWrongPathCount,  // the plan has `agent` paths for `other` agents (nothing else is checked)
  kEmptyPath,       // agent's path has no cells
  kWrongStart,      // agent's path starts at `cell`, not at its start `to`
  kWrongGoal,       // agent's path ends at `cell`, not at its goal `to`
  kOffGrid,         // agent's path has a cell id outside the grid, at `time`
  kBlockedCell,     // agent's path is on a blocked cell at `time`
  kIllegalMove,     // agent's step from `time` to `time` + 1 isn't a wait or a move to a 4-neighbour
  kVertexConflict,  // agents `agent` < `other` are both in `cell` at `time`
  kEdgeConflict,    // `agent` < `other` swap: agent goes `cell` -> `to` while other goes `to` -> `cell`,
                    // between `time` and `time` + 1
};

struct Problem {
  ProblemKind kind = ProblemKind::kWrongPathCount;
  std::size_t agent = 0;
  std::size_t other = 0;  // conflicts: the second agent; kWrongPathCount: the number of agents
  std::uint32_t time = 0;
  CellId cell = 0;  // the cell concerned (for kIllegalMove and kEdgeConflict, the step's from-cell)
  CellId to = 0;    // kIllegalMove and kEdgeConflict: the step's to-cell; kWrongStart, kWrongGoal: the right cell
  friend bool operator==(const Problem&, const Problem&) = default;
};

// A sentence for a person, with cells as (x, y).
std::string describe(const Problem& problem, const Grid& grid);

struct PlanCheck {
  // Everything wrong, in this order: the path count; each agent's path problems (agents in order; each agent's in
  // the order empty, start, goal, then cells and steps by time); vertex conflicts by (time, agent, other); edge
  // conflicts by (time, agent, other).
  std::vector<Problem> problems;
  // Each agent's path_cost, the sum of costs and the makespan: filled only when no path has a problem of its own
  // (the paths are legal even if they conflict).
  std::vector<std::uint32_t> costs;
  std::uint64_t sum_of_costs = 0;
  std::uint32_t makespan = 0;

  bool valid() const { return problems.empty(); }
};

// Checks a plan against an instance by DESIGN.md's rules, reporting everything it finds. Conflicts are looked for
// among the non-empty paths even when some are illegal, with each agent staying at its path's last cell after the
// path ends. Precondition: check_instance(instance) passes.
PlanCheck validate(const Instance& instance, const Plan& plan);

}  // namespace pathfinder
