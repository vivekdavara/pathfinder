#pragma once

// The multi-agent planners (DESIGN.md "The planners"): each takes an instance and limits and returns a plan, or says
// why there is none. Prioritized planning (day 2) is fast and incomplete; CBS (day 3) is optimal and can be slow.

#include <cstddef>
#include <cstdint>
#include <vector>

#include "pathfinder/instance.hpp"
#include "pathfinder/plan.hpp"

namespace pathfinder {

struct PlannerLimits {
  // Give up after this many space-time A* expansions over the whole run (0: no limit).
  std::uint64_t max_expansions = 0;
  // Give up after this many seconds of wall time (0, or a billion or more: no limit; negative or NaN: an error),
  // checked before each search and every 1,024 expansions of it.
  double time_limit_seconds = 0;
  // CBS only: give up rather than generate more than this many constraint-tree nodes, the root included (0: no
  // limit). Prioritized planning has no tree and ignores it.
  std::uint64_t max_nodes = 0;
};

enum class PlanStatus {
  kSolved,       // `plan` is a valid plan
  kUnreachableGoal,  // some agent's goal can't be reached from its start even alone: no planner can solve it
  kNoPath,       // prioritized planning: an agent found no path around the agents planned before it; CBS: the
                 // constraint tree ran out of nodes, which proves that no plan exists
  kLimit,        // the limits stopped the planner first
};

struct PlanResult {
  PlanStatus status = PlanStatus::kNoPath;
  // Solved: one path per agent, in the instance's order, valid by validate(). Otherwise, for prioritized planning,
  // the paths of the agents planned before it stopped and empty paths for the others; for CBS, empty paths.
  Plan plan;
  // Not solved: the agent the planner stopped at (kUnreachableGoal: the lowest-numbered agent that can't reach its
  // goal). CBS stops at no agent in particular, so it reports 0 for kNoPath and kLimit.
  std::size_t failed_agent = 0;
  // When solved: the sum of costs and the makespan, as validate() computes them.
  std::uint64_t sum_of_costs = 0;
  std::uint32_t makespan = 0;
  std::uint64_t expansions = 0;  // space-time A* expansions, all agents together
  double seconds = 0;            // wall time of the whole call
  // CBS only (0 for prioritized planning): constraint-tree nodes generated (the root included) and expanded (taken
  // off the open list and split; the solution, taken off last, isn't split).
  std::uint64_t nodes_generated = 0;
  std::uint64_t nodes_expanded = 0;
};

// Prioritized planning, the baseline: the agents one at a time in `order` (default: the instance's order), each by
// space-time A* around the paths of those before it, each then reserved: its cells and moves, and its goal from its
// arrival on (ReservationTable::reserve_path). Fast, but neither complete (an early agent can wall in a later one) nor
// optimal. Each agent's distance table is built when its turn comes and dropped after, so memory holds one table at
// a time. Throws std::invalid_argument unless `order` is empty or a permutation of the agents.
// Precondition: check_instance(instance) passes.
PlanResult prioritized_planning(const Instance& instance, const PlannerLimits& limits = {},
                                const std::vector<std::size_t>& order = {});

// Conflict-based search (Sharon et al., AIJ 2015): a best-first search over a constraint tree, which returns a plan of
// minimum sum of costs. The root plans every agent alone; each node is split on its plan's first conflict
// (ConflictScanner) into two children, each forbidding that conflict to one of its two agents and replanning only
// that agent, by space-time A* with the agent's constraints as its reservation table. Nodes are taken in order of
// sum of costs, then fewer conflicts, then creation; the first with no conflict is the answer. Each agent's distance
// table is built once and kept for the run. On an unsolvable instance whose goals are all reachable (two agents that
// must swap in a dead end) the tree can grow for ever: the limits are what stop it. A tree that runs out of nodes
// proves there is no plan (kNoPath). Precondition: check_instance(instance) passes.
PlanResult cbs(const Instance& instance, const PlannerLimits& limits = {});

}  // namespace pathfinder
