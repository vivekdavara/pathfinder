#pragma once

// The multi-agent planners (DESIGN.md "The planners"): each takes an instance and limits and returns a plan, or says
// why there is none. Day 2: prioritized planning. Day 3 adds CBS with the same limits and result.

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
  // checked before each agent and every 1,024 expansions of its search.
  double time_limit_seconds = 0;
};

enum class PlanStatus {
  kSolved,       // `plan` is a valid plan
  kUnreachableGoal,  // some agent's goal can't be reached from its start even alone: no planner can solve it
  kNoPath,       // prioritized planning: an agent found no path around the agents planned before it
  kLimit,        // the limits stopped the planner first
};

struct PlanResult {
  PlanStatus status = PlanStatus::kNoPath;
  // Solved: one path per agent, in the instance's order, valid by validate(). Otherwise the paths of the agents
  // planned before the planner stopped, and empty paths for the others.
  Plan plan;
  // Not solved: the agent the planner stopped at (kUnreachableGoal: the lowest-numbered agent that can't reach its goal).
  std::size_t failed_agent = 0;
  // When solved: the sum of costs and the makespan, as validate() computes them.
  std::uint64_t sum_of_costs = 0;
  std::uint32_t makespan = 0;
  std::uint64_t expansions = 0;  // space-time A* expansions, all agents together
  double seconds = 0;            // wall time of the whole call
};

// Prioritized planning, the baseline: the agents one at a time in `order` (default: the instance's order), each by
// space-time A* around the paths of those before it, each then reserved: its cells and moves, and its goal from its
// arrival on (ReservationTable::reserve_path). Fast, but neither complete (an early agent can wall in a later one) nor
// optimal. Each agent's distance table is built when its turn comes and dropped after, so memory holds one table at
// a time. Throws std::invalid_argument unless `order` is empty or a permutation of the agents.
// Precondition: check_instance(instance) passes.
PlanResult prioritized_planning(const Instance& instance, const PlannerLimits& limits = {},
                                const std::vector<std::size_t>& order = {});

}  // namespace pathfinder
