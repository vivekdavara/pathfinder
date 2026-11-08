#pragma once

// What every planner does the same way before it searches (planner.hpp): the time limit turned into a deadline, and
// the check that each agent can reach its goal at all.

#include <chrono>
#include <cstddef>
#include <optional>
#include <stdexcept>

#include "pathfinder/distance.hpp"
#include "pathfinder/instance.hpp"
#include "pathfinder/planner.hpp"

namespace pathfinder::detail {

// The deadline for limits.time_limit_seconds counted from `started`; none for 0 or a billion seconds or more
// (converting those to the clock's integer ticks could overflow). Throws std::invalid_argument for a negative or NaN
// limit.
inline std::optional<std::chrono::steady_clock::time_point> deadline_for(
    const PlannerLimits& limits, std::chrono::steady_clock::time_point started) {
  constexpr double kNoTimeLimitFrom = 1e9;
  if (!(limits.time_limit_seconds >= 0)) {  // also NaN
    throw std::invalid_argument("the time limit must be a number of seconds, 0 or more");
  }
  if (limits.time_limit_seconds > 0 && limits.time_limit_seconds < kNoTimeLimitFrom) {
    return started + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                         std::chrono::duration<double>(limits.time_limit_seconds));
  }
  return std::nullopt;
}

// The lowest-numbered agent whose goal is in another connected piece than its start (no planner can solve such an
// instance), from one components() pass; none if every agent can reach its goal alone.
inline std::optional<std::size_t> first_unreachable_agent(const Instance& instance) {
  const Components pieces = components(instance.grid);
  for (std::size_t i = 0; i < instance.agents.size(); ++i) {
    if (pieces.label[instance.agents[i].start] != pieces.label[instance.agents[i].goal]) {
      return i;
    }
  }
  return std::nullopt;
}

}  // namespace pathfinder::detail
