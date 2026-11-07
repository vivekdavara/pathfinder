#include <algorithm>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "pathfinder/distance.hpp"
#include "pathfinder/heuristic.hpp"
#include "pathfinder/planner.hpp"
#include "pathfinder/reservation.hpp"
#include "pathfinder/space_time_astar.hpp"

namespace pathfinder {

namespace {

constexpr double kNoTimeLimitFrom = 1e9;

std::vector<std::size_t> checked_order(const std::vector<std::size_t>& order, std::size_t agents) {
  if (order.empty()) {
    std::vector<std::size_t> identity(agents);
    for (std::size_t i = 0; i < agents; ++i) {
      identity[i] = i;
    }
    return identity;
  }
  if (order.size() != agents) {
    throw std::invalid_argument("the order names " + std::to_string(order.size()) + " agents, not " +
                                std::to_string(agents));
  }
  std::vector<char> seen(agents, 0);
  for (const std::size_t a : order) {
    if (a >= agents || seen[a] != 0) {
      throw std::invalid_argument("the order isn't a permutation of the agents (at agent " + std::to_string(a) + ")");
    }
    seen[a] = 1;
  }
  return order;
}

}  // namespace

PlanResult prioritized_planning(const Instance& instance, const PlannerLimits& limits,
                                const std::vector<std::size_t>& order_in) {
  const auto started = std::chrono::steady_clock::now();
  const Grid& grid = instance.grid;
  const std::size_t k = instance.agents.size();
  const std::vector<std::size_t> order = checked_order(order_in, k);
  if (!(limits.time_limit_seconds >= 0)) {  // also NaN
    throw std::invalid_argument("the time limit must be a number of seconds, 0 or more");
  }
  PlanResult result;
  result.plan.assign(k, Path{});
  const auto finish = [&](PlanStatus status, std::size_t failed) {
    result.status = status;
    result.failed_agent = failed;
    if (status != PlanStatus::kSolved) {
      result.sum_of_costs = 0;
      result.makespan = 0;
    }
    result.seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
    return result;
  };

  // An agent that can't reach its goal alone makes the instance unsolvable for any planner: say so before searching.
  const Components pieces = components(grid);
  for (std::size_t i = 0; i < k; ++i) {
    if (pieces.label[instance.agents[i].start] != pieces.label[instance.agents[i].goal]) {
      return finish(PlanStatus::kUnreachableGoal, i);
    }
  }

  SearchLimits search;
  // (A limit of a billion seconds or more, infinity included, is no limit: converting it to the clock's integer
  // ticks could overflow.)
  if (limits.time_limit_seconds > 0 && limits.time_limit_seconds < kNoTimeLimitFrom) {
    search.deadline = started + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                    std::chrono::duration<double>(limits.time_limit_seconds));
  }
  const auto index = std::make_shared<const PassableIndex>(grid);
  ReservationTable table(grid);
  for (const std::size_t i : order) {
    const Agent& a = instance.agents[i];
    if (limits.max_expansions != 0) {
      if (result.expansions >= limits.max_expansions) {
        return finish(PlanStatus::kLimit, i);
      }
      search.max_expansions = limits.max_expansions - result.expansions;
    }
    const DistanceTable h(grid, index, a.goal);  // this agent's turn only
    SearchResult found = space_time_astar(grid, a.start, h, table, search);
    result.expansions += found.expansions;
    if (found.status != SearchStatus::kFound) {
      return finish(found.status == SearchStatus::kLimit ? PlanStatus::kLimit : PlanStatus::kNoPath, i);
    }
    table.reserve_path(found.path);
    const std::uint32_t cost = path_cost(found.path, a.goal);
    result.sum_of_costs += cost;
    result.makespan = std::max(result.makespan, cost);
    result.plan[i] = std::move(found.path);
  }
  return finish(PlanStatus::kSolved, 0);
}

}  // namespace pathfinder
