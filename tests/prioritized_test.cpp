#include <gtest/gtest.h>

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

#include "pathfinder/distance.hpp"
#include "pathfinder/generate.hpp"
#include "pathfinder/movingai.hpp"
#include "pathfinder/plan.hpp"
#include "pathfinder/planner.hpp"

namespace pathfinder {
namespace {

Instance instance_of(const Grid& g, std::initializer_list<std::pair<Point, Point>> agents) {
  Instance inst{g, {}};
  for (const auto& [s, t] : agents) {
    inst.agents.push_back(Agent{g.id(s), g.id(t)});
  }
  check_instance(inst);
  return inst;
}

Instance random_instance(const Grid& g, std::size_t agents, std::uint64_t seed) {
  return make_instance(g, random_scenario(g, agents, seed, "test.map"), agents);
}

// Each solved plan must be valid by the oracle, with the costs the planner reported.
void expect_valid(const Instance& inst, const PlanResult& r) {
  ASSERT_EQ(r.status, PlanStatus::kSolved);
  const PlanCheck check = validate(inst, r.plan);
  ASSERT_TRUE(check.valid()) << describe(check.problems.front(), inst.grid);
  EXPECT_EQ(r.sum_of_costs, check.sum_of_costs);
  EXPECT_EQ(r.makespan, check.makespan);
}

// The sum of every agent's distance alone: no plan's sum of costs is below it.
std::uint64_t lower_bound(const Instance& inst) {
  std::uint64_t sum = 0;
  for (const Agent& a : inst.agents) {
    sum += distances(inst.grid, a.goal)[a.start];
  }
  return sum;
}

TEST(PrioritizedPlanning, OneAgentTakesAShortestPath) {
  const Grid g = random_map(20, 20, 25.0, 2);
  const Instance inst = random_instance(g, 1, 5);
  const PlanResult r = prioritized_planning(inst);
  expect_valid(inst, r);
  EXPECT_EQ(r.sum_of_costs, lower_bound(inst));
}

TEST(PrioritizedPlanning, AgentsAlreadyOnTheirGoalsCostNothing) {
  const Grid g(4, 4);
  const Instance inst = instance_of(g, {{{0, 0}, {0, 0}}, {{3, 3}, {3, 3}}, {{1, 2}, {1, 2}}});
  const PlanResult r = prioritized_planning(inst);
  expect_valid(inst, r);
  EXPECT_EQ(r.sum_of_costs, 0U);
  EXPECT_EQ(r.makespan, 0U);
  EXPECT_EQ(r.expansions, 0U);
}

TEST(PrioritizedPlanning, AnInstanceWithNoAgentsIsSolved) {
  const Instance inst{Grid(3, 3), {}};
  const PlanResult r = prioritized_planning(inst);
  EXPECT_EQ(r.status, PlanStatus::kSolved);
  EXPECT_TRUE(r.plan.empty());
}

TEST(PrioritizedPlanning, TwoAgentsCrossingInTheOpenBothGetThrough) {
  const Grid g(5, 5);
  // Head-on along row 2, and a third crossing them down column 2.
  const Instance inst = instance_of(g, {{{0, 2}, {4, 2}}, {{4, 2}, {0, 2}}, {{2, 0}, {2, 4}}});
  const PlanResult r = prioritized_planning(inst);
  expect_valid(inst, r);
  EXPECT_GE(r.sum_of_costs, lower_bound(inst));
}

// A dead-end corridor (4,0)-(3,0)-(2,0) off a 2 x 2 room. Agent 0 is next to the corridor with its goal half-way in;
// agent 1 is farther away with its goal at the end. Agent 0 first parks on its goal before agent 1 can get past it;
// agent 1 first goes in, and agent 0 follows.
Grid corridor_room() {
  return parse_map(
      "type octile\nheight 2\nwidth 5\nmap\n"
      ".....\n"
      "..@@@\n");
}

TEST(PrioritizedPlanning, AnEarlyAgentCanWallInALaterOne) {
  const Grid g = corridor_room();
  const Instance inst = instance_of(g, {{{1, 0}, {3, 0}}, {{0, 1}, {4, 0}}});
  const PlanResult r = prioritized_planning(inst);
  EXPECT_EQ(r.status, PlanStatus::kNoPath);
  EXPECT_EQ(r.failed_agent, 1U);
  // Agent 0's path is kept, agent 1's is empty; the costs are only reported for a solved plan.
  EXPECT_EQ(r.plan[0].back(), g.id(Point{3, 0}));
  EXPECT_TRUE(r.plan[1].empty());
  EXPECT_EQ(r.sum_of_costs, 0U);
  EXPECT_EQ(r.makespan, 0U);
}

TEST(PrioritizedPlanning, TheOtherOrderSolvesTheSameInstance) {
  const Grid g = corridor_room();
  const Instance inst = instance_of(g, {{{1, 0}, {3, 0}}, {{0, 1}, {4, 0}}});
  const PlanResult r = prioritized_planning(inst, {}, {1, 0});
  expect_valid(inst, r);
  EXPECT_EQ(r.plan[1].size(), 6U);  // agent 1 is planned first: its shortest path, 5 moves
}

TEST(PrioritizedPlanning, RefusesAnOrderThatIsntAPermutation) {
  const Grid g(3, 3);
  const Instance inst = instance_of(g, {{{0, 0}, {2, 2}}, {{2, 0}, {0, 2}}});
  EXPECT_THROW(prioritized_planning(inst, {}, {0}), std::invalid_argument);
  EXPECT_THROW(prioritized_planning(inst, {}, {0, 0}), std::invalid_argument);
  EXPECT_THROW(prioritized_planning(inst, {}, {0, 2}), std::invalid_argument);
  EXPECT_THROW(prioritized_planning(inst, {}, {1, 0, 2}), std::invalid_argument);
  EXPECT_NO_THROW(prioritized_planning(inst, {}, {1, 0}));
}

TEST(PrioritizedPlanning, RefusesANegativeOrNaNTimeLimit) {
  const Grid g(3, 3);
  const Instance inst = instance_of(g, {{{0, 0}, {2, 2}}});
  PlannerLimits limits;
  limits.time_limit_seconds = -1;
  EXPECT_THROW(prioritized_planning(inst, limits), std::invalid_argument);
  limits.time_limit_seconds = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(prioritized_planning(inst, limits), std::invalid_argument);
  limits.time_limit_seconds = std::numeric_limits<double>::infinity();  // no limit, not an overflow
  EXPECT_EQ(prioritized_planning(inst, limits).status, PlanStatus::kSolved);
  limits.time_limit_seconds = 1e300;
  EXPECT_EQ(prioritized_planning(inst, limits).status, PlanStatus::kSolved);
}

TEST(PrioritizedPlanning, ReportsAnUnreachableGoalBeforeSearching) {
  const Grid g = parse_map("type octile\nheight 2\nwidth 3\nmap\n.@.\n.@.\n");
  const Instance inst = instance_of(g, {{{0, 0}, {0, 1}}, {{2, 0}, {0, 0}}, {{0, 1}, {2, 1}}});
  const PlanResult r = prioritized_planning(inst);
  EXPECT_EQ(r.status, PlanStatus::kUnreachable);
  EXPECT_EQ(r.failed_agent, 1U);  // the lowest-numbered of agents 1 and 2
  EXPECT_EQ(r.expansions, 0U);
  for (const Path& p : r.plan) {
    EXPECT_TRUE(p.empty());
  }
}

TEST(PrioritizedPlanning, FailsAtFiftyAgentsOnTheWarehouseFloor) {
  const Grid g = warehouse_map(WarehouseLayout{});
  const Instance inst = random_instance(g, 50, 1);
  const PlanResult r = prioritized_planning(inst);
  ASSERT_EQ(r.status, PlanStatus::kNoPath);
  EXPECT_EQ(r.failed_agent, 27U);
  EXPECT_EQ(inst.agents[17].goal, inst.agents[27].start);
  // Every agent before it was planned, validly among themselves.
  Instance first{g, {inst.agents.begin(), inst.agents.begin() + 27}};
  const PlanCheck check = validate(first, Plan(r.plan.begin(), r.plan.begin() + 27));
  EXPECT_TRUE(check.valid());
}

TEST(PrioritizedPlanning, StopsAtTheExpansionLimitAcrossAgents) {
  const Grid g(30, 30);
  const Instance inst = random_instance(g, 20, 3);
  const PlanResult full = prioritized_planning(inst);
  expect_valid(inst, full);
  ASSERT_GT(full.expansions, 100U);
  PlannerLimits limits;
  limits.max_expansions = full.expansions / 2;
  const PlanResult r = prioritized_planning(inst, limits);
  EXPECT_EQ(r.status, PlanStatus::kLimit);
  EXPECT_LE(r.expansions, limits.max_expansions);
  EXPECT_GT(r.failed_agent, 0U);
  // The agents before the one it stopped at have the same paths as in the full run.
  for (std::size_t i = 0; i < r.failed_agent; ++i) {
    EXPECT_EQ(r.plan[i], full.plan[i]) << i;
  }
  for (std::size_t i = r.failed_agent; i < inst.agents.size(); ++i) {
    EXPECT_TRUE(r.plan[i].empty()) << i;
  }
  limits.max_expansions = full.expansions;  // exactly enough
  expect_valid(inst, prioritized_planning(inst, limits));
}

TEST(PrioritizedPlanning, StopsAtTheTimeLimit) {
  const Grid g(200, 200);
  const Instance inst = random_instance(g, 50, 9);
  PlannerLimits limits;
  limits.time_limit_seconds = 1e-9;  // passed before the first agent's first expansion
  const PlanResult r = prioritized_planning(inst, limits);
  EXPECT_EQ(r.status, PlanStatus::kLimit);
  EXPECT_EQ(r.failed_agent, 0U);
  EXPECT_EQ(r.expansions, 0U);
}

TEST(PrioritizedPlanning, EverySolvedPlanIsValidOnRandomInstances) {
  int solved = 0;
  int failed = 0;
  for (std::uint64_t seed = 1; seed <= 150; ++seed) {
    const auto side = static_cast<std::int32_t>(6 + seed % 10);
    const Grid g = random_map(side, side, static_cast<double>(seed % 30), seed);
    const Components c = components(g);
    const std::size_t room = c.sizes[c.largest];
    const std::size_t agents = 1 + seed * 7 % std::max<std::size_t>(1, room / 3);  // up to a third of the room
    const Instance inst = random_instance(g, agents, seed * 31);
    const PlanResult r = prioritized_planning(inst);
    if (r.status == PlanStatus::kSolved) {
      expect_valid(inst, r);
      EXPECT_GE(r.sum_of_costs, lower_bound(inst));
      ++solved;
    } else {
      EXPECT_EQ(r.status, PlanStatus::kNoPath) << seed;  // random_scenario keeps every goal reachable
      EXPECT_TRUE(r.plan[r.failed_agent].empty());
      ++failed;
    }
  }
  EXPECT_GT(solved, 60);
  EXPECT_GT(failed, 10);  // crowded instances, where its incompleteness shows
  std::cout << "[ planned  ] " << solved << " solved, " << failed << " not\n";
}

// The default warehouse floor (27 x 12, 204 free cells): 30 agents are solved on every seed tried (1-3); from 40
// on some seeds fail, as prioritized planning does when the floor fills up (at 50, seed 1, agent 27 starts in a
// corner on agent 17's goal and is walled in by the agents parked before it).
TEST(PrioritizedPlanning, SolvesAWarehouseFloorWithThirtyAgents) {
  const Grid g = warehouse_map(WarehouseLayout{});
  const Instance inst = random_instance(g, 30, 1);
  const PlanResult r = prioritized_planning(inst);
  expect_valid(inst, r);
  const PlanResult again = prioritized_planning(inst);
  EXPECT_EQ(again.plan, r.plan);  // deterministic
  EXPECT_EQ(again.expansions, r.expansions);
}

}  // namespace
}  // namespace pathfinder
