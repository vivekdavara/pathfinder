#include <gtest/gtest.h>

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

// A solved plan must be valid by the oracle, with the costs the planner reported.
void expect_valid(const Instance& inst, const PlanResult& r) {
  ASSERT_EQ(r.status, PlanStatus::kSolved);
  const PlanCheck check = validate(inst, r.plan);
  ASSERT_TRUE(check.valid()) << describe(check.problems.front(), inst.grid);
  EXPECT_EQ(r.sum_of_costs, check.sum_of_costs);
  EXPECT_EQ(r.makespan, check.makespan);
}

std::uint64_t lower_bound(const Instance& inst) {
  std::uint64_t sum = 0;
  for (const Agent& a : inst.agents) {
    sum += distances(inst.grid, a.goal)[a.start];
  }
  return sum;
}

PlannerLimits node_limit(std::uint64_t nodes) {
  PlannerLimits limits;
  limits.max_nodes = nodes;
  return limits;
}

// Every call that expects an answer still has a limit, so a broken CBS fails a test instead of hanging it.
PlannerLimits bounded() { return node_limit(20000); }

TEST(Cbs, AnInstanceWithNoAgentsIsSolved) {
  const Instance inst{Grid(3, 3), {}};
  const PlanResult r = cbs(inst, bounded());
  EXPECT_EQ(r.status, PlanStatus::kSolved);
  EXPECT_TRUE(r.plan.empty());
  EXPECT_EQ(r.nodes_generated, 1U);
  EXPECT_EQ(r.nodes_expanded, 0U);
}

TEST(Cbs, AgentsThatNeverMeetKeepTheirShortestPaths) {
  const Grid g(6, 6);
  const Instance inst = instance_of(g, {{{0, 0}, {5, 0}}, {{0, 5}, {5, 5}}, {{2, 2}, {2, 2}}});
  const PlanResult r = cbs(inst, bounded());
  expect_valid(inst, r);
  EXPECT_EQ(r.sum_of_costs, 10U);
  EXPECT_EQ(r.nodes_generated, 1U);  // the root has no conflict
}

// The day-2 corridor where prioritized planning walls agent 1 in: a dead end (2,0)-(3,0)-(4,0) off a 2 x 2 room,
// agent 0 next to it with its goal half-way in, agent 1 farther away with its goal at the end.
Grid corridor_room() {
  return parse_map(
      "type octile\nheight 2\nwidth 5\nmap\n"
      ".....\n"
      "..@@@\n");
}

TEST(Cbs, SolvesTheCorridorPrioritizedPlanningCant) {
  const Grid g = corridor_room();
  const Instance inst = instance_of(g, {{{1, 0}, {3, 0}}, {{0, 1}, {4, 0}}});
  ASSERT_EQ(prioritized_planning(inst).status, PlanStatus::kNoPath);
  const PlanResult r = cbs(inst, bounded());
  expect_valid(inst, r);
  // Agent 1 needs 5 moves. It is on (3,0) at t >= 4, so agent 0, which must end there after agent 1 has passed, gets
  // there at t >= 5; moving aside and following it does that. 5 + 5 = 10, against 7 for the two alone.
  EXPECT_EQ(r.sum_of_costs, 10U);
  EXPECT_EQ(lower_bound(inst), 7U);
  EXPECT_GT(r.nodes_expanded, 0U);
}

TEST(Cbs, AnAgentOnItsGoalStepsAsideAndComesBack) {
  // A corridor (0,0)-(1,0)-(2,0) with a pocket (1,1) under its middle. Agent 0 starts on its goal (1,0); agent 1
  // crosses from (0,0) to (2,0). Agent 0 must leave and return (cost >= 2), agent 1 needs 2 moves: SOC 4, which
  // agent 0 ducking into the pocket at t = 1 and following agent 1 back out at t = 2 achieves.
  const Grid g = parse_map("type octile\nheight 2\nwidth 3\nmap\n...\n@.@\n");
  const Instance inst = instance_of(g, {{{1, 0}, {1, 0}}, {{0, 0}, {2, 0}}});
  const PlanResult r = cbs(inst, bounded());
  expect_valid(inst, r);
  EXPECT_EQ(r.sum_of_costs, 4U);
  EXPECT_EQ(r.makespan, 2U);
  EXPECT_EQ(r.plan[0], (Path{g.id(Point{1, 0}), g.id(Point{1, 1}), g.id(Point{1, 0})}));
}

TEST(Cbs, TwoAgentsThatMustSwapInACorridorRunIntoTheLimits) {
  // Two cells, each agent on the other's goal: no plan exists, but the tree never runs out (each constraint only
  // pushes a move later), so only the limits stop it.
  const Grid g(2, 1);
  const Instance inst = instance_of(g, {{{0, 0}, {1, 0}}, {{1, 0}, {0, 0}}});
  PlanResult r = cbs(inst, node_limit(500));
  EXPECT_EQ(r.status, PlanStatus::kLimit);
  EXPECT_LE(r.nodes_generated, 500U);
  EXPECT_GE(r.nodes_generated, 499U);  // the last split may stop between its two children
  for (const Path& p : r.plan) {
    EXPECT_TRUE(p.empty());
  }
  EXPECT_EQ(r.sum_of_costs, 0U);
  PlannerLimits limits;
  limits.max_expansions = 2000;
  r = cbs(inst, limits);
  EXPECT_EQ(r.status, PlanStatus::kLimit);
  EXPECT_LE(r.expansions, 2000U);
}

TEST(Cbs, ReportsAnUnreachableGoalBeforeSearching) {
  const Grid g = parse_map("type octile\nheight 2\nwidth 3\nmap\n.@.\n.@.\n");
  const Instance inst = instance_of(g, {{{0, 0}, {0, 1}}, {{2, 0}, {0, 0}}, {{0, 1}, {2, 1}}});
  const PlanResult r = cbs(inst, bounded());
  EXPECT_EQ(r.status, PlanStatus::kUnreachableGoal);
  EXPECT_EQ(r.failed_agent, 1U);
  EXPECT_EQ(r.expansions, 0U);
  EXPECT_EQ(r.nodes_generated, 0U);
}

TEST(Cbs, RefusesANegativeOrNaNTimeLimit) {
  const Grid g(3, 3);
  const Instance inst = instance_of(g, {{{0, 0}, {2, 2}}});
  PlannerLimits limits;
  limits.time_limit_seconds = -1;
  EXPECT_THROW(cbs(inst, limits), std::invalid_argument);
  limits.time_limit_seconds = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(cbs(inst, limits), std::invalid_argument);
  limits.time_limit_seconds = std::numeric_limits<double>::infinity();
  EXPECT_EQ(cbs(inst, limits).status, PlanStatus::kSolved);
}

TEST(Cbs, StopsAtTheTimeLimitBeforeTheFirstSearch) {
  const Grid g(50, 50);
  const Instance inst = random_instance(g, 20, 4);
  PlannerLimits limits;
  limits.time_limit_seconds = 1e-9;
  const PlanResult r = cbs(inst, limits);
  EXPECT_EQ(r.status, PlanStatus::kLimit);
  EXPECT_EQ(r.expansions, 0U);
}

TEST(Cbs, TheNodeAndExpansionLimitsAreExact) {
  const Grid g = corridor_room();
  const Instance inst = instance_of(g, {{{1, 0}, {3, 0}}, {{0, 1}, {4, 0}}});
  const PlanResult full = cbs(inst, bounded());
  expect_valid(inst, full);
  ASSERT_GT(full.nodes_generated, 2U);
  // Exactly the nodes and expansions it used are enough; one fewer of either isn't.
  expect_valid(inst, cbs(inst, node_limit(full.nodes_generated)));
  EXPECT_EQ(cbs(inst, node_limit(full.nodes_generated - 1)).status, PlanStatus::kLimit);
  PlannerLimits limits = bounded();
  limits.max_expansions = full.expansions;
  expect_valid(inst, cbs(inst, limits));
  limits.max_expansions = full.expansions - 1;
  EXPECT_EQ(cbs(inst, limits).status, PlanStatus::kLimit);
}

TEST(Cbs, IsDeterministic) {
  const Grid g = warehouse_map(WarehouseLayout{});
  const Instance inst = random_instance(g, 12, 5);
  const PlanResult a = cbs(inst, node_limit(20000));
  expect_valid(inst, a);
  const PlanResult b = cbs(inst, node_limit(20000));
  EXPECT_EQ(a.plan, b.plan);
  EXPECT_EQ(a.nodes_generated, b.nodes_generated);
  EXPECT_EQ(a.expansions, b.expansions);
}

// On random instances: every plan CBS returns is valid, never cheaper than the agents alone, and never more
// expensive than prioritized planning's when both solve (CBS is optimal); CBS solves everything prioritized planning
// does that fits in its node limit.
TEST(Cbs, NeverWorseThanPrioritizedPlanningOnRandomInstances) {
  int both = 0;
  int cheaper = 0;
  int only_cbs = 0;
  int limited = 0;
  for (std::uint64_t seed = 1; seed <= 120; ++seed) {
    const auto side = static_cast<std::int32_t>(5 + seed % 6);
    const Grid g = random_map(side, side, static_cast<double>(seed % 25), seed);
    const Components c = components(g);
    const std::size_t room = c.sizes[c.largest];
    const std::size_t agents = 2 + seed * 5 % std::max<std::size_t>(1, room / 5);
    const Instance inst = random_instance(g, agents, seed * 17);
    const PlanResult pp = prioritized_planning(inst);
    const PlanResult r = cbs(inst, node_limit(5000));
    if (r.status == PlanStatus::kLimit) {
      ++limited;
      continue;
    }
    expect_valid(inst, r);
    EXPECT_GE(r.sum_of_costs, lower_bound(inst));
    if (pp.status == PlanStatus::kSolved) {
      EXPECT_LE(r.sum_of_costs, pp.sum_of_costs) << seed;
      ++both;
      cheaper += r.sum_of_costs < pp.sum_of_costs ? 1 : 0;
    } else {
      ++only_cbs;
    }
    if (HasFailure()) {
      FAIL() << "seed " << seed;
    }
  }
  EXPECT_GT(both, 60);
  EXPECT_GT(cheaper, 5);
  std::cout << "[ cbs      ] both solved " << both << " (CBS cheaper on " << cheaper << "), only CBS " << only_cbs
            << ", CBS at its node limit " << limited << "\n";
}

}  // namespace
}  // namespace pathfinder
