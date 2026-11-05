#include "pathfinder/plan.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <tuple>

#include "pathfinder/generate.hpp"
#include "pathfinder/rng.hpp"

namespace pathfinder {
namespace {

// y0: . . . .
// y1: . @ . .
// y2: . . . .
Grid small() {
  Grid g(4, 3);
  g.set_passable(Point{1, 1}, false);
  return g;
}

CellId at(const Grid& g, std::int32_t x, std::int32_t y) { return g.id(Point{x, y}); }

std::vector<ProblemKind> kinds(const PlanCheck& check) {
  std::vector<ProblemKind> out;
  for (const auto& p : check.problems) {
    out.push_back(p.kind);
  }
  return out;
}

TEST(PathCost, IsTheFinalArrivalTime) {
  EXPECT_EQ(path_cost({5, 6, 7}, 7), 2U);
  EXPECT_EQ(path_cost({5, 6, 7, 7, 7}, 7), 2U);  // waiting at the goal afterwards is free
  EXPECT_EQ(path_cost({7}, 7), 0U);
  EXPECT_EQ(path_cost({7, 7}, 7), 0U);
  EXPECT_EQ(path_cost({7, 6, 7}, 7), 2U);  // leaving the goal and coming back isn't
  EXPECT_EQ(path_cost({5, 5, 6, 7}, 7), 3U);  // waiting elsewhere isn't
  EXPECT_EQ(path_cost({5, 7, 6, 7, 7}, 7), 3U);
}

TEST(Validate, AcceptsAValidPlanAndComputesItsCosts) {
  const Grid g = small();
  const Instance inst{g, {Agent{at(g, 0, 0), at(g, 3, 0)}, Agent{at(g, 0, 2), at(g, 0, 0)}}};
  // Agent 0 goes along the top; agent 1 goes up the left side behind it, waiting once.
  const Plan plan{{at(g, 0, 0), at(g, 1, 0), at(g, 2, 0), at(g, 3, 0)},
                  {at(g, 0, 2), at(g, 0, 2), at(g, 0, 1), at(g, 0, 0), at(g, 0, 0)}};
  const PlanCheck check = validate(inst, plan);
  EXPECT_TRUE(check.valid()) << describe(check.problems.front(), g);
  EXPECT_EQ(check.costs, (std::vector<std::uint32_t>{3, 3}));
  EXPECT_EQ(check.sum_of_costs, 6U);
  EXPECT_EQ(check.makespan, 3U);
}

TEST(Validate, AnEmptyInstanceHasAnEmptyValidPlan) {
  const PlanCheck check = validate(Instance{small(), {}}, Plan{});
  EXPECT_TRUE(check.valid());
  EXPECT_TRUE(check.costs.empty());
  EXPECT_EQ(check.sum_of_costs, 0U);
  EXPECT_EQ(check.makespan, 0U);
}

TEST(Validate, AWrongPathCountIsTheOnlyProblemReported) {
  const Grid g = small();
  const Instance inst{g, {Agent{0, 3}, Agent{8, 11}}};
  const PlanCheck check = validate(inst, Plan{{0, 9, 9}});
  ASSERT_EQ(check.problems.size(), 1U);
  EXPECT_EQ(check.problems[0], (Problem{ProblemKind::kWrongPathCount, 1, 2, 0, 0, 0}));
  EXPECT_EQ(describe(check.problems[0], g), "the plan has 1 paths for 2 agents");
}

TEST(Validate, ReportsEmptyPathsWrongStartsAndWrongGoals) {
  const Grid g = small();
  const Instance inst{g, {Agent{at(g, 0, 0), at(g, 2, 0)}, Agent{at(g, 3, 2), at(g, 3, 0)}, Agent{8, 9}}};
  const Plan plan{{at(g, 1, 0), at(g, 2, 0)}, {at(g, 3, 2), at(g, 3, 1)}, {}};
  const PlanCheck check = validate(inst, plan);
  ASSERT_EQ(kinds(check), (std::vector{ProblemKind::kWrongStart, ProblemKind::kWrongGoal, ProblemKind::kEmptyPath}));
  EXPECT_EQ(check.problems[0], (Problem{ProblemKind::kWrongStart, 0, 0, 0, at(g, 1, 0), at(g, 0, 0)}));
  EXPECT_EQ(check.problems[1], (Problem{ProblemKind::kWrongGoal, 1, 0, 1, at(g, 3, 1), at(g, 3, 0)}));
  EXPECT_EQ(check.problems[2].agent, 2U);
  EXPECT_EQ(describe(check.problems[0], g), "agent 0's path starts at (1, 0), not at its start (0, 0)");
  EXPECT_EQ(describe(check.problems[1], g), "agent 1's path ends at (3, 1), not at its goal (3, 0)");
  EXPECT_EQ(describe(check.problems[2], g), "agent 2's path is empty");
  EXPECT_TRUE(check.costs.empty());  // not computed when a path is illegal
}

TEST(Validate, ReportsCellsOffTheGridBlockedCellsAndJumps) {
  const Grid g = small();
  const Instance inst{g, {Agent{at(g, 0, 0), at(g, 2, 2)}}};
  // (0,0) -> (1,1) blocked and diagonal -> (1,2) -> cell 99 -> (2,2)
  const Plan plan{{at(g, 0, 0), at(g, 1, 1), at(g, 1, 2), 99, at(g, 2, 2)}};
  const PlanCheck check = validate(inst, plan);
  ASSERT_EQ(kinds(check), (std::vector{ProblemKind::kIllegalMove, ProblemKind::kBlockedCell, ProblemKind::kOffGrid}));
  EXPECT_EQ(check.problems[0], (Problem{ProblemKind::kIllegalMove, 0, 0, 0, at(g, 0, 0), at(g, 1, 1)}));
  EXPECT_EQ(check.problems[1], (Problem{ProblemKind::kBlockedCell, 0, 0, 1, at(g, 1, 1), 0}));
  EXPECT_EQ(check.problems[2], (Problem{ProblemKind::kOffGrid, 0, 0, 3, 99, 0}));
  EXPECT_EQ(describe(check.problems[0], g), "agent 0 jumps from (0, 0) to (1, 1) between t=0 and t=1");
  EXPECT_EQ(describe(check.problems[1], g), "agent 0 is on the blocked cell (1, 1) at t=1");
  EXPECT_EQ(describe(check.problems[2], g), "agent 0 is at cell 99 (outside the grid) at t=3");
}

TEST(Validate, AStepAcrossARowsEndIsAJumpThoughTheIdsAreConsecutive) {
  const Grid g = small();
  const Instance inst{g, {Agent{at(g, 3, 0), at(g, 0, 1)}}};
  const PlanCheck check = validate(inst, Plan{{at(g, 3, 0), at(g, 0, 1)}});  // ids 3 and 4
  ASSERT_EQ(kinds(check), (std::vector{ProblemKind::kIllegalMove}));
  const PlanCheck two = validate(Instance{g, {Agent{0, 2}}}, Plan{{0, 2}});  // two cells in one step
  EXPECT_EQ(kinds(two), (std::vector{ProblemKind::kIllegalMove}));
}

TEST(Validate, ReportsVertexConflictsForEveryPairInACell) {
  const Grid g(3, 3);
  const CellId centre = g.id(Point{1, 1});
  const Instance inst{g, {Agent{1, 7}, Agent{3, 5}, Agent{5, 3}}};
  const Plan plan{{1, centre, 7}, {3, centre, 5}, {5, centre, 3}};
  const PlanCheck check = validate(inst, plan);
  ASSERT_EQ(check.problems.size(), 3U);
  EXPECT_EQ(check.problems[0], (Problem{ProblemKind::kVertexConflict, 0, 1, 1, centre, 0}));
  EXPECT_EQ(check.problems[1], (Problem{ProblemKind::kVertexConflict, 0, 2, 1, centre, 0}));
  EXPECT_EQ(check.problems[2], (Problem{ProblemKind::kVertexConflict, 1, 2, 1, centre, 0}));
  EXPECT_EQ(describe(check.problems[0], g), "agents 0 and 1 are both at (1, 1) at t=1");
  // The paths are legal, so the costs are there even though the plan isn't valid.
  EXPECT_EQ(check.costs, (std::vector<std::uint32_t>{2, 2, 2}));
}

TEST(Validate, AnAgentThatHasFinishedStillOccupiesItsGoal) {
  const Grid g(5, 1);
  // Agent 0 reaches (2, 0) at t=1 and stays; agent 1 crosses it at t=2.
  const Instance inst{g, {Agent{1, 2}, Agent{0, 4}}};
  const Plan plan{{1, 2}, {0, 0, 1, 2, 3, 4}};
  const PlanCheck check = validate(inst, plan);
  ASSERT_EQ(check.problems.size(), 1U);
  EXPECT_EQ(check.problems[0], (Problem{ProblemKind::kVertexConflict, 0, 1, 3, 2, 0}));
  // An agent already at its goal with a one-cell path is in the way too.
  const Instance parked{g, {Agent{2, 2}, Agent{0, 4}}};
  const PlanCheck crossed = validate(parked, Plan{{2}, {0, 1, 2, 3, 4}});
  ASSERT_EQ(crossed.problems.size(), 1U);
  EXPECT_EQ(crossed.problems[0], (Problem{ProblemKind::kVertexConflict, 0, 1, 2, 2, 0}));
  EXPECT_EQ(crossed.costs, (std::vector<std::uint32_t>{0, 4}));
}

TEST(Validate, ReportsSwapsButAllowsFollowingAndRotations) {
  const Grid g(4, 1);
  const Instance swap{g, {Agent{1, 2}, Agent{2, 1}}};
  const PlanCheck swapped = validate(swap, Plan{{1, 2}, {2, 1}});
  ASSERT_EQ(swapped.problems.size(), 1U);
  EXPECT_EQ(swapped.problems[0], (Problem{ProblemKind::kEdgeConflict, 0, 1, 0, 1, 2}));
  EXPECT_EQ(describe(swapped.problems[0], g), "agents 0 and 1 swap (1, 0) and (2, 0) between t=0 and t=1");
  // Following: agent 1 moves into the cell agent 0 leaves in the same step.
  const Instance follow{g, {Agent{1, 3}, Agent{0, 2}}};
  EXPECT_TRUE(validate(follow, Plan{{1, 2, 3}, {0, 1, 2}}).valid());
  // Four agents rotating around a 2x2 block of cells.
  const Grid square(2, 2);
  const Instance rotate{square, {Agent{0, 1}, Agent{1, 3}, Agent{3, 2}, Agent{2, 0}}};
  EXPECT_TRUE(validate(rotate, Plan{{0, 1}, {1, 3}, {3, 2}, {2, 0}}).valid());
}

TEST(Validate, OrdersProblemsAsDocumented) {
  const Grid g(4, 2);
  // Agents 2 and 3 swap at t=0 and agents 0 and 1 meet at t=1; agent 1 then jumps diagonally to its goal.
  const Instance inst{g, {Agent{0, 1}, Agent{2, 6}, Agent{4, 5}, Agent{5, 4}}};
  const Plan plan{{0, 1}, {2, 1, 6}, {4, 5}, {5, 4}};
  const PlanCheck check = validate(inst, plan);
  ASSERT_EQ(check.problems.size(), 3U);
  EXPECT_EQ(check.problems[0], (Problem{ProblemKind::kIllegalMove, 1, 0, 1, 1, 6}));
  EXPECT_EQ(check.problems[1], (Problem{ProblemKind::kVertexConflict, 0, 1, 1, 1, 0}));
  EXPECT_EQ(check.problems[2], (Problem{ProblemKind::kEdgeConflict, 2, 3, 0, 4, 5}));
}

// The same conflicts by the most direct method there is: every pair of agents at every time.
std::vector<Problem> pairwise_conflicts(const Plan& plan) {
  std::size_t longest = 0;
  for (const auto& p : plan) {
    longest = std::max(longest, p.size());
  }
  const auto pos = [&](std::size_t i, std::size_t t) { return plan[i][std::min(t, plan[i].size() - 1)]; };
  std::vector<Problem> vertex;
  std::vector<Problem> edge;
  for (std::size_t t = 0; t < longest; ++t) {
    for (std::size_t a = 0; a < plan.size(); ++a) {
      for (std::size_t b = a + 1; b < plan.size(); ++b) {
        const auto time = static_cast<std::uint32_t>(t);
        if (pos(a, t) == pos(b, t)) {
          vertex.push_back(Problem{ProblemKind::kVertexConflict, a, b, time, pos(a, t), 0});
        }
        if (t + 1 < longest && pos(a, t) != pos(a, t + 1) && pos(a, t) == pos(b, t + 1) &&
            pos(a, t + 1) == pos(b, t)) {
          edge.push_back(Problem{ProblemKind::kEdgeConflict, a, b, time, pos(a, t), pos(a, t + 1)});
        }
      }
    }
  }
  vertex.insert(vertex.end(), edge.begin(), edge.end());
  return vertex;
}

TEST(Validate, FindsTheSameConflictsAsAPairwiseCheckOnRandomWalks) {
  // 300 random instances of random walks (each step a random action, so the paths are legal and meet often);
  // the validator's conflicts must equal the pairwise check's, in the same order.
  SplitMix64 rng(2026);
  std::size_t total = 0;
  for (int round = 0; round < 300; ++round) {
    const Grid g = random_map(6, 5, 15.0, rng.next());
    const auto agents = 2 + rng.below(6);
    const auto scenario_seed = rng.next();
    Scenario s;
    try {
      s = random_scenario(g, agents, scenario_seed, "w.map");
    } catch (const std::invalid_argument&) {
      continue;  // a tiny largest piece
    }
    Plan plan;
    for (const auto& e : s.entries) {
      Path path{g.id(e.start)};
      const auto steps = rng.below(12);
      for (std::uint64_t i = 0; i < steps; ++i) {
        std::array<CellId, 4> next{};
        const int n = g.neighbors(path.back(), next);
        const auto pick = rng.below(static_cast<std::uint64_t>(n) + 1);
        path.push_back(pick == 0 ? path.back() : next[pick - 1]);
      }
      plan.push_back(std::move(path));
    }
    // Let each walk's end be its goal, so only conflicts can be wrong.
    Instance inst{g, {}};
    for (std::size_t i = 0; i < plan.size(); ++i) {
      inst.agents.push_back(Agent{plan[i].front(), plan[i].back()});
    }
    if (std::any_of(inst.agents.begin(), inst.agents.end(), [&](const Agent& a) {
          return std::count_if(inst.agents.begin(), inst.agents.end(),
                               [&](const Agent& b) { return b.goal == a.goal; }) > 1;
        })) {
      continue;  // two walks ended in the same cell: not a valid instance
    }
    const PlanCheck check = validate(inst, plan);
    ASSERT_EQ(check.problems, pairwise_conflicts(plan)) << "round " << round;
    total += check.problems.size();
  }
  EXPECT_GT(total, 100U);  // the walks did conflict
}

}  // namespace
}  // namespace pathfinder
