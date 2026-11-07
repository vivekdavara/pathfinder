// The day-2 review (an agent at commit 5089fd9, in its own worktree): the tests in suites named ReviewD2* other than
// ReviewD2Clean each failed on that commit because of the bug their comment names. ReviewD2Clean holds checks that
// passed there (things the review looked at and found right), kept for their coverage.

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "pathfinder/distance.hpp"
#include "pathfinder/generate.hpp"
#include "pathfinder/heuristic.hpp"
#include "pathfinder/instance.hpp"
#include "pathfinder/movingai.hpp"
#include "pathfinder/plan.hpp"
#include "pathfinder/planner.hpp"
#include "pathfinder/reservation.hpp"
#include "pathfinder/rng.hpp"
#include "pathfinder/space_time_astar.hpp"

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

// The earliest arrival the table allows (the agent at the goal at t and free to stay there for ever), trying every
// arrival up to `last`, layer by layer from the table's own answers; nullopt if none by then.
std::optional<std::uint32_t> earliest_arrival(const Grid& g, const ReservationTable& r, CellId start, CellId goal,
                                              std::uint32_t last) {
  const auto can_stay = [&](std::uint32_t t) {
    return !r.forbidden_from(goal) && (!r.last_vertex_time(goal) || *r.last_vertex_time(goal) < t);
  };
  if (!g.passable(start) || !r.vertex_free(start, 0)) {
    return std::nullopt;
  }
  std::vector<char> here(g.cell_count(), 0);
  here[start] = 1;
  std::array<CellId, 4> next{};
  for (std::uint32_t t = 0; t <= last; ++t) {
    if (here[goal] != 0 && can_stay(t)) {
      return t;
    }
    std::vector<char> after(g.cell_count(), 0);
    for (CellId c = 0; c < g.cell_count(); ++c) {
      if (here[c] == 0) {
        continue;
      }
      if (r.vertex_free(c, t + 1)) {
        after[c] = 1;
      }
      const int n = g.neighbors(c, next);
      for (int i = 0; i < n; ++i) {
        const CellId v = next[static_cast<std::size_t>(i)];
        if (r.vertex_free(v, t + 1) && r.move_free(c, v, t)) {
          after[v] = 1;
        }
      }
    }
    here.swap(after);
  }
  return std::nullopt;
}

CellId random_passable(const Grid& g, SplitMix64& rng) {
  for (;;) {
    const auto c = static_cast<CellId>(rng.below(g.cell_count()));
    if (g.passable(c)) {
      return c;
    }
  }
}

Path random_walk(const Grid& g, SplitMix64& rng, CellId from, std::size_t steps) {
  Path p{from};
  std::array<CellId, 4> next{};
  for (std::size_t i = 0; i < steps; ++i) {
    const int n = g.neighbors(p.back(), next);
    const auto pick = rng.below(static_cast<std::uint64_t>(n) + 1);
    p.push_back(pick == static_cast<std::uint64_t>(n) ? p.back() : next[pick]);
  }
  return p;
}

// ---------------------------------------------------------------------------------------------------------------------
// Bugs
// ---------------------------------------------------------------------------------------------------------------------

// src/prioritized.cpp, the budget check before each agent (`if (result.expansions >= limits.max_expansions) return
// finish(PlanStatus::kLimit, i);`): once the budget is used up exactly, the planner gives up on the next agent even
// when that agent's search needs no expansion at all (it starts on its goal and nobody crosses it). space_time_astar
// itself returns kFound for such an agent with any budget (SpaceTimeAStar.StopsAtAPassedDeadlineBeforeExpandingAnything
// says the same of the deadline), and tests/prioritized_test.cpp calls a budget equal to the unlimited run's
// expansions "exactly enough". Here agent 0 walks 3 cells (3 expansions) and agent 1 sits on its goal far away
// (0 expansions): the unlimited run uses 3 expansions, but max_expansions = 3 stops at agent 1 with kLimit, having
// expanded 3, never more than allowed.
TEST(ReviewD2Prioritized, ABudgetEqualToTheUnlimitedRunsExpansionsIsEnough) {
  const Grid g(5, 5);
  const Instance inst = instance_of(g, {{{0, 0}, {3, 0}}, {{4, 4}, {4, 4}}});
  const PlanResult full = prioritized_planning(inst);
  ASSERT_EQ(full.status, PlanStatus::kSolved);
  ASSERT_EQ(full.expansions, 3U);
  PlannerLimits limits;
  limits.max_expansions = full.expansions;
  const PlanResult r = prioritized_planning(inst, limits);
  EXPECT_EQ(r.status, PlanStatus::kSolved) << "stopped at agent " << r.failed_agent << " after " << r.expansions
                                           << " expansions";
  EXPECT_EQ(r.plan, full.plan);
}

// src/prioritized.cpp never looks at the clock itself; only space_time_astar does, and only before an expansion
// (space_time_astar.cpp:124). An agent that starts on its goal with nobody crossing it needs no expansion, so its
// search returns kFound however late it is, and the planner goes on to build the next agent's distance table (a BFS
// over the whole grid). PlannerLimits promises to "give up after this many seconds of wall time"; here, with 0.001 s,
// 80 such agents on a 400 x 400 grid run for one table build each and come back kSolved far past the limit (about
// 1 s in the Debug build). A planner that gives up returns kLimit, which passes however long it took.
TEST(ReviewD2Prioritized, GivesUpAtTheTimeLimitWhenAgentsNeedNoExpansions) {
  const Grid g(400, 400);
  Instance inst{g, {}};
  for (std::int32_t i = 0; i < 80; ++i) {
    const CellId c = g.id(Point{(i * 37) % 400, (i * 101) % 400});
    inst.agents.push_back(Agent{c, c});
  }
  check_instance(inst);
  PlannerLimits limits;
  limits.time_limit_seconds = 0.001;
  const PlanResult r = prioritized_planning(inst, limits);
  EXPECT_TRUE(r.status == PlanStatus::kLimit || r.seconds < 20 * limits.time_limit_seconds)
      << "status " << static_cast<int>(r.status) << " after " << r.seconds << " s with a limit of "
      << limits.time_limit_seconds << " s";
}

// src/heuristic.cpp:26, the guard `index_->size() != grid.passable_count()`: DistanceTable claims to refuse "an index
// that isn't this grid's" (the message, and DistanceTable.RefusesAnIndexOfAnotherGrid), but it compares only the
// number of passable cells. An index of another 4 x 4 grid with the same count (15) but a different blocked cell is
// accepted, and the BFS then reads dist_[PassableIndex::kBlocked] (index 4,294,967,295) for g's cell 15, which the
// other grid blocks: an out-of-bounds read and write, not an exception. Low severity: heuristic.hpp also states it as
// a precondition. (Run in a child process so the crash fails only this test.)
TEST(ReviewD2HeuristicDeathTest, RefusesAnIndexOfAnotherGridWithTheSamePassableCount) {
  GTEST_FLAG_SET(death_test_style, "threadsafe");
  Grid g(4, 4);
  g.set_passable(Point{0, 0}, false);
  Grid other(4, 4);
  other.set_passable(Point{3, 3}, false);
  ASSERT_EQ(g.passable_count(), other.passable_count());
  const auto wrong = std::make_shared<const PassableIndex>(other);
  EXPECT_EXIT(
      {
        try {
          const DistanceTable table(g, wrong, 5);
          (void)table;
        } catch (const std::invalid_argument&) {
          std::_Exit(0);
        }
        std::_Exit(1);
      },
      ::testing::ExitedWithCode(0), "");
}

// ---------------------------------------------------------------------------------------------------------------------
// Clean: checked and found right at 5089fd9
// ---------------------------------------------------------------------------------------------------------------------

// The budget is used up exactly by agent 0 (a found path after exactly 3 expansions); agent 1 needs more. The planner
// must stop with kLimit and never expand more than the budget. (space_time_astar reads max_expansions = 0 as "no
// limit", so the planner must not hand it a remaining budget of 0: deleting the pre-check in prioritized.cpp, or
// turning its >= into >, lets agent 1 search unlimited. The existing suite doesn't notice that mutant.)
TEST(ReviewD2Clean, ABudgetUsedUpExactlyByOneAgentStopsTheNext) {
  const Grid g(5, 5);
  const Instance inst = instance_of(g, {{{0, 0}, {3, 0}}, {{0, 4}, {4, 4}}});
  PlannerLimits limits;
  limits.max_expansions = 3;
  const PlanResult r = prioritized_planning(inst, limits);
  EXPECT_EQ(r.status, PlanStatus::kLimit);
  EXPECT_EQ(r.failed_agent, 1U);
  EXPECT_LE(r.expansions, 3U);
}

// DESIGN.md: ties on f go to the larger t, "then by insertion order". On an empty 3 x 3 grid every shortest path from
// (0, 0) to (2, 2) ties on f and t; neighbours are generated north, east, south, west, so the first-generated state
// at each step is the eastward one and the path is east, east, south, south. (The existing suite checks only the
// length of such paths: reversing the last tie-break, to the later-generated state, survives it.)
TEST(ReviewD2Clean, BreaksTiesOnFAndTByGenerationOrder) {
  const Grid g(3, 3);
  const SearchResult s = space_time_astar(g, 0, 8, ReservationTable(g));
  ASSERT_EQ(s.status, SearchStatus::kFound);
  EXPECT_EQ(s.path, (Path{0, 1, 2, 5, 8}));
}

// reserve_path against validate(): for random pairs of legal paths P (reserved) and Q (checked against the table),
// the table allows Q (every cell, every move, and staying at Q's end for ever) exactly when validate finds no
// conflict between P and Q. This checks the reservation semantics (following allowed, swaps forbidden, the earlier
// agent resting at its end for ever) and the direction encoding at the edges, independently of the search.
TEST(ReviewD2Clean, ReservePathForbidsExactlyWhatValidateCallsAConflict) {
  SplitMix64 rng(51);
  int allowed_count = 0;
  int refused = 0;
  for (int round = 0; round < 40000; ++round) {
    const auto w = static_cast<std::int32_t>(1 + rng.below(5));
    const auto h = static_cast<std::int32_t>(1 + rng.below(5));
    const Grid g = random_map(w, h, static_cast<double>(rng.below(30)), rng.next());
    if (g.passable_count() < 2) {
      continue;
    }
    const Path p = random_walk(g, rng, random_passable(g, rng), rng.below(8));
    const Path q = random_walk(g, rng, random_passable(g, rng), rng.below(8));
    if (p.front() == q.front() || p.back() == q.back()) {
      continue;  // not an instance
    }
    ReservationTable table(g);
    table.reserve_path(p);
    bool ok = true;
    for (std::size_t t = 0; t < q.size() && ok; ++t) {
      const auto time = static_cast<std::uint32_t>(t);
      ok = table.vertex_free(q[t], time);
      if (ok && t + 1 < q.size() && q[t] != q[t + 1]) {
        ok = table.move_free(q[t], q[t + 1], time);
      }
    }
    const auto end = static_cast<std::uint32_t>(q.size() - 1);
    if (ok) {
      ok = !table.forbidden_from(q.back()) &&
           (!table.last_vertex_time(q.back()) || *table.last_vertex_time(q.back()) <= end);
      // The cells after Q's end: Q sits at q.back(); the table must allow it at every later time.
      for (std::uint32_t t = end; ok && t <= table.horizon() + 1; ++t) {
        ok = table.vertex_free(q.back(), t);
      }
    }
    const Instance inst{g, {Agent{p.front(), p.back()}, Agent{q.front(), q.back()}}};
    const PlanCheck check = validate(inst, Plan{p, q});
    ASSERT_EQ(ok, check.valid()) << "round " << round << (check.valid() ? "" : ": " + describe(check.problems.front(), g));
    (ok ? allowed_count : refused) += 1;
  }
  EXPECT_GT(allowed_count, 5000);
  EXPECT_GT(refused, 5000);
}

// Prioritized planning on many tiny random instances, biased towards the interactions that matter: agents starting
// on their goals, starts that are other agents' goals, random orders. Every solved plan is valid with the reported
// costs; every agent's path is the earliest arrival around those before it (by a brute-force oracle over the same
// table); a kNoPath agent really has none; kUnreachableGoal names the lowest-numbered unreachable agent.
TEST(ReviewD2Clean, PrioritizedPlanningMatchesAnOracleAgentByAgent) {
  SplitMix64 rng(2026);
  int solved = 0;
  int nopath = 0;
  int unreachable = 0;
  for (int round = 0; round < 6000; ++round) {
    const auto w = static_cast<std::int32_t>(1 + rng.below(6));
    const auto h = static_cast<std::int32_t>(1 + rng.below(6));
    const Grid g = random_map(w, h, static_cast<double>(rng.below(30)), rng.next());
    const std::size_t passable = g.passable_count();
    if (passable == 0) {
      continue;
    }
    const std::size_t k = 1 + rng.below(std::min<std::size_t>(passable, 6));
    std::vector<CellId> cells;
    for (CellId c = 0; c < g.cell_count(); ++c) {
      if (g.passable(c)) {
        cells.push_back(c);
      }
    }
    // Starts: k distinct cells. Goals: a permutation-ish pick biased to starts (own or others').
    std::vector<CellId> pool = cells;
    std::vector<CellId> starts;
    for (std::size_t i = 0; i < k; ++i) {
      const auto j = rng.below(pool.size());
      starts.push_back(pool[j]);
      pool.erase(pool.begin() + static_cast<std::ptrdiff_t>(j));
    }
    std::vector<CellId> goals;
    std::vector<CellId> goal_pool = cells;
    for (std::size_t i = 0; i < k; ++i) {
      CellId pick = 0;
      const auto mode = rng.below(4);
      const auto own = std::find(goal_pool.begin(), goal_pool.end(), starts[i]);
      if (mode == 0 && own != goal_pool.end()) {
        pick = starts[i];
      } else if (mode == 1) {
        const CellId other = starts[rng.below(k)];
        pick = std::find(goal_pool.begin(), goal_pool.end(), other) != goal_pool.end() ? other
                                                                                       : goal_pool[rng.below(goal_pool.size())];
      } else {
        pick = goal_pool[rng.below(goal_pool.size())];
      }
      goals.push_back(pick);
      goal_pool.erase(std::find(goal_pool.begin(), goal_pool.end(), pick));
    }
    Instance inst{g, {}};
    for (std::size_t i = 0; i < k; ++i) {
      inst.agents.push_back(Agent{starts[i], goals[i]});
    }
    check_instance(inst);
    std::vector<std::size_t> order(k);
    for (std::size_t i = 0; i < k; ++i) {
      order[i] = i;
    }
    for (std::size_t i = k; i > 1; --i) {
      std::swap(order[i - 1], order[rng.below(i)]);
    }
    const PlanResult r = prioritized_planning(inst, {}, order);
    const auto where = ::testing::Message() << "round " << round;
    const std::vector<std::size_t> bad = unreachable_agents(inst);
    if (!bad.empty()) {
      ASSERT_EQ(r.status, PlanStatus::kUnreachableGoal) << where;
      ASSERT_EQ(r.failed_agent, bad.front()) << where;
      ++unreachable;
      continue;
    }
    ASSERT_NE(r.status, PlanStatus::kLimit) << where;
    ASSERT_NE(r.status, PlanStatus::kUnreachableGoal) << where;
    ReservationTable table(g);
    for (const std::size_t a : order) {
      const std::uint32_t bound = table.horizon() + static_cast<std::uint32_t>(passable);
      const auto expected = earliest_arrival(g, table, inst.agents[a].start, inst.agents[a].goal, bound);
      if (r.status == PlanStatus::kNoPath && r.failed_agent == a) {
        ASSERT_FALSE(expected.has_value()) << where << ", agent " << a;
        ASSERT_FALSE(earliest_arrival(g, table, inst.agents[a].start, inst.agents[a].goal, 2 * bound + 10))
            << where;
        ASSERT_TRUE(r.plan[a].empty()) << where;
        break;
      }
      ASSERT_TRUE(expected.has_value()) << where << ", agent " << a;
      ASSERT_EQ(r.plan[a].size(), *expected + 1U) << where << ", agent " << a;
      table.reserve_path(r.plan[a]);
    }
    if (r.status == PlanStatus::kSolved) {
      const PlanCheck check = validate(inst, r.plan);
      ASSERT_TRUE(check.valid()) << where << ": " << describe(check.problems.front(), g);
      ASSERT_EQ(r.sum_of_costs, check.sum_of_costs) << where;
      ASSERT_EQ(r.makespan, check.makespan) << where;
      ++solved;
    } else {
      ++nopath;
    }
  }
  EXPECT_GT(solved, 2000);
  EXPECT_GT(nopath, 200);
  EXPECT_GT(unreachable, 100);
}

}  // namespace
}  // namespace pathfinder
