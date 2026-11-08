// The day-3 review (CBS, at ee6d2ef). ReviewD3.* tests failed at ee6d2ef; ReviewD3Clean.* passed there and are kept
// for coverage the suite lacked.

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <functional>
#include <optional>
#include <queue>
#include <unordered_map>
#include <utility>
#include <vector>

#include "pathfinder/distance.hpp"
#include "pathfinder/generate.hpp"
#include "pathfinder/movingai.hpp"
#include "pathfinder/plan.hpp"
#include "pathfinder/planner.hpp"
#include "pathfinder/rng.hpp"

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

PlannerLimits node_limit(std::uint64_t nodes) {
  PlannerLimits limits;
  limits.max_nodes = nodes;
  return limits;
}

// BUG: planner.hpp says max_nodes makes CBS "give up rather than generate more than this many constraint-tree
// nodes", and cbs_test.cpp's TheNodeAndExpansionLimitsAreExact claims "exactly the nodes ... it used are enough".
// cbs() checks may_generate() before each child's low-level search, so a split whose child turns out to have no path
// (and so is never generated) still trips the limit: here the unlimited run solves with N nodes, but with
// max_nodes = N it gives up (kLimit) having generated exactly N, never needing an (N+1)-th. Found by a random search
// in the scratch harness: about 1% of solved tiny instances (48 of ~3,600 over four seeds).
TEST(ReviewD3, MaxNodesEqualToTheNodesAnUnlimitedRunGeneratedIsEnough) {
  const Grid g = parse_map("type octile\nheight 2\nwidth 4\nmap\n....\n.@..\n");
  const Instance inst = instance_of(g, {{{3, 1}, {0, 0}}, {{0, 0}, {3, 1}}, {{2, 1}, {2, 1}}, {{3, 0}, {3, 0}}});
  const PlanResult full = cbs(inst, node_limit(20000));
  ASSERT_EQ(full.status, PlanStatus::kSolved);
  const PlanResult limited = cbs(inst, node_limit(full.nodes_generated));
  EXPECT_EQ(limited.status, PlanStatus::kSolved)
      << "full run: " << full.nodes_generated << " nodes; limited run stopped at " << limited.nodes_generated;
  EXPECT_EQ(limited.plan, full.plan);
}

// The minimum sum of costs, by Dijkstra over joint states (independent of the library's search code): each agent's
// cell, plus a done bit; an agent not done pays 1 a step, an agent on its goal may become done for free and then
// waits for ever. -1: no plan; -2: more than `cap` states.
long long joint_optimum(const Instance& inst, std::size_t cap) {
  const Grid& g = inst.grid;
  const std::size_t k = inst.agents.size();
  const auto cell = [](std::uint64_t s, std::size_t i) { return static_cast<CellId>((s >> (8 * i)) & 0xFF); };
  const auto done = [](std::uint64_t s, std::size_t i) { return ((s >> (56 + i)) & 1U) != 0; };
  const std::uint64_t all = ((std::uint64_t{1} << k) - 1) << 56;
  std::uint64_t s0 = 0;
  for (std::size_t i = 0; i < k; ++i) {
    s0 |= std::uint64_t{inst.agents[i].start} << (8 * i);
  }
  using Item = std::pair<std::uint64_t, std::uint64_t>;
  std::priority_queue<Item, std::vector<Item>, std::greater<>> open;
  std::unordered_map<std::uint64_t, std::uint64_t> best;
  const auto relax = [&](std::uint64_t s, std::uint64_t c) {
    const auto [it, inserted] = best.try_emplace(s, c);
    if (inserted || c < it->second) {
      it->second = c;
      open.emplace(c, s);
    }
  };
  relax(s0, 0);
  std::vector<CellId> from(k);
  std::vector<CellId> to(k);
  while (!open.empty()) {
    const auto [c, s] = open.top();
    open.pop();
    if (best[s] != c) {
      continue;
    }
    if ((s & all) == all) {
      return static_cast<long long>(c);
    }
    if (best.size() > cap) {
      return -2;
    }
    std::uint64_t step = 0;
    for (std::size_t i = 0; i < k; ++i) {
      from[i] = cell(s, i);
      if (!done(s, i)) {
        ++step;
        if (from[i] == inst.agents[i].goal) {
          relax(s | (std::uint64_t{1} << (56 + i)), c);
        }
      }
    }
    const std::function<void(std::size_t)> choose = [&](std::size_t i) {
      if (i == k) {
        for (std::size_t a = 0; a < k; ++a) {
          for (std::size_t b = a + 1; b < k; ++b) {
            if (to[a] == to[b] || (to[a] == from[b] && to[b] == from[a])) {
              return;
            }
          }
        }
        std::uint64_t next = s & all;
        for (std::size_t a = 0; a < k; ++a) {
          next |= std::uint64_t{to[a]} << (8 * a);
        }
        relax(next, c + step);
        return;
      }
      to[i] = from[i];
      choose(i + 1);
      if (done(s, i)) {
        return;
      }
      std::array<CellId, 4> nb{};
      const int n = g.neighbors(from[i], nb);
      for (int j = 0; j < n; ++j) {
        to[i] = nb[static_cast<std::size_t>(j)];
        choose(i + 1);
      }
    };
    choose(0);
  }
  return -1;
}

// CLEAN: CBS's optimum against the joint-state optimum on tiny instances the existing oracle never generates:
// random_scenario never puts an agent on its own goal, so here a third of the instances have goals that are a
// permutation of the starts (every start is someone's goal, fixed points included) and a third have some agents
// starting on their goals; 1 x n corridors and 1-wide maps included, up to 5 agents. Every solved plan must be valid
// with validate()'s costs and the optimum's SOC; a solvable instance must never be kNoPath; an unsolvable one never
// kSolved. (Measured in the scratch harness at -O2: ~3,600 solvable and ~1,700 unsolvable instances over four seeds,
// up to 6 agents, no disagreement.)
TEST(ReviewD3Clean, CbsMatchesAJointStateOptimumWithAgentsOnGoals) {
  SplitMix64 rng(2026);
  int solvable = 0;
  int unsolvable = 0;
  int on_goal = 0;
  for (int round = 0; round < 260; ++round) {
    const auto w = static_cast<std::int32_t>(1 + rng.below(4));
    const auto h = static_cast<std::int32_t>(1 + rng.below(3));
    const Grid g = random_map(w, h, static_cast<double>(rng.below(35)), rng.next());
    const Components comp = components(g);
    if (comp.largest == Components::kNone) {
      continue;
    }
    std::vector<CellId> cells;
    for (CellId c = 0; c < g.cell_count(); ++c) {
      if (g.passable(c) && comp.label[c] == comp.largest) {
        cells.push_back(c);
      }
    }
    if (cells.size() < 2) {
      continue;
    }
    const std::size_t k = 2 + rng.below(std::min<std::size_t>(cells.size(), 5) - 1);
    const auto shuffled = [&rng](std::vector<CellId> v) {
      for (std::size_t i = v.size(); i > 1; --i) {
        std::swap(v[i - 1], v[rng.below(i)]);
      }
      return v;
    };
    const std::vector<CellId> p = shuffled(cells);
    const std::vector<CellId> starts(p.begin(), p.begin() + static_cast<std::ptrdiff_t>(k));
    std::vector<CellId> goals;
    const auto mode = rng.below(3);
    if (mode == 0) {
      goals = shuffled(starts);
    } else {
      const std::vector<CellId> q = shuffled(cells);
      goals.assign(q.begin(), q.begin() + static_cast<std::ptrdiff_t>(k));
      if (mode == 2) {
        for (std::size_t i = 0; i < k; ++i) {
          if (rng.below(3) == 0) {
            const auto it = std::find(goals.begin(), goals.end(), starts[i]);
            if (it != goals.end()) {
              std::swap(*it, goals[i]);
            } else {
              goals[i] = starts[i];
            }
          }
        }
      }
    }
    Instance inst{g, {}};
    for (std::size_t i = 0; i < k; ++i) {
      inst.agents.push_back(Agent{starts[i], goals[i]});
      on_goal += starts[i] == goals[i] ? 1 : 0;
    }
    check_instance(inst);
    const long long best = joint_optimum(inst, 200000);
    if (best == -2) {
      continue;
    }
    const PlanResult r = cbs(inst, node_limit(best >= 0 ? 1500 : 200));
    if (best >= 0) {
      ++solvable;
      ASSERT_NE(r.status, PlanStatus::kNoPath) << "round " << round;
      if (r.status == PlanStatus::kSolved) {
        const PlanCheck check = validate(inst, r.plan);
        ASSERT_TRUE(check.valid()) << "round " << round;
        ASSERT_EQ(r.sum_of_costs, check.sum_of_costs) << "round " << round;
        ASSERT_EQ(r.makespan, check.makespan) << "round " << round;
        ASSERT_EQ(static_cast<long long>(r.sum_of_costs), best) << "round " << round;
      }
    } else {
      ++unsolvable;
      ASSERT_NE(r.status, PlanStatus::kSolved) << "round " << round;
    }
    ASSERT_LE(r.nodes_expanded, r.nodes_generated);
    ASSERT_LE(r.nodes_generated, 1 + 2 * r.nodes_expanded);
  }
  EXPECT_GT(solvable, 100);
  EXPECT_GT(unsolvable, 10);
  EXPECT_GT(on_goal, 50);
}

// CLEAN (kills a mutant the suite missed: dropping plan_agent's clock check before each search). The low-level
// search looks at the clock only before an expansion, and an agent on its goal needs none, so with every agent on its
// goal only CBS's own check before each search can notice an expired limit.
TEST(ReviewD3Clean, AnExpiredTimeLimitStopsCbsWhenNoSearchNeedsAnExpansion) {
  const Grid g(30, 30);
  Instance inst{g, {}};
  for (CellId c = 0; c < 400; c += 2) {
    inst.agents.push_back(Agent{c, c});
  }
  PlannerLimits limits = node_limit(10);
  limits.time_limit_seconds = 1e-9;
  const PlanResult r = cbs(inst, limits);
  EXPECT_EQ(r.status, PlanStatus::kLimit);
  EXPECT_EQ(r.expansions, 0U);
  EXPECT_EQ(cbs(inst, node_limit(10)).status, PlanStatus::kSolved);
}

// CLEAN (pins the documented open-list order, which no test fixed: dropping the fewer-conflicts tie-break
// survived the suite, since every plan stays optimal; reversing the creation-order one survives this too). The
// node counts of ee6d2ef on one instance were 6,259 and 3,533; 766d3da (space-time A*'s goal bound, which changes
// which of several equally short paths the low level returns) made them 6,366 and 3,591, the numbers pinned here.
TEST(ReviewD3Clean, TheOpenListOrderGivesTheseNodeCounts) {
  const Grid g = parse_map("type octile\nheight 2\nwidth 4\nmap\n....\n.@..\n");
  const Instance inst = instance_of(g, {{{3, 1}, {0, 0}}, {{0, 0}, {3, 1}}, {{2, 1}, {2, 1}}, {{3, 0}, {3, 0}}});
  const PlanResult r = cbs(inst, node_limit(20000));
  ASSERT_EQ(r.status, PlanStatus::kSolved);
  EXPECT_EQ(r.nodes_generated, 6366U);
  EXPECT_EQ(r.nodes_expanded, 3591U);
}

}  // namespace
}  // namespace pathfinder
