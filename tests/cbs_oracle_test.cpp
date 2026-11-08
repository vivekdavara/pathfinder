// CBS against a brute-force optimum (DESIGN.md "Testing"). The oracle knows nothing of constraint trees or
// space-time A*: it is Dijkstra's algorithm over joint states of all the agents, built from DESIGN.md's model alone.

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <functional>
#include <iostream>
#include <optional>
#include <queue>
#include <stdexcept>
#include <unordered_map>
#include <vector>

#include "pathfinder/distance.hpp"
#include "pathfinder/generate.hpp"
#include "pathfinder/movingai.hpp"
#include "pathfinder/plan.hpp"
#include "pathfinder/planner.hpp"
#include "pathfinder/rng.hpp"

namespace pathfinder {
namespace {

// The minimum sum of costs of any valid plan, or nullopt if there is none.
//
// A joint state is every agent's cell and whether it is done: a done agent sits on its goal for ever and costs
// nothing more; an agent that isn't done costs 1 per step, wherever it is (its goal included, since it may still
// leave). An agent on its goal may become done at no cost. One step moves every agent that isn't done by a wait or a
// move to a passable neighbour, at once, with no two agents in one cell after it and no two swapping. An agent's cost
// is then the time it became done, which at best is its last arrival on its goal: the plan's cost by DESIGN.md. The
// model doesn't depend on time, so the state space is finite and Dijkstra's algorithm either reaches "all done" or
// runs out of states, proving there is no plan. Cells and agents are 8-bit fields of the 64-bit key (at most 6
// agents, 255 cells).
std::optional<std::uint64_t> brute_force_soc(const Instance& inst) {
  const Grid& g = inst.grid;
  const std::size_t k = inst.agents.size();
  EXPECT_LE(k, 6U);
  EXPECT_LE(g.cell_count(), 255U);
  const auto cell_of = [](std::uint64_t s, std::size_t i) { return static_cast<CellId>((s >> (8 * i)) & 0xFF); };
  const auto done = [](std::uint64_t s, std::size_t i) { return ((s >> (48 + i)) & 1U) != 0; };
  const std::uint64_t all_done = ((std::uint64_t{1} << k) - 1) << 48;
  std::uint64_t start = 0;
  for (std::size_t i = 0; i < k; ++i) {
    start |= std::uint64_t{inst.agents[i].start} << (8 * i);
  }

  using Item = std::pair<std::uint64_t, std::uint64_t>;  // (cost, state)
  std::priority_queue<Item, std::vector<Item>, std::greater<>> open;
  std::unordered_map<std::uint64_t, std::uint64_t> best;
  const auto relax = [&](std::uint64_t s, std::uint64_t cost) {
    const auto [it, inserted] = best.try_emplace(s, cost);
    if (inserted || cost < it->second) {
      it->second = cost;
      open.emplace(cost, s);
    }
  };
  relax(start, 0);
  std::vector<CellId> from(k);
  std::vector<CellId> to(k);
  while (!open.empty()) {
    const auto [cost, s] = open.top();
    open.pop();
    if (cost != best[s]) {
      continue;  // stale
    }
    if ((s & all_done) == all_done) {
      return cost;
    }
    std::uint64_t moving = 0;
    for (std::size_t i = 0; i < k; ++i) {
      from[i] = cell_of(s, i);
      if (!done(s, i)) {
        ++moving;
        if (from[i] == inst.agents[i].goal) {
          relax(s | (std::uint64_t{1} << (48 + i)), cost);  // done from now on, for nothing
        }
      }
    }
    // Every joint step: agent i's choices are a wait and its passable neighbours (a done agent only waits).
    const std::function<void(std::size_t)> choose = [&](std::size_t i) {
      if (i == k) {
        for (std::size_t a = 0; a < k; ++a) {
          for (std::size_t b = a + 1; b < k; ++b) {
            if (to[a] == to[b] || (to[a] == from[b] && to[b] == from[a] && from[a] != from[b])) {
              return;  // a vertex conflict, or a swap
            }
          }
        }
        std::uint64_t next = s & all_done;
        for (std::size_t a = 0; a < k; ++a) {
          next |= std::uint64_t{to[a]} << (8 * a);
        }
        relax(next, cost + moving);
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
  return std::nullopt;
}

TEST(CbsOracle, TheOracleAgreesWithHandDerivedOptima) {
  // The corridor of cbs_test.cpp (SOC 10), the pocket (SOC 4), and the two-cell swap (no plan).
  const Grid corridor = parse_map("type octile\nheight 2\nwidth 5\nmap\n.....\n..@@@\n");
  EXPECT_EQ(brute_force_soc(Instance{corridor, {{corridor.id(Point{1, 0}), corridor.id(Point{3, 0})},
                                                {corridor.id(Point{0, 1}), corridor.id(Point{4, 0})}}}),
            10U);
  const Grid pocket = parse_map("type octile\nheight 2\nwidth 3\nmap\n...\n@.@\n");
  EXPECT_EQ(brute_force_soc(Instance{pocket, {{1, 1}, {0, 2}}}), 4U);
  EXPECT_EQ(brute_force_soc(Instance{Grid(2, 1), {{0, 1}, {1, 0}}}), std::nullopt);
  // A rotation around a 2 x 2 square is allowed: each agent one move, SOC 4.
  EXPECT_EQ(brute_force_soc(Instance{Grid(2, 2), {{0, 1}, {1, 3}, {3, 2}, {2, 0}}}), 4U);
}

// Random tiny instances: CBS's sum of costs must be the oracle's exactly; where the oracle proves there is no plan,
// CBS must not return one (it stops at a limit, or runs out of tree, kNoPath, which only an unsolvable instance may
// cause). Prioritized planning, when it solves, can't beat the oracle either.
TEST(CbsOracle, CbsFindsTheOptimumOnTinyInstances) {
  SplitMix64 rng(3);
  int solvable = 0;
  int unsolvable = 0;
  int cbs_limited = 0;
  int exhausted = 0;
  int pp_worse = 0;
  int pp_failed = 0;
  std::uint64_t most_nodes = 0;
  std::vector<std::uint64_t> nodes;
  // Node limits: 3,000 where the oracle found a plan (on these instances CBS needed a median of 3 nodes, 60 at the
  // 90th percentile and 18,894 at the 99th; a few need over 200,000), 300 where it proved there is none (CBS can't
  // prove that here: it never ran out of tree, so it would use the whole limit).
  PlannerLimits limits;
  for (int round = 0; round < 1500; ++round) {
    const auto w = static_cast<std::int32_t>(2 + rng.below(3));
    const auto h = static_cast<std::int32_t>(2 + rng.below(3));
    const Grid g = random_map(w, h, static_cast<double>(rng.below(35)), rng.next());
    const Components c = components(g);
    if (c.largest == Components::kNone) {
      continue;
    }
    const std::size_t room = c.sizes[c.largest];
    if (room < 2) {
      continue;
    }
    const std::size_t agents = 2 + rng.below(std::min<std::uint64_t>(room - 1, w * h <= 9 ? 3 : 2));
    Scenario scen;
    try {
      scen = random_scenario(g, agents, rng.next(), "t.map");
    } catch (const std::invalid_argument&) {
      continue;  // no room for that many agents
    }
    const Instance inst = make_instance(g, scen, agents);
    const std::optional<std::uint64_t> best = brute_force_soc(inst);
    limits.max_nodes = best ? 3000 : 300;
    const PlanResult r = cbs(inst, limits);
    const PlanResult pp = prioritized_planning(inst);
    if (best) {
      ++solvable;
      if (r.status == PlanStatus::kLimit) {
        // (Checked here as well as at the end, so a CBS that has stopped converging fails in seconds, not minutes.)
        ASSERT_LT(++cbs_limited, 60) << "round " << round;
      } else {
        ASSERT_EQ(r.status, PlanStatus::kSolved) << "round " << round;
        const PlanCheck check = validate(inst, r.plan);
        ASSERT_TRUE(check.valid()) << "round " << round;
        ASSERT_EQ(r.sum_of_costs, *best) << "round " << round;
        most_nodes = std::max(most_nodes, r.nodes_generated);
        nodes.push_back(r.nodes_generated);
      }
      if (pp.status == PlanStatus::kSolved) {
        ASSERT_GE(pp.sum_of_costs, *best) << "round " << round;
        pp_worse += pp.sum_of_costs > *best ? 1 : 0;
      } else {
        ++pp_failed;
      }
    } else {
      ++unsolvable;
      ASSERT_NE(r.status, PlanStatus::kSolved) << "round " << round;
      ASSERT_NE(pp.status, PlanStatus::kSolved) << "round " << round;
      exhausted += r.status == PlanStatus::kNoPath ? 1 : 0;
    }
  }
  EXPECT_GT(solvable, 1000);
  EXPECT_GT(unsolvable, 20);
  EXPECT_LT(cbs_limited, solvable / 25);  // 32 of 1,148 at 3,000 nodes when this was written
  std::cout << "[ oracle   ] solvable " << solvable << " (CBS at its node limit on " << cbs_limited
            << "; prioritized planning worse on " << pp_worse << ", failed on " << pp_failed << "), unsolvable "
            << unsolvable << " (CBS ran out of tree on " << exhausted << "); most nodes on a solved one " << most_nodes
            << "\n";
  std::sort(nodes.begin(), nodes.end());
  if (!nodes.empty()) {
    std::cout << "[ oracle   ] nodes: median " << nodes[nodes.size() / 2] << ", 90th " << nodes[nodes.size() * 9 / 10]
              << ", 99th " << nodes[nodes.size() * 99 / 100] << "\n";
  }
}

}  // namespace
}  // namespace pathfinder
