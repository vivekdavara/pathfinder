// The day-3 second-round review (766d3da's goal bound in space-time A*, d79e431's CBS node limit and clock check).
// ReviewD3b.* tests fail at d79e431; ReviewD3bClean.* pass there and are kept for coverage.

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstdlib>
#include <iostream>
#include <cstdint>
#include <optional>
#include <vector>

#include "pathfinder/distance.hpp"
#include "pathfinder/generate.hpp"
#include "pathfinder/plan.hpp"
#include "pathfinder/planner.hpp"
#include "pathfinder/reservation.hpp"
#include "pathfinder/rng.hpp"
#include "pathfinder/space_time_astar.hpp"

namespace pathfinder {
namespace {

// Earliest arrival by layers of reachable cells (shares nothing with the search), up to `last`.
std::optional<std::uint32_t> oracle_arrival(const Grid& g, const ReservationTable& r, CellId start, CellId goal,
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

bool path_allowed(const Grid& g, const ReservationTable& r, const Path& p, CellId start, CellId goal) {
  if (p.empty() || p.front() != start || p.back() != goal) {
    return false;
  }
  for (std::uint32_t t = 0; t < p.size(); ++t) {
    if (!g.passable(p[t]) || !r.vertex_free(p[t], t)) {
      return false;
    }
    if (t > 0 && p[t] != p[t - 1]) {
      if (std::abs(g.point(p[t]).x - g.point(p[t - 1]).x) + std::abs(g.point(p[t]).y - g.point(p[t - 1]).y) != 1) {
        return false;
      }
      if (!r.move_free(p[t - 1], p[t], t - 1)) {
        return false;
      }
    }
  }
  const auto last = r.last_vertex_time(goal);
  return !r.forbidden_from(goal) && (!last || *last < p.size() - 1);
}

CellId random_passable(const Grid& g, SplitMix64& rng) {
  for (;;) {
    const auto c = static_cast<CellId>(rng.below(g.cell_count()));
    if (g.passable(c)) {
      return c;
    }
  }
}

// Space-time A* against the oracle with entries on the goal at many times: before, at and beyond the horizon of the
// other entries, on goals the agent starts on, alongside move entries, forbid_from walls elsewhere and other agents'
// paths; and several entries on the goal at once.
// `walls`: also forbid_from walls going up at t = 0..2 on up to half the cells (not the goal), where the heuristic is
// far from exact after the horizon and the search re-pushes merged late states with a lower t.
void oracle_rounds(std::uint64_t seed, bool walls, int rounds, int min_found, int min_none, int min_late) {
  SplitMix64 rng(seed);
  int found = 0;
  int none = 0;
  int late = 0;  // the arrival is the goal entry + 1 exactly: the bound was tight
  std::array<CellId, 4> next{};
  for (int round = 0; round < rounds; ++round) {
    const auto w = static_cast<std::int32_t>(1 + rng.below(7));
    const auto h = static_cast<std::int32_t>(1 + rng.below(6));
    const Grid g = random_map(w, h, static_cast<double>(rng.below(40)), rng.next());
    bool any = false;
    for (CellId c = 0; c < g.cell_count(); ++c) {
      any = any || g.passable(c);
    }
    if (!any) {
      continue;
    }
    const CellId start = random_passable(g, rng);
    const CellId goal = rng.below(4) == 0 ? start : random_passable(g, rng);
    ReservationTable r(g);
    const auto walks = rng.below(3);
    for (std::uint64_t i = 0; i < walks; ++i) {
      Path p{random_passable(g, rng)};
      const auto steps = rng.below(12);
      for (std::uint64_t s = 0; s < steps; ++s) {
        const int n = g.neighbors(p.back(), next);
        const auto pick = rng.below(static_cast<std::uint64_t>(n) + 1);
        p.push_back(pick == static_cast<std::uint64_t>(n) ? p.back() : next[pick]);
      }
      r.reserve_path(p);
    }
    const auto loose = rng.below(15);
    for (std::uint64_t i = 0; i < loose; ++i) {
      const CellId c = random_passable(g, rng);
      if (c == goal && rng.below(2) == 0) {
        continue;
      }
      const auto t = static_cast<std::uint32_t>(rng.below(15));
      switch (rng.below(8)) {
        case 0:
          if (c != goal) {
            r.forbid_from(c, t);
          }
          break;
        case 1:
        case 2:
        case 3: {
          const int n = g.neighbors(c, next);
          if (n > 0) {
            r.forbid_move(c, next[rng.below(static_cast<std::uint64_t>(n))], t);
          }
          break;
        }
        default:
          r.forbid_vertex(c, t);
      }
    }
    if (walls) {
      const auto n = rng.below(g.cell_count() / 2 + 1);
      for (std::uint64_t i = 0; i < n; ++i) {
        const auto c = static_cast<CellId>(rng.below(g.cell_count()));
        if (c != goal) {
          r.forbid_from(c, static_cast<std::uint32_t>(rng.below(3)));
        }
      }
    }
    // Entries on the goal.
    const auto goal_entries = rng.below(4);
    for (std::uint64_t i = 0; i < goal_entries; ++i) {
      std::uint32_t t = 0;
      switch (rng.below(6)) {
        case 0:
          t = static_cast<std::uint32_t>(rng.below(3));
          break;
        case 1:
          t = r.horizon();
          break;
        case 2:
          t = r.horizon() + 1 + static_cast<std::uint32_t>(rng.below(3));
          break;
        case 3:
          t = r.horizon() + static_cast<std::uint32_t>(rng.below(40));
          break;
        case 4:
          t = static_cast<std::uint32_t>(rng.below(r.horizon() + 1));
          break;
        default:
          t = 60 + static_cast<std::uint32_t>(rng.below(60));
      }
      r.forbid_vertex(goal, t);
      if (rng.below(3) == 0) {
        // A move into or out of the goal at about the same time.
        const int n = g.neighbors(goal, next);
        if (n > 0) {
          const CellId v = next[rng.below(static_cast<std::uint64_t>(n))];
          if (rng.below(2) == 0) {
            r.forbid_move(v, goal, t > 0 ? t - 1 : 0);
          } else {
            r.forbid_move(goal, v, t);
          }
        }
      }
    }
    const std::uint32_t bound = r.horizon() + static_cast<std::uint32_t>(g.cell_count()) + 2;
    const auto want = oracle_arrival(g, r, start, goal, bound);
    SearchLimits limits;
    limits.max_expansions = 2'000'000;
    const SearchResult got = space_time_astar(g, start, goal, r, limits);
    ASSERT_NE(got.status, SearchStatus::kLimit) << "round " << round;
    if (!want) {
      // None by the bound; and none twice as far.
      ASSERT_FALSE(oracle_arrival(g, r, start, goal, 2 * bound)) << "round " << round;
      ASSERT_EQ(got.status, SearchStatus::kNoPath) << "round " << round;
      ++none;
      continue;
    }
    ASSERT_EQ(got.status, SearchStatus::kFound) << "round " << round;
    ASSERT_EQ(got.path.size() - 1, *want) << "round " << round;
    ASSERT_TRUE(path_allowed(g, r, got.path, start, goal)) << "round " << round;
    if (r.last_vertex_time(goal) && *want == *r.last_vertex_time(goal) + 1) {
      ++late;
    }
    ++found;
  }
  std::cout << "found " << found << ", none " << none << ", arrival = goal entry + 1: " << late << "\n";
  EXPECT_GT(found, min_found);
  EXPECT_GT(none, min_none);
  EXPECT_GT(late, min_late);
}

TEST(ReviewD3bClean, SpaceTimeMatchesTheOracleWithGoalEntriesAtManyTimes) {
  oracle_rounds(766, false, 6000, 1000, 100, 300);
}

TEST(ReviewD3bClean, SpaceTimeMatchesTheOracleWithGoalEntriesBehindWalls) {
  oracle_rounds(767, true, 6000, 500, 100, 200);
}

// Random tiny instances (goals permuted among starts, agents on their goals, as in review_d3).
Instance tiny_instance(SplitMix64& rng) {
  for (;;) {
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
    if (rng.below(2) == 0) {
      goals = shuffled(starts);
    } else {
      const std::vector<CellId> q = shuffled(cells);
      goals.assign(q.begin(), q.begin() + static_cast<std::ptrdiff_t>(k));
    }
    Instance inst{g, {}};
    for (std::size_t i = 0; i < k; ++i) {
      inst.agents.push_back(Agent{starts[i], goals[i]});
    }
    return inst;
  }
}

// planner.hpp: CBS gives up rather than generate more than max_nodes (root included); the nodes an unlimited run
// generated are enough; a node-limited stop has generated exactly max_nodes. Checked for max_nodes = 1, 2, N - 1, N,
// N + 1 and a random M < N, where N is what the unlimited run generated, solvable or not.
TEST(ReviewD3bClean, CbsNodeLimitIsExactOnTinyInstances) {
  SplitMix64 rng(79);
  int instances = 0;
  int runs = 0;
  int unsolvable = 0;
  for (int round = 0; round < 700; ++round) {
    const Instance inst = tiny_instance(rng);
    PlannerLimits cap;
    cap.max_nodes = 400;
    const PlanResult full = cbs(inst, cap);
    if (full.status == PlanStatus::kLimit || full.status == PlanStatus::kUnreachableGoal) {
      continue;
    }
    ++instances;
    unsolvable += full.status == PlanStatus::kNoPath ? 1 : 0;
    const std::uint64_t n = full.nodes_generated;
    std::vector<std::uint64_t> ms{1, 2, n - 1, n, n + 1};
    if (n > 1) {
      ms.push_back(1 + rng.below(n - 1));
    }
    for (const std::uint64_t m : ms) {
      if (m == 0) {
        continue;
      }
      PlannerLimits limits;
      limits.max_nodes = m;
      const PlanResult got = cbs(inst, limits);
      ++runs;
      if (m >= n) {
        ASSERT_EQ(got.status, full.status) << "round " << round << " m " << m << " n " << n;
        ASSERT_EQ(got.nodes_generated, n) << "round " << round;
        ASSERT_EQ(got.nodes_expanded, full.nodes_expanded) << "round " << round;
        ASSERT_EQ(got.expansions, full.expansions) << "round " << round;
        ASSERT_EQ(got.plan, full.plan) << "round " << round;
      } else {
        ASSERT_EQ(got.status, PlanStatus::kLimit) << "round " << round << " m " << m << " n " << n;
        ASSERT_EQ(got.nodes_generated, m) << "round " << round << " m " << m << " n " << n;
        ASSERT_LE(got.nodes_expanded, full.nodes_expanded) << "round " << round;
      }
    }
  }
  std::cout << instances << " instances (" << unsolvable << " unsolvable), " << runs << " limited runs\n";
  EXPECT_GT(instances, 300);
}

// The expansion limit after the change: the expansions an unlimited run used are enough, one fewer is not (CBS and
// prioritized planning).
TEST(ReviewD3bClean, ExpansionLimitIsExactOnTinyInstances) {
  SplitMix64 rng(3);
  int cbs_checked = 0;
  int pp_checked = 0;
  for (int round = 0; round < 500; ++round) {
    const Instance inst = tiny_instance(rng);
    PlannerLimits cap;
    cap.max_nodes = 300;
    const PlanResult full = cbs(inst, cap);
    if (full.status != PlanStatus::kLimit && full.status != PlanStatus::kUnreachableGoal && full.expansions > 1) {
      PlannerLimits limits = cap;
      limits.max_expansions = full.expansions;
      const PlanResult same = cbs(inst, limits);
      ASSERT_EQ(same.status, full.status) << "round " << round;
      ASSERT_EQ(same.plan, full.plan) << "round " << round;
      limits.max_expansions = full.expansions - 1;
      const PlanResult fewer = cbs(inst, limits);
      ASSERT_EQ(fewer.status, PlanStatus::kLimit) << "round " << round << " expansions " << full.expansions;
      ASSERT_LE(fewer.expansions, full.expansions - 1) << "round " << round;
      ++cbs_checked;
    }
    const PlanResult pp = prioritized_planning(inst, {});
    if (pp.status != PlanStatus::kUnreachableGoal && pp.expansions > 1) {
      PlannerLimits limits;
      limits.max_expansions = pp.expansions;
      const PlanResult same = prioritized_planning(inst, limits);
      ASSERT_EQ(same.status, pp.status) << "round " << round;
      ASSERT_EQ(same.plan, pp.plan) << "round " << round;
      limits.max_expansions = pp.expansions - 1;
      const PlanResult fewer = prioritized_planning(inst, limits);
      ASSERT_EQ(fewer.status, PlanStatus::kLimit) << "round " << round << " expansions " << pp.expansions;
      ASSERT_LE(fewer.expansions, pp.expansions - 1) << "round " << round;
      ++pp_checked;
    }
  }
  std::cout << "cbs " << cbs_checked << ", prioritized " << pp_checked << "\n";
  EXPECT_GT(cbs_checked, 100);
  EXPECT_GT(pp_checked, 200);
}

}  // namespace
}  // namespace pathfinder
