#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <iostream>
#include <stdexcept>

#include "pathfinder/conflicts.hpp"
#include "pathfinder/generate.hpp"
#include "pathfinder/plan.hpp"
#include "pathfinder/rng.hpp"

namespace pathfinder {
namespace {

// The conflicts validate() reports for a plan, as the scanner should: count and earliest.
ConflictScan from_validate(const Instance& inst, const Plan& plan) {
  ConflictScan out;
  for (const Problem& p : validate(inst, plan).problems) {
    if (p.kind != ProblemKind::kVertexConflict && p.kind != ProblemKind::kEdgeConflict) {
      continue;
    }
    const bool edge = p.kind == ProblemKind::kEdgeConflict;
    const Conflict c{edge, p.agent, p.other, p.time, p.cell, edge ? p.to : 0};
    ++out.count;
    if (!out.first || earlier(c, *out.first)) {
      out.first = c;
    }
  }
  return out;
}

// An instance whose goals are the plan's last cells, so only conflicts can be wrong with it.
Instance instance_for(const Grid& g, const Plan& plan) {
  Instance inst{g, {}};
  for (const Path& p : plan) {
    inst.agents.push_back(Agent{p.front(), p.back()});
  }
  return inst;
}

void expect_same(const ConflictScan& got, const ConflictScan& want) {
  EXPECT_EQ(got.count, want.count);
  ASSERT_EQ(got.first.has_value(), want.first.has_value());
  if (want.first) {
    EXPECT_EQ(*got.first, *want.first);
  }
}

TEST(ConflictScanner, FindsWhatValidateFindsOnRandomWalks) {
  // Random walks meet often, in every way: same cell, swaps, one agent parked where another walks. Goals may repeat
  // here (the instance is only validate's frame), which validate's conflict search doesn't mind.
  SplitMix64 rng(77);
  // One scanner for every round (sized for the largest grid, 7 x 7), so each scan starts on what the last one left.
  ConflictScanner scanner(49);
  std::uint64_t conflicts = 0;
  std::uint64_t swaps = 0;
  int clean = 0;
  for (int round = 0; round < 3000; ++round) {
    const Grid g = random_map(static_cast<std::int32_t>(3 + rng.below(5)), static_cast<std::int32_t>(3 + rng.below(5)),
                              static_cast<double>(rng.below(30)), rng.next());
    if (g.passable_count() == 0) {
      continue;
    }
    std::vector<CellId> free;
    for (CellId c = 0; c < g.cell_count(); ++c) {
      if (g.passable(c)) {
        free.push_back(c);
      }
    }
    const std::size_t agents = 1 + rng.below(std::min<std::uint64_t>(free.size(), 6));
    Plan plan;
    for (std::size_t i = 0; i < agents; ++i) {
      Path path{free[rng.below(free.size())]};
      const auto steps = rng.below(10);
      for (std::uint64_t s = 0; s < steps; ++s) {
        std::array<CellId, 4> next{};
        const int n = g.neighbors(path.back(), next);
        const auto pick = rng.below(static_cast<std::uint64_t>(n) + 1);
        path.push_back(pick == 0 ? path.back() : next[pick - 1]);
      }
      plan.push_back(std::move(path));
    }
    const ConflictScan want = from_validate(instance_for(g, plan), plan);
    expect_same(scanner.scan(plan), want);
    // A fresh scanner agrees.
    expect_same(ConflictScanner(g.cell_count()).scan(plan), want);
    if (HasFailure()) {
      FAIL() << "round " << round;
    }
    conflicts += want.count;
    if (want.first && want.first->edge) {
      ++swaps;
    }
    clean += want.count == 0 ? 1 : 0;
  }
  EXPECT_GT(conflicts, 3000U);
  EXPECT_GT(swaps, 20U);  // plans whose first conflict is a swap
  EXPECT_GT(clean, 300);
  std::cout << "[ scanned  ] " << conflicts << " conflicts, " << swaps << " first swaps, " << clean << " clean plans\n";
}

TEST(ConflictScanner, ThreeAgentsInOneCellAreThreeConflicts) {
  const Grid g(3, 3);
  const CellId mid = g.id(Point{1, 1});
  const Plan plan{{g.id(Point{0, 1}), mid}, {g.id(Point{1, 0}), mid}, {g.id(Point{2, 1}), mid}};
  ConflictScanner s(g.cell_count());
  const ConflictScan r = s.scan(plan);
  EXPECT_EQ(r.count, 3U);
  ASSERT_TRUE(r.first);
  EXPECT_EQ(*r.first, (Conflict{false, 0, 1, 1, mid, 0}));
}

TEST(ConflictScanner, AnAgentThatHasFinishedStillOccupiesItsGoal) {
  const Grid g(5, 1);
  // Agent 0 is done at (2, 0) from t = 0; agent 1 walks through it at t = 2.
  const Plan plan{{g.id(Point{2, 0})}, {0, 1, 2, 3, 4}};
  ConflictScanner s(g.cell_count());
  const ConflictScan r = s.scan(plan);
  EXPECT_EQ(r.count, 1U);
  ASSERT_TRUE(r.first);
  EXPECT_EQ(*r.first, (Conflict{false, 0, 1, 2, 2, 0}));
}

TEST(ConflictScanner, FollowingAndRotatingAreNotConflicts) {
  const Grid g(2, 2);
  // Agent 1 follows agent 0 along a row; then the four cells of a 2 x 2 square rotate.
  ConflictScanner s(g.cell_count());
  EXPECT_EQ(s.scan(Plan{{1, 3}, {0, 1}}).count, 0U);
  // 0 -> 1 -> 3 -> 2 -> 0, clockwise.
  EXPECT_EQ(s.scan(Plan{{0, 1}, {1, 3}, {3, 2}, {2, 0}}).count, 0U);
}

TEST(ConflictScanner, TheFirstConflictIsTheEarliestInTimeNotTheLowestAgents) {
  const Grid g(6, 1);
  ConflictScanner s(g.cell_count());
  // Agents 0 and 1 meet in cell 1 at t = 1; agents 2 and 3 swap cells 4 and 5 before that, between t = 0 and 1.
  ConflictScan r = s.scan(Plan{{0, 1}, {2, 1}, {4, 5}, {5, 4}});
  EXPECT_EQ(r.count, 2U);
  ASSERT_TRUE(r.first);
  EXPECT_EQ(*r.first, (Conflict{true, 2, 3, 0, 4, 5}));
  // Agents 0 and 1 swap 4 and 5 between t = 0 and 1; agents 2 and 3 share cell 1 at t = 0, before that (no valid
  // instance starts so, but the scanner doesn't care where plans come from).
  r = s.scan(Plan{{4, 5}, {5, 4}, {1, 2}, {1, 0}});
  EXPECT_EQ(r.count, 2U);
  ASSERT_TRUE(r.first);
  EXPECT_EQ(*r.first, (Conflict{false, 2, 3, 0, 1, 0}));
}

TEST(ConflictScanner, RefusesEmptyPathsAndCellsOffTheGrid) {
  ConflictScanner s(4);
  EXPECT_THROW(s.scan(Plan{{0}, {}}), std::invalid_argument);
  EXPECT_THROW(s.scan(Plan{{0, 4}}), std::out_of_range);
  EXPECT_EQ(s.scan(Plan{}).count, 0U);
  EXPECT_FALSE(s.scan(Plan{}).first);
}

TEST(ConflictEarlier, OrdersByTimeThenVertexBeforeSwapThenAgents) {
  const Conflict v0{false, 3, 4, 0, 0, 0};
  const Conflict e0{true, 0, 1, 0, 0, 1};
  const Conflict v1{false, 0, 1, 1, 0, 0};
  const Conflict v1b{false, 0, 2, 1, 0, 0};
  EXPECT_TRUE(earlier(v0, e0));
  EXPECT_TRUE(earlier(e0, v1));
  EXPECT_TRUE(earlier(v1, v1b));
  EXPECT_FALSE(earlier(v1, v1));
  EXPECT_FALSE(earlier(e0, v0));
}

}  // namespace
}  // namespace pathfinder
