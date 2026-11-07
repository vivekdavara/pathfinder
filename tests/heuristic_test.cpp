#include "pathfinder/heuristic.hpp"

#include <gtest/gtest.h>

#include <array>
#include <memory>
#include <stdexcept>

#include "pathfinder/generate.hpp"
#include "pathfinder/movingai.hpp"

namespace pathfinder {
namespace {

TEST(PassableIndex, NumbersPassableCellsInRowMajorOrder) {
  const Grid g = parse_map(
      "type octile\nheight 2\nwidth 3\nmap\n"
      ".@.\n"
      "@..\n");
  const PassableIndex index(g);
  ASSERT_EQ(index.size(), 4U);
  EXPECT_EQ(index[0], 0U);
  EXPECT_EQ(index[1], PassableIndex::kBlocked);
  EXPECT_EQ(index[2], 1U);
  EXPECT_EQ(index[3], PassableIndex::kBlocked);
  EXPECT_EQ(index[4], 2U);
  EXPECT_EQ(index[5], 3U);
  EXPECT_EQ(index.cell(0), 0U);
  EXPECT_EQ(index.cell(1), 2U);
  EXPECT_EQ(index.cell(3), 5U);
  EXPECT_EQ(index.bytes(), 6 * 4 + 4 * 20U);  // 4 per cell; per passable cell, its cell id and 4 neighbours
}

TEST(PassableIndex, ListsEachPassableCellsNeighboursByNumber) {
  const Grid g = parse_map(
      "type octile\nheight 3\nwidth 3\nmap\n"
      ".@.\n"
      "...\n"
      "@..\n");
  const PassableIndex index(g);
  const auto none = PassableIndex::kBlocked;
  // Numbers: (0,0) 0, (2,0) 1, (0,1) 2, (1,1) 3, (2,1) 4, (1,2) 5, (2,2) 6. Order: north, east, south, west.
  EXPECT_EQ(index.neighbors(0), (std::array<std::uint32_t, 4>{2, none, none, none}));  // south only
  EXPECT_EQ(index.neighbors(3), (std::array<std::uint32_t, 4>{4, 5, 2, none}));       // east, south, west
  EXPECT_EQ(index.neighbors(4), (std::array<std::uint32_t, 4>{1, 6, 3, none}));       // north, south, west
  EXPECT_EQ(index.neighbors(6), (std::array<std::uint32_t, 4>{4, 5, none, none}));    // north, west
  for (std::uint32_t i = 0; i < index.size(); ++i) {
    std::array<CellId, 4> next{};
    const int n = g.neighbors(index.cell(i), next);
    for (int k = 0; k < 4; ++k) {
      const auto j = index.neighbors(i)[static_cast<std::size_t>(k)];
      EXPECT_EQ(j, k < n ? index[next[static_cast<std::size_t>(k)]] : none) << i << " " << k;
    }
  }
}

TEST(PassableIndex, OfAFullyBlockedGridIsEmpty) {
  Grid g(2, 2);
  for (int y = 0; y < 2; ++y) {
    for (int x = 0; x < 2; ++x) {
      g.set_passable(Point{x, y}, false);
    }
  }
  const PassableIndex index(g);
  EXPECT_EQ(index.size(), 0U);
  for (CellId c = 0; c < 4; ++c) {
    EXPECT_EQ(index[c], PassableIndex::kBlocked);
  }
}

// The table must hold exactly what distances() computes, on every cell, blocked and walled-off ones included.
TEST(DistanceTable, EqualsDistancesOnRandomMaps) {
  for (std::uint64_t seed = 1; seed <= 20; ++seed) {
    const auto side = static_cast<std::int32_t>(5 + seed * 3);
    const Grid g = random_map(side, side + 2, 35.0, seed);  // 35% blocked: several pieces, walled-off cells
    const auto index = std::make_shared<const PassableIndex>(g);
    for (CellId goal = 0; goal < g.cell_count(); goal += 7) {
      const DistanceTable table(g, index, goal);
      const auto expected = distances(g, goal);
      EXPECT_EQ(table.goal(), goal);
      for (CellId c = 0; c < g.cell_count(); ++c) {
        ASSERT_EQ(table(c), expected[c]) << "seed " << seed << ", goal " << goal << ", cell " << c;
      }
    }
  }
}

TEST(DistanceTable, OfABlockedGoalReachesNothing) {
  Grid g(3, 3);
  g.set_passable(Point{1, 1}, false);
  const auto index = std::make_shared<const PassableIndex>(g);
  const DistanceTable table(g, index, g.id(Point{1, 1}));
  for (CellId c = 0; c < g.cell_count(); ++c) {
    EXPECT_EQ(table(c), kUnreachable) << c;
  }
}

TEST(DistanceTable, HoldsFourBytesPerPassableCell) {
  const Grid g = random_map(40, 30, 50.0, 3);
  const auto index = std::make_shared<const PassableIndex>(g);
  const DistanceTable table(g, index, index->cell(0));
  EXPECT_EQ(table.bytes(), g.passable_count() * 4);
  EXPECT_EQ(g.passable_count(), 600U);  // exactly half of 1,200: random_map blocks round(n * percent / 100) cells
}

TEST(DistanceTable, RefusesAnIndexOfAnotherGrid) {
  const Grid g(4, 4);
  Grid other(4, 4);
  other.set_passable(Point{0, 0}, false);
  const auto wrong = std::make_shared<const PassableIndex>(other);
  EXPECT_THROW(DistanceTable(g, wrong, 5), std::invalid_argument);
  EXPECT_THROW(DistanceTable(g, nullptr, 5), std::invalid_argument);
}

}  // namespace
}  // namespace pathfinder
