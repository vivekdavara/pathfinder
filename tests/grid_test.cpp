#include "pathfinder/grid.hpp"

#include <gtest/gtest.h>

#include <array>
#include <stdexcept>

namespace pathfinder {
namespace {

TEST(Grid, StartsWithEveryCellPassable) {
  const Grid g(3, 2);
  EXPECT_EQ(g.width(), 3);
  EXPECT_EQ(g.height(), 2);
  EXPECT_EQ(g.cell_count(), 6U);
  EXPECT_EQ(g.passable_count(), 6U);
  for (std::int32_t y = 0; y < 2; ++y) {
    for (std::int32_t x = 0; x < 3; ++x) {
      EXPECT_TRUE(g.passable(Point{x, y}));
    }
  }
}

TEST(Grid, RefusesSizesOutsideTheLimits) {
  EXPECT_THROW(Grid(0, 5), std::invalid_argument);
  EXPECT_THROW(Grid(5, 0), std::invalid_argument);
  EXPECT_THROW(Grid(-1, 5), std::invalid_argument);
  EXPECT_THROW(Grid(kMaxDimension + 1, 1), std::invalid_argument);
  EXPECT_THROW(Grid(1, kMaxDimension + 1), std::invalid_argument);
  EXPECT_NO_THROW(Grid(kMaxDimension, 1));
  EXPECT_NO_THROW(Grid(1, 1));
}

TEST(Grid, NumbersCellsRowByRow) {
  const Grid g(4, 3);
  EXPECT_EQ(g.id(Point{0, 0}), 0U);
  EXPECT_EQ(g.id(Point{3, 0}), 3U);
  EXPECT_EQ(g.id(Point{0, 1}), 4U);
  EXPECT_EQ(g.id(Point{2, 2}), 10U);
  for (CellId c = 0; c < g.cell_count(); ++c) {
    EXPECT_EQ(g.id(g.point(c)), c);
  }
}

TEST(Grid, CellsOutsideAreBlocked) {
  const Grid g(2, 2);
  EXPECT_FALSE(g.passable(Point{-1, 0}));
  EXPECT_FALSE(g.passable(Point{0, -1}));
  EXPECT_FALSE(g.passable(Point{2, 0}));
  EXPECT_FALSE(g.passable(Point{0, 2}));
  EXPECT_FALSE(g.in_bounds(Point{2, 1}));
  EXPECT_TRUE(g.in_bounds(Point{1, 1}));
}

TEST(Grid, SetPassableChangesOneCellAndRefusesCellsOutside) {
  Grid g(3, 3);
  g.set_passable(Point{1, 1}, false);
  EXPECT_FALSE(g.passable(Point{1, 1}));
  EXPECT_FALSE(g.passable(g.id(Point{1, 1})));
  EXPECT_EQ(g.passable_count(), 8U);
  g.set_passable(Point{1, 1}, true);
  EXPECT_EQ(g.passable_count(), 9U);
  EXPECT_THROW(g.set_passable(Point{3, 0}, false), std::out_of_range);
  EXPECT_THROW(g.set_passable(Point{0, -1}, false), std::out_of_range);
}

TEST(Grid, NeighboursComeNorthEastSouthWestAndSkipBlockedAndOutside) {
  Grid g(3, 3);
  std::array<CellId, 4> out{};
  // The centre has all four.
  ASSERT_EQ(g.neighbors(g.id(Point{1, 1}), out), 4);
  EXPECT_EQ(out[0], g.id(Point{1, 0}));
  EXPECT_EQ(out[1], g.id(Point{2, 1}));
  EXPECT_EQ(out[2], g.id(Point{1, 2}));
  EXPECT_EQ(out[3], g.id(Point{0, 1}));
  // A corner has two, and nothing wraps around a row's end.
  ASSERT_EQ(g.neighbors(g.id(Point{2, 0}), out), 2);
  EXPECT_EQ(out[0], g.id(Point{2, 1}));
  EXPECT_EQ(out[1], g.id(Point{1, 0}));
  ASSERT_EQ(g.neighbors(g.id(Point{0, 1}), out), 3);
  EXPECT_EQ(out[0], g.id(Point{0, 0}));
  EXPECT_EQ(out[1], g.id(Point{1, 1}));
  EXPECT_EQ(out[2], g.id(Point{0, 2}));
  // Blocked cells aren't neighbours.
  g.set_passable(Point{1, 0}, false);
  g.set_passable(Point{0, 1}, false);
  ASSERT_EQ(g.neighbors(g.id(Point{1, 1}), out), 2);
  EXPECT_EQ(out[0], g.id(Point{2, 1}));
  EXPECT_EQ(out[1], g.id(Point{1, 2}));
}

TEST(Grid, OneByOneAndOneRowGridsHaveTheRightNeighbours) {
  const Grid single(1, 1);
  std::array<CellId, 4> out{};
  EXPECT_EQ(single.neighbors(0, out), 0);
  const Grid row(4, 1);
  ASSERT_EQ(row.neighbors(0, out), 1);
  EXPECT_EQ(out[0], 1U);
  ASSERT_EQ(row.neighbors(3, out), 1);
  EXPECT_EQ(out[0], 2U);
  const Grid column(1, 4);
  ASSERT_EQ(column.neighbors(1, out), 2);
  EXPECT_EQ(out[0], 0U);
  EXPECT_EQ(out[1], 2U);
}

TEST(Grid, AdjacentOrSameMeansOneStepApartWithoutWrapping) {
  const Grid g(3, 3);
  EXPECT_TRUE(g.adjacent_or_same(4, 4));
  EXPECT_TRUE(g.adjacent_or_same(4, 1));
  EXPECT_TRUE(g.adjacent_or_same(4, 5));
  EXPECT_FALSE(g.adjacent_or_same(4, 0));  // diagonal
  EXPECT_FALSE(g.adjacent_or_same(2, 3));  // the end of row 0 and the start of row 1
  EXPECT_FALSE(g.adjacent_or_same(0, 2));
}

TEST(Grid, EqualityComparesSizeAndEveryCell) {
  Grid a(2, 3);
  Grid b(2, 3);
  EXPECT_EQ(a, b);
  b.set_passable(Point{1, 2}, false);
  EXPECT_NE(a, b);
  EXPECT_NE(Grid(2, 3), Grid(3, 2));
}

}  // namespace
}  // namespace pathfinder
