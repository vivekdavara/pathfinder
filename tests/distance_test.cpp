#include "pathfinder/distance.hpp"

#include <gtest/gtest.h>

#include <cstdlib>

#include "pathfinder/movingai.hpp"

namespace pathfinder {
namespace {

TEST(Distances, OnAnOpenGridAreManhattanDistances) {
  const Grid g(7, 5);
  const CellId source = g.id(Point{2, 3});
  const auto d = distances(g, source);
  for (CellId c = 0; c < g.cell_count(); ++c) {
    const Point p = g.point(c);
    EXPECT_EQ(d[c], static_cast<std::uint32_t>(std::abs(p.x - 2) + std::abs(p.y - 3))) << c;
  }
}

TEST(Distances, GoAroundWallsAndMarkWhatTheyCantReach) {
  // A wall with one gap at the bottom; the passable cell (3, 0) is walled off.
  const Grid g = parse_map(
      "type octile\nheight 4\nwidth 5\nmap\n"
      "..@.@\n"
      "..@@.\n"
      "..@..\n"
      ".....\n");
  const auto d = distances(g, g.id(Point{0, 0}));
  EXPECT_EQ(d[g.id(Point{0, 0})], 0U);
  EXPECT_EQ(d[g.id(Point{1, 1})], 2U);
  EXPECT_EQ(d[g.id(Point{3, 2})], 7U);  // down to row 3, across, up one
  EXPECT_EQ(d[g.id(Point{4, 1})], 9U);  // down to row 3, across, and up the right-hand side
  EXPECT_EQ(d[g.id(Point{3, 0})], kUnreachable);  // passable, walled off
  EXPECT_EQ(d[g.id(Point{2, 0})], kUnreachable);  // blocked
  EXPECT_EQ(d[g.id(Point{4, 0})], kUnreachable);  // blocked
  // A blocked source reaches nothing.
  const auto none = distances(g, g.id(Point{2, 0}));
  for (const auto x : none) {
    EXPECT_EQ(x, kUnreachable);
  }
}

TEST(Distances, AreSymmetric) {
  const Grid g = parse_map(
      "type octile\nheight 5\nwidth 6\nmap\n"
      "......\n"
      ".@@@@.\n"
      ".@..@.\n"
      ".@.@@.\n"
      "......\n");
  for (CellId a = 0; a < g.cell_count(); ++a) {
    if (!g.passable(a)) {
      continue;
    }
    const auto from_a = distances(g, a);
    for (CellId b = 0; b < g.cell_count(); ++b) {
      if (g.passable(b)) {
        EXPECT_EQ(from_a[b], distances(g, b)[a]) << a << " " << b;
      }
    }
  }
}

TEST(Components, NumberPiecesByTheirLowestCellAndFindTheLargest) {
  const Grid g = parse_map(
      "type octile\nheight 3\nwidth 6\nmap\n"
      ".@..@.\n"
      ".@..@@\n"
      "@@..@.\n");
  const Components c = components(g);
  ASSERT_EQ(c.sizes.size(), 4U);
  EXPECT_EQ(c.label[g.id(Point{0, 0})], 0U);
  EXPECT_EQ(c.label[g.id(Point{2, 0})], 1U);
  EXPECT_EQ(c.label[g.id(Point{5, 0})], 2U);
  EXPECT_EQ(c.label[g.id(Point{5, 2})], 3U);
  EXPECT_EQ(c.label[g.id(Point{1, 0})], Components::kNone);
  EXPECT_EQ(c.sizes, (std::vector<std::size_t>{2, 6, 1, 1}));
  EXPECT_EQ(c.largest, 1U);
}

TEST(Components, TiesGoToTheLowerNumberedPieceAndAllBlockedHasNone) {
  const Grid ties = parse_map("type octile\nheight 1\nwidth 5\nmap\n..@..\n");
  const Components c = components(ties);
  EXPECT_EQ(c.sizes, (std::vector<std::size_t>{2, 2}));
  EXPECT_EQ(c.largest, 0U);
  const Grid walls = parse_map("type octile\nheight 1\nwidth 2\nmap\n@@\n");
  const Components none = components(walls);
  EXPECT_TRUE(none.sizes.empty());
  EXPECT_EQ(none.largest, Components::kNone);
}

TEST(Components, AgreeWithDistances) {
  const Grid g = parse_map(
      "type octile\nheight 4\nwidth 7\nmap\n"
      "..@....\n"
      "..@.@@.\n"
      "@@@.@..\n"
      "...@@..\n");
  const Components c = components(g);
  for (CellId a = 0; a < g.cell_count(); ++a) {
    if (!g.passable(a)) {
      continue;
    }
    const auto d = distances(g, a);
    for (CellId b = 0; b < g.cell_count(); ++b) {
      if (g.passable(b)) {
        EXPECT_EQ(c.label[a] == c.label[b], d[b] != kUnreachable) << a << " " << b;
      }
    }
  }
}

}  // namespace
}  // namespace pathfinder
