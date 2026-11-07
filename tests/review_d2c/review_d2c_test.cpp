// The day-2 third-round review (an agent at commit 98f60c4, reviewing 4db3e63..98f60c4, in its own worktree). It
// found no bug: every test here is in ReviewD2cClean, checks that passed at 98f60c4, kept for their coverage. The
// Python side (the GIL that prioritized_planning still releases, move_free's ValueError) is in
// tests/python/review_d2c/test_review_d2c.py.

#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

#include "pathfinder/grid.hpp"
#include "pathfinder/plan.hpp"
#include "pathfinder/reservation.hpp"

namespace pathfinder {
namespace {

// What neighbours() should say, from coordinates (no id arithmetic).
bool neighbours_by_points(const Grid& g, CellId a, CellId b) {
  if (a >= g.cell_count() || b >= g.cell_count()) {
    return false;
  }
  const Point p = g.point(a);
  const Point q = g.point(b);
  const int dx = p.x > q.x ? p.x - q.x : q.x - p.x;
  const int dy = p.y > q.y ? p.y - q.y : q.y - p.y;
  return dx + dy == 1;
}

// Every pair of cells of every grid up to 7 x 7 (1-wide and 1-tall ones included), and cells just off the grid and
// far off it on either side. Kills a mutant that drops `from >= cells_` from direction_or_none: from = cell_count()
// "north" to cell_count() - width is accepted then (reservation_test's off-grid cases all have x = 0 going west).
TEST(ReviewD2cClean, NeighboursMatchesCoordinatesOnSmallGridsAndRefusesOffGridCells) {
  for (std::int32_t w = 1; w <= 7; ++w) {
    for (std::int32_t h = 1; h <= 7; ++h) {
      const Grid g(w, h);
      const ReservationTable r(g);
      const auto n = static_cast<CellId>(g.cell_count());
      const auto uw = static_cast<CellId>(w);
      for (CellId a = 0; a < n; ++a) {
        for (CellId b = 0; b < n; ++b) {
          ASSERT_EQ(r.neighbours(a, b), neighbours_by_points(g, a, b)) << w << "x" << h << " " << a << " " << b;
        }
      }
      const std::vector<CellId> off = {n, n + 1, n + uw - 1, n + uw, n + uw + 1, 2 * n,
                                       std::numeric_limits<CellId>::max(), std::numeric_limits<CellId>::max() - uw};
      for (const CellId o : off) {
        for (CellId a = 0; a < n; ++a) {
          EXPECT_FALSE(r.neighbours(a, o)) << w << "x" << h << " " << a << " " << o;
          EXPECT_FALSE(r.neighbours(o, a)) << w << "x" << h << " " << o << " " << a;
        }
        EXPECT_FALSE(r.neighbours(o, o));
      }
    }
  }
}

// A table made for a default-constructed grid (width 0, no cells) answers false, without the `% 0` it would reach if
// the bounds check came later.
TEST(ReviewD2cClean, NeighboursOnATableForAnEmptyGridIsFalse) {
  const ReservationTable r{Grid{}};
  EXPECT_FALSE(r.neighbours(0, 0));
  EXPECT_FALSE(r.neighbours(0, 1));
  EXPECT_FALSE(r.neighbours(1, 0));
}

// The largest grid (10,000 x 10,000 = 10^8 cells): from + w stays far below 2^32, and the last row and column behave.
TEST(ReviewD2cClean, NeighboursOnTheLargestGrid) {
  const Grid g(kMaxDimension, kMaxDimension);
  const ReservationTable r(g);
  const auto w = static_cast<CellId>(kMaxDimension);
  const auto n = static_cast<CellId>(g.cell_count());
  EXPECT_TRUE(r.neighbours(n - 1, n - 2));
  EXPECT_TRUE(r.neighbours(n - 1, n - 1 - w));
  EXPECT_TRUE(r.neighbours(n - 1 - w, n - 1));
  EXPECT_FALSE(r.neighbours(n - w, n - w - 1));  // first column of the last row, west across a row's end
  EXPECT_FALSE(r.neighbours(n - w - 1, n - w));
  EXPECT_FALSE(r.neighbours(n - 1, n - 1 + w));  // "south" of the last row
  EXPECT_FALSE(r.neighbours(n, n - w));          // off the grid, "north" onto it
  EXPECT_TRUE(r.neighbours(w - 1, 2 * w - 1));
  EXPECT_FALSE(r.neighbours(w - 1, w));
}

// forbid_move and reserve_path: the same exception types as before the refactor (std::out_of_range for a cell off
// the grid, checked before the neighbour test even when the off-grid cell is id-arithmetically "south"; and
// std::invalid_argument for non-neighbours, a wait and t = kMaxTime), and nothing changed after any of them.
TEST(ReviewD2cClean, ForbidMoveAndReservePathThrowTheSameAndChangeNothing) {
  const Grid g(4, 3);
  ReservationTable r(g);
  r.forbid_vertex(5, 7);
  r.forbid_move(5, 6, 2);
  r.forbid_from(0, 4);
  const auto same = [&r] {
    return r.vertex_count() == 1 && r.move_count() == 1 && r.from_count() == 1 && r.horizon() == 7;
  };
  EXPECT_THROW(r.forbid_move(9, 13, 0), std::out_of_range);
  EXPECT_THROW(r.forbid_move(12, 8, 0), std::out_of_range);
  EXPECT_THROW(r.forbid_move(3, 4, 0), std::invalid_argument);
  EXPECT_THROW(r.forbid_move(4, 3, 0), std::invalid_argument);
  EXPECT_THROW(r.forbid_move(6, 6, 0), std::invalid_argument);
  EXPECT_THROW(r.forbid_move(5, 6, ReservationTable::kMaxTime), std::invalid_argument);
  EXPECT_THROW(r.forbid_move(5, 6, ReservationTable::kMaxTime + 1), std::invalid_argument);
  EXPECT_TRUE(same());
  EXPECT_THROW(r.reserve_path(Path{9, 13}), std::out_of_range);
  EXPECT_THROW(r.reserve_path(Path{1, 2, 3, 4}), std::invalid_argument);
  EXPECT_THROW(r.reserve_path(Path{1, 2, 2, 6, 6, 10, 14}), std::out_of_range);
  EXPECT_THROW(r.reserve_path(Path{}), std::invalid_argument);
  EXPECT_TRUE(same());
  // And the moves it does accept are the ones move_free then refuses, in that direction only.
  const Grid one(1, 5);
  ReservationTable col(one);
  col.reserve_path(Path{0, 1, 2});
  EXPECT_FALSE(col.move_free(1, 0, 0));
  EXPECT_FALSE(col.move_free(2, 1, 1));
  EXPECT_TRUE(col.move_free(0, 1, 0));
  EXPECT_TRUE(col.move_free(1, 2, 1));
}

}  // namespace
}  // namespace pathfinder
