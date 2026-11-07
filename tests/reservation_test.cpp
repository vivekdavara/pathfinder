#include "pathfinder/reservation.hpp"

#include <gtest/gtest.h>

#include <array>
#include <stdexcept>

namespace pathfinder {
namespace {

TEST(ReservationTable, StartsEmpty) {
  const Grid g(4, 3);
  const ReservationTable r(g);
  EXPECT_TRUE(r.empty());
  EXPECT_EQ(r.horizon(), 0U);
  for (CellId c = 0; c < g.cell_count(); ++c) {
    EXPECT_TRUE(r.vertex_free(c, 0));
    EXPECT_TRUE(r.vertex_free(c, 1000));
    EXPECT_FALSE(r.last_vertex_time(c).has_value());
    EXPECT_FALSE(r.forbidden_from(c).has_value());
  }
  EXPECT_TRUE(r.move_free(g.id(Point{1, 1}), g.id(Point{2, 1}), 0));
}

TEST(ReservationTable, AVertexEntryForbidsThatCellAtThatTimeOnly) {
  const Grid g(4, 3);
  ReservationTable r(g);
  const CellId c = g.id(Point{2, 1});
  r.forbid_vertex(c, 5);
  EXPECT_FALSE(r.vertex_free(c, 5));
  EXPECT_TRUE(r.vertex_free(c, 4));
  EXPECT_TRUE(r.vertex_free(c, 6));
  EXPECT_TRUE(r.vertex_free(c + 1, 5));
  EXPECT_EQ(r.last_vertex_time(c), 5U);
  EXPECT_EQ(r.horizon(), 5U);
  r.forbid_vertex(c, 2);  // an earlier time doesn't lower the last one
  EXPECT_EQ(r.last_vertex_time(c), 5U);
  r.forbid_vertex(c, 9);
  EXPECT_EQ(r.last_vertex_time(c), 9U);
  EXPECT_EQ(r.vertex_count(), 3U);
  EXPECT_EQ(r.horizon(), 9U);
}

TEST(ReservationTable, AMoveEntryForbidsThatDirectionAtThatTimeOnly) {
  const Grid g(4, 3);
  ReservationTable r(g);
  const CellId u = g.id(Point{1, 1});
  const CellId north = g.id(Point{1, 0});
  const CellId east = g.id(Point{2, 1});
  const CellId south = g.id(Point{1, 2});
  const CellId west = g.id(Point{0, 1});
  r.forbid_move(u, east, 3);
  EXPECT_FALSE(r.move_free(u, east, 3));
  EXPECT_TRUE(r.move_free(u, east, 2));
  EXPECT_TRUE(r.move_free(u, east, 4));
  EXPECT_TRUE(r.move_free(u, north, 3));
  EXPECT_TRUE(r.move_free(u, south, 3));
  EXPECT_TRUE(r.move_free(u, west, 3));
  EXPECT_TRUE(r.move_free(east, u, 3));  // the reverse is a different move
  EXPECT_TRUE(r.vertex_free(east, 4));   // a move entry says nothing about the cells
  EXPECT_EQ(r.horizon(), 4U);            // the move ends at t + 1
  r.forbid_move(u, west, 3);
  r.forbid_move(u, west, 3);  // twice is once
  EXPECT_FALSE(r.move_free(u, west, 3));
  EXPECT_TRUE(r.move_free(u, north, 3));
  EXPECT_EQ(r.move_count(), 2U);
}

TEST(ReservationTable, TellsAllFourDirectionsApartAtTheGridsEdges) {
  const Grid g(3, 3);
  for (CellId u = 0; u < g.cell_count(); ++u) {
    std::array<CellId, 4> next{};
    const int n = g.neighbors(u, next);
    for (int i = 0; i < n; ++i) {
      ReservationTable r(g);
      const CellId v = next[static_cast<std::size_t>(i)];
      r.forbid_move(u, v, 0);
      for (int j = 0; j < n; ++j) {
        const CellId w = next[static_cast<std::size_t>(j)];
        EXPECT_EQ(r.move_free(u, w, 0), w != v) << u << " -> " << v << ", asked " << w;
      }
    }
  }
}

TEST(ReservationTable, RefusesMovesBetweenCellsThatArentNeighbours) {
  const Grid g(4, 3);
  ReservationTable r(g);
  EXPECT_THROW(r.forbid_move(g.id(Point{3, 0}), g.id(Point{0, 1}), 0), std::invalid_argument);  // consecutive ids
  EXPECT_THROW(r.forbid_move(g.id(Point{0, 1}), g.id(Point{3, 0}), 0), std::invalid_argument);  // the other way
  EXPECT_THROW(r.forbid_move(g.id(Point{1, 1}), g.id(Point{1, 1}), 0), std::invalid_argument);  // a wait
  EXPECT_THROW(r.forbid_move(g.id(Point{0, 0}), g.id(Point{2, 0}), 0), std::invalid_argument);  // two cells away
  EXPECT_THROW(r.forbid_move(g.id(Point{0, 0}), g.id(Point{1, 1}), 0), std::invalid_argument);  // a diagonal
  EXPECT_TRUE(r.empty());
}

TEST(ReservationTable, RefusesCellsOutsideTheGridAndTimesPastTheLimit) {
  const Grid g(4, 3);
  ReservationTable r(g);
  EXPECT_THROW(r.forbid_vertex(12, 0), std::out_of_range);
  EXPECT_THROW(r.forbid_from(12, 0), std::out_of_range);
  EXPECT_THROW(r.forbid_move(11, 15, 0), std::out_of_range);  // "south" of the last row
  EXPECT_THROW(r.forbid_vertex(0, ReservationTable::kMaxTime + 1), std::invalid_argument);
  EXPECT_THROW(r.forbid_move(0, 1, ReservationTable::kMaxTime), std::invalid_argument);  // it would end after it
  r.forbid_vertex(0, ReservationTable::kMaxTime);
  r.forbid_move(0, 1, ReservationTable::kMaxTime - 1);
  EXPECT_EQ(r.horizon(), ReservationTable::kMaxTime);
}

TEST(ReservationTable, ForbiddingFromATimeCoversEveryLaterTime) {
  const Grid g(4, 3);
  ReservationTable r(g);
  const CellId c = g.id(Point{3, 2});
  r.forbid_from(c, 7);
  EXPECT_TRUE(r.vertex_free(c, 6));
  EXPECT_FALSE(r.vertex_free(c, 7));
  EXPECT_FALSE(r.vertex_free(c, 8));
  EXPECT_FALSE(r.vertex_free(c, ReservationTable::kMaxTime));
  EXPECT_EQ(r.forbidden_from(c), 7U);
  EXPECT_FALSE(r.last_vertex_time(c).has_value());  // not a vertex entry
  EXPECT_EQ(r.horizon(), 7U);
  r.forbid_from(c, 9);  // the earlier time stays
  EXPECT_EQ(r.forbidden_from(c), 7U);
  r.forbid_from(c, 4);
  EXPECT_EQ(r.forbidden_from(c), 4U);
  EXPECT_EQ(r.from_count(), 1U);
}

TEST(ReservationTable, ReservesAPathAsItsCellsItsReverseMovesAndItsRest) {
  const Grid g(4, 3);
  ReservationTable r(g);
  const CellId a = g.id(Point{0, 0});
  const CellId b = g.id(Point{1, 0});
  const CellId c = g.id(Point{1, 1});
  r.reserve_path({a, b, b, c});  // a move, a wait, a move; then it rests at c
  EXPECT_FALSE(r.vertex_free(a, 0));
  EXPECT_FALSE(r.vertex_free(b, 1));
  EXPECT_FALSE(r.vertex_free(b, 2));
  EXPECT_TRUE(r.vertex_free(b, 3));
  EXPECT_FALSE(r.vertex_free(c, 3));
  EXPECT_FALSE(r.vertex_free(c, 100));
  EXPECT_TRUE(r.vertex_free(c, 2));
  EXPECT_TRUE(r.vertex_free(a, 1));
  // Its moves a -> b at 0 and b -> c at 2: the searching agent may not make the reverse ones then.
  EXPECT_FALSE(r.move_free(b, a, 0));
  EXPECT_FALSE(r.move_free(c, b, 2));
  EXPECT_TRUE(r.move_free(a, b, 0));  // following it is allowed
  EXPECT_TRUE(r.move_free(b, a, 1));
  EXPECT_EQ(r.vertex_count(), 3U);
  EXPECT_EQ(r.move_count(), 2U);
  EXPECT_EQ(r.forbidden_from(c), 3U);
  EXPECT_EQ(r.horizon(), 3U);
}

TEST(ReservationTable, ReservesAOneCellPathAsARestFromTimeZero) {
  const Grid g(2, 2);
  ReservationTable r(g);
  r.reserve_path({3});
  EXPECT_FALSE(r.vertex_free(3, 0));
  EXPECT_EQ(r.forbidden_from(3), 0U);
  EXPECT_EQ(r.horizon(), 0U);
  EXPECT_THROW(r.reserve_path({}), std::invalid_argument);
}

TEST(ReservationTable, ABadPathChangesNothing) {
  const Grid g(4, 3);
  ReservationTable r(g);
  EXPECT_THROW(r.reserve_path({0, 1, 2, 3, 4}), std::invalid_argument);  // 3 -> 4 wraps to the next row
  EXPECT_THROW(r.reserve_path({0, 1, 13}), std::out_of_range);           // off the grid
  EXPECT_THROW(r.reserve_path({0, 0, 5}), std::invalid_argument);        // a diagonal
  EXPECT_TRUE(r.empty());
  EXPECT_EQ(r.horizon(), 0U);
}

TEST(ReservationTable, KnowsWhichCellsAreNeighbours) {
  const ReservationTable r(Grid(4, 3));
  EXPECT_TRUE(r.neighbours(1, 2));
  EXPECT_TRUE(r.neighbours(2, 1));
  EXPECT_TRUE(r.neighbours(1, 5));
  EXPECT_TRUE(r.neighbours(5, 1));
  EXPECT_FALSE(r.neighbours(3, 4));  // consecutive ids across a row's end
  EXPECT_FALSE(r.neighbours(4, 3));
  EXPECT_FALSE(r.neighbours(1, 1));
  EXPECT_FALSE(r.neighbours(0, 5));  // a diagonal
  EXPECT_FALSE(r.neighbours(9, 13));  // "south" of the last row
  EXPECT_FALSE(r.neighbours(12, 11));
}

TEST(ReservationTable, KnowsWhichGridsItFits) {
  const ReservationTable r(Grid(4, 3));
  EXPECT_TRUE(r.fits(Grid(4, 3)));
  EXPECT_FALSE(r.fits(Grid(3, 4)));  // same cell count, different rows
  EXPECT_FALSE(r.fits(Grid(4, 4)));
}

}  // namespace
}  // namespace pathfinder
