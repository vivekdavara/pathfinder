// The day-2 second-round review (an agent at commit 4db3e63, reviewing 5089fd9..4db3e63, in its own worktree). Its
// two bugs are in the Python bindings (tests/python/review_d2b/test_review_d2b.py); the C++ changes it reviewed held
// up. Every test here is in ReviewD2bClean: checks that passed at 4db3e63 (things the review looked at and found
// right), kept for their coverage. Some of them kill mutants of the changed code that the existing suite lets live
// (named in each comment).

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <set>
#include <stdexcept>
#include <tuple>
#include <vector>

#include "pathfinder/distance.hpp"
#include "pathfinder/grid.hpp"
#include "pathfinder/heuristic.hpp"
#include "pathfinder/plan.hpp"
#include "pathfinder/reservation.hpp"
#include "pathfinder/rng.hpp"
#include "pathfinder/space_time_astar.hpp"

namespace pathfinder {
namespace {

Grid random_grid(SplitMix64& rng, std::int32_t w, std::int32_t h, std::uint64_t blocked_percent) {
  Grid g(w, h);
  for (std::int32_t y = 0; y < h; ++y) {
    for (std::int32_t x = 0; x < w; ++x) {
      if (rng.below(100) < blocked_percent) {
        g.set_passable(Point{x, y}, false);
      }
    }
  }
  return g;
}

// PassableIndex::fits must look at the cell count, not only the width and the passable cells: the index of a 4 x 4
// grid has the same width, passable count and passable cells as a 4 x 5 grid whose extra row is blocked, but a
// DistanceTable over it would read index_[16..19] past its end. (Deleting `g.cell_count() != index_.size()` from
// fits() survives the existing suite, PassableIndex.FitsOnlyItsOwnGrid included; this test kills it.)
TEST(ReviewD2bClean, PassableIndexRefusesAGridWithExtraBlockedRows) {
  Grid g(4, 4);
  g.set_passable(Point{0, 0}, false);
  const auto index = std::make_shared<const PassableIndex>(g);
  Grid taller(4, 5);
  taller.set_passable(Point{0, 0}, false);
  for (std::int32_t x = 0; x < 4; ++x) {
    taller.set_passable(Point{x, 4}, false);
  }
  ASSERT_EQ(taller.passable_count(), g.passable_count());
  EXPECT_FALSE(index->fits(taller));
  EXPECT_THROW(DistanceTable(taller, index, 5), std::invalid_argument);
}

// The BFS over precomputed neighbour numbers gives distances()'s values for every cell and every goal, blocked goals
// included, on random grids of every shape from 1 x 1 to 7 x 7 (1-wide rows and columns, the last row and column,
// fully blocked grids), and fits() accepts exactly the grid's own index.
TEST(ReviewD2bClean, DistanceTablesEqualDistancesOnRandomGrids) {
  SplitMix64 rng(20261107);
  int compared = 0;
  for (std::int32_t w = 1; w <= 7; ++w) {
    for (std::int32_t h = 1; h <= 7; ++h) {
      for (int round = 0; round < 6; ++round) {
        const Grid g = random_grid(rng, w, h, static_cast<std::uint64_t>(round) * 18);  // 0% .. 90% blocked
        const auto index = std::make_shared<const PassableIndex>(g);
        ASSERT_TRUE(index->fits(g));
        for (CellId goal = 0; goal < g.cell_count(); ++goal) {
          const DistanceTable table(g, index, goal);
          const std::vector<std::uint32_t> want = distances(g, goal);
          for (CellId c = 0; c < g.cell_count(); ++c) {
            ASSERT_EQ(table(c), want[c]) << w << "x" << h << " round " << round << " goal " << goal << " cell " << c;
            ++compared;
          }
        }
      }
    }
  }
  EXPECT_GT(compared, 10000);
}

// The byte-per-cell marks against a model with no shortcut: random forbid_vertex / forbid_move / forbid_from calls on
// small grids (1-wide ones included), then vertex_free and move_free asked for every cell, every neighbour and every
// time up to past the horizon, must answer as plain sets of the entries do.
TEST(ReviewD2bClean, VertexAndMoveFreeMatchAPlainModel) {
  SplitMix64 rng(7);
  for (int round = 0; round < 300; ++round) {
    const auto w = static_cast<std::int32_t>(1 + rng.below(5));
    const auto h = static_cast<std::int32_t>(1 + rng.below(5));
    const Grid g(w, h);
    ReservationTable table(g);
    std::set<std::pair<CellId, std::uint32_t>> vertices;
    std::set<std::tuple<CellId, CellId, std::uint32_t>> moves;
    std::vector<std::int64_t> from(g.cell_count(), -1);
    const std::uint64_t cells = g.cell_count();  // no cast: uint64_t is size_t on Linux (GCC: useless cast)
    const int ops = static_cast<int>(rng.below(12));
    for (int op = 0; op < ops; ++op) {
      const auto c = static_cast<CellId>(rng.below(cells));
      const auto t = static_cast<std::uint32_t>(rng.below(8));
      switch (rng.below(3)) {
        case 0:
          table.forbid_vertex(c, t);
          vertices.emplace(c, t);
          break;
        case 1: {
          std::array<CellId, 4> next{};
          const int n = g.neighbors(c, next);
          if (n > 0) {
            const CellId to = next[rng.below(static_cast<std::uint64_t>(n))];
            table.forbid_move(c, to, t);
            moves.emplace(c, to, t);
          }
          break;
        }
        default:
          table.forbid_from(c, t);
          from[c] = from[c] < 0 ? t : std::min<std::int64_t>(from[c], t);
          break;
      }
    }
    for (CellId c = 0; c < g.cell_count(); ++c) {
      for (std::uint32_t t = 0; t <= table.horizon() + 2; ++t) {
        const bool free = vertices.count({c, t}) == 0 && (from[c] < 0 || t < from[c]);
        ASSERT_EQ(table.vertex_free(c, t), free) << "round " << round << " cell " << c << " t " << t;
        std::array<CellId, 4> next{};
        const int n = g.neighbors(c, next);
        for (int k = 0; k < n; ++k) {
          const CellId to = next[static_cast<std::size_t>(k)];
          ASSERT_EQ(table.move_free(c, to, t), moves.count({c, to, t}) == 0)
              << "round " << round << " " << c << "->" << to << " t " << t;
        }
      }
    }
  }
}

// reserve_path changes nothing when it throws, wherever the bad step is (first, middle, last) and whatever the table
// already holds; a good path afterwards is reserved as usual. A first cell off the grid is std::out_of_range, as
// reservation.hpp says (dropping `check(path[t], 0)` from the validation survives the existing suite and turns it into
// std::invalid_argument here).
TEST(ReviewD2bClean, AThrowingReservePathLeavesAFullTableAsItWas) {
  const Grid g(4, 3);  // ids: row 0 is 0..3, row 1 is 4..7, row 2 is 8..11
  ReservationTable table(g);
  table.reserve_path({5, 6, 7});
  const auto snapshot = [&table] {
    return std::make_tuple(table.vertex_count(), table.move_count(), table.from_count(), table.horizon());
  };
  const auto before = snapshot();
  EXPECT_THROW(table.reserve_path({7, 3, 4}), std::invalid_argument);              // last step wraps a row
  EXPECT_THROW(table.reserve_path({0, 1, 1, 2, 6, 7, 12}), std::out_of_range);       // last cell off the grid
  EXPECT_THROW(table.reserve_path({8, 10}), std::invalid_argument);                  // first step jumps a cell
  EXPECT_THROW(table.reserve_path({0, 4, 8, 8, 9, 5, 0}), std::invalid_argument);  // 5 -> 0 isn't a move
  EXPECT_THROW(table.reserve_path({13, 1}), std::out_of_range);  // off the grid, though 13 -> 1 isn't a move either
  EXPECT_EQ(snapshot(), before);
  EXPECT_TRUE(table.vertex_free(0, 0));
  EXPECT_TRUE(table.vertex_free(8, 2));
  table.reserve_path({0, 1});
  EXPECT_FALSE(table.vertex_free(0, 0));
  EXPECT_FALSE(table.move_free(1, 0, 0));  // the reverse of the move
  EXPECT_FALSE(table.vertex_free(1, 100));
}

// space_time_astar refuses a table of another shape and accepts one of the same shape (fits() is about the size,
// as documented); a budget of 0 lets through only a start that is done; a budget of n stops before the (n + 1)th
// expansion but still lets a goal popped then through.
TEST(ReviewD2bClean, SearchLimitsAtTheirEdges) {
  const Grid g(5, 1);
  const ReservationTable none(g);
  EXPECT_THROW(space_time_astar(g, 0, 4, ReservationTable(Grid(1, 5))), std::invalid_argument);
  Grid walled(5, 1);
  walled.set_passable(Point{2, 0}, false);
  EXPECT_EQ(space_time_astar(walled, 0, 1, ReservationTable(walled)).status, SearchStatus::kFound);
  SearchLimits limits;
  limits.max_expansions = 4;  // the walk 0 -> 4 expands 0, 1, 2, 3 and pops 4
  const SearchResult four = space_time_astar(g, 0, 4, none, limits);
  EXPECT_EQ(four.status, SearchStatus::kFound);
  EXPECT_EQ(four.expansions, 4U);
  limits.max_expansions = 3;
  EXPECT_EQ(space_time_astar(g, 0, 4, none, limits).status, SearchStatus::kLimit);
  limits.max_expansions = 0;
  EXPECT_EQ(space_time_astar(g, 4, 4, none, limits).status, SearchStatus::kFound);
  ReservationTable crossed(g);
  crossed.forbid_vertex(4, 2);  // someone crosses the goal at t = 2: staying there needs expansions
  EXPECT_EQ(space_time_astar(g, 4, 4, crossed, limits).status, SearchStatus::kLimit);
}

}  // namespace
}  // namespace pathfinder
