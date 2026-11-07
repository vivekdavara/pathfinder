#include "pathfinder/space_time_astar.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <memory>

#include "pathfinder/distance.hpp"
#include "pathfinder/generate.hpp"
#include "pathfinder/movingai.hpp"
#include "search_checks.hpp"

namespace pathfinder {
namespace {

using testing::allowed;

// A one-cell-wide corridor of n cells, (0, 0) to (n - 1, 0).
Grid corridor(std::int32_t n) { return Grid(n, 1); }

TEST(SpaceTimeAStar, WithNothingReservedFindsAShortestPath) {
  const Grid g = random_map(30, 20, 25.0, 4);
  const ReservationTable none(g);
  const auto index = std::make_shared<const PassableIndex>(g);
  int found = 0;
  for (CellId goal = 0; goal < g.cell_count(); goal += 37) {
    const DistanceTable h(g, index, goal);
    for (CellId start = 3; start < g.cell_count(); start += 41) {
      const SearchResult r = space_time_astar(g, start, h, none);
      if (h(start) == kUnreachable) {
        EXPECT_EQ(r.status, SearchStatus::kNoPath);
        EXPECT_EQ(r.expansions, 0U);  // known before searching
        continue;
      }
      ASSERT_EQ(r.status, SearchStatus::kFound) << start << " -> " << goal;
      EXPECT_EQ(r.path.size(), h(start) + 1U);
      EXPECT_EQ(allowed(g, none, r.path, start, goal), "");
      // With an exact heuristic and ties towards deeper states, A* walks straight down the shortest path.
      EXPECT_EQ(r.expansions, h(start));
      ++found;
    }
  }
  EXPECT_GT(found, 100);
}

TEST(SpaceTimeAStar, AnAgentAlreadyAtItsGoalStaysThere) {
  const Grid g(3, 3);
  const ReservationTable none(g);
  const SearchResult r = space_time_astar(g, 4, 4, none);
  ASSERT_EQ(r.status, SearchStatus::kFound);
  EXPECT_EQ(r.path, (Path{4}));
  EXPECT_EQ(r.expansions, 0U);
}

TEST(SpaceTimeAStar, WaitsForAReservedCellToClear) {
  const Grid g = corridor(5);
  ReservationTable r(g);
  r.forbid_vertex(2, 2);  // the agent would be in cell 2 at t = 2
  const SearchResult s = space_time_astar(g, 0, 4, r);
  ASSERT_EQ(s.status, SearchStatus::kFound);
  EXPECT_EQ(s.path.size(), 6U);  // one wait
  EXPECT_EQ(allowed(g, r, s.path, 0, 4), "");
  // Ties go to the deeper state, so it waits as late as it can: in cell 1 at t = 1 and t = 2.
  EXPECT_EQ(s.path, (Path{0, 1, 1, 2, 3, 4}));
}

TEST(SpaceTimeAStar, CantSwapWithAnAgentComingTheOtherWay) {
  const Grid g(2, 2);
  ReservationTable r(g);
  r.forbid_move(0, 1, 0);  // another agent goes 1 -> 0 between t = 0 and 1
  r.forbid_from(0, 1);     // and rests in 0 from t = 1
  const SearchResult s = space_time_astar(g, 0, 1, r);
  ASSERT_EQ(s.status, SearchStatus::kFound);
  EXPECT_EQ(allowed(g, r, s.path, 0, 1), "");
  EXPECT_EQ(s.path, (Path{0, 2, 3, 1}));  // around the square: it can neither stay, nor swap, nor come back
}

TEST(SpaceTimeAStar, AMoveEntryAloneForcesAWaitInACorridor) {
  const Grid g = corridor(2);
  ReservationTable r(g);
  r.forbid_move(0, 1, 0);
  const SearchResult s = space_time_astar(g, 0, 1, r);
  ASSERT_EQ(s.status, SearchStatus::kFound);
  EXPECT_EQ(s.path, (Path{0, 0, 1}));
}

TEST(SpaceTimeAStar, DoesntEndOnAGoalSomeoneStillHasToCross) {
  const Grid g = corridor(6);
  ReservationTable r(g);
  r.forbid_vertex(2, 9);  // someone passes the goal at t = 9
  const SearchResult s = space_time_astar(g, 0, 2, r);
  ASSERT_EQ(s.status, SearchStatus::kFound);
  EXPECT_EQ(allowed(g, r, s.path, 0, 2), "");
  EXPECT_EQ(s.path.size(), 11U);  // at the goal for good from t = 10
  EXPECT_NE(s.path[9], 2U);
}

TEST(SpaceTimeAStar, AnAgentOnItsGoalStepsAsideForSomeonePassing) {
  const Grid g = corridor(3);
  ReservationTable r(g);
  r.forbid_vertex(1, 3);
  const SearchResult s = space_time_astar(g, 1, 1, r);
  ASSERT_EQ(s.status, SearchStatus::kFound);
  EXPECT_EQ(allowed(g, r, s.path, 1, 1), "");
  EXPECT_EQ(s.path.size(), 5U);  // off the goal at t = 3, back at t = 4
}

TEST(SpaceTimeAStar, FindsNoPathWhenSomeoneRestsOnTheGoal) {
  const Grid g(4, 4);
  ReservationTable r(g);
  r.forbid_from(15, 50);
  const SearchResult s = space_time_astar(g, 0, 15, r);
  EXPECT_EQ(s.status, SearchStatus::kNoPath);
  EXPECT_TRUE(s.path.empty());
  EXPECT_EQ(s.expansions, 0U);
}

TEST(SpaceTimeAStar, FindsNoPathWhenTheStartIsTakenAtTimeZero) {
  const Grid g(4, 4);
  ReservationTable r(g);
  r.forbid_vertex(0, 0);
  EXPECT_EQ(space_time_astar(g, 0, 15, r).status, SearchStatus::kNoPath);
}

TEST(SpaceTimeAStar, FindsNoPathToAGoalInAnotherPiece) {
  const Grid g = parse_map("type octile\nheight 1\nwidth 3\nmap\n.@.\n");
  const ReservationTable none(g);
  const SearchResult s = space_time_astar(g, 0, 2, none);
  EXPECT_EQ(s.status, SearchStatus::kNoPath);
  EXPECT_EQ(s.expansions, 0U);
}

// The goal is reachable alone, but the only way to it is closed for ever from t = 4: the search must notice that
// nothing changes after the horizon and stop, rather than wait for ever.
TEST(SpaceTimeAStar, StopsWhenTheWayIsClosedForEver) {
  const Grid g = corridor(8);
  ReservationTable r(g);
  r.forbid_from(5, 4);   // someone parks in the corridor before the agent can get past (it would be there at t = 5)
  r.forbid_vertex(3, 2);  // and something timed, so the horizon isn't 0
  const SearchResult s = space_time_astar(g, 0, 7, r);
  EXPECT_EQ(s.status, SearchStatus::kNoPath);
  // At most (horizon + 2) states per passable cell.
  EXPECT_LE(s.expansions, (r.horizon() + 2U) * g.passable_count());
  EXPECT_GT(s.expansions, 0U);
}

TEST(SpaceTimeAStar, StopsAtTheExpansionLimit) {
  const Grid g(50, 50);
  ReservationTable r(g);
  r.forbid_from(g.id(Point{49, 49}), 0);  // unsolvable, so only the limit stops it early
  SearchLimits limits;
  limits.max_expansions = 100;
  // The goal isn't the forbidden cell: aim next to it; then wall the goal in for ever.
  const CellId goal = g.id(Point{48, 49});
  r.forbid_from(g.id(Point{48, 48}), 0);
  r.forbid_from(g.id(Point{47, 49}), 0);
  const SearchResult s = space_time_astar(g, 0, goal, r, limits);
  EXPECT_EQ(s.status, SearchStatus::kLimit);
  EXPECT_EQ(s.expansions, 100U);
  EXPECT_TRUE(s.path.empty());
  limits.max_expansions = 0;
  EXPECT_EQ(space_time_astar(g, 0, goal, r, limits).status, SearchStatus::kNoPath);
}

TEST(SpaceTimeAStar, StopsAtAPassedDeadlineBeforeExpandingAnything) {
  const Grid g(20, 20);
  const ReservationTable none(g);
  SearchLimits limits;
  limits.deadline = std::chrono::steady_clock::now() - std::chrono::seconds(1);
  const SearchResult s = space_time_astar(g, 0, 399, none, limits);
  EXPECT_EQ(s.status, SearchStatus::kLimit);
  EXPECT_EQ(s.expansions, 0U);
  // A start that is its own goal needs no expansion, so no deadline gets in the way.
  EXPECT_EQ(space_time_astar(g, 7, 7, none, limits).status, SearchStatus::kFound);
}

TEST(SpaceTimeAStar, GivesTheSamePathEveryTime) {
  const Grid g = random_map(40, 40, 20.0, 11);
  ReservationTable r(g);
  for (CellId c = 0; c < g.cell_count(); c += 13) {
    if (g.passable(c)) {
      r.forbid_vertex(c, c % 30);
    }
  }
  const auto index = std::make_shared<const PassableIndex>(g);
  const Components pieces = components(g);
  CellId start = 0;
  while (pieces.label[start] != pieces.largest || !r.vertex_free(start, 0)) {
    ++start;
  }
  auto goal = static_cast<CellId>(g.cell_count() - 1);
  while (pieces.label[goal] != pieces.largest) {
    --goal;
  }
  const DistanceTable h(g, index, goal);
  const SearchResult a = space_time_astar(g, start, h, r);
  const SearchResult b = space_time_astar(g, start, h, r);
  ASSERT_EQ(a.status, SearchStatus::kFound);
  EXPECT_EQ(a.path, b.path);
  EXPECT_EQ(a.expansions, b.expansions);
  EXPECT_EQ(a.generated, b.generated);
  EXPECT_EQ(allowed(g, r, a.path, start, goal), "");
}

}  // namespace
}  // namespace pathfinder
