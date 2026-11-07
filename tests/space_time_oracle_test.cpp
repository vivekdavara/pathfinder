// Space-time A* against a brute-force oracle on thousands of small random problems: the set of cells the agent can
// be in at each time, computed layer by layer from the reservation table's own answers. It shares nothing with the
// search (no heuristic, no open list, no merging of late states) and stops at day 1's explicit bound, horizon +
// passable cells, so it also checks the claim that lets the search drop that bound.

#include <gtest/gtest.h>

#include <array>
#include <cstdlib>
#include <iostream>
#include <optional>
#include <vector>

#include "pathfinder/generate.hpp"
#include "pathfinder/reservation.hpp"
#include "pathfinder/rng.hpp"
#include "pathfinder/space_time_astar.hpp"
#include "search_checks.hpp"

namespace pathfinder {
namespace {

using testing::allowed;

// The earliest arrival of any path the table allows from start to goal (one the agent can stay at the goal after),
// trying every arrival up to `last`; nullopt if there is none by then.
std::optional<std::uint32_t> earliest_arrival(const Grid& g, const ReservationTable& r, CellId start, CellId goal,
                                              std::uint32_t last) {
  const auto can_stay = [&](std::uint32_t t) {
    return !r.forbidden_from(goal) && (!r.last_vertex_time(goal) || *r.last_vertex_time(goal) < t);
  };
  std::vector<char> here(g.cell_count(), 0);
  if (!g.passable(start) || !r.vertex_free(start, 0)) {
    return std::nullopt;
  }
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

CellId random_passable(const Grid& g, SplitMix64& rng) {
  for (;;) {
    const auto c = static_cast<CellId>(rng.below(g.cell_count()));
    if (g.passable(c)) {
      return c;
    }
  }
}

// A random walk of `steps` steps from a random passable cell (waits included): another agent's path.
Path random_walk(const Grid& g, SplitMix64& rng, std::size_t steps) {
  Path p{random_passable(g, rng)};
  std::array<CellId, 4> next{};
  for (std::size_t i = 0; i < steps; ++i) {
    const int n = g.neighbors(p.back(), next);
    const auto pick = rng.below(static_cast<std::uint64_t>(n) + 1);
    p.push_back(pick == static_cast<std::uint64_t>(n) ? p.back() : next[pick]);
  }
  return p;
}

// Random reservations of all three kinds: other agents' paths (dense, realistic) and loose entries.
ReservationTable random_table(const Grid& g, SplitMix64& rng) {
  ReservationTable r(g);
  const auto walks = rng.below(4);
  for (std::uint64_t i = 0; i < walks; ++i) {
    r.reserve_path(random_walk(g, rng, rng.below(15)));
  }
  const auto loose = rng.below(25);
  std::array<CellId, 4> next{};
  for (std::uint64_t i = 0; i < loose; ++i) {
    const CellId c = random_passable(g, rng);
    const auto t = static_cast<std::uint32_t>(rng.below(20));
    switch (rng.below(10)) {
      case 0:
        r.forbid_from(c, t);
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
  return r;
}

// Walls that go up early and stay (forbid_from at t = 0..2) on an open grid, and a few timed entries: the heuristic,
// which ignores them, is far from exact after the horizon, which is where the search merges late states. (The first
// generator alone never made the search re-open a late state with a lower time; this one does, so dropping that
// re-push fails here: day 2's mutation check.)
ReservationTable maze_table(const Grid& g, SplitMix64& rng) {
  ReservationTable r(g);
  const auto walls = rng.below(g.cell_count() / 2 + 1);
  for (std::uint64_t i = 0; i < walls; ++i) {
    r.forbid_from(static_cast<CellId>(rng.below(g.cell_count())), static_cast<std::uint32_t>(rng.below(3)));
  }
  const auto timed = rng.below(12);
  for (std::uint64_t i = 0; i < timed; ++i) {
    r.forbid_vertex(static_cast<CellId>(rng.below(g.cell_count())), static_cast<std::uint32_t>(rng.below(12)));
  }
  return r;
}

struct Tally {
  int found = 0;
  int none = 0;
  int slowed = 0;  // the answer is later than the distance alone: the reservations mattered
};

// One problem: the search must find the oracle's earliest arrival with a path the table allows, or report no path
// exactly when the oracle finds none up to the bound (and none twice as far).
void expect_same_as_oracle(const Grid& g, const ReservationTable& r, CellId start, CellId goal, Tally& tally,
                           int round) {
  const std::uint32_t bound = r.horizon() + static_cast<std::uint32_t>(g.passable_count());
  const auto expected = earliest_arrival(g, r, start, goal, bound);
  const SearchResult s = space_time_astar(g, start, goal, r);
  const auto where = ::testing::Message() << "round " << round << ": " << g.width() << "x" << g.height() << ", "
                                          << start << " -> " << goal << ", horizon " << r.horizon();
  if (!expected) {
    ASSERT_EQ(s.status, SearchStatus::kNoPath) << where;
    ASSERT_FALSE(earliest_arrival(g, r, start, goal, 2 * bound + 10).has_value()) << where;
    ++tally.none;
    return;
  }
  ASSERT_EQ(s.status, SearchStatus::kFound) << where;
  ASSERT_EQ(s.path.size(), *expected + 1U) << where;
  ASSERT_EQ(allowed(g, r, s.path, start, goal), "") << where;
  // The state bound the search's documentation promises.
  ASSERT_LE(s.expansions, (std::uint64_t{r.horizon()} + 2) * g.passable_count()) << where;
  const auto alone = space_time_astar(g, start, goal, ReservationTable(g));
  tally.slowed += alone.path.size() < s.path.size() ? 1 : 0;
  ++tally.found;
}

// PATHFINDER_ORACLE_ROUNDS runs more of each (a mutation hunt); the suite runs 3,000.
int rounds() {
  const char* env = std::getenv("PATHFINDER_ORACLE_ROUNDS");
  return env != nullptr ? std::atoi(env) : 3000;
}

TEST(SpaceTimeAStarOracle, AgreesOnOtherAgentsPathsAndLooseEntries) {
  SplitMix64 rng(20261107);
  Tally tally;
  for (int round = 0; round < rounds(); ++round) {
    const auto w = static_cast<std::int32_t>(2 + rng.below(8));
    const auto h = static_cast<std::int32_t>(1 + rng.below(8));
    const Grid g = random_map(w, h, static_cast<double>(rng.below(35)), rng.next());
    if (g.passable_count() == 0) {
      continue;
    }
    const ReservationTable r = random_table(g, rng);
    const CellId start = random_passable(g, rng);
    const CellId goal = random_passable(g, rng);
    expect_same_as_oracle(g, r, start, goal, tally, round);
    if (HasFatalFailure()) {
      return;
    }
  }
  // The generator must exercise every branch: paths, no paths, and reservations that cost time.
  EXPECT_GT(tally.found, 1000);
  EXPECT_GT(tally.none, 200);
  EXPECT_GT(tally.slowed, 200);
  std::cout << "[ oracle   ] " << tally.found << " found, " << tally.none << " with no path, " << tally.slowed
            << " slowed by reservations\n";
}

TEST(SpaceTimeAStarOracle, AgreesBehindWallsTheHeuristicDoesntSee) {
  SplitMix64 rng(99);
  Tally tally;
  for (int round = 0; round < rounds(); ++round) {
    const Grid g(static_cast<std::int32_t>(3 + rng.below(10)), static_cast<std::int32_t>(3 + rng.below(10)));
    const ReservationTable r = maze_table(g, rng);
    const auto start = static_cast<CellId>(rng.below(g.cell_count()));
    const auto goal = static_cast<CellId>(rng.below(g.cell_count()));
    expect_same_as_oracle(g, r, start, goal, tally, round);
    if (HasFatalFailure()) {
      return;
    }
  }
  EXPECT_GT(tally.found, 1000);
  EXPECT_GT(tally.none, 200);
  EXPECT_GT(tally.slowed, 300);
  std::cout << "[ oracle   ] " << tally.found << " found, " << tally.none << " with no path, " << tally.slowed
            << " slowed by reservations\n";
}

}  // namespace
}  // namespace pathfinder
