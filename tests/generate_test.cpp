#include "pathfinder/generate.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>
#include <string_view>

#include "pathfinder/distance.hpp"
#include "pathfinder/rng.hpp"

namespace pathfinder {
namespace {

// FNV-1a, 64-bit: a fixed hash of generated text, so a change in the random stream or a layout fails here on every
// platform at once. The pinned values below were written from the computed ones.
std::uint64_t fnv1a(std::string_view s) {
  std::uint64_t h = 0xcbf29ce484222325ULL;
  for (const char c : s) {
    h ^= static_cast<unsigned char>(c);
    h *= 0x100000001b3ULL;
  }
  return h;
}

TEST(SplitMix64, MatchesTheReferenceOutputForSeed1234567) {
  SplitMix64 rng(1234567);
  const std::array<std::uint64_t, 5> expected{6457827717110365317ULL, 3203168211198807973ULL,
                                              9817491932198370423ULL, 4593380528125082431ULL,
                                              16408922859458223821ULL};
  for (const auto e : expected) {
    EXPECT_EQ(rng.next(), e);
  }
}

TEST(SplitMix64, BelowStaysInRangeAndHitsEveryValue) {
  SplitMix64 rng(7);
  std::array<int, 6> counts{};
  for (int i = 0; i < 6000; ++i) {
    const auto v = rng.below(6);
    ASSERT_LT(v, 6U);
    ++counts[v];
  }
  for (const int c : counts) {
    EXPECT_GT(c, 850);  // 1,000 expected; a standard deviation is about 29
    EXPECT_LT(c, 1150);
  }
  EXPECT_EQ(rng.below(1), 0U);
}

TEST(SplitMix64, BelowRejectsTheBiasedLowRange) {
  // For n = 3 * 2^62, 2^64 mod n = 2^62: a plain next() % n would map the top quarter of next()'s range onto
  // [0, 2^62), so half of all draws would land there instead of a third.
  const std::uint64_t quarter = std::uint64_t{1} << 62;
  const std::uint64_t n = 3 * quarter;
  SplitMix64 rng(99);
  int low = 0;
  for (int i = 0; i < 3000; ++i) {
    const auto v = rng.below(n);
    ASSERT_LT(v, n);
    if (v < quarter) {
      ++low;
    }
  }
  EXPECT_GT(low, 880);  // 1,000 expected with rejection, 1,500 without; a standard deviation is about 26
  EXPECT_LT(low, 1120);
}

TEST(RandomMap, BlocksExactlyTheRoundedShare) {
  EXPECT_EQ(random_map(10, 10, 20.0, 1).passable_count(), 80U);
  EXPECT_EQ(random_map(7, 3, 50.0, 1).passable_count(), 10U);  // 10.5 blocked rounds away from zero to 11
  EXPECT_EQ(random_map(32, 32, 0.0, 5), empty_map(32, 32));
  EXPECT_EQ(random_map(5, 4, 100.0, 5).passable_count(), 0U);
}

TEST(RandomMap, RefusesBadPercentagesAndSizes) {
  EXPECT_THROW(random_map(5, 5, -0.1, 1), std::invalid_argument);
  EXPECT_THROW(random_map(5, 5, 100.1, 1), std::invalid_argument);
  EXPECT_THROW(random_map(5, 5, std::numeric_limits<double>::quiet_NaN(), 1), std::invalid_argument);
  EXPECT_THROW(random_map(0, 5, 10.0, 1), std::invalid_argument);
}

TEST(RandomMap, IsTheSameForTheSameSeedAndDiffersOtherwise) {
  EXPECT_EQ(random_map(32, 32, 20.0, 1), random_map(32, 32, 20.0, 1));
  EXPECT_NE(random_map(32, 32, 20.0, 1), random_map(32, 32, 20.0, 2));
}

TEST(RandomMap, IsTheSameOnEveryPlatform) {
  EXPECT_EQ(fnv1a(write_map(random_map(32, 32, 20.0, 1))), 0xccba3a993502f7faULL);
  EXPECT_EQ(fnv1a(write_map(random_map(64, 48, 35.5, 42))), 0x751a555b243e850aULL);
}

TEST(WarehouseMap, LaysOutShelvesAislesAndTheBorder) {
  const Grid g = warehouse_map(WarehouseLayout{2, 3, 3, 1, 1, 1});
  EXPECT_EQ(write_map(g),
            "type octile\nheight 5\nwidth 13\nmap\n"
            ".............\n"
            ".@@@.@@@.@@@.\n"
            ".............\n"
            ".@@@.@@@.@@@.\n"
            ".............\n");
  const Grid wide = warehouse_map(WarehouseLayout{1, 2, 2, 2, 2, 2});
  EXPECT_EQ(write_map(wide),
            "type octile\nheight 6\nwidth 10\nmap\n"
            "..........\n"
            "..........\n"
            "..@@..@@..\n"
            "..@@..@@..\n"
            "..........\n"
            "..........\n");
}

TEST(WarehouseMap, HasTheStatedSizeAndOnePiece) {
  for (const WarehouseLayout l : {WarehouseLayout{}, WarehouseLayout{5, 2, 10, 2, 1, 1}, WarehouseLayout{1, 1, 1, 1, 1, 1},
                                  WarehouseLayout{4, 6, 3, 3, 2, 3}}) {
    const Grid g = warehouse_map(l);
    EXPECT_EQ(g.width(), 2 * l.margin + l.shelf_columns * l.shelf_width + (l.shelf_columns - 1) * l.aisle);
    EXPECT_EQ(g.height(), 2 * l.margin + l.shelf_rows * l.shelf_height + (l.shelf_rows - 1) * l.aisle);
    EXPECT_EQ(g.cell_count() - g.passable_count(),
              static_cast<std::size_t>(l.shelf_rows * l.shelf_columns * l.shelf_width * l.shelf_height));
    EXPECT_EQ(components(g).sizes.size(), 1U);
  }
}

TEST(WarehouseMap, RefusesZeroFieldsAndOversizedFloors) {
  for (int field = 0; field < 6; ++field) {
    WarehouseLayout l;
    std::array<std::int32_t*, 6> f{&l.shelf_rows, &l.shelf_columns, &l.shelf_width, &l.shelf_height, &l.aisle,
                                   &l.margin};
    *f[static_cast<std::size_t>(field)] = 0;
    EXPECT_THROW(warehouse_map(l), std::invalid_argument) << field;
  }
  EXPECT_THROW(warehouse_map(WarehouseLayout{1, 2000, 5, 1, 1, 1}), std::invalid_argument);
  EXPECT_NO_THROW(warehouse_map(WarehouseLayout{1, 1666, 5, 1, 1, 1}));  // 2 + 8,330 + 1,665 = 9,997 wide
}

TEST(WarehouseMap, IsTheSameOnEveryPlatform) {
  EXPECT_EQ(fnv1a(write_map(warehouse_map(WarehouseLayout{}))), 0x808a76f5f88e0d90ULL);
}

TEST(RandomScenario, DrawsDistinctStartsAndGoalsInTheLargestPiece) {
  const Grid g = random_map(40, 30, 30.0, 3);
  const Components pieces = components(g);
  const Scenario s = random_scenario(g, 200, 11, "r.map");
  ASSERT_EQ(s.entries.size(), 200U);
  std::set<CellId> starts;
  std::set<CellId> goals;
  for (const auto& e : s.entries) {
    EXPECT_EQ(e.map, "r.map");
    EXPECT_EQ(e.width, 40);
    EXPECT_EQ(e.height, 30);
    const CellId a = g.id(e.start);
    const CellId b = g.id(e.goal);
    EXPECT_EQ(pieces.label[a], pieces.largest);
    EXPECT_EQ(pieces.label[b], pieces.largest);
    EXPECT_NE(a, b);
    starts.insert(a);
    goals.insert(b);
    const double d = octile_distances(g, a)[b];
    EXPECT_EQ(e.optimal_length, d);
    EXPECT_EQ(e.bucket, static_cast<int>(std::floor(d / 4.0)));
    EXPECT_NE(distances(g, a)[b], kUnreachable);
  }
  EXPECT_EQ(starts.size(), 200U);
  EXPECT_EQ(goals.size(), 200U);
  // What it writes reads back and builds an instance.
  EXPECT_EQ(parse_scenario(write_scenario(s)), s);
}

TEST(RandomScenario, NeedsMoreCellsThanAgentsAndAWritableName) {
  const Grid g(3, 1);
  EXPECT_NO_THROW(random_scenario(g, 2, 1, "m.map"));
  EXPECT_THROW(random_scenario(g, 3, 1, "m.map"), std::invalid_argument);
  EXPECT_THROW(random_scenario(random_map(3, 3, 100.0, 1), 0, 1, "m.map"), std::invalid_argument);
  EXPECT_THROW(random_scenario(g, 1, 1, "my map"), std::invalid_argument);
  EXPECT_THROW(random_scenario(g, 1, 1, ""), std::invalid_argument);
  EXPECT_TRUE(random_scenario(g, 0, 1, "m.map").entries.empty());
}

TEST(RandomScenario, CanFillAllButOneCell) {
  // Two cells and one agent: its start and goal are the two cells; with 9 cells, 8 agents.
  const Grid pair(2, 1);
  for (std::uint64_t seed = 0; seed < 20; ++seed) {
    const Scenario s = random_scenario(pair, 1, seed, "p.map");
    EXPECT_NE(s.entries[0].start, s.entries[0].goal);
  }
  const Scenario full = random_scenario(Grid(3, 3), 8, 5, "f.map");
  std::set<std::pair<int, int>> starts;
  for (const auto& e : full.entries) {
    EXPECT_NE(e.start, e.goal);
    starts.insert({e.start.x, e.start.y});
  }
  EXPECT_EQ(starts.size(), 8U);
}

TEST(RandomScenario, StartsAreUniform) {
  // One agent on a 4-cell row, 4,000 seeds: each cell should start about 1,000 of them.
  const Grid g(4, 1);
  std::array<int, 4> counts{};
  for (std::uint64_t seed = 0; seed < 4000; ++seed) {
    ++counts[static_cast<std::size_t>(random_scenario(g, 1, seed, "u.map").entries[0].start.x)];
  }
  for (const int c : counts) {
    EXPECT_GT(c, 880);  // a standard deviation is about 27
    EXPECT_LT(c, 1120);
  }
}

TEST(RandomScenario, IsTheSameOnEveryPlatform) {
  const Grid g = warehouse_map(WarehouseLayout{});
  EXPECT_EQ(fnv1a(write_scenario(random_scenario(g, 50, 2026, "warehouse.map"))), 0x3e3c169c6dc3b76cULL);
}

}  // namespace
}  // namespace pathfinder
