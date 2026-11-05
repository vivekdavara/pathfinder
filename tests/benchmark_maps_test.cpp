// The MovingAI MAPF benchmark set, when scripts/fetch_maps.py has fetched it into data/ (it isn't committed, so CI
// skips these tests and says why).

#include <gtest/gtest.h>

#include <algorithm>
#include <filesystem>
#include <map>
#include <string>
#include <vector>

#include "pathfinder/distance.hpp"
#include "pathfinder/instance.hpp"
#include "pathfinder/movingai.hpp"

namespace pathfinder {
namespace {

namespace fs = std::filesystem;
const fs::path kData = fs::path(PATHFINDER_SOURCE_DIR) / "data";

std::vector<fs::path> files(const fs::path& dir, const std::string& suffix) {
  std::vector<fs::path> out;
  for (const auto& entry : fs::directory_iterator(dir)) {
    if (entry.is_regular_file() && entry.path().extension() == suffix) {
      out.push_back(entry.path());
    }
  }
  std::sort(out.begin(), out.end());
  return out;
}

class BenchmarkMaps : public ::testing::Test {
 protected:
  void SetUp() override {
    if (!fs::is_directory(kData / "mapf-map") || !fs::is_directory(kData / "scen-random")) {
      GTEST_SKIP() << "no benchmark maps in " << kData << " (run: python3 -I scripts/fetch_maps.py)";
    }
  }
};

TEST_F(BenchmarkMaps, EveryMapParsesAndWritesBackTheSameGrid) {
  const auto maps = files(kData / "mapf-map", ".map");
  EXPECT_EQ(maps.size(), 33U);
  for (const auto& path : maps) {
    SCOPED_TRACE(path.filename().string());
    const Grid g = load_map(path);
    EXPECT_GT(g.passable_count(), 0U);
    EXPECT_EQ(parse_map(write_map(g)), g);
  }
}

TEST_F(BenchmarkMaps, EveryScenarioIsAValidSolvableInstanceOfAllItsEntries) {
  std::map<std::string, Grid> grids;
  std::map<std::string, Components> pieces;
  const auto scens = files(kData / "scen-random", ".scen");
  EXPECT_EQ(scens.size(), 825U);  // 25 per map
  std::size_t entries = 0;
  std::size_t lengths_checked = 0;
  for (const auto& path : scens) {
    SCOPED_TRACE(path.filename().string());
    const Scenario s = load_scenario(path);
    ASSERT_FALSE(s.entries.empty());
    const std::string map = s.entries.front().map;
    for (const auto& e : s.entries) {
      ASSERT_EQ(e.map, map);
    }
    if (grids.find(map) == grids.end()) {
      grids.emplace(map, load_map(kData / "mapf-map" / map));
      pieces.emplace(map, components(grids.at(map)));
    }
    const Grid& g = grids.at(map);
    const Instance inst = make_instance(g, s, s.entries.size());  // throws on a broken rule
    const Components& c = pieces.at(map);
    for (const Agent& a : inst.agents) {
      ASSERT_EQ(c.label[a.start], c.label[a.goal]) << "an unreachable goal";
    }
    entries += s.entries.size();
    // The stated optimal lengths are octile distances (8-connected, no corner cutting), rounded to 8 decimals:
    // the first entry of each file here (to keep `make check` quick); `make check-maps` compares all of them.
    const auto& first = s.entries.front();
    EXPECT_NEAR(octile_distances(g, g.id(first.start))[g.id(first.goal)], first.optimal_length, 1e-6);
    ++lengths_checked;
  }
  EXPECT_EQ(grids.size(), 33U);
  EXPECT_GT(entries, 0U);
  EXPECT_EQ(lengths_checked, scens.size());
}

}  // namespace
}  // namespace pathfinder
