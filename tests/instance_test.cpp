#include "pathfinder/instance.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <string>

#include "pathfinder/distance.hpp"

namespace pathfinder {
namespace {

const std::filesystem::path kData = std::filesystem::path(PATHFINDER_SOURCE_DIR) / "tests" / "data";

// The message check_instance or make_instance throws, or "" if it doesn't.
template <class F>
std::string refusal(F f) {
  try {
    f();
  } catch (const std::invalid_argument& e) {
    return e.what();
  }
  return "";
}

TEST(Instance, TakesTheScenariosFirstKEntries) {
  const Grid g = load_map(kData / "small.map");
  const Scenario s = load_scenario(kData / "small.scen");
  const Instance two = make_instance(g, s, 2);
  ASSERT_EQ(two.agents.size(), 2U);
  EXPECT_EQ(two.agents[0], (Agent{g.id(Point{0, 0}), g.id(Point{4, 3})}));
  EXPECT_EQ(two.agents[1], (Agent{g.id(Point{4, 0}), g.id(Point{0, 2})}));
  EXPECT_EQ(make_instance(g, s, 0).agents.size(), 0U);
  EXPECT_EQ(make_instance(g, s, 3).agents.size(), 3U);
  EXPECT_NE(refusal([&] { make_instance(g, s, 4); }).find("has 3 entries, not 4"), std::string::npos);
}

TEST(Instance, TheSmallScenariosLengthsAreItsBfsDistances) {
  const Grid g = load_map(kData / "small.map");
  const Scenario s = load_scenario(kData / "small.scen");
  for (const auto& e : s.entries) {
    EXPECT_EQ(distances(g, g.id(e.start))[g.id(e.goal)], static_cast<std::uint32_t>(e.optimal_length));
  }
}

TEST(Instance, RefusesAScenarioForAnotherMapSize) {
  const Grid g(5, 5);
  Scenario s;
  s.entries.push_back(ScenarioEntry{0, "m.map", 5, 4, Point{0, 0}, Point{1, 1}, 2});
  EXPECT_NE(refusal([&] { make_instance(g, s, 1); }).find("is for a 5x4 map, but the map is 5x5"), std::string::npos);
}

TEST(Instance, RefusesBlockedStartsAndGoals) {
  Grid g(3, 3);
  g.set_passable(Point{1, 1}, false);
  Scenario s;
  s.entries.push_back(ScenarioEntry{0, "m.map", 3, 3, Point{1, 1}, Point{0, 0}, 0});
  EXPECT_EQ(refusal([&] { make_instance(g, s, 1); }), "agent 0's start (1, 1) is blocked");
  s.entries[0] = ScenarioEntry{0, "m.map", 3, 3, Point{0, 0}, Point{1, 1}, 0};
  EXPECT_EQ(refusal([&] { make_instance(g, s, 1); }), "agent 0's goal (1, 1) is blocked");
}

TEST(Instance, RefusesSharedStartsOrSharedGoalsButNotAStartThatIsAnotherAgentsGoal) {
  const Grid g(4, 1);
  Instance shared_start{g, {Agent{0, 3}, Agent{0, 2}}};
  EXPECT_EQ(refusal([&] { check_instance(shared_start); }), "agent 1's start (0, 0) is also agent 0's start");
  Instance shared_goal{g, {Agent{0, 3}, Agent{1, 2}, Agent{2, 3}}};
  EXPECT_EQ(refusal([&] { check_instance(shared_goal); }), "agent 2's goal (3, 0) is also agent 0's goal");
  Instance crossing{g, {Agent{0, 1}, Agent{1, 0}}};
  EXPECT_EQ(refusal([&] { check_instance(crossing); }), "");
  Instance own{g, {Agent{2, 2}}};
  EXPECT_EQ(refusal([&] { check_instance(own); }), "");  // already at its goal
}

TEST(Instance, RefusesCellsOutsideTheGrid) {
  const Grid g(2, 2);
  Instance outside{g, {Agent{0, 4}}};
  EXPECT_NE(refusal([&] { check_instance(outside); }).find("agent 0's goal (cell 4) is outside the 2x2 grid"),
            std::string::npos);
}

TEST(Instance, FindsAgentsWhoseGoalIsInAnotherPiece) {
  const Grid g = parse_map("type octile\nheight 2\nwidth 5\nmap\n..@..\n..@..\n");
  const Instance inst{g, {Agent{g.id(Point{0, 0}), g.id(Point{1, 1})}, Agent{g.id(Point{1, 0}), g.id(Point{3, 0})},
                          Agent{g.id(Point{4, 1}), g.id(Point{0, 1})}, Agent{g.id(Point{4, 0}), g.id(Point{3, 1})}}};
  check_instance(inst);
  EXPECT_EQ(unreachable_agents(inst), (std::vector<std::size_t>{1, 2}));
}

}  // namespace
}  // namespace pathfinder
