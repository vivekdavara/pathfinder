#include <gtest/gtest.h>

#include <filesystem>
#include <string>

#include "pathfinder/generate.hpp"
#include "pathfinder/movingai.hpp"
#include "pathfinder/plan.hpp"
#include "pathfinder/rng.hpp"

namespace pathfinder {
namespace {

const std::filesystem::path kData = std::filesystem::path(PATHFINDER_SOURCE_DIR) / "tests" / "data";

int plan_error(const std::string& text, const Grid& grid, std::string* message = nullptr) {
  try {
    parse_plan(text, grid);
  } catch (const ParseError& e) {
    if (message != nullptr) {
      *message = e.what();
    }
    return e.line();
  }
  return -1;
}

TEST(PlanFile, ParsesTheExampleInDesignMd) {
  const Grid g(8, 8);
  const Plan plan = parse_plan(
      "version 1\nagents 3\n0 (1,4) (2,4) (3,4)\n1 (5,5) (5,5) (5,6)\n2 (0,0)\n", g);
  ASSERT_EQ(plan.size(), 3U);
  EXPECT_EQ(plan[0], (Path{g.id(Point{1, 4}), g.id(Point{2, 4}), g.id(Point{3, 4})}));
  EXPECT_EQ(plan[1], (Path{g.id(Point{5, 5}), g.id(Point{5, 5}), g.id(Point{5, 6})}));
  EXPECT_EQ(plan[2], (Path{0}));
}

TEST(PlanFile, AcceptsTabsCrlfAndTrailingBlankLinesAndNoAgents) {
  const Grid g(4, 4);
  EXPECT_EQ(parse_plan("version 1\r\nagents 1\r\n0\t(1,2)  (1,3)\r\n\r\n \n", g), (Plan{{9, 13}}));
  EXPECT_TRUE(parse_plan("version 1\nagents 0\n", g).empty());
}

TEST(PlanFile, EachErrorNamesItsLine) {
  const Grid g(4, 3);
  std::string m;
  EXPECT_EQ(plan_error("", g), 1);
  EXPECT_EQ(plan_error("version 2\nagents 0\n", g), 1);
  EXPECT_EQ(plan_error("version 1\n", g), 2);
  EXPECT_EQ(plan_error("version 1\nagents\n", g), 2);
  EXPECT_EQ(plan_error("version 1\nagents -1\n", g), 2);
  EXPECT_EQ(plan_error("version 1\nagents 1000001\n", g, &m), 2);
  EXPECT_NE(m.find("at most 1000000"), std::string::npos) << m;
  EXPECT_EQ(plan_error("version 1\nagents 2\n0 (0,0)\n", g, &m), 4);
  EXPECT_NE(m.find("ended after 1 of 2 agents"), std::string::npos) << m;
  EXPECT_EQ(plan_error("version 1\nagents 2\n1 (0,0)\n0 (1,0)\n", g, &m), 3);
  EXPECT_NE(m.find("expected agent 0's line"), std::string::npos) << m;
  EXPECT_EQ(plan_error("version 1\nagents 2\n0 (0,0)\n\n1 (1,0)\n", g), 4);  // no blank lines between agents
  EXPECT_EQ(plan_error("version 1\nagents 1\n0\n", g, &m), 3);
  EXPECT_NE(m.find("agent 0 has no cells"), std::string::npos) << m;
  EXPECT_EQ(plan_error("version 1\nagents 1\n0 (0,0)\n1 (1,0)\n", g, &m), 4);
  EXPECT_NE(m.find("unexpected text after the last of the 1 agents"), std::string::npos) << m;
}

TEST(PlanFile, CellsAreParenthesisedPairsOfWholeNumbersInsideTheGrid) {
  const Grid g(4, 3);
  const auto with_cell = [](const std::string& c) { return "version 1\nagents 1\n0 (0,0) " + c + "\n"; };
  EXPECT_EQ(plan_error(with_cell("(3,2)"), g), -1);
  for (const std::string bad : {"(1, 2)", "1,2", "(1,2", "1,2)", "(1;2)", "(-1,2)", "(1,+2)", "(,2)", "(1,)", "()",
                                "(1,2,3)", "(a,b)", "(1.0,2)"}) {
    std::string m;
    EXPECT_EQ(plan_error(with_cell(bad), g, &m), 3) << bad;
    EXPECT_NE(m.find("agent 0's cell at t=1 must be written (x,y)"), std::string::npos) << bad << ": " << m;
  }
  std::string m;
  // (4,0) would be id 4, which is the cell (0,1): out of the grid, not wrapped onto the next row.
  EXPECT_EQ(plan_error(with_cell("(4,0)"), g, &m), 3);
  EXPECT_NE(m.find("agent 0's cell (4,0) at t=1 is outside the 4x3 map"), std::string::npos) << m;
  EXPECT_EQ(plan_error(with_cell("(0,3)"), g), 3);
}

TEST(PlanFile, WritesTheCanonicalFormAndReadsItBack) {
  const Grid g(8, 8);
  const Plan plan{{g.id(Point{1, 4}), g.id(Point{2, 4})}, {g.id(Point{7, 7})}};
  const std::string text = write_plan(plan, g);
  EXPECT_EQ(text, "version 1\nagents 2\n0 (1,4) (2,4)\n1 (7,7)\n");
  EXPECT_EQ(parse_plan(text, g), plan);
  EXPECT_EQ(write_plan(Plan{}, g), "version 1\nagents 0\n");
}

TEST(PlanFile, WriteRefusesWhatCouldntBeReadBack) {
  const Grid g(2, 2);
  EXPECT_THROW(write_plan(Plan{{0, 1}, {}}, g), std::invalid_argument);
  EXPECT_THROW(write_plan(Plan{{0, 4}}, g), std::invalid_argument);
}

TEST(PlanFile, RandomPlansRoundTrip) {
  SplitMix64 rng(5);
  for (int round = 0; round < 200; ++round) {
    const Grid g = random_map(static_cast<std::int32_t>(1 + rng.below(30)), static_cast<std::int32_t>(1 + rng.below(30)),
                              0.0, rng.next());
    Plan plan(rng.below(8));
    for (auto& path : plan) {
      const auto length = 1 + rng.below(10);
      for (std::uint64_t t = 0; t < length; ++t) {
        path.push_back(static_cast<CellId>(rng.below(g.cell_count())));
      }
    }
    ASSERT_EQ(parse_plan(write_plan(plan, g), g), plan) << "round " << round;
  }
}

TEST(PlanFile, LoadsAFileAndNamesItInErrors) {
  const Grid g = load_map(kData / "small.map");
  const Plan plan = load_plan(kData / "small.plan", g);
  ASSERT_EQ(plan.size(), 2U);
  const Instance inst = make_instance(g, load_scenario(kData / "small.scen"), 2);
  const PlanCheck check = validate(inst, plan);
  EXPECT_TRUE(check.valid());
  EXPECT_EQ(check.sum_of_costs, 13U);
  EXPECT_EQ(check.makespan, 7U);
  try {
    load_plan(kData / "small.scen", g);
    FAIL() << "a scenario parsed as a plan";
  } catch (const ParseError& e) {
    EXPECT_EQ(e.line(), 2);
    EXPECT_EQ(std::string(e.what()).rfind((kData / "small.scen").string() + ": line 2: expected 'agents <k>'", 0), 0U)
        << e.what();
  }
}

}  // namespace
}  // namespace pathfinder
