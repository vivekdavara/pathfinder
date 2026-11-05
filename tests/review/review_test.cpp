// The day-1 review (an agent at commit 5ef5763, in its own worktree): each test here failed on that commit and passes
// with the fix that names it.

#include <gtest/gtest.h>

#include <filesystem>
#include <map>
#include <stdexcept>
#include <string>
#include <utility>

#include "pathfinder/generate.hpp"
#include "pathfinder/instance.hpp"
#include "pathfinder/movingai.hpp"

namespace pathfinder {
namespace {

const std::filesystem::path kData = std::filesystem::path(PATHFINDER_SOURCE_DIR) / "tests" / "data";

// src/instance.cpp make_instance: an entry whose point lies outside its (matching) width and height is turned into
// some other cell by grid.id() (y * width + x wraps), and check_instance can't tell, so the instance is accepted.
// DESIGN.md: "Coordinates must lie inside the stated width and height". Scenarios built in code (C++ or Python's
// ScenarioEntry constructor) never went through the parser that checks this.
TEST(ReviewInstance, MakeInstanceRefusesPointsOutsideTheStatedSize) {
  const Grid g(4, 4);
  for (const Point bad : {Point{4, 0}, Point{5, -1}, Point{-1, 1}, Point{0, 4}}) {
    Scenario s;
    s.entries.push_back(ScenarioEntry{0, "m.map", 4, 4, bad, Point{3, 3}, 1.0});
    EXPECT_THROW(make_instance(g, s, 1), std::invalid_argument) << "start (" << bad.x << ", " << bad.y << ")";
    s.entries[0] = ScenarioEntry{0, "m.map", 4, 4, Point{3, 3}, bad, 1.0};
    EXPECT_THROW(make_instance(g, s, 1), std::invalid_argument) << "goal (" << bad.x << ", " << bad.y << ")";
  }
}

// src/movingai.cpp rethrow_with_file: include/pathfinder/movingai.hpp says what() is "<file>: line N: ..." from the
// load_ functions, but the rethrown ParseError's constructor prefixes "line N: " again, giving
// "line N: <file>: line N: ...".
TEST(ReviewMovingAi, LoadErrorsReadFileThenLine) {
  const auto path = kData / "short_row.map";
  try {
    load_map(path);
    FAIL() << "short_row.map parsed";
  } catch (const ParseError& e) {
    EXPECT_EQ(std::string(e.what()).rfind(path.string() + ": line 6: ", 0), 0U) << e.what();
  }
}

// src/movingai.cpp read_file: opening a directory succeeds and the failed read isn't noticed (in.bad() stays false),
// so load_map reports a parse error of an empty file ("line 1: the file ended before the 'map' line") instead of the
// documented std::runtime_error for a file that can't be read. Any read failure is likewise taken as end of file.
TEST(ReviewMovingAi, ReadingADirectoryIsAReadErrorNotAParseError) {
  try {
    load_map(kData);
    FAIL() << "a directory loaded as a map";
  } catch (const ParseError& e) {
    FAIL() << "a ParseError: " << e.what();
  } catch (const std::runtime_error&) {
    SUCCEED();
  }
}

}  // namespace
}  // namespace pathfinder
