#include "pathfinder/movingai.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <string>

namespace pathfinder {
namespace {

const std::filesystem::path kData = std::filesystem::path(PATHFINDER_SOURCE_DIR) / "tests" / "data";

// The line a ParseError names, or -1 if parsing succeeded; message gets what().
template <class Parse>
int error_line(Parse parse, const std::string& text, std::string* message = nullptr) {
  try {
    parse(text);
  } catch (const ParseError& e) {
    if (message != nullptr) {
      *message = e.what();
    }
    return e.line();
  }
  return -1;
}

int map_error(const std::string& text, std::string* message = nullptr) {
  return error_line([](const std::string& t) { return parse_map(t); }, text, message);
}

int scen_error(const std::string& text, std::string* message = nullptr) {
  return error_line([](const std::string& t) { return parse_scenario(t); }, text, message);
}

TEST(Terrain, FollowsMovingAisLegend) {
  for (const char c : {'.', 'G', 'S'}) {
    EXPECT_TRUE(terrain_passable(c)) << c;
  }
  for (const char c : {'@', 'O', 'T', 'W'}) {
    EXPECT_FALSE(terrain_passable(c)) << c;
  }
  for (const char c : {' ', 'g', 'x', '\t', '\r', '#', '0'}) {
    EXPECT_THROW(terrain_passable(c), std::invalid_argument) << static_cast<int>(c);
  }
}

TEST(MapFormat, ParsesTheExampleInDesignMd) {
  const Grid g = parse_map("type octile\nheight 4\nwidth 6\nmap\n......\n.@@...\n.@@.T.\n......\n");
  EXPECT_EQ(g.width(), 6);
  EXPECT_EQ(g.height(), 4);
  EXPECT_EQ(g.passable_count(), 19U);
  EXPECT_FALSE(g.passable(Point{1, 1}));
  EXPECT_FALSE(g.passable(Point{2, 2}));
  EXPECT_FALSE(g.passable(Point{4, 2}));
  EXPECT_TRUE(g.passable(Point{3, 2}));
  EXPECT_TRUE(g.passable(Point{5, 3}));
}

TEST(MapFormat, ReadsEveryTerrainCharacterFromAFile) {
  // tests/data/terrain.map has each of the seven characters in its first row.
  const Grid g = load_map(kData / "terrain.map");
  ASSERT_EQ(g.width(), 7);
  ASSERT_EQ(g.height(), 3);
  const std::string row0 = ".GS@OTW";
  for (std::int32_t x = 0; x < 7; ++x) {
    EXPECT_EQ(g.passable(Point{x, 0}), terrain_passable(row0[static_cast<std::size_t>(x)])) << x;
  }
  EXPECT_EQ(g.passable_count(), 3U + 7U + 7U);
}

TEST(MapFormat, AcceptsCrlfLineEndsNoFinalNewlineTrailingBlankLinesAndHeaderInAnyOrder) {
  const Grid expected = parse_map("type octile\nheight 2\nwidth 3\nmap\n.@.\n...\n");
  EXPECT_EQ(parse_map("type octile\r\nheight 2\r\nwidth 3\r\nmap\r\n.@.\r\n...\r\n"), expected);
  EXPECT_EQ(parse_map("type octile\nheight 2\nwidth 3\nmap\n.@.\n..."), expected);
  EXPECT_EQ(parse_map("type octile\nheight 2\nwidth 3\nmap\n.@.\n...\n\n  \n\t\n"), expected);
  EXPECT_EQ(parse_map("width 3\ntype octile\nheight 2\nmap\n.@.\n...\n"), expected);
  EXPECT_EQ(parse_map("type   octile\nheight\t2\nwidth 3 \nmap\n.@.\n...\n"), expected);
}

TEST(MapFormat, EachHeaderErrorNamesItsLine) {
  std::string m;
  EXPECT_EQ(map_error("", &m), 1);
  EXPECT_NE(m.find("before the 'map' line"), std::string::npos) << m;
  EXPECT_EQ(map_error("type octile\nheight 2\nwidth 3\n"), 4);
  EXPECT_EQ(map_error("type octile\nheight 2\nmap\n...\n...\n", &m), 3);
  EXPECT_NE(m.find("'width' is missing"), std::string::npos) << m;
  EXPECT_EQ(map_error("height 2\nwidth 3\nmap\n...\n...\n", &m), 3);
  EXPECT_NE(m.find("'type' is missing"), std::string::npos) << m;
  EXPECT_EQ(map_error("type octile\nwidth 3\nmap\n"), 3);
  EXPECT_EQ(map_error("type grid\nheight 2\nwidth 3\nmap\n", &m), 1);
  EXPECT_NE(m.find("only 'octile'"), std::string::npos) << m;
  EXPECT_EQ(map_error("type octile\ntype octile\nheight 2\nwidth 3\nmap\n", &m), 2);
  EXPECT_NE(m.find("given twice"), std::string::npos) << m;
  EXPECT_EQ(map_error("type octile\nheight 2\nheight 2\nwidth 3\nmap\n"), 3);
  EXPECT_EQ(map_error("type octile\nheight 2\nwidth 3\ncolour blue\nmap\n", &m), 4);
  EXPECT_NE(m.find("unknown header field 'colour'"), std::string::npos) << m;
  EXPECT_EQ(map_error("type octile\nheight 2 3\nwidth 3\nmap\n"), 2);
  EXPECT_EQ(map_error("\ntype octile\nheight 2\nwidth 3\nmap\n"), 1);
  EXPECT_EQ(map_error("type octile\nheight 2\nwidth 3\nmaps\n"), 4);
}

TEST(MapFormat, DimensionsAreDigitsOnlyWithinTheLimits) {
  const auto with_height = [](const std::string& h) {
    return "type octile\nheight " + h + "\nwidth 1\nmap\n.\n";
  };
  EXPECT_EQ(map_error(with_height("1")), -1);
  for (const std::string bad : {"0", "-1", "+1", "1.0", "0x1", "10001", "99999999999999999999", "", "1a"}) {
    std::string m;
    const int line = map_error(with_height(bad), &m);
    EXPECT_EQ(line, 2) << "height '" << bad << "': " << m;
  }
  std::string m;
  EXPECT_EQ(map_error("type octile\nheight 1\nwidth 10001\nmap\n.\n", &m), 3);
  EXPECT_NE(m.find("width must be a whole number 1..10000"), std::string::npos) << m;
}

TEST(MapFormat, RowsMustHaveExactlyTheStatedSizeAndKnownCharacters) {
  std::string m;
  EXPECT_EQ(map_error("type octile\nheight 2\nwidth 3\nmap\n...\n..\n", &m), 6);
  EXPECT_NE(m.find("row 1 has 2 characters, expected 3"), std::string::npos) << m;
  EXPECT_EQ(map_error("type octile\nheight 2\nwidth 3\nmap\n....\n...\n"), 5);
  EXPECT_EQ(map_error("type octile\nheight 2\nwidth 3\nmap\n... \n...\n"), 5);  // a trailing space
  EXPECT_EQ(map_error("type octile\nheight 2\nwidth 3\nmap\n...\n", &m), 6);
  EXPECT_NE(m.find("ended after 1 of 2 rows"), std::string::npos) << m;
  EXPECT_EQ(map_error("type octile\nheight 2\nwidth 3\nmap\n...\n.x.\n", &m), 6);
  EXPECT_NE(m.find("'x' at column 2"), std::string::npos) << m;
  EXPECT_EQ(map_error(std::string("type octile\nheight 1\nwidth 3\nmap\n.\x01.\n"), &m), 5);
  EXPECT_NE(m.find("'\\x01'"), std::string::npos) << m;
  EXPECT_EQ(map_error("type octile\nheight 1\nwidth 3\nmap\n.\r.\n"), 5);  // a '\r' inside a row
}

TEST(MapFormat, RefusesTextAfterTheLastRow) {
  std::string m;
  EXPECT_EQ(map_error("type octile\nheight 1\nwidth 3\nmap\n...\n\n...\n", &m), 7);
  EXPECT_NE(m.find("after the last of the 1 rows"), std::string::npos) << m;
}

TEST(MapFormat, WritesTheCanonicalFormAndReadsItBack) {
  Grid g(4, 3);
  g.set_passable(Point{0, 0}, false);
  g.set_passable(Point{3, 2}, false);
  g.set_passable(Point{1, 1}, false);
  const std::string text = write_map(g);
  EXPECT_EQ(text, "type octile\nheight 3\nwidth 4\nmap\n@...\n.@..\n...@\n");
  EXPECT_EQ(parse_map(text), g);
  // Parsing then writing a file in another form gives the canonical form.
  EXPECT_EQ(write_map(load_map(kData / "terrain.map")),
            "type octile\nheight 3\nwidth 7\nmap\n...@@@@\n.......\n.......\n");
}

TEST(MapFormat, LoadNamesTheFileInErrors) {
  try {
    load_map(kData / "short_row.map");
    FAIL() << "short_row.map parsed";
  } catch (const ParseError& e) {
    EXPECT_EQ(e.line(), 6);
    EXPECT_NE(std::string(e.what()).find("short_row.map: line 6: row 1 has"), std::string::npos) << e.what();
  }
  EXPECT_THROW(load_map(kData / "no_such_file.map"), std::runtime_error);
}

TEST(ScenarioFormat, ParsesTabsOrSpacesAndBothVersionSpellings) {
  const Scenario s = parse_scenario(
      "version 1\n"
      "0\tmaze-32-32-2.map\t32\t32\t11\t29\t14\t30\t4\n"
      "3 maze-32-32-2.map 32 32 0 0 31 31 5.82842712\n");
  ASSERT_EQ(s.entries.size(), 2U);
  const auto& e = s.entries[0];
  EXPECT_EQ(e.bucket, 0);
  EXPECT_EQ(e.map, "maze-32-32-2.map");
  EXPECT_EQ(e.width, 32);
  EXPECT_EQ(e.height, 32);
  EXPECT_EQ(e.start, (Point{11, 29}));
  EXPECT_EQ(e.goal, (Point{14, 30}));
  EXPECT_EQ(e.optimal_length, 4.0);
  EXPECT_EQ(s.entries[1].bucket, 3);
  EXPECT_EQ(s.entries[1].goal, (Point{31, 31}));
  EXPECT_EQ(s.entries[1].optimal_length, 5.82842712);
  EXPECT_EQ(parse_scenario("version 1.0\r\n0\tm.map\t2\t2\t0\t0\t1\t1\t2\r\n\r\n"),
            parse_scenario("version 1\n0 m.map 2 2 0 0 1 1 2\n"));
  EXPECT_TRUE(parse_scenario("version 1\n").entries.empty());
}

TEST(ScenarioFormat, EachErrorNamesItsLine) {
  const std::string v = "version 1\n";
  std::string m;
  EXPECT_EQ(scen_error("", &m), 1);
  EXPECT_EQ(scen_error("version 2\n"), 1);
  EXPECT_EQ(scen_error("0\tm.map\t2\t2\t0\t0\t1\t1\t2\n"), 1);  // no version line
  EXPECT_EQ(scen_error(v + "0\tm.map\t2\t2\t0\t0\t1\t1\n", &m), 2);
  EXPECT_NE(m.find("expected 9 fields"), std::string::npos) << m;
  EXPECT_EQ(scen_error(v + "0\tm.map\t2\t2\t0\t0\t1\t1\t2\t9\n"), 2);
  EXPECT_EQ(scen_error(v + "\n0\tm.map\t2\t2\t0\t0\t1\t1\t2\n-1\tm.map\t2\t2\t0\t0\t1\t1\t2\n", &m), 4);
  EXPECT_NE(m.find("bucket"), std::string::npos) << m;
  EXPECT_EQ(scen_error(v + "0\tm.map\t0\t2\t0\t0\t1\t1\t2\n"), 2);
  EXPECT_EQ(scen_error(v + "0\tm.map\t2\t2\t2\t0\t1\t1\t2\n", &m), 2);
  EXPECT_NE(m.find("start x 2 is outside the map's width 2"), std::string::npos) << m;
  EXPECT_EQ(scen_error(v + "0\tm.map\t2\t2\t0\t0\t1\t5\t2\n", &m), 2);
  EXPECT_NE(m.find("goal y 5 is outside the map's height 2"), std::string::npos) << m;
  EXPECT_EQ(scen_error(v + "0\tm.map\t2\t2\t0\t-0\t1\t1\t2\n"), 2);
  // A '\r' inside a line isn't a line end, and it can't be part of a map name (found by the robustness test: the
  // parser accepted it, then the writer refused to write the scenario back).
  EXPECT_EQ(scen_error(v + "0\tm\r.map\t2\t2\t0\t0\t1\t1\t2\n", &m), 2);
  EXPECT_NE(m.find("the map name 'm\\x0d.map' has a control character"), std::string::npos) << m;
  EXPECT_EQ(scen_error(v + std::string("0\tm\0.map\t2\t2\t0\t0\t1\t1\t2\n", 23)), 2);
  EXPECT_EQ(scen_error(v + "0\tm\x7f.map\t2\t2\t0\t0\t1\t1\t2\n"), 2);
  EXPECT_EQ(scen_error(v + "0\tcarte-\xc3\xa9t\xc3\xa9.map\t2\t2\t0\t0\t1\t1\t2\n"), -1);  // UTF-8 is fine
}

TEST(ScenarioFormat, MapNamesAreOneFieldWithoutControlCharacters) {
  EXPECT_TRUE(valid_map_name("maze-32-32-2.map"));
  EXPECT_TRUE(valid_map_name("a"));
  EXPECT_FALSE(valid_map_name(""));
  for (const char c : {' ', '\t', '\r', '\n', '\0', '\x1f', '\x7f'}) {
    EXPECT_FALSE(valid_map_name(std::string("a") + c + "b")) << static_cast<int>(c);
  }
}

TEST(ScenarioFormat, OptimalLengthsAreDecimalsWithoutSignsOrExponents) {
  const auto with_length = [](const std::string& l) { return "version 1\n0\tm.map\t2\t2\t0\t0\t1\t1\t" + l + "\n"; };
  for (const std::string good : {"0", "2", "2.5", "5.82842712", "1000000000", "007"}) {
    EXPECT_EQ(scen_error(with_length(good)), -1) << good;
  }
  for (const std::string bad : {"-1", "+1", "1e3", ".5", "5.", "1.2.3", "inf", "nan", "1000000001", "0x10", "1,5"}) {
    EXPECT_EQ(scen_error(with_length(bad)), 2) << bad;
  }
}

TEST(ScenarioFormat, WritesTabSeparatedAndReadsItBackExactly) {
  Scenario s;
  s.entries.push_back(ScenarioEntry{0, "a.map", 32, 16, Point{1, 2}, Point{30, 15}, 4.0});
  s.entries.push_back(ScenarioEntry{7, "a.map", 32, 16, Point{0, 0}, Point{31, 0}, 5.82842712});
  s.entries.push_back(ScenarioEntry{2, "a.map", 32, 16, Point{5, 5}, Point{5, 6}, 0.1});
  const std::string text = write_scenario(s);
  EXPECT_EQ(text,
            "version 1\n"
            "0\ta.map\t32\t16\t1\t2\t30\t15\t4\n"
            "7\ta.map\t32\t16\t0\t0\t31\t0\t5.82842712\n"
            "2\ta.map\t32\t16\t5\t5\t5\t6\t0.1\n");
  EXPECT_EQ(parse_scenario(text), s);
}

TEST(ScenarioFormat, WriteRefusesWhatParseWouldReject) {
  const ScenarioEntry good{0, "a.map", 4, 4, Point{0, 0}, Point{3, 3}, 6.0};
  const auto refused = [](ScenarioEntry e) {
    Scenario s;
    s.entries.push_back(std::move(e));
    try {
      write_scenario(s);
    } catch (const std::invalid_argument&) {
      return true;
    }
    return false;
  };
  EXPECT_FALSE(refused(good));
  auto e = good;
  e.map = "my map.map";
  EXPECT_TRUE(refused(e));
  e.map = "";
  EXPECT_TRUE(refused(e));
  e.map = "a\rb.map";
  EXPECT_TRUE(refused(e));
  e = good;
  e.bucket = -1;
  EXPECT_TRUE(refused(e));
  e = good;
  e.goal = Point{4, 0};
  EXPECT_TRUE(refused(e));
  e = good;
  e.width = 0;
  EXPECT_TRUE(refused(e));
  e = good;
  e.optimal_length = -0.5;
  EXPECT_TRUE(refused(e));
  e.optimal_length = 2e9;
  EXPECT_TRUE(refused(e));
}

TEST(ScenarioFormat, LoadsAFileAndNamesItInErrors) {
  const Scenario s = load_scenario(kData / "small.scen");
  ASSERT_EQ(s.entries.size(), 3U);
  EXPECT_EQ(s.entries[2].map, "small.map");
  try {
    parse_scenario("version 1\nx\n");
    FAIL();
  } catch (const ParseError& e) {
    EXPECT_EQ(std::string(e.what()).rfind("line 2: ", 0), 0U) << e.what();
  }
  EXPECT_THROW(load_scenario(kData / "no_such_file.scen"), std::runtime_error);
}

}  // namespace
}  // namespace pathfinder
