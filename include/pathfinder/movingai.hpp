#pragma once

// The MovingAI benchmark file formats (movingai.com/benchmarks/formats.html), as DESIGN.md "Maps and scenarios"
// specifies them: what is accepted, what is an error, and the canonical form the writers produce.

#include <filesystem>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "pathfinder/grid.hpp"

namespace pathfinder {

// A file that doesn't follow the format. what() is "line N: ..." (or "<file>: line N: ..." from the load_
// functions); line() is the 1-based line, or one past the last line when the file ends too early.
class ParseError : public std::runtime_error {
 public:
  ParseError(int line, const std::string& message);
  int line() const { return line_; }

 private:
  int line_;
};

// Whether a map character is passable: '.', 'G' and 'S' are; '@', 'O', 'T' and 'W' aren't. Throws
// std::invalid_argument for any other character.
bool terrain_passable(char c);

// Parses a .map file's text. Throws ParseError.
Grid parse_map(std::string_view text);
// Reads and parses a .map file. Throws ParseError (with the file name) or std::runtime_error if it can't be read.
Grid load_map(const std::filesystem::path& path);
// The canonical text of a grid: "type octile", height, width, "map", then '.' for passable and '@' for blocked,
// '\n' line ends. parse_map(write_map(g)) == g.
std::string write_map(const Grid& grid);

// One line of a scenario: an agent's start and goal on a map.
struct ScenarioEntry {
  int bucket = 0;
  std::string map;  // the map's file name, as written in the scenario
  std::int32_t width = 0;
  std::int32_t height = 0;
  Point start;
  Point goal;
  double optimal_length = 0.0;
  friend bool operator==(const ScenarioEntry&, const ScenarioEntry&) = default;
};

struct Scenario {
  std::vector<ScenarioEntry> entries;
  friend bool operator==(const Scenario&, const Scenario&) = default;
};

// Whether a scenario may name this map: at least one character, and no spaces, tabs or other control characters
// (bytes 0x00-0x20 and 0x7f), so that it is exactly one field of a line. The parser, the writer and the generator
// all use this rule.
bool valid_map_name(std::string_view name);

// Parses a .scen file's text (version 1). Throws ParseError.
Scenario parse_scenario(std::string_view text);
// Reads and parses a .scen file. Throws ParseError (with the file name) or std::runtime_error if it can't be read.
Scenario load_scenario(const std::filesystem::path& path);
// "version 1", then one tab-separated line per entry; the optimal length in the shortest form that reads back
// exactly. parse_scenario(write_scenario(s)) == s.
std::string write_scenario(const Scenario& scenario);

// A whole file's contents. Throws std::runtime_error naming the file if it can't be opened or read.
std::string read_file(const std::filesystem::path& path);

}  // namespace pathfinder
