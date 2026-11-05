#include "pathfinder/movingai.hpp"

#include "text.hpp"

#include <array>
#include <charconv>
#include <climits>
#include <cmath>
#include <cstdio>
#include <memory>
#include <optional>
#include <system_error>

namespace pathfinder {
namespace {

using text::blank;
using text::digits;
using text::fields;
using text::Lines;
using text::shown;

// The largest optimal length a scenario may state. The writers print lengths in fixed notation, so a bound keeps
// that short; real benchmark lengths are at most a few thousand.
constexpr double kMaxOptimalLength = 1e9;

std::int32_t dimension(std::string_view what, std::string_view s, int line) {
  const auto value = digits(s);
  if (!value || *value < 1 || *value > kMaxDimension) {
    throw ParseError(line, std::string(what) + " must be a whole number 1.." + std::to_string(kMaxDimension) +
                               ", got '" + shown(s) + "'");
  }
  return static_cast<std::int32_t>(*value);
}

// An optimal length: digits, optionally a '.' and more digits; at most kMaxOptimalLength.
std::optional<double> length(std::string_view s) {
  const auto dot = s.find('.');
  const auto whole = s.substr(0, dot);
  const auto frac = dot == std::string_view::npos ? std::string_view{} : s.substr(dot + 1);
  const auto all_digits = [](std::string_view part) {
    for (const char c : part) {
      if (c < '0' || c > '9') {
        return false;
      }
    }
    return true;
  };
  if (whole.empty() || !all_digits(whole) || (dot != std::string_view::npos && (frac.empty() || !all_digits(frac)))) {
    return std::nullopt;
  }
  double value = 0.0;
  const auto [end, ec] = std::from_chars(s.data(), s.data() + s.size(), value, std::chars_format::fixed);
  if (ec != std::errc{} || end != s.data() + s.size() || !(value <= kMaxOptimalLength)) {
    return std::nullopt;
  }
  return value;
}

[[noreturn]] void rethrow_with_file(const ParseError& e, const std::filesystem::path& path) {
  throw ParseError(path.string(), e.line(), e.message());
}

}  // namespace

ParseError::ParseError(int line, const std::string& message)
    : std::runtime_error("line " + std::to_string(line) + ": " + message), line_(line), message_(message) {}

ParseError::ParseError(const std::string& file, int line, const std::string& message)
    : std::runtime_error(file + ": line " + std::to_string(line) + ": " + message), line_(line), message_(message) {}

bool terrain_passable(char c) {
  switch (c) {
    case '.':
    case 'G':
    case 'S':
      return true;
    case '@':
    case 'O':
    case 'T':
    case 'W':
      return false;
    default:
      throw std::invalid_argument("unknown terrain character '" + shown(std::string_view(&c, 1)) + "'");
  }
}

Grid parse_map(std::string_view text) {
  Lines lines(text);
  std::string_view line;
  std::optional<std::int32_t> height;
  std::optional<std::int32_t> width;
  bool typed = false;
  while (true) {
    if (!lines.next(line)) {
      throw ParseError(lines.number() + 1, "the file ended before the 'map' line");
    }
    const int n = lines.number();
    const auto f = fields(line);
    if (f.size() == 1 && f[0] == "map") {
      break;
    }
    if (f.size() != 2) {
      throw ParseError(n, "expected 'type octile', 'height <n>', 'width <n>' or 'map', got '" + shown(line) + "'");
    }
    if (f[0] == "type") {
      if (typed) {
        throw ParseError(n, "'type' is given twice");
      }
      if (f[1] != "octile") {
        throw ParseError(n, "map type '" + shown(f[1]) + "' isn't supported (only 'octile')");
      }
      typed = true;
    } else if (f[0] == "height" || f[0] == "width") {
      auto& slot = f[0] == "height" ? height : width;
      if (slot) {
        throw ParseError(n, "'" + std::string(f[0]) + "' is given twice");
      }
      slot = dimension(f[0], f[1], n);
    } else {
      throw ParseError(n, "unknown header field '" + shown(f[0]) + "'");
    }
  }
  const int map_line = lines.number();
  if (!typed) {
    throw ParseError(map_line, "'type' is missing before 'map'");
  }
  if (!height) {
    throw ParseError(map_line, "'height' is missing before 'map'");
  }
  if (!width) {
    throw ParseError(map_line, "'width' is missing before 'map'");
  }

  Grid grid(*width, *height);
  const auto w = static_cast<std::size_t>(*width);
  for (std::int32_t y = 0; y < *height; ++y) {
    if (!lines.next(line)) {
      throw ParseError(lines.number() + 1, "the file ended after " + std::to_string(y) + " of " +
                                               std::to_string(*height) + " rows");
    }
    if (line.size() != w) {
      throw ParseError(lines.number(), "row " + std::to_string(y) + " has " + std::to_string(line.size()) +
                                           " characters, expected " + std::to_string(w));
    }
    for (std::size_t x = 0; x < w; ++x) {
      bool passable = false;
      try {
        passable = terrain_passable(line[x]);
      } catch (const std::invalid_argument& e) {
        throw ParseError(lines.number(), std::string(e.what()) + " at column " + std::to_string(x + 1));
      }
      if (!passable) {
        grid.set_passable(Point{static_cast<std::int32_t>(x), y}, false);
      }
    }
  }
  while (lines.next(line)) {
    if (!blank(line)) {
      throw ParseError(lines.number(),
                       "unexpected text after the last of the " + std::to_string(*height) + " rows: '" + shown(line) +
                           "'");
    }
  }
  return grid;
}

Grid load_map(const std::filesystem::path& path) {
  const std::string text = read_file(path);
  try {
    return parse_map(text);
  } catch (const ParseError& e) {
    rethrow_with_file(e, path);
  }
}

std::string write_map(const Grid& grid) {
  std::string out = "type octile\nheight " + std::to_string(grid.height()) + "\nwidth " +
                    std::to_string(grid.width()) + "\nmap\n";
  out.reserve(out.size() + grid.cell_count() + static_cast<std::size_t>(grid.height()));
  for (std::int32_t y = 0; y < grid.height(); ++y) {
    for (std::int32_t x = 0; x < grid.width(); ++x) {
      out += grid.passable(Point{x, y}) ? '.' : '@';
    }
    out += '\n';
  }
  return out;
}

bool valid_map_name(std::string_view name) {
  if (name.empty()) {
    return false;
  }
  for (const char c : name) {
    const auto b = static_cast<unsigned char>(c);
    if (b <= 0x20 || b == 0x7f) {
      return false;
    }
  }
  return true;
}

Scenario parse_scenario(std::string_view text) {
  Lines lines(text);
  std::string_view line;
  if (!lines.next(line)) {
    throw ParseError(1, "the file is empty; expected 'version 1'");
  }
  {
    const auto f = fields(line);
    if (f.size() != 2 || f[0] != "version" || (f[1] != "1" && f[1] != "1.0")) {
      throw ParseError(1, "expected 'version 1', got '" + shown(line) + "'");
    }
  }
  Scenario scenario;
  while (lines.next(line)) {
    if (blank(line)) {
      continue;
    }
    const int n = lines.number();
    const auto f = fields(line);
    if (f.size() != 9) {
      throw ParseError(n, "expected 9 fields (bucket, map, width, height, start x, start y, goal x, goal y, "
                          "optimal length), got " +
                              std::to_string(f.size()));
    }
    ScenarioEntry e;
    const auto bucket = digits(f[0]);
    if (!bucket || *bucket > INT_MAX) {
      throw ParseError(n, "bucket must be a whole number, got '" + shown(f[0]) + "'");
    }
    e.bucket = static_cast<int>(*bucket);
    if (!valid_map_name(f[1])) {
      throw ParseError(n, "the map name '" + shown(f[1]) + "' has a control character");
    }
    e.map = std::string(f[1]);
    e.width = dimension("width", f[2], n);
    e.height = dimension("height", f[3], n);
    const auto coordinate = [&](std::string_view name, std::string_view s, std::int32_t limit,
                                std::string_view limit_name) {
      const auto value = digits(s);
      if (!value) {
        throw ParseError(n, std::string(name) + " must be a whole number, got '" + shown(s) + "'");
      }
      if (*value >= limit) {
        throw ParseError(n, std::string(name) + " " + std::to_string(*value) + " is outside the map's " +
                                std::string(limit_name) + " " + std::to_string(limit));
      }
      return static_cast<std::int32_t>(*value);
    };
    e.start.x = coordinate("start x", f[4], e.width, "width");
    e.start.y = coordinate("start y", f[5], e.height, "height");
    e.goal.x = coordinate("goal x", f[6], e.width, "width");
    e.goal.y = coordinate("goal y", f[7], e.height, "height");
    const auto optimal = length(f[8]);
    if (!optimal) {
      throw ParseError(n, "optimal length must be a decimal number 0..1e9 (digits, optionally a '.' and digits), "
                          "got '" +
                              shown(f[8]) + "'");
    }
    e.optimal_length = *optimal;
    scenario.entries.push_back(std::move(e));
  }
  return scenario;
}

Scenario load_scenario(const std::filesystem::path& path) {
  const std::string text = read_file(path);
  try {
    return parse_scenario(text);
  } catch (const ParseError& e) {
    rethrow_with_file(e, path);
  }
}

std::string write_scenario(const Scenario& scenario) {
  std::string out = "version 1\n";
  std::size_t i = 0;
  for (const auto& e : scenario.entries) {
    const auto refuse = [&](const std::string& why) {
      throw std::invalid_argument("scenario entry " + std::to_string(i) + " can't be written: " + why);
    };
    if (e.bucket < 0) {
      refuse("negative bucket");
    }
    if (!valid_map_name(e.map)) {
      refuse("the map name must be non-empty with no spaces, tabs or control characters");
    }
    if (e.width < 1 || e.width > kMaxDimension || e.height < 1 || e.height > kMaxDimension) {
      refuse("width and height must be 1.." + std::to_string(kMaxDimension));
    }
    for (const Point p : {e.start, e.goal}) {
      if (p.x < 0 || p.y < 0 || p.x >= e.width || p.y >= e.height) {
        refuse("a start or goal is outside the stated width and height");
      }
    }
    if (!(e.optimal_length >= 0.0 && e.optimal_length <= kMaxOptimalLength)) {
      refuse("the optimal length must be 0..1e9");
    }
    // -0.0 passes the range check but would be written "-0", which the parser refuses; and the shortest fixed form
    // of a tiny length is long (the smallest double, 5e-324, takes 326 characters), so the buffer holds any value
    // in range. Both from the day-1 review.
    const double length = e.optimal_length == 0.0 ? 0.0 : e.optimal_length;
    std::array<char, 512> buf{};
    const auto [end, ec] = std::to_chars(buf.data(), buf.data() + buf.size(), length, std::chars_format::fixed);
    if (ec != std::errc{}) {
      refuse("the optimal length didn't format");
    }
    out += std::to_string(e.bucket) + '\t' + e.map + '\t' + std::to_string(e.width) + '\t' +
           std::to_string(e.height) + '\t' + std::to_string(e.start.x) + '\t' + std::to_string(e.start.y) + '\t' +
           std::to_string(e.goal.x) + '\t' + std::to_string(e.goal.y) + '\t' + std::string(buf.data(), end) + '\n';
    ++i;
  }
  return out;
}

std::string read_file(const std::filesystem::path& path) {
  // C stdio rather than an ifstream: an ifstream reports a failed read (a directory, an I/O error) as an early end
  // of file, so a broken file would be parsed as a short one; ferror() tells the two apart.
  struct Closer {
    void operator()(std::FILE* f) const { std::fclose(f); }
  };
  const std::unique_ptr<std::FILE, Closer> file(std::fopen(path.string().c_str(), "rb"));
  if (!file) {
    throw std::runtime_error(path.string() + ": can't open the file");
  }
  std::string text;
  std::array<char, 1 << 16> buf{};
  while (true) {
    const std::size_t n = std::fread(buf.data(), 1, buf.size(), file.get());
    text.append(buf.data(), n);
    if (n < buf.size()) {
      break;
    }
  }
  if (std::ferror(file.get()) != 0) {
    throw std::runtime_error(path.string() + ": can't read the file");
  }
  return text;
}

}  // namespace pathfinder
