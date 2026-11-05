// The plan file format (DESIGN.md "Plan files").

#include <algorithm>
#include <stdexcept>
#include <string>

#include "pathfinder/movingai.hpp"
#include "pathfinder/plan.hpp"
#include "text.hpp"

namespace pathfinder {
namespace {

using text::blank;
using text::digits;
using text::fields;
using text::Lines;
using text::shown;

// What parse_plan reserves before it has seen the agents' lines, so a damaged count can't allocate gigabytes.
constexpr std::size_t kReserveAtMost = 1 << 16;

// "(x,y)" with x and y digits only, inside the grid.
CellId cell(std::string_view token, const Grid& grid, std::size_t agent, std::size_t t, int line) {
  const auto comma = token.find(',');
  const bool shaped = token.size() >= 5 && token.front() == '(' && token.back() == ')' && comma != std::string::npos;
  const auto x = shaped ? digits(token.substr(1, comma - 1)) : std::nullopt;
  const auto y = shaped ? digits(token.substr(comma + 1, token.size() - comma - 2)) : std::nullopt;
  if (!x || !y) {
    throw ParseError(line, "agent " + std::to_string(agent) + "'s cell at t=" + std::to_string(t) +
                               " must be written (x,y) with whole numbers, got '" + shown(token) + "'");
  }
  if (*x >= grid.width() || *y >= grid.height()) {
    throw ParseError(line, "agent " + std::to_string(agent) + "'s cell (" + std::to_string(*x) + "," +
                               std::to_string(*y) + ") at t=" + std::to_string(t) + " is outside the " +
                               std::to_string(grid.width()) + "x" + std::to_string(grid.height()) + " map");
  }
  return grid.id(Point{static_cast<std::int32_t>(*x), static_cast<std::int32_t>(*y)});
}

}  // namespace

Plan parse_plan(std::string_view text, const Grid& grid) {
  Lines lines(text);
  std::string_view line;
  if (!lines.next(line) || fields(line) != std::vector<std::string_view>{"version", "1"}) {
    throw ParseError(1, "expected 'version 1'");
  }
  if (!lines.next(line)) {
    throw ParseError(2, "the file ended before the 'agents' line");
  }
  const auto header = fields(line);
  const auto count = header.size() == 2 && header[0] == "agents" ? digits(header[1]) : std::nullopt;
  // At most one agent per cell: no instance on this grid has more (starts are distinct), and write_plan refuses the
  // same. (A fixed bound of a million, the first version, refused plans write_plan had written: the day-1 second
  // review.)
  if (!count || static_cast<std::size_t>(*count) > grid.cell_count()) {
    throw ParseError(2, "expected 'agents <k>' with k at most the grid's " + std::to_string(grid.cell_count()) +
                            " cells, got '" + shown(line) + "'");
  }
  const auto k = static_cast<std::size_t>(*count);
  Plan plan;
  plan.reserve(std::min(k, kReserveAtMost));
  for (std::size_t i = 0; i < k; ++i) {
    if (!lines.next(line)) {
      throw ParseError(lines.number() + 1,
                       "the file ended after " + std::to_string(i) + " of " + std::to_string(k) + " agents");
    }
    const int n = lines.number();
    const auto f = fields(line);
    const auto number = f.empty() ? std::nullopt : digits(f[0]);
    if (!number || static_cast<std::size_t>(*number) != i) {
      throw ParseError(n, "expected agent " + std::to_string(i) + "'s line, got '" + shown(line) + "'");
    }
    if (f.size() < 2) {
      throw ParseError(n, "agent " + std::to_string(i) + " has no cells");
    }
    Path path;
    path.reserve(f.size() - 1);
    for (std::size_t t = 1; t < f.size(); ++t) {
      path.push_back(cell(f[t], grid, i, t - 1, n));
    }
    plan.push_back(std::move(path));
  }
  while (lines.next(line)) {
    if (!blank(line)) {
      throw ParseError(lines.number(), "unexpected text after the last of the " + std::to_string(k) +
                                           " agents: '" + shown(line) + "'");
    }
  }
  return plan;
}

Plan load_plan(const std::filesystem::path& path, const Grid& grid) {
  const std::string contents = read_file(path);
  try {
    return parse_plan(contents, grid);
  } catch (const ParseError& e) {
    throw ParseError(path.string(), e.line(), e.message());
  }
}

std::string write_plan(const Plan& plan, const Grid& grid) {
  if (plan.size() > grid.cell_count()) {
    throw std::invalid_argument("the plan has " + std::to_string(plan.size()) + " paths, more than the grid's " +
                                std::to_string(grid.cell_count()) + " cells; a plan file can't hold it");
  }
  std::string out = "version 1\nagents " + std::to_string(plan.size()) + "\n";
  for (std::size_t i = 0; i < plan.size(); ++i) {
    if (plan[i].empty()) {
      throw std::invalid_argument("agent " + std::to_string(i) + "'s path is empty; a plan file can't hold it");
    }
    out += std::to_string(i);
    for (const CellId c : plan[i]) {
      if (c >= grid.cell_count()) {
        throw std::invalid_argument("agent " + std::to_string(i) + "'s path has cell " + std::to_string(c) +
                                    ", outside the grid; a plan file can't hold it");
      }
      const Point p = grid.point(c);
      out += " (" + std::to_string(p.x) + "," + std::to_string(p.y) + ")";
    }
    out += '\n';
  }
  return out;
}

}  // namespace pathfinder
