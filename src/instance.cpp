#include "pathfinder/instance.hpp"

#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>

#include "pathfinder/distance.hpp"

namespace pathfinder {
namespace {

std::string at(const Grid& grid, CellId c) {
  const Point p = grid.point(c);
  return "(" + std::to_string(p.x) + ", " + std::to_string(p.y) + ")";
}

}  // namespace

void check_instance(const Instance& instance) {
  const Grid& grid = instance.grid;
  std::unordered_map<CellId, std::size_t> start_of;
  std::unordered_map<CellId, std::size_t> goal_of;
  for (std::size_t i = 0; i < instance.agents.size(); ++i) {
    const Agent& a = instance.agents[i];
    const auto who = "agent " + std::to_string(i);
    for (const auto& [what, c] : {std::pair{"start", a.start}, std::pair{"goal", a.goal}}) {
      if (c >= grid.cell_count()) {
        throw std::invalid_argument(who + "'s " + what + " (cell " + std::to_string(c) + ") is outside the " +
                                    std::to_string(grid.width()) + "x" + std::to_string(grid.height()) + " grid");
      }
      if (!grid.passable(c)) {
        throw std::invalid_argument(who + "'s " + what + " " + at(grid, c) + " is blocked");
      }
    }
    if (const auto [it, fresh] = start_of.emplace(a.start, i); !fresh) {
      throw std::invalid_argument(who + "'s start " + at(grid, a.start) + " is also agent " +
                                  std::to_string(it->second) + "'s start");
    }
    if (const auto [it, fresh] = goal_of.emplace(a.goal, i); !fresh) {
      throw std::invalid_argument(who + "'s goal " + at(grid, a.goal) + " is also agent " +
                                  std::to_string(it->second) + "'s goal");
    }
  }
}

Instance make_instance(const Grid& grid, const Scenario& scenario, std::size_t agents) {
  if (agents > scenario.entries.size()) {
    throw std::invalid_argument("the scenario has " + std::to_string(scenario.entries.size()) + " entries, not " +
                                std::to_string(agents));
  }
  Instance instance{grid, {}};
  instance.agents.reserve(agents);
  for (std::size_t i = 0; i < agents; ++i) {
    const ScenarioEntry& e = scenario.entries[i];
    if (e.width != grid.width() || e.height != grid.height()) {
      throw std::invalid_argument("scenario entry " + std::to_string(i) + " is for a " + std::to_string(e.width) +
                                  "x" + std::to_string(e.height) + " map, but the map is " +
                                  std::to_string(grid.width()) + "x" + std::to_string(grid.height()));
    }
    // The parser checks this, but a scenario built in code (or from Python) never went through it, and grid.id()
    // would wrap a point off the grid onto another cell.
    for (const auto& [what, p] : {std::pair{"start", e.start}, std::pair{"goal", e.goal}}) {
      if (!grid.in_bounds(p)) {
        throw std::invalid_argument("scenario entry " + std::to_string(i) + "'s " + what + " (" +
                                    std::to_string(p.x) + ", " + std::to_string(p.y) + ") is outside the " +
                                    std::to_string(grid.width()) + "x" + std::to_string(grid.height()) + " map");
      }
    }
    instance.agents.push_back(Agent{grid.id(e.start), grid.id(e.goal)});
  }
  check_instance(instance);
  return instance;
}

std::vector<std::size_t> unreachable_agents(const Instance& instance) {
  const Components pieces = components(instance.grid);
  std::vector<std::size_t> out;
  for (std::size_t i = 0; i < instance.agents.size(); ++i) {
    const Agent& a = instance.agents[i];
    if (pieces.label[a.start] != pieces.label[a.goal]) {
      out.push_back(i);
    }
  }
  return out;
}

}  // namespace pathfinder
