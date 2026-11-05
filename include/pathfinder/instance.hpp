#pragma once

#include <cstddef>
#include <vector>

#include "pathfinder/grid.hpp"
#include "pathfinder/movingai.hpp"

namespace pathfinder {

struct Agent {
  CellId start = 0;
  CellId goal = 0;
  friend bool operator==(Agent, Agent) = default;
};

// A MAPF problem: a grid and the agents on it (DESIGN.md "The model").
struct Instance {
  Grid grid;
  std::vector<Agent> agents;
};

// Checks an instance's rules: every start and goal inside the grid and passable, starts pairwise distinct, goals
// pairwise distinct. Throws std::invalid_argument naming the first agent that breaks one. (Reachability is not a
// rule; see unreachable_agents.)
void check_instance(const Instance& instance);

// The instance made of a scenario's first `agents` entries on `grid`: each entry's stated width and height must be
// the grid's, and the result must pass check_instance. Throws std::invalid_argument (naming the entry) otherwise, or
// if the scenario has fewer entries.
Instance make_instance(const Grid& grid, const Scenario& scenario, std::size_t agents);

// The agents whose goal can't be reached from their start even with no other agent on the grid (so the instance has
// no solution), in increasing order. Precondition: check_instance(instance) passes.
std::vector<std::size_t> unreachable_agents(const Instance& instance);

}  // namespace pathfinder
