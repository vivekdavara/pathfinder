// Conflict-based search (DESIGN.md "The planners", planner.hpp).

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <deque>
#include <memory>
#include <optional>
#include <queue>
#include <utility>
#include <vector>

#include "pathfinder/conflicts.hpp"
#include "pathfinder/heuristic.hpp"
#include "pathfinder/planner.hpp"
#include "pathfinder/reservation.hpp"
#include "pathfinder/space_time_astar.hpp"
#include "planner_common.hpp"

namespace pathfinder {

namespace {

constexpr std::uint32_t kNone = 0xFFFF'FFFFU;

// What a node forbids its agent: being in `cell` at t (a vertex constraint, `to` == kNone), or moving cell -> to
// between t and t + 1 (an edge constraint).
struct Constraint {
  CellId cell = 0;
  CellId to = kNone;
  std::uint32_t t = 0;
};

// A constraint-tree node. It stores only what it changed: one constraint on one agent and that agent's new path; its
// other constraints and paths are its ancestors'. (A node's agent, constraint and path are kNone, unused and empty
// for the root, whose paths are kept apart.)
struct Node {
  std::uint32_t parent = kNone;
  std::uint32_t agent = kNone;
  Constraint constraint;
  Path path;
  std::uint64_t soc = 0;
  ConflictScan conflicts;  // of the node's whole plan
};

// The open list's order: lower sum of costs, then fewer conflicts, then the earlier node.
struct Entry {
  std::uint64_t soc;
  std::uint64_t conflicts;
  std::uint32_t node;
};
struct Later {
  bool operator()(const Entry& x, const Entry& y) const {
    if (x.soc != y.soc) {
      return x.soc > y.soc;
    }
    if (x.conflicts != y.conflicts) {
      return x.conflicts > y.conflicts;
    }
    return x.node > y.node;
  }
};

}  // namespace

PlanResult cbs(const Instance& instance, const PlannerLimits& limits) {
  const auto started = std::chrono::steady_clock::now();
  const Grid& grid = instance.grid;
  const std::size_t k = instance.agents.size();
  SearchLimits search;
  search.deadline = detail::deadline_for(limits, started);
  PlanResult result;
  result.plan.assign(k, Path{});
  const auto finish = [&](PlanStatus status, std::size_t failed) {
    result.status = status;
    result.failed_agent = failed;
    if (status != PlanStatus::kSolved) {
      result.plan.assign(k, Path{});
      result.sum_of_costs = 0;
      result.makespan = 0;
    }
    result.seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
    return result;
  };
  if (const auto unreachable = detail::first_unreachable_agent(instance)) {
    return finish(PlanStatus::kUnreachableGoal, *unreachable);
  }

  // One distance table per agent for the whole run: any node may replan any agent.
  const auto index = std::make_shared<const PassableIndex>(grid);
  std::vector<DistanceTable> h;
  h.reserve(k);
  for (const Agent& a : instance.agents) {
    h.emplace_back(grid, index, a.goal);
  }
  // One low-level search for `agent` within what the limits have left; its path into `out` when found.
  enum class Low { kFound, kNoPath, kLimit };
  const auto plan_agent = [&](std::size_t agent, const ReservationTable& table, Path& out) {
    if (search.deadline && std::chrono::steady_clock::now() >= *search.deadline) {
      return Low::kLimit;  // (the search itself looks only before an expansion, and some need none)
    }
    if (limits.max_expansions != 0) {
      search.max_expansions = limits.max_expansions - result.expansions;
    }
    SearchResult found = space_time_astar(grid, instance.agents[agent].start, h[agent], table, search);
    result.expansions += found.expansions;
    if (found.status == SearchStatus::kLimit) {
      return Low::kLimit;
    }
    if (found.status == SearchStatus::kNoPath) {
      return Low::kNoPath;
    }
    out = std::move(found.path);
    return Low::kFound;
  };
  const auto cost_of = [&](std::size_t agent, const Path& p) { return path_cost(p, instance.agents[agent].goal); };
  // Room for one more node? (Node numbers are 32-bit, kNone excluded.)
  const auto may_generate = [&]() {
    return (limits.max_nodes == 0 || result.nodes_generated < limits.max_nodes) &&
           result.nodes_generated < std::uint64_t{kNone};
  };

  // The tree. A deque keeps every node (and its path) where it is while the tree grows, so a plan can be a list of
  // pointers into it.
  std::deque<Node> tree;
  std::vector<Path> root_paths(k);
  ConflictScanner scanner(grid.cell_count());
  std::vector<const Path*> plan(k);
  // Node n's plan into `plan`: each agent's path from the nearest node on the way up that replanned it, else the root.
  const auto collect_plan = [&](std::uint32_t n) {
    std::fill(plan.begin(), plan.end(), nullptr);
    for (std::uint32_t m = n; tree[m].parent != kNone; m = tree[m].parent) {
      if (plan[tree[m].agent] == nullptr) {
        plan[tree[m].agent] = &tree[m].path;
      }
    }
    for (std::size_t i = 0; i < k; ++i) {
      if (plan[i] == nullptr) {
        plan[i] = &root_paths[i];
      }
    }
  };

  // The root: every agent alone.
  if (!may_generate()) {
    return finish(PlanStatus::kLimit, 0);
  }
  const ReservationTable empty(grid);
  tree.emplace_back();
  for (std::size_t i = 0; i < k; ++i) {
    const Low low = plan_agent(i, empty, root_paths[i]);
    if (low != Low::kFound) {
      // (Alone, with every goal reachable, a path always exists: only a limit stops it.)
      return finish(low == Low::kLimit ? PlanStatus::kLimit : PlanStatus::kNoPath, 0);
    }
    tree[0].soc += cost_of(i, root_paths[i]);
  }
  collect_plan(0);
  tree[0].conflicts = scanner.scan(plan);
  ++result.nodes_generated;

  std::priority_queue<Entry, std::vector<Entry>, Later> open;
  open.push(Entry{tree[0].soc, tree[0].conflicts.count, 0});
  while (!open.empty()) {
    const std::uint32_t n = open.top().node;
    open.pop();
    if (tree[n].conflicts.count == 0) {
      collect_plan(n);
      for (std::size_t i = 0; i < k; ++i) {
        result.plan[i] = *plan[i];
        result.makespan = std::max(result.makespan, cost_of(i, *plan[i]));
      }
      result.sum_of_costs = tree[n].soc;
      return finish(PlanStatus::kSolved, 0);
    }
    ++result.nodes_expanded;
    const Conflict c = *tree[n].conflicts.first;
    // Two children: one forbids the conflict to a, the other to b. A swap's constraints are each agent's own move.
    const std::array<std::pair<std::size_t, Constraint>, 2> sides{{
        {c.a, Constraint{c.cell, c.edge ? c.to : kNone, c.time}},
        {c.b, c.edge ? Constraint{c.to, c.cell, c.time} : Constraint{c.cell, kNone, c.time}},
    }};
    for (const auto& [agent, constraint] : sides) {
      if (!may_generate()) {
        return finish(PlanStatus::kLimit, 0);
      }
      // The agent's constraints: the new one and every one on the way up to the root.
      ReservationTable table(grid);
      const auto add = [&table](const Constraint& x) {
        if (x.to == kNone) {
          table.forbid_vertex(x.cell, x.t);
        } else {
          table.forbid_move(x.cell, x.to, x.t);
        }
      };
      add(constraint);
      for (std::uint32_t m = n; tree[m].parent != kNone; m = tree[m].parent) {
        if (tree[m].agent == agent) {
          add(tree[m].constraint);
        }
      }
      Node child;
      const Low low = plan_agent(agent, table, child.path);
      if (low == Low::kLimit) {
        return finish(PlanStatus::kLimit, 0);
      }
      if (low == Low::kNoPath) {
        continue;  // no plan satisfies this child's constraints: nothing to add
      }
      collect_plan(n);
      child.soc = tree[n].soc - cost_of(agent, *plan[agent]) + cost_of(agent, child.path);
      plan[agent] = &child.path;
      child.conflicts = scanner.scan(plan);
      child.parent = n;
      child.agent = static_cast<std::uint32_t>(agent);
      child.constraint = constraint;
      const auto id = static_cast<std::uint32_t>(tree.size());
      tree.push_back(std::move(child));
      ++result.nodes_generated;
      open.push(Entry{tree[id].soc, tree[id].conflicts.count, id});
    }
  }
  return finish(PlanStatus::kNoPath, 0);
}

}  // namespace pathfinder
