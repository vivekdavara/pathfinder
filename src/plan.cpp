#include "pathfinder/plan.hpp"

#include <algorithm>
#include <tuple>
#include <unordered_map>

namespace pathfinder {
namespace {

std::string where(const Grid& grid, CellId c) {
  if (c >= grid.cell_count()) {
    return "cell " + std::to_string(c) + " (outside the grid)";
  }
  const Point p = grid.point(c);
  return "(" + std::to_string(p.x) + ", " + std::to_string(p.y) + ")";
}

bool by_time_then_agents(const Problem& a, const Problem& b) {
  return std::tie(a.time, a.agent, a.other) < std::tie(b.time, b.agent, b.other);
}

}  // namespace

std::uint32_t path_cost(const Path& path, CellId goal) {
  std::size_t t = path.size() - 1;
  while (t > 0 && path[t - 1] == goal) {
    --t;
  }
  return static_cast<std::uint32_t>(t);
}

std::string describe(const Problem& p, const Grid& grid) {
  const auto agent = "agent " + std::to_string(p.agent);
  const auto agents = "agents " + std::to_string(p.agent) + " and " + std::to_string(p.other);
  const auto t = std::to_string(p.time);
  const auto t1 = std::to_string(p.time + 1);
  switch (p.kind) {
    case ProblemKind::kWrongPathCount:
      return "the plan has " + std::to_string(p.agent) + " paths for " + std::to_string(p.other) + " agents";
    case ProblemKind::kEmptyPath:
      return agent + "'s path is empty";
    case ProblemKind::kWrongStart:
      return agent + "'s path starts at " + where(grid, p.cell) + ", not at its start " + where(grid, p.to);
    case ProblemKind::kWrongGoal:
      return agent + "'s path ends at " + where(grid, p.cell) + ", not at its goal " + where(grid, p.to);
    case ProblemKind::kOffGrid:
      return agent + " is at " + where(grid, p.cell) + " at t=" + t;
    case ProblemKind::kBlockedCell:
      return agent + " is on the blocked cell " + where(grid, p.cell) + " at t=" + t;
    case ProblemKind::kIllegalMove:
      return agent + " jumps from " + where(grid, p.cell) + " to " + where(grid, p.to) + " between t=" + t +
             " and t=" + t1;
    case ProblemKind::kVertexConflict:
      return agents + " are both at " + where(grid, p.cell) + " at t=" + t;
    case ProblemKind::kEdgeConflict:
      return agents + " swap " + where(grid, p.cell) + " and " + where(grid, p.to) + " between t=" + t + " and t=" +
             t1;
  }
  return "unknown problem";
}

PlanCheck validate(const Instance& instance, const Plan& plan) {
  PlanCheck out;
  const Grid& grid = instance.grid;
  const std::size_t k = instance.agents.size();
  if (plan.size() != k) {
    out.problems.push_back(Problem{ProblemKind::kWrongPathCount, plan.size(), k, 0, 0, 0});
    return out;
  }

  // 2 and 3: each path on its own.
  bool legal = true;
  std::size_t longest = 0;
  for (std::size_t i = 0; i < k; ++i) {
    const Path& path = plan[i];
    const Agent& a = instance.agents[i];
    const auto report = [&](ProblemKind kind, std::size_t time, CellId cell, CellId to) {
      out.problems.push_back(Problem{kind, i, 0, static_cast<std::uint32_t>(time), cell, to});
      legal = false;
    };
    if (path.empty()) {
      report(ProblemKind::kEmptyPath, 0, 0, 0);
      continue;
    }
    longest = std::max(longest, path.size());
    if (path.front() != a.start) {
      report(ProblemKind::kWrongStart, 0, path.front(), a.start);
    }
    if (path.back() != a.goal) {
      report(ProblemKind::kWrongGoal, path.size() - 1, path.back(), a.goal);
    }
    for (std::size_t t = 0; t < path.size(); ++t) {
      const CellId c = path[t];
      if (c >= grid.cell_count()) {
        report(ProblemKind::kOffGrid, t, c, 0);
        continue;
      }
      if (!grid.passable(c)) {
        report(ProblemKind::kBlockedCell, t, c, 0);
      }
      if (t + 1 < path.size() && path[t + 1] < grid.cell_count() && !grid.adjacent_or_same(c, path[t + 1])) {
        report(ProblemKind::kIllegalMove, t, c, path[t + 1]);
      }
    }
  }

  // 4 and 5: conflicts among the non-empty paths, each agent staying at its last cell once its path ends.
  const auto at = [&](std::size_t i, std::size_t t) {
    const Path& path = plan[i];
    return path[std::min(t, path.size() - 1)];
  };
  std::vector<Problem> vertex;
  std::vector<Problem> edge;
  std::unordered_map<CellId, std::vector<std::size_t>> occupants;
  for (std::size_t t = 0; t < longest; ++t) {
    occupants.clear();
    for (std::size_t i = 0; i < k; ++i) {
      if (!plan[i].empty()) {
        occupants[at(i, t)].push_back(i);
      }
    }
    const auto time = static_cast<std::uint32_t>(t);
    const std::size_t vertex_before = vertex.size();
    for (const auto& [cell, here] : occupants) {
      for (std::size_t x = 0; x < here.size(); ++x) {
        for (std::size_t y = x + 1; y < here.size(); ++y) {
          vertex.push_back(Problem{ProblemKind::kVertexConflict, here[x], here[y], time, cell, 0});
        }
      }
    }
    std::sort(vertex.begin() + static_cast<std::ptrdiff_t>(vertex_before), vertex.end(), by_time_then_agents);
    if (t + 1 == longest) {
      break;  // no step after the last time
    }
    const std::size_t edge_before = edge.size();
    for (std::size_t a = 0; a < k; ++a) {
      if (plan[a].empty()) {
        continue;
      }
      const CellId u = at(a, t);
      const CellId v = at(a, t + 1);
      if (u == v) {
        continue;
      }
      const auto there = occupants.find(v);
      if (there == occupants.end()) {
        continue;
      }
      for (const std::size_t b : there->second) {
        if (a < b && at(b, t + 1) == u) {
          edge.push_back(Problem{ProblemKind::kEdgeConflict, a, b, time, u, v});
        }
      }
    }
    std::sort(edge.begin() + static_cast<std::ptrdiff_t>(edge_before), edge.end(), by_time_then_agents);
  }
  out.problems.insert(out.problems.end(), vertex.begin(), vertex.end());
  out.problems.insert(out.problems.end(), edge.begin(), edge.end());

  if (legal) {
    out.costs.reserve(k);
    for (std::size_t i = 0; i < k; ++i) {
      const std::uint32_t c = path_cost(plan[i], instance.agents[i].goal);
      out.costs.push_back(c);
      out.sum_of_costs += c;
      out.makespan = std::max(out.makespan, c);
    }
  }
  return out;
}

}  // namespace pathfinder
