#include "pathfinder/space_time_astar.hpp"

#include <algorithm>
#include <array>
#include <memory>
#include <queue>
#include <stdexcept>
#include <unordered_map>
#include <vector>

namespace pathfinder {

namespace {

struct Node {
  CellId cell;
  std::uint32_t t;
  std::uint32_t parent;  // index into the node list; the root is its own parent
};

// An open-list entry. g = t (every action costs 1), so f = t + h.
struct Entry {
  std::uint32_t f;
  std::uint32_t t;
  std::uint32_t node;  // generation order, for the last tie-break
};

// std::priority_queue pops the "largest": the entry with the lowest f, then the highest t, then the lowest node.
struct Later {
  bool operator()(const Entry& a, const Entry& b) const {
    if (a.f != b.f) {
      return a.f > b.f;
    }
    if (a.t != b.t) {
      return a.t < b.t;
    }
    return a.node > b.node;
  }
};

// What the search knows about a state: the lowest t it was pushed with, and whether it has been expanded.
struct Seen {
  std::uint32_t t;
  bool closed;
};

struct Mix {
  std::size_t operator()(std::uint64_t k) const {
    k ^= k >> 30;
    k *= 0xBF58476D1CE4E5B9ULL;
    k ^= k >> 27;
    k *= 0x94D049BB133111EBULL;
    k ^= k >> 31;
    return static_cast<std::size_t>(k);
  }
};

}  // namespace

SearchResult space_time_astar(const Grid& grid, CellId start, const DistanceTable& h, const ReservationTable& table,
                              const SearchLimits& limits) {
  if (!table.fits(grid)) {
    throw std::invalid_argument("space_time_astar: the reservation table was made for a grid of another size");
  }
  SearchResult result;
  const CellId goal = h.goal();
  // No path: the goal is unreachable even alone, someone rests on the goal for ever (so the agent could never stay
  // there), or the start is taken at t = 0.
  if (h(start) == kUnreachable || table.forbidden_from(goal).has_value() || !table.vertex_free(start, 0)) {
    return result;
  }
  const std::optional<std::uint32_t> goal_taken_until = table.last_vertex_time(goal);
  // From `settled` on, a cell's state doesn't depend on the time: every later time of a cell is the same state.
  const std::uint32_t settled = table.horizon() + 1;
  const auto state = [settled](CellId c, std::uint32_t t) {
    return (std::uint64_t{std::min(t, settled)} << 32) | c;
  };

  std::vector<Node> nodes;
  std::priority_queue<Entry, std::vector<Entry>, Later> open;
  std::unordered_map<std::uint64_t, Seen, Mix> seen;
  const auto push = [&](CellId c, std::uint32_t t, std::uint32_t parent) {
    const auto [it, inserted] = seen.try_emplace(state(c, t), Seen{t, false});
    if (!inserted) {
      // Before `settled` every push of a state has the same t; after it, only a lower t is news (and then the
      // state can't have been expanded yet: with a consistent heuristic it would have come off with the lower t).
      if (it->second.closed || it->second.t <= t) {
        return;
      }
      it->second.t = t;
    }
    const auto index = static_cast<std::uint32_t>(nodes.size());
    nodes.push_back(Node{c, t, parent});
    open.push(Entry{t + h(c), t, index});
    ++result.generated;
  };

  push(start, 0, 0);
  std::array<CellId, 4> next{};
  while (!open.empty()) {
    const Entry e = open.top();
    open.pop();
    const Node n = nodes[e.node];
    Seen& s = seen.find(state(n.cell, n.t))->second;
    if (s.closed) {
      // A stale entry: the state came off already. (That includes a late state pushed again with a lower t: same
      // cell, same h, so the lower t has the lower f and came off first.)
      continue;
    }
    if (n.cell == goal && (!goal_taken_until || n.t > *goal_taken_until)) {
      result.status = SearchStatus::kFound;
      result.path.resize(std::size_t{n.t} + 1);
      for (std::uint32_t i = e.node;; i = nodes[i].parent) {
        result.path[nodes[i].t] = nodes[i].cell;
        if (nodes[i].t == 0) {
          break;
        }
      }
      return result;
    }
    // (The node list's indices are 32-bit: a search that would outgrow them stops as if at a limit. It would hold
    // over 48 GB of nodes first.)
    if ((limits.max_expansions && result.expansions >= *limits.max_expansions) ||
        nodes.size() > std::size_t{0xFFFF'FFFFU} - 5) {
      result.status = SearchStatus::kLimit;
      return result;
    }
    if ((result.expansions & 1023U) == 0 && limits.deadline && std::chrono::steady_clock::now() >= *limits.deadline) {
      result.status = SearchStatus::kLimit;
      return result;
    }
    s.closed = true;
    ++result.expansions;
    const std::uint32_t t1 = n.t + 1;
    const int k = grid.neighbors(n.cell, next);
    for (int i = 0; i < k; ++i) {
      const CellId v = next[static_cast<std::size_t>(i)];
      if (table.vertex_free(v, t1) && table.move_free(n.cell, v, n.t)) {
        push(v, t1, e.node);
      }
    }
    if (table.vertex_free(n.cell, t1)) {
      push(n.cell, t1, e.node);  // wait
    }
  }
  return result;  // kNoPath
}

SearchResult space_time_astar(const Grid& grid, CellId start, CellId goal, const ReservationTable& table,
                              const SearchLimits& limits) {
  const DistanceTable h(grid, std::make_shared<const PassableIndex>(grid), goal);
  return space_time_astar(grid, start, h, table, limits);
}

}  // namespace pathfinder
