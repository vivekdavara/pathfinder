#include "pathfinder/distance.hpp"

#include <array>
#include <cmath>
#include <functional>
#include <queue>
#include <utility>

namespace pathfinder {

std::vector<std::uint32_t> distances(const Grid& grid, CellId source) {
  std::vector<std::uint32_t> dist(grid.cell_count(), kUnreachable);
  if (!grid.passable(source)) {
    return dist;
  }
  // The frontier as a vector used as a FIFO queue: every cell enters at most once.
  std::vector<CellId> queue;
  queue.reserve(grid.passable_count());
  dist[source] = 0;
  queue.push_back(source);
  std::array<CellId, 4> next{};
  for (std::size_t head = 0; head < queue.size(); ++head) {
    const CellId c = queue[head];
    const int n = grid.neighbors(c, next);
    for (int i = 0; i < n; ++i) {
      const CellId d = next[static_cast<std::size_t>(i)];
      if (dist[d] == kUnreachable) {
        dist[d] = dist[c] + 1;
        queue.push_back(d);
      }
    }
  }
  return dist;
}

namespace {

// Dijkstra over 8-connected moves without corner cutting, from `source`; stops as soon as `stop` is settled (pass
// an id past the grid's cells never to stop early).
std::vector<double> octile_search(const Grid& grid, CellId source, CellId stop) {
  const double inf = std::numeric_limits<double>::infinity();
  std::vector<double> dist(grid.cell_count(), inf);
  if (!grid.passable(source)) {
    return dist;
  }
  const double diagonal = std::sqrt(2.0);
  using Entry = std::pair<double, CellId>;
  std::priority_queue<Entry, std::vector<Entry>, std::greater<>> open;
  dist[source] = 0.0;
  open.emplace(0.0, source);
  while (!open.empty()) {
    const auto [d, c] = open.top();
    open.pop();
    if (d > dist[c]) {
      continue;  // a stale entry
    }
    if (c == stop) {
      break;  // settled: nothing later can lower it
    }
    const Point p = grid.point(c);
    for (std::int32_t dy = -1; dy <= 1; ++dy) {
      for (std::int32_t dx = -1; dx <= 1; ++dx) {
        if (dx == 0 && dy == 0) {
          continue;
        }
        const Point q{p.x + dx, p.y + dy};
        if (!grid.passable(q)) {
          continue;
        }
        const bool diag = dx != 0 && dy != 0;
        if (diag && (!grid.passable(Point{p.x + dx, p.y}) || !grid.passable(Point{p.x, p.y + dy}))) {
          continue;  // it would cut a corner
        }
        const double nd = d + (diag ? diagonal : 1.0);
        const CellId n = grid.id(q);
        if (nd < dist[n]) {
          dist[n] = nd;
          open.emplace(nd, n);
        }
      }
    }
  }
  return dist;
}

}  // namespace

std::vector<double> octile_distances(const Grid& grid, CellId source) {
  return octile_search(grid, source, static_cast<CellId>(grid.cell_count()));
}

double octile_distance(const Grid& grid, CellId source, CellId target) {
  return octile_search(grid, source, target)[target];
}

Components components(const Grid& grid) {
  Components out;
  out.label.assign(grid.cell_count(), Components::kNone);
  std::vector<CellId> stack;
  std::array<CellId, 4> next{};
  for (CellId c = 0; c < grid.cell_count(); ++c) {
    if (!grid.passable(c) || out.label[c] != Components::kNone) {
      continue;
    }
    const auto id = static_cast<std::uint32_t>(out.sizes.size());
    std::size_t size = 0;
    out.label[c] = id;
    stack.push_back(c);
    while (!stack.empty()) {
      const CellId x = stack.back();
      stack.pop_back();
      ++size;
      const int n = grid.neighbors(x, next);
      for (int i = 0; i < n; ++i) {
        const CellId d = next[static_cast<std::size_t>(i)];
        if (out.label[d] == Components::kNone) {
          out.label[d] = id;
          stack.push_back(d);
        }
      }
    }
    out.sizes.push_back(size);
    if (out.largest == Components::kNone || size > out.sizes[out.largest]) {
      out.largest = id;
    }
  }
  return out;
}

}  // namespace pathfinder
