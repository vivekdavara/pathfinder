#include "pathfinder/distance.hpp"

#include <array>

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
