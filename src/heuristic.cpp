#include "pathfinder/heuristic.hpp"

#include <algorithm>
#include <array>
#include <stdexcept>
#include <utility>

namespace pathfinder {

PassableIndex::PassableIndex(const Grid& grid) : width_(grid.width()), index_(grid.cell_count(), kBlocked) {
  cells_.reserve(grid.passable_count());
  for (CellId c = 0; c < grid.cell_count(); ++c) {
    if (grid.passable(c)) {
      index_[c] = static_cast<std::uint32_t>(cells_.size());
      cells_.push_back(c);
    }
  }
  // Each passable cell's neighbours by number, so a search over numbers never goes back to CellIds.
  neighbors_.resize(cells_.size());
  std::array<CellId, 4> next{};
  for (std::size_t i = 0; i < cells_.size(); ++i) {
    neighbors_[i].fill(kBlocked);
    const int n = grid.neighbors(cells_[i], next);
    for (int k = 0; k < n; ++k) {
      neighbors_[i][static_cast<std::size_t>(k)] = index_[next[static_cast<std::size_t>(k)]];
    }
  }
}

std::size_t PassableIndex::bytes() const {
  return index_.size() * sizeof(std::uint32_t) + cells_.size() * sizeof(CellId) +
         neighbors_.size() * sizeof(neighbors_[0]);
}

bool PassableIndex::fits(const Grid& g) const {
  if (g.width() != width_ || g.cell_count() != index_.size() || g.passable_count() != cells_.size()) {
    return false;
  }
  return std::all_of(cells_.begin(), cells_.end(), [&g](CellId c) { return g.passable(c); });
}

DistanceTable::DistanceTable(const Grid& grid, std::shared_ptr<const PassableIndex> index, CellId goal)
    : index_(std::move(index)), goal_(goal) {
  // The whole check, not just the passable count: an index of another grid with as many passable cells used to be
  // accepted, and its BFS then wrote far outside the table (the day-2 review).
  if (!index_ || !index_->fits(grid)) {
    throw std::invalid_argument("DistanceTable: the passable index isn't this grid's");
  }
  dist_.assign(index_->size(), kUnreachable);
  const std::uint32_t source = (*index_)[goal];
  if (source == PassableIndex::kBlocked) {
    return;
  }
  // The same breadth-first search as distances(), with the queue and the table by passable number.
  std::vector<std::uint32_t> queue;
  queue.reserve(index_->size());
  dist_[source] = 0;
  queue.push_back(source);
  for (std::size_t head = 0; head < queue.size(); ++head) {
    const std::uint32_t i = queue[head];
    for (const std::uint32_t j : index_->neighbors(i)) {
      if (j == PassableIndex::kBlocked) {
        break;  // the neighbours come first, then kBlocked
      }
      if (dist_[j] == kUnreachable) {
        dist_[j] = dist_[i] + 1;
        queue.push_back(j);
      }
    }
  }
}

}  // namespace pathfinder
