#include "pathfinder/heuristic.hpp"

#include <array>
#include <stdexcept>
#include <utility>

namespace pathfinder {

PassableIndex::PassableIndex(const Grid& grid) : index_(grid.cell_count(), kBlocked) {
  cells_.reserve(grid.passable_count());
  for (CellId c = 0; c < grid.cell_count(); ++c) {
    if (grid.passable(c)) {
      index_[c] = static_cast<std::uint32_t>(cells_.size());
      cells_.push_back(c);
    }
  }
}

std::size_t PassableIndex::bytes() const {
  return index_.size() * sizeof(std::uint32_t) + cells_.size() * sizeof(CellId);
}

DistanceTable::DistanceTable(const Grid& grid, std::shared_ptr<const PassableIndex> index, CellId goal)
    : index_(std::move(index)), goal_(goal) {
  if (!index_ || index_->size() != grid.passable_count()) {
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
  std::array<CellId, 4> next{};
  for (std::size_t head = 0; head < queue.size(); ++head) {
    const std::uint32_t i = queue[head];
    const int n = grid.neighbors(index_->cell(i), next);
    for (int k = 0; k < n; ++k) {
      const std::uint32_t j = (*index_)[next[static_cast<std::size_t>(k)]];
      if (dist_[j] == kUnreachable) {
        dist_[j] = dist_[i] + 1;
        queue.push_back(j);
      }
    }
  }
}

}  // namespace pathfinder
