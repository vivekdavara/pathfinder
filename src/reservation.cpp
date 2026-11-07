#include "pathfinder/reservation.hpp"

#include <algorithm>
#include <bit>
#include <stdexcept>
#include <string>

namespace pathfinder {

ReservationTable::ReservationTable(const Grid& grid)
    : width_(grid.width()), cells_(grid.cell_count()), marks_(grid.cell_count(), 0) {}

void ReservationTable::check(CellId cell, std::uint32_t t) const {
  if (cell >= cells_) {
    throw std::out_of_range("reservation of cell " + std::to_string(cell) + " outside a grid of " +
                            std::to_string(cells_) + " cells");
  }
  if (t > kMaxTime) {
    throw std::invalid_argument("reservation at time " + std::to_string(t) + ", after the latest allowed, " +
                                std::to_string(kMaxTime));
  }
}

unsigned ReservationTable::direction_or_none(CellId from, CellId to) const {
  if (from >= cells_ || to >= cells_) {
    return kNoDirection;
  }
  const auto w = static_cast<CellId>(width_);
  const CellId x = from % w;
  if (from >= w && to == from - w) {
    return 0;
  }
  if (x + 1 < w && to == from + 1) {
    return 1;
  }
  if (to == from + w) {  // to < cells_: a row below exists
    return 2;
  }
  if (x > 0 && to == from - 1) {
    return 3;
  }
  return kNoDirection;
}

unsigned ReservationTable::direction(CellId from, CellId to) const {
  const unsigned d = direction_or_none(from, to);
  if (d != kNoDirection) {
    return d;
  }
  throw std::invalid_argument("a move from cell " + std::to_string(from) + " to cell " + std::to_string(to) +
                              ", which aren't 4-neighbours");
}

void ReservationTable::forbid_vertex(CellId cell, std::uint32_t t) {
  check(cell, t);
  vertices_.insert(key(cell, t));
  marks_[cell] |= kCellMark;
  auto [it, inserted] = last_vertex_.try_emplace(cell, t);
  if (!inserted) {
    it->second = std::max(it->second, t);
  }
  horizon_ = std::max(horizon_, t);
}

void ReservationTable::forbid_move(CellId from, CellId to, std::uint32_t t) {
  check(from, t);
  check(to, t);
  if (t == kMaxTime) {
    throw std::invalid_argument("a move at time " + std::to_string(t) + " would end after the latest allowed time");
  }
  const unsigned d = direction(from, to);
  moves_[key(from, t)] |= static_cast<std::uint8_t>(1U << d);
  marks_[from] |= kMoveMark;
  horizon_ = std::max(horizon_, t + 1);
}

void ReservationTable::forbid_from(CellId cell, std::uint32_t t) {
  check(cell, t);
  auto [it, inserted] = from_.try_emplace(cell, t);
  if (!inserted) {
    it->second = std::min(it->second, t);
  }
  marks_[cell] |= kCellMark;
  horizon_ = std::max(horizon_, t);
}

void ReservationTable::reserve_path(const Path& path) {
  if (path.empty()) {
    throw std::invalid_argument("reserve_path: an empty path");
  }
  if (path.size() - 1 > kMaxTime) {
    throw std::invalid_argument("reserve_path: a path longer than the latest allowed time");
  }
  // Check every cell and step first, so a bad path changes nothing.
  for (std::size_t t = 0; t < path.size(); ++t) {
    check(path[t], 0);
    if (t + 1 < path.size() && path[t] != path[t + 1]) {
      check(path[t + 1], 0);
      direction(path[t], path[t + 1]);
    }
  }
  for (std::size_t t = 0; t + 1 < path.size(); ++t) {
    const auto time = static_cast<std::uint32_t>(t);
    forbid_vertex(path[t], time);
    if (path[t] != path[t + 1]) {
      forbid_move(path[t + 1], path[t], time);  // the reverse: whoever searches can't swap with this agent
    }
  }
  forbid_from(path.back(), static_cast<std::uint32_t>(path.size() - 1));
}

bool ReservationTable::cell_free(CellId cell, std::uint32_t t) const {
  if (const auto it = from_.find(cell); it != from_.end() && t >= it->second) {
    return false;
  }
  return !vertices_.contains(key(cell, t));
}

bool ReservationTable::move_allowed(CellId from, CellId to, std::uint32_t t) const {
  const auto it = moves_.find(key(from, t));
  return it == moves_.end() || (it->second & (1U << direction(from, to))) == 0;
}

std::optional<std::uint32_t> ReservationTable::last_vertex_time(CellId cell) const {
  const auto it = last_vertex_.find(cell);
  return it == last_vertex_.end() ? std::nullopt : std::optional<std::uint32_t>(it->second);
}

std::optional<std::uint32_t> ReservationTable::forbidden_from(CellId cell) const {
  const auto it = from_.find(cell);
  return it == from_.end() ? std::nullopt : std::optional<std::uint32_t>(it->second);
}

std::size_t ReservationTable::move_count() const {
  std::size_t n = 0;
  for (const auto& [k, bits] : moves_) {
    n += static_cast<std::size_t>(std::popcount(static_cast<unsigned>(bits)));
  }
  return n;
}

}  // namespace pathfinder
