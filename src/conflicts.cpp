#include "pathfinder/conflicts.hpp"

#include <algorithm>
#include <stdexcept>
#include <string>
#include <tuple>

namespace pathfinder {

bool earlier(const Conflict& x, const Conflict& y) {
  // A vertex conflict at t sits at 2t, a swap between t and t + 1 at 2t + 1.
  const std::uint64_t xt = 2 * std::uint64_t{x.time} + (x.edge ? 1U : 0U);
  const std::uint64_t yt = 2 * std::uint64_t{y.time} + (y.edge ? 1U : 0U);
  return std::tie(xt, x.a, x.b) < std::tie(yt, y.a, y.b);
}

ConflictScanner::ConflictScanner(std::size_t cell_count) : epoch_of_(cell_count, 0), head_(cell_count, kNone) {}

ConflictScan ConflictScanner::scan(const Plan& plan) {
  std::vector<const Path*> paths;
  paths.reserve(plan.size());
  for (const Path& p : plan) {
    paths.push_back(&p);
  }
  return scan(paths);
}

ConflictScan ConflictScanner::scan(std::span<const Path* const> plan) {
  const std::size_t k = plan.size();
  if (k >= kNone) {
    throw std::invalid_argument("ConflictScanner: too many agents");
  }
  std::size_t longest = 0;
  for (std::size_t i = 0; i < k; ++i) {
    const Path& p = *plan[i];
    if (p.empty()) {
      throw std::invalid_argument("ConflictScanner: agent " + std::to_string(i) + "'s path is empty");
    }
    for (const CellId c : p) {
      if (c >= epoch_of_.size()) {
        throw std::out_of_range("ConflictScanner: agent " + std::to_string(i) + "'s path leaves the grid");
      }
    }
    longest = std::max(longest, p.size());
  }
  next_.assign(k, kNone);

  ConflictScan out;
  const auto found = [&out](const Conflict& c) {
    ++out.count;
    if (!out.first || earlier(c, *out.first)) {
      out.first = c;
    }
  };
  const auto at = [&plan](std::size_t i, std::size_t t) {
    const Path& p = *plan[i];
    return p[std::min(t, p.size() - 1)];
  };

  for (std::size_t t = 0; t < longest; ++t) {
    if (++epoch_ == 0) {  // the counter wrapped: forget every old list once
      std::fill(epoch_of_.begin(), epoch_of_.end(), 0U);
      epoch_ = 1;
    }
    const auto time = static_cast<std::uint32_t>(t);
    // Each agent joins its cell's list for time t, conflicting with every agent already on it (all lower-numbered).
    for (std::size_t i = 0; i < k; ++i) {
      const CellId c = at(i, t);
      if (epoch_of_[c] != epoch_) {
        epoch_of_[c] = epoch_;
        next_[i] = kNone;
      } else {
        for (std::uint32_t j = head_[c]; j != kNone; j = next_[j]) {
          found(Conflict{false, j, i, time, c, 0});
        }
        next_[i] = head_[c];
      }
      head_[c] = static_cast<std::uint32_t>(i);
    }
    if (t + 1 == longest) {
      break;  // no step after the last time
    }
    // A swap: a moves u -> v while an agent b > a that was in v at t is in u at t + 1.
    for (std::size_t a = 0; a < k; ++a) {
      const CellId u = at(a, t);
      const CellId v = at(a, t + 1);
      if (u == v || epoch_of_[v] != epoch_) {
        continue;
      }
      for (std::uint32_t b = head_[v]; b != kNone; b = next_[b]) {
        if (a < b && at(b, t + 1) == u) {
          found(Conflict{true, a, b, time, u, v});
        }
      }
    }
  }
  return out;
}

}  // namespace pathfinder
