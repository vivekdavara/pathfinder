#include "pathfinder/generate.hpp"

#include <cmath>
#include <numeric>
#include <stdexcept>
#include <utility>
#include <vector>

#include "pathfinder/distance.hpp"
#include "pathfinder/rng.hpp"

namespace pathfinder {

Grid empty_map(std::int32_t width, std::int32_t height) { return Grid(width, height); }

Grid random_map(std::int32_t width, std::int32_t height, double obstacle_percent, std::uint64_t seed) {
  if (!(obstacle_percent >= 0.0 && obstacle_percent <= 100.0)) {
    throw std::invalid_argument("the obstacle percentage must be 0..100");
  }
  Grid grid(width, height);
  const std::size_t n = grid.cell_count();
  // IEEE arithmetic is the same everywhere, so this count is too.
  const auto blocked = static_cast<std::size_t>(std::llround(static_cast<double>(n) * obstacle_percent / 100.0));
  std::vector<CellId> cells(n);
  std::iota(cells.begin(), cells.end(), CellId{0});
  SplitMix64 rng(seed);
  for (std::size_t i = 0; i < blocked; ++i) {
    const std::size_t j = i + rng.below(n - i);
    std::swap(cells[i], cells[j]);
    grid.set_passable(grid.point(cells[i]), false);
  }
  return grid;
}

Grid warehouse_map(const WarehouseLayout& layout) {
  const auto& l = layout;
  if (l.shelf_rows < 1 || l.shelf_columns < 1 || l.shelf_width < 1 || l.shelf_height < 1 || l.aisle < 1 ||
      l.margin < 1) {
    throw std::invalid_argument("every warehouse layout field must be at least 1");
  }
  const auto side = [](std::int64_t count, std::int64_t size, std::int64_t aisle, std::int64_t margin) {
    return 2 * margin + count * size + (count - 1) * aisle;
  };
  const std::int64_t width = side(l.shelf_columns, l.shelf_width, l.aisle, l.margin);
  const std::int64_t height = side(l.shelf_rows, l.shelf_height, l.aisle, l.margin);
  if (width > kMaxDimension || height > kMaxDimension) {
    throw std::invalid_argument("the warehouse would be " + std::to_string(width) + "x" + std::to_string(height) +
                                ", over " + std::to_string(kMaxDimension) + " on a side");
  }
  Grid grid(static_cast<std::int32_t>(width), static_cast<std::int32_t>(height));
  for (std::int32_t r = 0; r < l.shelf_rows; ++r) {
    for (std::int32_t c = 0; c < l.shelf_columns; ++c) {
      const std::int32_t x0 = l.margin + c * (l.shelf_width + l.aisle);
      const std::int32_t y0 = l.margin + r * (l.shelf_height + l.aisle);
      for (std::int32_t y = y0; y < y0 + l.shelf_height; ++y) {
        for (std::int32_t x = x0; x < x0 + l.shelf_width; ++x) {
          grid.set_passable(Point{x, y}, false);
        }
      }
    }
  }
  return grid;
}

Scenario random_scenario(const Grid& grid, std::size_t agents, std::uint64_t seed, const std::string& map_name) {
  if (map_name.empty() || map_name.find_first_of(" \t\r\n") != std::string::npos) {
    throw std::invalid_argument("the map name must be non-empty with no spaces, tabs or line breaks");
  }
  const Components pieces = components(grid);
  const std::size_t size = pieces.largest == Components::kNone ? 0 : pieces.sizes[pieces.largest];
  if (agents >= size) {
    throw std::invalid_argument("the largest connected piece has " + std::to_string(size) + " cells; " +
                                std::to_string(agents) + " agents need at least " + std::to_string(agents + 1));
  }
  std::vector<CellId> pool;
  pool.reserve(size);
  for (CellId c = 0; c < grid.cell_count(); ++c) {
    if (pieces.label[c] == pieces.largest) {
      pool.push_back(c);
    }
  }
  SplitMix64 rng(seed);
  // Starts: the first `agents` cells of a partial Fisher-Yates shuffle.
  std::vector<CellId> starts = pool;
  for (std::size_t i = 0; i < agents; ++i) {
    std::swap(starts[i], starts[i + rng.below(size - i)]);
  }
  // Goals: the same, from a fresh copy, redrawing a goal equal to the agent's own start. At step i, size - i >= 2
  // cells remain (agents < size), so at least one isn't the start and the redraw ends.
  std::vector<CellId> goals = pool;
  for (std::size_t i = 0; i < agents; ++i) {
    std::size_t j = 0;
    do {
      j = i + rng.below(size - i);
    } while (goals[j] == starts[i]);
    std::swap(goals[i], goals[j]);
  }
  Scenario scenario;
  scenario.entries.reserve(agents);
  for (std::size_t i = 0; i < agents; ++i) {
    const std::uint32_t d = distances(grid, starts[i])[goals[i]];
    scenario.entries.push_back(ScenarioEntry{static_cast<int>(d / 4), map_name, grid.width(), grid.height(),
                                             grid.point(starts[i]), grid.point(goals[i]), static_cast<double>(d)});
  }
  return scenario;
}

}  // namespace pathfinder
