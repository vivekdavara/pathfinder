#pragma once

// Seeded map and scenario generators (DESIGN.md "Generated maps and scenarios"). The same arguments give the same
// output on every platform; the tests pin hashes of it.

#include <cstddef>
#include <cstdint>
#include <string>

#include "pathfinder/grid.hpp"
#include "pathfinder/movingai.hpp"

namespace pathfinder {

// A width x height grid with every cell passable. Throws std::invalid_argument for sizes outside 1..kMaxDimension.
Grid empty_map(std::int32_t width, std::int32_t height);

// A width x height grid with exactly round(width * height * obstacle_percent / 100) blocked cells, chosen uniformly
// (a partial Fisher-Yates shuffle driven by SplitMix64(seed)). Throws std::invalid_argument for a size outside
// 1..kMaxDimension or a percentage outside 0..100.
Grid random_map(std::int32_t width, std::int32_t height, double obstacle_percent, std::uint64_t seed);

// A warehouse floor: shelf_rows x shelf_columns blocks of shelf_width x shelf_height blocked cells, `aisle` free
// cells between blocks, and a free border `margin` cells wide around them all.
struct WarehouseLayout {
  std::int32_t shelf_rows = 3;
  std::int32_t shelf_columns = 4;
  std::int32_t shelf_width = 5;
  std::int32_t shelf_height = 2;
  std::int32_t aisle = 1;
  std::int32_t margin = 2;
};

// The layout's map: 2 * margin + shelf_columns * shelf_width + (shelf_columns - 1) * aisle wide, and
// 2 * margin + shelf_rows * shelf_height + (shelf_rows - 1) * aisle high. Every free cell is connected to every
// other. Throws std::invalid_argument if any field is below 1 or the map would exceed kMaxDimension on a side.
Grid warehouse_map(const WarehouseLayout& layout);

// `agents` start-goal pairs drawn from the grid's largest connected piece with SplitMix64(seed): starts pairwise
// distinct and uniform, goals pairwise distinct and uniform, and each agent's goal different from its own start.
// Each entry names `map_name` and the grid's size; as in MovingAI's files, its optimal length is the octile
// distance (octile_distances) and its bucket that length / 4, rounded down. Throws std::invalid_argument unless
// agents < the largest piece's size (so a goal other than the agent's own start always remains), or unless
// valid_map_name(map_name).
Scenario random_scenario(const Grid& grid, std::size_t agents, std::uint64_t seed, const std::string& map_name);

}  // namespace pathfinder
