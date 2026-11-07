// What day 1's building blocks cost, on generated maps (synthetic: random 20% obstacles, seed 1), from the optimised
// build: `make bench`. The planners will run BFS once per agent for their heuristic and the validator on every plan,
// so these set the budget for days 2 to 4.

#include <benchmark/benchmark.h>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "pathfinder/distance.hpp"
#include "pathfinder/generate.hpp"
#include "pathfinder/heuristic.hpp"
#include "pathfinder/instance.hpp"
#include "pathfinder/movingai.hpp"
#include "pathfinder/plan.hpp"
#include "pathfinder/rng.hpp"

namespace pathfinder {
namespace {

// A square random map and a passable cell in its largest piece to search from.
struct Square {
  Grid grid;
  CellId source = 0;
};

Square square(std::int64_t side) {
  const auto s = static_cast<std::int32_t>(side);
  Square out{random_map(s, s, 20.0, 1), 0};
  const Components c = components(out.grid);
  while (c.label[out.source] != c.largest) {
    ++out.source;
  }
  return out;
}

void BM_Distances(benchmark::State& state) {
  const Square sq = square(state.range(0));
  for (auto _ : state) {
    auto d = distances(sq.grid, sq.source);
    benchmark::DoNotOptimize(d.data());
  }
  state.SetItemsProcessed(state.iterations() * static_cast<std::int64_t>(sq.grid.cell_count()));
  state.counters["MB"] = static_cast<double>(sq.grid.cell_count() * sizeof(std::uint32_t)) / 1e6;
}
BENCHMARK(BM_Distances)->Arg(32)->Arg(128)->Arg(512)->Arg(1024)->Unit(benchmark::kMicrosecond);

// The same search into a DistanceTable (by passable number, the index shared and built once): the planners' heuristic.
void BM_DistanceTable(benchmark::State& state) {
  const Square sq = square(state.range(0));
  const auto index = std::make_shared<const PassableIndex>(sq.grid);
  for (auto _ : state) {
    DistanceTable t(sq.grid, index, sq.source);
    benchmark::DoNotOptimize(t);
  }
  state.SetItemsProcessed(state.iterations() * static_cast<std::int64_t>(sq.grid.cell_count()));
  state.counters["MB"] = static_cast<double>(sq.grid.passable_count() * sizeof(std::uint32_t)) / 1e6;
}
BENCHMARK(BM_DistanceTable)->Arg(32)->Arg(128)->Arg(512)->Arg(1024)->Unit(benchmark::kMicrosecond);

void BM_OctileDistances(benchmark::State& state) {
  const Square sq = square(state.range(0));
  for (auto _ : state) {
    auto d = octile_distances(sq.grid, sq.source);
    benchmark::DoNotOptimize(d.data());
  }
  state.SetItemsProcessed(state.iterations() * static_cast<std::int64_t>(sq.grid.cell_count()));
}
BENCHMARK(BM_OctileDistances)->Arg(32)->Arg(128)->Arg(512)->Arg(1024)->Unit(benchmark::kMicrosecond);

// One start-goal distance, as random_scenario and `pathfinder check` need it: the full search over the map against
// the single-pair search that stops at the goal, over the same 64 random goals in the largest piece.
std::vector<CellId> goals_in(const Square& sq) {
  const Components c = components(sq.grid);
  SplitMix64 rng(7);
  std::vector<CellId> out;
  while (out.size() < 64) {
    const auto cell = static_cast<CellId>(rng.below(sq.grid.cell_count()));
    if (c.label[cell] == c.largest) {
      out.push_back(cell);
    }
  }
  return out;
}

void BM_OctilePairByFullSearch(benchmark::State& state) {
  const Square sq = square(state.range(0));
  const auto goals = goals_in(sq);
  std::size_t i = 0;
  for (auto _ : state) {
    double d = octile_distances(sq.grid, sq.source)[goals[i++ % goals.size()]];
    benchmark::DoNotOptimize(d);
  }
}
BENCHMARK(BM_OctilePairByFullSearch)->Arg(128)->Arg(512)->Unit(benchmark::kMicrosecond);

void BM_OctilePairBySinglePairSearch(benchmark::State& state) {
  const Square sq = square(state.range(0));
  const auto goals = goals_in(sq);
  std::size_t i = 0;
  for (auto _ : state) {
    double d = octile_distance(sq.grid, sq.source, goals[i++ % goals.size()]);
    benchmark::DoNotOptimize(d);
  }
}
BENCHMARK(BM_OctilePairBySinglePairSearch)->Arg(128)->Arg(512)->Unit(benchmark::kMicrosecond);

void BM_ParseMap(benchmark::State& state) {
  const std::string text = write_map(square(state.range(0)).grid);
  for (auto _ : state) {
    Grid g = parse_map(text);
    benchmark::DoNotOptimize(g);
  }
  state.SetBytesProcessed(state.iterations() * static_cast<std::int64_t>(text.size()));
}
BENCHMARK(BM_ParseMap)->Arg(32)->Arg(128)->Arg(512)->Arg(1024)->Unit(benchmark::kMicrosecond);

// The validator on a valid plan of k agents over 200 time steps: each agent waits on its own cell (its start and
// goal), so every step of every agent goes through the occupancy map and nothing is reported. Items are agent-steps.
void BM_Validate(benchmark::State& state) {
  const auto k = static_cast<std::size_t>(state.range(0));
  constexpr std::size_t kSteps = 200;
  const Grid g(128, 128);
  Instance inst{g, {}};
  Plan plan;
  for (std::size_t i = 0; i < k; ++i) {
    const auto c = static_cast<CellId>(i * 7 % g.cell_count());
    inst.agents.push_back(Agent{c, c});
    plan.emplace_back(kSteps, c);
  }
  check_instance(inst);
  for (auto _ : state) {
    PlanCheck check = validate(inst, plan);
    if (!check.valid()) {
      state.SkipWithError("the benchmark's plan should be valid");
      break;
    }
    benchmark::DoNotOptimize(check);
  }
  state.SetItemsProcessed(state.iterations() * static_cast<std::int64_t>(k * kSteps));
}
BENCHMARK(BM_Validate)->Arg(10)->Arg(100)->Arg(1000)->Unit(benchmark::kMicrosecond);

}  // namespace
}  // namespace pathfinder

BENCHMARK_MAIN();
