// What the building blocks and the planners cost, from the optimised build: `make bench`. Generated maps are
// synthetic (random 20% obstacles, seed 1; the default warehouse floor); the *OnMap benchmarks read MovingAI maps
// from data/ (`make maps`) and are registered only when the files are there.

#include <benchmark/benchmark.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include "pathfinder/conflicts.hpp"
#include "pathfinder/distance.hpp"
#include "pathfinder/generate.hpp"
#include "pathfinder/heuristic.hpp"
#include "pathfinder/instance.hpp"
#include "pathfinder/movingai.hpp"
#include "pathfinder/plan.hpp"
#include "pathfinder/planner.hpp"
#include "pathfinder/reservation.hpp"
#include "pathfinder/space_time_astar.hpp"
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

// The same plans through the ConflictScanner, which CBS runs on every node's plan instead of validate(): one scanner
// for all iterations, as CBS keeps one for its run. Items are agent-steps.
void BM_ConflictScan(benchmark::State& state) {
  const auto k = static_cast<std::size_t>(state.range(0));
  constexpr std::size_t kSteps = 200;
  const Grid g(128, 128);
  Plan plan;
  for (std::size_t i = 0; i < k; ++i) {
    plan.emplace_back(kSteps, static_cast<CellId>(i * 7 % g.cell_count()));
  }
  ConflictScanner scanner(g.cell_count());
  for (auto _ : state) {
    ConflictScan scan = scanner.scan(plan);
    if (scan.count != 0) {
      state.SkipWithError("the benchmark's plan should have no conflict");
      break;
    }
    benchmark::DoNotOptimize(scan);
  }
  state.SetItemsProcessed(state.iterations() * static_cast<std::int64_t>(k * kSteps));
}
BENCHMARK(BM_ConflictScan)->Arg(10)->Arg(100)->Arg(1000)->Unit(benchmark::kMicrosecond);

// One agent across a square map by space-time A*: with nothing reserved, and around the paths of the 50 agents
// after it in the same scenario, planned first (prioritized planning's situation). Items are expansions.
void BM_SpaceTimeAStar(benchmark::State& state) {
  const Square sq = square(state.range(0));
  const auto others = static_cast<std::size_t>(state.range(1));
  constexpr std::size_t kAgents = 51;  // the same agent 0 whatever `others` is
  const Instance inst = make_instance(sq.grid, random_scenario(sq.grid, kAgents, 3, "b.map"), kAgents);
  ReservationTable table(sq.grid);
  if (others > 0) {
    Instance rest{sq.grid, {inst.agents.begin() + 1, inst.agents.begin() + 1 + static_cast<std::ptrdiff_t>(others)}};
    const PlanResult planned = prioritized_planning(rest);
    if (planned.status != PlanStatus::kSolved) {
      state.SkipWithError("the other agents weren't solved");
      return;
    }
    for (const Path& p : planned.plan) {
      table.reserve_path(p);
    }
  }
  const DistanceTable h(sq.grid, std::make_shared<const PassableIndex>(sq.grid), inst.agents[0].goal);
  std::uint64_t expansions = 0;
  for (auto _ : state) {
    SearchResult r = space_time_astar(sq.grid, inst.agents[0].start, h, table);
    if (r.status != SearchStatus::kFound) {
      state.SkipWithError("no path");
      break;
    }
    expansions += r.expansions;
    benchmark::DoNotOptimize(r);
  }
  state.SetItemsProcessed(static_cast<std::int64_t>(expansions));
  state.counters["expansions"] =
      static_cast<double>(expansions) / static_cast<double>(std::max<benchmark::IterationCount>(1, state.iterations()));
}
BENCHMARK(BM_SpaceTimeAStar)->Args({128, 0})->Args({128, 50})->Args({512, 0})->Args({512, 50})->Unit(benchmark::kMicrosecond);

// Prioritized planning of k agents on a 64 x 64 random map (20% obstacles; random scenario, seed 5).
void BM_PrioritizedPlanning(benchmark::State& state) {
  const Square sq = square(64);
  const auto k = static_cast<std::size_t>(state.range(0));
  const Instance inst = make_instance(sq.grid, random_scenario(sq.grid, k, 5, "b.map"), k);
  for (auto _ : state) {
    PlanResult r = prioritized_planning(inst);
    if (r.status != PlanStatus::kSolved) {
      state.SkipWithError("not solved");
      break;
    }
    state.counters["SOC"] = static_cast<double>(r.sum_of_costs);
    benchmark::DoNotOptimize(r);
  }
}
BENCHMARK(BM_PrioritizedPlanning)->Arg(10)->Arg(50)->Arg(100)->Unit(benchmark::kMillisecond);

// CBS on the same map and scenario as prioritized planning, at the agent counts it solves quickly (at 40 and 50 it
// didn't finish within 500,000 nodes: README "Results (day 3)"). Counters: the tree's nodes, the low level's
// expansions, and the SOC, which prioritized planning's may exceed.
void BM_Cbs(benchmark::State& state) {
  const Square sq = square(64);
  const auto k = static_cast<std::size_t>(state.range(0));
  const Instance inst = make_instance(sq.grid, random_scenario(sq.grid, k, 5, "b.map"), k);
  PlannerLimits limits;
  limits.max_nodes = 100000;
  for (auto _ : state) {
    PlanResult r = cbs(inst, limits);
    if (r.status != PlanStatus::kSolved) {
      state.SkipWithError("not solved");
      break;
    }
    state.counters["SOC"] = static_cast<double>(r.sum_of_costs);
    state.counters["nodes"] = static_cast<double>(r.nodes_generated);
    state.counters["expansions"] = static_cast<double>(r.expansions);
    benchmark::DoNotOptimize(r);
  }
}
BENCHMARK(BM_Cbs)->Arg(10)->Arg(20)->Arg(30)->Unit(benchmark::kMillisecond);

// One heuristic table on a MovingAI map: all cells (distances) against passable cells only (DistanceTable), from the
// map's first passable cell. orz900d is the case DESIGN.md's budget worried about: a tenth of its cells passable.
void register_map_benchmarks() {
  const std::filesystem::path dir = std::filesystem::path(PATHFINDER_SOURCE_DIR) / "data" / "mapf-map";
  for (const char* name : {"den520d", "orz900d", "warehouse-20-40-10-2-2"}) {
    const auto file = dir / (std::string(name) + ".map");
    if (!std::filesystem::exists(file)) {
      continue;
    }
    benchmark::RegisterBenchmark((std::string("BM_DistancesOnMap/") + name).c_str(),
                                 [file](benchmark::State& state) {
                                   const Grid g = load_map(file);
                                   const auto index = PassableIndex(g);
                                   const CellId source = index.cell(0);
                                   for (auto _ : state) {
                                     auto d = distances(g, source);
                                     benchmark::DoNotOptimize(d.data());
                                   }
                                   state.counters["MB"] = static_cast<double>(g.cell_count() * 4) / 1e6;
                                 })
        ->Unit(benchmark::kMillisecond);
    benchmark::RegisterBenchmark((std::string("BM_DistanceTableOnMap/") + name).c_str(),
                                 [file](benchmark::State& state) {
                                   const Grid g = load_map(file);
                                   const auto index = std::make_shared<const PassableIndex>(g);
                                   const CellId source = index->cell(0);
                                   for (auto _ : state) {
                                     DistanceTable t(g, index, source);
                                     benchmark::DoNotOptimize(t);
                                   }
                                   state.counters["MB"] = static_cast<double>(g.passable_count() * 4) / 1e6;
                                   state.counters["index_MB"] = static_cast<double>(index->bytes()) / 1e6;
                                 })
        ->Unit(benchmark::kMillisecond);
  }
}

}  // namespace
}  // namespace pathfinder

int main(int argc, char** argv) {
  pathfinder::register_map_benchmarks();
  benchmark::Initialize(&argc, argv);
  if (benchmark::ReportUnrecognizedArguments(argc, argv)) {
    return 1;
  }
  benchmark::RunSpecifiedBenchmarks();
  benchmark::Shutdown();
  return 0;
}

