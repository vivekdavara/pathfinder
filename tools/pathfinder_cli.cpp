// pathfinder: the command-line tool over the library. Exit status 0 when all is well, 1 when `check` finds a
// problem with the instance, 2 for a usage error or a file that can't be read or parsed.

#include <charconv>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "pathfinder/distance.hpp"
#include "pathfinder/generate.hpp"
#include "pathfinder/instance.hpp"
#include "pathfinder/movingai.hpp"

namespace pf = pathfinder;

namespace {

constexpr int kOk = 0;
constexpr int kProblem = 1;
constexpr int kUsage = 2;

constexpr std::string_view kUsageText =
    "usage:\n"
    "  pathfinder info MAP\n"
    "      size, passable cells and connected pieces of a .map file\n"
    "  pathfinder gen-map empty WIDTH HEIGHT\n"
    "  pathfinder gen-map random WIDTH HEIGHT OBSTACLE_PERCENT SEED\n"
    "  pathfinder gen-map warehouse SHELF_ROWS SHELF_COLUMNS SHELF_WIDTH SHELF_HEIGHT AISLE MARGIN\n"
    "      print a generated .map file\n"
    "  pathfinder gen-scen MAP AGENTS SEED [MAP_NAME]\n"
    "      print a .scen file of AGENTS random start-goal pairs on MAP (MAP_NAME defaults to MAP's file name)\n"
    "  pathfinder check MAP SCEN [AGENTS]\n"
    "      check that the scenario's first AGENTS entries (default: all) make an instance on MAP, that every\n"
    "      goal is reachable, and how many stated optimal lengths are the 4-connected BFS distance\n";

struct UsageError : std::runtime_error {
  using std::runtime_error::runtime_error;
};

template <class Int>
Int whole(std::string_view what, std::string_view s) {
  Int value{};
  const auto [end, ec] = std::from_chars(s.data(), s.data() + s.size(), value);
  if (s.empty() || s[0] == '-' || s[0] == '+' || ec != std::errc{} || end != s.data() + s.size()) {
    throw UsageError(std::string(what) + " must be a whole number, got '" + std::string(s) + "'");
  }
  return value;
}

double decimal(std::string_view what, std::string_view s) {
  double value = 0.0;
  const auto [end, ec] = std::from_chars(s.data(), s.data() + s.size(), value, std::chars_format::fixed);
  if (s.empty() || ec != std::errc{} || end != s.data() + s.size()) {
    throw UsageError(std::string(what) + " must be a decimal number, got '" + std::string(s) + "'");
  }
  return value;
}

void need(const std::vector<std::string_view>& args, std::size_t min, std::size_t max) {
  if (args.size() < min || args.size() > max) {
    throw UsageError("wrong number of arguments");
  }
}

int info(const std::vector<std::string_view>& args) {
  need(args, 1, 1);
  const pf::Grid g = pf::load_map(std::filesystem::path(args[0]));
  const pf::Components c = pf::components(g);
  const std::size_t largest = c.largest == pf::Components::kNone ? 0 : c.sizes[c.largest];
  std::cout << "size " << g.width() << "x" << g.height() << "\n"
            << "passable " << g.passable_count() << " of " << g.cell_count() << "\n"
            << "pieces " << c.sizes.size() << "\n"
            << "largest piece " << largest << "\n";
  return kOk;
}

int gen_map(const std::vector<std::string_view>& args) {
  if (args.empty()) {
    throw UsageError("gen-map needs a kind: empty, random or warehouse");
  }
  const std::vector<std::string_view> rest(args.begin() + 1, args.end());
  std::optional<pf::Grid> g;
  try {
    if (args[0] == "empty") {
      need(rest, 2, 2);
      g = pf::empty_map(whole<std::int32_t>("WIDTH", rest[0]), whole<std::int32_t>("HEIGHT", rest[1]));
    } else if (args[0] == "random") {
      need(rest, 4, 4);
      g = pf::random_map(whole<std::int32_t>("WIDTH", rest[0]), whole<std::int32_t>("HEIGHT", rest[1]),
                         decimal("OBSTACLE_PERCENT", rest[2]), whole<std::uint64_t>("SEED", rest[3]));
    } else if (args[0] == "warehouse") {
      need(rest, 6, 6);
      const pf::WarehouseLayout layout{
          whole<std::int32_t>("SHELF_ROWS", rest[0]),   whole<std::int32_t>("SHELF_COLUMNS", rest[1]),
          whole<std::int32_t>("SHELF_WIDTH", rest[2]),  whole<std::int32_t>("SHELF_HEIGHT", rest[3]),
          whole<std::int32_t>("AISLE", rest[4]),        whole<std::int32_t>("MARGIN", rest[5])};
      g = pf::warehouse_map(layout);
    } else {
      throw UsageError("unknown map kind '" + std::string(args[0]) + "'");
    }
  } catch (const std::invalid_argument& e) {
    throw UsageError(e.what());
  }
  std::cout << pf::write_map(*g);
  return kOk;
}

int gen_scen(const std::vector<std::string_view>& args) {
  need(args, 3, 4);
  const std::filesystem::path path(args[0]);
  const pf::Grid g = pf::load_map(path);
  const auto agents = whole<std::size_t>("AGENTS", args[1]);
  const auto seed = whole<std::uint64_t>("SEED", args[2]);
  const std::string name = args.size() == 4 ? std::string(args[3]) : path.filename().string();
  try {
    std::cout << pf::write_scenario(pf::random_scenario(g, agents, seed, name));
  } catch (const std::invalid_argument& e) {
    throw UsageError(e.what());
  }
  return kOk;
}

int check(const std::vector<std::string_view>& args) {
  need(args, 2, 3);
  const pf::Grid g = pf::load_map(std::filesystem::path(args[0]));
  const pf::Scenario s = pf::load_scenario(std::filesystem::path(args[1]));
  const std::size_t agents = args.size() == 3 ? whole<std::size_t>("AGENTS", args[2]) : s.entries.size();
  if (agents > s.entries.size()) {
    throw UsageError("the scenario has " + std::to_string(s.entries.size()) + " entries, not " +
                     std::to_string(agents));
  }
  pf::Instance inst;
  try {
    inst = pf::make_instance(g, s, agents);
  } catch (const std::invalid_argument& e) {
    std::cout << "invalid instance: " << e.what() << "\n";
    return kProblem;
  }
  std::cout << "instance of " << agents << " agents on a " << g.width() << "x" << g.height() << " map: valid\n";
  const auto unreachable = pf::unreachable_agents(inst);
  // The stated optimal lengths: the MAPF sets' are 4-connected distances; the older sets' are octile (8-connected)
  // ones, so a mismatch is reported, not an error.
  std::size_t matching = 0;
  std::vector<std::size_t> mismatched;
  for (std::size_t i = 0; i < agents; ++i) {
    const auto& a = inst.agents[i];
    const auto d = pf::distances(g, a.start)[a.goal];
    if (d != pf::kUnreachable && static_cast<double>(d) == s.entries[i].optimal_length) {
      ++matching;
    } else if (d != pf::kUnreachable) {
      mismatched.push_back(i);
    }
  }
  std::cout << "optimal lengths equal to the BFS distance: " << matching << " of " << agents - unreachable.size()
            << " reachable agents\n";
  for (std::size_t i = 0; i < mismatched.size() && i < 5; ++i) {
    const auto& a = inst.agents[mismatched[i]];
    std::cout << "  agent " << mismatched[i] << ": stated " << s.entries[mismatched[i]].optimal_length << ", BFS "
              << pf::distances(g, a.start)[a.goal] << "\n";
  }
  if (!unreachable.empty()) {
    std::cout << "unreachable goals: " << unreachable.size() << " agents (first: agent " << unreachable.front()
              << ")\n";
    return kProblem;
  }
  return kOk;
}

}  // namespace

int main(int argc, char** argv) {
  const std::vector<std::string_view> all(argv + 1, argv + argc);
  if (all.empty() || all[0] == "-h" || all[0] == "--help") {
    std::cout << kUsageText;
    return all.empty() ? kUsage : kOk;
  }
  const std::vector<std::string_view> args(all.begin() + 1, all.end());
  try {
    if (all[0] == "info") {
      return info(args);
    }
    if (all[0] == "gen-map") {
      return gen_map(args);
    }
    if (all[0] == "gen-scen") {
      return gen_scen(args);
    }
    if (all[0] == "check") {
      return check(args);
    }
    throw UsageError("unknown command '" + std::string(all[0]) + "'");
  } catch (const UsageError& e) {
    std::cerr << "pathfinder: " << e.what() << "\n" << kUsageText;
    return kUsage;
  } catch (const std::exception& e) {  // ParseError and unreadable files
    std::cerr << "pathfinder: " << e.what() << "\n";
    return kUsage;
  }
}
