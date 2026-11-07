// pathfinder: the command-line tool over the library. Exit status 0 when all is well, 1 when `check` finds a
// problem with the instance, `validate` one with the plan or its instance, or `plan` doesn't solve the instance; 2
// for a usage error, a file that can't be read or parsed, or output that can't be written.

#include <algorithm>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "pathfinder/distance.hpp"
#include "pathfinder/generate.hpp"
#include "pathfinder/instance.hpp"
#include "pathfinder/movingai.hpp"
#include "pathfinder/plan.hpp"
#include "pathfinder/planner.hpp"
#include "text.hpp"

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
    "      goal is reachable, and how many stated optimal lengths are the octile distance (MovingAI's convention)\n"
    "  pathfinder validate MAP SCEN PLAN\n"
    "      check a .plan file for the scenario's first k entries (k from the plan): its sum of costs and makespan,\n"
    "      or every problem with it\n"
    "  pathfinder plan MAP SCEN AGENTS [--out PLAN] [--order scenario|reverse|longest|shortest]\n"
    "                 [--max-expansions N] [--time-limit SECONDS]\n"
    "      plan the scenario's first AGENTS entries with prioritized planning (agents in the scenario's order,\n"
    "      its reverse, or by distance alone, longest or shortest first; ties by scenario order) and print the\n"
    "      outcome; when solved, the plan is checked with validate and written to PLAN if given\n";

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

// The same strict format as the scenario files' lengths: digits, optionally a '.' and more digits.
double decimal(std::string_view what, std::string_view s) {
  const auto value = pf::text::decimal(s);
  if (!value) {
    throw UsageError(std::string(what) + " must be a decimal number (digits, optionally a '.' and digits), got '" +
                     std::string(s) + "'");
  }
  return *value;
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
  // The stated optimal lengths. MovingAI's convention, MAPF sets included, is the octile distance (8-connected,
  // diagonals sqrt(2), no corner cutting), which the files round to 8 decimals; the agents here move 4-connected, so
  // this only checks the file, and a mismatch is reported, not an error.
  std::size_t matching = 0;
  std::vector<std::pair<std::size_t, double>> mismatched;
  for (std::size_t i = 0; i < agents; ++i) {
    const auto& a = inst.agents[i];
    const double d = pf::octile_distance(g, a.start, a.goal);
    if (std::abs(d - s.entries[i].optimal_length) <= 1e-6) {
      ++matching;
    } else {
      mismatched.emplace_back(i, d);
    }
  }
  std::cout << "stated optimal lengths equal to the octile distance: " << matching << " of " << agents << "\n";
  for (std::size_t i = 0; i < mismatched.size() && i < 5; ++i) {
    const auto [agent, d] = mismatched[i];
    std::cout << "  agent " << agent << ": stated " << s.entries[agent].optimal_length << ", octile " << d << "\n";
  }
  if (!unreachable.empty()) {
    std::cout << "unreachable goals: " << unreachable.size() << " agents (first: agent " << unreachable.front()
              << ")\n";
    return kProblem;
  }
  return kOk;
}

int validate_plan(const std::vector<std::string_view>& args) {
  need(args, 3, 3);
  const pf::Grid g = pf::load_map(std::filesystem::path(args[0]));
  const pf::Scenario s = pf::load_scenario(std::filesystem::path(args[1]));
  const pf::Plan plan = pf::load_plan(std::filesystem::path(args[2]), g);
  if (plan.size() > s.entries.size()) {
    throw UsageError("the plan has " + std::to_string(plan.size()) + " agents, the scenario only " +
                     std::to_string(s.entries.size()) + " entries");
  }
  pf::Instance inst;
  try {
    inst = pf::make_instance(g, s, plan.size());
  } catch (const std::invalid_argument& e) {
    std::cout << "invalid instance: " << e.what() << "\n";
    return kProblem;
  }
  const pf::PlanCheck check = pf::validate(inst, plan);
  if (check.valid()) {
    std::cout << "valid plan for " << plan.size() << " agents: sum of costs " << check.sum_of_costs << ", makespan "
              << check.makespan << "\n";
    return kOk;
  }
  constexpr std::size_t kShown = 20;
  std::cout << "invalid plan: " << check.problems.size() << (check.problems.size() == 1 ? " problem" : " problems")
            << "\n";
  for (std::size_t i = 0; i < check.problems.size() && i < kShown; ++i) {
    std::cout << "  " << pf::describe(check.problems[i], g) << "\n";
  }
  if (check.problems.size() > kShown) {
    std::cout << "  ... and " << check.problems.size() - kShown << " more\n";
  }
  return kProblem;
}

// The agents in the order `name` asks for. `dist` is each agent's distance alone; ties keep the scenario's order.
std::vector<std::size_t> agent_order(std::string_view name, const std::vector<std::uint32_t>& dist) {
  std::vector<std::size_t> order(dist.size());
  for (std::size_t i = 0; i < order.size(); ++i) {
    order[i] = i;
  }
  if (name == "scenario") {
    return order;
  }
  if (name == "reverse") {
    std::reverse(order.begin(), order.end());
  } else if (name == "longest") {
    std::stable_sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) { return dist[a] > dist[b]; });
  } else if (name == "shortest") {
    std::stable_sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) { return dist[a] < dist[b]; });
  } else {
    throw UsageError("unknown order '" + std::string(name) + "' (scenario, reverse, longest or shortest)");
  }
  return order;
}

int plan(const std::vector<std::string_view>& args) {
  if (args.size() < 3) {
    throw UsageError("wrong number of arguments");
  }
  std::optional<std::filesystem::path> out;
  std::string_view order_name = "scenario";
  pf::PlannerLimits limits;
  for (std::size_t i = 3; i < args.size(); i += 2) {
    if (i + 1 >= args.size()) {
      throw UsageError("option " + std::string(args[i]) + " needs a value");
    }
    const std::string_view value = args[i + 1];
    if (args[i] == "--out") {
      out = std::filesystem::path(value);
    } else if (args[i] == "--order") {
      order_name = value;
    } else if (args[i] == "--max-expansions") {
      limits.max_expansions = whole<std::uint64_t>("--max-expansions", value);
    } else if (args[i] == "--time-limit") {
      limits.time_limit_seconds = decimal("--time-limit", value);
    } else {
      throw UsageError("unknown option '" + std::string(args[i]) + "'");
    }
  }
  const pf::Grid g = pf::load_map(std::filesystem::path(args[0]));
  const pf::Scenario s = pf::load_scenario(std::filesystem::path(args[1]));
  const auto agents = whole<std::size_t>("AGENTS", args[2]);
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
  // Each agent's distance alone: for the order, and their sum is a lower bound on any plan's sum of costs.
  std::vector<std::uint32_t> dist(agents);
  std::uint64_t lower_bound = 0;
  for (std::size_t i = 0; i < agents; ++i) {
    dist[i] = pf::distances(g, inst.agents[i].goal)[inst.agents[i].start];
    lower_bound += dist[i] == pf::kUnreachable ? 0 : dist[i];
  }
  const std::vector<std::size_t> order = agent_order(order_name, dist);
  const pf::PlanResult r = pf::prioritized_planning(inst, limits, order);
  const auto ms = [&r] { return std::to_string(std::llround(r.seconds * 1000)) + " ms"; };
  const std::string stats = std::to_string(r.expansions) + " expansions, " + ms();
  switch (r.status) {
    case pf::PlanStatus::kSolved: {
      const pf::PlanCheck check = pf::validate(inst, r.plan);
      if (!check.valid() || check.sum_of_costs != r.sum_of_costs || check.makespan != r.makespan) {
        throw std::logic_error("internal error: the planner's plan doesn't check out (" +
                               (check.valid() ? std::string("its costs differ") : pf::describe(check.problems[0], g)) +
                               ")");
      }
      std::cout << "solved " << agents << " agents: sum of costs " << r.sum_of_costs << " (lower bound "
                << lower_bound << "), makespan " << r.makespan << ", " << stats << "\n";
      if (out) {
        std::ofstream file(*out, std::ios::binary);
        file << pf::write_plan(r.plan, g);
        file.close();
        if (!file) {
          throw std::runtime_error("can't write " + out->string());
        }
      }
      return kOk;
    }
    case pf::PlanStatus::kUnreachableGoal:
      std::cout << "not solved: agent " << r.failed_agent << " can't reach its goal even alone\n";
      return kProblem;
    case pf::PlanStatus::kNoPath: {
      const auto before = static_cast<std::size_t>(
          std::find(order.begin(), order.end(), r.failed_agent) - order.begin());
      std::cout << "not solved: agent " << r.failed_agent << " found no path around the " << before
                << (before == 1 ? " agent" : " agents") << " planned before it, " << stats << "\n";
      return kProblem;
    }
    case pf::PlanStatus::kLimit:
      std::cout << "not solved: stopped by a limit at agent " << r.failed_agent << ", " << stats << "\n";
      return kProblem;
  }
  return kProblem;
}

// Runs one command; main() then checks the output reached its destination.
int run(const std::vector<std::string_view>& all) {
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
    if (all[0] == "validate") {
      return validate_plan(args);
    }
    if (all[0] == "plan") {
      return plan(args);
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

}  // namespace

int main(int argc, char** argv) {
  const int status = run(std::vector<std::string_view>(argv + 1, argv + argc));
  // A closed or full stdout must not look like success: a script redirecting gen-map into a file on a full disk
  // would otherwise get a truncated map and status 0 (the day-1 review).
  std::cout.flush();
  if (!std::cout) {
    std::cerr << "pathfinder: can't write the output\n";
    return kUsage;
  }
  return status;
}
