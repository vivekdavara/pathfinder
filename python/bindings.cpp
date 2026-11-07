// The Python face of the library: pathfinder._core. Cells cross the boundary as CellId integers (y * width + x) or as
// (x, y) through Point; C++ exceptions become ValueError (std::invalid_argument, and ParseError, which also carries
// .line), IndexError (std::out_of_range) and RuntimeError (std::runtime_error). Every __eq__ is an operator, so
// comparing with another type gives False (NotImplemented), not a TypeError.

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl/filesystem.h>

#include <array>
#include <chrono>
#include <optional>
#include <string>
#include <vector>

#include "pathfinder/distance.hpp"
#include "pathfinder/generate.hpp"
#include "pathfinder/grid.hpp"
#include "pathfinder/instance.hpp"
#include "pathfinder/movingai.hpp"
#include "pathfinder/plan.hpp"
#include "pathfinder/planner.hpp"
#include "pathfinder/reservation.hpp"
#include "pathfinder/space_time_astar.hpp"

namespace py = pybind11;
using namespace pybind11::literals;

namespace pathfinder {
namespace {

std::string repr(Point p) { return "Point(" + std::to_string(p.x) + ", " + std::to_string(p.y) + ")"; }

// A coordinate as a Python int: anything with __index__, as the fixed-width parameters elsewhere accept (an int, a
// bool, numpy's integers), but not a float. A TypeError otherwise. (Taking py::int_ directly refused __index__
// objects: the day-1 third review.)
py::int_ as_index(const py::handle& v) {
  PyObject* index = PyNumber_Index(v.ptr());
  if (index == nullptr) {
    throw py::error_already_set();
  }
  return py::reinterpret_steal<py::int_>(index);
}

// The value of a Python int if it fits a long long.
std::optional<long long> small(const py::int_& v) {
  int overflow = 0;
  const long long value = PyLong_AsLongLongAndOverflow(v.ptr(), &overflow);
  if (overflow != 0) {
    return std::nullopt;
  }
  return value;
}

// The cell (x, y) if both name one inside the grid; nullopt for any other integer, however large or negative (a
// fixed-width parameter would make those a TypeError instead).
std::optional<Point> inside(const Grid& g, const py::handle& x, const py::handle& y) {
  const auto axis = [](const py::handle& v, std::int32_t size) -> std::optional<std::int32_t> {
    const auto value = small(as_index(v));
    if (!value || *value < 0 || *value >= size) {
      return std::nullopt;
    }
    return static_cast<std::int32_t>(*value);
  };
  const auto column = axis(x, g.width());
  const auto row = axis(y, g.height());
  if (!column || !row) {
    return std::nullopt;
  }
  return Point{*column, *row};
}

Point inside_or_throw(const Grid& g, const py::handle& x, const py::handle& y) {
  const auto p = inside(g, x, y);
  if (!p) {
    // Not py::str(x): Python 3.11 refuses to print an int of over 4,300 digits, which would turn this IndexError into
    // a ValueError (the day-1 third review).
    const auto show = [](const py::handle& v) {
      const auto value = small(as_index(v));
      return value ? std::to_string(*value) : std::string("<a number too large to show>");
    };
    throw py::index_error("(" + show(x) + ", " + show(y) + ") is outside the " + std::to_string(g.width()) + "x" +
                          std::to_string(g.height()) + " grid");
  }
  return *p;
}

void check_cell(const Grid& g, CellId c) {
  if (c >= g.cell_count()) {
    throw py::index_error("cell " + std::to_string(c) + " is outside the " + std::to_string(g.width()) + "x" +
                          std::to_string(g.height()) + " grid");
  }
}

void bind_grid(py::module_& m) {
  py::class_<Point>(m, "Point", "A cell's coordinates: x is the column, y the row, (0, 0) the top left.")
      .def(py::init<std::int32_t, std::int32_t>(), "x"_a, "y"_a)
      .def_readwrite("x", &Point::x)
      .def_readwrite("y", &Point::y)
      .def("__eq__", [](Point a, Point b) { return a == b; }, py::is_operator())
      .def("__hash__", [](Point p) { return py::hash(py::make_tuple(p.x, p.y)); })
      .def("__iter__", [](Point p) { return py::iter(py::make_tuple(p.x, p.y)); })
      .def("__repr__", &repr);

  py::class_<Grid>(m, "Grid", "A 4-connected grid of passable and blocked cells.")
      .def(py::init<std::int32_t, std::int32_t>(), "width"_a, "height"_a,
           "A width x height grid, every cell passable (ValueError outside 1..10000).")
      .def_property_readonly("width", &Grid::width)
      .def_property_readonly("height", &Grid::height)
      .def_property_readonly("cell_count", &Grid::cell_count)
      .def_property_readonly("passable_count", &Grid::passable_count)
      .def(
          "passable",
          [](const Grid& g, const py::object& x, const py::object& y) {
            const auto p = inside(g, x, y);
            return p && g.passable(*p);
          },
          "x"_a, "y"_a, "Whether (x, y) is passable; False outside the grid (any integer, of any size).")
      .def(
          "set_passable",
          [](Grid& g, const py::object& x, const py::object& y, bool passable) {
            g.set_passable(inside_or_throw(g, x, y), passable);
          },
          "x"_a, "y"_a, "passable"_a, "IndexError outside the grid.")
      .def(
          "id", [](const Grid& g, const py::object& x, const py::object& y) { return g.id(inside_or_throw(g, x, y)); },
          "x"_a, "y"_a, "The cell id of (x, y): y * width + x; IndexError outside the grid.")
      .def(
          "point",
          [](const Grid& g, CellId c) {
            check_cell(g, c);
            return g.point(c);
          },
          "cell"_a)
      .def(
          "neighbors",
          [](const Grid& g, CellId c) {
            check_cell(g, c);
            std::array<CellId, 4> out{};
            const int n = g.neighbors(c, out);
            return std::vector<CellId>(out.begin(), out.begin() + n);
          },
          "cell"_a, "The passable 4-neighbours of a cell, north, east, south, west.")
      .def("__eq__", [](const Grid& a, const Grid& b) { return a == b; }, py::is_operator())
      .def("__repr__", [](const Grid& g) {
        return "Grid(" + std::to_string(g.width()) + "x" + std::to_string(g.height()) + ", " +
               std::to_string(g.passable_count()) + " passable)";
      });

  m.attr("MAX_DIMENSION") = kMaxDimension;
}

void bind_formats(py::module_& m) {
  // ParseError is a ValueError with the line it names as .line.
  PYBIND11_CONSTINIT static py::gil_safe_call_once_and_store<py::object> parse_error;
  parse_error.call_once_and_store_result(
      [&]() { return py::object(py::exception<ParseError>(m, "ParseError", PyExc_ValueError)); });
  py::register_exception_translator([](std::exception_ptr p) {
    try {
      if (p) {
        std::rethrow_exception(p);
      }
    } catch (const ParseError& e) {
      const py::object& type = parse_error.get_stored();
      py::object error = type(e.what());
      error.attr("line") = e.line();
      PyErr_SetObject(type.ptr(), error.ptr());
    }
  });

  m.def("terrain_passable", &terrain_passable, "c"_a);
  m.def("parse_map", &parse_map, "text"_a, "A grid from a .map file's text (ParseError if it isn't one).");
  m.def("load_map", &load_map, "path"_a);
  m.def("write_map", &write_map, "grid"_a, "The canonical .map text of a grid.");

  py::class_<ScenarioEntry>(m, "ScenarioEntry", "One line of a .scen file.")
      .def(py::init([](int bucket, std::string map, std::int32_t width, std::int32_t height, Point start, Point goal,
                       double optimal_length) {
             return ScenarioEntry{bucket, std::move(map), width, height, start, goal, optimal_length};
           }),
           "bucket"_a, "map"_a, "width"_a, "height"_a, "start"_a, "goal"_a, "optimal_length"_a)
      .def_readwrite("bucket", &ScenarioEntry::bucket)
      .def_readwrite("map", &ScenarioEntry::map)
      .def_readwrite("width", &ScenarioEntry::width)
      .def_readwrite("height", &ScenarioEntry::height)
      .def_readwrite("start", &ScenarioEntry::start)
      .def_readwrite("goal", &ScenarioEntry::goal)
      .def_readwrite("optimal_length", &ScenarioEntry::optimal_length)
      .def("__eq__", [](const ScenarioEntry& a, const ScenarioEntry& b) { return a == b; }, py::is_operator())
      .def("__repr__", [](const ScenarioEntry& e) {
        return "ScenarioEntry(" + e.map + ", " + repr(e.start) + " -> " + repr(e.goal) + ")";
      });

  py::class_<Scenario>(m, "Scenario")
      .def(py::init<>())
      .def(py::init([](std::vector<ScenarioEntry> entries) { return Scenario{std::move(entries)}; }), "entries"_a)
      .def_readwrite("entries", &Scenario::entries)
      .def("__len__", [](const Scenario& s) { return s.entries.size(); })
      .def("__eq__", [](const Scenario& a, const Scenario& b) { return a == b; }, py::is_operator());

  m.def("parse_scenario", &parse_scenario, "text"_a);
  m.def("load_scenario", &load_scenario, "path"_a);
  m.def("write_scenario", &write_scenario, "scenario"_a);
}

void bind_instances(py::module_& m) {
  py::class_<Agent>(m, "Agent")
      .def(py::init([](CellId start, CellId goal) { return Agent{start, goal}; }), "start"_a, "goal"_a)
      .def_readwrite("start", &Agent::start)
      .def_readwrite("goal", &Agent::goal)
      .def("__eq__", [](Agent a, Agent b) { return a == b; }, py::is_operator())
      .def("__repr__", [](Agent a) {
        return "Agent(" + std::to_string(a.start) + " -> " + std::to_string(a.goal) + ")";
      });

  py::class_<Instance>(m, "Instance", "A grid and its agents; the constructor enforces DESIGN.md's rules.")
      .def(py::init([](Grid grid, std::vector<Agent> agents) {
             Instance instance{std::move(grid), std::move(agents)};
             check_instance(instance);
             return instance;
           }),
           "grid"_a, "agents"_a)
      // Copies: a reference would let Python change a grid check_instance has already accepted (pybind11 ignores
      // const), e.g. block an agent's start (the day-1 review).
      .def_property_readonly(
          "grid", [](const Instance& i) { return i.grid; }, "A copy of the instance's grid.")
      .def_property_readonly(
          "agents", [](const Instance& i) { return i.agents; }, "A copy of the instance's agents.");

  m.def("make_instance", &make_instance, "grid"_a, "scenario"_a, "agents"_a,
        "The instance of a scenario's first `agents` entries.");
  m.def("unreachable_agents", &unreachable_agents, "instance"_a);

  m.attr("UNREACHABLE") = kUnreachable;
  m.def(
      "distances",
      [](const Grid& g, CellId source) {
        check_cell(g, source);
        return distances(g, source);
      },
      "grid"_a, "source"_a, "BFS distance from source to every cell (UNREACHABLE where there is no way).");

  m.def(
      "octile_distances",
      [](const Grid& g, CellId source) {
        check_cell(g, source);
        return octile_distances(g, source);
      },
      "grid"_a, "source"_a,
      "Octile (8-connected, no corner cutting) distances: MovingAI's 'optimal length' convention; inf if none.");

  py::class_<Components>(m, "Components")
      .def_readonly("label", &Components::label)
      .def_readonly("sizes", &Components::sizes)
      .def_readonly("largest", &Components::largest);
  m.attr("NO_COMPONENT") = Components::kNone;
  m.def("components", &components, "grid"_a);
}

void bind_generators(py::module_& m) {
  m.def("empty_map", &empty_map, "width"_a, "height"_a);
  m.def("random_map", &random_map, "width"_a, "height"_a, "obstacle_percent"_a, "seed"_a);

  py::class_<WarehouseLayout>(m, "WarehouseLayout")
      .def(py::init([](std::int32_t shelf_rows, std::int32_t shelf_columns, std::int32_t shelf_width,
                       std::int32_t shelf_height, std::int32_t aisle, std::int32_t margin) {
             return WarehouseLayout{shelf_rows, shelf_columns, shelf_width, shelf_height, aisle, margin};
           }),
           "shelf_rows"_a = WarehouseLayout{}.shelf_rows, "shelf_columns"_a = WarehouseLayout{}.shelf_columns,
           "shelf_width"_a = WarehouseLayout{}.shelf_width, "shelf_height"_a = WarehouseLayout{}.shelf_height,
           "aisle"_a = WarehouseLayout{}.aisle, "margin"_a = WarehouseLayout{}.margin)
      .def_readwrite("shelf_rows", &WarehouseLayout::shelf_rows)
      .def_readwrite("shelf_columns", &WarehouseLayout::shelf_columns)
      .def_readwrite("shelf_width", &WarehouseLayout::shelf_width)
      .def_readwrite("shelf_height", &WarehouseLayout::shelf_height)
      .def_readwrite("aisle", &WarehouseLayout::aisle)
      .def_readwrite("margin", &WarehouseLayout::margin);
  m.def("warehouse_map", &warehouse_map, "layout"_a = WarehouseLayout{});
  m.def("random_scenario", &random_scenario, "grid"_a, "agents"_a, "seed"_a, "map_name"_a);
}

void bind_plans(py::module_& m) {
  py::enum_<ProblemKind>(m, "ProblemKind")
      .value("WRONG_PATH_COUNT", ProblemKind::kWrongPathCount)
      .value("EMPTY_PATH", ProblemKind::kEmptyPath)
      .value("WRONG_START", ProblemKind::kWrongStart)
      .value("WRONG_GOAL", ProblemKind::kWrongGoal)
      .value("OFF_GRID", ProblemKind::kOffGrid)
      .value("BLOCKED_CELL", ProblemKind::kBlockedCell)
      .value("ILLEGAL_MOVE", ProblemKind::kIllegalMove)
      .value("VERTEX_CONFLICT", ProblemKind::kVertexConflict)
      .value("EDGE_CONFLICT", ProblemKind::kEdgeConflict);

  py::class_<Problem>(m, "Problem")
      .def_readonly("kind", &Problem::kind)
      .def_readonly("agent", &Problem::agent)
      .def_readonly("other", &Problem::other)
      .def_readonly("time", &Problem::time)
      .def_readonly("cell", &Problem::cell)
      .def_readonly("to", &Problem::to)
      .def("__eq__", [](const Problem& a, const Problem& b) { return a == b; }, py::is_operator());

  py::class_<PlanCheck>(m, "PlanCheck")
      .def_readonly("problems", &PlanCheck::problems)
      .def_readonly("costs", &PlanCheck::costs)
      .def_readonly("sum_of_costs", &PlanCheck::sum_of_costs)
      .def_readonly("makespan", &PlanCheck::makespan)
      .def_property_readonly("valid", &PlanCheck::valid);

  m.def("path_cost", [](const Path& path, CellId goal) {
    if (path.empty() || path.back() != goal) {
      throw py::value_error("the path must be non-empty and end at the goal");
    }
    return path_cost(path, goal);
  }, "path"_a, "goal"_a);
  m.def("describe", &describe, "problem"_a, "grid"_a);
  m.def("parse_plan", &parse_plan, "text"_a, "grid"_a, "A plan (a list of paths of cell ids) from a .plan file's text.");
  m.def("load_plan", &load_plan, "path"_a, "grid"_a);
  m.def("write_plan", &write_plan, "plan"_a, "grid"_a);
  m.def("validate", &validate, "instance"_a, "plan"_a,
        "Checks a plan (a list of paths, each a list of cell ids) by DESIGN.md's rules.");
}

void check_table_cell(const ReservationTable& r, CellId c) {
  if (c >= r.cell_count()) {
    throw py::index_error("cell " + std::to_string(c) + " is outside the table's grid of " +
                          std::to_string(r.cell_count()) + " cells");
  }
}

// A wall-time limit in seconds as SearchLimits' deadline, refusing what the planners refuse (negative, NaN).
SearchLimits search_limits(std::uint64_t max_expansions, double time_limit) {
  if (!(time_limit >= 0)) {
    throw py::value_error("the time limit must be a number of seconds, 0 or more");
  }
  SearchLimits limits;
  if (max_expansions != 0) {  // Python's 0 is "no limit", as in PlannerLimits
    limits.max_expansions = max_expansions;
  }
  if (time_limit > 0 && time_limit < 1e9) {
    limits.deadline = std::chrono::steady_clock::now() + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                                             std::chrono::duration<double>(time_limit));
  }
  return limits;
}

void bind_planners(py::module_& m) {
  py::class_<ReservationTable>(m, "ReservationTable",
                               "What a space-time search must avoid, from the searching agent's side.")
      .def(py::init<const Grid&>(), "grid"_a)
      .def(
          "forbid_vertex",
          [](ReservationTable& r, CellId cell, std::uint32_t t) { r.forbid_vertex(cell, t); }, "cell"_a, "t"_a,
          "The agent may not be in cell at time t.")
      .def(
          "forbid_move",
          [](ReservationTable& r, CellId from, CellId to, std::uint32_t t) { r.forbid_move(from, to, t); },
          "from_cell"_a, "to_cell"_a, "t"_a, "The agent may not move from_cell -> to_cell between t and t + 1.")
      .def(
          "forbid_from",
          [](ReservationTable& r, CellId cell, std::uint32_t t) { r.forbid_from(cell, t); }, "cell"_a, "t"_a,
          "The agent may not be in cell at time t or later.")
      .def("reserve_path", &ReservationTable::reserve_path, "path"_a,
           "Another agent's path: its cells, the reverse of its moves, its last cell for ever after.")
      .def(
          "vertex_free",
          [](const ReservationTable& r, CellId cell, std::uint32_t t) {
            check_table_cell(r, cell);
            return r.vertex_free(cell, t);
          },
          "cell"_a, "t"_a)
      .def(
          "move_free",
          [](const ReservationTable& r, CellId from, CellId to, std::uint32_t t) {
            check_table_cell(r, from);
            check_table_cell(r, to);
            // C++ makes this a precondition; checked here, the answer can't depend on what else the table holds
            // (the per-cell marks skip the direction check for a cell with no move entries: the day-2 second review).
            if (!r.neighbours(from, to)) {
              throw py::value_error("a move from cell " + std::to_string(from) + " to cell " + std::to_string(to) +
                                    ", which aren't 4-neighbours");
            }
            return r.move_free(from, to, t);
          },
          "from_cell"_a, "to_cell"_a, "t"_a)
      .def(
          "last_vertex_time",
          [](const ReservationTable& r, CellId cell) {
            check_table_cell(r, cell);
            return r.last_vertex_time(cell);
          },
          "cell"_a)
      .def(
          "forbidden_from",
          [](const ReservationTable& r, CellId cell) {
            check_table_cell(r, cell);
            return r.forbidden_from(cell);
          },
          "cell"_a)
      .def_property_readonly("horizon", &ReservationTable::horizon)
      .def_property_readonly("empty", &ReservationTable::empty)
      .def("__repr__", [](const ReservationTable& r) {
        return "ReservationTable(" + std::to_string(r.vertex_count()) + " vertex, " + std::to_string(r.move_count()) +
               " move, " + std::to_string(r.from_count()) + " from-on entries; horizon " +
               std::to_string(r.horizon()) + ")";
      });
  m.attr("MAX_RESERVATION_TIME") = ReservationTable::kMaxTime;

  py::enum_<SearchStatus>(m, "SearchStatus")
      .value("FOUND", SearchStatus::kFound)
      .value("NO_PATH", SearchStatus::kNoPath)
      .value("LIMIT", SearchStatus::kLimit);
  py::class_<SearchResult>(m, "SearchResult")
      .def_readonly("status", &SearchResult::status)
      .def_readonly("path", &SearchResult::path)
      .def_readonly("expansions", &SearchResult::expansions)
      .def_readonly("generated", &SearchResult::generated);
  m.def(
      "space_time_astar",
      [](const Grid& g, CellId start, CellId goal, const ReservationTable& table, std::uint64_t max_expansions,
         double time_limit) {
        check_cell(g, start);
        check_cell(g, goal);
        if (!table.fits(g)) {
          throw py::value_error("the reservation table was made for a grid of another size");
        }
        const SearchLimits limits = search_limits(max_expansions, time_limit);
        // The search keeps the GIL: `g` and `table` are ordinary mutable Python objects, and with the GIL released
        // another thread could change them mid-search (a forbid_vertex rehashing the set the search is reading
        // crashed the interpreter: the day-2 second review). prioritized_planning can release it, because an
        // Instance can't be changed from Python.
        return space_time_astar(g, start, goal, table, limits);
      },
      "grid"_a, "start"_a, "goal"_a, "table"_a, "max_expansions"_a = 0, "time_limit"_a = 0.0,
      "The earliest-arriving path from start to goal that the table allows (0: no limit).");

  py::enum_<PlanStatus>(m, "PlanStatus")
      .value("SOLVED", PlanStatus::kSolved)
      .value("UNREACHABLE_GOAL", PlanStatus::kUnreachableGoal)
      .value("NO_PATH", PlanStatus::kNoPath)
      .value("LIMIT", PlanStatus::kLimit);
  py::class_<PlanResult>(m, "PlanResult")
      .def_readonly("status", &PlanResult::status)
      .def_readonly("plan", &PlanResult::plan)
      .def_readonly("failed_agent", &PlanResult::failed_agent)
      .def_readonly("sum_of_costs", &PlanResult::sum_of_costs)
      .def_readonly("makespan", &PlanResult::makespan)
      .def_readonly("expansions", &PlanResult::expansions)
      .def_readonly("seconds", &PlanResult::seconds)
      .def_property_readonly("solved", [](const PlanResult& r) { return r.status == PlanStatus::kSolved; });
  m.def(
      "prioritized_planning",
      [](const Instance& instance, std::uint64_t max_expansions, double time_limit,
         const std::optional<std::vector<std::size_t>>& order) {
        PlannerLimits limits;
        limits.max_expansions = max_expansions;
        limits.time_limit_seconds = time_limit;
        const std::vector<std::size_t> o = order.value_or(std::vector<std::size_t>{});
        if (order && order->empty() && !instance.agents.empty()) {
          throw py::value_error("an empty order for " + std::to_string(instance.agents.size()) + " agents");
        }
        py::gil_scoped_release release;  // safe: Python has no way to change an Instance (grid and agents are copies)
        return prioritized_planning(instance, limits, o);
      },
      "instance"_a, "max_expansions"_a = 0, "time_limit"_a = 0.0, "order"_a = py::none(),
      "Prioritized planning: agents one at a time in `order` (default: the instance's), each around those before.");
}

}  // namespace
}  // namespace pathfinder

PYBIND11_MODULE(_core, m) {
  m.doc() = "Multi-agent path finding on grid maps: the C++ library pathfinder.";
  m.attr("__version__") = PATHFINDER_VERSION;
  pathfinder::bind_grid(m);
  pathfinder::bind_formats(m);
  pathfinder::bind_instances(m);
  pathfinder::bind_generators(m);
  pathfinder::bind_plans(m);
  pathfinder::bind_planners(m);
}
