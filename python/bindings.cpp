// The Python face of the library: pathfinder._core. Cells cross the boundary as CellId integers (y * width + x) or as
// (x, y) through Point; C++ exceptions become ValueError (std::invalid_argument, and ParseError, which also carries
// .line), IndexError (std::out_of_range) and RuntimeError (std::runtime_error).

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl/filesystem.h>

#include <array>
#include <string>
#include <vector>

#include "pathfinder/distance.hpp"
#include "pathfinder/generate.hpp"
#include "pathfinder/grid.hpp"
#include "pathfinder/instance.hpp"
#include "pathfinder/movingai.hpp"
#include "pathfinder/plan.hpp"

namespace py = pybind11;
using namespace pybind11::literals;

namespace pathfinder {
namespace {

std::string repr(Point p) { return "Point(" + std::to_string(p.x) + ", " + std::to_string(p.y) + ")"; }

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
      .def("__eq__", [](Point a, Point b) { return a == b; })
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
          "passable", [](const Grid& g, std::int32_t x, std::int32_t y) { return g.passable(Point{x, y}); }, "x"_a,
          "y"_a, "Whether (x, y) is passable; False outside the grid.")
      .def(
          "set_passable",
          [](Grid& g, std::int32_t x, std::int32_t y, bool passable) { g.set_passable(Point{x, y}, passable); },
          "x"_a, "y"_a, "passable"_a)
      .def(
          "id",
          [](const Grid& g, std::int32_t x, std::int32_t y) {
            if (!g.in_bounds(Point{x, y})) {
              throw py::index_error("(" + std::to_string(x) + ", " + std::to_string(y) + ") is outside the grid");
            }
            return g.id(Point{x, y});
          },
          "x"_a, "y"_a, "The cell id of (x, y): y * width + x.")
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
      .def("__eq__", [](const Grid& a, const Grid& b) { return a == b; })
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
      .def("__eq__", [](const ScenarioEntry& a, const ScenarioEntry& b) { return a == b; })
      .def("__repr__", [](const ScenarioEntry& e) {
        return "ScenarioEntry(" + e.map + ", " + repr(e.start) + " -> " + repr(e.goal) + ")";
      });

  py::class_<Scenario>(m, "Scenario")
      .def(py::init<>())
      .def(py::init([](std::vector<ScenarioEntry> entries) { return Scenario{std::move(entries)}; }), "entries"_a)
      .def_readwrite("entries", &Scenario::entries)
      .def("__len__", [](const Scenario& s) { return s.entries.size(); })
      .def("__eq__", [](const Scenario& a, const Scenario& b) { return a == b; });

  m.def("parse_scenario", &parse_scenario, "text"_a);
  m.def("load_scenario", &load_scenario, "path"_a);
  m.def("write_scenario", &write_scenario, "scenario"_a);
}

void bind_instances(py::module_& m) {
  py::class_<Agent>(m, "Agent")
      .def(py::init([](CellId start, CellId goal) { return Agent{start, goal}; }), "start"_a, "goal"_a)
      .def_readwrite("start", &Agent::start)
      .def_readwrite("goal", &Agent::goal)
      .def("__eq__", [](Agent a, Agent b) { return a == b; })
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
      .def_readonly("grid", &Instance::grid)
      .def_readonly("agents", &Instance::agents);

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
      .def("__eq__", [](const Problem& a, const Problem& b) { return a == b; });

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
  m.def("validate", &validate, "instance"_a, "plan"_a,
        "Checks a plan (a list of paths, each a list of cell ids) by DESIGN.md's rules.");
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
}
