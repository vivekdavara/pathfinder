"""Step-by-step execution of a plan, with robots that can be delayed (DESIGN.md "Python and the simulator").

A plan says where every robot is at every time step. Real robots slip: a wheel spins, a person steps in front of
one, a battery swap runs long. The simulator executes a valid plan one time step at a time, lets any robot be
delayed at any step (a delayed robot holds its cell for that step), checks every step against the model's rules by
itself, and records what happened. It executes a plan in one of two ways:

- ``"timetable"``: every robot keeps to its own path, one entry per step, and a delayed robot falls one step behind
  its path for each step it is delayed; nobody else changes anything. This is a plan run as a timetable. A plan has
  no slack, so a delay makes robots collide: each collision is recorded and the run goes on.
- ``"ordered"``: see :class:`Simulator`; delays cost time, never a collision.

    import pathfinder as pf
    from pathfinder.simulator import Simulator, random_delays

    result = pf.cbs(instance, max_nodes=10_000)
    sim = Simulator(instance, result.plan, policy="ordered")
    run = sim.run(random_delays(0.05, seed=1))
    run.sum_of_costs - result.sum_of_costs   # what the delays cost
"""

from __future__ import annotations

import random
from collections.abc import Callable, Iterable, Sequence
from dataclasses import dataclass, field

from . import _core as pf

POLICIES = ("timetable", "ordered")

# A delay schedule: given the time step about to be executed (t, moving the robots from t to t + 1) and the simulator,
# the robots delayed in that step. Robots already finished are ignored.
DelaySchedule = Callable[[int, "Simulator"], Iterable[int]]


class SimulationError(RuntimeError):
    """The simulator broke its own guarantee (a collision or a deadlock under the ordered policy): a bug."""


@dataclass(frozen=True)
class Collision:
    """Two robots breaking the model's rules at one step, with validate()'s conventions.

    ``"vertex"``: robots a < b are both in ``cell`` at ``time``. ``"swap"``: a goes ``cell`` -> ``to`` while b goes
    ``to`` -> ``cell``, between ``time`` and ``time`` + 1.
    """

    kind: str
    a: int
    b: int
    time: int
    cell: int
    to: int | None = None


@dataclass(frozen=True)
class Step:
    """What one call of :meth:`Simulator.step` did: the time it moved the robots to, who moved, who was delayed (of
    the robots not yet finished), and the collisions it found (always empty under the ordered policy)."""

    time: int
    moved: tuple[int, ...]
    delayed: tuple[int, ...]
    collisions: tuple[Collision, ...]


@dataclass(frozen=True)
class Execution:
    """A finished (or stopped) run.

    ``completed``: every robot reached the end of what it had to do. Then ``costs`` is each robot's cost on the
    trajectory it actually drove (DESIGN.md's cost: the last time it arrived at its goal), ``sum_of_costs`` and
    ``makespan`` theirs; otherwise those are empty and 0. ``delay_steps``: robot-steps of delay that applied (to robots
    not yet finished). ``collisions``: every one recorded, in step order.
    """

    policy: str
    completed: bool
    steps: int
    planned_sum_of_costs: int
    costs: list[int] = field(default_factory=list)
    sum_of_costs: int = 0
    makespan: int = 0
    delay_steps: int = 0
    collisions: list[Collision] = field(default_factory=list)

    @property
    def extra_cost(self) -> int:
        """What the run cost beyond the plan: executed sum of costs minus planned (only when completed)."""
        if not self.completed:
            raise ValueError("the run didn't complete")
        return self.sum_of_costs - self.planned_sum_of_costs


class Simulator:
    """Executes a plan for an instance one time step at a time.

    The plan must be valid (``pf.validate``): the simulator is for running good plans badly, not for finding out
    whether a plan is good. Positions start at the robots' starts at time 0; each :meth:`step` moves them to the next
    time.

    Under the ``"ordered"`` policy every robot visits its path's cells in the plan's order, but not on the plan's
    clock: the plan's waits are dropped, and a robot moves into a cell only when it is the next robot the plan sends
    through that cell and the robot before it there has left (or leaves in the same step: following and rotations
    stay allowed). This is executing the plan's action dependency graph (Hönig et al., "Persistent and Robust Execution
    of MAPF Schedules in Warehouses", RA-L 2019). Two robots never meet, because each cell is visited in the plan's
    order one robot at a time; a head-on swap would need the plan to have one; and with no robot delayed in a step,
    the earliest-planned move still to happen can always be made, so nothing deadlocks. With no delays at all, every
    move happens no later than the plan says, so each robot's cost is at most its planned cost. A collision or a
    deadlock would break these arguments, so the simulator raises :class:`SimulationError` if it ever finds one.
    """

    def __init__(self, instance: pf.Instance, plan: Sequence[Sequence[int]], policy: str = "ordered") -> None:
        if policy not in POLICIES:
            raise ValueError(f"policy must be one of {POLICIES}, not {policy!r}")
        check = pf.validate(instance, [list(p) for p in plan])
        if not check.valid:
            raise ValueError(f"the plan isn't valid: {pf.describe(check.problems[0], instance.grid)}")
        self.policy = policy
        self.grid = instance.grid
        self.goals = [a.goal for a in instance.agents]
        self.planned_costs = list(check.costs)
        self.planned_sum_of_costs = check.sum_of_costs
        self.time = 0
        self._paths = [list(p) for p in plan]
        self._neighbours: dict[int, frozenset[int]] = {}
        self._collisions: list[Collision] = []
        self._delay_steps = 0
        n = len(self._paths)
        if policy == "timetable":
            self._index = [0] * n  # each robot's position in its own path
        else:
            self._init_ordered()
        self.positions = [p[0] for p in self._paths]
        self.trajectories = [[c] for c in self.positions]

    # --- what the robots are doing ---

    @property
    def robots(self) -> int:
        return len(self._paths)

    def finished(self, robot: int) -> bool:
        """Whether the robot has done everything its plan asks (it is at its goal and stays there)."""
        if self.policy == "timetable":
            return self._index[robot] == len(self._paths[robot]) - 1
        return self._visit[robot] == len(self._cells[robot]) - 1

    def unfinished(self) -> list[int]:
        return [i for i in range(self.robots) if not self.finished(i)]

    @property
    def done(self) -> bool:
        return all(self.finished(i) for i in range(self.robots))

    @property
    def collisions(self) -> list[Collision]:
        return list(self._collisions)

    # --- executing ---

    def step(self, delayed: Iterable[int] = ()) -> Step:
        """Moves every robot from ``time`` to ``time`` + 1, except the ``delayed`` ones, which hold their cells."""
        held = set()
        for i in delayed:
            if not 0 <= i < self.robots:
                raise IndexError(f"robot {i} of {self.robots}")
            held.add(i)
        held = {i for i in held if not self.finished(i)}
        before = list(self.positions)
        if self.policy == "timetable":
            moved = self._step_timetable(held)
        else:
            moved = self._step_ordered(held)
        self.time += 1
        for i, c in enumerate(self.positions):
            self.trajectories[i].append(c)
        found = self._check_step(before, self.positions)
        if found and self.policy == "ordered":
            raise SimulationError(f"the ordered policy let robots collide: {found[0]}")
        self._collisions.extend(found)
        self._delay_steps += len(held)
        return Step(self.time, tuple(sorted(moved)), tuple(sorted(held)), tuple(found))

    def run(self, delays: DelaySchedule | None = None, max_steps: int | None = None) -> Execution:
        """Steps until every robot is finished, or ``max_steps`` steps have been executed in all (default: no bound for
        a run without delays, which always finishes; ten times the plan's makespan plus 1,000 with them)."""
        if max_steps is None and delays is not None:
            max_steps = 10 * max((len(p) for p in self._paths), default=0) + 1000
        while not self.done and (max_steps is None or self.time < max_steps):
            self.step(delays(self.time, self) if delays is not None else ())
        return self.execution()

    def execution(self) -> Execution:
        """The run so far, as an :class:`Execution`."""
        result = Execution(self.policy, self.done, self.time, self.planned_sum_of_costs,
                           delay_steps=self._delay_steps, collisions=list(self._collisions))
        if not result.completed:
            return result
        costs = [pf.path_cost(t, g) for t, g in zip(self.trajectories, self.goals)]
        return Execution(self.policy, True, self.time, self.planned_sum_of_costs, costs, sum(costs),
                         max(costs, default=0), self._delay_steps, list(self._collisions))

    # --- the timetable policy ---

    def _step_timetable(self, held: set[int]) -> list[int]:
        moved = []
        for i, path in enumerate(self._paths):
            if i in held or self._index[i] == len(path) - 1:
                continue
            self._index[i] += 1
            if path[self._index[i]] != self.positions[i]:
                moved.append(i)
            self.positions[i] = path[self._index[i]]
        return moved

    # --- the ordered policy ---

    def _init_ordered(self) -> None:
        # Each robot's path as its visits: the cells it goes through, the plan's waits dropped. A visit is (robot, k):
        # robot's k-th cell. Each cell's visits are queued in the order the plan enters them (no two at the same time:
        # that would be a vertex conflict), and _next[cell] is the place in the queue of the next visit allowed in.
        self._cells: list[list[int]] = []
        entries = []
        for i, path in enumerate(self._paths):
            cells: list[int] = []
            for t, c in enumerate(path):
                if not cells or cells[-1] != c:
                    entries.append((t, c, i, len(cells)))
                    cells.append(c)
            self._cells.append(cells)
        entries.sort()
        self._queue: dict[int, list[tuple[int, int]]] = {}
        for _, c, i, k in entries:
            self._queue.setdefault(c, []).append((i, k))
        self._next = dict.fromkeys(self._queue, 0)
        self._visit = [0] * len(self._paths)
        # Who is in each occupied cell. Every robot starts in its first visit, the first in its cell's queue (it
        # entered at time 0).
        self._occupant: dict[int, int] = {}
        for i, cells in enumerate(self._cells):
            assert self._queue[cells[0]][0] == (i, 0)
            self._next[cells[0]] = 1
            self._occupant[cells[0]] = i

    def _step_ordered(self, held: set[int]) -> list[int]:
        # Who may move: a robot not delayed, with a cell still to go to, whose visit there is the next in that cell's
        # queue. (The robot before it in the queue may still be in the cell; it is the only one who can be.)
        want: dict[int, int] = {}
        for i, cells in enumerate(self._cells):
            k = self._visit[i]
            if i in held or k == len(cells) - 1:
                continue
            v = cells[k + 1]
            if self._queue[v][self._next[v]] == (i, k + 1):
                want[i] = v
        # Who does move: a robot whose cell-to-be is empty, or whose occupant moves out in the same step. Dropping the
        # others until nothing changes leaves chains of robots following each other and rotations around cycles.
        moving = set(want)
        changed = True
        while changed:
            changed = False
            for i in list(moving):
                occupant = self._occupant.get(want[i])
                if occupant is not None and occupant not in moving:
                    moving.discard(i)
                    changed = True
        for i in moving:
            occupant = self._occupant.get(want[i])
            if occupant is not None and want[occupant] == self.positions[i]:
                raise SimulationError(f"robots {i} and {occupant} would swap between {self.time} and {self.time + 1}")
        if not moving and not held and not self.done:
            raise SimulationError(f"no robot can move at time {self.time} and none is delayed: a deadlock")
        for i in moving:
            del self._occupant[self.positions[i]]
        for i in moving:
            v = want[i]
            self._occupant[v] = i
            self._next[v] += 1
            self._visit[i] += 1
            self.positions[i] = v
        return list(moving)

    # --- the checks ---

    def _neighbours_of(self, cell: int) -> frozenset[int]:
        found = self._neighbours.get(cell)
        if found is None:
            found = self._neighbours[cell] = frozenset(self.grid.neighbors(cell))
        return found

    def _check_step(self, before: list[int], after: list[int]) -> list[Collision]:
        """The step's collisions, found from the positions alone (not from how the policy chose them), as validate()
        would report them on the trajectories: every pair of robots in one cell, every head-on swap. An illegal move
        can only be a bug in the simulator itself, so it raises."""
        t = self.time - 1
        for i, (u, v) in enumerate(zip(before, after)):
            if u != v and v not in self._neighbours_of(u):
                raise SimulationError(f"robot {i} jumped from cell {u} to cell {v} between {t} and {t + 1}")
        found = []
        by_cell: dict[int, list[int]] = {}
        for i, v in enumerate(after):
            by_cell.setdefault(v, []).append(i)
        for cell, robots in by_cell.items():
            for x in range(len(robots)):
                for y in range(x + 1, len(robots)):
                    found.append(Collision("vertex", robots[x], robots[y], t + 1, cell))
        # Robots by move. A list, not one robot: under the timetable policy two robots that already share a cell can
        # make the same move, and each of them swaps with whoever comes the other way.
        moves: dict[tuple[int, int], list[int]] = {}
        for i, (u, v) in enumerate(zip(before, after)):
            if u != v:
                moves.setdefault((u, v), []).append(i)
        for (u, v), robots in moves.items():
            for i in robots:
                for j in moves.get((v, u), ()):
                    if i < j:
                        found.append(Collision("swap", i, j, t, u, v))
        found.sort(key=lambda c: (c.kind != "vertex", c.a, c.b))
        return found


def delay_robot(robot: int, at: int, steps: int = 1) -> DelaySchedule:
    """A schedule delaying one robot for ``steps`` consecutive steps, from the step that starts at time ``at``."""
    if steps < 0 or at < 0:
        raise ValueError("at and steps must be 0 or more")
    return lambda t, sim: (robot,) if at <= t < at + steps else ()


def random_delays(probability: float, seed: int) -> DelaySchedule:
    """A schedule delaying each robot at each step with ``probability``, independently. A draw is made for every robot
    at every step, finished or not, so two runs with the same seed see the same delays at the same steps whatever the
    policy (a draw for a finished robot just doesn't apply)."""
    if not 0 <= probability <= 1:
        raise ValueError("the probability must be between 0 and 1")
    rng = random.Random(seed)

    def schedule(t: int, sim: Simulator) -> list[int]:
        return [i for i in range(sim.robots) if rng.random() < probability]

    return schedule
