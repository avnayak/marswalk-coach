"""Marswalk Coach — Field D* route planner (team spacebuzz).

Plans the walking route from START to the Mount Sharp summit with Field D*,
the interpolation-based planner from Ferguson & Stentz (2006), "Using
Interpolation to Improve Path Planning: The Field D* Algorithm", Journal of
Field Robotics 23(2), pp. 79-101 (tech report CMU-TR-RI-05-19,
https://www.ri.cmu.edu/pub_files/pub4/ferguson_david_2005_3/ferguson_david_2005_3.pdf).
Field D* flew on NASA's Mars Exploration Rovers (Spirit and Opportunity)
as their global path planner.

Why Field D* instead of A*? A* only allows 8 headings (multiples of 45
degrees), so its paths zigzag. Field D* uses linear interpolation between
neighbouring cells to estimate the cost of any point in the grid, so its
paths are smooth, with any heading — and it can replan fast when the world
changes, updating only the cells the change affects instead of planning
from scratch.

How it works (paper Figures 7 and 8):
  1. Nodes sit at the corners of grid cells; each cell has a traversal
     cost from the MOLA elevation grid (steep ground costs more; slopes
     over 25 degrees are blocked, JPL's rover-driving limit).
  2. The search runs backwards from the goal (D* Lite): each node holds
     g (cost to the goal) and rhs (one-step lookahead). rhs is computed
     with the paper's ComputeCost — linear interpolation across the edge
     between two neighbouring nodes, with a closed-form minimum.
  3. The path is extracted by gradient descent on the interpolated
     cost-to-go field: from the current point, the next waypoint is the
     best point on a cell boundary, with the paper's one-step lookahead.
  4. When costs change (a new danger appears), only the corners of the
     changed cells are updated and the search repairs the plan — the
     replanning demo below shows this.

The route is planned as three legs through the science stops, which fixes
the A* planner's weakness (it went straight and skipped the stops):
  START -> Boxwork -> High ground -> Mount Sharp summit.
Then a replan demo adds a fresh rock field on the second leg and replans
it, the way the coach would reroute the astronaut mid-walk.

After the replan demo, the planner also learns: it writes the new rock
field into obstacles.json as OBS-04, and on the next run it reads that
entry back and plans around it from the start. The route in
field_dstar_route.json is the first walk's route, planned before the
danger was found; a later
run plans the longer detour instead, and prints that it did.

Run it with: python3 field_dstar.py
It writes field_dstar_route.json (waypoints, distance, climb, score,
replan numbers). Run the self-tests with: python3 field_dstar.py --test

Honest limits: the grid is 463 m per cell, so small rocks and narrow
cracks are invisible. The danger-spot positions are APPROXIMATE (see
astar_route.py). The 25/12 degree slope limits are JPL's rover-driving
limits, used here as a stand-in for a walker's limits. Field D* needs one
cost per cell, so the A* climb penalty — which depends on which way you
step — is not in the Field D* cost; slope already captures steepness.
Start and goal are snapped to the top-left corner of the stop's cell for
the search (at most half a cell, about 230 m); the extracted path still starts and ends
at the true stop positions. The search uses a zero heuristic (see the
FieldDStar docstring): a focused search leaves the cost-to-go undefined
off the optimal path, which breaks smooth path extraction. This is a
demo planner, not flight software.
"""

import heapq
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))

from astar_route import (
    Grid, haversine_km, bresenham, length_class,
    SLOPE_BLOCKED_DEG, SLOPE_COST_FROM_DEG, OBSTACLE_EXTRA,
)

try:
    from route_scorer import climb_points_from_meters
except ImportError:
    def climb_points_from_meters(total_climb_m):
        if total_climb_m < 1000:
            return 2
        if total_climb_m <= 5000:
            return 1
        return 0

INF = float("inf")
SQRT2 = math.sqrt(2.0)

START = (137.4417, -4.5895)     # lon E, lat S
BOXWORK = (137.5308, -4.7197)
HIGH_GROUND = (137.75, -5.0)
SUMMIT = (137.793, -4.981)
LEGS = [
    ("START to Boxwork", START, BOXWORK),
    ("Boxwork to High ground", BOXWORK, HIGH_GROUND),
    ("High ground to Mount Sharp", HIGH_GROUND, SUMMIT),
]

ROCK_FIELD_RADIUS_M = 1000.0  # the new danger in the replan demo
ROCK_FIELD_AT_FRACTION = 0.45  # where on leg 2 it appears (45% along)

OBSTACLES_PATH = os.path.join(HERE, "obstacles.json")
LEARNED_OBSTACLE_ID = "OBS-04"  # the id the learn step writes


def compute_cost(g1, g2, c, b):
    """Paper Figure 7: path cost of a node via the edge between two of
    its consecutive neighbours, minimised over every candidate path.

    g1 = cost-to-go of the edge-adjacent neighbour (s1), g2 = cost-to-go
    of the diagonal neighbour (s2). c = traversal cost of the cell with
    corners s, s1, s2; b = traversal cost of the cell sharing the edge
    (s, s1) on the other side. Returns the cheapest cost of a path from
    s to the goal through this edge, or inf if there is none.

    One correction to the paper's Figure 7: its case analysis only
    considers the straight path to s1 (cost min(c, b) + g1) when
    g1 <= g2, but that path can be the cheapest even when g1 > g2
    (for example, when the bottom cell is very cheap). We therefore
    take the minimum of the paper's value and the straight-to-s1
    cost, so this returns the true minimum of the paper's Section 3
    objective — verified against brute-force minimisation in the
    test suite (test_compute_cost). Without this, rhs values can be
    overestimated and paths suboptimal.
    """
    m = c if c < b else b
    if m == INF:
        return INF
    if g1 == INF and g2 == INF:
        return INF
    if g2 == INF:
        return m + g1
    if g1 <= g2:
        return m + g1
    # g1 > g2: the diagonal neighbour is cheaper, so the best path may
    # cut across the cell towards it — but the straight path to s1 can
    # still win, so it stays a candidate (see the docstring).
    straight = m + g1
    f = g1 - g2 if g1 != INF else INF
    if f <= b:
        if c <= f:
            vs = c * SQRT2 + g2
        else:
            d = c * c - f * f
            if d <= 0.0:
                vs = c * SQRT2 + g2
            else:
                y = f / math.sqrt(d)
                if y > 1.0:
                    y = 1.0
                vs = c * math.sqrt(1.0 + y * y) + f * (1.0 - y) + g2
    elif c <= b:
        vs = c * SQRT2 + g2
    else:
        d = c * c - b * b
        if d <= 0.0:
            vs = c * SQRT2 + g2
        else:
            t = b / math.sqrt(d)
            if t > 1.0:
                t = 1.0
            x = 1.0 - t
            vs = c * math.sqrt(1.0 + (1.0 - x) ** 2) + b * x + g2
    return vs if vs < straight else straight


# Consecutive neighbour pairs around a corner node s = (i, j), from the
# paper's Figure 1(c). Each entry: (s1 offset, s2 offset, centre-cell
# offset, bottom-cell offset). s1 is edge-adjacent to s, s2 is diagonal.
# The centre cell has corners s, s1, s2; the bottom cell shares the edge
# (s, s1) with it on the other side.
PAIRS = (
    ((0, 1), (1, 1), (0, 0), (-1, 0)),      # E / SE
    ((1, 0), (1, 1), (0, 0), (0, -1)),      # S / SE
    ((1, 0), (1, -1), (0, -1), (0, 0)),     # S / SW
    ((0, -1), (1, -1), (0, -1), (-1, -1)),  # W / SW
    ((0, -1), (-1, -1), (-1, -1), (0, -1)), # W / NW
    ((-1, 0), (-1, -1), (-1, -1), (-1, 0)), # N / NW
    ((-1, 0), (-1, 1), (-1, 0), (-1, -1)),  # N / NE
    ((0, 1), (-1, 1), (-1, 0), (0, 0)),    # E / NE
)

NEIGHBOURS = tuple(
    (di, dj) for di in (-1, 0, 1) for dj in (-1, 0, 1) if (di, dj) != (0, 0)
)


class FieldDStar:
    """Field D* (Ferguson & Stentz 2006, Figures 7 and 8).

    Nodes sit at the corners of the cost grid's cells. `costs` is a
    rows x cols list of per-cell traversal costs (inf = blocked). The
    search runs backwards from the goal (D* Lite): call plan() once,
    then replan_after_cost_change() to repair the plan when costs
    change, and extract_path() to get the smooth route.

    `heuristic_weight` scales the D* Lite heuristic (Euclidean
    distance to the start). It defaults to 0.0 — a Dijkstra-like,
    unfocused search — because a focused search leaves the cost-to-go
    undefined off the optimal path, and smooth path extraction needs
    accurate cost-to-go values all around the path. The interpolation
    (Field D*'s core) is unchanged; only the search order differs.
    """

    def __init__(self, costs, heuristic_weight=0.0):
        self.rows = len(costs)
        self.cols = len(costs[0])
        self.heuristic_weight = heuristic_weight
        # The planner owns a copy: replanning changes costs here only.
        self.costs = [row[:] for row in costs]
        self.g = [[INF] * (self.cols + 1) for _ in range(self.rows + 1)]
        self.rhs = [[INF] * (self.cols + 1) for _ in range(self.rows + 1)]
        self.open_heap = []
        self.open_keys = {}
        self._counter = 0
        self.start = None
        self.goal = None
        self.pops = 0       # states expanded by ComputeShortestPath
        self.updates = 0    # UpdateState calls

    # -- grid helpers -------------------------------------------------
    def cell_cost(self, r, c):
        if 0 <= r < self.rows and 0 <= c < self.cols:
            return self.costs[r][c]
        return INF

    def _in_nodes(self, i, j):
        return 0 <= i <= self.rows and 0 <= j <= self.cols

    def _heuristic(self, i, j):
        # Euclidean distance to the start, times the minimum cell cost
        # (1.0) and the heuristic weight: admissible, per the paper's
        # footnote on Figure 8. Weight 0.0 (the default) is a
        # Dijkstra-like unfocused search; see the class docstring.
        si, sj = self.start
        return self.heuristic_weight * math.hypot(i - si, j - sj)

    def value(self, i, j):
        """Best cost-to-go estimate at a node, for path extraction.
        The focused search leaves g = inf for states off the optimal
        path, but their rhs (the one-step lookahead from UpdateState)
        is finite wherever a neighbour was expanded — so min(g, rhs)
        is the best available estimate there. On the optimal path
        g == rhs, so this equals g."""
        return self.g[i][j] if self.g[i][j] < self.rhs[i][j] \
            else self.rhs[i][j]

    def _key(self, i, j):
        m = self.g[i][j] if self.g[i][j] < self.rhs[i][j] else self.rhs[i][j]
        return (m + self._heuristic(i, j), m)

    # -- paper Figure 8 -----------------------------------------------
    def update_state(self, i, j):
        """UpdateState: recompute rhs(s) from the interpolated costs
        of its consecutive neighbour pairs, then (re)insert s into
        OPEN if it is inconsistent (g != rhs)."""
        self.updates += 1
        if (i, j) != self.goal:
            best = INF
            g = self.g
            for (d1i, d1j), (d2i, d2j), (dci, dcj), (dbi, dbj) in PAIRS:
                i1, j1 = i + d1i, j + d1j
                i2, j2 = i + d2i, j + d2j
                if not (self._in_nodes(i1, j1) and self._in_nodes(i2, j2)):
                    continue
                v = compute_cost(
                    g[i1][j1], g[i2][j2],
                    self.cell_cost(i + dci, j + dcj),
                    self.cell_cost(i + dbi, j + dbj),
                )
                if v < best:
                    best = v
            self.rhs[i][j] = best
        # Lazy removal from OPEN: the heap entry goes stale and is
        # skipped when popped.
        self.open_keys.pop((i, j), None)
        if self.g[i][j] != self.rhs[i][j]:
            key = self._key(i, j)
            self.open_keys[(i, j)] = key
            heapq.heappush(self.open_heap,
                           (key[0], key[1], self._counter, i, j))
            self._counter += 1

    def _min_key(self):
        """(key, i, j) of the smallest valid OPEN entry, or Nones.
        Stale heap entries are discarded as they surface."""
        heap = self.open_heap
        keys = self.open_keys
        while heap:
            k1, k2, _, i, j = heap[0]
            if keys.get((i, j)) == (k1, k2):
                return (k1, k2), i, j
            heapq.heappop(heap)
        return None, None, None

    def compute_shortest_path(self):
        """ComputeShortestPath: expand states in key order until the
        start is consistent and no OPEN key is smaller than its key."""
        max_pops = 100 * (self.rows + 1) * (self.cols + 1)
        si, sj = self.start
        while True:
            key, i, j = self._min_key()
            if key is None:
                return
            if not (key < self._key(si, sj) or
                    self.rhs[si][sj] != self.g[si][sj]):
                return
            heapq.heappop(self.open_heap)
            self.open_keys.pop((i, j), None)
            self.pops += 1
            if self.pops > max_pops:
                raise RuntimeError("ComputeShortestPath did not "
                                   "terminate (safety cap hit)")
            if self.g[i][j] > self.rhs[i][j]:
                self.g[i][j] = self.rhs[i][j]
                for di, dj in NEIGHBOURS:
                    ni, nj = i + di, j + dj
                    if self._in_nodes(ni, nj):
                        self.update_state(ni, nj)
            else:
                self.g[i][j] = INF
                self.update_state(i, j)
                for di, dj in NEIGHBOURS:
                    ni, nj = i + di, j + dj
                    if self._in_nodes(ni, nj):
                        self.update_state(ni, nj)

    def plan(self, start, goal):
        """Fresh plan from `start` to `goal` (corner nodes). Returns
        True if the goal is reachable."""
        self.start = start
        self.goal = goal
        for row in self.g:
            for j in range(len(row)):
                row[j] = INF
        for row in self.rhs:
            for j in range(len(row)):
                row[j] = INF
        self.open_heap = []
        self.open_keys = {}
        self.pops = 0
        self.updates = 0
        self.rhs[goal[0]][goal[1]] = 0.0
        key = self._key(*goal)
        self.open_keys[goal] = key
        heapq.heappush(self.open_heap,
                       (key[0], key[1], self._counter, goal[0], goal[1]))
        self._counter += 1
        self.compute_shortest_path()
        return self.g[start[0]][start[1]] != INF

    def replan_after_cost_change(self, changed_cells):
        """Repair the plan after cell costs changed (paper Figure 8,
        Main lines 20-23). `changed_cells` is a list of (r, c) cells
        whose cost in self.costs was already updated. Only the corner
        states of those cells are touched; the search then repairs the
        plan. Returns True if the goal is still reachable."""
        seen = set()
        for r, c in changed_cells:
            for i, j in ((r, c), (r, c + 1), (r + 1, c), (r + 1, c + 1)):
                if (i, j) not in seen and self._in_nodes(i, j):
                    seen.add((i, j))
                    self.update_state(i, j)
        self.compute_shortest_path()
        return self.g[self.start[0]][self.start[1]] != INF


def _best_on_edge(p, a, b, ga, gb, c):
    """Best point on the segment a-b for a path from p: minimises
    c * |p - q| + interpolated g at q. Golden-section search (the
    function is convex in t). Returns (t, cost) or None if the edge
    is unusable (an endpoint has infinite cost)."""
    if ga == INF or gb == INF or c == INF:
        return None
    pr, pc = p
    ar, ac = a
    br, bc = b
    dr, dc = br - ar, bc - ac

    def cost(t):
        qr, qc = ar + t * dr, ac + t * dc
        return c * math.hypot(qr - pr, qc - pc) + (1.0 - t) * ga + t * gb

    gr = (math.sqrt(5.0) - 1.0) / 2.0
    x0, x1 = 0.0, 1.0
    x2 = x1 - gr * (x1 - x0)
    x3 = x0 + gr * (x1 - x0)
    f2, f3 = cost(x2), cost(x3)
    for _ in range(30):
        if f2 > f3:
            x0 = x2
            x2, f2 = x3, f3
            x3 = x0 + gr * (x1 - x0)
            f3 = cost(x3)
        else:
            x1 = x3
            x3, f3 = x2, f2
            x2 = x1 - gr * (x1 - x0)
            f2 = cost(x2)
    t = 0.5 * (x0 + x1)
    # Snap to an endpoint when the optimum sits on one: the search
    # converges to ~1e-7, and a point a hair off a corner would make
    # the next step miss the cells touching that corner.
    if t < 1e-6:
        t = 0.0
    elif t > 1.0 - 1e-6:
        t = 1.0
    return t, cost(t)


# The four edges of a cell (r, c): ((corner A), (corner B), adjacent cell).
def _cell_edges(r, c):
    return (
        ((r, c), (r, c + 1), (r - 1, c)),          # north
        ((r + 1, c), (r + 1, c + 1), (r + 1, c)),  # south
        ((r, c), (r + 1, c), (r, c - 1)),          # west
        ((r, c + 1), (r + 1, c + 1), (r, c + 1)),  # east
    )


def _downsample(waypoints, min_turn_deg=2.0, max_gap_cells=5.0):
    """Thin a polyline: keep waypoints where the heading changes, plus
    at least one every ~5 cells. First and last points are kept."""
    if len(waypoints) <= 2:
        return list(waypoints)
    out = [waypoints[0]]
    prev = waypoints[0]
    for i in range(1, len(waypoints) - 1):
        p = waypoints[i]
        nxt = waypoints[i + 1]
        v1 = (p[0] - prev[0], p[1] - prev[1])
        v2 = (nxt[0] - p[0], nxt[1] - p[1])
        d1 = math.hypot(*v1)
        d2 = math.hypot(*v2)
        keep = d1 >= max_gap_cells
        if not keep and d1 > 1e-9 and d2 > 1e-9:
            cosang = (v1[0] * v2[0] + v1[1] * v2[1]) / (d1 * d2)
            cosang = max(-1.0, min(1.0, cosang))
            keep = math.degrees(math.acos(cosang)) >= min_turn_deg
        if keep:
            out.append(p)
            prev = p
    out.append(waypoints[-1])
    return out


def _candidate_cells(p, rows, cols):
    """Cells touching point p (corner coordinates). A point strictly
    inside a cell touches one cell; a point on an edge touches two;
    a point on a corner touches four. This matters because the path
    often lands exactly on corners, where the 'current cell' is
    ambiguous — the next step must consider every cell around the
    point, or the path gets trapped in the wrong one. Coordinates
    within 1e-6 of an integer count as on the boundary (the edge
    search converges to ~1e-7, so a hair's-breadth miss must not
    hide a touching cell)."""
    eps = 1e-6
    pr = round(p[0]) if abs(p[0] - round(p[0])) < eps else p[0]
    pc = round(p[1]) if abs(p[1] - round(p[1])) < eps else p[1]
    r0 = int(math.floor(pr))
    c0 = int(math.floor(pc))
    rows_here = [r0]
    cols_here = [c0]
    if abs(pr - r0) < eps and r0 > 0:
        rows_here.append(r0 - 1)
    if abs(pc - c0) < eps and c0 > 0:
        cols_here.append(c0 - 1)
    return [(r, c) for r in rows_here for c in cols_here
            if 0 <= r < rows and 0 <= c < cols]


# Path extraction methods, added to FieldDStar below (kept separate so
# the core search above reads cleanly against the paper's Figure 8).

def _field_dstar_lookahead(self, q, adj, exclude_edge):
    """Recompute the cost-to-go of point q (which sits on an edge of
    the adjacent cell `adj`) via the other edges of that cell — the
    paper's Section 6 one-step lookahead. Returns the refined cost,
    or None if it cannot be computed."""
    r, c = adj
    if not (0 <= r < self.rows and 0 <= c < self.cols):
        return None
    cell_c = self.costs[r][c]
    if cell_c == INF:
        return None
    best = None
    for a, b, _ in _cell_edges(r, c):
        if (a, b) == exclude_edge or (b, a) == exclude_edge:
            continue
        res = _best_on_edge(q, a, b, self.value(a[0], a[1]),
                            self.value(b[0], b[1]), cell_c)
        if res is None:
            continue
        if best is None or res[1] < best:
            best = res[1]
    return best


def _field_dstar_best_step(self, p, goal_node):
    """Best next waypoint from point p: the point on a boundary of
    one of the cells touching p minimising (travel cost from p) +
    (refined cost-to-go). Returns the next point, or None."""
    best = None  # (total cost, point)
    for r, c in _candidate_cells(p, self.rows, self.cols):
        cell_c = self.costs[r][c]
        if cell_c == INF:
            continue
        for a, b, adj in _cell_edges(r, c):
            res = _best_on_edge(p, a, b, self.value(a[0], a[1]),
                                self.value(b[0], b[1]), cell_c)
            if res is None:
                continue
            t, edge_cost = res
            q = (a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))
            if math.hypot(q[0] - p[0], q[1] - p[1]) < 0.05:
                continue  # too small a step: the descent would crawl
            nr, nc = adj
            if not (0 <= nr < self.rows and 0 <= nc < self.cols):
                # Boundary edge: usable only if the goal sits on it.
                if math.hypot(q[0] - goal_node[0],
                              q[1] - goal_node[1]) >= 0.75:
                    continue
                total = edge_cost
            else:
                refined = self._lookahead(q, adj, (a, b))
                if refined is None:
                    total = edge_cost
                else:
                    total = (cell_c * math.hypot(q[0] - p[0],
                                                 q[1] - p[1]) + refined)
            if best is None or total < best[0]:
                best = (total, q)
    if best is None:
        return None
    return best[1]


def _field_dstar_nudge(self, p):
    """Fallback: step 0.3 cell towards the lowest-value corner among
    the cells touching p (excluding corners p already sits on).
    Returns the new point, or None."""
    best = None
    for r, c in _candidate_cells(p, self.rows, self.cols):
        for i, j in ((r, c), (r, c + 1), (r + 1, c), (r + 1, c + 1)):
            if math.hypot(i - p[0], j - p[1]) < 0.02:
                continue
            v = self.value(i, j)
            if v != INF and (best is None or v < best[0]):
                best = (v, (float(i), float(j)))
    if best is None:
        return None
    target = best[1]
    d = math.hypot(target[0] - p[0], target[1] - p[1])
    step = min(0.3, d)
    return (p[0] + (target[0] - p[0]) / d * step,
            p[1] + (target[1] - p[1]) / d * step)


def _field_dstar_extract_path(self, start_point, goal_node,
                              max_steps=20000):
    """Gradient descent on the interpolated cost-to-go field (paper
    Section 6): from the current point, move to the best point on a
    boundary of one of the cells touching it, with the paper's
    one-step lookahead. `start_point` is a continuous (row, col) in
    corner coordinates; `goal_node` is a corner node. Returns a list
    of continuous waypoints, or None if extraction fails."""
    goal_r, goal_c = goal_node
    p = (float(start_point[0]), float(start_point[1]))
    path = [p]
    visited = {(round(p[0], 3), round(p[1], 3))}
    for _ in range(max_steps):
        if math.hypot(p[0] - goal_r, p[1] - goal_c) < 0.75:
            path.append((float(goal_r), float(goal_c)))
            return path
        q = self._best_step(p, goal_node)
        if q is None:
            q = self._nudge(p)
            if q is None:
                return None
        if math.hypot(q[0] - p[0], q[1] - p[1]) < 0.05:
            q = self._nudge(p)
            if q is None:
                return None
        key = (round(q[0], 3), round(q[1], 3))
        # A least-cost path never revisits a point: returning to one
        # means the descent is looping, so fail honestly.
        if key in visited:
            return None
        visited.add(key)
        path.append(q)
        p = q
    return None


FieldDStar._lookahead = _field_dstar_lookahead
FieldDStar._best_step = _field_dstar_best_step
FieldDStar._nudge = _field_dstar_nudge
FieldDStar.extract_path = _field_dstar_extract_path


# ----------------------------------------------------------------------
# Mars helpers: cost grid, stats, scoring
# ----------------------------------------------------------------------

def build_costs(grid):
    """Per-cell traversal cost from the MOLA grid: 1 + slope cost,
    extra inside a danger zone, infinite where the slope is over
    JPL's 25-degree limit. (Field D* needs one cost per cell, so the
    A* climb penalty — which depends on step direction — is not
    included; slope already captures steepness.)"""
    costs = []
    for r in range(grid.rows):
        row = []
        for c in range(grid.cols):
            if grid.blocked[r][c]:
                row.append(INF)
            else:
                v = 1.0 + grid.slope_cost(r, c)
                if grid.obstacle[r][c]:
                    v += OBSTACLE_EXTRA
                row.append(v)
        costs.append(row)
    return costs


def load_learned_obstacles(grid, costs):
    """Read obstacles.json and block the cells of any learned entry —
    a danger found on an earlier walk, stored with lon/lat — so a
    fresh plan avoids it from the start, with no replan needed.
    Entries without coordinates are left alone: OBS-01/OBS-02 are
    already in the Grid's danger layer, and OBS-03 (the climb) is
    scored by the climb rule. Returns the list of obstacle ids
    applied. A missing or unreadable log is not an error: the
    planner just plans without learned dangers."""
    applied = []
    try:
        with open(OBSTACLES_PATH) as f:
            log = json.load(f)
    except (OSError, ValueError):
        print("No obstacle log found; planning without learned dangers.")
        return applied
    for entry in log.get("obstacles", []):
        if "lon_e" not in entry or "lat" not in entry:
            continue
        radius_m = entry.get("radius_m", ROCK_FIELD_RADIUS_M)
        r0, c0 = grid.cell_of(entry["lon_e"], entry["lat"])
        radius_cells = radius_m / grid.cell_m
        rr = int(math.ceil(radius_cells))
        n = 0
        for dr in range(-rr, rr + 1):
            for dc in range(-rr, rr + 1):
                r, c = r0 + dr, c0 + dc
                if not (0 <= r < grid.rows and 0 <= c < grid.cols):
                    continue
                if math.hypot(dr, dc) * grid.cell_m > radius_m:
                    continue
                if costs[r][c] != INF:
                    costs[r][c] = INF
                    n += 1
        print("Learned danger {}: {} cells blocked around {:.4f}E, "
              "{:.4f}S (remembered from an earlier walk)".format(
                  entry["id"], n, entry["lon_e"], -entry["lat"]))
        applied.append(entry["id"])
    return applied


def learn_obstacle(centre_lon, centre_lat, cells_blocked):
    """Remember the replan demo's rock field: write it into
    obstacles.json as OBS-04, so the next run plans around it from
    the start. The entry is honest about being a demo — a simulated
    sighting, not a real observation. Idempotent: if OBS-04 is
    already in the log, nothing is written. Returns True if a new
    entry was written."""
    try:
        with open(OBSTACLES_PATH) as f:
            log = json.load(f)
    except (OSError, ValueError):
        print("Could not read {}; the new rock field was not "
              "remembered.".format(OBSTACLES_PATH))
        return False
    entries = log.get("obstacles", [])
    if any(e.get("id") == LEARNED_OBSTACLE_ID for e in entries):
        print("The map already remembers {} — nothing new to "
              "learn.".format(LEARNED_OBSTACLE_ID))
        return False
    entries.append({
        "confidence": "Demo",
        "confidence_note": "Simulated sighting from the replan demo, "
                           "not a real observation.",
        "id": LEARNED_OBSTACLE_ID,
        "lat": round(centre_lat, 4),
        "location": "Boxwork to High ground, Gale Crater "
                    "({:.4f}E, {:.4f}S)".format(centre_lon, -centre_lat),
        "lon_e": round(centre_lon, 4),
        "radius_m": ROCK_FIELD_RADIUS_M,
        "source": "Field D* replan demo — simulated sighting, shows "
                  "how the map would learn from a real sighting",
        "type": "Fresh rock field, found during the walk",
    })
    note = log.get("note", "")
    note = note.replace("3 obstacles", "4 obstacles")
    log["note"] = note + " OBS-04 was learned by the Field D* replan " \
        "demo: a fresh rock field it found and remembered, so the " \
        "next walk avoids it from the start."
    with open(OBSTACLES_PATH, "w") as f:
        json.dump(log, f, indent=1, sort_keys=True)
    print("The map learned: {} written to obstacles.json ({} cells "
          "around {:.4f}E, {:.4f}S).".format(
              LEARNED_OBSTACLE_ID, cells_blocked,
              centre_lon, -centre_lat))
    return True


def corner_lonlat(grid, i, j):
    """Corner node (i, j) -> (lon, lat). Corner (i, j) is the boundary
    between cells; cell (r, c) spans corners (r..r+1, c..c+1)."""
    lon = grid.west + j / grid.ppd
    lat = grid.north - i / grid.ppd
    return lon, lat


def _clamp_cell(grid, r, c):
    return (min(max(r, 0), grid.rows - 1),
            min(max(c, 0), grid.cols - 1))


def leg_stats(grid, waypoints):
    """Distance (km) and total climb (m) along continuous waypoints
    (corner coordinates). Elevation is sampled at every grid cell
    each segment passes through, like astar_route.path_stats."""
    total_km = 0.0
    total_climb = 0
    prev_elev = None
    for a, b in zip(waypoints, waypoints[1:]):
        lon1, lat1 = corner_lonlat(grid, a[0], a[1])
        lon2, lat2 = corner_lonlat(grid, b[0], b[1])
        total_km += haversine_km(lon1, lat1, lon2, lat2)
        ca = _clamp_cell(grid, int(math.floor(a[0])), int(math.floor(a[1])))
        cb = _clamp_cell(grid, int(math.floor(b[0])), int(math.floor(b[1])))
        for r, c in bresenham(ca[0], ca[1], cb[0], cb[1]):
            elev = grid.grid[r][c]
            if prev_elev is not None and elev > prev_elev:
                total_climb += elev - prev_elev
            prev_elev = elev
    return total_km, total_climb


def zones_crossed(grid, waypoints):
    """Danger zones the polyline enters (same rule as astar_route)."""
    hit = set()
    for a, b in zip(waypoints, waypoints[1:]):
        ca = _clamp_cell(grid, int(math.floor(a[0])), int(math.floor(a[1])))
        cb = _clamp_cell(grid, int(math.floor(b[0])), int(math.floor(b[1])))
        for r, c in bresenham(ca[0], ca[1], cb[0], cb[1]):
            if grid.obstacle[r][c]:
                lon, lat = grid.lonlat_of(r, c)
                best, best_d = None, 1e18
                for name, (olon, olat) in grid.obstacle_lonlat.items():
                    d = haversine_km(lon, lat, olon, olat)
                    if d < best_d:
                        best, best_d = name, d
                hit.add(best)
    return hit


def blocked_cells_on_path(costs, waypoints):
    """Cells with infinite cost that the polyline passes through
    (should be 0 for a valid path). `costs` is the planner's cost
    array, so this also catches the replan demo's rock field."""
    n = 0
    rows = len(costs)
    cols = len(costs[0])
    for a, b in zip(waypoints, waypoints[1:]):
        ca = (min(max(int(math.floor(a[0])), 0), rows - 1),
              min(max(int(math.floor(a[1])), 0), cols - 1))
        cb = (min(max(int(math.floor(b[0])), 0), rows - 1),
              min(max(int(math.floor(b[1])), 0), cols - 1))
        for r, c in bresenham(ca[0], ca[1], cb[0], cb[1]):
            if costs[r][c] == INF:
                n += 1
    return n


def _point_at_fraction(waypoints, fraction):
    """Point (corner coords) at `fraction` of the way along a
    polyline, by distance."""
    total = sum(math.hypot(b[0] - a[0], b[1] - a[1])
                for a, b in zip(waypoints, waypoints[1:]))
    target = total * fraction
    walked = 0.0
    for a, b in zip(waypoints, waypoints[1:]):
        seg = math.hypot(b[0] - a[0], b[1] - a[1])
        if walked + seg >= target:
            t = 0.0 if seg == 0 else (target - walked) / seg
            return (a[0] + t * (b[0] - a[0]),
                    a[1] + t * (b[1] - a[1]))
        walked += seg
    return waypoints[-1]


def main():
    grid = Grid(os.path.join(HERE, "elevation_grid.json"))
    costs = build_costs(grid)
    learned_applied = load_learned_obstacles(grid, costs)
    print("Grid: {}x{} cells, {} m per cell".format(
        grid.rows, grid.cols, round(grid.cell_m)))
    print("Field D* nodes: {}x{} corners".format(grid.rows + 1,
                                                  grid.cols + 1))
    blocked_n = sum(1 for row in costs for v in row if v == INF)
    print("Blocked cells (slope > {} deg): {} ({:.1f}%)".format(
        SLOPE_BLOCKED_DEG, blocked_n,
        100 * blocked_n / (grid.rows * grid.cols)))

    legs_out = []
    full_waypoints = []
    total_km = 0.0
    total_climb = 0
    leg2 = None  # (planner, start node/point, goal node/point, wps, km, climb)
    for name, lonlat_a, lonlat_b in LEGS:
        ra, ca = grid.cell_of(*lonlat_a)
        rb, cb = grid.cell_of(*lonlat_b)
        # The search runs between corner nodes: the top-left corner of
        # each stop's cell (at most half a cell from the true stop).
        start_node = (ra, ca)
        goal_node = (rb, cb)
        start_point = (ra + 0.5, ca + 0.5)  # start cell centre
        goal_point = (rb + 0.5, cb + 0.5)   # goal cell centre
        planner = FieldDStar(costs)
        if not planner.plan(start_node, goal_node):
            print("No route for leg '{}': goal unreachable.".format(name))
            return 1
        raw = planner.extract_path(start_point, goal_node)
        if raw is None:
            print("Path extraction failed for leg '{}'.".format(name))
            return 1
        # Extraction ends at the goal corner; finish at the goal cell
        # centre (the true stop position).
        if math.hypot(raw[-1][0] - goal_point[0],
                      raw[-1][1] - goal_point[1]) > 1e-9:
            raw.append(goal_point)
        wps = _downsample(raw)
        km, climb = leg_stats(grid, wps)
        total_km += km
        total_climb += climb
        print("Leg '{}': {:.1f} km, +{} m climb, {} waypoints "
              "({} raw, {} states expanded)".format(
                  name, km, climb, len(wps), len(raw), planner.pops))
        legs_out.append({
            "name": name,
            "from": {"lon_e": lonlat_a[0], "lat": lonlat_a[1]},
            "to": {"lon_e": lonlat_b[0], "lat": lonlat_b[1]},
            "distance_km": round(km, 1),
            "climb_m": climb,
            "waypoint_count": len(wps),
            "raw_waypoint_count": len(raw),
            "states_expanded": planner.pops,
            "waypoints": [
                {"lon_e": round(corner_lonlat(grid, r, c)[0], 4),
                 "lat": round(corner_lonlat(grid, r, c)[1], 4),
                 "elevation_m": grid.grid[
                     _clamp_cell(grid, int(math.floor(r)),
                                 int(math.floor(c)))[0]][
                     _clamp_cell(grid, int(math.floor(r)),
                                 int(math.floor(c)))[1]]}
                for r, c in wps
            ],
        })
        if full_waypoints:
            full_waypoints.extend(wps[1:])
        else:
            full_waypoints.extend(wps)
        if name.startswith("Boxwork"):
            leg2 = (planner, start_node, start_point, goal_node,
                    goal_point, wps, km, climb)

    crossed = zones_crossed(grid, full_waypoints)
    danger_points = {0: 2, 1: 1}.get(len(crossed), 0)
    length_name, length_points = length_class(total_km)
    climb_points = climb_points_from_meters(total_climb)
    total_points = danger_points + length_points + climb_points
    print("Field D* route: {:.1f} km, +{} m climb".format(total_km,
                                                           total_climb))
    print("Scorer: danger {} ({} zone(s) crossed), length {} ({}), "
          "climb {} ({} m) -> {} points".format(
              danger_points, len(crossed), length_points, length_name,
              climb_points, total_climb, total_points))
    print("Routes A/B/C score 4/2/4. A* scores 5 (but skips the "
          "science stops). Field D* scores {}.".format(total_points))

    # --- Replan demo: a fresh rock field appears on leg 2 ------------
    planner, start_node, start_point, goal_node, goal_point, \
        leg2_wps, leg2_km, leg2_climb = leg2
    centre = _point_at_fraction(leg2_wps, ROCK_FIELD_AT_FRACTION)
    centre_lon, centre_lat = corner_lonlat(grid, centre[0], centre[1])
    cr, cc = int(math.floor(centre[0])), int(math.floor(centre[1]))
    radius_cells = ROCK_FIELD_RADIUS_M / grid.cell_m
    rr = int(math.ceil(radius_cells))
    changed = []
    for dr in range(-rr, rr + 1):
        for dc in range(-rr, rr + 1):
            r, c = cr + dr, cc + dc
            if not (0 <= r < grid.rows and 0 <= c < grid.cols):
                continue
            if math.hypot(dr, dc) * grid.cell_m > ROCK_FIELD_RADIUS_M:
                continue
            if planner.costs[r][c] != INF:
                planner.costs[r][c] = INF
                changed.append((r, c))
    print("Rock field on leg 2: {} cells blocked around {:.4f}E, "
          "{:.4f}S ({}% along the leg)".format(
              len(changed), centre_lon, -centre_lat,
              int(ROCK_FIELD_AT_FRACTION * 100)))
    pops0, updates0 = planner.pops, planner.updates
    if not planner.replan_after_cost_change(changed):
        print("Replan failed: goal unreachable after the rock field.")
        return 1
    inc_pops = planner.pops - pops0
    inc_updates = planner.updates - updates0
    new_raw = planner.extract_path(start_point, goal_node)
    if new_raw is None:
        print("Replanned path extraction failed.")
        return 1
    if math.hypot(new_raw[-1][0] - goal_point[0],
                  new_raw[-1][1] - goal_point[1]) > 1e-9:
        new_raw.append(goal_point)
    new_wps = _downsample(new_raw)
    new_km, new_climb = leg_stats(grid, new_wps)
    blocked_hit = blocked_cells_on_path(planner.costs, new_wps)
    print("Replanned leg 2: {:.1f} km (was {:.1f}), +{} m climb "
          "(was +{}), {} blocked cells on the new path".format(
              new_km, leg2_km, new_climb, leg2_climb, blocked_hit))
    print("Incremental replan: {} states expanded, {} updates".format(
        inc_pops, inc_updates))

    # Full replan from scratch, for comparison (same rock field).
    fresh_costs = [row[:] for row in costs]
    for r, c in changed:
        fresh_costs[r][c] = INF
    fresh = FieldDStar(fresh_costs)
    if not fresh.plan(start_node, goal_node):
        print("Full replan failed: goal unreachable after the rock field.")
        return 1
    print("Full replan from scratch: {} states expanded, {} updates "
          "(incremental replan touched {:.0f}% as many)".format(
              fresh.pops, fresh.updates,
              100.0 * inc_pops / fresh.pops if fresh.pops else 0.0))
    # The incremental repair must reach the same optimal cost as the
    # fresh plan; check it.
    g_inc = planner.g[start_node[0]][start_node[1]]
    g_full = fresh.g[start_node[0]][start_node[1]]
    print("Cost-to-go at the leg start: incremental {:.2f}, fresh "
          "{:.2f}".format(g_inc, g_full))

    # --- Learn: remember the new rock field for the next walk --------
    learned_new = learn_obstacle(centre_lon, centre_lat, len(changed))

    out = {
        "project": "Marswalk Coach — team spacebuzz",
        "planner": "Field D* (Ferguson & Stentz 2006) on a MOLA cost "
                   "grid, three legs through the science stops",
        "learned_obstacles_applied": learned_applied,
        "learned_this_run": LEARNED_OBSTACLE_ID if learned_new else None,
        "legs": legs_out,
        "total_distance_km": round(total_km, 1),
        "total_climb_m": total_climb,
        "danger_zones_crossed": sorted(crossed),
        "score": {
            "danger_points": danger_points,
            "length_class": length_name,
            "length_points": length_points,
            "climb_points": climb_points,
            "total_points": total_points,
            "comparison": {
                "Route A (hand-drawn, via the stops)": 4,
                "Route B (straight line)": 2,
                "Route C (wide detour)": 4,
                "A* (straight line, skips the stops)": 5,
            },
        },
        "replan_demo": {
            "leg": "Boxwork to High ground",
            "new_danger": "rock field, 1 km radius, centred at "
                          "{:.4f}E, {:.4f}S ({}% along the leg)".format(
                              centre_lon, abs(centre_lat),
                              int(ROCK_FIELD_AT_FRACTION * 100)),
            "cells_blocked": len(changed),
            "original_leg_distance_km": round(leg2_km, 1),
            "original_leg_climb_m": leg2_climb,
            "replanned_leg_distance_km": round(new_km, 1),
            "replanned_leg_climb_m": new_climb,
            "blocked_cells_on_replanned_path": blocked_hit,
            "incremental_replan_states_expanded": inc_pops,
            "incremental_replan_updates": inc_updates,
            "full_replan_states_expanded": fresh.pops,
            "full_replan_updates": fresh.updates,
            "incremental_cost_to_go": round(g_inc, 2),
            "fresh_cost_to_go": round(g_full, 2),
        },
        "note": "Waypoints are Field D* path points (corner coordinates "
                "converted to lon/lat). Danger-spot positions are "
                "approximate (see astar_route.py). Slope limits: over 25 "
                "deg blocked, 12-25 deg costs extra (JPL rover limits). "
                "Field D* uses one cost per cell, so the A* climb "
                "penalty is not in its cost model. Start/goal are "
                "snapped to the top-left corner of the stop's cell "
                "for the search (at most half a cell); the path "
                "still starts and ends at the true stop positions. Leg distances "
                "are rounded individually; the total is rounded from the "
                "unrounded sum.",
    }
    with open(os.path.join(HERE, "field_dstar_route.json"), "w") as f:
        json.dump(out, f, indent=1)
    print("Wrote field_dstar_route.json")
    return 0


# ----------------------------------------------------------------------
# Self-tests: python3 field_dstar.py --test
# ----------------------------------------------------------------------

def _path_length(path):
    return sum(math.hypot(b[0] - a[0], b[1] - a[1])
               for a, b in zip(path, path[1:]))


def _cells_on_segments(path, rows, cols):
    """Cells a continuous polyline passes through (for test grids)."""
    cells = []
    for a, b in zip(path, path[1:]):
        ca = (min(max(int(math.floor(a[0])), 0), rows - 1),
              min(max(int(math.floor(a[1])), 0), cols - 1))
        cb = (min(max(int(math.floor(b[0])), 0), rows - 1),
              min(max(int(math.floor(b[1])), 0), cols - 1))
        cells.extend(bresenham(ca[0], ca[1], cb[0], cb[1]))
    return cells


def test_compute_cost():
    """Closed form (paper Figure 7) vs brute-force minimisation of the
    paper's Section 3 objective over (x, y) in [0, 1]^2."""
    import random
    random.seed(7)
    worst_gap = 0.0
    trials = 200
    n = 80
    for _ in range(trials):
        g1 = random.uniform(0, 20)
        g2 = random.uniform(0, 20)
        c = random.uniform(0.5, 5)
        b = random.uniform(0.5, 5)
        if random.random() < 0.1:
            c = INF
        if random.random() < 0.1:
            b = INF
        got = compute_cost(g1, g2, c, b)
        best = INF
        for ix in range(n + 1):
            x = ix / n
            for iy in range(n + 1):
                y = iy / n
                if c == INF:
                    cut = 0.0 if (x == 1.0 and y == 0.0) else INF
                else:
                    cut = c * math.sqrt((1 - x) ** 2 + y * y)
                bx = 0.0 if x == 0.0 else (INF if b == INF else b * x)
                val = bx + cut + y * g2 + (1 - y) * g1
                if val < best:
                    best = val
        if got == INF and best == INF:
            continue
        assert got != INF and best != INF, \
            "compute_cost/brute-force disagree on reachability"
        # The closed form is the exact minimum, so it can never be
        # worse than the grid search.
        assert got <= best + 1e-9, \
            "closed form worse than brute force: {} vs {}".format(got, best)
        gap = best - got
        assert gap <= 0.02 * best + 0.05, \
            "closed form suspiciously better than brute force: {} vs {}".format(got, best)
        worst_gap = max(worst_gap, gap)
    print("compute_cost: OK ({} random cases, worst grid gap {:.4f})".format(
        trials, worst_gap))


def test_open_grid():
    """Open grid: the path should be near-straight (within 2% of the
    straight-line distance) and smooth (few waypoints after
    downsampling)."""
    n = 20
    costs = [[1.0] * n for _ in range(n)]
    fd = FieldDStar(costs)
    assert fd.plan((0, 0), (n, n)), "open grid: no path found"
    path = fd.extract_path((0.5, 0.5), (n, n))
    assert path is not None, "open grid: extraction failed"
    length = _path_length(path)
    straight = math.hypot(n - 0.5, n - 0.5)
    assert length < straight * 1.02, \
        "open grid: path {:.2f} vs straight {:.2f}".format(length, straight)
    thin = _downsample(path)
    assert len(thin) <= 6, \
        "open grid: downsampled path has {} waypoints".format(len(thin))
    print("open grid: OK (path {:.2f} vs straight {:.2f}, {} raw / {} "
          "downsampled waypoints)".format(length, straight, len(path),
                                          len(thin)))


def test_wall():
    """A wall with one gap: the path must detour through the gap and
    never cross a blocked cell."""
    n = 20
    costs = [[1.0] * n for _ in range(n)]
    for c in range(n):
        if c not in (14, 15):
            costs[10][c] = INF
    fd = FieldDStar(costs)
    assert fd.plan((0, 0), (n, n)), "wall grid: no path found"
    path = fd.extract_path((0.5, 0.5), (n, n))
    assert path is not None, "wall grid: extraction failed"
    for r, c in _cells_on_segments(path, n, n):
        assert costs[r][c] != INF, \
            "wall grid: path crosses blocked cell ({}, {})".format(r, c)
    in_wall_row = [p for p in path if 10.0 <= p[0] <= 11.0]
    assert in_wall_row, "wall grid: no waypoint in the wall row"
    assert all(13.9 <= p[1] <= 16.1 for p in in_wall_row), \
        "wall grid: path crosses the wall outside the gap: {}".format(
            in_wall_row)
    length = _path_length(path)
    straight = math.hypot(n - 0.5, n - 0.5)
    assert length > straight * 1.02, \
        "wall grid: path suspiciously short: {:.2f}".format(length)
    print("wall grid: OK (path {:.2f} vs straight {:.2f}, crosses the "
          "wall through the gap)".format(length, straight))


def test_replan():
    """Plan on an open grid, add a wall across the path, replan: the
    new path must avoid the wall, the incremental repair must reach
    the same optimal cost as a fresh plan, and it must expand fewer
    states than the fresh plan."""
    n = 30
    base = [[1.0] * n for _ in range(n)]
    fd = FieldDStar(base)
    assert fd.plan((0, 0), (n, n)), "replan: initial plan failed"
    path1 = fd.extract_path((0.5, 0.5), (n, n))
    assert path1 is not None, "replan: initial extraction failed"
    changed = [(15, c) for c in range(10, 20)]
    for r, c in changed:
        fd.costs[r][c] = INF
    pops0, updates0 = fd.pops, fd.updates
    assert fd.replan_after_cost_change(changed), "replan: repair failed"
    inc_pops = fd.pops - pops0
    inc_updates = fd.updates - updates0
    path2 = fd.extract_path((0.5, 0.5), (n, n))
    assert path2 is not None, "replan: replanned extraction failed"
    for r, c in _cells_on_segments(path2, n, n):
        assert fd.costs[r][c] != INF, \
            "replan: new path crosses blocked cell ({}, {})".format(r, c)
    assert _path_length(path2) > _path_length(path1) * 1.01, \
        "replan: new path did not change"
    walled = [row[:] for row in base]
    for r, c in changed:
        walled[r][c] = INF
    fresh = FieldDStar(walled)
    assert fresh.plan((0, 0), (n, n)), "replan: fresh plan failed"
    g_inc = fd.g[0][0]
    g_full = fresh.g[0][0]
    assert abs(g_inc - g_full) < 1e-6, \
        "replan: incremental cost {:.4f} != fresh cost {:.4f}".format(
            g_inc, g_full)
    assert inc_pops < fresh.pops, \
        "replan: incremental ({} pops) not cheaper than fresh ({} pops)".format(
            inc_pops, fresh.pops)
    print("replan: OK (incremental {} pops / {} updates vs fresh {} "
          "pops / {} updates; same optimal cost {:.2f})".format(
              inc_pops, inc_updates, fresh.pops, fresh.updates, g_inc))


def test_unreachable():
    """A wall with no gap: the planner must report no path."""
    n = 10
    costs = [[1.0] * n for _ in range(n)]
    for c in range(n):
        costs[5][c] = INF
    fd = FieldDStar(costs)
    assert not fd.plan((0, 0), (n, n)), \
        "unreachable: planner claimed a path through a full wall"
    print("unreachable: OK (no path reported)")


def run_tests():
    test_compute_cost()
    test_open_grid()
    test_wall()
    test_replan()
    test_unreachable()
    print("All Field D* tests passed.")


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "--test":
        run_tests()
    else:
        raise SystemExit(main())
