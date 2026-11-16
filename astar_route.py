"""Marswalk Coach — Weighted A* route planner (team spacebuzz).

Finds the best walking route from START to the Mount Sharp summit on its
own, from real MOLA elevation data — instead of ranking three hand-drawn
routes like route_scorer.py does.

How it works:
  1. Reads the MOLA elevation window from elevation_grid.json (real NASA
     heights, 463 m per cell, fetched once from the MOLA MEGDR tile and
     saved so this runs offline).
  2. Builds a cost grid. Each cell costs more when the ground is steep
     (JPL's published slope limits for Mars driving: over 25 degrees is
     blocked, 12-25 degrees costs extra) and when the step climbs uphill.
     The two known danger spots (crater, rough ground) cost extra too.
  3. Runs Weighted A* (weight 1.5) from START to the summit, then smooths
     the path by skipping waypoints wherever the straight line between
     them crosses no blocked cell and no danger zone.
  4. Scores the found route with route_scorer.py's own rules and prints
     how it compares to the hand-drawn routes A, B, and C.

Run it with: python3 astar_route.py
It writes astar_route.json (waypoints, distance, climb, score).

Honest limits: the grid is 463 m per cell, so small rocks and narrow
cracks are invisible. The danger-spot positions are APPROXIMATE — they
come from the red X marks on the drawn route map, converted with a
linear pixel-to-lon/lat fit from the three stop dots, and the dots were
drawn at even visual spacing, not at exact geographic positions, so the
mapped spots may be off by several kilometres. The 25/12 degree slope
limits are JPL's rover-driving limits, used here as a stand-in for a
walker's limits. This is a demo planner, not flight software.
"""

import heapq
import json
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))

# Same climb rule as route_scorer.py (imported so there is one source
# of truth; the fallback is the same rule, in case the import fails).
try:
    from route_scorer import climb_points_from_meters
except ImportError:
    def climb_points_from_meters(total_climb_m):
        if total_climb_m < 1000:
            return 2
        if total_climb_m <= 5000:
            return 1
        return 0

MARS_RADIUS_KM = 3389.5
WEIGHT = 1.5            # weighted A* weight (1.0 = plain A*)
SLOPE_BLOCKED_DEG = 25.0  # JPL limit, bedrock: steeper is blocked
SLOPE_COST_FROM_DEG = 12.0  # JPL limit, sand: steeper costs extra
CLIMB_WEIGHT = 0.01     # extra cost per metre climbed (100 m = 1 cell)
OBSTACLE_EXTRA = 10.0   # extra cost per cell inside a danger zone
OBSTACLE_RADIUS_M = 1000.0  # danger zone radius around each mapped spot

START = (137.4417, -4.5895)   # lon E, lat S
GOAL = (137.793, -4.981)      # Mount Sharp summit area
STOPS = {
    "Stop 1 (boxwork)": (137.5308, -4.7197),
    "Stop 2 (high ground)": (137.75, -5.0),
}

# Danger-spot marks on gale-route-map.png (1632x878), found by measuring
# the red X pixels. Converted to lon/lat with a linear fit from the three
# stop dots below. APPROXIMATE — see the module docstring.
X_PIXELS = {
    "OBS-01 crater": (1069, 543),
    "OBS-02 rough ground": (1190, 435),
}
# Pixel (x, y) on the route map -> (lon E, lat) of the three stop dots.
DOT_FIT = [
    ((975, 330), (137.4417, -4.5895)),   # START
    ((1055, 384), (137.75, -5.0)),       # Stop 2
    ((1097, 416), (137.793, -4.981)),    # Mount Sharp summit
]

# Length classes, in km. Team judgement, consistent with the scorer:
# Route B (straight line, 31.0 km) is "short", Route A (33.1 km) is
# "medium", Route C (wide detour) is "long".
LENGTH_BANDS = [(32.0, "short", 2), (36.0, "medium", 1)]
LENGTH_DEFAULT = ("long", 0)


def fit_line(xs, ys):
    """Least-squares fit y = a*x + b. Returns (a, b)."""
    n = len(xs)
    mx = sum(xs) / n
    my = sum(ys) / n
    num = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    den = sum((x - mx) ** 2 for x in xs)
    a = num / den
    return a, my - a * mx


def haversine_km(lon1, lat1, lon2, lat2):
    """Great-circle distance on Mars, in km."""
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlam = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dlam / 2) ** 2
    return 2 * MARS_RADIUS_KM * math.asin(math.sqrt(a))


def bresenham(r0, c0, r1, c1):
    """Cells on the straight line from (r0, c0) to (r1, c1), inclusive."""
    cells = []
    dr, dc = abs(r1 - r0), abs(c1 - c0)
    sr = 1 if r0 < r1 else -1
    sc = 1 if c0 < c1 else -1
    err = dr - dc
    r, c = r0, c0
    while True:
        cells.append((r, c))
        if r == r1 and c == c1:
            break
        e2 = 2 * err
        if e2 > -dc:
            err -= dc
            r += sr
        if e2 < dr:
            err += dr
            c += sc
    return cells


class Grid:
    """The elevation window plus the cost layers built from it."""

    def __init__(self, path):
        with open(path) as f:
            d = json.load(f)
        self.grid = d["grid"]
        self.rows = d["rows"]
        self.cols = d["cols"]
        self.cell_m = d["meters_per_pixel"]
        self.ppd = d["pixels_per_degree"]
        self.west = d["west_lon_e"]
        self.north = d["north_lat"]
        self.slope = [[0.0] * self.cols for _ in range(self.rows)]
        self.blocked = [[False] * self.cols for _ in range(self.rows)]
        self.obstacle = [[False] * self.cols for _ in range(self.rows)]
        self._build_slopes()
        self.obstacle_lonlat = self._mark_obstacles()

    def cell_of(self, lon, lat):
        """lon/lat -> (row, col) of the containing cell."""
        col = int((lon - self.west) * self.ppd)
        row = int((self.north - lat) * self.ppd)
        return row, col

    def lonlat_of(self, row, col):
        """(row, col) -> (lon, lat) of the cell centre."""
        lon = self.west + (col + 0.5) / self.ppd
        lat = self.north - (row + 0.5) / self.ppd
        return lon, lat

    def _build_slopes(self):
        # Slope at each cell: steepest gradient to any of the 8 neighbours.
        for r in range(self.rows):
            for c in range(self.cols):
                worst = 0.0
                for dr in (-1, 0, 1):
                    for dc in (-1, 0, 1):
                        if dr == 0 and dc == 0:
                            continue
                        rr, cc = r + dr, c + dc
                        if not (0 <= rr < self.rows and 0 <= cc < self.cols):
                            continue
                        dist = self.cell_m * math.hypot(dr, dc)
                        grad = abs(self.grid[rr][cc] - self.grid[r][c]) / dist
                        worst = max(worst, grad)
                deg = math.degrees(math.atan(worst))
                self.slope[r][c] = deg
                if deg > SLOPE_BLOCKED_DEG:
                    self.blocked[r][c] = True

    def _mark_obstacles(self):
        # Linear pixel -> lon/lat fits from the three stop dots.
        px = [p[0][0] for p in DOT_FIT]
        py = [p[0][1] for p in DOT_FIT]
        lon_a, lon_b = fit_line(px, [p[1][0] for p in DOT_FIT])
        lat_a, lat_b = fit_line(py, [p[1][1] for p in DOT_FIT])
        mapped = {}
        radius_cells = OBSTACLE_RADIUS_M / self.cell_m
        for name, (x, y) in X_PIXELS.items():
            lon = lon_a * x + lon_b
            lat = lat_a * y + lat_b
            mapped[name] = (lon, lat)
            r0, c0 = self.cell_of(lon, lat)
            rr = int(math.ceil(radius_cells))
            for dr in range(-rr, rr + 1):
                for dc in range(-rr, rr + 1):
                    r, c = r0 + dr, c0 + dc
                    if not (0 <= r < self.rows and 0 <= c < self.cols):
                        continue
                    if math.hypot(dr, dc) * self.cell_m <= OBSTACLE_RADIUS_M:
                        self.obstacle[r][c] = True
        return mapped

    def slope_cost(self, r, c):
        """Extra cost for steep ground: 0 below 12 deg, ramping to 3 at 25."""
        s = self.slope[r][c]
        if s <= SLOPE_COST_FROM_DEG:
            return 0.0
        return (s - SLOPE_COST_FROM_DEG) / (SLOPE_BLOCKED_DEG - SLOPE_COST_FROM_DEG) * 3.0

    def move_cost(self, r0, c0, r1, c1):
        """Cost of stepping from cell (r0, c0) to neighbour (r1, c1)."""
        dist = math.hypot(r1 - r0, c1 - c0)  # 1.0 or sqrt(2), in cells
        cost = dist * (1.0 + self.slope_cost(r1, c1))
        gain = self.grid[r1][c1] - self.grid[r0][c0]
        if gain > 0:
            cost += gain * CLIMB_WEIGHT
        if self.obstacle[r1][c1]:
            cost += OBSTACLE_EXTRA
        return cost


def astar(grid, start, goal, weight=WEIGHT):
    """Weighted A* on the grid. Returns the cell path [(r, c), ...]."""
    def heuristic(r, c):
        return math.hypot(r - goal[0], c - goal[1])

    open_heap = [(weight * heuristic(*start), 0.0, start)]
    came_from = {start: None}
    cost_so_far = {start: 0.0}
    while open_heap:
        _, g, current = heapq.heappop(open_heap)
        if current == goal:
            break
        if g > cost_so_far[current]:
            continue
        r0, c0 = current
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                if dr == 0 and dc == 0:
                    continue
                r1, c1 = r0 + dr, c0 + dc
                if not (0 <= r1 < grid.rows and 0 <= c1 < grid.cols):
                    continue
                if grid.blocked[r1][c1]:
                    continue
                # No corner-cutting through two blocked diagonals.
                if dr and dc and (grid.blocked[r0 + dr][c0] and grid.blocked[r0][c0 + dc]):
                    continue
                new_g = g + grid.move_cost(r0, c0, r1, c1)
                nxt = (r1, c1)
                if nxt not in cost_so_far or new_g < cost_so_far[nxt]:
                    cost_so_far[nxt] = new_g
                    heapq.heappush(open_heap, (new_g + weight * heuristic(r1, c1), new_g, nxt))
                    came_from[nxt] = current
    if goal not in came_from:
        return None
    path = []
    node = goal
    while node is not None:
        path.append(node)
        node = came_from[node]
    path.reverse()
    return path


def smooth(grid, path):
    """Greedy line-of-sight shortcutting: from each waypoint, jump to the
    farthest later waypoint whose straight segment crosses no blocked
    cell and no danger zone."""
    if len(path) <= 2:
        return path
    out = [path[0]]
    i = 0
    while i < len(path) - 1:
        jump = i + 1
        for j in range(len(path) - 1, i + 1, -1):
            cells = bresenham(path[i][0], path[i][1], path[j][0], path[j][1])
            if all(not grid.blocked[r][c] and not grid.obstacle[r][c] for r, c in cells):
                jump = j
                break
        out.append(path[jump])
        i = jump
    return out


def path_stats(grid, waypoints):
    """Distance (km) and total climb (m) along the smoothed waypoints.
    Elevation is sampled at every grid cell each segment passes through."""
    total_km = 0.0
    total_climb = 0
    prev_elev = None
    for a, b in zip(waypoints, waypoints[1:]):
        lon1, lat1 = grid.lonlat_of(*a)
        lon2, lat2 = grid.lonlat_of(*b)
        total_km += haversine_km(lon1, lat1, lon2, lat2)
        for r, c in bresenham(a[0], a[1], b[0], b[1]):
            elev = grid.grid[r][c]
            if prev_elev is not None and elev > prev_elev:
                total_climb += elev - prev_elev
            prev_elev = elev
    return total_km, total_climb


def zones_crossed(grid, waypoints):
    """How many danger zones (1 km radius) the smoothed path enters."""
    hit = set()
    for a, b in zip(waypoints, waypoints[1:]):
        for r, c in bresenham(a[0], a[1], b[0], b[1]):
            if grid.obstacle[r][c]:
                # Which zone? Nearest mapped obstacle centre.
                lon, lat = grid.lonlat_of(r, c)
                best, best_d = None, 1e18
                for name, (olon, olat) in grid.obstacle_lonlat.items():
                    d = haversine_km(lon, lat, olon, olat)
                    if d < best_d:
                        best, best_d = name, d
                hit.add(best)
    return hit


def length_class(total_km):
    """Length class + points, same bands as route_scorer.py's routes."""
    for limit, name, points in LENGTH_BANDS:
        if total_km < limit:
            return name, points
    return LENGTH_DEFAULT


def main():
    grid = Grid(os.path.join(HERE, "elevation_grid.json"))
    start = grid.cell_of(*START)
    goal = grid.cell_of(*GOAL)
    print("Grid: {}x{} cells, {} m per cell".format(grid.rows, grid.cols, round(grid.cell_m)))
    print("START cell {} ({} m), summit cell {} ({} m)".format(
        start, grid.grid[start[0]][start[1]],
        goal, grid.grid[goal[0]][goal[1]]))
    blocked_n = sum(sum(row) for row in grid.blocked)
    print("Blocked cells (slope > {} deg): {} ({:.1f}%)".format(
        SLOPE_BLOCKED_DEG, blocked_n, 100 * blocked_n / (grid.rows * grid.cols)))
    for name, (lon, lat) in grid.obstacle_lonlat.items():
        r, c = grid.cell_of(lon, lat)
        inside = 0 <= r < grid.rows and 0 <= c < grid.cols
        print("Danger zone {}: mapped to {:.4f}E, {:.4f}S (APPROXIMATE) "
              "-> cell {} {}".format(name, lon, -lat, (r, c),
                                     "inside grid" if inside else "OUTSIDE GRID"))

    raw = astar(grid, start, goal)
    if raw is None:
        print("No route found: the summit is unreachable with these slope limits.")
        return 1
    waypoints = smooth(grid, raw)
    print("A* path: {} cells, smoothed to {} waypoints".format(len(raw), len(waypoints)))

    total_km, total_climb = path_stats(grid, waypoints)
    crossed = zones_crossed(grid, waypoints)
    danger_points = {0: 2, 1: 1}.get(len(crossed), 0)
    length_name, length_points = length_class(total_km)
    climb_points = climb_points_from_meters(total_climb)
    total_points = danger_points + length_points + climb_points
    print("A* route: {:.1f} km, +{} m climb".format(total_km, total_climb))
    print("Scorer: danger {} ({} zone(s) crossed), length {} ({}), "
          "climb {} ({} m) -> {} points".format(
              danger_points, len(crossed), length_points, length_name,
              climb_points, total_climb, total_points))
    print("Routes A/B/C score 4/2/4. A* scores {}.".format(total_points))

    # How close does the A* route come to the hand-drawn route's stops?
    # Measure against the RAW path (every cell the planner walked), not
    # the smoothed waypoints: smoothing collapsed the path to just 2
    # endpoints, so measuring only those understates how close the
    # walked path really comes.
    wp_lonlat = [grid.lonlat_of(r, c) for r, c in waypoints]
    raw_lonlat = [grid.lonlat_of(r, c) for r, c in raw]
    closest_approach = {}
    for name, (slon, slat) in STOPS.items():
        best = min(haversine_km(lon, lat, slon, slat) for lon, lat in raw_lonlat)
        closest_approach[name] = round(best, 2)
        print("Closest approach to {}: {:.2f} km".format(name, best))

    out = {
        "project": "Marswalk Coach — team spacebuzz",
        "planner": "Weighted A* (weight {}) on a MOLA cost grid, line-of-sight smoothed".format(WEIGHT),
        "start": {"lon_e": START[0], "lat": START[1]},
        "goal": {"lon_e": GOAL[0], "lat": GOAL[1]},
        "total_distance_km": round(total_km, 1),
        "total_climb_m": total_climb,
        "waypoint_count": len(waypoints),
        "raw_cell_count": len(raw),
        "danger_zones_crossed": sorted(crossed),
        "closest_approach_km": closest_approach,
        "score": {
            "danger_points": danger_points,
            "length_class": length_name,
            "length_points": length_points,
            "climb_points": climb_points,
            "total_points": total_points,
            "hand_drawn_routes": {"A": 4, "B": 2, "C": 4},
        },
        "obstacle_positions_approximate": {
            name: {"lon_e": round(lon, 4), "lat": round(lat, 4)}
            for name, (lon, lat) in grid.obstacle_lonlat.items()
        },
        "note": "Waypoints are smoothed A* path cells. Danger-spot positions are approximate (see astar_route.py). Slope limits: over 25 deg blocked, 12-25 deg costs extra (JPL rover limits).",
        "waypoints": [
            {"lon_e": round(lon, 4), "lat": round(lat, 4),
             "elevation_m": grid.grid[r][c]}
            for (r, c), (lon, lat) in zip(waypoints, wp_lonlat)
        ],
    }
    with open(os.path.join(HERE, "astar_route.json"), "w") as f:
        json.dump(out, f, indent=1)
    print("Wrote astar_route.json")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
