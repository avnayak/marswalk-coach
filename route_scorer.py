"""Marswalk Coach — small route-scoring demo (team spacebuzz).

A tiny rule-based route planner. It reads the obstacle log
(obstacles.json), scores three candidate routes with plain rules,
and prints the safest one.

Rules (most points wins):
  - Danger spots crossed: 0 = 2, 1 = 1, 2 or more = 0
  - Length: short = 2, medium = 1, long = 0
  - Climb: Route A uses the measured MOLA total climb (see
    climb_points_from_meters below). Routes B and C use stated
    climb classes (flat = 2, some climb = 1, steep = 0) because
    no elevation profile was sampled for those lines.
  - Tie-break: the shorter route wins (less time outside).

Honest limits: this is a demo, not self-driving code. Route A's
climb comes from real MOLA elevation numbers (elevation.json).
The danger crossings, lengths, and routes B and C climb classes
were read by eye from the Mars Trek map or stated as assumptions.
It does not read rover photos by itself. The steep slope (OBS-03)
is the climb itself: it is scored by the climb rule, not counted
as a danger crossed.
"""

import json
import os

HERE = os.path.dirname(os.path.abspath(__file__))

DANGER_POINTS = {0: 2, 1: 1}  # 2 or more crossings = 0
LENGTH_POINTS = {"short": 2, "medium": 1, "long": 0}
CLIMB_POINTS = {"flat": 2, "some": 1, "steep": 0}
LENGTH_RANK = {"short": 1, "medium": 2, "long": 3}  # for the tie-break
CLIMB_WORDS = {"flat": "Gentle", "some": "Some climb", "steep": "Steep",
               "measured": "Some climb"}


def climb_points_from_meters(total_climb_m):
    """Map a measured total climb (meters) to climb points.

    Team rubric, stated here in the open:
      under 1000 m of climbing is gentle ground: 2 points
      1000 to 5000 m is a serious climb: 1 point
      over 5000 m is extreme: 0 points
    Route A climbs 4991 m, so it scores 1 point.
    """
    if total_climb_m < 1000:
        return 2
    if total_climb_m <= 5000:
        return 1
    return 0

# Candidate routes. Danger crossings and lengths were read by eye
# from the Mars Trek route map. Route A's climb is measured from
# real MOLA elevations (elevation.json); routes B and C climb
# classes are the team's stated assumptions.
ROUTES = [
    {
        "name": "A — Our route (yellow)",
        "danger_crossed": 0,
        "length": "medium",
        "climb": "measured",
    },
    {
        "name": "B — Straight line",
        "danger_crossed": 2,
        "length": "short",
        "climb": "steep",
    },
    {
        "name": "C — Wide detour south",
        "danger_crossed": 0,
        "length": "long",
        "climb": "flat",
    },
]


def score(route, measured_climb_points):
    """Score one route. Returns (danger, length, climb, total)."""
    danger = DANGER_POINTS.get(route["danger_crossed"], 0)
    length = LENGTH_POINTS[route["length"]]
    if route["climb"] == "measured":
        climb = measured_climb_points
    else:
        climb = CLIMB_POINTS[route["climb"]]
    return danger, length, climb, danger + length + climb


def main():
    with open(os.path.join(HERE, "obstacles.json")) as f:
        log = json.load(f)
    print("Obstacle log: {} obstacles".format(len(log["obstacles"])))
    for obs in log["obstacles"]:
        print("  {} — {} ({}), confidence: {}".format(
            obs["id"], obs["type"], obs["location"], obs["confidence"]))
    with open(os.path.join(HERE, "elevation.json")) as f:
        elev = json.load(f)
    pts = elev["points"]
    measured_climb = pts[-1]["elevation_m"] - pts[0]["elevation_m"]
    measured_points = climb_points_from_meters(measured_climb)
    print("Route A measured climb (MOLA): {} m -> {} m = +{} m "
          "-> {} climb point(s).".format(
              pts[0]["elevation_m"], pts[-1]["elevation_m"],
              measured_climb, measured_points))
    print()
    print("{:<26} {:<7} {:<7} {:<11} {:<5}".format(
        "Route", "Danger", "Length", "Climb", "Total"))
    scored = []
    for route in ROUTES:
        danger, length, climb, total = score(route, measured_points)
        scored.append((total, LENGTH_RANK[route["length"]], route["name"]))
        print("{:<26} {:<7} {:<7} {:<11} {:<5}".format(
            route["name"], danger, length,
            CLIMB_WORDS[route["climb"]], total))
    print()
    # Most points wins; ties go to the shorter route.
    scored.sort(key=lambda s: (-s[0], s[1]))
    best_total = scored[0][0]
    tied = [s for s in scored if s[0] == best_total]
    winner = scored[0][2]
    if len(tied) > 1:
        print("Winner: {} — tied on {} points, but shortest, "
              "so less time outside.".format(winner, best_total))
    else:
        print("Winner: {} — {} points.".format(winner, best_total))


if __name__ == "__main__":
    main()
