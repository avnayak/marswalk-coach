# Marswalk Coach

**A calm mind is survival equipment.**

Team spacebuzz · NASA Space Apps Challenge 2026 · San Jose, CA
Challenge: Interplanetary Survival Guide: Martian Map

## What it is

A Mars map that plans a safe multi-day trip to Mount Sharp — and when a dust
storm hits during one walk, a calm AI coach (three scripted lines) helps
the astronaut breathe,
check one thing, decide one thing, and follow the safe route home. Earth is
up to 20 minutes away by radio, so the coach lives on the map, not on Earth.
Next step: the guide in VR glasses, reading the astronaut's cues — so
the coach knows when to speak without being asked.

The demo is 7 slides, plus a small route-scoring code demo (see below).
The map is Gale Crater and Mount Sharp, from NASA Mars Trek. The coach
lives on the map.

## The story

1. The map plans the trip: one safe route from the landing plain to Mount Sharp,
   33.1 km over several days, with science stops (boxwork formations,
   high ground) and danger spots marked. The three measured legs are:
   START to Boxwork 9.3 km, Boxwork to High ground 21.0 km, High ground
   to Mount Sharp 2.8 km. The total climb is 4,991 m — the sum of the
   three legs' height gains (+1,048 / +3,718 / +225 m). Our story follows
   one walk on one day of it.
2. The walk begins, calm.
3. A dust storm rises early in the day's walk. Heart pounding. Hard to think.
4. The coach on the map: breathe (in 2-3-4, out 2-3-4), one check only
   (what does the map show for the storm?), one decision (turn back now,
   while things are steady). Follow the white dashes home.
5. Quiet ending: In the story, the storm passes by nightfall. Rest is part of the mission too.

## How to view it

Demo slides: https://docs.google.com/presentation/d/1tObTyyL6Y6GO90HJBAsx1TiFUGwUSs6prrjBB6G0_8Q/edit?usp=sharing

Interactive map page: https://muse.ai/s/marswalk-coach-map-xvxx6ixpr0kuc7/6ISh8tnLEI6YA2V27dYyA5fMcSWeStl5XputRSvuewM

Coach guide: COACH_GUIDE.md in this repo — what the coach says in a dust
storm, and why each step is real.

Map images in this repo: gale-route-map.png (the route), gale-route-zoom.png
(a zoomed crop of the route, so the stops are easy to read), gale-clean.png,
gale-route-closeup.png, gale-storm-map.png, gale-storm-zoom.png,
gale-true-map-trek.png (the route at true positions, from MOLA heights —
the map on the page), gale-true-storm-map-trek.png (the true map in a
storm). gale-true-map.png and gale-true-storm-map.png are the
true-position route and storm maps used in the deck.

## Code demo

A small route-planning demo, in this repo:

- `obstacles.json` — the obstacle log: 3 real dangers (crater, rough ground,
  steep slope), plus 1 demo-learned one (a fresh rock field found during
  the walk, OBS-04). The first two were checked by eye on the Mars Trek map;
  the slope was measured from real MOLA heights.

  The Mars Trek route pictures are schematic — stops spaced for reading,
  dangers drawn near the route to show the idea. In real Gale Crater
  coordinates, these regional hazards sit 20 to 40 km away from the safe
  corridor; the true-position map (gale-true-map-trek.png) shows them where
  they are, as best as we know.
- `route_scorer.py` — scores 3 candidate routes by plain rules (danger
  spots crossed, length, climb) and picks the safest. Run it with
  `python3 route_scorer.py`. Every push runs it automatically
  (GitHub Actions) and checks that Route A still wins.
- `astar_route.py` — a real path-planner: Weighted A* on the MOLA
  elevation grid (`elevation_grid.json`) finds the safest route from
  START to the summit on its own, avoiding steep slopes and danger
  zones. Run it with `python3 astar_route.py`; it writes
  `astar_route.json` and scores its route against the hand-drawn routes.
- `field_dstar.py` — the routing method NASA flew on its rovers: Field D*
  (Ferguson & Stentz 2006), the interpolation-based planner from Carnegie
  Mellon that flew on the Mars Exploration Rovers. It plans the route as three legs
  through the science stops on the MOLA grid, with smooth any-angle
  paths, and includes a replan demo: a fresh rock field appears on the
  second leg and the planner reroutes around it, touching only 4% as
  many states as planning from scratch. It also remembers what it
  finds: the new rock field is written into `obstacles.json` as
  OBS-04, so the next run plans around it from the start. Run it with
  `python3 field_dstar.py`; it writes `field_dstar_route.json`.
  Self-tests: `python3 field_dstar.py --test`. Note: re-running the
  planner now applies the learned danger (OBS-04) from the start, so
  a fresh run plans the detour: 37.8 km, +5,017 m, 2 points; a fresh
  run also gives different replan numbers (11% of states rechecked,
  not 4%). The committed `field_dstar_route.json` is the first walk's
  route, before the danger was found. The scorer counts net climb
  (4,991 m); the planners count every metre climbed along the way
  (Field D* 5,003 m) — that is why Field D* crosses the 5,000 m line.

It is a demo, not self-driving code. The code reads the measured MOLA
heights from elevation.json and elevation_grid.json. Danger crossings and lengths were read by
eye from the map; the code does not read rover photos by itself. The
steep slope is scored by the climb rule, not counted as a danger
crossed. Routes B and C were scored by eye from the map; our A* program
measured zero danger crossings on the straight line. Route A is the
mission route because it visits the science stops. Measured the same
way, the straight line would score 5 — the scoring alone does not pick
Route A; the science stops do.

## NASA data used

Short list — full details, links, and attribution in DATA_SOURCES.md.

- NASA Mars Trek — the map and the walk route (official challenge resource)
- HiRISE images — danger spots
- CRISM mineral maps — cited as a reference; drove no decision
- MOLA elevation grid — real heights along the route, measured by us
- Curiosity Mastcam — boxwork formations (photographed 2025, west of our route); Stop 1 sits at the boxwork layer's elevation.
- Mars climate data, NASA Ames simulations — the dust storm (model data, not live observations)
- Curiosity RAD — cited as a reference; drove no decision
- Curiosity Hazcam archive — ground photos behind the obstacle log
- Curiosity location map — the rover's real path in Gale Crater

## AI tools used

Muse (Meta AI) helped draw the route and danger marks on the Mars Trek map
images and build the demo slides, working from Akshata Nayak's plan, words,
and decisions. Muse also wrote the three programs in this repo: the
route-scoring program, the A* planner, and the Field D* planner.
The story, the coach's words, and every choice are hers.

## License

Apache License 2.0 — see LICENSE. Third-party data and images keep their own
terms; see DATA_SOURCES.md.

## Team

Akshata Nayak (team owner) and Rahul Munagala — team spacebuzz.
