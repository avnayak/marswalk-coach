# Data sources

Every dataset, image, and map layer used in Marswalk Coach — what it is,
how it is used, and whether it is a real observation or model data.

## 1. NASA Mars Trek — the map (official challenge resource)

- What: interactive Mars mapping tool from NASA JPL. Layers used: CTX mosaic
  imagery and MOLA hillshade elevation.
- How used: the Gale Crater / Mount Sharp base map in the demo. The walk
  route, science stops, and danger marks are drawn on it.
- Real or model: real observations — CTX images from Mars Reconnaissance
  Orbiter; MOLA elevation from Mars Global Surveyor.
- Source: NASA JPL Mars Trek — https://trek.nasa.gov/mars/
- Attribution: NASA / JPL.

## 2. HiRISE images — danger spots

- What: high-resolution camera images from Mars Reconnaissance Orbiter.
- How used: to spot boulders and rough ground. The dangers are regional
  hazards shown on the map; in real coordinates they sit 20 to 40 km
  from the safe corridor.
- Real or model: real observations.
- Source: HiRISE, University of Arizona / NASA JPL — used as layers inside
  Mars Trek.
- Attribution: NASA / JPL / University of Arizona.

## 3. CRISM mineral maps — reference only

- What: mineral maps from the Compact Reconnaissance Imaging Spectrometer
  for Mars (Mars Reconnaissance Orbiter).
- How used: cited as a reference; it drove no decision. The science stops
  came from the challenge's own example (boxwork) and Curiosity Mastcam
  photos, not from CRISM.
- Real or model: real observations.
- Source: CRISM, Johns Hopkins APL / NASA — used as layers inside Mars Trek.
- Attribution: NASA / JPL / JHUAPL.

## 4. Mars climate data — the dust storm (MODEL DATA)

- What: Mars climate simulations from the NASA Ames Mars Climate Modeling
  Center.
- How used: the dust-storm scenario — how storms rise and behave.
- Real or model: MODEL / SIMULATED data — computer simulations, not live
  observations from Mars.
- Source: NASA Ames Mars Climate Modeling Center —
  https://www.nasa.gov/mars-climate-modeling-center-ames
- Attribution: NASA Ames Research Center.

## 5. Curiosity RAD — reference only

- What: radiation measurements from the Radiation Assessment Detector on the
  Curiosity rover, in Gale Crater.
- How used: cited as a reference; it drove no decision. No RAD number
  appears anywhere in the project; the shelter guidance ("Shelter: none
  on this route") comes from the map, not from radiation data.
- Real or model: real observations.
- Source: NASA MSL Curiosity. Rover location map:
  https://science.nasa.gov/mission/msl-curiosity/location-map/
- Attribution: NASA / JPL-Caltech.

## 6. USGS Global Geologic Map of Mars — background context

- What: global geologic map of Mars (SIM 3292).
- How used: background context only — cited as a reference; it drove
  no decision. No rock types from this map appear anywhere in the
  project.
- Real or model: a map drawn from real observations.
- Source: https://usgs.maps.arcgis.com/apps/webappviewer/index.html?id=fc004c3d21b64c398ed5458580ab7c58
- Attribution: USGS Astrogeology.

## 7. USGS "Map Status: Mars" — index of all USGS Mars maps

- What: a whole-Mars index showing every USGS Mars map published or in
  progress (148 maps).
- How used: background check. It lists a Mount Sharp (Aeolis Mons) geologic
  map as "In Progress, 1:60K" — the detailed Mount Sharp geology map is
  still being made, which is why this project leans on Mars Trek plus the
  global geologic map above.
- Real or model: an index of maps drawn from real observations.
- Source: https://usgs.maps.arcgis.com/apps/webappviewer/index.html?id=f693b580b2464e6989f32e4266199552
- Attribution: USGS Astrogeology.

## 8. USGS Mars Global Map Compilation IMAP 1802-A, B, C (1986–1987)

- What: a whole-Mars geologic map from the Viking era, in three sheets
  (western equatorial, eastern equatorial, polar regions).
- How used: global geologic context only — it is not Gale-specific.
- Real or model: a map drawn from Viking-era observations.
- Source: https://experience.arcgis.com/experience/7b73bd96f1b248259ba7029a04a1e823
- Attribution: USGS (Scott and Tanaka, 1986; Greeley and Guest, 1987;
  Tanaka and Scott, 1987).

## 9. Curiosity Hazcam dataset — ground photos of obstacles

- What: Front and Rear Hazard Avoidance Camera images from the Curiosity
  rover in Gale Crater. NASA describes this dataset's use in rover
  traverse planning, post-traverse assessment, and localization.
- How used: the ground-photo source behind the obstacle log on the map
  page. The crater and rough-ground obstacles were checked by eye on the
  Mars Trek map; this archive is where closer ground photos of such
  obstacles live.
- Ground check (sol 1000, May 30, 2015): the two Front Hazcam photos
  from that day (a left/right pair of the same ground) were studied one
  by one. Both show smooth sand in front, then broken layered rock slabs
  and scattered angular rocks rising to a rocky ridge. This confirms
  that the "rough ground, rocks" obstacle type in our log is real ground
  in Gale Crater. The photos show the kind of ground, not our exact
  route spots.
- Real or model: real observations.
- Source: https://pds.nasa.gov/ds-view/pds/viewProfile.jsp?dsid=MSL-M-HAZCAM-2-EDR-V1.0
- Attribution: NASA / JPL-Caltech.

## 10. MGS MOLA MEGDR elevation grid — real heights for the route

- What: the Mars Orbiter Laser Altimeter (MOLA) global elevation grid
  (Mission Experiment Gridded Data Record), tile megt00n090hb at 128
  pixels per degree (463 m per pixel). Elevations are meters above the
  Mars areoid (Mars "sea level").
- How used: real heights for the route scorer. Four approximate route
  points were sampled from the grid: START near Bradbury Landing
  (-4503 m), Stop 1 at the boxwork layer (-3455 m; 137.5308E,
  4.7197S — about 1 km above the crater floor; Curiosity
  photographed boxwork up close in 2025, west of our route),
  Stop 2 on the mound flank (+263 m),
  and the Mount Sharp summit area (+488 m — the highest
  ground near the planned summit point; the point 5.08S, 137.85E
  itself samples +12 m). Total climb: +4,991 m — the sum of the
  three legs' height gains (+1,048 / +3,718 / +225 m). The route
  runs 33.1 km via the stops, in three measured legs: START to
  Boxwork 9.3 km, Boxwork to High ground 21.0 km, High ground to
  Mount Sharp 2.8 km. The numbers live in
  elevation.json, and route_scorer.py scores Route A's climb from
  them (under 1000 m = 2 points, 1000–5000 m = 1 point,
  over 5000 m = 0 points).
- Real or model: real observations — laser altimetry from Mars Global
  Surveyor, 1999–2001.
- Source: https://pds-geosciences.wustl.edu/mgs/urn-nasa-pds-mgs_mola_topography_derived/meg128/megt00n090hb.img
  (label: .../megt00n090hb.xml). PDS4 bundle "MGS MOLA Topography
  Derived", NASA Planetary Data System Geosciences Node.
- Attribution: NASA / Goddard Space Flight Center (MOLA Science Team).

## 11. Curiosity Mastcam — boxwork formations (Stop 1's science goal)

- What: Mastcam photos of boxwork formations — low mineral ridges in the
  foothills of Mount Sharp, photographed up close by Curiosity in 2025
  (sol 4536, May 22, 2025). The ridges formed when groundwater moved
  through cracks in the rock billions of years ago. The boxwork
  layer sits about 1 km above the Gale crater floor.
- How used: Stop 1 sits at the boxwork layer's elevation — the challenge's own
  example science goal.
- Real or model: real observations.
- Source: https://www.jpl.nasa.gov/images/pia26560-curiosity-views-boxwork-patterns-at-a-distance/
- Attribution: NASA / JPL-Caltech / MSSS.

## 12. Clancey et al. 2003 — CapCom's role must move onto Mars

- What: "The Mobile Agents Integrated Field Test" (Clancey et al.,
  2003, NASA Ames) — research on mobile agents supporting astronauts
  in the field.
- How used: research backing for the coach — CapCom's role must move
  onto Mars because of the radio delay; the coach is a small piece of
  that idea, in three lines.
- Real or model: published research.
- Source: https://ntrs.nasa.gov/citations/20040043676
- Attribution: NASA Ames Research Center.

## Notes

- All map pictures in the demo are screenshots of NASA Mars Trek with the
  route and marks drawn on top. No Mars pictures were invented or
  AI-generated.
- The darkening on the storm slide is a visual effect for the story; the
  storm itself comes from the climate-model data above.
- This repo's own words, design, and slides are Apache 2.0 (see LICENSE).
  Third-party datasets above keep their own terms.
