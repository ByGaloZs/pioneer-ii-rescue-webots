# AI development instructions

This is a C++ Webots project for autonomous rescue using Pioneer II. Read `README.md`, `docs/architecture.md`, `docs/decisions.md`, and `docs/experiments.md` before changing functionality.

## Non-negotiable rules
- Keep all source code, identifiers, filenames, class and function names in English. Documentation is in Spanish.
- Do not treat planned features as implemented or claim tests that were not run.
- Keep `controllers/fool_controller/fool_controller.cpp` as the Webots entry point; do not rename `fool_controller` without updating every world and build reference.
- Use standard Webots C++ controller APIs. Do not invent APIs or add unrelated dependencies.
- Prefer small modules with one responsibility, explicit inputs/outputs and minimal shared mutable state.
- Do not mix experiments, results, maps or debug output into production source folders.
- Keep the Makefile in sync with any added .cpp files.

## Component ownership
- `config/`: validated Webots device identifiers, wheel geometry, calibration and parameters.
- `sensors/`: initialization, snapshot collection, calibrated range estimates and sensor geometry.
- `control/`: wheel commands and safety stop.
- Planned `localization/`: pose estimation from GPS, compass and encoders.
- Planned `perception/`: visual radial observation and fusion with proximity sensors.
- Planned `mapping/`: local and accumulated occupancy grid.
- Planned `exploration/`: reachable goal-biased frontier selection.
- Planned `planning/`: A* on known, inflated free space.
- Planned `mission/`: startup, exploration, approach/rescue, return, stop and time management.

## Sensor safety
- Preserve `RobotConfig.h` and `DistanceSensorCalibration.*` unless a change has measured evidence and regression tests.
- The spherical camera is not a metric LiDAR. Validate pixel-to-distance models and uncertainty before mapping.
- Fuse observations only within valid sensor ranges and known fields of view. 'No obstacle reading' is not proof of free space.
- Maintain free, occupied and unknown cells distinctly. Include robot footprint and safety margin.
- Reactively stop or avoid collisions, even when a planned route says to proceed.
- Never infer a scenario identifier from world metadata or preload ground-truth obstacle geometry at runtime.

## Navigation and mission
- Use incremental occupancy mapping; Frontier-Based Exploration selects where to discover; A* plans how to reach a confirmed-free frontier.
- Preserve a usable representation of previously traversed space to support return.
- Do not assume fixed initial x/z/yaw. Mission target is the opposite end, two green victims within 1 m, a full turn for each, then return; simulated time limit 240 s.
- Monitor simulated time rather than wall-clock time.

## Experiments and validation
- Create `experiments/exp_NNN_short_name/` for each hypothesis; start from `experiments/_template/README.md`.
- Record setup, commit SHA, Webots version, world, starting pose, number of repetitions, parameters, procedure, metrics, failures and conclusion.
- Keep prototypes in the experiment directory, not in the controller. Only integrate after evidence and regression tests.
- Store small CSV results and summaries in each experiment's `results/`; ignore bulky logs, binaries and videos.
- Start with a controlled world; confirm generalization in all 10 `worlds/final_scenario*.wbt`, using varying initial poses.
- Update `docs/decisions.md` for significant new decisions; preserve unsuccessful experiment conclusions.
- Before reporting completion: run available tests, build in Webots where possible, state what was *not* validated, and avoid invented metrics.

## Scenario-specific testing plan
- Follow `docs/test_plan.md` and `docs/scenario_catalog.md` when selecting representative WBT scenarios and designing regressions.
- Every experiment must have its own `experiments/exp_NNN_short_name/README.md`, even if it is only planned. Document reasons if the chosen scenario matrix changes.
- For outbound navigation and the complete mission, schedule 10 scenarios × 3 verified starting poses, and record results per run. The source WBT robot pose is not the competition start pose.
- Do not alter canonical `worlds/final_scenario*.wbt` when producing test variants; place reproducible variants outside the reference worlds and record parameters.
- All ten experiments are currently planned, not executed. Never fabricate results.

## Original scenario provenance and compatibility
- The canonical `worlds/final_scenario01.wbt` through `final_scenario10.wbt` files are byte-for-byte copies of the user's original `Práctica06_src.zip` archive. Preserve them exactly; their checksums are listed in `worlds/README.md`.
- Do not silently rewrite the WBT header, Viewpoint, robot pose, `Pioneer2` node, camera slots, lights or obstacle geometry. Any compatibility adaptation belongs in a separately documented test copy, not the canonical source.
- These originals use `Pioneer2`, whereas `protos/Pioneer2_Robot.proto` also exists. Verify device-name and sensor-calibration compatibility in Webots before claiming runtime support; retain the calibrated controller source unchanged unless explicitly authorized.

## EXP-002 self-occlusion pilot
- `camera_s` includes visible robot body. Never classify its occluded pixels as free floor or convert them to range measurements.
- `experiments/exp_002_visual_ranging/config/self_occlusion_pilot.json` is an **unverified example** until inspected against real spherical frames. Keep excluded pixels unknown.
- Generate any test variant using `src/create_pilot_world.py`, never overwrite the ten canonical `.wbt` files.
- A usable image-space boundary does not imply near-field safety or metric range. Calibration, image registration and metrics are still pending.

## EXP-002 camera orientation variants
- The three `worlds/exp002_camera_01_*.wbt` are explicitly versioned pilot variants generated from scenario 01, not replacements for original final scenarios.
- In each variant, GPS resolution is `-1`, robot starts at `(0, 0, -8.8)` with yaw 0, and only `camera_s` tilt differs. Do not infer physical bearing without verifying camera extrinsics.
- Keep an orientation-specific chassis mask if a mask is needed; do not transfer the old downward-facing mask unchanged.
- Generate/check variants with `experiments/exp_002_visual_ranging/src/create_pilot_world.py`; preserve canonical worlds and captured evidence.

## EXP-002 calibration capture fixture
- Three versioned pilot worlds: `worlds/exp002_calibration_01_{p1,p2,p3}.wbt`. They are offline-derived test variants only and have unique `controllerArgs` trial labels.
- Do not expose wall geometry or target distances to the runtime controller. The label is recording metadata, not a measurement or world-identification mechanism.
- Only the calibration runs start capture after 1 simulated second and finalize on the first saved frame. Noncalibration runs remain periodic.
- The visual camera markers are removed only in calibration copies; no canonical WBT is changed.
- Distances in the README refer to robot-center horizontal reference, not the camera optical ray. Do not claim metric ranging or a passing Webots run without observations.
