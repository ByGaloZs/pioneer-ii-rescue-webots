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
