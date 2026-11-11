# AI development instructions

This is a C++ Webots project for autonomous rescue using Pioneer II. Read `README.md`, `docs/architecture.md`, `docs/decisions.md`, and `docs/experiments.md` before changing functionality.

## Non-negotiable rules
- Keep all source code, identifiers, filenames, class and function names in English. Documentation is in Spanish.
- Do not treat planned features as implemented or claim tests that were not run.
- Keep `controllers/fool_controller/fool_controller.cpp` as the Webots entry point; do not rename `fool_controller` without updating every world and build reference.
- Use standard Webots C++ controller APIs. Do not invent APIs or add unrelated dependencies.
- Prefer small modules with one responsibility, explicit inputs/outputs and minimal shared mutable state.
- Do not mix experiments, results, maps or debug output into production source folders.
- Keep `controllers/fool_controller/` free of experiment-specific loggers and prototypes. All Webots experiments that need C++ runtime code reuse one `controllers/experiment_test_controller/` with its own entry point and Makefile; the shared pilot world selects it explicitly.
- Never duplicate the full production controller as an experiment. Reuse stable modules by explicit references where practical. Keep experiment-specific code/results in their experiment folders; use one shared scenario-01 test-world copy and one shared test controller, configured for the active experiment.
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
- Keep prototype logic and analysis in the experiment directory. Use `controllers/experiment_test_controller/` as the single editable C++ runtime harness for the active Webots experiment; never toggle experiment loggers by editing the production entry point.
- Generate one reproducible test WBT from `worlds/final_scenario01.wbt` at `worlds/experiment_test_scenario01.wbt`. Configure it for the active experiment and give each run an explicit label. Write only to that experiment's `experiments/exp_NNN_short_name/results/`; missing/invalid labels or result roots must fail closed, with no generic results fallback.
- Store small CSV results and summaries in each experiment's `results/`; ignore bulky logs, binaries and videos.
- First validate each experiment on the shared scenario-01 test-world copy. Once the full baseline workflow works, test generalization across the chosen map conditions; confirm outbound navigation and the complete mission in all 10 canonical scenarios with varying initial poses.
- Update `docs/decisions.md` for significant new decisions; preserve unsuccessful experiment conclusions.
- Before reporting completion: run available tests, build in Webots where possible, state what was *not* validated, and avoid invented metrics.

## Scenario-specific testing plan
- Follow `docs/test_plan.md` and `docs/scenario_catalog.md` when selecting representative WBT scenarios and designing regressions.
- Every experiment must have its own `experiments/exp_NNN_short_name/README.md`, even if it is only planned. Document reasons if the chosen scenario matrix changes.
- For outbound navigation and the complete mission, schedule 10 scenarios × 3 verified starting poses, and record results per run. The source WBT robot pose is not the competition start pose.
- Keep the canonical GPS setting at `resolution -1` in all `worlds/final_scenario*.wbt`; do not modify other canonical fields when producing test variants. Put additional pose/sensor variants in the single shared test-world copy and record parameters.
- Consult each experiment README for its actual state; EXP-001 has accepted initial sensor checks, EXP-002 is in progress, and the remaining experiments are planned unless their records say otherwise. Never fabricate results.

## Original scenario provenance and compatibility
- The canonical `worlds/final_scenario01.wbt` through `final_scenario10.wbt` files derive from the user's original `Práctica06_src.zip` archive. Their only authorized source-world change is GPS `resolution -1`; original and current hashes are listed in `worlds/README.md`.
- Do not rewrite the WBT header, Viewpoint, robot pose, `Pioneer2` node, camera slots, lights or obstacle geometry. Any other compatibility adaptation belongs in a separately documented test copy, not the canonical source.
- These originals use `Pioneer2`, whereas `protos/Pioneer2_Robot.proto` also exists. Verify device-name and sensor-calibration compatibility in Webots before claiming runtime support; retain the calibrated controller source unchanged unless explicitly authorized.

## EXP-002 self-occlusion pilot
- `camera_s` includes visible robot body. Never classify its occluded pixels as free floor or convert them to range measurements.
- `experiments/exp_002_visual_ranging/config/self_occlusion_pilot.json` is an **unverified example** until inspected against real spherical frames. Keep excluded pixels unknown.
- Generate/update the shared test world using `experiments/exp_002_visual_ranging/src/create_pilot_world.py`, never overwrite the ten canonical `.wbt` files. The shared world selects `experiment_test_controller` and carries an explicit experiment/trial label.
- A usable image-space boundary does not imply near-field safety or metric range. Calibration, image registration and metrics are still pending.

## EXP-002 camera orientation variants
- `worlds/experiment_test_scenario01.wbt` is the single reusable test copy generated from scenario 01, not a replacement for original final scenarios. Its pose and camera settings are changed reproducibly for the active run.
- The shared test world uses GPS resolution `-1`; verify its configured pose and camera orientation in each trial. Do not infer physical bearing without verifying camera extrinsics.
- Keep an orientation-specific chassis mask if a mask is needed; do not transfer the old downward-facing mask unchanged.
- Generate/check the shared world with `experiments/exp_002_visual_ranging/src/create_pilot_world.py`; preserve canonical worlds and captured evidence. Build with `make -C controllers/experiment_test_controller`, never by adding experiment loggers to `fool_controller`.

## EXP-002 calibration capture fixture
- Calibration poses P1/P2/P3 are sequential configurations of the one shared test WBT, regenerated with distinct trial labels; do not keep separate WBT copies for each pose.
- Do not expose wall geometry or target distances to the runtime controller. The label is recording metadata, not a measurement or world-identification mechanism.
- Only `calib01_*` trials start capture after 1 simulated second and finalize on the first saved frame. Other EXP-002 trials remain periodic.
- The visual camera markers are removed only in the generated test copy when calibration is selected; no canonical WBT is changed.
- Distances in the README refer to robot-center horizontal reference, not the camera optical ray. Do not claim metric ranging or a passing Webots run without observations.

## EXP-002 V3 geometry guardrails
- Use `spherical_geometry.py` and `angular_observations.py` for future spherical-camera analysis. The radial functions in `camera_projection.py` are legacy image-space only and MUST NOT drive metric mapping.
- The FOV configured in the current WBT is ~180° horizontally, not 360°. Verify azimuth sign with an off-axis marker before navigation.
- Ground-ray distance is only a candidate for a floor/obstacle contact in clear view and with verified orientation-specific chassis mask. Unknown is not free.
- P1/P2/P3 have no established visible ground-contact boundary or measured MAE. Do not assert numeric ranging accuracy until independent reference and out-of-sample validation exist.
