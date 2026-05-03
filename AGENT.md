# AGENT.md

## Current Mission

Right now the active priority is **dataset capture**, not a broad follow-stack redesign.

Current datasets in focus:
- leader UAV dataset from `dji0`
- support UAV datasets from `dji1` and `dji2`

Primary workflows currently being exercised:
- `./run.sh collect_leader_dataset baylands`
- support-camera sweep plus support capture tooling

Current known blocker for the **next session**:
- Baylands **AMCL pose / Nav2 waypoint pose consistency** is still not trustworthy enough.
- In practice, this means waypoint YAMLs and AMCL initial poses for Baylands need more debugging before the automated capture flow is truly reliable.

## Operator Preferences

These are important and should guide the next agent's behavior:

- **Reuse existing code first.** Prefer extending current scripts, launch files, tools, and helpers instead of creating new files.
- **Do not create new files unless they are clearly necessary.**
- **Check with the user on the plan before substantial changes.**
- Prefer small, targeted edits over broad rewrites.
- Keep current topic names, parameter names, and launch argument styles stable unless the user explicitly wants them changed.

## Start Here

Read these first at the start of a session:
- `README.md`
- `src/lrs_halmstad/README.md`
- `CURRENT_STATE.md`
- `RUNNING_SIM.md`
- `FOLLOW_MODES_GUIDE.md`
- `RUN_FOLLOW_DEFAULTS_AUDIT.md`
- `VISUAL_FOLLOW_PLAN.md`

Quick orientation:
- Main package: `src/lrs_halmstad`
- Operator entrypoints: `./run.sh` and `./stop.sh`
- Baylands maps and waypoint CSVs: `maps/`
- Baylands Nav2 waypoint YAMLs: `src/lrs_halmstad/config/baylands_waypoints/`

Before editing:
- Run `git status --short --branch`
- Do not revert unrelated user changes
- Prefer small, targeted edits over broad rewrites
- Confirm the plan with the user before doing larger implementation passes

## Workspace Facts

Important current Baylands facts:
- Default Baylands Nav2 map: `maps/baylands_finished_v3_nav_20cm.yaml`
- Pose-compatible test variant: `maps/baylands_finished_v3_nav_20cm_merged.yaml`
- Baylands grouped spawn and localization waypoints come from `maps/waypoints_baylands_groups.csv`
- Baylands flat waypoint list lives in `maps/waypoints_baylands.csv`
- Baylands route YAML files live in `src/lrs_halmstad/config/baylands_waypoints/`

Scripts that matter for Baylands:
- `scripts/run_gazebo_sim.sh` resolves `waypoint:=...` spawn names
- `scripts/run_localization.sh` chooses the default Baylands map and initial pose behavior
- `scripts/run_nav2.sh` loads Baylands Nav2 params
- `scripts/run_save_waypoint_csv.sh` captures Gazebo plus AMCL pose into CSV
- `scripts/run_save_waypoint_yaml.sh` writes Nav2-ready YAML waypoint files
- `scripts/run_collect_leader_dataset.sh` runs the leader dataset collector
- `scripts/run_support_camera_scan.sh` sweeps support UAV cameras
- `scripts/run_support_capture_pair.sh` captures `dji1` and `dji2` datasets in parallel

## Code Conventions

Stay consistent with the existing codebase:
- Reuse existing functions and classes before adding new ones
- Extend current modules instead of creating parallel replacements
- Keep parameter names, topic names, and namespaces stable unless there is a strong reason to change them
- Follow the existing launch argument style and shell `name:=value` argument style
- Keep YAML defaults and launch-time overrides aligned

When working in specific areas:
- Follow logic: reuse helpers from `lrs_halmstad/follow/follow_core.py`, `follow_math.py`, and related follow modules instead of inlining geometry or control math
- Nav2 logic: extend `lrs_halmstad/nav/ugv_nav2_driver.py` and its helper flow instead of adding a second waypoint-loading path
- Launch files: preserve the current world-specific override style and existing argument names
- Shell scripts: preserve the existing parsing style and default-to-workspace behavior

Avoid:
- Duplicating helper functions that already exist nearby
- Adding "temporary" alternate paths when the current code can be extended cleanly
- Adding new files when an existing file or helper can be extended instead
- Renaming files, topics, or parameters casually in this workspace

## Baylands Map Rules

If editing Baylands Nav2 maps:
- Treat `baylands_finished_v3_nav_20cm.yaml` as the pose-compatible base
- Keep the map frame compatible unless intentionally rebuilding it
- Keep these values unchanged for pose-compatible edits:
  - `resolution: 0.200`
  - `origin: [-227, -444.5, 0.0]`
  - `mode: trinary`
  - `occupied_thresh: 0.65`
  - `free_thresh: 0.196`
- Keep the PGM trinary:
  - white = free
  - black = occupied
  - gray = unknown
- Edit at the same canvas size in GIMP and export a full-resolution PGM
- Use extra black to block bad, bumpy, or misleading routes that Nav2 should avoid
- Validate in RViz against the live scan before switching defaults

## Validation

After code changes, use the smallest useful validation:
- Python-only change: `python3 -m py_compile <file>`
- Package-level change: `colcon build --symlink-install`
- Launch or node contract change: run `ros2 run ... --help` or `ros2 launch ... --show-args`

After workflow or operator-facing changes:
- Update `README.md` and `src/lrs_halmstad/README.md`

## Notes For Future Sessions

Current handoff notes:
- The dataset collector exists and is the active leader-data path.
- Support-UAV camera sweep and paired capture tooling also exist and are part of the current data-collection effort.
- The current debugging lane is **not** "invent a new architecture"; it is "make the existing capture workflows reliable."
- The current likely failure area is Baylands AMCL / Nav2 initial pose and waypoint alignment, not the simplified follow math itself.
- If something fails while collecting data, inspect the current capture scripts and launch wrappers first before proposing new tooling.

If documentation and code disagree, trust the code paths in:
- `scripts/`
- `src/lrs_halmstad/launch/`
- `src/lrs_halmstad/lrs_halmstad/`

If Baylands behavior looks odd, check these first:
- active map YAML under `maps/`
- spawn waypoint resolution in `run_gazebo_sim.sh`
- AMCL initial pose logic in `run_localization.sh`
- grouped waypoint CSV contents in `maps/waypoints_baylands_groups.csv`
