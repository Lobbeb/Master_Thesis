# Practical → Writing 1-to-1 Mapping (UAV-side scope)

Use this as a checklist: when a practical item is done, you immediately know what to write/update in the thesis.

## 1. Upstream simulation + control-chain discipline

### 1.1 Practical: Track upstream sim updates (new UAV model + realistic control path)
What “done” means: you can bring up the world + spawn UAVs + run your follower stack with the updated model.

Write/update:
1. Chapter 3.2.1 *Software stack and simulation assets*: mention the updated UAV model/control path and where it comes from (repo/branch).
2. Appendix A (reproducibility): list the exact launch/script entrypoints and required terminals.

Evidence to cite (internal): terminal commands + run directory metadata (meta.yaml) showing successful runs.

### 1.2 Practical: No manual commands; publish plan/trajectory to a controller
What “done” means: your node publishes intent/targets; a controller handles actuation.

Write/update:
1. Chapter 3.4 *Coordination Interface and Data Flow*: explicitly state “planner/trajectory → controller → simulator”.
2. Chapter 3.2.2 *Baseline benchmark scenario*: clarify that motion is deterministic/scripted and controller-driven (not manual).

Evidence: rosbag topics showing command streams + controller topics (and that you do not joystick/manual drive).

---

## 2. Freeze the interface contract (UGV treated as black box)

### 2.1 Practical: Lock requirement that UAV does not depend on UGV self-state broadcast
What “done” means: your estimator/controller does not require UGV pose/odom as a hard dependency.

Write/update:
1. Chapter 3.4: explicitly separate “leader inputs” from “UGV state broadcast” (black-box constraint).
2. Chapter 3.6.3 *Topic contract* (or your interface table): mark any UGV-state topics as “optional/for evaluation only” if used.

Evidence: dependency check (contract_check) + node parameter list (shows what is required vs optional).

---

## 3. Perception → estimate pipeline (YOLO + filtering + status)

### 3.1 Practical: YOLO inference from UAV camera stream
What “done” means: detections are produced reliably in sim from the bridged camera topic.

Write/update:
1. Chapter 3.6.4 *Estimation pipeline*: “image → YOLO inference → best detection”.
2. Appendix A: list the camera topics and how they are bridged.

Evidence: rosbag contains camera topics + estimator status events.

### 3.2 Practical: Convert detections into a coordination-grade estimate (pose proxy) + confidence
What “done” means: you publish a stable leader estimate message and a status stream.

Write/update:
1. Chapter 3.6.3 *Topic contract*: `/coord/leader_estimate` + `/coord/leader_estimate_status`.
2. Chapter 3.6.4: bearing/range/projection steps (as implemented).

Evidence: rosbag includes leader_estimate + status.

### 3.3 Practical: Kalman filter (or equivalent) smoothing + short-horizon prediction
What “done” means: estimate is smoothed; brief dropouts do not create violent jumps; short prediction exists.

Write/update:
1. Chapter 3.6.4: add a short “filtering/prediction” sentence (do not overclaim).
2. Appendix (parameters): filter gains / window sizes.

Evidence: compare raw vs filtered traces (plots) + status logs showing dropouts handled.

### 3.4 Practical: Failure handling (stale/hold/reacquire), status is primary diagnostic
What “done” means: clear state/status for missing camera, YOLO disabled, stale estimate, reacquire.

Write/update:
1. Chapter 3.6.5 *Failure handling*: list the exact cases and how you behave.
2. Appendix A: execution modes and how to reproduce failure cases (optional).

Evidence: status messages and event markers in rosbag.

---

## 4. Estimate-driven following + hybrid fallback

### 4.1 Practical: Follow-point generation from estimate (pos/vel) and leash constraints
What “done” means: estimate-driven mode produces a target and respects leash rules.

Write/update:
1. Chapter 3.6.6 *Execution modes*: define “estimate-driven follow”.
2. Chapter 3.2.2 baseline vs follow scenario: clarify deterministic UGV motion + follower behavior.

Evidence: tracking error metrics + leash clamp events.

### 4.2 Practical: Hybrid novelty (fallback when perception is weak)
What “done” means: system falls back to baseline behavior (or holds) rather than oscillating.

Write/update:
1. Chapter 3.6.5 and 3.6.6: clearly describe fallback logic and when it triggers.
2. Results section: show one controlled dropout/disturbance and how fallback prevents failure.

Evidence: a paired run (with/without dropout) + metrics comparing stability.

---

## 5. Comms-aware hooks (stub now, OMNeT++ later)

### 5.1 Practical: Define and publish a comm-quality signal (synthetic first)
What “done” means: you have a topic that represents link-quality or degradation.

Write/update:
1. Chapter 3.5 *Communication Conditions*: define the signal and how it is injected in sim.
2. Appendix A: list the topic name and parameter to enable it.

Evidence: rosbag includes comm-quality topic + event markers.

### 5.2 Practical: Thresholds + hysteresis for comm-based leash switching
What “done” means: comm dips do not cause rapid mode flapping; switching is stable.

Write/update:
1. Chapter 3.5: describe the impairment knobs + switching policy (high-level).
2. Results: show a controlled comm dip and stable response (no oscillation).

Evidence: event markers (MODE_SWITCH / LEASH_STATE) + comm-quality time series.

---

## 6. Run plan + logging (so writing and results are “backed”)

### 6.1 Practical: Three core modes are reproducible
Modes:
- Baseline (no perception)
- Perception-only (smoke)
- Estimate-driven follow

Write/update:
1. Chapter 3.6.6: table of execution modes (already in your draft).
2. Appendix A: commands/entrypoints for each mode.

Evidence: one run directory per mode (rosbag + meta.yaml) with consistent topics.

### 6.2 Practical: “Done” logging contract is frozen
What “done” means: same bag topics + meta.yaml fields across all modes.

Write/update:
1. Chapter 3.2.3 *Experiment execution and run outputs*: define the run directory contents.
2. Results: reference run IDs consistently (no silent changes in logging).

Evidence: run folders show identical logging schema across runs.

---

## 7. Quick “what to write next” checklist

When you finish:
- **Upstream update + control chain** → update 3.2.1 + 3.4 + Appendix A
- **YOLO + estimate + status** → update 3.6.3–3.6.5 + params appendix
- **Estimate-driven follow + fallback** → update 3.6.6 + results disturbance figure
- **Comms hook + hysteresis** → update 3.5 + results comm dip figure
- **Frozen logging** → update 3.2.3 + keep results traceable to run IDs
