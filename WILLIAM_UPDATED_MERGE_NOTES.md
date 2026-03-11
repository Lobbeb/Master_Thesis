# `william_updated` Merge Notes

Updated: 2026-03-11

## Branch refs

- Current working branch: `splitting_everything` at `4461e74`
- `main`: local `4402712`
- Local stale remote-tracking ref: `origin/william_updated` at `e43cd5d`
- Live GitHub ref fetched separately: `github_live/william_updated` at `eb5eb47`

## Ported Into Current Branch

### Recorder and tmux support

- Added `scripts/run_record_experiment.sh`
- Extended `scripts/run_tmux_1to1.sh` with:
  - `record:=true|false`
  - `record_profile:=default|vision`
  - `record_tag:=...`
  - `record_out:=...`
  - `record_delay_s:=...`
- Extended `scripts/stop_tmux_1to1.sh` so it also stops the recorder and cleans up matching `ros2 bag record` processes as fallback
- Adapted the recorder flow into the current `scripts/` layout instead of William's root-level script layout
- Current default bag root is `bags/experiments/...`

### Perception metadata

- Added `track_hits`, `track_age_s`, `track_state`, and `track_switched` to `src/lrs_halmstad/lrs_halmstad/perception/detection_protocol.py`
- Populated those fields in `src/lrs_halmstad/lrs_halmstad/perception/leader_tracker.py`

### Detection status ownership cleanup

- Added `src/lrs_halmstad/lrs_halmstad/perception/detection_status.py` as a shared helper for detection-status lines
- Added detector/tracker-owned `/coord/leader_detection_status`
- `leader_estimator` consumes `/coord/leader_detection_status` only for `leader_debug_image`
- `leader_estimator` status no longer mirrors tracker-owned metadata into `/coord/leader_estimate_status`
